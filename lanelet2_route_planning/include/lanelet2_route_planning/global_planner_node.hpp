#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


#include "lanelet2_map_interface/lanelet2_map_interface.hpp"
#include "lanelet2_route_planning_ifs/action/global_maneuver.hpp"
#include "lanelet2_utilities/lanelet2_utils.hpp"

#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

using namespace std::chrono_literals;

class GlobalPlanner : public rclcpp::Node
{
    public:
        GlobalPlanner();
        void initializeMapInterface();
        
        
    private:
        // Variables
        LL2MapInterface *ll2if_;
        nav_msgs::msg::Odometry ego_pose_;
        traffic_rules::TrafficRulesPtr trafficRules_ = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, std::string(lanelet::Participants::Vehicle) + ":ika");
        routing::RoutingGraphUPtr routingGraph_;
        // Dummy traffic rules and routing graph for bicycles used to incorporate drivable bicycle lanes in our lane boundaries
        traffic_rules::TrafficRulesPtr trafficRulesBicycle_ = traffic_rules::TrafficRulesFactory::create(std::string(Locations::Germany) + ":dummy", Participants::Bicycle);
        routing::RoutingGraphUPtr routingGraphBicycle_;


        lanelet::ConstLanelet start_ll_;    // most probable current Lanelet
        lanelet::ConstLanelet target_ll_;

        std::vector<int64_t> shortest_path_ll_ids_;
        lanelet::BasicLineString2d shortest_path_centerline_;

        double ds_sample_ = 2.0;
        double smooth_factor_ = 2.0;
        
        // Timer
        rclcpp::TimerBase::SharedPtr startup_timer_;

        // Subscriptions
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr map_pose_sub_;

        // Actions
        rclcpp_action::Server<lanelet2_route_planning_ifs::action::GlobalManeuver>::SharedPtr maneuver_action_server_;
        lanelet2_route_planning_ifs::action::GlobalManeuver::Feedback::SharedPtr maneuver_feedback_;
        lanelet2_route_planning_ifs::action::GlobalManeuver::Result::SharedPtr maneuver_result_;

        // Publisher
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_destination_pub_;


        // Function Definitions
        // global_planner_node.cpp
        void initializeGlobalPlanner();
        bool egoPositionSanityCheck();
        bool targetPositionSanityCheck(double target_x, double target_y);
        bool planRoute(lanelet::ConstLanelet start_ll, lanelet::ConstLanelet target_ll);
        
        // maneuver_action_fcns.cpp
        rclcpp_action::GoalResponse actionHandleGoal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const lanelet2_route_planning_ifs::action::GlobalManeuver::Goal> goal);

        rclcpp_action::CancelResponse actionHandleCancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle);

        void actionHandleAccepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle);

        void actionExecute(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle);

        // callbacks.cpp
        void mapPoseCallback(nav_msgs::msg::Odometry::SharedPtr msg);

        // utils.cpp
        void processLineString(lanelet::BasicLineString2d& line_string, const std::string& desc, visualization_msgs::msg::MarkerArray &marker_array, std::vector<float> colors, std::vector<float> colors_smoothed);

        // visualization.cpp
        visualization_msgs::msg::Marker convertDestination2Marker(double target_x, double target_y, std::string frame_id);

};