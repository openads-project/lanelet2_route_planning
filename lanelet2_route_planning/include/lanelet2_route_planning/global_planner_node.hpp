#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


#include "lanelet2_map_interface/lanelet2_map_interface.hpp"
#include "lanelet2_route_planning_interfaces/action/global_maneuver.hpp"
#include "lanelet2_utilities/lanelet2_utils.hpp"

#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <lanelet2_core/utility/Units.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>

#include "lanelet2_route_planning/ll2_route_planning_datatypes.hpp"

using namespace std::chrono_literals;

class GlobalPlanner : public rclcpp::Node
{
    public:
        GlobalPlanner();
        void initializeMapInterface();
        
        
    private:
        // Variables
        LL2MapInterface *ll2if_;
        geometry_msgs::msg::PoseWithCovarianceStamped ego_pose_;
        lanelet::LaneletMapConstPtr llmap_;
        traffic_rules::TrafficRulesPtr trafficRules_ = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, std::string(lanelet::Participants::Vehicle) + ":ika");
        routing::RoutingGraphUPtr routingGraph_;
        // Dummy traffic rules and routing graph for bicycles used to incorporate drivable bicycle lanes in our lane boundaries
        traffic_rules::TrafficRulesPtr trafficRulesBicycle_ = traffic_rules::TrafficRulesFactory::create(std::string(Locations::Germany) + ":dummy", Participants::Bicycle);
        routing::RoutingGraphUPtr routingGraphBicycle_;

        int visualize_lvl_=1;

        // Route Planning
        lanelet::ConstLanelet start_ll_;    // most probable current Lanelet
        int16_t start_lane_id_;
        lanelet::ConstLanelet target_ll_;
        int16_t target_lane_id_;
        double target_lane_s_dest_;

        std::vector<int64_t> shortest_path_ll_ids_;

        double ds_sample_ = 2.0;
        double smooth_factor_ = 2.0;

        Optional<lanelet::routing::Route> route_;
        Lanelet2RoutePlanningDatatypes::LaneletLaneNetwork lane_network_;

        // Maneuver Execution
        rclcpp::Time maneuver_start_time_;

        // Local Path Extraction
        
        // Timer
        rclcpp::TimerBase::SharedPtr startup_timer_;

        // Subscriptions
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr map_pose_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

        // Action Client
        rclcpp_action::Client<lanelet2_route_planning_interfaces::action::GlobalManeuver>::SharedPtr maneuver_action_client_;

        // Action Server
        rclcpp_action::Server<lanelet2_route_planning_interfaces::action::GlobalManeuver>::SharedPtr maneuver_action_server_;
        lanelet2_route_planning_interfaces::action::GlobalManeuver::Feedback::SharedPtr maneuver_feedback_;
        lanelet2_route_planning_interfaces::action::GlobalManeuver::Result::SharedPtr maneuver_result_;

        // Publisher
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_destination_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_route_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_boundary_pub_;

        // Function Definitions
        // global_planner_node.cpp
        void initializeGlobalPlanner();
        bool egoPositionSanityCheck();
        bool targetPositionSanityCheck(double target_x, double target_y);
        bool planRoute(lanelet::ConstLanelet start_ll, lanelet::ConstLanelet target_ll);
        void constructLaneNetwork(const lanelet::routing::LaneletPath &shortestPath, visualization_msgs::msg::MarkerArray &viz_marker_array);
        lanelet::BasicLineString2d sampleBoundaries(const lanelet::BasicLineString2d &centerline,
                                                    const double test_dis,
                                                    const bool &b_right,
                                                    std::vector<int>& index_mapping,
                                                    const lanelet::BasicLineString2d& lane_boundary,
                                                    visualization_msgs::msg::MarkerArray& marker_array);
        lanelet::BasicLineString2d sampleDrivableSpace(const lanelet::BasicLineString2d &centerline,
                                                        const double test_dis,
                                                        const bool &b_right,
                                                        std::vector<int>& index_mapping,
                                                        visualization_msgs::msg::MarkerArray& marker_array);

        // local_path_extraction.cpp
        


        // maneuver_action_fcns.cpp
        rclcpp_action::GoalResponse actionHandleGoal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const lanelet2_route_planning_interfaces::action::GlobalManeuver::Goal> goal);

        rclcpp_action::CancelResponse actionHandleCancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_interfaces::action::GlobalManeuver>> goal_handle);

        void actionHandleAccepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_interfaces::action::GlobalManeuver>> goal_handle);

        void actionExecute(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_interfaces::action::GlobalManeuver>> goal_handle);

        // callbacks.cpp
        void mapPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
        void goalPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);


        // utils.cpp
        void processLineString(lanelet::BasicLineString2d& line_string, const std::string& desc, visualization_msgs::msg::MarkerArray &marker_array, std::vector<float> colors, std::vector<float> colors_smoothed);
        bool handleInwardCorner(const lanelet::BasicPoint2d &base_p, lanelet::BasicPoint2d& best_point,
                                                const std::pair<lanelet::BasicLineString2d, size_t>*& last_intersection_free_test_line,
                                                lanelet::BasicLineString2d& previous_test_line,
                                                const uint& idx, std::deque<std::pair<lanelet::BasicLineString2d, size_t>>& last_test_lines,
                                                lanelet::BasicLineString2d& bound, std::vector<int>& index_mapping);
        bool checkLineDrivability(const lanelet::ConstLineString3d &lineToCheck);


        // visualization.cpp
        void visualizeLinestring(std::vector<geometry_msgs::msg::Point>& line_string, const std::string& desc, visualization_msgs::msg::MarkerArray& marker_array, std::vector<float> colors);
        visualization_msgs::msg::Marker convertDestination2Marker(double target_x, double target_y, std::string frame_id);
        void visualizeIndexMapping(visualization_msgs::msg::Marker& marker, visualization_msgs::msg::MarkerArray& marker_array, const lanelet::BasicLineString2d& bound,
                                    const std::string& left_right_string, const std::string& ns, const std::vector<int>& index_mapping);

};