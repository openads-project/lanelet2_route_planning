#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "lanelet2_map_interface/lanelet2_map_interface.hpp"
#include "route_planning_interfaces/action/global_maneuver.hpp"
#include "lanelet2_utilities/lanelet2_utils.hpp"

#include "route_planning_interfaces/msg/boundaries.hpp"
#include "route_planning_interfaces/msg/driveable_space.hpp"
#include "route_planning_interfaces/msg/regulatory_element.hpp"
#include "route_planning_interfaces/msg/road_marking.hpp"
#include "route_planning_interfaces/msg/route.hpp"

#include "route_planning_interfaces/tf2_route_planning_interfaces.hpp"

#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <lanelet2_core/utility/Units.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>

using namespace std::chrono_literals;

class GlobalPlanner : public rclcpp::Node
{
    public:
        GlobalPlanner();
        void initializeMapInterface();
        
        
    private:
        //tf2
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        // Variables
        LL2MapInterface *ll2if_;
        geometry_msgs::msg::PoseWithCovarianceStamped ego_pose_;
        lanelet::LaneletMapConstPtr llmap_;
        traffic_rules::TrafficRulesPtr trafficRules_ = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, std::string(lanelet::Participants::Vehicle) + ":ika");
        routing::RoutingGraphUPtr routingGraph_;
        // Dummy traffic rules and routing graph for bicycles used to incorporate drivable bicycle lanes in our lane boundaries
        traffic_rules::TrafficRulesPtr trafficRulesBicycle_ = traffic_rules::TrafficRulesFactory::create(std::string(Locations::Germany) + ":dummy", Participants::Bicycle);
        routing::RoutingGraphUPtr routingGraphBicycle_;

        // Route Planning
        lanelet::ConstLanelet start_ll_;    // most probable current Lanelet
        lanelet::ConstLanelet target_ll_;
        route_planning_interfaces::msg::DriveableSpace global_driveable_space_;
        route_planning_interfaces::msg::Route global_route_;

        double ds_sample_ = 2.0;
        double smooth_factor_ = 2.0;

        Optional<lanelet::routing::Route> route_;

        // Maneuver Execution
        rclcpp::Time maneuver_start_time_;

        // Local Path Extraction
        std::string local_vehicle_frame_id_="base_link";
        double path_extraction_rate_=10.0;
        unsigned int target_sample_cl_;
        unsigned int ego_pos_sample_cl_;
        unsigned int lbehind_sample_drivspace_left_, lbehind_sample_drivspace_right_;
        unsigned int lahead_sample_drivspace_left_, lahead_sample_drivspace_right_;
        std::vector<geometry_msgs::msg::Point> remaining_shortest_path_;
        double look_ahead_time_ = 10.0;
        double look_ahead_distance_min_ = 50.0;
        double look_behind_distance_ = 20.0;
        rclcpp::Publisher<route_planning_interfaces::msg::Route>::SharedPtr local_route_pub_;
        rclcpp::Publisher<route_planning_interfaces::msg::DriveableSpace>::SharedPtr local_driveable_space_pub_;
        
        // Timer
        rclcpp::TimerBase::SharedPtr startup_timer_;

        // Subscriptions
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr map_pose_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

        // Action Client
        rclcpp_action::Client<route_planning_interfaces::action::GlobalManeuver>::SharedPtr maneuver_action_client_;
        std::shared_future<rclcpp_action::ClientGoalHandle<route_planning_interfaces::action::GlobalManeuver>::SharedPtr> goal_handle_future_;

        // Action Server
        rclcpp_action::Server<route_planning_interfaces::action::GlobalManeuver>::SharedPtr maneuver_action_server_;
        route_planning_interfaces::action::GlobalManeuver::Feedback::SharedPtr maneuver_feedback_;
        route_planning_interfaces::action::GlobalManeuver::Result::SharedPtr maneuver_result_;

        // Publisher
        rclcpp::Publisher<route_planning_interfaces::msg::Route>::SharedPtr route_pub_;
        rclcpp::Publisher<route_planning_interfaces::msg::DriveableSpace>::SharedPtr driveable_space_pub_;

        // Function Definitions
        // global_planner_node.cpp
        void initializeGlobalPlanner();
        bool egoPositionSanityCheck();
        bool targetPositionSanityCheck(double target_x, double target_y);
        bool planRoute(lanelet::ConstLanelet start_ll, lanelet::ConstLanelet target_ll);
        void constructLaneNetwork(const lanelet::routing::LaneletPath &shortestPath, visualization_msgs::msg::MarkerArray &viz_marker_array);

        // local_path_extraction.cpp
        void initializeLocalPathExtraction(const route_planning_interfaces::msg::Route& route_global);
        void extractLocalMapInfo(const geometry_msgs::msg::PoseWithCovarianceStamped& cur_pose,
                                const route_planning_interfaces::msg::DriveableSpace& driveable_space_global,
                                route_planning_interfaces::msg::DriveableSpace& driveable_space_local,
                                const route_planning_interfaces::msg::Route& route_global,
                                route_planning_interfaces::msg::Route& route_local);
        double accumulatedLength(const std::vector<geometry_msgs::msg::Point>& point_list, std::vector<double>& accumulated_length);
        double distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
        unsigned int findNearestSample(const geometry_msgs::msg::Point& ref_point, const std::vector<geometry_msgs::msg::Point>& point_list, const unsigned int& start_index=0);
        unsigned int findNearestSampleReverse(const geometry_msgs::msg::Point& ref_point, const std::vector<geometry_msgs::msg::Point>& point_list);


        // maneuver_action_fcns.cpp
        rclcpp_action::GoalResponse actionHandleGoal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const route_planning_interfaces::action::GlobalManeuver::Goal> goal);

        rclcpp_action::CancelResponse actionHandleCancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_interfaces::action::GlobalManeuver>> goal_handle);

        void actionHandleAccepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_interfaces::action::GlobalManeuver>> goal_handle);

        void actionExecute(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_interfaces::action::GlobalManeuver>> goal_handle);

        // callbacks.cpp
        void mapPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
        void goalPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);


        // utils.cpp
        std::vector<geometry_msgs::msg::Point> processLineString(
                                                lanelet::BasicLineString2d& line_string,
                                                const std::string& desc,
                                                visualization_msgs::msg::MarkerArray &marker_array,
                                                std::vector<float> colors,
                                                std::vector<float> colors_smoothed);
        route_planning_interfaces::msg::DriveableSpace sampleDriveableSpace(
                                                        const lanelet::BasicLineString2d &centerline,
                                                        visualization_msgs::msg::MarkerArray& marker_array);
        std::vector<geometry_msgs::msg::Point> sampleLinestring(
                                          const lanelet::BasicLineString2d &centerline,
                                          const double test_dis,
                                          bool b_right);
        // lanelet::BasicLineString2d sampleBoundaries(const lanelet::BasicLineString2d &centerline,
        //                                             const double test_dis,
        //                                             const bool &b_right,
        //                                             std::vector<int>& index_mapping,
        //                                             const lanelet::BasicLineString2d& lane_boundary,
        //                                             visualization_msgs::msg::MarkerArray& marker_array);

        bool handleInwardCorner(const lanelet::BasicPoint2d &base_p, lanelet::BasicPoint2d& best_point,
                                                const std::pair<lanelet::BasicLineString2d, size_t>*& last_intersection_free_test_line,
                                                lanelet::BasicLineString2d& previous_test_line,
                                                const uint& idx, std::deque<std::pair<lanelet::BasicLineString2d, size_t>>& last_test_lines,
                                                lanelet::BasicLineString2d& bound);
        bool checkLineDrivability(const lanelet::ConstLineString3d &lineToCheck);

};