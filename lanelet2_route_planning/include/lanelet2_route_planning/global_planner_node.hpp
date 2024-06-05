#include <boost/geometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2/impl/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_route_planning_msgs/tf2_route_planning_msgs.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include <route_planning_msgs/action/global_maneuver.hpp>
#include <route_planning_msgs/msg/boundaries.hpp>
#include <route_planning_msgs/msg/driveable_space.hpp>
#include <route_planning_msgs/msg/lane.hpp>
#include <route_planning_msgs/msg/lane_separator.hpp>
#include <route_planning_msgs/msg/regulatory_element.hpp>
#include <route_planning_msgs/msg/route.hpp>

#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs_utils/object_access.hpp>

#include <lanelet2_map_interface/lanelet2_map_interface.hpp>
#include <lanelet2_utilities/lanelet2_utils.hpp>

#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

using namespace std::chrono_literals;

class GlobalPlanner : public rclcpp::Node {
 public:
  GlobalPlanner();

 private:
  //tf2
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Inputs
  std::unique_ptr<LL2MapInterface> ll2if_;
  perception_msgs::msg::EgoData ego_data_;

  // Lanelet2 Variables
  traffic_rules::TrafficRulesPtr trafficRules_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, std::string(lanelet::Participants::Vehicle) + ":ika");
  // Dummy traffic rules and routing graph for bicycles used to incorporate drivable bicycle lanes in our lane boundaries
  traffic_rules::TrafficRulesPtr trafficRulesBicycle_ =
      traffic_rules::TrafficRulesFactory::create(std::string(Locations::Germany) + ":dummy", Participants::Bicycle);

  // Outputs
  route_planning_msgs::msg::Route route_;

  // Parameters
  std::string vehicle_frame_id_ = "base_link";
  std::string map_server_name_ = "ll2_map_server";

  double ds_sample_ = 0.5;
  double smooth_factor_ = 2.0;
  double lateral_driv_space_width_ = 100.0;

  double ego_data_timeout_ = 0.2;
  double vel_threshold_target_ = 1.0;  // m/s
  double target_reached_thr_ = 1.0;    // m
  bool require_standstill_ = false;

  double offset_behind_distance_ = 0.0;
  double offset_ahead_distance_ = 0.0;

  double cancel_distance_ahead_ = 20.0;

  // Maneuver Execution
  rclcpp::Time maneuver_start_time_;

  // Map Extraction
  double path_extraction_rate_ = 10.0;
  unsigned int initial_ego_pos_sample_cl_ = 0;
  unsigned int target_pos_sample_cl_ = 0;
  double look_ahead_time_ = 10.0;
  double look_ahead_distance_min_ = 50.0;
  double look_behind_distance_ = 20.0;
  rclcpp::Publisher<route_planning_msgs::msg::Route>::SharedPtr route_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr startup_timer_;

  // Subscriptions
  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr map_pose_sub_;

  // Action Server
  rclcpp_action::Server<route_planning_msgs::action::GlobalManeuver>::SharedPtr maneuver_action_server_;
  route_planning_msgs::action::GlobalManeuver::Feedback::SharedPtr maneuver_feedback_;
  route_planning_msgs::action::GlobalManeuver::Result::SharedPtr maneuver_result_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> maneuver_goal_handle_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;

  // Function Definitions
  // global_planner_node.cpp
  void initializeGlobalPlanner();
  void loadParameters();
  void egoDataCallback(perception_msgs::msg::EgoData::SharedPtr msg);
  bool deriveEgoLanelet(const perception_msgs::msg::EgoData ego_data, lanelet::ConstLanelet& ego_lanelet);
  bool deriveDestinationLanelet(const geometry_msgs::msg::PointStamped destination,
                                lanelet::ConstLanelet& destination_lanelet);
  bool planLaneletRoute(const perception_msgs::msg::EgoData ego_data,
                        const geometry_msgs::msg::PointStamped destination, lanelet::routing::Route& lanelet_route,
                        lanelet::BasicPoint2d& start_offset_point, lanelet::BasicPoint3d& destination_on_centerline,
                        lanelet::BasicPoint2d& destination_offset_point);
  void processRoute(const perception_msgs::msg::EgoData ego_data, const lanelet::routing::Route ll_route,
                    const lanelet::BasicPoint2d& start_offset_point,
                    const lanelet::BasicPoint3d& destination_on_centerline,
                    const lanelet::BasicPoint2d& destination_offset_point, route_planning_msgs::msg::Route& route_out,
                    int& initial_ego_pos_sample_cl_out, int& target_pos_sample_cl_out);
  void publishEmptyRoute();

  // map_extraction.cpp
  bool extractLocalMapInfo(const perception_msgs::msg::EgoData& ego_data,
                           const route_planning_msgs::msg::Route& route_global,
                           route_planning_msgs::msg::Route& route_local);
  double distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2, const bool ignore_z = true);
  unsigned int findNearestSample(const geometry_msgs::msg::Point& ref_point,
                                 const std::vector<geometry_msgs::msg::Point>& point_list,
                                 const unsigned int& start_index = 0);
  unsigned int findNearestSampleReverse(const geometry_msgs::msg::Point& ref_point,
                                        const std::vector<geometry_msgs::msg::Point>& point_list);

  // maneuver_action_fcns.cpp
  rclcpp_action::GoalResponse actionHandleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const route_planning_msgs::action::GlobalManeuver::Goal> goal);

  rclcpp_action::CancelResponse actionHandleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> goal_handle);

  void actionHandleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> goal_handle);

  void actionExecute(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> goal_handle);

  // utils.cpp
  void accumulateDistanceAlong2DPath(std::vector<geometry_msgs::msg::Point>& path, const double initial_distance = 0.0);
  std::vector<geometry_msgs::msg::Point> processLineString(lanelet::BasicLineString2d& line_string);
  route_planning_msgs::msg::DriveableSpace sampleDriveableSpace(const lanelet::BasicLineString2d& centerline);
  std::vector<geometry_msgs::msg::Point> sampleLinestring(const lanelet::BasicLineString2d& centerline,
                                                          const double test_dis, const bool b_right);
  void sampleRouteBoundary(const lanelet::routing::Route& route, const lanelet::routing::LaneletPath& shortest_path,
                           std::vector<geometry_msgs::msg::Point>& bound_left,
                           std::vector<geometry_msgs::msg::Point>& bound_right);

  bool handleInwardCorner(const lanelet::BasicPoint2d& base_p, lanelet::BasicPoint2d& best_point,
                          const std::pair<lanelet::BasicLineString2d, size_t>*& last_intersection_free_test_line,
                          lanelet::BasicLineString2d& previous_test_line, const uint& idx,
                          std::deque<std::pair<lanelet::BasicLineString2d, size_t>>& last_test_lines,
                          lanelet::BasicLineString2d& bound);
  bool checkLineDrivability(const lanelet::ConstLineString3d& lineToCheck);
  route_planning_msgs::msg::LaneSeparator deriveLaneSeparator(const lanelet::ConstLineString2d& linestring);
  uint8_t deriveValueForSpeedLimitType(const std::shared_ptr<const lanelet::RegulatoryElement> regelem,
                                       const std::vector<lanelet::ConstLineString3d> refering_elems);
  uint8_t trafficSignCode2Type(const std::string tsign_code);
  bool calcIntersection(const geometry_msgs::msg::Point p1, const geometry_msgs::msg::Point p2,
                        const geometry_msgs::msg::Point p3, const geometry_msgs::msg::Point p4, double& lambda);
  void setEffectLineS(route_planning_msgs::msg::Route& route);
};