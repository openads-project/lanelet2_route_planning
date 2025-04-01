#pragma once

#include <memory>
#include <string>
#include <vector>

#include <lanelet2_routing/Route.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <lanelet2_map_interface/lanelet2_map_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <route_planning_msgs/action/global_maneuver.hpp>
#include <route_planning_msgs/msg/route.hpp>

namespace lanelet2_route_planning {

template <typename C>
struct is_vector : std::false_type {};

template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};

template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

/**
 * @brief Lanelet2 route planning node
 */
class Lanelet2RoutePlanning : public rclcpp::Node {
 public:
  /**
   * @brief Constructor
   */
  Lanelet2RoutePlanning();

 private:
  /**
   * @brief Declares and loads a ROS parameter
   *
   * @param[in] name name
   * @param[in] param parameter variable to load into
   * @param[in] description description
   * @param[in] add_to_auto_reconfigurable_params enable reconfiguration of parameter
   * @param[in] is_required whether failure to load parameter will stop node
   * @param[in] read_only set parameter to read-only
   * @param[in] from_value parameter range minimum
   * @param[in] to_value parameter range maximum
   * @param[in] step_value parameter range step
   * @param[in] additional_constraints additional constraints description
   */
  template <typename T>
  void declareAndLoadParameter(const std::string &name, T &param, const std::string &description,
                               const bool add_to_auto_reconfigurable_params = true, const bool is_required = false,
                               const bool read_only = false, const std::optional<double> &from_value = std::nullopt,
                               const std::optional<double> &to_value = std::nullopt,
                               const std::optional<double> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");

  /**
   * @brief Handles reconfiguration when a parameter value is changed
   *
   * @param[in] parameters parameters
   * @return parameter change result
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  /**
   * @brief Sets up subscribers, publishers, etc. to configure the node
   */
  void setup();

  /**
   * @brief Loads the map and sets up the routing graph
   */
  void setupRoutingGraph();

  /**
   * @brief Callback for EgoData
   *
   * Recomputes the local route if the node is currently publishing a route.
   *
   * @param[in] msg
   */
  void egoDataCallback(const perception_msgs::msg::EgoData::SharedPtr msg);

  /**
   * @brief Callback to periodically publish the route
   */
  void publishTimerCallback();

  /**
   * @brief Action goal callback: processes a route planning request
   *
   * Plans a global route to the destination before accepting the goal.
   * Aborts any existing running action.
   *
   * @param[in] uuid unique goal identifier
   * @param[in] goal action goal
   * @return goal response
   */
  rclcpp_action::GoalResponse actionHandleGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const route_planning_msgs::action::GlobalManeuver::Goal> goal);

  /**
   * @brief Action cancel callback: cancels a running action
   *
   * Sets a flag to cancel action execution.
   *
   * @param[in] goal_handle action goal handle
   * @return cancel response
   */
  rclcpp_action::CancelResponse actionHandleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> goal_handle);

  /**
   * @brief Action accepted callback: starts action execution
   *
   * Initializes action feedback and starts action execution in a separate thread.
   *
   * @param[in] goal_handle action goal handle
   */
  void actionHandleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> goal_handle);

  /**
   * @brief Action execution: continually publishes route progress
   *
   * Periodically publishes route traveling progress.
   * Checks for goal completion or cancellation.
   *
   * @param[in] goal_handle action goal handle
   */
  void actionExecute(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> goal_handle);

  /**
   * @brief Plans a lanelet route to the destination
   *
   * Planned lanelet route is stored in latest_route_.
   *
   * @param[in] destination destination
   * @return whether route planning was successful
   */
  bool planRoute(const geometry_msgs::msg::PointStamped &destination);

  /**
   * @brief Builds a global ROS route message from the latest planned lanelet route
   *
   * The global route message only contains a global reference line along a single lane of the shortest path of the route.
   *
   * @return whether successful
   */
  bool buildGlobalRouteMessage();

  /**
   * @brief Builds an enriched ROS route message from the latest planned lanelet route
   *
   * The enriched route message is locally enriched around the ego position.
   * The enriched part contains adjacent lanes, drivable space, and regulatory elements.
   *
   * @return whether successful
   */
  bool buildEnrichedRouteMessage();

 private:
  /**
   * @brief Auto-reconfigurable parameters for dynamic reconfiguration
   */
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;

  /**
   * @brief Callback handle for dynamic parameter reconfiguration
   */
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  /**
   * @brief Timer for delayed setup required for proper lanelet2_map_server initialization
   */
  rclcpp::TimerBase::SharedPtr delayed_setup_timer_;

  /**
   * @brief Subscriber for ego data
   */
  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr subscriber_ego_data_;

  /**
   * @brief Publisher for enriched route
   */
  rclcpp::Publisher<route_planning_msgs::msg::Route>::SharedPtr publisher_route_;

  /**
   * @brief Timer for publishing route
   */
  rclcpp::TimerBase::SharedPtr publish_timer_;

  /**
   * @brief Action server
   */
  rclcpp_action::Server<route_planning_msgs::action::GlobalManeuver>::SharedPtr action_server_;

  /**
   * @brief Latest action feedback
   */
  route_planning_msgs::action::GlobalManeuver::Feedback::SharedPtr action_feedback_;

  /**
   * @brief Latest action result
   */
  route_planning_msgs::action::GlobalManeuver::Result::SharedPtr action_result_;

  /**
   * @brief Latest action goal handle
   */
  std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> action_goal_handle_;

  /**
   * @brief Callback group for action server
   */
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;

  /**
   * @brief Latest action start time to compute action duration
   */
  rclcpp::Time action_start_time_;

  /**
   * @brief Transform buffer
   */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
   * @brief Transform listener
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /**
   * @brief Latest ego data
   */
  perception_msgs::msg::EgoData latest_ego_data_;

  /**
   * @brief Lanelet2 map interface
   */
  std::unique_ptr<LL2MapInterface> ll2_interface_;

  /**
   * @brief Lanelet routing graph for current map
   */
  lanelet::routing::RoutingGraphUPtr routing_graph_;

  /**
   * @brief Destination point in map frame
   */
  geometry_msgs::msg::Point destination_;

  /**
   * @brief Latest planned lanelet route
   */
  lanelet::routing::Route latest_route_;

  /**
   * @brief Controlling flag for route publication
   */
  bool is_publishing_route_;

  /**
   * @brief Latest route message to publish
   */
  route_planning_msgs::msg::Route latest_route_msg_;

  /**
   * @brief Latest mapping between global route reference line and lanelet indices
   *
   * Used to easily find lanelet for a given point on the global reference line.
   * Indexes into latest_route_.shortestPath().
   */
  std::vector<size_t> latest_lanelet_idx_by_reference_line_point_idx_;

  /**
   * @brief Name of lanelet2_map_server node (parameter)
   */
  std::string ll2_map_server_name_ = "ll2_map_server";

  /**
   * @brief Frequency of route publication [Hz] (parameter)
   */
  double publish_frequency_ = 1.0;

  /**
   * @brief Frequency of action feedback publication [Hz] (parameter)
   */
  double action_feedback_frequency_ = 1.0;

  /**
   * @brief Distance between resampled points along route [m] (parameter)
   */
  double sampling_distance_ = 0.5;

  /**
   * @brief Distance ahead of ego position where global route is enriched with more information [m] (negative=unlimited) (parameter)
   */
  double local_route_ahead_distance_ = 30.0;

  /**
   * @brief Distance behind ego position where global route is enriched with more information [m] (negative=unlimited) (parameter)
   */
  double local_route_behind_distance_ = 10.0;

  /**
   * @brief Undershoot route by this distance before ego position [m] (parameter)
   */
  double route_undershoot_distance_ = 0.0;

  /**
   * @brief Overshoot route by this distance behind destination [m] (parameter)
   */
  double route_overshoot_distance_ = 0.0;
};

}  // namespace lanelet2_route_planning
