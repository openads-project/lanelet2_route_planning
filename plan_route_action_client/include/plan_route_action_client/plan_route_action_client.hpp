#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <route_planning_msgs/action/global_maneuver.hpp>  // TODO: rename to PlanRoute.action

namespace plan_route_action_client {

template <typename C>
struct is_vector : std::false_type {};

template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};

template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

/**
 * @brief Action client node for planning a route
 */
class PlanRouteActionClient : public rclcpp::Node {
  using GlobalManeuver = route_planning_msgs::action::GlobalManeuver;
  using GoalHandleGlobalManeuver = rclcpp_action::ClientGoalHandle<GlobalManeuver>;

 public:
  /**
  * @brief Constructor
  */
  PlanRouteActionClient();

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
   * @brief Callback for goal pose (most likely received from RViz)
   *
   * @param[in] msg goal pose
   */
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Callback for automatically planning a route, e.g., if waypoints are given
   *
   * Does nothing, if no waypoints are given and random planning is disabled.
   * Precedence: random planning, waypoints
   */
  void autoPlanningTimerCallback();

  /**
   * @brief Plans to next waypoint
   */
  void planToNextWaypoint();

  /**
   * @brief Plans to a random destination
   */
  void planToRandomDestination();

  /**
   * @brief Sends a goal to the action server
   *
   * @param[in] msg goal pose
   */
  void sendGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Callback for goal response from the action server
   *
   * @param[in] goal_handle goal handle
   */
  void goalResponseCallback(const GoalHandleGlobalManeuver::SharedPtr &goal_handle);

  /**
   * @brief Callback for feedback from the action server
   *
   * @param[in] goal_handle goal handle
   * @param[in] feedback action feedback
   */
  void feedbackCallback(GoalHandleGlobalManeuver::SharedPtr goal_handle,
                        const std::shared_ptr<const GlobalManeuver::Feedback> feedback);

  /**
   * @brief Callback for result from the action server
   *
   * @param[in] result action result
   */
  void resultCallback(const GoalHandleGlobalManeuver::WrappedResult &result);

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
   * @brief Subscriber for goal pose
   */
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;

  /**
   * @brief Action client
   */
  rclcpp_action::Client<GlobalManeuver>::SharedPtr action_client_;

  /**
   * @brief Goal handle
   */
  std::shared_future<GoalHandleGlobalManeuver::SharedPtr> goal_handle_future_;

  /**
   * @brief Timer to automatically plan route, e.g., if waypoints are given
   */
  rclcpp::TimerBase::SharedPtr auto_planning_timer_;

  /**
   * @brief WGS84 waypoints to endlessly follow
   */
  std::vector<std::pair<double, double>> waypoints_;

  /**
   * @brief Index of next waypoint to follow
   */
  size_t waypoint_idx_ = 0;

  /**
   * @brief Whether one goal has been completed (succeeded or failed)
   */
  bool has_completed_one_goal_ = false;

  /**
   * @brief WGS84 waypoints to endlessly follow (parameter)
   *
   * list of strings with comma-separated '<LATITUDE>,<LONGITUDE>'
   */
  std::vector<std::string> waypoints_param_;

  /**
   * @brief Whether to plan a route to a random destination (parameter)
   */
  bool enable_random_destination_ = false;

  /**
   * @brief Whether to continuously plan a new route (parameter)
   *
   * Either to the next waypoint or to a random destination, if enabled
   */
  bool enable_continuous_planning_ = false;

  /**
   * @brief Flag to cancel the route planning action (parameter)
   */
  bool cancel_route_ = false;
};

}  // namespace plan_route_action_client
