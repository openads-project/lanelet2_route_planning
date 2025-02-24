#pragma once

#include <memory>
#include <string>
#include <vector>

#include <lanelet2_map_interface/lanelet2_map_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <new_lanelet2_route_planning_interfaces/action/global_maneuver.hpp>

namespace new_lanelet2_route_planning {

template <typename C>
struct is_vector : std::false_type {};
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

class NewLanelet2RoutePlanning : public rclcpp::Node {
 public:
  NewLanelet2RoutePlanning();

 private:
  template <typename T>
  void declareAndLoadParameter(const std::string &name, T &param, const std::string &description,
                               const bool add_to_auto_reconfigurable_params = true, const bool is_required = false,
                               const bool read_only = false, const std::optional<double> &from_value = std::nullopt,
                               const std::optional<double> &to_value = std::nullopt,
                               const std::optional<double> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  void setup();

  rclcpp_action::GoalResponse actionHandleGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const new_lanelet2_route_planning_interfaces::action::GlobalManeuver::Goal> goal);

  rclcpp_action::CancelResponse actionHandleCancel(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<new_lanelet2_route_planning_interfaces::action::GlobalManeuver>>
          goal_handle);

  void actionHandleAccepted(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<new_lanelet2_route_planning_interfaces::action::GlobalManeuver>>
          goal_handle);

  void actionExecute(const std::shared_ptr<
                     rclcpp_action::ServerGoalHandle<new_lanelet2_route_planning_interfaces::action::GlobalManeuver>>
                         goal_handle);

 private:
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  rclcpp_action::Server<new_lanelet2_route_planning_interfaces::action::GlobalManeuver>::SharedPtr action_server_;

  std::unique_ptr<LL2MapInterface> ll2_interface_;

  std::string ll2_map_server_name_ = "ll2_map_server";
};

}  // namespace new_lanelet2_route_planning
