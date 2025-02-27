#pragma once

#include <memory>
#include <string>
#include <vector>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/Route.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <lanelet2_map_interface/lanelet2_map_interface.hpp>
#include <new_lanelet2_route_planning_interfaces/action/global_maneuver.hpp>
#include <new_lanelet2_route_planning_interfaces/msg/route.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace new_lanelet2_route_planning {

template <typename C>
struct is_vector : std::false_type {};
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

namespace ll = lanelet;

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

  void egoDataCallback(const perception_msgs::msg::EgoData::SharedPtr msg);

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

  bool planRoute(const geometry_msgs::msg::Point &destination, ll::routing::Route &route);

 private:
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr subscriber_ego_data_;

  rclcpp_action::Server<new_lanelet2_route_planning_interfaces::action::GlobalManeuver>::SharedPtr action_server_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  perception_msgs::msg::EgoData latest_ego_data_;

  std::unique_ptr<LL2MapInterface> ll2_interface_;

  std::string ll2_map_server_name_ = "ll2_map_server";

  double route_undershoot_distance_ = 0.0;

  double route_overshoot_distance_ = 0.0;
};

}  // namespace new_lanelet2_route_planning
