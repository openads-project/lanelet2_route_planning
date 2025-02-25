#include <functional>
#include <thread>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <perception_msgs/msg/ego_data.hpp>

#include "new_lanelet2_route_planning/new_lanelet2_route_planning.hpp"
#include "new_lanelet2_route_planning/utilities.hpp"

namespace new_lanelet2_route_planning {

/**
 * @brief Constructor
 *
 * @param options node options
 */
NewLanelet2RoutePlanning::NewLanelet2RoutePlanning() : Node("new_lanelet2_route_planning") {
  this->declareAndLoadParameter("ll2_map_server_name", ll2_map_server_name_, "Name of lanelet2_map_server node", false,
                                false, true);

  this->setup();
  test_timer_ = create_wall_timer(1.0s, std::bind(&NewLanelet2RoutePlanning::planRoute, this));  // TODO: remove
}

/**
 * @brief Declares and loads a ROS parameter
 *
 * @param name name
 * @param param parameter variable to load into
 * @param description description
 * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
 * @param is_required whether failure to load parameter will stop node
 * @param read_only set parameter to read-only
 * @param from_value parameter range minimum
 * @param to_value parameter range maximum
 * @param step_value parameter range step
 * @param additional_constraints additional constraints description
 */
template <typename T>
void NewLanelet2RoutePlanning::declareAndLoadParameter(
    const std::string& name, T& param, const std::string& description, const bool add_to_auto_reconfigurable_params,
    const bool is_required, const bool read_only, const std::optional<double>& from_value,
    const std::optional<double>& to_value, const std::optional<double>& step_value,
    const std::string& additional_constraints) {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
      range.set__from_value(static_cast<T>(from_value.value()))
          .set__to_value(static_cast<T>(to_value.value()))
          .set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1.0);
      range.set__from_value(static_cast<T>(from_value.value()))
          .set__to_value(static_cast<T>(to_value.value()))
          .set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range",
                  name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr (is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "]");
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr (is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "]");
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
      param = p.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

/**
 * @brief Handles reconfiguration when a parameter value is changed
 *
 * @param parameters parameters
 * @return parameter change result
 */
rcl_interfaces::msg::SetParametersResult NewLanelet2RoutePlanning::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s'", param.get_name().c_str());
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

/**
 * @brief Sets up subscribers, publishers, etc. to configure the node
 */
void NewLanelet2RoutePlanning::setup() {
  // initialize lanelet2 interface
  ll2_interface_ = std::make_unique<LL2MapInterface>(*this, ll2_map_server_name_);

  // callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&NewLanelet2RoutePlanning::parametersCallback, this, std::placeholders::_1));

  // publisher for publishing outgoing messages
  // publisher_ = this->create_publisher<std_msgs::msg::Int32>("~/output", 10);
  // RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());

  // action server for handling action goal requests
  action_server_ = rclcpp_action::create_server<new_lanelet2_route_planning_interfaces::action::GlobalManeuver>(
      this, "~/action",
      std::bind(&NewLanelet2RoutePlanning::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NewLanelet2RoutePlanning::actionHandleCancel, this, std::placeholders::_1),
      std::bind(&NewLanelet2RoutePlanning::actionHandleAccepted, this, std::placeholders::_1));
}

/**
 * @brief Processes action goal requests
 *
 * @param uuid unique goal identifier
 * @param goal action goal
 * @return goal response
 */
rclcpp_action::GoalResponse NewLanelet2RoutePlanning::actionHandleGoal(
    const rclcpp_action::GoalUUID& uuid,
    new_lanelet2_route_planning_interfaces::action::GlobalManeuver::Goal::ConstSharedPtr goal) {
  (void)uuid;
  (void)goal;

  RCLCPP_INFO(this->get_logger(), "Received action goal request");

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Processes action cancel requests
 *
 * @param goal_handle action goal handle
 * @return cancel response
 */
rclcpp_action::CancelResponse NewLanelet2RoutePlanning::actionHandleCancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<new_lanelet2_route_planning_interfaces::action::GlobalManeuver>>
        goal_handle) {
  (void)goal_handle;

  RCLCPP_INFO(this->get_logger(), "Received request to cancel action goal");

  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Processes accepted action goal requests
 *
 * @param goal_handle action goal handle
 */
void NewLanelet2RoutePlanning::actionHandleAccepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<new_lanelet2_route_planning_interfaces::action::GlobalManeuver>>
        goal_handle) {
  // execute action in a separate thread to avoid blocking
  std::thread{std::bind(&NewLanelet2RoutePlanning::actionExecute, this, std::placeholders::_1), goal_handle}.detach();
}

/**
 * @brief Executes an action
 *
 * @param goal_handle action goal handle
 */
void NewLanelet2RoutePlanning::actionExecute(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<new_lanelet2_route_planning_interfaces::action::GlobalManeuver>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing action goal");
}

void NewLanelet2RoutePlanning::planRoute() {
  if (!ll2_interface_->map_loaded_) {
    RCLCPP_ERROR(get_logger(), "Cannot plan route, map not loaded by '%s'", ll2_map_server_name_.c_str());
    return;
  }

  // get map and traffic rules
  ll::LaneletMapConstPtr map = ll2_interface_->getMapPtr();
  ll::traffic_rules::TrafficRulesPtr traffic_rules = ll::traffic_rules::TrafficRulesFactory::create(
      ll::Locations::Germany, std::string(lanelet::Participants::Vehicle) + ":ika");  // TODO: what is this postfix?

  // project ego position to lanelet
  const perception_msgs::msg::EgoData ego_data;  // TODO: move elsewhere
  ll::ConstLanelet ego_ll = findLaneletAtEgoPosition(map, ll2_interface_->map_frame_id_, ego_data, traffic_rules);
  ll::BasicPoint2d ego_ll_position = projectPointToCenterline(ego_data, ego_ll);

  // project destination to lanelet
  const geometry_msgs::msg::PointStamped destination;  // TODO: move elsewhere
  ll::ConstLanelet destination_ll = findLaneletAtPoint(map, destination.point, traffic_rules);
  ll::BasicPoint2d destination_ll_position = projectPointToCenterline(destination.point, destination_ll);

  // TODO: add offsets to start and end

  // plan route
  ll::routing::RoutingGraphUPtr routing_graph = ll::routing::RoutingGraph::build(*map, *traffic_rules);
  auto route = routing_graph->getRoute(ego_ll, destination_ll);
  if (route) {
    // TODO: what to do with the route?
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to plan route from lanelet %ld to lanelet %ld", ego_ll.id(),
                 destination_ll.id());
  }
}

}  // namespace new_lanelet2_route_planning

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<new_lanelet2_route_planning::NewLanelet2RoutePlanning>());
  rclcpp::shutdown();

  return 0;
}
