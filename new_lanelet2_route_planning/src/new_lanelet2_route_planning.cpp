#include <functional>
#include <thread>

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
  this->declareAndLoadParameter("route_undershoot_distance", route_undershoot_distance_,
                                "Undershoot route by this distance before ego position", true, false, false);
  this->declareAndLoadParameter("route_overshoot_distance", route_overshoot_distance_,
                                "Overshoot route by this distance behind destination", true, false, false);

  // initialize lanelet2 interface
  ll2_interface_ = std::make_unique<LL2MapInterface>(*this, ll2_map_server_name_);

  // delay setup to spin to allow to load the map
  delayed_setup_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() { this->setup(); });
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

  delayed_setup_timer_.reset();

  // build routing graph
  this->setupRoutingGraph();

  // callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&NewLanelet2RoutePlanning::parametersCallback, this, std::placeholders::_1));

  // tf transform listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // publishers
  publisher_route_ = this->create_publisher<route_planning_msgs::msg::Route>("~/route", 1);
  publish_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                           [this]() { publisher_route_->publish(latest_route_); }); // TODO: remove timer?

  // subscribers
  subscriber_ego_data_ = this->create_subscription<perception_msgs::msg::EgoData>(
      "~/ego_data", 1, std::bind(&NewLanelet2RoutePlanning::egoDataCallback, this, std::placeholders::_1));

  // action server for handling action goal requests
  action_server_ = rclcpp_action::create_server<route_planning_msgs::action::GlobalManeuver>(
      this, "~/route",
      std::bind(&NewLanelet2RoutePlanning::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NewLanelet2RoutePlanning::actionHandleCancel, this, std::placeholders::_1),
      std::bind(&NewLanelet2RoutePlanning::actionHandleAccepted, this, std::placeholders::_1));
}

void NewLanelet2RoutePlanning::setupRoutingGraph() {

  bool success;
  if (!ll2_interface_->map_loaded_) {
    RCLCPP_FATAL(get_logger(), "Cannot build routing graph, map not loaded by '%s'", ll2_map_server_name_.c_str());
    exit(EXIT_FAILURE);
  }

  // get map and traffic rules
  ll::LaneletMapConstPtr map = ll2_interface_->getMapPtr();
  ll::traffic_rules::TrafficRulesPtr traffic_rules = ll::traffic_rules::TrafficRulesFactory::create(
      ll::Locations::Germany,
      std::string(lanelet::Participants::Vehicle) + ":ika");  // TODO: what is this postfix? move to constant?

  // build routing graph
  success = buildRoutingGraph(map, traffic_rules, routing_graph_);
  if (!success) {
    RCLCPP_FATAL(get_logger(), "Failed to build routing graph");
    exit(EXIT_FAILURE);
  }
}

void NewLanelet2RoutePlanning::egoDataCallback(const perception_msgs::msg::EgoData::SharedPtr msg) {
  latest_ego_data_ = *msg;
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
    route_planning_msgs::action::GlobalManeuver::Goal::ConstSharedPtr goal) {
  (void)uuid;
  (void)goal;

  const geometry_msgs::msg::PointStamped& destination = goal->destination;
  RCLCPP_INFO(this->get_logger(), "Received request to plan route to destination (%.3f, %.3f, %.3f) in frame '%s'",
              destination.point.x, destination.point.y, destination.point.z, destination.header.frame_id.c_str());

  // transform destination to map frame
  geometry_msgs::msg::PointStamped destination_map;
  try {
    destination_map = tf_buffer_->transform(destination, ll2_interface_->map_frame_id_);
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform destination from frame '%s' to frame '%s': %s",
                 destination.header.frame_id.c_str(), ll2_interface_->map_frame_id_.c_str(), ex.what());
    return rclcpp_action::GoalResponse::REJECT;
  }

  // plan route
  RCLCPP_INFO(this->get_logger(), "Planning route to destination ...");
  auto t0 = std::chrono::steady_clock::now();
  ll::routing::Route route;
  bool success = this->planRoute(destination_map.point, route);
  auto t1 = std::chrono::steady_clock::now();
  auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan route to destination, rejecting request");
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    RCLCPP_INFO(this->get_logger(), "Successfully planned route to destination (%.3fs)", dt);
  }

  // TODO: check egoIsOnRoute? is there any way ego lanelet could not be on the route?

  // TODO: abort current action if running

  // convert route to custom format
  route_planning_msgs::msg::Route route_msg = laneletToRosRoute(route, ll2_interface_->map_frame_id_, routing_graph_);
  route_msg.header.stamp = this->now();
  latest_route_ = route_msg;




  // accept action goal request
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
        rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>>
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
        rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>>
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
        rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing action goal");
}

bool NewLanelet2RoutePlanning::planRoute(const geometry_msgs::msg::Point& destination, ll::routing::Route& route) {
  bool success;
  if (!ll2_interface_->map_loaded_) {
    RCLCPP_ERROR(get_logger(), "Cannot plan route, map not loaded by '%s'", ll2_map_server_name_.c_str());
    return false;
  }

  // get map and traffic rules
  ll::LaneletMapConstPtr map = ll2_interface_->getMapPtr();
  ll::traffic_rules::TrafficRulesPtr traffic_rules = ll::traffic_rules::TrafficRulesFactory::create(
      ll::Locations::Germany,
      std::string(lanelet::Participants::Vehicle) + ":ika");  // TODO: what is this postfix? move to constant?

  // project ego position to lanelet
  ll::ConstLanelet ego_ll;
  success = findLaneletAtEgoPosition(map, ll2_interface_->map_frame_id_, latest_ego_data_, ego_ll, traffic_rules);
  if (!success) {
    RCLCPP_ERROR(get_logger(), "Failed to find lanelet at ego position");
    return false;
  }
  ll::BasicPoint2d ego_ll_position = projectPointToCenterline(latest_ego_data_, ego_ll);

  // project destination to lanelet
  ll::ConstLanelet destination_ll;
  success = findLaneletAtPoint(map, destination, destination_ll, traffic_rules);
  if (!success) {
    RCLCPP_ERROR(get_logger(), "Failed to find lanelet at destination");
    return false;
  }
  ll::BasicPoint2d destination_ll_position = projectPointToCenterline(destination, destination_ll);

  // undershoot/overshoot route endpoints
  // TODO: still needed?
  ll::ConstLanelet undershot_ego_ll =
      followLanelet(routing_graph_, ego_ll, ego_ll_position, -std::abs(route_undershoot_distance_));
  ll::ConstLanelet overshot_destination_ll =
      followLanelet(routing_graph_, destination_ll, destination_ll_position, route_overshoot_distance_);
  // TODO: check that start/end are not the same lanelet (?) (see L314 in global_planner_node.cpp)

  // plan route
  const int routing_cost_id = 0;  // RoutingCostDistance
  auto planned_route = routing_graph_->getRoute(undershot_ego_ll, overshot_destination_ll, routing_cost_id);
  if (planned_route) {
    route = std::move(*planned_route);
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to plan route from lanelet %ld to lanelet %ld", ego_ll.id(),
                 destination_ll.id());
    return false;
  }
}

}  // namespace new_lanelet2_route_planning

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<new_lanelet2_route_planning::NewLanelet2RoutePlanning>());
  rclcpp::shutdown();

  return 0;
}
