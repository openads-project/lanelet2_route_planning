#include "lanelet2_route_planning/global_planner_node.hpp"

rclcpp_action::GoalResponse GlobalPlanner::actionHandleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const route_planning_msgs::action::GlobalManeuver::Goal> goal) {
  (void)uuid;

  // if there is an update of the lanelet2-map pending, we can reset the flag since we're replanning with the latest map
  ll2if_->update_pending_ = false;

  const geometry_msgs::msg::PointStamped& destination = goal->destination;
  RCLCPP_INFO(this->get_logger(), "Received global maneuver request to destination (%.3f, %.3f, %.3f) in frame '%s'",
              destination.point.x, destination.point.y, destination.point.z, destination.header.frame_id.c_str());

  // transform destination to map frame
  geometry_msgs::msg::PointStamped destination_map;
  try {
    destination_map = tf_buffer_->transform(destination, ll2if_->map_frame_id_);
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform destination from frame '%s' to frame '%s': %s",
                 destination.header.frame_id.c_str(), ll2if_->map_frame_id_.c_str(), ex.what());
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "Transformed global maneuver request to destination (%.3f, %.3f, %.3f) in frame '%s'",
              destination_map.point.x, destination_map.point.y, destination_map.point.z,
              destination_map.header.frame_id.c_str());

  RCLCPP_INFO(this->get_logger(), "Planning route to destination");
  lanelet::routing::Route ll_route;
  lanelet::BasicPoint2d start_offset_point, destination_offset_point;
  lanelet::BasicPoint3d destination_on_centerline;
  if (!planLaneletRoute(ego_data_, destination, ll_route, start_offset_point, destination_on_centerline,
                        destination_offset_point)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan route, rejecting action goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  route_planning_msgs::msg::Route new_route;
  int new_initial_ego_pos_sample_cl, new_target_pos_sample_cl;
  processRoute(ego_data_, ll_route, start_offset_point, destination_on_centerline, destination_offset_point,
              new_route, new_initial_ego_pos_sample_cl, new_target_pos_sample_cl);

  if (!this->egoIsOnRoute(ego_data_, ll_route)) {
    RCLCPP_ERROR(this->get_logger(), "Ego is not on planned route, rejecting action goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // abort current action if running
  if (maneuver_goal_handle_ && maneuver_goal_handle_->is_active()) {
    RCLCPP_WARN(this->get_logger(), "Existing action detected, aborting before accepting new goal");
    maneuver_result_->destination_reached = false;
    maneuver_goal_handle_->abort(maneuver_result_);
  }

  // accept action goal request
  route_ = new_route;
  initial_ego_pos_sample_cl_ = new_initial_ego_pos_sample_cl;
  target_pos_sample_cl_ = new_target_pos_sample_cl;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanner::actionHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> goal_handle) {
  (void)goal_handle;

  // this callback is invoked when a running action is requested to cancel
  RCLCPP_WARN(get_logger(), "Received request to cancel action");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalPlanner::actionHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> goal_handle) {
  // this callback is invoked when an action goal request is accepted

  maneuver_goal_handle_ = goal_handle;

  // initialize feedback and result
  maneuver_start_time_ = now();
  maneuver_feedback_ = std::make_shared<route_planning_msgs::action::GlobalManeuver::Feedback>();
  maneuver_result_ = std::make_shared<route_planning_msgs::action::GlobalManeuver::Result>();
  // derive remaining distance while accounting for the offsets within the path length
  maneuver_feedback_->distance_remaining =
      route_.remaining_route[target_pos_sample_cl_].z - route_.remaining_route[initial_ego_pos_sample_cl_].z;
  maneuver_feedback_->time_remaining = rclcpp::Duration::from_seconds(
      maneuver_feedback_->distance_remaining /
      (route_.current_speed_limit / 3.6));  // TODO: improve estimate by accumulating with speed limits over path

  // execute the action in a separate thread to avoid blocking
  std::thread{std::bind(&GlobalPlanner::actionExecute, this, std::placeholders::_1), goal_handle}.detach();
}

void GlobalPlanner::actionExecute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> goal_handle) {
  RCLCPP_INFO(get_logger(), "Starting action execution");

  const auto goal = goal_handle->get_goal();
  const geometry_msgs::msg::PointStamped& destination = goal->destination;

  // transform destination to map frame
  geometry_msgs::msg::PointStamped destination_map;
  try {
    destination_map = tf_buffer_->transform(destination, ll2if_->map_frame_id_);
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not transform destination from frame '%s' to frame '%s', canceling action: %s",
                 destination.header.frame_id.c_str(), ll2if_->map_frame_id_.c_str(), ex.what());
    goal_handle->canceled(maneuver_result_);
    return;
  }

  rclcpp::Rate loop_rate(path_extraction_rate_);

  bool is_cancelling = false;
  while (goal_handle->is_executing() && !maneuver_result_->destination_reached) {
    // if requested, cancel route by setting destination to point closely ahead of ego along route
    if (goal_handle->is_canceling()) {
      if (!is_cancelling) {
        RCLCPP_WARN(this->get_logger(), "Canceling by re-planning route to destination %.2fm ahead",
                    cancel_distance_ahead_);
        // find route sample 'cancel_distance_ahead_' ahead of ego to set as new destination
        int ego_pos_sample_cl = findNearestSample(perception_msgs::object_access::getPosition(ego_data_),
                                                  route_.remaining_route, initial_ego_pos_sample_cl_);
        auto& ego_pos = route_.remaining_route[ego_pos_sample_cl];
        for (size_t i = ego_pos_sample_cl; i < route_.remaining_route.size(); i++) {
          double distance_ahead = route_.remaining_route[i].z - ego_pos.z;
          if (distance_ahead > cancel_distance_ahead_) {
            target_pos_sample_cl_ = i;
            route_.destination = route_.remaining_route[target_pos_sample_cl_];
            break;
          }
        }
        is_cancelling = true;
      }
    }

    lanelet::ConstLanelet cur_ego_lanelet;
    if (deriveEgoLanelet(ego_data_, cur_ego_lanelet)) {
      // check if destination is reached -> goal succeeded
      double velocity = perception_msgs::object_access::getVelLon(ego_data_);
      if (geometry::distance(lanelet::BasicPoint2d(route_.destination.x, route_.destination.y),
                             lanelet::BasicPoint2d(perception_msgs::object_access::getX(ego_data_),
                                                   perception_msgs::object_access::getY(ego_data_))) <
              target_reached_thr_ &&
          (std::fabs(velocity) < vel_threshold_target_ || !require_standstill_)) {
        RCLCPP_INFO(get_logger(), "Destination reached!");
        publishEmptyRoute();
        maneuver_result_->destination_reached = true;
        maneuver_result_->distance_traveled = route_.remaining_route.back().z;
        maneuver_result_->time_traveled = this->now() - maneuver_start_time_;
        goal_handle->succeed(maneuver_result_);
        return;
      }

      // Extract local section of driveable space and route
      route_planning_msgs::msg::Route route_local;
      if (extractLocalMapInfo(ego_data_, route_, route_local)) {
        // check if lanelet map needs to be reloaded, cancel action
        if (ll2if_->update_pending_) {
          RCLCPP_ERROR(this->get_logger(), "Lanelet map update pending, canceling action");
          this->publishEmptyRoute();
          maneuver_result_->destination_reached = false;
          ll2if_->update_pending_ = false;
          goal_handle->abort(maneuver_result_);
          return;
        }

        // check if route has been completed without reaching destination -> abort goal
        if (route_local.remaining_route.size() <= 1) {
          RCLCPP_ERROR(this->get_logger(), "Route completed without reaching destination");
          this->publishEmptyRoute();
          maneuver_result_->destination_reached = false;
          maneuver_result_->distance_traveled = route_local.traveled_route.back().z;
          maneuver_result_->time_traveled = this->now() - maneuver_start_time_;
          goal_handle->abort(maneuver_result_);
          return;
        }

        // update feedback
        double distance_traveled_to_last_path_point = 0.0;
        double distance_last_path_point_to_ego = 0.0;
        if (!route_local.traveled_route.empty()) {
          distance_traveled_to_last_path_point = route_local.traveled_route.back().z;
          distance_last_path_point_to_ego = std::sqrt(std::pow(route_local.traveled_route.back().x, 2) +
                                                      std::pow(route_local.traveled_route.back().y, 2));
        }
        maneuver_feedback_->distance_traveled = distance_traveled_to_last_path_point + distance_last_path_point_to_ego;
        maneuver_feedback_->time_traveled = this->now() - maneuver_start_time_;
        maneuver_feedback_->distance_remaining = 0.0;
        maneuver_feedback_->time_remaining = rclcpp::Duration::from_seconds(0.0);
        if (!route_local.remaining_route.empty()) {
          maneuver_feedback_->distance_remaining =
              route_local.remaining_route.back().z - maneuver_feedback_->distance_traveled;
          maneuver_feedback_->time_remaining = rclcpp::Duration::from_seconds(
              maneuver_feedback_->distance_remaining /
              (route_local.current_speed_limit /
               3.6));  // TODO: improve estimate by accumulating with speed limits over path
        }

        // publish the current sequence as action feedback
        goal_handle->publish_feedback(maneuver_feedback_);
        RCLCPP_INFO(get_logger(), "Publishing action feedback");
      } else {
        publishEmptyRoute();
        maneuver_result_->destination_reached = false;
        goal_handle->abort(maneuver_result_);
        return;
      }
    }

    // sleep
    loop_rate.sleep();
    continue;
  }
}