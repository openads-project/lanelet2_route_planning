#include "lanelet2_route_planning/global_planner_node.hpp"

bool GlobalPlanner::locateInLaneNetwork(Lanelet2RoutePlanningDatatypes::LaneletLaneNetwork lane_network, lanelet::BasicPoint2d& cur_pos, lanelet::ConstLanelet& current_ll, size_t& lane_network_route_index, size_t& lane_network_spatial_index, int16& current_lane)
{
    // Current position
    cur_pos = lanelet::BasicPoint2d(ego_pose_.pose.pose.position.x, ego_pose_.pose.pose.position.y)
    // Detect most probable current lanelet
    std::vector<std::pair<double, lanelet::ConstLanelet>> nearest_lls = lanelet::geometry::findWithin2d(llmap_->laneletLayer, cur_pos, 5.0);
    if (!nearest_lls.size())
    {
        RCLCPP_ERROR_STREAM(get_logger(), "No Lanelet at current position: " << cur_pos.x() << ", " << cur_pos.y());
        return false;
    }
    else
    {
        // Determine Yaw Angle
        // Get actual Heading
        tf2::Quaternion quat;
        tf2::fromMsg(ego_pose_.pose.pose.orientation, quat);
        float yaw = tf2::impl::getYaw(quat);
        Lanelet2Utilities::laneletSorting(cur_pos, nearest_lls, yaw, *trafficRules_, {});

        // Locate current lane and lanelet in lane network
        bool b_located = false;
        for(size_t nearest_ll_id = 0; nearest_ll_id < nearest_lls.size(); nearest_ll_id++)
        {
            for(size_t route_index = 0; route_index < lane_network.lane_hierarchy.size(); route_index++)
            {
                for(size_t spatial_index = 0; spatial_index < lane_network.lane_hierarchy[route_index].neighboring_lanelets.size(); spatial_index++)
                {
                    if(lane_network.lane_hierarchy[route_index].neighboring_lanelets[spatial_index].lanelet_id == nearest_lls[nearest_ll_id].second.id())
                    {
                        // Save information
                        current_ll = nearest_lls[nearest_ll_id].second;
                        maneuver_feedback_->current_lanelet = current_ll.id();
                        lane_network_route_index = route_index;
                        lane_network_spatial_index = spatial_index;
                        current_lane = lane_network.lane_hierarchy[route_index].neighboring_lanelets[spatial_index].lane_id;
                        b_located = true;
                        break;
                    }
                }
                if(b_located)
                {
                    break;
                }
            }
            if(b_located)
            {
                break;
            }
        }
            
        if (!b_located)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Near lanelets are not part of lane network at current position: " << cur_pos.x() << ", " << cur_pos.y());
            return false;
        }
    }
}