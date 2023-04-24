#include "lanelet2_route_planning/global_planner_node.hpp"

        void GlobalPlanner::initializeLocalPathExtraction(const route_planning_interfaces::msg::Route& route_global)
        {
            // Reset sample values
            ego_pos_sample_cl_ = 0;
            lbehind_sample_drivspace_left_ = 0;
            lbehind_sample_drivspace_right_ = 0;
            lahead_sample_drivspace_left_ = 0;
            lahead_sample_drivspace_right_ = 0;

            // Get the shortest-path-centerline sample of the target position
            target_sample_cl_ = findNearestSampleReverse(route_global.target_position, route_global.shortest_path);

        }
        
        void GlobalPlanner::extractLocalMapInfo(const geometry_msgs::msg::PoseWithCovarianceStamped& cur_pose,
                                const route_planning_interfaces::msg::DriveableSpace& driveable_space_global,
                                route_planning_interfaces::msg::DriveableSpace& driveable_space_local,
                                const route_planning_interfaces::msg::Route& route_global,
                                route_planning_interfaces::msg::Route& route_local)
        {
            rclcpp::Time stamp_time = now();
            // Find sample of shortest path centerline correspondint to the current ego-position
            ego_pos_sample_cl_ = findNearestSample(cur_pose.pose.pose.position, route_global.shortest_path, ego_pos_sample_cl_);
            if(ego_pos_sample_cl_>=target_sample_cl_)
            {
                return;
            }
            RCLCPP_INFO_STREAM(get_logger(), "The sample in the shortest path corresponding to the current ego-position has id " << ego_pos_sample_cl_);
            RCLCPP_INFO_STREAM(get_logger(), "The sample in the shortest path corresponding to the current target-position has id " << target_sample_cl_);
            remaining_shortest_path_ = {route_global.shortest_path.begin() + ego_pos_sample_cl_, route_global.shortest_path.begin() + target_sample_cl_}; 
            std::vector<double> sp_accumulated_length_vec;
            double sp_length = accumulatedLength(remaining_shortest_path_, sp_accumulated_length_vec);
            RCLCPP_INFO_STREAM(get_logger(), "Length of remaining shortest path: " << sp_length);
            // Get the start and end sample of the local shortest path with respect to look-ahead/behind distance
            double velocity = 38.0/3.6; // To-Do: add actual vehicle velocity here
            double look_ahead_distance = std::max(look_ahead_distance_min_, look_ahead_time_*velocity);
            // Find the look-ahead sample
            unsigned int look_ahead_sample;
            for(int i=0; i<sp_accumulated_length_vec.size(); i++)
            {
                look_ahead_sample=ego_pos_sample_cl_+i;
                if(sp_accumulated_length_vec[i]>=look_ahead_distance)
                {
                    break;
                }
            }
            RCLCPP_INFO_STREAM(get_logger(), "The the look-ahead-sample of the centerline has id " << look_ahead_sample);

            // Find the look-behind sample
            unsigned int look_behind_sample;
            double accum_length=0.0;
            for(int i=ego_pos_sample_cl_; i>0; i--)
            {
                accum_length+=distance(route_global.shortest_path[i],route_global.shortest_path[i-1]);
                look_behind_sample=i;
                if(accum_length>=look_behind_distance_)
                {
                    break;
                }
            }
            RCLCPP_INFO_STREAM(get_logger(), "The the look-behind-sample of the centerline has id " << look_behind_sample);

            // To-Do: Rest of Route-Object
            // ...

            // Now we've got our local-section of the route
            route_planning_interfaces::msg::Route temp_route;
            temp_route.header.stamp = stamp_time;
            temp_route.target_position = route_global.target_position;
            temp_route.header.frame_id = ll2if_->map_frame_id_; // currently it's map --> will be changed through transform function
            temp_route.shortest_path = {route_global.shortest_path.begin() + look_behind_sample, route_global.shortest_path.begin() + look_ahead_sample};
   

            // Now extract the local driveable space
            // Find nearest Boundary-Sample for left and right boundary at look-ahead and look-behind point
            lbehind_sample_drivspace_left_ = findNearestSample(temp_route.shortest_path.front(), driveable_space_global.boundaries.left, lbehind_sample_drivspace_left_);
            RCLCPP_INFO_STREAM(get_logger(), "lbehind_sample_drivspace_left_ " << lbehind_sample_drivspace_left_);
            lbehind_sample_drivspace_right_ = findNearestSample(temp_route.shortest_path.front(), driveable_space_global.boundaries.right, lbehind_sample_drivspace_right_);
            RCLCPP_INFO_STREAM(get_logger(), "lbehind_sample_drivspace_right_ " << lbehind_sample_drivspace_right_);
            lahead_sample_drivspace_left_ = findNearestSample(temp_route.shortest_path.back(), driveable_space_global.boundaries.left, lahead_sample_drivspace_left_);
            RCLCPP_INFO_STREAM(get_logger(), "lahead_sample_drivspace_left_ " << lahead_sample_drivspace_left_);
            lahead_sample_drivspace_right_ = findNearestSample(temp_route.shortest_path.back(), driveable_space_global.boundaries.right, lahead_sample_drivspace_right_);
            RCLCPP_INFO_STREAM(get_logger(), "lahead_sample_drivspace_right_ " << lahead_sample_drivspace_right_);

            // To-Do: Extract restricting areas
            // ...

            // Now we've got our local-section of the driveable space
            route_planning_interfaces::msg::DriveableSpace temp_ds;
            temp_ds.header.stamp = stamp_time;
            temp_ds.header.frame_id = ll2if_->map_frame_id_; // currently it's map --> will be changed through transform function
            temp_ds.boundaries.left = {driveable_space_global.boundaries.left.begin() + lbehind_sample_drivspace_left_, driveable_space_global.boundaries.left.begin() + lahead_sample_drivspace_left_};
            temp_ds.boundaries.right = {driveable_space_global.boundaries.right.begin() + lbehind_sample_drivspace_right_, driveable_space_global.boundaries.right.begin() + lahead_sample_drivspace_right_};
            RCLCPP_INFO_STREAM(get_logger(), "temp_ds.boundaries.left.size() " << temp_ds.boundaries.left.size());
            RCLCPP_INFO_STREAM(get_logger(), "temp_ds.boundaries.right.size() " << temp_ds.boundaries.right.size());
            // Now transform the route- and driveable-space-object
            geometry_msgs::msg::TransformStamped tf;
            try {
                tf = tf_buffer_->lookupTransform(local_vehicle_frame_id_, ll2if_->map_frame_id_, tf2::TimePointZero);
                tf2::doTransform(temp_route, route_local, tf);
                tf2::doTransform(temp_ds, driveable_space_local, tf);
                local_route_pub_->publish(route_local);
                local_driveable_space_pub_->publish(driveable_space_local);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Could not transform " << ll2if_->map_frame_id_ << "to " << local_vehicle_frame_id_ << ": " << ex.what());
                return;
            }
        } 

        double GlobalPlanner::accumulatedLength(const std::vector<geometry_msgs::msg::Point>& point_list, std::vector<double>& accumulated_length)
        {
            double length=0.0;
            accumulated_length.push_back(length);
            for(int i=0; i<point_list.size()-1; i++)
            {
                length+=distance(point_list[i],point_list[i+1]);
                accumulated_length.push_back(length);
            }
            return length;
        }

        double GlobalPlanner::distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
        {
            return std::sqrt(std::pow(p1.x - p2.x, 2) +
                            std::pow(p1.y - p2.y, 2) +
                            std::pow(p1.z - p2.z, 2));
        }

        unsigned int GlobalPlanner::findNearestSample(const geometry_msgs::msg::Point& ref_point, const std::vector<geometry_msgs::msg::Point>& point_list, const unsigned int& start_index)
        {
            double min_distance = std::numeric_limits<double>::max();
            unsigned int nearest_index = start_index;
            for (unsigned int i = start_index; i<point_list.size(); i++) {
                double dist = distance(ref_point, point_list[i]);
                if (dist < min_distance) {
                    min_distance = dist;
                    nearest_index = i; // Update the last index to the current index
                } else if (dist > min_distance) {
                    break; // Stop searching if the distance starts increasing again
                }
            }
            return nearest_index;
        }

        unsigned int GlobalPlanner::findNearestSampleReverse(const geometry_msgs::msg::Point& ref_point, const std::vector<geometry_msgs::msg::Point>& point_list)
        {
            double min_distance = std::numeric_limits<double>::max();
            unsigned int start_index = point_list.size()-1;
            unsigned int nearest_index = start_index;
            for (unsigned int i = start_index; i>=0; i--) {
                double dist = distance(ref_point, point_list[i]);
                if (dist < min_distance) {
                    min_distance = dist;
                    nearest_index = i; // Update the last index to the current index
                } else if (dist > min_distance) {
                    break; // Stop searching if the distance starts increasing again
                }
            }
            return nearest_index;
        }