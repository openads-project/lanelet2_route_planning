#include "lanelet2_route_planning/global_planner_node.hpp"

        void GlobalPlanner::initializeLocalPathExtraction(const lanelet2_route_planning_interfaces::msg::Route& route_global)
        {
            // Reset ego-position corresponding sample values
            ego_pos_sample_cl_ = 0;
            ego_pos_sample_lb_ = 0;
            ego_pos_sample_rb_ = 0;
            // Get the shortest-path-centerline sample of the target position
            target_sample_cl_ = findNearestSample(route_global.target_position, route_global.shortest_path);
        }
        
        void GlobalPlanner::extractLocalMapInfo(const geometry_msgs::msg::PoseWithCovarianceStamped& cur_pose,
                                const lanelet2_route_planning_interfaces::msg::DriveableSpace& driveable_space_global,
                                lanelet2_route_planning_interfaces::msg::DriveableSpace& driveable_space_local,
                                const lanelet2_route_planning_interfaces::msg::Route& route_global,
                                lanelet2_route_planning_interfaces::msg::Route& route_local)
        {
            // Find sample of shortest path centerline correspondint to the current ego-position
            ego_pos_sample_cl_ = findNearestSample(cur_pose.pose.pose.position, route_global.shortest_path, ego_pos_sample_cl_);
            RCLCPP_INFO_STREAM(get_logger(), "The sample in the shortest path corresponding to the current ego-position has id " << ego_pos_sample_cl_);
            remaining_shortest_path_={route_global.shortest_path.begin() + ego_pos_sample_cl_, route_global.shortest_path.begin() + target_sample_cl_}; 
            //RCLCPP_INFO_STREAM(get_logger(), "Current Ego Position: (" << cur_pose.pose.pose.position.x << "|" << cur_pose.pose.pose.position.y <<")");
            //RCLCPP_INFO_STREAM(get_logger(), "Next Shortest Path Sample: (" << route_global.shortest_path[ego_pos_sample_cl_].x << "|" << route_global.shortest_path[ego_pos_sample_cl_].y <<")");
            RCLCPP_INFO_STREAM(get_logger(), "Length of remaining shortest path: " << accumulatedLength(remaining_shortest_path_));
        } 

        double GlobalPlanner::accumulatedLength(const std::vector<geometry_msgs::msg::Point>& point_list)
        {
            double length=0.0;
            for(int i=0; i<point_list.size()-1; i++)
            {
                length+=distance(point_list[i],point_list[i+1]);
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