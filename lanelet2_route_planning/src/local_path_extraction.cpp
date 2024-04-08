#include "lanelet2_route_planning/global_planner_node.hpp"

        void GlobalPlanner::initializeLocalPathExtraction(const route_planning_msgs::msg::Route& route_global)
        {
            // Reset sample values
            ego_pos_sample_cl_ = 0;
            lbehind_sample_rbound_left_ = 0;
            lbehind_sample_rbound_right_ = 0;
            lahead_sample_rbound_left_ = 0;
            lahead_sample_rbound_right_ = 0;
            lbehind_sample_drivspace_left_ = 0;
            lbehind_sample_drivspace_right_ = 0;
            lahead_sample_drivspace_left_ = 0;
            lahead_sample_drivspace_right_ = 0;

            // Get the shortest-path-centerline sample of the target position
            target_sample_cl_ = findNearestSampleReverse(route_global.target_position, route_global.shortest_path);

        }
        
        void GlobalPlanner::extractLocalMapInfo(const geometry_msgs::msg::PoseWithCovariance& cur_pose,
                                const route_planning_msgs::msg::DriveableSpace& driveable_space_global,
                                route_planning_msgs::msg::DriveableSpace& driveable_space_local,
                                const route_planning_msgs::msg::Route& route_global,
                                route_planning_msgs::msg::Route& route_local)
        {
            rclcpp::Time stamp_time = now();

            // Determine current lanelet
            // Get actual Heading
            tf2::Quaternion quat;
            tf2::fromMsg(cur_pose.pose.orientation, quat);
            float yaw = tf2::impl::getYaw(quat);
            // Get nearest Lanelets
            std::vector<std::pair<double, lanelet::ConstLanelet>> nearestLanelets = lanelet::geometry::findNearest(llmap_->laneletLayer, lanelet::BasicPoint2d(cur_pose.pose.position.x, cur_pose.pose.position.y), 5);
            // Sort laneletes
            Lanelet2Utilities::laneletSorting(lanelet::BasicPoint2d(cur_pose.pose.position.x, cur_pose.pose.position.y), nearestLanelets, yaw, trafficRules_, {});
            lanelet::ConstLanelet current_ego_ll = nearestLanelets.at(0).second; // most probable current Lanelet
            
            // Find sample of shortest path centerline correspondint to the current ego-position
            ego_pos_sample_cl_ = findNearestSample(cur_pose.pose.position, route_global.shortest_path, ego_pos_sample_cl_);
            if(ego_pos_sample_cl_>=target_sample_cl_)
            {
                return;
            }
            remaining_shortest_path_ = {route_global.shortest_path.begin() + ego_pos_sample_cl_, route_global.shortest_path.begin() + target_sample_cl_}; 
            std::vector<double> sp_accumulated_length_vec;
            double sp_length = accumulatedLength(remaining_shortest_path_, sp_accumulated_length_vec);
            // Get the start and end sample of the local shortest path with respect to look-ahead/behind distance
            double velocity = perception_msgs::object_access::getVelLon(ego_data_)/3.6;
            double look_ahead_distance = std::max(look_ahead_distance_min_, look_ahead_time_*velocity);
            // Find the look-ahead sample
            unsigned int look_ahead_sample;
            for(size_t i=0; i<sp_accumulated_length_vec.size(); i++)
            {
                look_ahead_sample=ego_pos_sample_cl_+i;
                if(sp_accumulated_length_vec[i]>=look_ahead_distance)
                {
                    break;
                }
            }
            
            // Find the look-behind sample
            unsigned int look_behind_sample;
            double accum_length=0.0;
            for(size_t i=ego_pos_sample_cl_; i>0; i--)
            {
                accum_length+=distance(route_global.shortest_path[i],route_global.shortest_path[i-1]);
                look_behind_sample=i;
                if(accum_length>=look_behind_distance_)
                {
                    break;
                }
            }

            // Now we can extract the local-section of the route
            route_planning_msgs::msg::Route temp_route;
            temp_route.header.stamp = stamp_time;
            temp_route.target_position = route_global.target_position;
            temp_route.header.frame_id = ll2if_->map_frame_id_; // currently it's map --> will be changed through transform function
            temp_route.shortest_path = {route_global.shortest_path.begin() + look_behind_sample, route_global.shortest_path.begin() + look_ahead_sample};
            
            // Find nearest Boundary-Sample for left and right boundary at look-ahead and look-behind point
            lbehind_sample_rbound_left_ = findNearestSample(temp_route.shortest_path.front(), route_global.boundaries.left, lbehind_sample_rbound_left_);
            lbehind_sample_rbound_right_ = findNearestSample(temp_route.shortest_path.front(), route_global.boundaries.right, lbehind_sample_rbound_right_);
            lahead_sample_rbound_left_ = findNearestSample(temp_route.shortest_path.back(), route_global.boundaries.left, lahead_sample_rbound_left_);
            lahead_sample_rbound_right_ = findNearestSample(temp_route.shortest_path.back(), route_global.boundaries.right, lahead_sample_rbound_right_);
            temp_route.boundaries.left = {route_global.boundaries.left.begin() + lbehind_sample_rbound_left_, route_global.boundaries.left.begin() + lahead_sample_rbound_left_};
            temp_route.boundaries.right = {route_global.boundaries.right.begin() + lbehind_sample_rbound_right_, route_global.boundaries.right.begin() + lahead_sample_rbound_right_};

            // Now extract the local driveable space
            // Find nearest Boundary-Sample for left and right boundary at look-ahead and look-behind point
            lbehind_sample_drivspace_left_ = findNearestSample(temp_route.shortest_path.front(), driveable_space_global.boundaries.left, lbehind_sample_drivspace_left_);
            lbehind_sample_drivspace_right_ = findNearestSample(temp_route.shortest_path.front(), driveable_space_global.boundaries.right, lbehind_sample_drivspace_right_);
            lahead_sample_drivspace_left_ = findNearestSample(temp_route.shortest_path.back(), driveable_space_global.boundaries.left, lahead_sample_drivspace_left_);
            lahead_sample_drivspace_right_ = findNearestSample(temp_route.shortest_path.back(), driveable_space_global.boundaries.right, lahead_sample_drivspace_right_);
            
            // To-Do: Extract restricting areas
            // ...

            // Now we've got our local-section of the driveable space
            route_planning_msgs::msg::DriveableSpace temp_ds;
            temp_ds.header.stamp = stamp_time;
            temp_ds.header.frame_id = ll2if_->map_frame_id_; // currently it's map --> will be changed through transform function
            
            // Check if the found samples are valid
            if (lbehind_sample_drivspace_left_ < driveable_space_global.boundaries.left.size() && lahead_sample_drivspace_left_ <= driveable_space_global.boundaries.left.size() && lahead_sample_drivspace_left_ > lbehind_sample_drivspace_left_) {
                temp_ds.boundaries.left = {driveable_space_global.boundaries.left.begin() + lbehind_sample_drivspace_left_, driveable_space_global.boundaries.left.begin() + lahead_sample_drivspace_left_};
            } else {
                temp_ds.boundaries.left = driveable_space_global.boundaries.left;
            }

            if (lbehind_sample_drivspace_right_ < driveable_space_global.boundaries.right.size() && lahead_sample_drivspace_right_ <= driveable_space_global.boundaries.right.size() && lahead_sample_drivspace_right_ > lbehind_sample_drivspace_right_) {
                temp_ds.boundaries.right = {driveable_space_global.boundaries.right.begin() + lbehind_sample_drivspace_right_, driveable_space_global.boundaries.right.begin() + lahead_sample_drivspace_right_};
            } else {
                temp_ds.boundaries.right = driveable_space_global.boundaries.right;
            }
            // To-Do: Rest of Route-Object
            // ...
            // First we need to identify the area of interest to extract all Regulatory Elements and Lanes within this area
            // The area of interest (AoI) is derived as an rectangle that envelops the entire local driveable space
            double min_x=INFINITY, min_y=INFINITY, max_x=-INFINITY, max_y=-INFINITY; // Parameters describing the rectangle of the area of interest
            
            // Iterate over left boundary of driveable space
            for(size_t i=0; i<temp_ds.boundaries.left.size(); i++)
            {
                if(temp_ds.boundaries.left[i].x>max_x) max_x = temp_ds.boundaries.left[i].x;
                if(temp_ds.boundaries.left[i].y>max_y) max_y = temp_ds.boundaries.left[i].y;
                if(temp_ds.boundaries.left[i].x<min_x) min_x = temp_ds.boundaries.left[i].x;
                if(temp_ds.boundaries.left[i].y<min_y) min_y = temp_ds.boundaries.left[i].y;
            }
            // Iterate over right boundary of driveable space
            for(size_t i=0; i<temp_ds.boundaries.left.size(); i++)
            {
                if(temp_ds.boundaries.left[i].x>max_x) max_x = temp_ds.boundaries.left[i].x;
                if(temp_ds.boundaries.left[i].y>max_y) max_y = temp_ds.boundaries.left[i].y;
                if(temp_ds.boundaries.left[i].x<min_x) min_x = temp_ds.boundaries.left[i].x;
                if(temp_ds.boundaries.left[i].y<min_y) min_y = temp_ds.boundaries.left[i].y;
            }
            
            // Create a bounding-box (via an area) that envelops the entire local driveable space
            lanelet::LineString3d top(lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), max_x, min_y, 0}, lanelet::Point3d{utils::getId(), max_x, max_y, 0}});
            lanelet::LineString3d right(lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), max_x, max_y, 0}, lanelet::Point3d{utils::getId(), min_x, max_y, 0}});
            lanelet::LineString3d bottom(lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), min_x, max_y, 0}, lanelet::Point3d{utils::getId(), min_x, min_y, 0}});
            lanelet::LineString3d left(lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), min_x, min_y, 0}, lanelet::Point3d{utils::getId(), max_y, min_y, 0}});
            lanelet::Area aoi_area(lanelet::utils::getId(), {top, right, bottom, left});
            lanelet::BoundingBox2d aoi_box = lanelet::geometry::boundingBox2d(aoi_area);

                        // Find all Lanelets within AoI
            std::vector<lanelet::ConstLanelet> aoi_lanelets = llmap_->laneletLayer.search(aoi_box);
            for(size_t i = 0; i<aoi_lanelets.size(); i++)
            {
                // Generate a Lane-Object from each Lanelet
                route_planning_msgs::msg::Lane lane;
                Lanelet::ConstLanelet cur_ll = aoi_lanelets[i];
                lane.centerline = Lanelet2Utilities::convertLaneletLine2Linestring(cur_ll.centerline().basicLineString());
                lane.left = deriveLaneSeparator(cur_ll.leftBound());
                lane.right = deriveLaneSeparator(cur_ll.rightBound());
                temp_route.lanes.push_back(lane);
            }

                        // Find all Regulatory Elements within AoI
            std::vector<std::shared_ptr<const lanelet::RegulatoryElement>> aoi_regelems = llmap_->regulatoryElementLayer.search(aoi_box);
                        for(size_t i = 0; i<aoi_regelems.size(); i++)
            {
                // Generate Regulatory Elements
                route_planning_msgs::msg::RegulatoryElement regelem;
                // Set type and state to unknown initially
                regelem.type = route_planning_msgs::msg::RegulatoryElement::TYPE_UNKNOWN;
                regelem.value = route_planning_msgs::msg::RegulatoryElement::STATE_UNKNOWN;
                // Get the ref-line Linestring
                std::vector<lanelet::ConstLineString3d> ref_lines = aoi_regelems[i]->getParameters<lanelet::ConstLineString3d>(RoleName::RefLine);
                if(ref_lines.size())
                {
                    std::vector<geometry_msgs::msg::Point> ref_points = Lanelet2Utilities::convertLaneletLine2Linestring(ref_lines[0].basicLineString());
                    if(ref_points.size()>1)
                    {
                        regelem.effect_line[0] = ref_points.front();
                        regelem.effect_line[1] = ref_points.back();
                    }
                }
                                // Get all refering elements
                std::vector<lanelet::ConstLineString3d> refering_elems = aoi_regelems[i]->getParameters<lanelet::ConstLineString3d>(RoleName::Refers);
                for(size_t j=0; j<refering_elems.size(); j++)
                {
                    std::vector<geometry_msgs::msg::Point> ref_points = Lanelet2Utilities::convertLaneletLine2Linestring(refering_elems[j].basicLineString());
                    regelem.signal_positions.push_back(ref_points[0]);
                }
                                // Set the Type
                if (aoi_regelems[i]->hasAttribute("subtype"))
                {
                    std::string subtype = aoi_regelems[i]->attribute("subtype").value();
                    if(subtype == "traffic_light") regelem.type = route_planning_msgs::msg::RegulatoryElement::TYPE_TRAFFIC_LIGHT;
                    if(subtype == "speed_limit")
                    {
                        regelem.type = route_planning_msgs::msg::RegulatoryElement::TYPE_SPEED_LIMIT;
                                                regelem.value = deriveValueForSpeedLimitType(aoi_regelems[i], refering_elems);
                    }
                                        if(subtype == "right_of_way") regelem.type = route_planning_msgs::msg::RegulatoryElement::TYPE_YIELD;
                    if(subtype == "all_way_stop") regelem.type = route_planning_msgs::msg::RegulatoryElement::TYPE_STOP;
                    if(subtype == "traffic_sign")
                    {
                        if(refering_elems.size() && refering_elems[0].hasAttribute("type") && refering_elems[0].attribute("type").value()=="traffic_sign" && refering_elems[0].hasAttribute("subtype"))
                        {
                            std::string tsign_code = refering_elems[0].attribute("subtype").value();
                            regelem.value = trafficSignCode2Type(tsign_code);
                        }
                    }
                }
                                // Add to route
                temp_route.regulatory_elements.push_back(regelem);
            }

            // Get the current speed limit
            temp_route.current_speed_limit = std::round(lanelet::units::KmHQuantity(trafficRules_->speedLimit(current_ego_ll).speedLimit).value());
            
            // Now transform the route- and driveable-space-object
            geometry_msgs::msg::TransformStamped tf;
            try {
                tf = tf_buffer_->lookupTransform(local_vehicle_frame_id_, ll2if_->map_frame_id_, tf2::TimePointZero);
                tf2::doTransform(temp_route, route_local, tf);
                tf2::doTransform(temp_ds, driveable_space_local, tf);
                local_route_pub_->publish(route_local);
                local_driveable_space_pub_->publish(driveable_space_local);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Could not transform " << ll2if_->map_frame_id_ << " to " << local_vehicle_frame_id_ << ": " << ex.what());
                return;
            }
        } 

        double GlobalPlanner::accumulatedLength(const std::vector<geometry_msgs::msg::Point>& point_list, std::vector<double>& accumulated_length)
        {
            double length=0.0;
            accumulated_length.push_back(length);
            for(size_t i=0; i<point_list.size()-1; i++)
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
                }
                // Comment else-if to stop searach for now, since finding the nearest sample seems to be buggy 
                // else if (dist > min_distance) {
                //     break; // Stop searching if the distance starts increasing again
                // }
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