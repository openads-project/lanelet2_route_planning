#include "lanelet2_route_planning/global_planner_node.hpp"

        bool GlobalPlanner::extractLocalMapInfo(const perception_msgs::msg::EgoData& ego_data,
                                const route_planning_msgs::msg::Route& route_global,
                                route_planning_msgs::msg::Route& route_local) {
            rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
            rclcpp::Time map_extraction_t0 = wall_clock.now();

            rclcpp::Time stamp_time = now();

            // Find sample of shortest path centerline corresponding to the current ego-position (limited by end of route)
            unsigned int ego_pos_sample_cl = initial_ego_pos_sample_cl_;
            ego_pos_sample_cl = findNearestSample(perception_msgs::object_access::getPosition(ego_data), route_global.remaining_route, ego_pos_sample_cl);
            if (ego_pos_sample_cl >= target_pos_sample_cl_) ego_pos_sample_cl = target_pos_sample_cl_;

            // create temporary local route to fill in this function
            // local route = full route with local boundaries, lanes, regulatory elements, ...
            route_planning_msgs::msg::Route route_local_tmp;
            route_local_tmp.header.stamp = stamp_time;
            route_local_tmp.destination = route_global.destination;
            route_local_tmp.header.frame_id = ll2if_->map_frame_id_; // currently it's map --> will be changed through transform function
            // Check if the found samples are valid
            if(initial_ego_pos_sample_cl_ < route_global.remaining_route.size() && ego_pos_sample_cl <= route_global.remaining_route.size() && ego_pos_sample_cl>=initial_ego_pos_sample_cl_) {
                route_local_tmp.traveled_route = {route_global.remaining_route.begin() + initial_ego_pos_sample_cl_, route_global.remaining_route.begin() + ego_pos_sample_cl};
            }
            if(ego_pos_sample_cl < route_global.remaining_route.size() && target_pos_sample_cl_ <= route_global.remaining_route.size() && target_pos_sample_cl_>=ego_pos_sample_cl) {
                route_local_tmp.remaining_route = {route_global.remaining_route.begin() + ego_pos_sample_cl, route_global.remaining_route.begin() + target_pos_sample_cl_};
            }
            // Remove offsets in travelled distance for traveled- and remaining_route
            for(size_t i=0; i<route_local_tmp.traveled_route.size(); ++i) {
                route_local_tmp.traveled_route[i].z -= route_global.remaining_route[initial_ego_pos_sample_cl_].z;
            }
            for(size_t i=0; i<route_local_tmp.remaining_route.size(); ++i) {
                route_local_tmp.remaining_route[i].z -= route_global.remaining_route[initial_ego_pos_sample_cl_].z;
            }
            // only extract local information if there is a remaining route
            if (route_local_tmp.remaining_route.size() > 1) {
                // Get the start and end sample of the local shortest path with respect to look-ahead/behind distance
                double velocity = perception_msgs::object_access::getVelLon(ego_data_)/3.6;
                double look_ahead_distance = std::max(look_ahead_distance_min_, look_ahead_time_*velocity);
                // Find the look-ahead sample
                unsigned int look_ahead_sample = route_global.remaining_route.size()-1;
                for(size_t i=ego_pos_sample_cl; i<route_global.remaining_route.size(); ++i)
                {
                    if(route_global.remaining_route[i].z-route_global.remaining_route[ego_pos_sample_cl].z>=look_ahead_distance)
                    {
                        look_ahead_sample=i;
                        break;
                    }
                }

                // Find the look-behind sample
                unsigned int look_behind_sample = 0;
                for(size_t i=ego_pos_sample_cl; i>=0; --i)
                {
                    if(route_global.remaining_route[ego_pos_sample_cl].z-route_global.remaining_route[i].z>=look_behind_distance_ || i==0)
                    {
                        look_behind_sample=i;
                        break;
                    }
                }

                // Now we can extract the local-section of the route
                std::vector<geometry_msgs::msg::Point> extraction_path;
                // Check if the found samples are valid
                if(look_behind_sample < route_global.remaining_route.size() && look_ahead_sample <= route_global.remaining_route.size() && look_ahead_sample > look_behind_sample) {
                    extraction_path = {route_global.remaining_route.begin() + look_behind_sample, route_global.remaining_route.begin() + look_ahead_sample};
                }
                // another safety check
                if (!extraction_path.empty()) {
                    unsigned int lbehind_sample_rbound_left = 0;
                    unsigned int lbehind_sample_rbound_right = 0;
                    unsigned int lahead_sample_rbound_left = 0;
                    unsigned int lahead_sample_rbound_right = 0;

                    // Find nearest Boundary-Sample for left and right boundary at look-ahead and look-behind point
                    lbehind_sample_rbound_left = findNearestSample(extraction_path.front(), route_global.boundaries.left, lbehind_sample_rbound_left);
                    lbehind_sample_rbound_right = findNearestSample(extraction_path.front(), route_global.boundaries.right, lbehind_sample_rbound_right);
                    lahead_sample_rbound_left = findNearestSample(extraction_path.back(), route_global.boundaries.left, lahead_sample_rbound_left);
                    lahead_sample_rbound_right = findNearestSample(extraction_path.back(), route_global.boundaries.right, lahead_sample_rbound_right);
                    
                    if(lbehind_sample_rbound_left == route_global.boundaries.left.size()-1) lbehind_sample_rbound_left = 0;
                    if(lbehind_sample_rbound_right == route_global.boundaries.right.size()-1) lbehind_sample_rbound_right = 0;
                    if(lahead_sample_rbound_left == 0) lahead_sample_rbound_left = route_global.boundaries.left.size()-1;
                    if(lahead_sample_rbound_right == 0) lahead_sample_rbound_right = route_global.boundaries.right.size()-1;
                    // Check if the found samples are valid
                    if (lbehind_sample_rbound_left < route_global.boundaries.left.size() && lahead_sample_rbound_left <= route_global.boundaries.left.size() && lahead_sample_rbound_left > lbehind_sample_rbound_left) {
                        route_local_tmp.boundaries.left = {route_global.boundaries.left.begin() + lbehind_sample_rbound_left, route_global.boundaries.left.begin() + lahead_sample_rbound_left};
                    } else {
                        RCLCPP_WARN_STREAM(get_logger(), "Unable to extract local route boundary. Using global route boundary instead!");
                        route_local_tmp.boundaries.left = route_global.boundaries.left;
                    }
                    if (lbehind_sample_rbound_right < route_global.boundaries.right.size() && lahead_sample_rbound_right <= route_global.boundaries.right.size() && lahead_sample_rbound_right > lbehind_sample_rbound_right) {
                        route_local_tmp.boundaries.right = {route_global.boundaries.right.begin() + lbehind_sample_rbound_right, route_global.boundaries.right.begin() + lahead_sample_rbound_right};
                    } else {
                        RCLCPP_WARN_STREAM(get_logger(), "Unable to extract local route boundary. Using global route boundary instead!");
                        route_local_tmp.boundaries.right = route_global.boundaries.right;
                    }
                    
                    unsigned int lbehind_sample_drivspace_left = 0;
                    unsigned int lbehind_sample_drivspace_right = 0;
                    unsigned int lahead_sample_drivspace_left = 0;
                    unsigned int lahead_sample_drivspace_right = 0;

                    // Now extract the local driveable space
                    // Find nearest Boundary-Sample for left and right boundary at look-ahead and look-behind point
                    lbehind_sample_drivspace_left = findNearestSample(extraction_path.front(), route_global.driveable_space.boundaries.left, lbehind_sample_drivspace_left);
                    lbehind_sample_drivspace_right = findNearestSample(extraction_path.front(), route_global.driveable_space.boundaries.right, lbehind_sample_drivspace_right);
                    lahead_sample_drivspace_left = findNearestSample(extraction_path.back(), route_global.driveable_space.boundaries.left, lahead_sample_drivspace_left);
                    lahead_sample_drivspace_right = findNearestSample(extraction_path.back(), route_global.driveable_space.boundaries.right, lahead_sample_drivspace_right);

                    if(lbehind_sample_drivspace_left == route_global.driveable_space.boundaries.left.size()-1) lbehind_sample_drivspace_left = 0;
                    if(lbehind_sample_drivspace_right == route_global.driveable_space.boundaries.right.size()-1) lbehind_sample_drivspace_right = 0;
                    if(lahead_sample_drivspace_left == 0) lahead_sample_drivspace_left = route_global.driveable_space.boundaries.left.size()-1;
                    if(lahead_sample_drivspace_right == 0) lahead_sample_drivspace_right = route_global.driveable_space.boundaries.right.size()-1;

                    // Check if the found samples are valid
                    if (lbehind_sample_drivspace_left < route_global.driveable_space.boundaries.left.size() && lahead_sample_drivspace_left <= route_global.driveable_space.boundaries.left.size() && lahead_sample_drivspace_left > lbehind_sample_drivspace_left) {
                        route_local_tmp.driveable_space.boundaries.left = {route_global.driveable_space.boundaries.left.begin() + lbehind_sample_drivspace_left, route_global.driveable_space.boundaries.left.begin() + lahead_sample_drivspace_left};
                    } else {
                        RCLCPP_WARN_STREAM(get_logger(), "Unable to extract local driveable space boundary. Using global driveable space boundary instead!");
                        route_local_tmp.driveable_space.boundaries.left = route_global.driveable_space.boundaries.left;
                    }
                    if (lbehind_sample_drivspace_right < route_global.driveable_space.boundaries.right.size() && lahead_sample_drivspace_right <= route_global.driveable_space.boundaries.right.size() && lahead_sample_drivspace_right > lbehind_sample_drivspace_right) {
                        route_local_tmp.driveable_space.boundaries.right = {route_global.driveable_space.boundaries.right.begin() + lbehind_sample_drivspace_right, route_global.driveable_space.boundaries.right.begin() + lahead_sample_drivspace_right};
                    } else {
                        RCLCPP_WARN_STREAM(get_logger(), "Unable to extract local driveable space boundary. Using global driveable space boundary instead!");
                        route_local_tmp.driveable_space.boundaries.right = route_global.driveable_space.boundaries.right;
                    }

                    // To-Do: Extract restricting areas
                    // ...

                    // Rest of Route-Object
                    // First we need to identify the area of interest to extract all Regulatory Elements and Lanes within this area
                    // The area of interest (AoI) is derived as an rectangle that envelops the entire local driveable space
                    double min_x=INFINITY, min_y=INFINITY, max_x=-INFINITY, max_y=-INFINITY; // Parameters describing the rectangle of the area of interest

                    // Iterate over left boundary of driveable space
                    for(size_t i=0; i<route_local_tmp.driveable_space.boundaries.left.size(); i++)
                    {
                        if(route_local_tmp.driveable_space.boundaries.left[i].x>max_x) max_x = route_local_tmp.driveable_space.boundaries.left[i].x;
                        if(route_local_tmp.driveable_space.boundaries.left[i].y>max_y) max_y = route_local_tmp.driveable_space.boundaries.left[i].y;
                        if(route_local_tmp.driveable_space.boundaries.left[i].x<min_x) min_x = route_local_tmp.driveable_space.boundaries.left[i].x;
                        if(route_local_tmp.driveable_space.boundaries.left[i].y<min_y) min_y = route_local_tmp.driveable_space.boundaries.left[i].y;
                    }
                    // Iterate over right boundary of driveable space
                    for(size_t i=0; i<route_local_tmp.driveable_space.boundaries.right.size(); i++)
                    {
                        if(route_local_tmp.driveable_space.boundaries.right[i].x>max_x) max_x = route_local_tmp.driveable_space.boundaries.right[i].x;
                        if(route_local_tmp.driveable_space.boundaries.right[i].y>max_y) max_y = route_local_tmp.driveable_space.boundaries.right[i].y;
                        if(route_local_tmp.driveable_space.boundaries.right[i].x<min_x) min_x = route_local_tmp.driveable_space.boundaries.right[i].x;
                        if(route_local_tmp.driveable_space.boundaries.right[i].y<min_y) min_y = route_local_tmp.driveable_space.boundaries.right[i].y;
                    }

                    // Create a bounding-box (via an area) that encloses the entire local driveable space
                    lanelet::LineString3d top(lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), max_x, min_y, 0}, lanelet::Point3d{utils::getId(), max_x, max_y, 0}});
                    lanelet::LineString3d right(lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), max_x, max_y, 0}, lanelet::Point3d{utils::getId(), min_x, max_y, 0}});
                    lanelet::LineString3d bottom(lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), min_x, max_y, 0}, lanelet::Point3d{utils::getId(), min_x, min_y, 0}});
                    lanelet::LineString3d left(lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), min_x, min_y, 0}, lanelet::Point3d{utils::getId(), max_x, min_y, 0}});
                    lanelet::Area aoi_area(lanelet::utils::getId(), {top, right, bottom, left});
                    lanelet::BoundingBox2d aoi_box = lanelet::geometry::boundingBox2d(aoi_area);

                    // Find all Lanelets within AoI
                    lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
                    std::vector<lanelet::ConstLanelet> aoi_lanelets = llmap->laneletLayer.search(aoi_box);
                    for(size_t i = 0; i<aoi_lanelets.size(); i++)
                    {
                        // Generate a Lane-Object from each Lanelet
                        route_planning_msgs::msg::Lane lane;
                        Lanelet::ConstLanelet cur_ll = aoi_lanelets[i];
                        lane.centerline = Lanelet2Utilities::convertLaneletLine2Linestring(cur_ll.centerline().basicLineString());
                        lane.left = deriveLaneSeparator(cur_ll.leftBound());
                        lane.right = deriveLaneSeparator(cur_ll.rightBound());
                        route_local_tmp.lanes.push_back(lane);
                    }

                    // Find all Regulatory Elements within AoI
                    std::vector<std::shared_ptr<const lanelet::RegulatoryElement>> aoi_regelems = llmap->regulatoryElementLayer.search(aoi_box);
                    for(size_t i = 0; i<aoi_regelems.size(); i++)
                    {
                        // Generate Regulatory Elements
                        route_planning_msgs::msg::RegulatoryElement regelem;
                        // Set type and state to unknown initially
                        regelem.type = route_planning_msgs::msg::RegulatoryElement::TYPE_UNKNOWN;
                        regelem.value = route_planning_msgs::msg::RegulatoryElement::STATE_UNKNOWN;
                        // Get the ref-line Linestring
                        std::vector<lanelet::ConstLineString3d> ref_lines = aoi_regelems[i]->getParameters<lanelet::ConstLineString3d>(RoleName::RefLine);
                        if(ref_lines.size()) {
                            std::vector<geometry_msgs::msg::Point> ref_points = Lanelet2Utilities::convertLaneletLine2Linestring(ref_lines[0].basicLineString());
                            if(ref_points.size()>1) {
                                regelem.effect_line[0] = ref_points.front();
                                regelem.effect_line[1] = ref_points.back();
                            }
                        }
                        // Get all refering elements
                        std::vector<lanelet::ConstLineString3d> refering_elems = aoi_regelems[i]->getParameters<lanelet::ConstLineString3d>(RoleName::Refers);
                        for(size_t j=0; j<refering_elems.size(); ++j) {
                            std::vector<geometry_msgs::msg::Point> ref_points = Lanelet2Utilities::convertLaneletLine2Linestring(refering_elems[j].basicLineString());
                            regelem.signal_positions.push_back(ref_points[0]);
                        }
                        // Set the Type
                        if (aoi_regelems[i]->hasAttribute("subtype")) {
                            std::string subtype = aoi_regelems[i]->attribute("subtype").value();
                            if(subtype == "traffic_light") regelem.type = route_planning_msgs::msg::RegulatoryElement::TYPE_TRAFFIC_LIGHT;
                            if(subtype == "speed_limit") {
                                regelem.type = route_planning_msgs::msg::RegulatoryElement::TYPE_SPEED_LIMIT;
                                regelem.value = deriveValueForSpeedLimitType(aoi_regelems[i], refering_elems);
                            }
                            if(subtype == "right_of_way") regelem.type = route_planning_msgs::msg::RegulatoryElement::TYPE_YIELD;
                            if(subtype == "all_way_stop") regelem.type = route_planning_msgs::msg::RegulatoryElement::TYPE_STOP;
                            if(subtype == "traffic_sign") {
                                if(refering_elems.size() && refering_elems[0].hasAttribute("type") && refering_elems[0].attribute("type").value()=="traffic_sign" && refering_elems[0].hasAttribute("subtype")) {
                                    std::string tsign_code = refering_elems[0].attribute("subtype").value();
                                    regelem.value = trafficSignCode2Type(tsign_code);
                                }
                            }
                        }
                        // Add to route
                        route_local_tmp.regulatory_elements.push_back(regelem);
                    }
                } else {
                    RCLCPP_ERROR_STREAM(get_logger(), "Unable to extract path segment for extracting map information!");
                    return false;
                }
            }
            else {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Remaining route is empty. Unable to extract local map information!");
                return false;
            }

            // Get the current speed limit
            lanelet::ConstLanelet current_ego_ll;
            if(!deriveEgoLanelet(ego_data, current_ego_ll)) return false;
            route_local_tmp.current_speed_limit = std::round(lanelet::units::KmHQuantity(trafficRules_->speedLimit(current_ego_ll).speedLimit).value());

            // Now transform the route-object
            geometry_msgs::msg::TransformStamped tf;
            try {
                tf = tf_buffer_->lookupTransform(local_vehicle_frame_id_, ll2if_->map_frame_id_, tf2::TimePointZero);
                tf2::doTransform(route_local_tmp, route_local, tf);
                route_pub_->publish(route_local);
                rclcpp::Duration map_extraction_duration = wall_clock.now() - map_extraction_t0;
                RCLCPP_INFO(this->get_logger(), "Extraction of map information took %.3f ms", map_extraction_duration.nanoseconds() / 1e6);
                return true;
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Could not transform " << ll2if_->map_frame_id_ << " to " << local_vehicle_frame_id_ << ": " << ex.what());
                return false;
            }
        }

        double GlobalPlanner::distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2, const bool ignore_z)
        {
            if (ignore_z) {
                return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
            } else {
                return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
            }
        }

        unsigned int GlobalPlanner::findNearestSample(const geometry_msgs::msg::Point& ref_point, const std::vector<geometry_msgs::msg::Point>& point_list, const unsigned int& start_index)
        {
            double min_distance = std::numeric_limits<double>::max();
            unsigned int nearest_index = start_index;
            for (size_t i = start_index; i<point_list.size(); ++i) {
                double dist = distance(ref_point, point_list[i], true);
                if (dist < min_distance) {
                    min_distance = dist;
                    nearest_index = i; // Update the last index to the current index
                }
            }
            return nearest_index;
        }

        unsigned int GlobalPlanner::findNearestSampleReverse(const geometry_msgs::msg::Point& ref_point, const std::vector<geometry_msgs::msg::Point>& point_list)
        {
            double min_distance = std::numeric_limits<double>::max();
            unsigned int nearest_index = point_list.size()-1;
            for (size_t i = point_list.size()-1; i>=0; --i) {
                double dist = distance(ref_point, point_list[i], true);
                if (dist < min_distance) {
                    min_distance = dist;
                    nearest_index = i; // Update the last index to the current index
                }
                if(i==0) break;
            }
            return nearest_index;
        }