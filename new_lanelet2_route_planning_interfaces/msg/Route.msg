std_msgs/Header header

geometry_msgs/Point destination                             # route destination (x,y,z)

uint8 current_speed_limit                                   # speed limit at current position [km/h]

geometry_msgs/Point[] traveled_route                        # traveled route points (x,y,s)
geometry_msgs/Point[] remaining_route                       # remaining route points (x,y,s)

new_lanelet2_route_planning_interfaces/DriveableSpace driveable_space          # restricting driveable space (may be local)
new_lanelet2_route_planning_interfaces/Boundaries boundaries                   # restricting route boundaries (may be local)
new_lanelet2_route_planning_interfaces/Lane[] lanes                            # driving lanes (may be local)
new_lanelet2_route_planning_interfaces/RegulatoryElement[] regulatory_elements # regulatory elements (may be local)
