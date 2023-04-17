std_msgs/Header header

geometry_msgs/Point target_position # Position of the route target

Boundaries boundaries # Restricting boundaries of the route
geometry_msgs/Point[] shortest_path # Shortest path centerline along the route
RoadMarking[] road_markings # road-markings along the route
RegulatoryElement[] regulatory_elements # Regulatory elements along the route

uint8 current_speed_limit # speed limit at current ego position (for the local section of the route)
