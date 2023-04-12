std_msgs/Header header

Boundaries boundaries # Restricting boundaries of the route
geometry_msgs/Point[] shortest_path # Shortest path centerline along the route
RoadMarking[] road_markings # road-markings along the route
RegulatoryElement[] regulatory_elemements # Regulatory elements along the route

uint8 current_speed_limit # speed limit at current ego position (for the local section of the route)
