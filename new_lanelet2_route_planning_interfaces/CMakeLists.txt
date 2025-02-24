cmake_minimum_required(VERSION 3.8)
project(new_lanelet2_route_planning_interfaces)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  action/GlobalManeuver.action
  msg/Boundaries.msg
  msg/DriveableSpace.msg
  msg/Lane.msg
  msg/LaneSeparator.msg
  msg/RegulatoryElement.msg
  msg/Route.msg
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
