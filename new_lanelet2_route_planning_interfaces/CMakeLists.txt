cmake_minimum_required(VERSION 3.8)
project(new_lanelet2_route_planning_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  action/GlobalManeuver.action
  msg/Boundaries.msg
  msg/DriveableSpace.msg
  msg/Lane.msg
  msg/LaneSeparator.msg
  msg/RegulatoryElement.msg
  msg/Route.msg
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
