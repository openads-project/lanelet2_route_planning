cmake_minimum_required(VERSION 3.8)
project(lanelet2_route_planning_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GlobalManeuver.action"
  "msg/Boundaries.msg"
  "msg/DriveableSpace.msg"
  "msg/RoadMarking.msg"
  "msg/RegulatoryElement.msg"
  "msg/Route.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs
)

add_library(tf_${PROJECT_NAME} INTERFACE)

target_include_directories(tf_${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(tf_${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime tf2 tf2_geometry_msgs tf2_ros)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS tf_${PROJECT_NAME}
  EXPORT tf_${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
