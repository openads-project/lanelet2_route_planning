cmake_minimum_required(VERSION 3.8)
project(lanelet2_route_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_map_interface REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)
find_package(OpenMP REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(perception_msgs_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(route_planning_msgs REQUIRED)
find_package(route_planning_msgs_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_perception_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(TARGET_NAME lanelet2_route_planning)

add_executable(${TARGET_NAME}
  src/conversions.cpp
  src/geometry.cpp
  src/lanelet2_route_planning.cpp
  src/utils.cpp
)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  lanelet2_core
  lanelet2_map_interface
  lanelet2_routing
  lanelet2_traffic_rules
  OpenMP
  perception_msgs
  perception_msgs_utils
  rclcpp
  rclcpp_action
  route_planning_msgs
  route_planning_msgs_utils
  tf2
  tf2_geometry_msgs
  tf2_perception_msgs
  tf2_ros
)

# https://github.com/openads-project/lanelet2_map_server/tree/main/lanelet2_map_interface#linting-downstream-packages
target_dependencies_as_system(${TARGET_NAME})

install(TARGETS ${TARGET_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/format/.clang-format)
  set(ament_cmake_clang_tidy_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/lint/.clang-tidy)
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/lint/ament_flake8.ini)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
