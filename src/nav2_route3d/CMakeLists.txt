cmake_minimum_required(VERSION 3.22)
project(nav2_route3d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

set(route3d_core_sources
  src/edge_scorer_3d.cpp
  src/file_loader_3d.cpp
  src/goal_intent_extractor_3d.cpp
  src/graph_3d.cpp
  src/graph_builder_3d.cpp
  src/gridmap_2d.cpp
  src/kdtree_3d.cpp
  src/path_converter_3d.cpp
  src/pcd_io.cpp
  src/route_operations_3d.cpp
  src/route_planner_3d.cpp
  src/route_tracker_3d.cpp
)

add_library(nav2_route3d_core SHARED ${route3d_core_sources})
ament_target_dependencies(nav2_route3d_core
  builtin_interfaces
  geometry_msgs
  nav2_msgs
  nav2_util
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)
target_link_libraries(nav2_route3d_core nlohmann_json::nlohmann_json)
target_include_directories(nav2_route3d_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_executable(route_server_3d src/route_server_3d.cpp)
ament_target_dependencies(route_server_3d
  builtin_interfaces
  geometry_msgs
  nav2_msgs
  nav2_util
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)
target_link_libraries(route_server_3d nav2_route3d_core nlohmann_json::nlohmann_json)

add_executable(route_graph_builder_3d src/route_graph_builder_3d_main.cpp)
ament_target_dependencies(route_graph_builder_3d rclcpp)
target_link_libraries(route_graph_builder_3d nav2_route3d_core nlohmann_json::nlohmann_json)

install(TARGETS
  nav2_route3d_core
  route_server_3d
  route_graph_builder_3d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch rviz examples docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_route_planner_3d test/test_route_planner_3d.cpp)
  target_link_libraries(test_route_planner_3d nav2_route3d_core nlohmann_json::nlohmann_json)
  ament_target_dependencies(test_route_planner_3d geometry_msgs nav2_msgs nav_msgs)

  ament_add_gtest(test_graph_builder_pruning test/test_graph_builder_pruning.cpp)
  target_link_libraries(test_graph_builder_pruning nav2_route3d_core nlohmann_json::nlohmann_json)
  ament_target_dependencies(test_graph_builder_pruning geometry_msgs nav2_msgs nav_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_route3d_core)
ament_export_dependencies(
  geometry_msgs
  nav2_msgs
  nav2_util
  nav_msgs
  nlohmann_json
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_ros
)
ament_package()
