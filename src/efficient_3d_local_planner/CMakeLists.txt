cmake_minimum_required(VERSION 3.16)
project(efficient_3d_local_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(efficient_3d_local_planner_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(node_dependencies
  diagnostic_msgs efficient_3d_local_planner_msgs geometry_msgs message_filters
  nav_msgs rcl_interfaces rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
  visualization_msgs)

add_executable(local_voxel_mapper_node src/local_voxel_mapper_node.cpp)
target_include_directories(local_voxel_mapper_node PRIVATE include)
target_link_libraries(local_voxel_mapper_node Eigen3::Eigen)
ament_target_dependencies(local_voxel_mapper_node ${node_dependencies})

add_executable(corridor_astar_planner_node src/corridor_astar_planner_node.cpp)
target_include_directories(corridor_astar_planner_node PRIVATE include)
target_link_libraries(corridor_astar_planner_node Eigen3::Eigen)
ament_target_dependencies(corridor_astar_planner_node ${node_dependencies})

add_executable(local_path_follower_node src/local_path_follower_node.cpp)
target_include_directories(local_path_follower_node PRIVATE include)
target_link_libraries(local_path_follower_node Eigen3::Eigen)
ament_target_dependencies(local_path_follower_node ${node_dependencies})

install(TARGETS local_voxel_mapper_node corridor_astar_planner_node local_path_follower_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/bag_metrics.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rolling_voxel_map test/test_rolling_voxel_map.cpp)
  target_include_directories(test_rolling_voxel_map PRIVATE include)
  target_link_libraries(test_rolling_voxel_map Eigen3::Eigen)

  ament_add_gtest(test_guided_astar test/test_guided_astar.cpp)
  target_include_directories(test_guided_astar PRIVATE include)
  target_link_libraries(test_guided_astar Eigen3::Eigen)

  ament_add_gtest(test_path_follower test/test_path_follower.cpp)
  target_include_directories(test_path_follower PRIVATE include)
  target_link_libraries(test_path_follower Eigen3::Eigen)

  ament_add_gtest(test_replanning_policy test/test_replanning_policy.cpp)
  target_include_directories(test_replanning_policy PRIVATE include)

  ament_add_gtest(test_bspline_path_optimizer test/test_bspline_path_optimizer.cpp)
  target_include_directories(test_bspline_path_optimizer PRIVATE include)
  target_link_libraries(test_bspline_path_optimizer Eigen3::Eigen)
endif()

ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
