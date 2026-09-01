cmake_minimum_required(VERSION 3.16)
project(obstacle_occlusion_extension)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(efficient_3d_local_planner_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(obstacle_occlusion_extension_node src/obstacle_occlusion_extension_node.cpp)
target_include_directories(obstacle_occlusion_extension_node PRIVATE include)
ament_target_dependencies(obstacle_occlusion_extension_node
  efficient_3d_local_planner_msgs geometry_msgs nav_msgs rcl_interfaces rclcpp sensor_msgs
  tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS obstacle_occlusion_extension_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_occlusion_extension test/test_occlusion_extension.cpp)
  target_include_directories(test_occlusion_extension PRIVATE include)
endif()

ament_export_include_directories(include)
ament_package()
