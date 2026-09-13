cmake_minimum_required(VERSION 3.8)
project(route3d_m20_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(drdds REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(m20_route_adapter_node src/m20_route_adapter_node.cpp)
ament_target_dependencies(
  m20_route_adapter_node
  drdds geometry_msgs nlohmann_json rclcpp std_msgs)

add_executable(odometry_tf_broadcaster_node src/odometry_tf_broadcaster_node.cpp)
ament_target_dependencies(
  odometry_tf_broadcaster_node
  geometry_msgs nav_msgs rclcpp tf2_ros)

install(
  TARGETS m20_route_adapter_node odometry_tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
