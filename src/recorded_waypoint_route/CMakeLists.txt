cmake_minimum_required(VERSION 3.16)
project(recorded_waypoint_route)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 录点器和路径生成器仍是低频 Python 工具；高频到点判断由下面的 C++ 节点承担。
ament_python_install_package(${PROJECT_NAME})

add_executable(target_route_sequencer_node src/target_route_sequencer_node.cpp)
target_include_directories(target_route_sequencer_node PRIVATE include)
ament_target_dependencies(target_route_sequencer_node
  diagnostic_msgs geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS target_route_sequencer_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/target_path_generator_node
  scripts/target_recorder_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_route_sequencer_core test/test_route_sequencer_core.cpp)
  target_include_directories(test_route_sequencer_core PRIVATE include)

  ament_add_pytest_test(test_route_utils test/test_route_utils.py)
  ament_add_pytest_test(test_route_progress test/test_route_progress.py)
endif()

ament_export_include_directories(include)
ament_package()
