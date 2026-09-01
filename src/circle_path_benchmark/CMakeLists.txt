cmake_minimum_required(VERSION 3.16)
project(circle_path_benchmark)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(generate_circle_path src/generate_circle_path.cpp)
target_include_directories(generate_circle_path PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(generate_circle_path ${PCL_LIBRARIES} yaml-cpp)

add_executable(circle_path_benchmark_node src/circle_path_benchmark_node.cpp)
target_include_directories(circle_path_benchmark_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(circle_path_benchmark_node ${PCL_LIBRARIES} yaml-cpp)
ament_target_dependencies(circle_path_benchmark_node
  diagnostic_msgs geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs
  std_srvs tf2 tf2_geometry_msgs visualization_msgs)

install(TARGETS generate_circle_path circle_path_benchmark_node
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/plot_circle_result
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_circle_path test/test_circle_path.cpp)
  target_include_directories(test_circle_path PRIVATE include)
endif()

ament_package()
