cmake_minimum_required(VERSION 3.16)
project(path_tracking_benchmark)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(fixed_path_benchmark_node src/fixed_path_benchmark_node.cpp)
target_include_directories(fixed_path_benchmark_node PRIVATE include)
ament_target_dependencies(fixed_path_benchmark_node
  diagnostic_msgs geometry_msgs nav_msgs rclcpp std_srvs tf2 tf2_geometry_msgs visualization_msgs)

install(TARGETS fixed_path_benchmark_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/plot_tracking_report.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_benchmark_math test/test_benchmark_math.cpp)
  target_include_directories(test_benchmark_math PRIVATE include)
endif()

ament_package()
