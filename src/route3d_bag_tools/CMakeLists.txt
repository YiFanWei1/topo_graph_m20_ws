cmake_minimum_required(VERSION 3.16)
project(route3d_bag_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(bag_to_route3d_inputs src/bag_to_route3d_inputs.cpp)
ament_target_dependencies(bag_to_route3d_inputs
  nav_msgs
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
)
target_link_libraries(bag_to_route3d_inputs nlohmann_json::nlohmann_json)

install(TARGETS bag_to_route3d_inputs
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/route_corner_target_generator
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_route_corner_target_generator
    test/test_route_corner_target_generator.py
  )
endif()

ament_package()
