cmake_minimum_required(VERSION 3.16)
project(route3d_pid_controller)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(route3d_route_slicer REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GaitTransition.msg"
  "srv/AcknowledgeGaitTransition.srv")

add_library(route3d_pid_controller_core STATIC
  src/controller_core.cpp
  src/elevation_collision_checker.cpp
  src/safety_clear_gate.cpp
  src/timestamped_pose_buffer.cpp)
set_target_properties(route3d_pid_controller_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(route3d_pid_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(pid_controller_node src/pid_controller_node.cpp)
target_link_libraries(pid_controller_node
  route3d_pid_controller_core
  nlohmann_json::nlohmann_json)
ament_target_dependencies(pid_controller_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  route3d_route_slicer
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pid_controller_node "${cpp_typesupport_target}")

install(TARGETS route3d_pid_controller_core pid_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller_core test/test_controller_core.cpp)
  target_link_libraries(test_controller_core route3d_pid_controller_core)
  ament_add_gtest(test_safety_clear_gate test/test_safety_clear_gate.cpp)
  target_link_libraries(test_safety_clear_gate route3d_pid_controller_core)
  ament_add_gtest(test_timestamped_pose_buffer test/test_timestamped_pose_buffer.cpp)
  target_link_libraries(test_timestamped_pose_buffer route3d_pid_controller_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(route3d_pid_controller_core)
ament_package()
