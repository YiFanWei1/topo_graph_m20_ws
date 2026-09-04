cmake_minimum_required(VERSION 3.16)
project(route3d_route_slicer)

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
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(route3d_dijkstra_planner REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RouteWaypoint.msg"
  "msg/RouteTask.msg"
  "msg/RouteTaskArray.msg"
  DEPENDENCIES geometry_msgs std_msgs)

add_library(route3d_route_slicer_core STATIC src/route_slicer.cpp)
target_include_directories(route3d_route_slicer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(route3d_route_slicer_core route3d_dijkstra_planner)

add_executable(route_slicer_node src/route_slicer_node.cpp)
target_link_libraries(route_slicer_node
  route3d_route_slicer_core nlohmann_json::nlohmann_json)
ament_target_dependencies(route_slicer_node
  geometry_msgs rclcpp route3d_dijkstra_planner std_msgs visualization_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(route_slicer_node "${cpp_typesupport_target}")

install(TARGETS route3d_route_slicer_core route_slicer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_route_slicer test/test_route_slicer.cpp)
  target_link_libraries(test_route_slicer route3d_route_slicer_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(route3d_route_slicer_core)
ament_export_dependencies(route3d_dijkstra_planner)
ament_package()
