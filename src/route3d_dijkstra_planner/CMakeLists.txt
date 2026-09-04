cmake_minimum_required(VERSION 3.16)
project(route3d_dijkstra_planner)

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
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(route3d_dijkstra_core src/topology_graph.cpp)
set_target_properties(route3d_dijkstra_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(route3d_dijkstra_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(route3d_dijkstra_core PUBLIC nlohmann_json::nlohmann_json)

add_executable(dijkstra_planner_node src/dijkstra_planner_node.cpp)
target_link_libraries(dijkstra_planner_node
  route3d_dijkstra_core nlohmann_json::nlohmann_json)
ament_target_dependencies(dijkstra_planner_node
  ament_index_cpp geometry_msgs nav_msgs rclcpp std_msgs visualization_msgs)

add_executable(dijkstra_demo_requests src/dijkstra_demo_requests.cpp)
ament_target_dependencies(dijkstra_demo_requests rclcpp std_msgs)

add_executable(dijkstra_stress_benchmark src/dijkstra_stress_benchmark.cpp)
target_link_libraries(dijkstra_stress_benchmark PRIVATE
  route3d_dijkstra_core nlohmann_json::nlohmann_json)

install(TARGETS
  route3d_dijkstra_core
  dijkstra_planner_node
  dijkstra_demo_requests
  dijkstra_stress_benchmark
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dijkstra test/test_dijkstra.cpp)
  target_link_libraries(test_dijkstra route3d_dijkstra_core)
  target_compile_definitions(test_dijkstra PRIVATE
    TEST_GRAPH_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/complex_topology_v2.json")
endif()

ament_export_include_directories(include)
ament_export_libraries(route3d_dijkstra_core)
ament_export_dependencies(nlohmann_json)
ament_package()
