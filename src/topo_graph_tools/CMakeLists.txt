cmake_minimum_required(VERSION 3.16)
project(topo_graph_tools)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/nav2_to_topo_single
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
