cmake_minimum_required(VERSION 3.16)
project(topo_graph_tools)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/nav2_to_topo_single
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_topology_conversion test/test_topology_conversion.py)
endif()

ament_package()
