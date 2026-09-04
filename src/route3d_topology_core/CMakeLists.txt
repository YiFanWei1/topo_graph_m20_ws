cmake_minimum_required(VERSION 3.16)
project(route3d_topology_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/build_topology_graph
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_incremental_topology test/test_incremental_topology.py)
  ament_add_pytest_test(test_pcd_validation test/test_pcd_validation.py)
  ament_add_pytest_test(test_topology_schema test/test_topology_schema.py)
endif()

ament_package()
