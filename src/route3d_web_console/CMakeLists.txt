cmake_minimum_required(VERSION 3.16)
project(route3d_web_console)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)

add_library(route3d_web_console_core
  src/process_manager.cpp
  src/topology_store.cpp
  src/point_cloud_codec.cpp)
target_include_directories(route3d_web_console_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(route3d_web_console_core
  Boost::system nlohmann_json::nlohmann_json ${PCL_LIBRARIES})
ament_target_dependencies(route3d_web_console_core sensor_msgs pcl_conversions)

add_executable(route3d_web_console_node src/web_console_node.cpp)
target_include_directories(route3d_web_console_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(route3d_web_console_node
  route3d_web_console_core Boost::system nlohmann_json::nlohmann_json ${PCL_LIBRARIES})
ament_target_dependencies(route3d_web_console_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs pcl_conversions)

install(TARGETS route3d_web_console_core route3d_web_console_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config web docs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_topology_store test/test_topology_store.cpp)
  if(TARGET test_topology_store)
    target_link_libraries(test_topology_store route3d_web_console_core)
  endif()
  ament_add_gtest(test_point_cloud_codec test/test_point_cloud_codec.cpp)
  if(TARGET test_point_cloud_codec)
    target_link_libraries(test_point_cloud_codec route3d_web_console_core)
  endif()
  ament_add_gtest(test_process_manager test/test_process_manager.cpp)
  if(TARGET test_process_manager)
    target_link_libraries(test_process_manager route3d_web_console_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(route3d_web_console_core)
ament_package()
