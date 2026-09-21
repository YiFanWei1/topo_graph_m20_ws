cmake_minimum_required(VERSION 3.16)
project(route3d_topology_editor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME} SHARED
  include/route3d_topology_editor/topology_editor_panel.hpp
  include/route3d_topology_editor/topology_select_tool.hpp
  src/topology_document.cpp
  src/topology_editor_panel.cpp
  src/topology_select_tool.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  nlohmann_json::nlohmann_json)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs pluginlib rclcpp rviz_common std_msgs visualization_msgs)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_topology_document test/test_topology_document.cpp)
  target_link_libraries(test_topology_document
    ${PROJECT_NAME} nlohmann_json::nlohmann_json)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
