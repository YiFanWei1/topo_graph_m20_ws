cmake_minimum_required(VERSION 3.16)
project(route3d_online_skeleton)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/online_route_skeleton_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
