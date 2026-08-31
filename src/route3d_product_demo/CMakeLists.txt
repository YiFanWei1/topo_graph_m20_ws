cmake_minimum_required(VERSION 3.8)
project(route3d_product_demo)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/route3d_product_demo
    scripts/route3d_live_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
