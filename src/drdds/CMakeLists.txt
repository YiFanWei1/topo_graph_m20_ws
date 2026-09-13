cmake_minimum_required(VERSION 3.8)
project(drdds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Timestamp.msg"
  "msg/MetaType.msg"
  "msg/MotionStateValue.msg"
  "msg/GaitValue.msg"
  "msg/NavCmdValue.msg"
  "msg/MotionInfoValue.msg"
  "msg/MotionState.msg"
  "msg/Gait.msg"
  "msg/NavCmd.msg"
  "msg/MotionInfo.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
