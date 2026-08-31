cmake_minimum_required(VERSION 3.16)
project(go2_gait_switch_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(go2_gait_switch_demo_node src/go2_gait_switch_demo_node.cpp)
ament_target_dependencies(go2_gait_switch_demo_node rclcpp std_srvs)
target_link_libraries(go2_gait_switch_demo_node unitree_sdk2)

# unitree_sdk2 links its bundled Cyclone DDS libraries dynamically. Preserve the
# SDK installation's library directory so the installed ROS executable can find
# libddsc.so.0/libddscxx.so.0 without requiring a global LD_LIBRARY_PATH.
get_target_property(UNITREE_SDK2_LIBRARY unitree_sdk2 IMPORTED_LOCATION)
get_filename_component(UNITREE_SDK2_LIBRARY_DIR "${UNITREE_SDK2_LIBRARY}" DIRECTORY)
set_target_properties(go2_gait_switch_demo_node PROPERTIES
  BUILD_RPATH "${UNITREE_SDK2_LIBRARY_DIR}"
  INSTALL_RPATH "${UNITREE_SDK2_LIBRARY_DIR}")

# ROS 2 Jazzy adds its architecture-specific library directory to
# LD_LIBRARY_PATH.  That directory also contains libddsc.so.0, but Unitree's
# prebuilt libddscxx.so.0 must be paired with the libddsc.so.0 shipped by the
# same SDK.  Emit DT_RPATH instead of DT_RUNPATH so LD_LIBRARY_PATH cannot
# silently replace only the C half of that pair at runtime.
target_link_options(go2_gait_switch_demo_node PRIVATE
  "LINKER:--disable-new-dtags")

install(TARGETS go2_gait_switch_demo_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
