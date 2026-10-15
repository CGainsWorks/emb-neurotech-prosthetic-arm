cmake_minimum_required(VERSION 3.16)
project(arm_bringup)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arm_msgs REQUIRED)
find_package(arm_control REQUIRED)
find_package(arm_interface REQUIRED)
find_package(arm_kinematics REQUIRED)

# The end-to-end run. No middleware: it links the same libraries the nodes use
# and drives them directly, so it is deterministic and finishes in seconds.
# The imported targets, not the bare library names. ament_export_targets
# namespaces what it exports and only the namespaced target carries the
# include directories and the transitive dependencies with it; linking the bare
# name finds the library and then fails to find its headers.
add_executable(sim_demo src/sim_demo.cpp)
target_link_libraries(sim_demo arm_control::arm_control_core arm_interface::arm_device)

# A scripted classifier, for driving the distributed graph without one.
add_executable(gesture_replay src/gesture_replay_node.cpp)
target_link_libraries(gesture_replay arm_control::arm_control_core)
ament_target_dependencies(gesture_replay rclcpp arm_msgs)

install(TARGETS sim_demo gesture_replay DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
