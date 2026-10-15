cmake_minimum_required(VERSION 3.16)
project(arm_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(arm_msgs REQUIRED)
find_package(arm_kinematics REQUIRED)

add_library(arm_control_core
  src/gesture_map.cpp
  src/joint_controller.cpp
)
target_include_directories(arm_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(arm_control_core PUBLIC arm_kinematics::arm_kinematics)

add_executable(gesture_mapper src/gesture_mapper_node.cpp)
# Plain signature, because ament_target_dependencies below uses the plain
# one and CMake refuses to mix the two on a single target.
target_link_libraries(gesture_mapper arm_control_core)
ament_target_dependencies(gesture_mapper rclcpp sensor_msgs trajectory_msgs arm_msgs)

add_executable(joint_controller src/joint_controller_node.cpp)
target_link_libraries(joint_controller arm_control_core)
ament_target_dependencies(joint_controller rclcpp sensor_msgs trajectory_msgs arm_msgs)

install(TARGETS arm_control_core
  EXPORT arm_controlTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS gesture_mapper joint_controller DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(arm_control_core)
ament_export_targets(arm_controlTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(arm_kinematics)

if(BUILD_TESTING)
  enable_testing()
  foreach(test_name test_gesture_map test_joint_controller)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} arm_control_core)
    target_include_directories(${test_name} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../test_support)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()

ament_package()
