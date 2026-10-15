cmake_minimum_required(VERSION 3.16)
project(arm_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arm_msgs REQUIRED)
# The kinematics library and the firmware core are plain CMake packages built
# by colcon ahead of this one. arm_interface links both: the same dynamics the
# planner uses and the same protocol the board runs.
find_package(arm_kinematics REQUIRED)
find_package(pa_core REQUIRED)

add_library(arm_device
  src/simulated_device.cpp
  src/device_link.cpp
)
target_include_directories(arm_device PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(arm_device PUBLIC arm_kinematics::arm_kinematics pa_core::pa_core)

add_executable(firmware_bridge src/firmware_bridge_node.cpp)
# Plain signature, because ament_target_dependencies below uses the plain
# one and CMake refuses to mix the two on a single target.
target_link_libraries(firmware_bridge arm_device)
ament_target_dependencies(firmware_bridge rclcpp sensor_msgs arm_msgs)

install(TARGETS arm_device
  EXPORT arm_interfaceTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS firmware_bridge DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(arm_device)
ament_export_targets(arm_interfaceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(arm_kinematics pa_core)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_simulated_device test/test_simulated_device.cpp)
  target_link_libraries(test_simulated_device arm_device)
  target_include_directories(test_simulated_device PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../test_support)
  add_test(NAME test_simulated_device COMMAND test_simulated_device)
endif()

ament_package()
