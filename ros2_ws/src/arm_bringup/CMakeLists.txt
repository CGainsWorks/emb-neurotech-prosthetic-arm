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
find_package(ament_index_cpp REQUIRED)

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

# The sEMG replay and the classifier interface stub: the two ends of the
# classifier, for driving the graph from sEMG rather than from a script.
add_executable(semg_replay src/semg_replay_node.cpp)
ament_target_dependencies(semg_replay rclcpp arm_msgs ament_index_cpp)

add_executable(classifier_stub src/classifier_stub_node.cpp)
target_link_libraries(classifier_stub arm_control::arm_control_core arm_interface::arm_device)
ament_target_dependencies(classifier_stub rclcpp arm_msgs)

# The synthetic recording the replay plays, generated rather than committed.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(SEMG_REPLAY_CSV ${CMAKE_CURRENT_BINARY_DIR}/semg_replay.csv)
add_custom_command(
  OUTPUT ${SEMG_REPLAY_CSV}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_semg_replay.py ${SEMG_REPLAY_CSV}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_semg_replay.py
  COMMENT "Generating the synthetic sEMG replay"
)
add_custom_target(semg_replay_data ALL DEPENDS ${SEMG_REPLAY_CSV})

install(TARGETS sim_demo gesture_replay semg_replay classifier_stub
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES ${SEMG_REPLAY_CSV} DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  # The whole graph, launched and checked end to end: sEMG replay, classifier
  # stub, mapper, controller, bridge and simulated joints.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_end_to_end.launch.py TIMEOUT 120)
endif()

ament_package()
