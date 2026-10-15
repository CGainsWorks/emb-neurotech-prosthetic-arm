cmake_minimum_required(VERSION 3.16)
project(arm_kinematics VERSION 1.0.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# The forward and inverse kinematics, dynamics and trajectory library.
#
# Plain CMake, no ROS and no third-party dependency. It builds standalone with
# ./run_tests.sh on any machine with a C++17 compiler and colcon picks it up
# as a plain-cmake package (see package.xml) so the ROS 2 nodes can
# find_package it. One library, two consumers, one copy of the maths.
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(ARM_KINEMATICS_BUILD_TESTS "Build the unit tests" ON)
option(ARM_KINEMATICS_BUILD_TOOLS "Build the command line tools" ON)

if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

add_library(arm_kinematics
  src/model.cpp
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
  src/dynamics.cpp
  src/trajectory.cpp
)
add_library(arm_kinematics::arm_kinematics ALIAS arm_kinematics)

target_include_directories(arm_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(arm_kinematics PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# Tools. Each writes JSON on stdout so a script or a test can consume it;
# nothing in this repository quotes a number that one of these did not print.
# ---------------------------------------------------------------------------
if(ARM_KINEMATICS_BUILD_TOOLS)
  foreach(tool dump_model workspace_report ik_benchmark)
    add_executable(${tool} tools/${tool}.cpp)
    target_link_libraries(${tool} PRIVATE arm_kinematics)
  endforeach()
endif()

if(ARM_KINEMATICS_BUILD_TESTS)
  enable_testing()
  file(GLOB ARM_KINEMATICS_TESTS CONFIGURE_DEPENDS test/test_*.cpp)
  foreach(test_src ${ARM_KINEMATICS_TESTS})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE arm_kinematics)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../test_support)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()

# ---------------------------------------------------------------------------
# Install, so ros2_ws/src/arm_control can find_package(arm_kinematics).
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS arm_kinematics
  EXPORT arm_kinematicsTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(ARM_KINEMATICS_BUILD_TOOLS)
  install(TARGETS dump_model workspace_report ik_benchmark
          RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

install(EXPORT arm_kinematicsTargets
  FILE arm_kinematicsTargets.cmake
  NAMESPACE arm_kinematics::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/arm_kinematics
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/arm_kinematicsConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm_kinematicsConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/arm_kinematicsConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/arm_kinematics
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/arm_kinematicsConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/arm_kinematicsConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/arm_kinematics
)
