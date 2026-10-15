cmake_minimum_required(VERSION 3.16)
project(arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch rviz
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The description is checked against the model it was generated from by
  # walking both chains numerically. See test/test_urdf_matches_model.py.
  ament_add_pytest_test(urdf_matches_model test/test_urdf_matches_model.py
    TIMEOUT 120)
endif()

ament_package()
