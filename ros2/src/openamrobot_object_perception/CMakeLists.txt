# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)
project(openamrobot_object_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/object_pose_estimator.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_box_fit test/test_box_fit.py TIMEOUT 120)
endif()

ament_package()
