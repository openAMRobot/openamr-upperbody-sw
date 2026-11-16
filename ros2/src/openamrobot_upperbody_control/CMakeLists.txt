# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_control)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/openamrobot_upperbody_control)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(arm_controllers_yaml test/test_arm_controllers_yaml.py TIMEOUT 300)
  # the simulation-only overlay (upstream update rate, gripper max_effort): equal to the simulator dynamics profile, adds no controller
  ament_add_pytest_test(sim_overlay_yaml test/test_sim_overlay_yaml.py TIMEOUT 300)
endif()

ament_package()
