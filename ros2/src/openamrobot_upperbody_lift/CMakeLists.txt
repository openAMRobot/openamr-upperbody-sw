# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_lift)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/lift_relay.py scripts/lift_guard.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # no ROS graph: the decision logic and the command stub
  ament_add_pytest_test(lift_evaluator test/test_lift_evaluator.py TIMEOUT 120)
  ament_add_pytest_test(lift_guard_logic test/test_lift_guard_logic.py TIMEOUT 120)
  ament_add_pytest_test(lift_command_stub test/test_lift_command_stub.py TIMEOUT 120)
  # relay and guard nodes against fake publishers (own ROS_DOMAIN_ID, localhost only)
  ament_add_pytest_test(lift_relay_node test/test_lift_relay_node.py TIMEOUT 180)
  ament_add_pytest_test(lift_guard_node test/test_lift_guard_node.py TIMEOUT 180)
endif()

ament_package()
