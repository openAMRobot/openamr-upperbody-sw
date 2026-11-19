# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)
project(openamrobot_control_room)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/control_room_monitor.py
  scripts/vcan_monitor.py
  scripts/mission_report.py
  scripts/control_room_replay.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # No ROS graph: the read-only rule on the configurations, the arithmetic, the CAN counters, the CAN decoder and the
  # topic list against two recorded runs (test/fixtures/runs), the HTML report and the installed-package check.
  foreach(name static state can_monitor can_decode topic_audit report installed_check)
    ament_add_pytest_test(test_${name} test/test_${name}.py TIMEOUT 120)
  endforeach()
  # ROS 2: the nodes' graph (no publisher outside /control_room/, no service or action client), the synthetic bag
  # through the report generator, the receive-only CAN monitor on vcan, the read-only foxglove_bridge and the REPLAY.
  foreach(name monitor_graph bag_report vcan_listen foxglove_readonly replay)
    ament_add_pytest_test(test_${name} test/test_${name}.py TIMEOUT 300)
  endforeach()
endif()

ament_package()
