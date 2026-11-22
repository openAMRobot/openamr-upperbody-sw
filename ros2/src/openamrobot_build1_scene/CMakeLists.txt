# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)
project(openamrobot_build1_scene)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch urdf rviz traces DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/base_state_adapter.py
  scripts/can_replay.py
  scripts/finger_mimic_follower.py
  scripts/gravity_feedforward.py
  scripts/footprint_publisher.py
  scripts/generate_world.py
  scripts/inject_stale_sensors.py
  scripts/planning_scene_loader.py
  scripts/rtf_monitor.py
  scripts/wait_for_clock.py
  scripts/clock_capture.py
  scripts/run_scenario.py
  scripts/setup_vcan_all.sh
  scripts/single_writer_check.py
  scripts/wheel_command_adapter.py
  scripts/count_build1_tests.py
  scripts/diag_table.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Static tests: no simulator, no virtual CAN. They run on any machine.
  foreach(name scene world_and_map launch_support command_map footprint odometry candump docs_and_pins gravity baseline1_phases baseline1_record baseline1_strict clock_gate baseline1_cancel station_departure arm_frames lift_plant diagnostics station_marker lift_permit_wait permit_waits_static scene_reach_targets)
    ament_add_pytest_test(test_${name} test/test_${name}.py TIMEOUT 120)
  endforeach()
  # Component liveness at the end of every scene (rig.run), static.
  ament_add_pytest_test(test_component_liveness test/test_component_liveness.py TIMEOUT 120)

  # Virtual CAN tests: BLOCKED, never passed, where the interfaces cannot be had.
  ament_add_pytest_test(test_canfd_vcan test/test_canfd_vcan.py TIMEOUT 120)
  ament_add_pytest_test(test_replay_vcan test/test_replay_vcan.py TIMEOUT 180)
  set_tests_properties(test_canfd_vcan test_replay_vcan PROPERTIES RESOURCE_LOCK build1_vcan RUN_SERIAL TRUE)

  # Scenarios: the whole stack, headless. One launch per scenario, each with its own timeout.
  # BUILD1_SCENARIOS=OFF leaves them out (they need Gazebo, about 4 cores and several minutes each).
  option(BUILD1_SCENARIOS "Register the headless Build 1 scenario tests" ON)
  if(BUILD1_SCENARIOS)
    set(scenarios
      scn01_drive:1500
      scn02_lift_permit:1500
      scn03_arms_reach:2400
      scn04_grasp:2400
      scn05_bus_loss:2400
      scn06_stale_sensors:1800
      scn07_cancel_resume:1800
      scn08_arm_power_cut:1500
      scn09_single_writer:1500
      scn10_profiles:1500
      scn11_profile_sil_fine:1800
      scn12_profile_fast:1800
      scn13_profile_fine:2400
      scn14_drive_fast:1500
      scn15_grasp_fast:1800
      scn16_lift_fast:1500
      scn17_base_yaw_fast:1500)
    foreach(entry ${scenarios})
      string(REPLACE ":" ";" parts ${entry})
      list(GET parts 0 name)
      list(GET parts 1 seconds)
      add_launch_test(test/test_${name}.py TARGET test_${name} TIMEOUT ${seconds})
      set_tests_properties(test_${name} PROPERTIES RESOURCE_LOCK build1_vcan RUN_SERIAL TRUE)
    endforeach()
    # Integrated Digital Twin Baseline 1: the whole-robot mission, run by .github/workflows/baseline1-acceptance.yml.
    add_launch_test(test/test_baseline1_mission.py TARGET test_baseline1_mission TIMEOUT 5400)
    set_tests_properties(test_baseline1_mission PROPERTIES RESOURCE_LOCK build1_vcan RUN_SERIAL TRUE)
    # Arm diagnostics (.github/workflows/build1-arm-diagnostics.yml): scn03 and scn04 with contact sensors and a recorder.
    # They need BUILD1_CONTACT_SENSORS=1 in the environment; the scene workflow does not select them.
    foreach(name diag_grasp diag_reach_return)
      add_launch_test(test/test_${name}.py TARGET test_${name} TIMEOUT 2400)
      set_tests_properties(test_${name} PROPERTIES RESOURCE_LOCK build1_vcan RUN_SERIAL TRUE)
    endforeach()
  endif()
endif()

ament_package()
