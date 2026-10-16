cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_moveit)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/regenerate_acm.sh
  scripts/derive_acm.py
  scripts/validate_acm.py
  scripts/joint_state_clamp.py
  scripts/limits_sensitivity.py
  scripts/planning_time.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Static checks of the SRDF and the configuration files: no ROS runtime, no simulator.
  ament_add_pytest_test(srdf test/test_srdf.py TIMEOUT 600)
  ament_add_pytest_test(srdf_import test/test_srdf_import.py TIMEOUT 600)
  ament_add_pytest_test(acm_generation test/test_acm_generation.py TIMEOUT 600)
  ament_add_pytest_test(acm_derive test/test_acm_derive.py TIMEOUT 600)
  ament_add_pytest_test(joint_state_clamp_logic test/test_joint_state_clamp_logic.py TIMEOUT 600)
  # How the clamp ends: exit 0 on SIGINT and SIGTERM (a crash of a node this repo owns is fixed, not allow-listed).
  ament_add_pytest_test(joint_state_clamp_shutdown test/test_joint_state_clamp_shutdown.py TIMEOUT 600)
  ament_add_pytest_test(gripper_sign test/test_gripper_sign.py TIMEOUT 600)
  ament_add_pytest_test(config_files test/test_config_files.py TIMEOUT 600)
  # M03 and M04 build a MoveIt planning scene from the combined model (moveit_py) and need no controller and no simulator.
  ament_add_pytest_test(m03_acm_sampling test/test_m03_acm_sampling.py TIMEOUT 3000)
  ament_add_pytest_test(m06_limits_sensitivity test/test_m06_limits_sensitivity.py TIMEOUT 600)
  # M04 starts move_group and a joint state publisher (launch_testing), in its own ROS_DOMAIN_ID; it plans only, nothing executes.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_m04_collision_goal_rejected.py TIMEOUT 240)
  # The clamp in the loop: the defect without it (clamp:=false), and what the clamp does and does not hide (clamp:=true).
  add_launch_test(test/test_joint_state_clamp_loop.py TARGET joint_state_clamp_off ARGS "clamp:=false" TIMEOUT 240)
  add_launch_test(test/test_joint_state_clamp_loop.py TARGET joint_state_clamp_on ARGS "clamp:=true" TIMEOUT 240)
endif()

ament_package()
