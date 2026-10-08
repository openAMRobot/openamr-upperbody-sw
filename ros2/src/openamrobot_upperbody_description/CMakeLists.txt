cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes config launch DESTINATION share/openamrobot_upperbody_description)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)

  # ament_add_pytest_test only warns when pytest is missing, which would leave
  # an empty suite reported as "0 failures". Refuse that instead.
  ament_has_pytest(_has_pytest QUIET)
  if(NOT _has_pytest)
    message(FATAL_ERROR
      "pytest not importable by ${Python3_EXECUTABLE}; tests would be silently empty")
  endif()

  # Every configuration combination: xacro expansion and frame poses, no ROS graph.
  ament_add_pytest_test(test_upperbody_configurations test/test_upperbody_configurations.py
    TIMEOUT 300)
  # Every buildable combination published by robot_state_publisher and read back from TF.
  ament_add_ros_isolated_pytest_test(test_upperbody_tf test/test_upperbody_tf.py
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST TIMEOUT 900)
endif()

ament_package()
