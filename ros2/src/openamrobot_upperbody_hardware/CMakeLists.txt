# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# The safety core: no ROS, no CAN library.
add_library(${PROJECT_NAME}_core src/safe_arm_core.cpp)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# The plugin needs the CAN library, which is built from source (pinned in config/upstream_pins.yaml of the description
# package). Without it only the core and its tests are built, so CI without openarm_can still tests the safety logic.
find_package(OpenArmCAN QUIET)
find_package(hardware_interface QUIET)
find_package(pluginlib QUIET)
find_package(rclcpp QUIET)
find_package(rclcpp_lifecycle QUIET)
if(OpenArmCAN_FOUND AND hardware_interface_FOUND AND pluginlib_FOUND AND rclcpp_FOUND AND rclcpp_lifecycle_FOUND)
  add_library(${PROJECT_NAME}_plugin SHARED
    src/openarm_can_port.cpp
    src/openarm_safe_hardware.cpp)
  # Pluginlib's class list names the library "openamrobot_upperbody_hardware_plugin".
  target_link_libraries(${PROJECT_NAME}_plugin PUBLIC ${PROJECT_NAME}_core OpenArmCAN::openarm_can)
  ament_target_dependencies(${PROJECT_NAME}_plugin PUBLIC hardware_interface pluginlib rclcpp rclcpp_lifecycle)
  pluginlib_export_plugin_description_file(hardware_interface openamrobot_upperbody_hardware.xml)
  install(TARGETS ${PROJECT_NAME}_plugin DESTINATION lib)
else()
  message(WARNING "OpenArmCAN or the ros2_control packages were not found: building the safety core only, "
                  "no hardware plugin")
endif()

install(TARGETS ${PROJECT_NAME}_core EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safe_arm_core test/test_safe_arm_core.cpp)
  target_link_libraries(test_safe_arm_core ${PROJECT_NAME}_core)
  if(TARGET ${PROJECT_NAME}_plugin)
    # on_init refuses a missing read_timeout_ms (no default); needs the plugin, so only with openarm_can
    ament_add_gtest(test_plugin_init test/test_plugin_init.cpp)
    target_link_libraries(test_plugin_init ${PROJECT_NAME}_plugin)
    ament_target_dependencies(test_plugin_init hardware_interface)
  endif()

  # scripts/openarm_can_setup.sh against a fake ip: needs a checkout (the script lives at the repository root)
  find_package(ament_cmake_pytest REQUIRED)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/openarm_can_setup.sh)
    ament_add_pytest_test(openarm_can_setup test/test_openarm_can_setup.py TIMEOUT 120)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_package()
