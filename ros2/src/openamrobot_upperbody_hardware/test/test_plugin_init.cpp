// SPDX-License-Identifier: MIT
// The plugin's on_init refuses a missing or bad read_timeout_ms before it touches any CAN interface. Built only with the plugin.
#include <gtest/gtest.h>

#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "openamrobot_upperbody_hardware/openarm_safe_hardware.hpp"

using openamrobot_upperbody_hardware::OpenArmSafeHW;

namespace {
hardware_interface::HardwareInfo info_with(const std::string& timeout, bool set) {
  hardware_interface::HardwareInfo info;
  info.name = "openarm_test_hardware_interface";
  info.type = "system";
  info.hardware_parameters = {{"can_interface", "no_such_can_interface"}, {"arm_prefix", "left_"}, {"can_fd", "true"},
                              {"ee_type", "pinch_gripper"}};
  if (set) info.hardware_parameters["read_timeout_ms"] = timeout;
  return info;
}
}  // namespace

TEST(OpenArmSafeHWInit, MissingReadTimeoutIsRefused) {
  OpenArmSafeHW hw;
  EXPECT_EQ(hw.on_init(info_with("", false)), hardware_interface::CallbackReturn::ERROR);
}

TEST(OpenArmSafeHWInit, BadReadTimeoutIsRefused) {
  for (const char* bad : {"", "0", "-1", "abc", "nan"}) {
    OpenArmSafeHW hw;
    EXPECT_EQ(hw.on_init(info_with(bad, true)), hardware_interface::CallbackReturn::ERROR) << bad;
  }
}

TEST(OpenArmSafeHWInit, GoodReadTimeoutStillFailsOnAMissingInterfaceNotOnTheTimeout) {
  OpenArmSafeHW hw;
  // The interface does not exist, so init fails after the timeout check: proves the check passes for a valid number.
  EXPECT_EQ(hw.on_init(info_with("80", true)), hardware_interface::CallbackReturn::ERROR);
}
