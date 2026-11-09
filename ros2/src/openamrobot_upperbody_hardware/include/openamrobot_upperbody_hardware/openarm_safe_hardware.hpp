// SPDX-License-Identifier: MIT
// ros2_control SystemInterface for one OpenArm: the safety core (safe_arm_core.hpp) on the openarm_can port.
// Same hardware parameters as upstream's openarm_hardware/OpenArmHW (can_interface, arm_prefix, hand, can_fd, ee_type,
// kp1..7, kd1..7, kp_hand, kd_hand) plus read_timeout_ms.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "openamrobot_upperbody_hardware/openarm_can_port.hpp"
#include "openamrobot_upperbody_hardware/safe_arm_core.hpp"

namespace openamrobot_upperbody_hardware {

class OpenArmSafeHW : public hardware_interface::SystemInterface {
 public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  std::vector<std::string> joint_names_;
  std::unique_ptr<OpenArmCanPort> port_;
  std::unique_ptr<SafeArmCore> core_;
};

}  // namespace openamrobot_upperbody_hardware
