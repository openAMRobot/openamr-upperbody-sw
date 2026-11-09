// SPDX-License-Identifier: MIT
#include "openamrobot_upperbody_hardware/openarm_safe_hardware.hpp"

#include <algorithm>
#include <cctype>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace openamrobot_upperbody_hardware {

namespace {
rclcpp::Logger log() { return rclcpp::get_logger("OpenArmSafeHW"); }

std::string param(const hardware_interface::HardwareInfo& info, const std::string& key, const std::string& fallback) {
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}

bool as_bool(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return std::tolower(c); });
  return v == "true";
}
}  // namespace

hardware_interface::CallbackReturn OpenArmSafeHW::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  try {
    const std::string can_interface = param(info, "can_interface", "can0");
    const std::string prefix = param(info, "arm_prefix", "");
    const bool hand = as_bool(param(info, "hand", "true"));
    const bool can_fd = as_bool(param(info, "can_fd", "true"));

    CoreConfig cfg;
    cfg.hand = hand;
    cfg.ee_type = param(info, "ee_type", "parallel_link");
    cfg.kp = {70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0};  // upstream defaults, overridden by the xacro gains
    cfg.kd = {2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5};
    for (size_t i = 1; i <= kArmDof; ++i) {
      cfg.kp[i - 1] = std::stod(param(info, "kp" + std::to_string(i), std::to_string(cfg.kp[i - 1])));
      cfg.kd[i - 1] = std::stod(param(info, "kd" + std::to_string(i), std::to_string(cfg.kd[i - 1])));
    }
    cfg.gripper_kp = std::stod(param(info, "kp_hand", std::to_string(cfg.gripper_kp)));
    cfg.gripper_kd = std::stod(param(info, "kd_hand", std::to_string(cfg.gripper_kd)));
    // No default: the value is a bench measurement (TODO(Om)). Refuse to initialise without it.
    std::string why;
    const auto timeout = info.hardware_parameters.find("read_timeout_ms");
    if (!parse_read_timeout_ms(timeout == info.hardware_parameters.end() ? "" : timeout->second, cfg.read_timeout_s, why)) {
      RCLCPP_ERROR(log(), "%s", why.c_str());
      return CallbackReturn::ERROR;
    }

    for (size_t i = 1; i <= kArmDof; ++i) joint_names_.push_back("openarm_" + prefix + "joint" + std::to_string(i));
    if (hand) joint_names_.push_back("openarm_" + prefix + "finger_joint1");

    port_ = std::make_unique<OpenArmCanPort>(can_interface, can_fd, hand);
    core_ = std::make_unique<SafeArmCore>(*port_, cfg);
    RCLCPP_INFO(log(), "%s: CAN %s fd=%d hand=%d read_timeout=%.0f ms", prefix.c_str(), can_interface.c_str(), can_fd,
                hand, cfg.read_timeout_s * 1000.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(log(), "init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmSafeHW::on_configure(const rclcpp_lifecycle::State&) {
  port_->prime();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OpenArmSafeHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &core_->pos_state[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &core_->vel_state[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &core_->tau_state[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> OpenArmSafeHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &core_->pos_cmd[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &core_->vel_cmd[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &core_->tau_cmd[i]);
  }
  return out;
}

hardware_interface::CallbackReturn OpenArmSafeHW::on_activate(const rclcpp_lifecycle::State&) {
  if (!core_->activate()) {
    RCLCPP_ERROR(log(), "%s", core_->fault().c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(log(), "activated, holding the measured position");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmSafeHW::on_deactivate(const rclcpp_lifecycle::State&) {
  RCLCPP_WARN(log(), "deactivating: motors are disabled, the arm becomes back-drivable");
  core_->deactivate();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmSafeHW::read(const rclcpp::Time&, const rclcpp::Duration&) {
  if (core_->read() == Result::kOk) return hardware_interface::return_type::OK;
  RCLCPP_ERROR_THROTTLE(log(), *get_node()->get_clock(), 1000, "read fault: %s", core_->fault().c_str());
  return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type OpenArmSafeHW::write(const rclcpp::Time&, const rclcpp::Duration&) {
  if (core_->write() == Result::kOk) return hardware_interface::return_type::OK;
  RCLCPP_ERROR_THROTTLE(log(), *get_node()->get_clock(), 1000, "write fault: %s", core_->fault().c_str());
  return hardware_interface::return_type::ERROR;
}

}  // namespace openamrobot_upperbody_hardware

PLUGINLIB_EXPORT_CLASS(openamrobot_upperbody_hardware::OpenArmSafeHW, hardware_interface::SystemInterface)
