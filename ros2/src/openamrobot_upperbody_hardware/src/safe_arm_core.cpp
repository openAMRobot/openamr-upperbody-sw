// SPDX-License-Identifier: MIT
#include "openamrobot_upperbody_hardware/safe_arm_core.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace openamrobot_upperbody_hardware {

namespace {
// Upstream's constants for the parallel_link end effector (prismatic 0..0.044 m <-> motor 0..-1.0472 rad).
constexpr double kParallelOpenJoint = 0.044;
constexpr double kParallelOpenMotor = -1.0472;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

double gripper_motor_to_joint(const std::string& ee_type, double motor_rad) {
  if (ee_type == "pinch_gripper") return motor_rad;
  return kParallelOpenJoint * (motor_rad / kParallelOpenMotor);
}

double gripper_joint_to_motor(const std::string& ee_type, double joint) {
  if (ee_type == "pinch_gripper") return joint;
  return (joint / kParallelOpenJoint) * kParallelOpenMotor;
}

SafeArmCore::SafeArmCore(ArmPort& port, CoreConfig config)
    : port_(port), cfg_(std::move(config)), joints_(kArmDof + (cfg_.hand ? 1 : 0)) {
  if (cfg_.kp.size() != kArmDof || cfg_.kd.size() != kArmDof) {
    throw std::invalid_argument("kp and kd need one entry per arm joint");
  }
  pos_state.assign(joints_, 0.0);
  vel_state.assign(joints_, 0.0);
  tau_state.assign(joints_, 0.0);
  pos_cmd.assign(joints_, 0.0);
  vel_cmd.assign(joints_, 0.0);
  tau_cmd.assign(joints_, 0.0);
  if (cfg_.hand) {
    vel_state[kArmDof] = kNaN;
    tau_state[kArmDof] = kNaN;
  }
}

void SafeArmCore::latch(const std::string& why) {
  if (!faulted_) {
    faulted_ = true;
    fault_ = why;
  }
}

bool SafeArmCore::fresh_state(const std::vector<AxisState>& axes, std::string& why) const {
  if (axes.size() != joints_) {
    why = "port reports " + std::to_string(axes.size()) + " axes, expected " + std::to_string(joints_);
    return false;
  }
  for (size_t i = 0; i < axes.size(); ++i) {
    const auto& a = axes[i];
    if (!a.ever_responded) {
      why = "axis " + std::to_string(i + 1) + " has never answered";
      return false;
    }
    if (!(a.age_s <= cfg_.read_timeout_s)) {
      why = "axis " + std::to_string(i + 1) + " silent for " + std::to_string(a.age_s) + " s";
      return false;
    }
    if (!std::isfinite(a.q)) {
      why = "axis " + std::to_string(i + 1) + " position is not finite";
      return false;
    }
  }
  return true;
}

bool SafeArmCore::bus_ok(std::string& why) const {
  const BusReport b = port_.bus();
  if (!b.link_running) {
    why = "CAN link is down";
    return false;
  }
  if (b.bus_off) {
    why = "CAN bus-off";
    return false;
  }
  if (b.error_events > base_events_) {
    why = "CAN controller reported errors since activation";
    return false;
  }
  if (b.write_failures > base_write_failures_) {
    why = "CAN frame write failed";
    return false;
  }
  return true;
}

void SafeArmCore::copy_state(const std::vector<AxisState>& axes) {
  for (size_t i = 0; i < kArmDof; ++i) {
    pos_state[i] = axes[i].q;
    vel_state[i] = axes[i].dq;
    tau_state[i] = axes[i].tau;
  }
  if (cfg_.hand) {
    pos_state[kArmDof] = gripper_motor_to_joint(cfg_.ee_type, axes[kArmDof].q);
    // Not mapped to joint space (upstream marks it unimplemented): unavailable, not zero.
    vel_state[kArmDof] = kNaN;
    tau_state[kArmDof] = kNaN;
  }
}

std::vector<MitCommand> SafeArmCore::commands() const {
  std::vector<MitCommand> out;
  out.reserve(joints_);
  for (size_t i = 0; i < kArmDof; ++i) {
    out.push_back({cfg_.kp[i], cfg_.kd[i], pos_cmd[i], vel_cmd[i], tau_cmd[i]});
  }
  if (cfg_.hand) {
    out.push_back({cfg_.gripper_kp, cfg_.gripper_kd, gripper_joint_to_motor(cfg_.ee_type, pos_cmd[kArmDof]), 0.0, 0.0});
  }
  return out;
}

bool SafeArmCore::activate() {
  active_ = false;
  faulted_ = false;
  fault_.clear();

  std::string why;
  bool ok = false;
  for (int i = 0; i < cfg_.activation_attempts && !ok; ++i) {
    port_.request_state();
    ok = fresh_state(port_.axes(), why);
  }
  if (!ok) {
    latch("activation refused, " + why);
    return false;
  }
  port_.clear_bus_counters();
  const BusReport b = port_.bus();
  if (!b.link_running) {
    latch("activation refused, CAN link is down");
    return false;
  }
  base_events_ = b.error_events;
  base_write_failures_ = b.write_failures;

  if (!port_.enable()) {
    latch("activation refused, enable frames could not be sent");
    return false;
  }
  // Enabling can take a moment to show in the replies: measure again, and hold where the arm is NOW.
  ok = false;
  for (int i = 0; i < cfg_.activation_attempts && !ok; ++i) {
    port_.request_state();
    ok = fresh_state(port_.axes(), why);
  }
  if (!ok) {
    latch("activation failed after enable, " + why);
    return false;
  }
  const auto axes = port_.axes();
  copy_state(axes);
  for (size_t i = 0; i < joints_; ++i) {
    pos_cmd[i] = pos_state[i];
    vel_cmd[i] = 0.0;
    tau_cmd[i] = 0.0;
  }
  if (!bus_ok(why)) {
    latch("activation failed, " + why);
    return false;
  }
  if (!port_.send(commands()) || !bus_ok(why)) {
    latch("activation failed, hold command could not be sent");
    return false;
  }
  active_ = true;
  return true;
}

void SafeArmCore::deactivate() {
  active_ = false;
  port_.disable();
}

Result SafeArmCore::read() {
  if (faulted_) return Result::kError;
  port_.request_state();
  std::string why;
  if (!active_) {
    // Not commanding (before activation or after deactivation): publish state when it is fresh, never claim a fault.
    const auto idle_axes = port_.axes();
    if (fresh_state(idle_axes, why)) copy_state(idle_axes);
    return Result::kOk;
  }
  if (!bus_ok(why)) {
    latch(why);
    return Result::kError;
  }
  const auto axes = port_.axes();
  if (!fresh_state(axes, why)) {
    latch(why);
    return Result::kError;
  }
  copy_state(axes);
  return Result::kOk;
}

Result SafeArmCore::write() {
  if (faulted_) return Result::kError;
  if (!active_) return Result::kOk;  // nothing is sent while not active
  for (size_t i = 0; i < joints_; ++i) {
    if (!std::isfinite(pos_cmd[i]) || !std::isfinite(vel_cmd[i]) || !std::isfinite(tau_cmd[i])) {
      latch("command for joint " + std::to_string(i + 1) + " is not finite");
      return Result::kError;
    }
  }
  std::string why;
  if (!port_.send(commands())) {
    latch("CAN send failed");
    return Result::kError;
  }
  if (!bus_ok(why)) {
    latch(why);
    return Result::kError;
  }
  return Result::kOk;
}

}  // namespace openamrobot_upperbody_hardware
