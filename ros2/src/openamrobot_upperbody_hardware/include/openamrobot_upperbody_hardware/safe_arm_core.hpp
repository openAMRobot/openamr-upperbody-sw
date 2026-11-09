// SPDX-License-Identifier: MIT
// Safety logic of the OpenArm hardware interface, independent of ROS and of the CAN library.
//
// What it changes against enactic/openarm_ros2 openarm_hardware (4e837e1, openarm_simple_hardware.cpp):
//   - activation holds the arm where it is (command = state); upstream interpolates both arms to joint zero;
//   - read() returns ERROR when a motor has not answered within read_timeout_s, never answered, the link is down, or the
//     bus went bus-off; upstream returns OK always;
//   - write() returns ERROR when a frame could not be sent or a command is not finite; upstream returns OK always;
//   - the gripper velocity and effort states are NaN (not available), not a hard-coded 0.
// After the first fault the core stays faulted until activate() is called again, so a transient recovery of the bus
// cannot hide that state was missing for a while.
#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "openamrobot_upperbody_hardware/arm_port.hpp"

namespace openamrobot_upperbody_hardware {

constexpr size_t kArmDof = 7;

enum class Result { kOk, kError };

struct CoreConfig {
  std::vector<double> kp;  // size kArmDof
  std::vector<double> kd;  // size kArmDof
  bool hand = true;
  std::string ee_type = "pinch_gripper";  // or "parallel_link", the two mappings upstream knows
  double gripper_kp = 5.0;
  double gripper_kd = 0.1;
  // A reply older than this is a fault. There is deliberately no default: the number comes from a bench measurement
  // (TODO(Om)), and SafeArmCore refuses to be built without a finite value greater than 0.
  double read_timeout_s = std::numeric_limits<double>::quiet_NaN();
  // How often activation asks for state before giving up (the adapter's request_state() blocks briefly each time).
  int activation_attempts = 10;
};

// Parses the read_timeout_ms parameter text into seconds. False, with the reason in `why`, when the text is empty, not a
// number, not finite or not greater than 0. Shared by the plugin and the tests, so "no default" has one definition.
bool parse_read_timeout_ms(const std::string& text, double& seconds, std::string& why);

// Upstream's gripper mapping (openarm_simple_hardware.cpp, marked approximate there). Pure functions, tested.
double gripper_motor_to_joint(const std::string& ee_type, double motor_rad);
double gripper_joint_to_motor(const std::string& ee_type, double joint);

class SafeArmCore {
 public:
  SafeArmCore(ArmPort& port, CoreConfig config);

  size_t joint_count() const { return joints_; }

  // Read state, require every axis fresh, command = state (velocity and torque 0), enable, read again, send a hold
  // command at the measured position. Never sends a position the arm is not already at.
  bool activate();
  void deactivate();
  Result read();
  Result write();

  bool faulted() const { return faulted_; }
  const std::string& fault() const { return fault_; }

  // ros2_control interface storage, one entry per joint (arm joints, then gripper). The adapter exports pointers to these.
  std::vector<double> pos_state, vel_state, tau_state;
  std::vector<double> pos_cmd, vel_cmd, tau_cmd;

 private:
  bool fresh_state(const std::vector<AxisState>& axes, std::string& why) const;
  bool bus_ok(std::string& why) const;
  void copy_state(const std::vector<AxisState>& axes);
  void latch(const std::string& why);
  std::vector<MitCommand> commands() const;

  ArmPort& port_;
  CoreConfig cfg_;
  size_t joints_;
  bool active_ = false;
  bool faulted_ = false;
  std::string fault_;
  uint64_t base_events_ = 0;
  uint64_t base_write_failures_ = 0;
};

}  // namespace openamrobot_upperbody_hardware
