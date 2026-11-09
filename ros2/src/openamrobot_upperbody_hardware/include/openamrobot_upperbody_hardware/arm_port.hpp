// SPDX-License-Identifier: MIT
// The CAN side of one OpenArm, as the safety core sees it. No ROS and no openarm_can types in here, so the core builds
// and is tested on any Linux runner with a mock port.
#pragma once

#include <cstdint>
#include <vector>

namespace openamrobot_upperbody_hardware {

// One MIT-mode command for one motor (Damiao): PD gains, target position, target velocity, feed-forward torque.
struct MitCommand {
  double kp = 0.0;
  double kd = 0.0;
  double q = 0.0;
  double dq = 0.0;
  double tau = 0.0;
};

// Last reply of one motor. Positions are motor-side (the gripper is mapped to joint space by the core).
struct AxisState {
  double q = 0.0;
  double dq = 0.0;
  double tau = 0.0;
  bool ever_responded = false;
  // Seconds since the last reply from this motor. Ignored while ever_responded is false.
  double age_s = 0.0;
};

// State of the interface, not of one motor.
struct BusReport {
  bool link_running = true;       // interface is up and running
  bool bus_off = false;           // bus-off seen since the port was cleared
  uint64_t error_events = 0;      // every other controller or driver error counted since the port was cleared
  uint64_t write_failures = 0;    // frames the kernel refused, since the port was cleared
};

class ArmPort {
 public:
  virtual ~ArmPort() = default;

  // Ask every motor for its state and receive what arrives (one control cycle's worth of I/O).
  virtual void request_state() = 0;
  // Axes in order: arm joints 1..7, then the gripper when present.
  virtual std::vector<AxisState> axes() const = 0;
  virtual BusReport bus() const = 0;
  // Reset the bus counters. The core calls this once at activation so that only new events count as faults.
  virtual void clear_bus_counters() = 0;
  // Enable the motors. Returns false if the frames could not be sent.
  virtual bool enable() = 0;
  // Send one command per axis, same order as axes(). Returns false if the frames could not be sent.
  virtual bool send(const std::vector<MitCommand>& commands) = 0;
  // Disable the motors (the arm becomes back-drivable).
  virtual void disable() = 0;
};

}  // namespace openamrobot_upperbody_hardware
