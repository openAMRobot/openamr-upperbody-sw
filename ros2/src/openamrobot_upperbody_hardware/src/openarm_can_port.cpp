// SPDX-License-Identifier: MIT
#include "openamrobot_upperbody_hardware/openarm_can_port.hpp"

#include <chrono>
#include <thread>

#include "openamrobot_upperbody_hardware/safe_arm_core.hpp"

namespace openamrobot_upperbody_hardware {

namespace dm = openarm::damiao_motor;

namespace {
// OpenArm 2.0 motor layout as the pinned upstream hardware interface has it (openarm_simple_hardware.hpp, 4e837e1).
const std::vector<dm::MotorType> kMotorTypes = {dm::MotorType::DM8009, dm::MotorType::DM8009, dm::MotorType::DM4340,
                                                dm::MotorType::DM4340, dm::MotorType::DM4310, dm::MotorType::DM4310,
                                                dm::MotorType::DM4310};
const std::vector<uint32_t> kSendIds = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
const std::vector<uint32_t> kRecvIds = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
constexpr uint32_t kGripperSendId = 0x08;
constexpr uint32_t kGripperRecvId = 0x18;
// How long recv_all waits for the first reply of a cycle.
constexpr int kRecvFirstTimeoutUs = 2000;

AxisState axis_of(const dm::Motor& m, const dm::MotorLinkStats& s) {
  AxisState a;
  a.q = m.get_position();
  a.dq = m.get_velocity();
  a.tau = m.get_torque();
  a.ever_responded = s.ever_responded();
  a.age_s = a.ever_responded ? std::chrono::duration<double>(s.since_last_response()).count() : 0.0;
  return a;
}
}  // namespace

OpenArmCanPort::OpenArmCanPort(const std::string& can_interface, bool can_fd, bool hand)
    : hand_(hand), arm_(std::make_unique<openarm::can::socket::OpenArm>(can_interface, can_fd)) {
  arm_->init_arm_motors(kMotorTypes, kSendIds, kRecvIds);
  if (hand_) arm_->init_gripper_motor(dm::MotorType::DM4310, kGripperSendId, kGripperRecvId);
}

void OpenArmCanPort::prime() {
  arm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  arm_->recv_all();
}

void OpenArmCanPort::request_state() {
  arm_->refresh_all();
  arm_->recv_all(kRecvFirstTimeoutUs);
}

std::vector<AxisState> OpenArmCanPort::axes() const {
  std::vector<AxisState> out;
  const auto motors = arm_->get_arm().get_motors();
  for (size_t i = 0; i < motors.size(); ++i) {
    out.push_back(axis_of(motors[i], arm_->get_arm().get_link_stats(static_cast<int>(i))));
  }
  if (hand_) {
    const auto g = arm_->get_gripper().get_motors();
    if (!g.empty()) out.push_back(axis_of(g[0], arm_->get_gripper().get_link_stats(0)));
  }
  return out;
}

BusReport OpenArmCanPort::bus() const {
  const auto& s = arm_->get_bus_status();
  BusReport r;
  r.link_running = arm_->is_link_running();
  r.bus_off = static_cast<bool>(s.bus_off);
  r.error_events = s.error_passive.count + s.tx_overflow.count + s.rx_overflow.count + s.ack_error.count +
                   s.tx_timeout.count;
  r.write_failures = s.write_net_down.count + s.write_no_buffer.count + s.write_other.count;
  return r;
}

void OpenArmCanPort::clear_bus_counters() { arm_->clear_bus_status(); }

bool OpenArmCanPort::enable() {
  arm_->set_callback_mode_all(dm::CallbackMode::STATE);
  arm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  arm_->recv_all();
  return bus().write_failures == 0;
}

bool OpenArmCanPort::send(const std::vector<MitCommand>& commands) {
  const uint64_t before = bus().write_failures;
  std::vector<dm::MITParam> arm_params;
  for (size_t i = 0; i < kArmDof && i < commands.size(); ++i) {
    const auto& c = commands[i];
    arm_params.push_back({c.kp, c.kd, c.q, c.dq, c.tau});
  }
  arm_->get_arm().mit_control_all(arm_params);
  if (hand_ && commands.size() > kArmDof) {
    const auto& c = commands[kArmDof];
    arm_->get_gripper().mit_control_all({{c.kp, c.kd, c.q, c.dq, c.tau}});
  }
  return bus().write_failures == before;
}

void OpenArmCanPort::disable() {
  for (int i = 0; i < 3; ++i) {
    arm_->disable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    arm_->recv_all();
  }
}

}  // namespace openamrobot_upperbody_hardware
