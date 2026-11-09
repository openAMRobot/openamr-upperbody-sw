// SPDX-License-Identifier: MIT
// Safety core against a mock CAN layer: no hardware, no CAN library, no ROS graph.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "openamrobot_upperbody_hardware/safe_arm_core.hpp"

using namespace openamrobot_upperbody_hardware;

namespace {

class MockPort : public ArmPort {
 public:
  explicit MockPort(bool hand = true) : n_(kArmDof + (hand ? 1 : 0)) {
    axes_.resize(n_);
    for (size_t i = 0; i < n_; ++i) {
      axes_[i].q = 0.1 * static_cast<double>(i + 1);  // the arm is NOT at zero
      axes_[i].dq = 0.01;
      axes_[i].tau = 0.5;
      axes_[i].ever_responded = true;
      axes_[i].age_s = 0.001;
    }
  }
  void request_state() override { ++requests; }
  std::vector<AxisState> axes() const override { return axes_; }
  BusReport bus() const override { return bus_; }
  void clear_bus_counters() override {
    ++clears;
    bus_.bus_off = false;
    bus_.error_events = 0;
    bus_.write_failures = 0;
  }
  bool enable() override {
    ++enables;
    return enable_ok;
  }
  bool send(const std::vector<MitCommand>& c) override {
    sent.push_back(c);
    if (!send_ok) ++bus_.write_failures;
    return send_ok;
  }
  void disable() override { ++disables; }

  size_t n_;
  std::vector<AxisState> axes_;
  BusReport bus_;
  bool enable_ok = true;
  bool send_ok = true;
  int requests = 0, clears = 0, enables = 0, disables = 0;
  std::vector<std::vector<MitCommand>> sent;
};

CoreConfig config() {
  CoreConfig c;
  c.kp = {70, 70, 70, 60, 10, 10, 10};
  c.kd = {2.75, 2.5, 2, 2, 0.7, 0.6, 0.5};
  c.ee_type = "pinch_gripper";
  c.read_timeout_s = 0.05;
  return c;
}

}  // namespace

TEST(SafeArmCore, ActivationHoldsMeasuredPositionAndNeverSendsZero) {
  MockPort port;
  SafeArmCore core(port, config());
  ASSERT_TRUE(core.activate());
  ASSERT_EQ(port.sent.size(), 1u);
  for (size_t i = 0; i < core.joint_count(); ++i) {
    EXPECT_DOUBLE_EQ(core.pos_cmd[i], core.pos_state[i]);
    EXPECT_DOUBLE_EQ(port.sent[0][i].q, port.axes_[i].q) << "joint " << i;
    EXPECT_NE(port.sent[0][i].q, 0.0);
    EXPECT_DOUBLE_EQ(port.sent[0][i].dq, 0.0);
    EXPECT_DOUBLE_EQ(port.sent[0][i].tau, 0.0);
  }
  EXPECT_EQ(port.enables, 1);
}

TEST(SafeArmCore, ActivationUsesPositionMeasuredAfterEnable) {
  struct Drift : MockPort {
    void request_state() override {
      MockPort::request_state();
      if (enables > 0) axes_[0].q = 0.5;  // the joint settled elsewhere once enabled
    }
  } port;
  SafeArmCore core(port, config());
  ASSERT_TRUE(core.activate());
  EXPECT_DOUBLE_EQ(port.sent[0][0].q, 0.5);
}

TEST(SafeArmCore, ActivationRefusedWhenAMotorNeverAnswered) {
  MockPort port;
  port.axes_[3].ever_responded = false;
  SafeArmCore core(port, config());
  EXPECT_FALSE(core.activate());
  EXPECT_EQ(port.enables, 0);
  EXPECT_TRUE(port.sent.empty());
  EXPECT_NE(core.fault().find("axis 4"), std::string::npos);
  EXPECT_EQ(core.read(), Result::kError);
}

TEST(SafeArmCore, ActivationRefusedWhenLinkDownOrEnableFails) {
  {
    MockPort port;
    port.bus_.link_running = false;
    SafeArmCore core(port, config());
    EXPECT_FALSE(core.activate());
    EXPECT_TRUE(port.sent.empty());
  }
  {
    MockPort port;
    port.enable_ok = false;
    SafeArmCore core(port, config());
    EXPECT_FALSE(core.activate());
    EXPECT_TRUE(port.sent.empty());
  }
}

TEST(SafeArmCore, ActivationFailsWhenHoldCommandCannotBeSent) {
  MockPort port;
  port.send_ok = false;
  SafeArmCore core(port, config());
  EXPECT_FALSE(core.activate());
}

TEST(SafeArmCore, ReadOkWhenFreshAndCopiesState) {
  MockPort port;
  SafeArmCore core(port, config());
  ASSERT_TRUE(core.activate());
  port.axes_[2].q = 0.9;
  EXPECT_EQ(core.read(), Result::kOk);
  EXPECT_DOUBLE_EQ(core.pos_state[2], 0.9);
}

TEST(SafeArmCore, ReadTimeoutIsError) {
  MockPort port;
  SafeArmCore core(port, config());
  ASSERT_TRUE(core.activate());
  port.axes_[5].age_s = 0.2;  // one motor went quiet
  EXPECT_EQ(core.read(), Result::kError);
  EXPECT_TRUE(core.faulted());
  EXPECT_NE(core.fault().find("axis 6"), std::string::npos);
}

TEST(SafeArmCore, ReadTimeoutIsConfigurable) {
  MockPort port;
  auto cfg = config();
  cfg.read_timeout_s = 0.5;
  SafeArmCore core(port, cfg);
  ASSERT_TRUE(core.activate());
  port.axes_[0].age_s = 0.2;
  EXPECT_EQ(core.read(), Result::kOk);
  port.axes_[0].age_s = 0.6;
  EXPECT_EQ(core.read(), Result::kError);
}

TEST(SafeArmCore, GripperSilenceIsAlsoError) {
  MockPort port;
  SafeArmCore core(port, config());
  ASSERT_TRUE(core.activate());
  port.axes_[kArmDof].age_s = 1.0;
  EXPECT_EQ(core.read(), Result::kError);
}

TEST(SafeArmCore, BusOffIsError) {
  MockPort port;
  SafeArmCore core(port, config());
  ASSERT_TRUE(core.activate());
  port.bus_.bus_off = true;
  EXPECT_EQ(core.read(), Result::kError);
  EXPECT_EQ(core.fault(), "CAN bus-off");
}

TEST(SafeArmCore, LinkDownIsError) {
  MockPort port;
  SafeArmCore core(port, config());
  ASSERT_TRUE(core.activate());
  port.bus_.link_running = false;
  EXPECT_EQ(core.read(), Result::kError);
}

TEST(SafeArmCore, ControllerErrorEventsAfterActivationAreError) {
  MockPort port;
  port.bus_.error_events = 3;  // before activation: cleared, not a fault
  SafeArmCore core(port, config());
  ASSERT_TRUE(core.activate());
  EXPECT_EQ(core.read(), Result::kOk);
  port.bus_.error_events = 1;
  EXPECT_EQ(core.read(), Result::kError);
}

TEST(SafeArmCore, WriteSendFailureIsError) {
  MockPort port;
  SafeArmCore core(port, config());
  ASSERT_TRUE(core.activate());
  port.send_ok = false;
  EXPECT_EQ(core.write(), Result::kError);
  EXPECT_TRUE(core.faulted());
}

TEST(SafeArmCore, NonFiniteCommandIsErrorAndNothingIsSent) {
  MockPort port;
  SafeArmCore core(port, config());
  ASSERT_TRUE(core.activate());
  const size_t before = port.sent.size();
  core.pos_cmd[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(core.write(), Result::kError);
  EXPECT_EQ(port.sent.size(), before);
}

TEST(SafeArmCore, WriteSendsTheCommandedPosition) {
  MockPort port;
  SafeArmCore core(port, config());
  ASSERT_TRUE(core.activate());
  core.pos_cmd[0] = 0.3;
  EXPECT_EQ(core.write(), Result::kOk);
  EXPECT_DOUBLE_EQ(port.sent.back()[0].q, 0.3);
  EXPECT_DOUBLE_EQ(port.sent.back()[0].kp, 70.0);
}

TEST(SafeArmCore, FaultLatchesEvenIfBusRecovers) {
  MockPort port;
  SafeArmCore core(port, config());
  ASSERT_TRUE(core.activate());
  port.axes_[0].age_s = 1.0;
  EXPECT_EQ(core.read(), Result::kError);
  port.axes_[0].age_s = 0.001;
  EXPECT_EQ(core.read(), Result::kError);
  EXPECT_EQ(core.write(), Result::kError);
  const size_t before = port.sent.size();
  EXPECT_EQ(core.write(), Result::kError);
  EXPECT_EQ(port.sent.size(), before);
  // A new activation clears it, and holds where the arm is.
  EXPECT_TRUE(core.activate());
  EXPECT_FALSE(core.faulted());
  EXPECT_EQ(core.read(), Result::kOk);
}

TEST(SafeArmCore, NothingIsSentWhileNotActive) {
  MockPort port;
  SafeArmCore core(port, config());
  core.pos_cmd[0] = 1.0;
  EXPECT_EQ(core.write(), Result::kOk);
  EXPECT_TRUE(port.sent.empty());
  ASSERT_TRUE(core.activate());
  core.deactivate();
  EXPECT_EQ(port.disables, 1);
  const size_t before = port.sent.size();
  EXPECT_EQ(core.write(), Result::kOk);
  EXPECT_EQ(port.sent.size(), before);
}

TEST(SafeArmCore, IdleReadPublishesStateWithoutFault) {
  MockPort port;
  SafeArmCore core(port, config());
  EXPECT_EQ(core.read(), Result::kOk);
  EXPECT_FALSE(core.faulted());
  port.axes_[0].age_s = 5.0;  // stale while idle: not copied, no fault, no command ever sent
  port.axes_[0].q = 2.0;
  EXPECT_EQ(core.read(), Result::kOk);
  EXPECT_NE(core.pos_state[0], 2.0);
}

TEST(SafeArmCore, GripperVelocityAndEffortAreUnavailableNotZero) {
  MockPort port;
  SafeArmCore core(port, config());
  EXPECT_TRUE(std::isnan(core.vel_state[kArmDof]));
  EXPECT_TRUE(std::isnan(core.tau_state[kArmDof]));
  ASSERT_TRUE(core.activate());
  ASSERT_EQ(core.read(), Result::kOk);
  EXPECT_TRUE(std::isnan(core.vel_state[kArmDof]));
  EXPECT_TRUE(std::isnan(core.tau_state[kArmDof]));
  EXPECT_FALSE(std::isnan(core.vel_state[0]));
}

TEST(SafeArmCore, NoHandMeansSevenJoints) {
  MockPort port(false);
  auto cfg = config();
  cfg.hand = false;
  SafeArmCore core(port, cfg);
  EXPECT_EQ(core.joint_count(), kArmDof);
  ASSERT_TRUE(core.activate());
  EXPECT_EQ(port.sent[0].size(), kArmDof);
}

TEST(SafeArmCore, PortAxisCountMismatchRefusesActivation) {
  MockPort port(false);
  SafeArmCore core(port, config());  // config expects the hand
  EXPECT_FALSE(core.activate());
}

TEST(SafeArmCore, GripperMappings) {
  EXPECT_DOUBLE_EQ(gripper_motor_to_joint("pinch_gripper", 0.7), 0.7);
  EXPECT_DOUBLE_EQ(gripper_joint_to_motor("pinch_gripper", 0.7), 0.7);
  EXPECT_NEAR(gripper_motor_to_joint("parallel_link", -1.0472), 0.044, 1e-12);
  EXPECT_NEAR(gripper_joint_to_motor("parallel_link", 0.044), -1.0472, 1e-12);
}

TEST(SafeArmCore, WrongGainCountThrows) {
  MockPort port;
  auto cfg = config();
  cfg.kp.pop_back();
  EXPECT_THROW(SafeArmCore(port, cfg), std::invalid_argument);
}
