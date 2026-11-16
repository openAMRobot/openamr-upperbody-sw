// SPDX-License-Identifier: MIT
// The emulated motor and bus without a CAN interface: frames go in through ArmEmulator::inject or straight to MotorModel.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include "openamrobot_openarm_emulator/arm_emulator.hpp"

using namespace openarm_emulator;  // NOLINT

namespace
{
MotorConfig j1() {return default_arm_motors("left_")[0];}

/// The control law itself, with the numerical damping treatment off, so that a single step returns the law's value.
MotorOptions explicit_law()
{
  MotorOptions options;
  options.implicit_damping = false;
  return options;
}

Payload mit(const MotorModel & m, double q, double kp, double kd, double tau = 0.0)
{
  return encode_mit({q, 0.0, kp, kd, tau}, m.limits());
}

ConfigFrame config_frame(uint16_t id, uint8_t command, uint8_t rid = 0, uint32_t raw = 0)
{
  ConfigFrame f;
  f.motor_id = id;
  f.command = command;
  f.rid = rid;
  f.raw = raw;
  return f;
}
}  // namespace

TEST(ArmTable, IsTheUpstreamAddressing)
{
  const auto motors = default_arm_motors("right_");
  ASSERT_EQ(motors.size(), 8u);
  const MotorType types[8] = {MotorType::DM8009, MotorType::DM8009, MotorType::DM4340, MotorType::DM4340,
    MotorType::DM4310, MotorType::DM4310, MotorType::DM4310, MotorType::DM4310};
  for (uint32_t i = 0; i < 8; ++i) {
    EXPECT_EQ(motors[i].send_id, 0x01 + i);
    EXPECT_EQ(motors[i].recv_id, 0x11 + i);
    EXPECT_EQ(motors[i].type, types[i]);
  }
  EXPECT_EQ(motors[0].joint, "openarm_right_joint1");
  EXPECT_EQ(motors[6].joint, "openarm_right_joint7");
  EXPECT_EQ(motors[7].joint, "openarm_right_finger_joint1");
}

TEST(MotorModel, StartsDisabledAndAnswersRefresh)
{
  MotorModel m(j1(), MotorOptions());
  EXPECT_FALSE(m.status().enabled);
  EXPECT_EQ(m.step(0.3, 0.0, 0.0, 0.001, 25.6), 0.0);
  EXPECT_EQ(m.on_config_frame(config_frame(1, kCfgRefresh), 0.0).kind, ReplyKind::State);
  const StateFeedback s = decode_state(m.state_frame(), m.limits());
  EXPECT_EQ(s.motor_id, 1);
  EXPECT_EQ(s.code, kCodeDisabled);
  EXPECT_NEAR(s.q, 0.3, 25.0 / 65535);
  EXPECT_EQ(s.t_mos, 25);
}

TEST(MotorModel, EnableMitDisable)
{
  MotorModel m(j1(), explicit_law());
  // A control frame to a disabled motor is answered and ignored.
  EXPECT_EQ(m.on_control_frame(0, mit(m, 1.0, 70.0, 2.0), 0.0).kind, ReplyKind::State);
  EXPECT_EQ(m.step(0.0, 0.0, 0.0, 0.001, 25.6), 0.0);

  EXPECT_EQ(m.on_control_frame(0, encode_special(kCmdEnable), 0.0).kind, ReplyKind::State);
  EXPECT_TRUE(m.status().enabled);
  EXPECT_EQ(decode_state(m.state_frame(), m.limits()).code, kCodeEnabled);
  // Enabled and no command yet: no torque.
  EXPECT_EQ(m.step(0.5, 0.0, 0.0, 0.001, 25.6), 0.0);

  m.on_control_frame(0, mit(m, 0.6, 70.0, 2.0), 0.0);
  // The law uses the values the frame carries. A zero dq or tau is one quantum below zero on the wire (truncation).
  const MitCommand w = decode_mit(mit(m, 0.6, 70.0, 2.0), m.limits());
  EXPECT_LT(w.dq, 0.0);
  EXPECT_GT(w.dq, -90.0 / 4095);
  const auto law = [&w](double q, double dq) {return w.kp * (w.q - q) + w.kd * (w.dq - dq) + w.tau;};
  EXPECT_NEAR(m.step(0.5, 0.0, 0.0, 0.001, 25.6), law(0.5, 0.0), 1e-9);
  EXPECT_NEAR(law(0.5, 0.0), 70.0 * 0.1, 0.1);
  EXPECT_TRUE(m.status().first_mit_valid);
  EXPECT_NEAR(m.status().first_mit.q, 0.6, 1e-3);
  // The setpoint is held between frames.
  EXPECT_NEAR(m.step(0.55, 0.0, 5.0, 0.001, 25.6), law(0.55, 0.0), 1e-9);
  // Feed-forward torque and damping.
  m.on_control_frame(0, encode_mit({0.0, 0.0, 0.0, 2.0, 3.0}, m.limits()), 0.0);
  const MitCommand c = m.status().last_mit;
  EXPECT_NEAR(m.step(0.0, -0.5, 0.0, 0.001, 25.6), c.kd * (c.dq + 0.5) + c.tau, 1e-9);
  EXPECT_NEAR(c.tau, 3.0, 108.0 / 4095);

  EXPECT_EQ(m.on_control_frame(0, encode_special(kCmdDisable), 0.0).kind, ReplyKind::State);
  EXPECT_FALSE(m.status().enabled);
  EXPECT_EQ(m.step(0.0, 0.0, 0.0, 0.001, 25.6), 0.0);
  EXPECT_EQ(decode_state(m.state_frame(), m.limits()).code, kCodeDisabled);
}

TEST(MotorModel, TorqueIsClampedToThePeak)
{
  MotorModel m(j1(), explicit_law());  // DM8009: encoding range 54 N m, peak 40 N m
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.on_control_frame(0, mit(m, 5.0, 500.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(m.step(0.0, 0.0, 0.0, 0.001, 25.6), 40.0);
  EXPECT_DOUBLE_EQ(m.step(10.0, 0.0, 0.0, 0.001, 25.6), -40.0);
  EXPECT_NEAR(decode_state(m.state_frame(), m.limits()).tau, -40.0, 108.0 / 4095);
}

TEST(MotorModel, ParameterReadWriteSave)
{
  MotorModel m(default_arm_motors("left_")[7], MotorOptions());  // gripper, DM4310
  Reply r = m.on_config_frame(config_frame(8, kCfgRead, kRidPMax), 0.0);
  ASSERT_EQ(r.kind, ReplyKind::Param);
  ConfigFrame back = decode_config(r.param);
  EXPECT_EQ(back.motor_id, 8);
  EXPECT_EQ(back.command, kCfgRead);
  EXPECT_EQ(back.rid, kRidPMax);
  EXPECT_FLOAT_EQ(bits_float(back.raw), 12.5f);
  EXPECT_FLOAT_EQ(bits_float(decode_config(m.on_config_frame(config_frame(8, kCfgRead, kRidVMax), 0.0).param).raw), 30.0f);
  EXPECT_FLOAT_EQ(bits_float(decode_config(m.on_config_frame(config_frame(8, kCfgRead, kRidTMax), 0.0).param).raw), 10.0f);
  EXPECT_EQ(decode_config(m.on_config_frame(config_frame(8, kCfgRead, kRidMasterId), 0.0).param).raw, 0x18u);
  EXPECT_EQ(decode_config(m.on_config_frame(config_frame(8, kCfgRead, kRidSlaveId), 0.0).param).raw, 0x08u);
  EXPECT_EQ(decode_config(m.on_config_frame(config_frame(8, kCfgRead, kRidControlMode), 0.0).param).raw, 1u);

  // What OpenArmHW sends for the gripper at init: write CTRL_MODE = MIT. The reply echoes the request.
  r = m.on_config_frame(config_frame(8, kCfgWrite, kRidControlMode, 1), 0.0);
  ASSERT_EQ(r.kind, ReplyKind::Param);
  EXPECT_EQ(r.param, (Payload{0x08, 0x00, 0x55, 0x0A, 0x01, 0x00, 0x00, 0x00}));

  // A write changes RAM only. A power cycle brings the saved value back.
  m.on_config_frame(config_frame(8, kCfgWrite, kRidControlMode, 2), 0.0);
  EXPECT_EQ(m.status().mode, ControlMode::POS_VEL);
  m.set_power(false, 1.0);
  m.set_power(true, 2.0);
  EXPECT_EQ(m.status().mode, ControlMode::MIT);
  m.on_config_frame(config_frame(8, kCfgWrite, kRidControlMode, 2), 0.0);
  r = m.on_config_frame(config_frame(8, kCfgSave), 0.0);
  ASSERT_EQ(r.kind, ReplyKind::Param);
  EXPECT_EQ(r.param[2], kCfgSave);
  EXPECT_EQ(r.param[3], 0x01);
  m.set_power(false, 3.0);
  m.set_power(true, 4.0);
  EXPECT_EQ(m.status().mode, ControlMode::POS_VEL);
}

TEST(MotorModel, PositionVelocityMode)
{
  MotorModel m(default_arm_motors("left_")[4], explicit_law());
  m.on_config_frame(config_frame(5, kCfgWrite, kRidControlMode, 2), 0.0);
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  // An MIT frame in position-velocity mode is answered and not used.
  m.on_control_frame(0, mit(m, 1.0, 100.0, 1.0), 0.0);
  EXPECT_EQ(m.step(0.0, 0.0, 0.0, 0.001, 25.6), 0.0);
  m.on_control_frame(kPosVelOffset, encode_posvel({1.0, 2.0}), 0.0);
  EXPECT_GT(m.step(0.0, 0.0, 0.0, 0.001, 25.6), 0.0);   // behind the target: push forward
  EXPECT_LT(m.step(0.0, 3.0, 0.0, 0.001, 25.6), 0.0);   // faster than the 2 rad/s limit: brake
  EXPECT_LT(m.step(1.5, 0.0, 0.0, 0.001, 25.6), 0.0);   // past the target: push back
  EXPECT_NEAR(m.step(1.0, 0.0, 0.0, 0.001, 25.6), 0.0, 1e-6);
}

TEST(MotorModel, FaultCodesDisableAndLatch)
{
  MotorModel m(j1(), explicit_law());
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.on_control_frame(0, mit(m, 1.0, 70.0, 2.0), 0.0);
  EXPECT_GT(m.step(0.0, 0.0, 0.0, 0.001, 25.6), 0.0);

  // Over-voltage: above the threshold the motor trips 0x8 and lets go.
  EXPECT_EQ(m.step(0.0, 0.0, 0.1, 0.001, 33.0), 0.0);
  EXPECT_EQ(m.status().code, kCodeOverVoltage);
  EXPECT_FALSE(m.status().enabled);
  EXPECT_EQ(decode_state(m.state_frame(), m.limits()).code, kCodeOverVoltage);
  // It still answers, and the supply coming back does not clear it. Enable does not either (an assumption, README).
  EXPECT_EQ(m.on_config_frame(config_frame(1, kCfgRefresh), 0.2).kind, ReplyKind::State);
  m.step(0.0, 0.0, 0.2, 0.001, 25.6);
  m.on_control_frame(0, encode_special(kCmdEnable), 0.3);
  EXPECT_FALSE(m.status().enabled);
  EXPECT_EQ(m.status().code, kCodeOverVoltage);
  // Clear error, then enable.
  m.on_control_frame(0, encode_special(kCmdClearError), 0.4);
  EXPECT_EQ(m.status().code, 0);
  m.on_control_frame(0, encode_special(kCmdEnable), 0.5);
  EXPECT_TRUE(m.status().enabled);

  // Under-voltage.
  m.step(0.0, 0.0, 0.6, 0.001, 12.0);
  EXPECT_EQ(m.status().code, kCodeUnderVoltage);
}

TEST(MotorModel, SupplyRangeOfTheRobotIsNotAFault)
{
  for (double volts : {25.6, 27.0, 29.2}) {
    MotorModel m(j1(), explicit_law());
    m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
    m.on_control_frame(0, mit(m, 1.0, 70.0, 2.0), 0.0);
    EXPECT_GT(m.step(0.0, 0.0, 0.0, 0.001, volts), 0.0) << volts;
    EXPECT_TRUE(m.status().enabled) << volts;
    EXPECT_EQ(decode_state(m.state_frame(), m.limits()).code, kCodeEnabled) << volts;
  }
}

TEST(MotorModel, NoLoadSpeedScalesWithSupply)
{
  // DM8009: 16.755 rad/s at 24 V. Linear model: 17.87 rad/s at 25.6 V, 20.39 rad/s at 29.2 V.
  for (double volts : {24.0, 25.6, 29.2}) {
    MotorModel m(j1(), explicit_law());
    m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
    m.on_control_frame(0, encode_mit({0.0, 0.0, 0.0, 0.0, 40.0}, m.limits()), 0.0);
    const double no_load = 16.755 * volts / 24.0;
    EXPECT_NEAR(m.step(0.0, 0.0, 0.0, 0.001, volts), 40.0, 0.02) << volts;
    EXPECT_NEAR(m.step(0.0, 0.5 * no_load, 0.0, 0.001, volts), 20.0, 0.02) << volts;
    EXPECT_NEAR(m.step(0.0, no_load, 0.0, 0.001, volts), 0.0, 1e-6) << volts;
    EXPECT_NEAR(m.step(0.0, 1.2 * no_load, 0.0, 0.001, volts), 0.0, 1e-6) << volts;
    // Braking is not limited by this model.
    EXPECT_NEAR(m.step(0.0, -no_load, 0.0, 0.001, volts), 40.0, 0.02) << volts;
  }
}

TEST(MotorModel, OverTemperatureRampsThenTrips)
{
  MotorModel m(j1(), MotorOptions());
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.set_temperature_target(130.0, 25.0, 100.0);  // 100 C/s
  for (int i = 0; i < 500; ++i) {m.step(0.0, 0.0, i * 0.001, 0.001, 25.6);}
  EXPECT_NEAR(m.status().t_mos, 75.0, 0.5);
  EXPECT_TRUE(m.status().enabled);
  EXPECT_EQ(decode_state(m.state_frame(), m.limits()).t_mos, 75);
  for (int i = 500; i < 1100; ++i) {m.step(0.0, 0.0, i * 0.001, 0.001, 25.6);}
  EXPECT_EQ(m.status().code, kCodeMosOverTemperature);
  EXPECT_FALSE(m.status().enabled);
  EXPECT_EQ(decode_state(m.state_frame(), m.limits()).t_mos, 130);

  MotorModel rotor(j1(), MotorOptions());
  rotor.set_temperature_target(25.0, 140.0, 0.0);  // rate 0: a step
  rotor.step(0.0, 0.0, 0.0, 0.001, 25.6);
  EXPECT_EQ(rotor.status().code, kCodeCoilOverTemperature);
  EXPECT_EQ(decode_state(rotor.state_frame(), rotor.limits()).t_rotor, 140);
}

TEST(MotorModel, CommunicationTimeoutIsOffByDefault)
{
  MotorModel m(j1(), explicit_law());
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.on_control_frame(0, mit(m, 1.0, 70.0, 2.0), 0.0);
  EXPECT_GT(m.step(0.0, 0.0, 100.0, 0.001, 25.6), 0.0);  // 100 s of silence: it still holds the setpoint
  EXPECT_TRUE(m.status().enabled);

  MotorOptions options = explicit_law();
  options.can_timeout_s = 0.1;
  MotorModel t(j1(), options);
  t.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  t.on_control_frame(0, mit(t, 1.0, 70.0, 2.0), 0.0);
  EXPECT_GT(t.step(0.0, 0.0, 0.05, 0.001, 25.6), 0.0);
  EXPECT_EQ(t.step(0.0, 0.0, 0.2, 0.001, 25.6), 0.0);
  EXPECT_EQ(t.status().code, kCodeCommunicationLost);
}

TEST(MotorModel, PowerCutIsSilentAndComesBackDisabled)
{
  MotorModel m(j1(), explicit_law());
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.on_control_frame(0, mit(m, 1.0, 70.0, 2.0), 0.0);
  m.set_power(false, 1.0);
  EXPECT_EQ(m.on_config_frame(config_frame(1, kCfgRefresh), 1.1).kind, ReplyKind::None);
  EXPECT_EQ(m.on_control_frame(0, mit(m, 1.0, 70.0, 2.0), 1.1).kind, ReplyKind::None);
  EXPECT_EQ(m.on_control_frame(0, encode_special(kCmdEnable), 1.1).kind, ReplyKind::None);
  EXPECT_EQ(m.step(0.2, -1.0, 1.2, 0.001, 0.0), 0.0);
  m.set_power(true, 2.0);
  EXPECT_FALSE(m.status().enabled);
  EXPECT_EQ(m.on_config_frame(config_frame(1, kCfgRefresh), 2.1).kind, ReplyKind::State);
  m.step(-0.7, 0.0, 2.1, 0.001, 25.6);
  EXPECT_NEAR(decode_state(m.state_frame(), m.limits()).q, -0.7, 25.0 / 65535);  // absolute encoder: position is not lost
  // MIT frames to the disabled motor do nothing until an enable arrives.
  m.on_control_frame(0, mit(m, 1.0, 70.0, 2.0), 2.2);
  EXPECT_EQ(m.step(-0.7, 0.0, 2.2, 0.001, 25.6), 0.0);
}

// ---- Current and temperature (README, "Current and temperature") ----

TEST(MotorElectrical, ElectricalDefaultsAreTheDatasheetValues)
{
  // config/motor_electrical.yaml: gear ratio and phase resistance from the datasheets, kt = rated torque / (gear ratio *
  // rated current). DM8009: 9, 20 N m, 20 A, 0.09 ohm. DM4340: 40, 9 N m, 2.5 A, 0.76 ohm. DM4310: 10, 3 N m, 2.5 A,
  // 0.65 ohm. Thresholds and thermal values are unknown: off.
  const auto motors = default_arm_motors("left_");
  const double gear[8] = {9, 9, 40, 40, 10, 10, 10, 10};
  const double kt[8] = {20.0 / 180.0, 20.0 / 180.0, 0.09, 0.09, 0.12, 0.12, 0.12, 0.12};
  const double r[8] = {0.09, 0.09, 0.76, 0.76, 0.65, 0.65, 0.65, 0.65};
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_DOUBLE_EQ(motors[i].gear_ratio, gear[i]) << i;
    EXPECT_NEAR(motors[i].kt, kt[i], 1e-12) << i;
    EXPECT_DOUBLE_EQ(motors[i].phase_resistance, r[i]) << i;
    EXPECT_EQ(motors[i].over_current_a, 0.0) << i;
    EXPECT_EQ(motors[i].thermal_resistance, 0.0) << i;
    EXPECT_EQ(motors[i].thermal_time_constant, 0.0) << i;
  }
  // The rated point: rated torque at the output is rated current.
  EXPECT_NEAR(20.0 / (motors[0].kt * motors[0].gear_ratio), 20.0, 1e-9);
  EXPECT_NEAR(9.0 / (motors[2].kt * motors[2].gear_ratio), 2.5, 1e-9);
  EXPECT_NEAR(3.0 / (motors[4].kt * motors[4].gear_ratio), 2.5, 1e-9);
  // Register 20 (Gr) reads the gear ratio.
  MotorModel m(motors[2], MotorOptions());
  EXPECT_FLOAT_EQ(bits_float(m.param_raw(kRidGearRatio)), 40.0f);
}

TEST(MotorElectrical, CurrentIsTorqueOverKtAndGearRatio)
{
  const auto motors = default_arm_motors("left_");
  for (size_t i : {0u, 2u, 4u}) {
    MotorModel m(motors[i], explicit_law());
    m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
    for (double tau : {-2.0, 0.0, 1.5, 3.0}) {
      m.on_control_frame(0, encode_mit({0.0, 0.0, 0.0, 0.0, tau}, m.limits()), 0.0);
      const double applied = m.step(0.0, 0.0, 0.0, 0.001, 25.6);
      const double current = applied / (motors[i].kt * motors[i].gear_ratio);
      EXPECT_NEAR(m.status().current, current, 1e-9) << i << " " << tau;
      EXPECT_NEAR(m.status().copper_loss, 1.5 * motors[i].phase_resistance * current * current, 1e-9) << i << " " << tau;
    }
  }
  // DM4310 at its rated 3 N m: 2.5 A, 1.5 * 0.65 * 2.5^2 = 6.09 W.
  MotorModel wrist(motors[4], explicit_law());
  wrist.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  wrist.on_control_frame(0, encode_mit({0.0, 0.0, 0.0, 0.0, 3.0}, wrist.limits()), 0.0);
  const double applied = wrist.step(0.0, 0.0, 0.0, 0.001, 25.6);
  EXPECT_NEAR(wrist.status().current, 2.5 * applied / 3.0, 1e-9);
  EXPECT_NEAR(applied, 3.0, 0.01);  // the 12-bit torque field in the MIT frame
  // Disabled or unpowered: no current.
  wrist.on_control_frame(0, encode_special(kCmdDisable), 0.1);
  wrist.step(0.0, 0.0, 0.1, 0.001, 25.6);
  EXPECT_EQ(wrist.status().current, 0.0);
  EXPECT_EQ(wrist.status().copper_loss, 0.0);
}

TEST(MotorElectrical, OverCurrentTripsWhenAThresholdIsSet)
{
  MotorConfig c = default_arm_motors("left_")[4];  // DM4310, 1.2 N m/A at the output
  MotorModel off(c, explicit_law());
  off.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  off.on_control_frame(0, encode_mit({0.0, 0.0, 0.0, 0.0, 6.0}, off.limits()), 0.0);
  EXPECT_NEAR(off.step(0.0, 0.0, 0.0, 0.001, 25.6), 6.0, 0.01);  // 5 A: no threshold, no trip
  EXPECT_TRUE(off.status().enabled);

  c.over_current_a = 4.0;  // a test value. The real threshold (register 3) is unknown
  MotorModel m(c, explicit_law());
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.on_control_frame(0, encode_mit({0.0, 0.0, 0.0, 0.0, 4.5}, m.limits()), 0.0);
  EXPECT_NEAR(m.step(0.0, 0.0, 0.0, 0.001, 25.6), 4.5, 0.01);   // 3.75 A
  EXPECT_TRUE(m.status().enabled);
  m.on_control_frame(0, encode_mit({0.0, 0.0, 0.0, 0.0, 6.0}, m.limits()), 0.001);
  EXPECT_EQ(m.step(0.0, 0.0, 0.001, 0.001, 25.6), 0.0);         // 5 A: trip, no torque
  EXPECT_EQ(m.status().code, kCodeOverCurrent);
  EXPECT_FALSE(m.status().enabled);
  EXPECT_EQ(m.status().current, 0.0);
  const Payload frame = m.state_frame();
  EXPECT_EQ(frame[0], 0xA5);  // code A in the high nibble, motor id 5 in the low nibble
}

TEST(MotorElectrical, ThermalModelIsOffByDefault)
{
  MotorModel m(default_arm_motors("left_")[4], explicit_law());
  EXPECT_FALSE(m.thermal_active());
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.on_control_frame(0, encode_mit({0.0, 0.0, 0.0, 0.0, 7.0}, m.limits()), 0.0);
  for (int i = 0; i < 6000; ++i) {m.step(0.0, 0.0, i * 0.01, 0.01, 25.6);}  // 60 s at peak torque
  EXPECT_GT(m.status().copper_loss, 0.0);
  EXPECT_DOUBLE_EQ(m.status().t_rotor, 25.0);
  EXPECT_TRUE(m.status().enabled);

  // Switched on without the two thermal values (no source gives them): still off.
  MotorOptions options = explicit_law();
  options.thermal_model = true;
  MotorModel no_values(default_arm_motors("left_")[4], options);
  EXPECT_FALSE(no_values.thermal_active());
}

TEST(MotorElectrical, ThermalModelRisesToTheSteadyStateWithTestParameters)
{
  // Test values, not motor data: 2 K/W, 10 s. DM4310 at 3 N m: 2.5 A, 6.09 W, steady rise 12.19 K.
  MotorConfig c = default_arm_motors("left_")[4];
  c.thermal_resistance = 2.0;
  c.thermal_time_constant = 10.0;
  MotorOptions options = explicit_law();
  options.thermal_model = true;
  MotorModel m(c, options);
  ASSERT_TRUE(m.thermal_active());
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.on_control_frame(0, encode_mit({0.0, 0.0, 0.0, 0.0, 3.0}, m.limits()), 0.0);
  m.step(0.0, 0.0, 0.0, 0.01, 25.6);
  const double power = m.status().copper_loss;
  ASSERT_NEAR(power, 1.5 * 0.65 * 2.5 * 2.5, 0.05);
  const double rise = power * 2.0;
  for (int i = 1; i <= 1000; ++i) {m.step(0.0, 0.0, i * 0.01, 0.01, 25.6);}  // one time constant
  EXPECT_NEAR(m.status().t_rotor - 25.0, rise * (1.0 - std::exp(-1.0)), 0.01);
  for (int i = 1001; i <= 6000; ++i) {m.step(0.0, 0.0, i * 0.01, 0.01, 25.6);}  // six time constants
  EXPECT_NEAR(m.status().t_rotor - 25.0, rise, 0.05);
  EXPECT_EQ(decode_state(m.state_frame(), m.limits()).t_rotor, static_cast<uint8_t>(std::lround(25.0 + rise)));  // D7
  EXPECT_DOUBLE_EQ(m.status().t_mos, 25.0);  // no MOSFET model
  // Torque off: it cools with the same time constant.
  m.on_control_frame(0, encode_special(kCmdDisable), 60.0);
  for (int i = 6001; i <= 7000; ++i) {m.step(0.0, 0.0, i * 0.01, 0.01, 25.6);}
  EXPECT_NEAR(m.status().t_rotor - 25.0, rise * std::exp(-1.0), 0.05);
}

TEST(MotorElectrical, ThermalModelTripsCoilOverTemperature)
{
  // Test values: 20 K/W, 1 s. At the 7 N m peak the steady state is far above the 120 C threshold.
  MotorConfig c = default_arm_motors("left_")[4];
  c.thermal_resistance = 20.0;
  c.thermal_time_constant = 1.0;
  MotorOptions options = explicit_law();
  options.thermal_model = true;
  MotorModel m(c, options);
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.on_control_frame(0, encode_mit({0.0, 0.0, 0.0, 0.0, 7.0}, m.limits()), 0.0);
  int steps = 0;
  while (m.status().enabled && steps < 10000) {
    m.step(0.0, 0.0, steps * 0.001, 0.001, 25.6);
    steps++;
  }
  EXPECT_EQ(m.status().code, kCodeCoilOverTemperature);
  EXPECT_GT(m.status().t_rotor, 120.0);
  EXPECT_LT(m.status().t_rotor, 121.0);
  const StateFeedback s = decode_state(m.state_frame(), m.limits());
  EXPECT_EQ(s.code, kCodeCoilOverTemperature);
  EXPECT_GE(s.t_rotor, 120);
  EXPECT_EQ(m.status().tau, 0.0);
}

TEST(MotorFaults, EveryFaultCodeIsInTheHighNibbleOfD0)
{
  // Datasheet feedback frame: D0 = ID | ERR << 4. Codes 8 to E.
  for (uint8_t code = kCodeOverVoltage; code <= kCodeOverload; ++code) {
    MotorModel m(default_arm_motors("left_")[6], explicit_law());  // J7, id 7
    m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
    m.force_code(code);
    m.step(0.0, 0.0, 0.0, 0.001, 25.6);
    const Payload frame = m.state_frame();
    EXPECT_EQ(frame[0], static_cast<uint8_t>((code << 4) | 0x07)) << int(code);
    EXPECT_EQ(decode_state(frame, m.limits()).code, code) << int(code);
    EXPECT_FALSE(m.status().enabled) << int(code);
    EXPECT_EQ(m.on_config_frame(config_frame(7, kCfgRefresh), 0.0).kind, ReplyKind::State) << int(code);
  }
  // No fault: 0 disabled, 1 enabled.
  MotorModel ok(default_arm_motors("left_")[6], explicit_law());
  EXPECT_EQ(ok.state_frame()[0], 0x07);
  ok.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  EXPECT_EQ(ok.state_frame()[0], 0x17);
}

TEST(MotorFaults, TimeoutRegisterWriteTakesEffect)
{
  MotorModel m(j1(), explicit_law());
  EXPECT_EQ(m.timeout_s(), 0.0);
  // The host writes TIMEOUT = 2000 counts: 0.1 s.
  const Reply echo = m.on_config_frame(config_frame(1, kCfgWrite, kRidTimeout, 2000), 0.0);
  EXPECT_EQ(echo.kind, ReplyKind::Param);
  EXPECT_NEAR(m.timeout_s(), 0.1, 1e-12);
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.on_control_frame(0, mit(m, 1.0, 70.0, 2.0), 0.0);
  EXPECT_GT(m.step(0.0, 0.0, 0.09, 0.001, 25.6), 0.0);
  EXPECT_EQ(m.step(0.0, 0.0, 0.11, 0.001, 25.6), 0.0);
  EXPECT_EQ(m.status().code, kCodeCommunicationLost);
  EXPECT_EQ(m.state_frame()[0], 0xD1);
  EXPECT_EQ(m.on_config_frame(config_frame(1, kCfgRefresh), 0.2).kind, ReplyKind::State);  // it still answers
  // Not saved: a power cycle restores the flash value (off), and the fault is gone with the power.
  m.set_power(false, 1.0);
  m.set_power(true, 2.0);
  EXPECT_EQ(m.timeout_s(), 0.0);
  EXPECT_EQ(m.status().code, 0);
  // Written and saved (disabled): kept across a power cycle.
  m.on_config_frame(config_frame(1, kCfgWrite, kRidTimeout, timeout_counts(0.05)), 3.0);
  m.on_config_frame(config_frame(1, kCfgSave), 3.0);
  m.set_power(false, 4.0);
  m.set_power(true, 5.0);
  EXPECT_NEAR(m.timeout_s(), 0.05, 1e-12);
  // The maker's tool path (set_timeout) writes RAM and flash.
  m.set_timeout(0.0);
  EXPECT_EQ(m.param_raw(kRidTimeout), 0u);
}

TEST(MotorModel, SilentMotorRunsButDoesNotAnswer)
{
  MotorModel m(j1(), explicit_law());
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.set_silent(true);
  EXPECT_EQ(m.on_control_frame(0, mit(m, 1.0, 70.0, 2.0), 0.0).kind, ReplyKind::None);
  EXPECT_EQ(m.on_config_frame(config_frame(1, kCfgRefresh), 0.0).kind, ReplyKind::None);
  EXPECT_GT(m.step(0.0, 0.0, 0.0, 0.001, 25.6), 0.0);
}

namespace
{
/// A motor on one rigid inertia with a constant load, explicit Euler at dt, as a physics engine steps a joint.
struct OneJoint
{
  double inertia, load, q{0.0}, dq{0.0}, max_speed{0.0};
  int sign_changes{0};
  void run(MotorModel & m, double seconds, double dt = 0.001)
  {
    double last = 0.0;
    for (int i = 0; i < static_cast<int>(seconds / dt); ++i) {
      const double tau = m.step(q, dq, i * dt, dt, 25.6);
      dq += dt * (tau + load) / inertia;
      q += dt * dq;
      max_speed = std::max(max_speed, std::fabs(dq));
      if (dq * last < 0.0) {sign_changes++;}
      last = dq;
    }
  }
};
}  // namespace

TEST(MotorDamping, ExplicitLawChattersOnALightLinkAtOneMillisecond)
{
  // A wrist link as Gazebo steps it: 1.4e-4 kg m^2, kp 10, kd 0.7. kd * dt / inertia = 5, above the limit of 2.
  MotorModel m(default_arm_motors("left_")[4], explicit_law());
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.on_control_frame(0, mit(m, 0.3, 10.0, 0.7), 0.0);
  OneJoint joint{1.4e-4, 0.0};
  joint.run(m, 1.0);
  EXPECT_GT(joint.sign_changes, 400);   // the velocity flips sign nearly every step
  EXPECT_GT(joint.max_speed, 20.0);     // at the torque limit: 7 N m * 1 ms / 1.4e-4 = 50 rad/s per step
}

TEST(MotorDamping, ImplicitDampingIsStableAndKeepsTheSlowResponse)
{
  MotorModel m(default_arm_motors("left_")[4], MotorOptions());
  m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
  m.on_control_frame(0, mit(m, 0.3, 10.0, 0.7), 0.0);
  OneJoint joint{1.4e-4, 0.0};
  // The law is heavily overdamped: the position follows with the time constant kd / kp = 70 ms.
  joint.run(m, 0.07);
  EXPECT_NEAR(joint.q, 0.3 * (1.0 - std::exp(-1.0)), 0.03);
  joint.run(m, 0.93);
  EXPECT_NEAR(joint.q, 0.3, 2e-3);
  EXPECT_NEAR(joint.dq, 0.0, 0.01);
  EXPECT_LT(joint.sign_changes, 5);
  EXPECT_LT(joint.max_speed, 0.3 * 10.0 / 0.7 * 2.0);   // near the law's own kp / kd * error, not 21 rad/s in one step
}

TEST(MotorDamping, StaticTorqueIsTheLawsOwn)
{
  // A loaded shoulder: 0.5 kg m^2, 6 N m of load. At rest the torque is kp * error, whatever the damping treatment.
  for (bool implicit : {false, true}) {
    MotorOptions options;
    options.implicit_damping = implicit;
    MotorModel m(j1(), options);
    m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
    m.on_control_frame(0, mit(m, 0.2, 70.0, 2.75), 0.0);
    OneJoint joint{0.5, -6.0};
    joint.run(m, 4.0);
    const MitCommand c = m.status().last_mit;
    EXPECT_NEAR(joint.dq, 0.0, 1e-3) << implicit;
    EXPECT_NEAR(m.status().tau, 6.0, 0.01) << implicit;
    EXPECT_NEAR(joint.q, c.q - (6.0 - c.kd * c.dq - c.tau) / c.kp, 1e-3) << implicit;   // the sag: load / kp
  }
}

TEST(MotorDamping, AHeavyJointMovesTheSameWithAndWithoutIt)
{
  // kd * dt / inertia = 0.0055: the explicit step is fine here, and the implicit one must not change the motion.
  double q_at[2];
  for (bool implicit : {false, true}) {
    MotorOptions options;
    options.implicit_damping = implicit;
    MotorModel m(j1(), options);
    m.on_control_frame(0, encode_special(kCmdEnable), 0.0);
    m.on_control_frame(0, mit(m, 0.5, 70.0, 2.75), 0.0);
    OneJoint joint{0.5, 0.0};
    joint.run(m, 0.15);
    q_at[implicit] = joint.q;
  }
  EXPECT_GT(q_at[0], 0.2);
  EXPECT_NEAR(q_at[1], q_at[0], 0.02);
}

TEST(BusTiming, FrameTimesAreAboutTheAuditEstimate)
{
  BusTimingConfig c;  // 1 Mbit/s and 5 Mbit/s
  EXPECT_NEAR(frame_time_s(c, true, true), 52e-6, 6e-6);     // the audit estimates about 50 us
  EXPECT_NEAR(frame_time_s(c, false, false), 121e-6, 10e-6);  // classic CAN, 8 bytes, 1 Mbit/s
  EXPECT_GT(frame_time_s(c, true, false), frame_time_s(c, true, true));
}

TEST(BusTiming, OneFrameAtATimeLowestIdFirst)
{
  BusTimingConfig c;
  BusTimingModel bus(c);
  const double t = frame_time_s(c, true, true);
  // Three frames ready together: 0x7FF, 0x12, 0x11.
  bus.push({0.0, 0x7FF, true, true, false, 1, 0.0});
  bus.push({0.0, 0x12, true, true, true, 2, 0.0});
  bus.push({0.0, 0x11, true, true, true, 3, 0.0});
  EXPECT_TRUE(bus.advance(0.5 * t).empty());
  std::vector<BusTimingModel::Frame> done;
  for (auto next = bus.advance(10 * t); !next.empty(); next = bus.advance(10 * t)) {done.push_back(next[0]);}
  ASSERT_EQ(done.size(), 3u);
  EXPECT_EQ(done[0].token, 3u);
  EXPECT_EQ(done[1].token, 2u);
  EXPECT_EQ(done[2].token, 1u);
  EXPECT_NEAR(done[0].end, t, 1e-12);
  EXPECT_NEAR(done[1].end, 2 * t, 1e-12);
  EXPECT_NEAR(done[2].end, 3 * t, 1e-12);
  EXPECT_EQ(bus.pending(), 0u);
}

TEST(BusTiming, AFrameThatBecomesReadyLaterDoesNotJumpAStartedOne)
{
  BusTimingConfig c;
  BusTimingModel bus(c);
  const double t = frame_time_s(c, true, true);
  bus.push({0.0, 0x7FF, true, true, false, 1, 0.0});
  EXPECT_TRUE(bus.advance(0.1 * t).empty());  // 0x7FF is on the wire
  bus.push({0.2 * t, 0x01, true, true, false, 2, 0.0});
  std::vector<BusTimingModel::Frame> done;
  for (auto next = bus.advance(5 * t); !next.empty(); next = bus.advance(5 * t)) {done.push_back(next[0]);}
  ASSERT_EQ(done.size(), 2u);
  EXPECT_EQ(done[0].token, 1u);
  EXPECT_NEAR(done[1].end, 2 * t, 1e-12);
  EXPECT_TRUE(std::isinf(bus.next_wake()));
}

TEST(ArmEmulator, RefreshCycleIsSerialisedOnTheModelledBus)
{
  ArmEmulatorConfig config;
  config.motors = default_arm_motors("left_");
  ArmEmulator arm(config);
  const double t = frame_time_s(config.timing, true, true);
  // What OpenArmHW::read() sends: eight refresh frames, back to back.
  std::vector<ArmEmulator::WireFrame> replies;
  for (uint16_t id = 1; id <= 8; ++id) {
    const auto got = arm.inject({kConfigCanId, encode_config(config_frame(id, kCfgRefresh)), true, 0.0}, 0.0, id == 8 ? 1.0 : 0.0);
    replies.insert(replies.end(), got.begin(), got.end());
  }
  ASSERT_EQ(replies.size(), 8u);
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_EQ(replies[i].can_id, 0x11 + i);       // in id order
    EXPECT_EQ(replies[i].data[0] & 0x0F, static_cast<int>(i + 1));
    EXPECT_TRUE(replies[i].fd);
  }
  // A reply (0x11..0x18) wins arbitration against the refresh frames still queued (0x7FF), so the first answer is on
  // the wire after two frame times, and the whole exchange takes sixteen.
  EXPECT_NEAR(replies[0].time, 2 * t, 1e-9);
  EXPECT_NEAR(replies[7].time, 16 * t, 1e-9);
  EXPECT_GT(replies[7].time, 500e-6);  // later than the 500 us read() waits for the first frame
}

TEST(ArmEmulator, TimingModelOffAnswersAtOnce)
{
  ArmEmulatorConfig config;
  config.motors = default_arm_motors("left_");
  config.timing.enabled = false;
  ArmEmulator arm(config);
  const auto got = arm.inject({kConfigCanId, encode_config(config_frame(3, kCfgRefresh)), false, 0.0}, 2.0, 2.0);
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].can_id, 0x13u);
  EXPECT_FALSE(got[0].fd);  // a classic request gets a classic reply
  EXPECT_DOUBLE_EQ(got[0].time, 2.0);
}

TEST(ArmEmulator, FaultsAtArmLevel)
{
  ArmEmulatorConfig config;
  config.motors = default_arm_motors("left_");
  config.timing.enabled = false;
  ArmEmulator arm(config);
  const auto refresh = [&](uint16_t id) {
      return arm.inject({kConfigCanId, encode_config(config_frame(id, kCfgRefresh)), true, 0.0}, 0.0, 0.0).size();
    };
  EXPECT_EQ(refresh(1), 1u);
  EXPECT_EQ(refresh(9), 0u);  // no such motor
  EXPECT_EQ(arm.stats().rx_unknown, 1u);

  arm.set_motor_silent(2, true);  // single-motor timeout
  EXPECT_EQ(refresh(3), 0u);
  EXPECT_EQ(refresh(4), 1u);
  arm.set_motor_silent(2, false);

  arm.set_can_loss(true);  // nothing reaches the motors, nothing comes back
  const uint64_t before = arm.status(0).refresh_frames;
  EXPECT_EQ(refresh(1), 0u);
  EXPECT_EQ(arm.status(0).refresh_frames, before);
  arm.set_can_loss(false);
  EXPECT_EQ(refresh(1), 1u);

  arm.set_power(false);  // E-stop power cut: every motor silent
  for (uint16_t id = 1; id <= 8; ++id) {EXPECT_EQ(refresh(id), 0u);}
  arm.set_power(true);
  for (uint16_t id = 1; id <= 8; ++id) {EXPECT_EQ(refresh(id), 1u);}
  for (size_t i = 0; i < 8; ++i) {EXPECT_FALSE(arm.status(i).enabled);}

  arm.force_code(5, kCodeOverload);
  std::vector<double> q(8, 0.0), dq(8, 0.0), tau;
  arm.step(q, dq, 0.001, tau);
  const auto got = arm.inject({kConfigCanId, encode_config(config_frame(6, kCfgRefresh)), true, 0.0}, 0.0, 0.0);
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].data[0], 0xE6);

  arm.set_supply_voltage(29.2);
  EXPECT_DOUBLE_EQ(arm.supply_voltage(), 29.2);
}

TEST(ArmEmulator, PowerCutSequenceOnTheWire)
{
  ArmEmulatorConfig config;
  config.motors = default_arm_motors("left_");
  config.timing.enabled = false;
  ArmEmulator arm(config);
  const auto send = [&](uint32_t id, const Payload & data) {return arm.inject({id, data, true, 0.0}, 0.0, 0.0);};
  const MotorModel probe(config.motors[1], MotorOptions());
  const Payload hold = encode_mit({1.0, 0.0, 70.0, 2.5, 0.0}, probe.limits());
  std::vector<double> q(8, 0.0), dq(8, 0.0), tau;

  // Running: J2 enabled and holding a setpoint.
  auto got = send(0x02, encode_special(kCmdEnable));
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].data[0], 0x12);
  send(0x02, hold);
  arm.step(q, dq, 0.001, tau);
  EXPECT_GT(tau[1], 0.0);
  EXPECT_GT(arm.status(1).current, 0.0);

  // Power cut: no motor answers anything, no torque, no current.
  arm.set_power(false);
  for (uint16_t id = 1; id <= 8; ++id) {
    EXPECT_TRUE(send(kConfigCanId, encode_config(config_frame(id, kCfgRefresh))).empty()) << id;
    EXPECT_TRUE(send(id, hold).empty()) << id;
  }
  arm.step(q, dq, 0.001, tau);
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_EQ(tau[i], 0.0) << i;
    EXPECT_EQ(arm.status(i).current, 0.0) << i;
  }

  // Supply back: every motor answers, disabled (code 0), and ignores MIT frames until an enable arrives.
  arm.set_power(true);
  for (uint16_t id = 1; id <= 8; ++id) {
    got = send(kConfigCanId, encode_config(config_frame(id, kCfgRefresh)));
    ASSERT_EQ(got.size(), 1u) << id;
    EXPECT_EQ(got[0].data[0], id) << id;
  }
  got = send(0x02, hold);
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].data[0], 0x02);
  arm.step(q, dq, 0.001, tau);
  EXPECT_EQ(tau[1], 0.0);
  send(0x02, encode_special(kCmdEnable));
  got = send(0x02, hold);
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].data[0], 0x12);
  arm.step(q, dq, 0.001, tau);
  EXPECT_GT(tau[1], 0.0);
}

TEST(ArmEmulator, TimeoutRegisterAndCanLossGiveCodeD)
{
  ArmEmulatorConfig config;
  config.motors = default_arm_motors("left_");
  config.timing.enabled = false;
  ArmEmulator arm(config);
  arm.set_timeout(0.05);
  EXPECT_EQ(arm.status(0).control_frames, 0u);
  const double t0 = ArmEmulator::clock_now();
  arm.inject({0x01, encode_special(kCmdEnable), true, 0.0}, t0, t0);
  std::vector<double> q(8, 0.0), dq(8, 0.0), tau;
  arm.step(q, dq, 0.001, tau);
  EXPECT_TRUE(arm.status(0).enabled);
  arm.set_can_loss(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  arm.step(q, dq, 0.001, tau);
  EXPECT_EQ(arm.status(0).code, kCodeCommunicationLost);
  EXPECT_FALSE(arm.status(1).enabled);  // never enabled: no timeout, no code
  EXPECT_EQ(arm.status(1).code, 0);
  arm.set_can_loss(false);
  const double t1 = ArmEmulator::clock_now();
  const auto got = arm.inject({kConfigCanId, encode_config(config_frame(1, kCfgRefresh)), true, 0.0}, t1, t1);
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].data[0], 0xD1);
}
