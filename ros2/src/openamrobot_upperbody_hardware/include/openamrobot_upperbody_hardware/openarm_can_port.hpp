// SPDX-License-Identifier: MIT
// ArmPort on top of enactic/openarm_can (the pinned version has per-motor link stats and a latched bus status).
// Built only when OpenArmCAN is found; the core and its tests do not need it.
#pragma once

#include <memory>
#include <string>

#include <openarm/can/socket/openarm.hpp>

#include "openamrobot_upperbody_hardware/arm_port.hpp"

namespace openamrobot_upperbody_hardware {

class OpenArmCanPort : public ArmPort {
 public:
  OpenArmCanPort(const std::string& can_interface, bool can_fd, bool hand);

  void request_state() override;
  std::vector<AxisState> axes() const override;
  BusReport bus() const override;
  void clear_bus_counters() override;
  bool enable() override;
  bool send(const std::vector<MitCommand>& commands) override;
  void disable() override;

  // Called by the hardware interface in on_configure: refresh and receive once so the first read has data.
  void prime();

 private:
  bool hand_;
  std::unique_ptr<openarm::can::socket::OpenArm> arm_;
};

}  // namespace openamrobot_upperbody_hardware
