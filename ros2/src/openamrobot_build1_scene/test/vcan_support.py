# SPDX-License-Identifier: MIT
"""Whether the five virtual CAN interfaces can be had on this machine. BLOCKED is not a pass."""
import os
import subprocess

from conftest import package_path
from openamrobot_build1_scene import candump
from openamrobot_build1_scene.launch_support import ALL_BUSES, ARM_BUSES, BASE_BUSES, CAN_FD_MTU


def blocked_reason():
    """None when all five interfaces are up with the right MTU, else a text that starts with BLOCKED."""
    def missing():
        return [bus for bus in ALL_BUSES if candump.interface_mtu(bus) is None]

    def wrong():
        return [bus for bus in ALL_BUSES if candump.interface_mtu(bus) != (CAN_FD_MTU if bus in ARM_BUSES.values() else candump.CAN_MTU)]
    if missing() or wrong():
        done = subprocess.run(["sh", package_path("scripts", "setup_vcan_all.sh")], capture_output=True, text=True)
        if missing():
            detail = (done.stdout + done.stderr).strip().splitlines()
            return ("BLOCKED: no virtual CAN on this machine: " + ", ".join(missing()) + " missing. "
                    + (detail[-1] if detail else "") + f" (kernel {os.uname().release})")
    classic = [bus for bus in ARM_BUSES.values() if candump.interface_mtu(bus) != CAN_FD_MTU]
    if classic:
        return f"BLOCKED: {', '.join(classic)} have no CAN FD (MTU is not {CAN_FD_MTU}). The arm buses are CAN FD"
    wide = [bus for bus in BASE_BUSES if candump.interface_mtu(bus) != candump.CAN_MTU]
    if wide:
        return f"BLOCKED: {', '.join(wide)} are not classic CAN (MTU is not {candump.CAN_MTU}). Run setup_vcan_all.sh"
    try:
        candump.open_socket(ALL_BUSES[0]).close()
    except OSError as error:
        return f"BLOCKED: cannot open a raw CAN socket: {error}"
    return None
