# SPDX-License-Identifier: MIT
"""Decode the frames of the five Build 1 virtual CAN buses into engineering values. No ROS import, nothing sent.

Every layout here is copied from the code that produces or consumes the frame in the pinned simulation stack, never
from a datasheet: docs/demo/can_signal_paths.md names the file and commit of each one. The devices on these buses are
emulators and host stand-ins (see that table); decoding a frame says what the frame carries, not that a real device
would send it.

  vcan_drive  ZLAC8015D emulator, CANopen node 1 (openamrobot_sil zlac_gazebo.py, vendor zlac8015d_emulator.py).
              The STM32 slot is the host stand-in (stm32_host.py DriveHost): expedited SDO on 601h/581h.
  vcan_aux    CTL-017 emulator, CANopen node 0x10 (vendor ctl017_emulator.py, CTL-017 v1 proposal), the host
              stand-in's core heartbeat 701h and the SIL lift permit PDO 410h (sil.yaml, SIL test layout).
  vcan_bms    Daly BMS emulator, 29-bit IDs (daly_bms.py; protocol unconfirmed for the delivered unit).
  vcan_arm_l, vcan_arm_r
              OpenArm Damiao motors 0x01..0x08, emulated (openamrobot_openarm_emulator codec.cpp), commanded by the
              unmodified openarm_ros2 driver through openarm_can (CAN FD).
"""
import struct

# ---------------------------------------------------------------- CANopen (CiA 301) common
NMT_STATES = {0x00: "boot-up", 0x04: "stopped", 0x05: "operational", 0x7F: "pre-operational"}
NMT_COMMANDS = {0x01: "start", 0x02: "stop", 0x80: "enter pre-operational", 0x81: "reset node", 0x82: "reset communication"}
SDO_ABORTS = {0x06020000: "object does not exist", 0x06090011: "sub-index does not exist", 0x06010002: "read only",
              0x06010001: "write only", 0x06090030: "value range", 0x08000000: "general error",
              0x08000021: "local control (PDO only)", 0x08000022: "device state"}

# ---------------------------------------------------------------- ZLAC8015D (vcan_drive)
DRIVE_NODE = 1
WHEEL_RADIUS_M = 0.098        # openamrobot_sil config/sil.yaml drive.wheel_radius (931e631); used only for the m/s column
RPM_TO_RAD_S = 2.0 * 3.141592653589793 / 60.0
ZLAC_OBJECTS = {
    0x6040: ("controlword", None), 0x6041: ("statusword", None), 0x603F: ("error code", None),
    0x6060: ("mode of operation", None), 0x60FF: ("target velocity", "r/min"), 0x606C: ("actual velocity", "0.1 r/min"),
    0x2000: ("link-loss time", "ms"), 0x2010: ("save to EEPROM", None), 0x1017: ("heartbeat producer time", "ms"),
    0x6083: ("acceleration time", "ms"), 0x6084: ("deceleration time", "ms"), 0x1000: ("device type", None),
}
ZLAC_SIGNED = (0x60FF, 0x606C, 0x6060)
ZLAC_STATUS_BITS = ((0x0001, "ready"), (0x0002, "switched on"), (0x0004, "operation enabled"), (0x0008, "fault"),
                    (0x0010, "voltage"))
ZLAC_QUICK_STOP_N = 0x0020      # low while quick stop is active
ZLAC_FAULT_BITS = ((0x0001, "over-voltage"), (0x0002, "under-voltage"), (0x0004, "over-current"), (0x0008, "overload"),
                   (0x0020, "encoder tolerance"), (0x0800, "encoder error"))
WHEELS = {1: "left", 2: "right"}  # 60FFh/606Ch sub 1 and 2: left and right in stm32_host.py (signs are SIL test values)

# ---------------------------------------------------------------- CTL-017 (vcan_aux)
LIFT_NODE, CORE_NODE = 0x10, 0x01
LIFT_STATES = ("INIT", "UNHOMED", "HOMING", "IDLE", "MOVING", "STOPPING", "FAULT")
LIFT_OPS = ("STOP", "MOVE_TO_STOP", "CLEAR_FAULT", "START_HOMING")
LIFT_FLAGS = ("position valid", "at stop", "moving", "lower limit", "upper limit", "fault", "motor power", "recovery")
LIFT_INPUTS = ("stop sensor L1000", "stop sensor L1175", "stop sensor L1350", "second E-stop input", "Hall supply good")
LIFT_AUX = ((1, "F09"), (2, "head light on"), (4, "Getac enable"))
LIFT_RESULTS = ("OK", "BAD_STOP", "POS_INVALID", "FAULT", "LIMIT", "BUSY", "RECOVERY", "CLEAR_UNMET", "HOMING_UNMET",
                "BAD_OP", "SEQ_CONFLICT", "NO_MOTOR_POWER")
LIFT_STOP_NAMES = ("L1000", "L1175", "L1350")
# The permit PDO: sil.yaml lift.permit (SIL test values, not a contract): COB-ID 410h, flag u8, boot u16, seq u16,
# reason u8, little endian; flag 1 is the permit, 0 none. cmd_result 0x80 is the SIL refusal for lack of a permit.
PERMIT_COB_ID, PERMIT_FORMAT, PERMIT_VALUE, PERMIT_REFUSED = 0x410, "<BHHB", 1, 0x80

# ---------------------------------------------------------------- Daly BMS (vcan_bms)
BMS_ADDRESS, BMS_HOST = 0x01, 0x40
DALY_IDS = {0x90: "voltage, current, SOC", 0x91: "cell voltage min/max", 0x92: "temperature min/max",
            0x93: "MOS state, remaining capacity", 0x94: "status", 0x98: "failure bits"}

# ---------------------------------------------------------------- Damiao (vcan_arm_l, vcan_arm_r)
MOTOR_TYPES = ("DM8009", "DM8009", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310", "DM4310")   # ids 0x01..0x08
ENCODING = {"DM4310": (12.5, 30.0, 10.0), "DM4340": (12.5, 10.0, 28.0), "DM8009": (12.5, 45.0, 54.0)}  # p, v, t max
KP_MAX, KD_MAX = 500.0, 5.0
CONFIG_ID = 0x7FF
CONFIG_COMMANDS = {0x33: "read register", 0x55: "write register", 0xAA: "save", 0xCC: "refresh (state request)"}
SPECIAL = {0xFB: "clear error", 0xFC: "enable", 0xFD: "disable", 0xFE: "set zero"}
MOTOR_CODES = {0x0: "disabled", 0x1: "enabled", 0x8: "over-voltage", 0x9: "under-voltage", 0xA: "over-current",
               0xB: "MOS over-temperature", 0xC: "coil over-temperature", 0xD: "communication lost", 0xE: "overload"}
ARM_SIDE = {"vcan_arm_l": "left", "vcan_arm_r": "right"}


def joint_of(side, motor):
    """openarm_<side>_joint1..7 for motors 1..7, the gripper for motor 8 (sim_controllers.yaml id order)."""
    return f"openarm_{side}_joint{motor}" if motor <= 7 else f"openarm_{side}_gripper"


def _bits(value, names):
    if isinstance(names[0], tuple):
        return [name for mask, name in names if value & mask]
    return [name for i, name in enumerate(names) if value & (1 << i)]


def _u(x, lo, hi, bits):
    return x / ((1 << bits) - 1) * (hi - lo) + lo


def _round(value, digits=4):
    return round(value, digits) if isinstance(value, float) else value


# ---------------------------------------------------------------- per bus
def _sdo(data, server_reply):
    """An expedited SDO frame (both directions)."""
    if len(data) < 4:
        return {"kind": "SDO", "error": "short"}
    cs, index, sub = data[0], struct.unpack_from("<H", data, 1)[0], data[3]
    name, unit = ZLAC_OBJECTS.get(index, (f"object {index:04X}h", None))
    out = {"index": f"{index:04X}h", "sub": sub, "object": name}
    raw = data[4:8].ljust(4, b"\0")
    if not server_reply and cs in (0x23, 0x2B, 0x2F):
        size = {0x23: 4, 0x2B: 2, 0x2F: 1}[cs]
        out.update(kind="SDO write", value=int.from_bytes(raw[:size], "little", signed=index in ZLAC_SIGNED))
    elif not server_reply and cs == 0x40:
        out.update(kind="SDO read request")
    elif server_reply and cs == 0x60:
        out.update(kind="SDO write confirmed")
    elif server_reply and cs in (0x43, 0x4B, 0x4F):
        size = {0x43: 4, 0x4B: 2, 0x4F: 1}[cs]
        out.update(kind="SDO read reply", value=int.from_bytes(raw[:size], "little", signed=index in ZLAC_SIGNED))
    elif cs == 0x80:
        code = struct.unpack("<I", raw)[0]
        out.update(kind="SDO abort", abort=f"{code:08X}h", abort_text=SDO_ABORTS.get(code, "unknown"))
    else:
        out.update(kind=f"SDO cs {cs:02X}h")
    if unit:
        out["unit"] = unit
    if "value" in out:
        value = out["value"]
        if index == 0x60FF:
            out.update(wheel=WHEELS.get(sub), rpm=value, rad_s=_round(value * RPM_TO_RAD_S),
                       wheel_m_s=_round(value * RPM_TO_RAD_S * WHEEL_RADIUS_M))
        elif index == 0x606C:
            rpm = value / 10.0
            out.update(wheel=WHEELS.get(sub), rpm=rpm, rad_s=_round(rpm * RPM_TO_RAD_S),
                       wheel_m_s=_round(rpm * RPM_TO_RAD_S * WHEEL_RADIUS_M))
        elif index == 0x6041:
            out.update(bits=_bits(value, ZLAC_STATUS_BITS), quick_stop=not value & ZLAC_QUICK_STOP_N)
        elif index == 0x603F:
            out.update(faults_low=_bits(value & 0xFFFF, ZLAC_FAULT_BITS), faults_high=_bits(value >> 16, ZLAC_FAULT_BITS))
        elif index == 0x6060:
            out.update(mode={3: "profile velocity"}.get(value, str(value)))
    return out


def decode_drive(can_id, data):
    node = DRIVE_NODE
    if can_id == 0x600 + node:
        return dict(device="ZLAC8015D", direction="STM32 slot to drive", **_sdo(data, False))
    if can_id == 0x580 + node:
        return dict(device="ZLAC8015D", direction="drive to STM32 slot", **_sdo(data, True))
    if can_id == 0x200 + node and len(data) == 8:
        left, right = struct.unpack("<ii", bytes(data))
        return {"device": "ZLAC8015D", "kind": "RPDO1 target velocity (SIL)", "rpm": [left, right]}
    if can_id == 0x300 + node and len(data) >= 2:
        return {"device": "ZLAC8015D", "kind": "RPDO2 controlword (SIL)", "value": struct.unpack_from("<H", bytes(data))[0]}
    return _canopen_common(can_id, data, {node: "ZLAC8015D"})


def decode_aux(can_id, data):
    d = bytes(data)
    if can_id == 0x200 + LIFT_NODE and len(d) >= 3:
        op = d[1]
        return {"device": "CTL-017", "kind": "RPDO1 command", "seq": d[0], "op": LIFT_OPS[op] if op < len(LIFT_OPS) else op,
                "stop_id": d[2], "stop": LIFT_STOP_NAMES[d[2]] if d[2] < 3 and op == 1 else None}
    if can_id == 0x300 + LIFT_NODE and len(d) >= 3:
        return {"device": "CTL-017", "kind": "RPDO2 light and Getac", "head_light": d[0], "dim": d[1], "getac": d[2]}
    if can_id == 0x180 + LIFT_NODE and len(d) == 8:
        state, flags, pos, stop, last_seq, result, counter = struct.unpack("<BBHBBBB", d)
        return {"device": "CTL-017", "kind": "TPDO1 status",
                "state": LIFT_STATES[state] if state < len(LIFT_STATES) else state, "flags": _bits(flags, LIFT_FLAGS),
                "flags_raw": flags, "position_mm": None if pos == 0xFFFF else pos / 10.0,
                "active_stop": None if stop == 0xFF else stop, "last_seq": last_seq,
                "cmd_result": "none" if result == 0xFF else ("PERMIT_REFUSED (SIL)" if result == PERMIT_REFUSED
                                                             else (LIFT_RESULTS[result] if result < len(LIFT_RESULTS) else result)),
                "counter": counter}
    if can_id == 0x280 + LIFT_NODE and len(d) == 8:
        inputs, target, current, velocity, faults, aux = struct.unpack("<BBHhBB", d)
        return {"device": "CTL-017", "kind": "TPDO2 inputs and drive", "inputs": _bits(inputs, LIFT_INPUTS),
                "target_stop": None if target == 0xFF else target, "motor_current_ma": current,
                "velocity_mm_s": velocity / 10.0, "faults": [f"F0{i + 1}" for i in range(8) if faults & (1 << i)],
                "faults_raw": faults, "aux": _bits(aux, LIFT_AUX)}
    if can_id == PERMIT_COB_ID and len(d) >= struct.calcsize(PERMIT_FORMAT):
        flag, boot, seq, reason = struct.unpack_from(PERMIT_FORMAT, d)
        return {"device": "lift permit (SIL)", "kind": "permit PDO", "permit": flag == PERMIT_VALUE, "flag": flag,
                "boot_id": boot, "seq": seq, "reason": reason}
    if can_id == 0x080 + LIFT_NODE and len(d) >= 4:
        code, register, fault = struct.unpack_from("<HBB", d)
        mask = struct.unpack_from("<H", d, 4)[0] if len(d) >= 6 else None
        return {"device": "CTL-017", "kind": "EMCY", "code": f"{code:04X}h", "error_register": register,
                "fault": f"F0{fault}" if fault else "reset", "faults_raw": mask}
    if can_id == 0x600 + LIFT_NODE:
        return dict(device="CTL-017", direction="to CTL-017", **_sdo(d, False))
    if can_id == 0x580 + LIFT_NODE:
        return dict(device="CTL-017", direction="from CTL-017", **_sdo(d, True))
    return _canopen_common(can_id, d, {LIFT_NODE: "CTL-017", CORE_NODE: "core (STM32 slot)"})


def _canopen_common(can_id, d, nodes):
    d = bytes(d)
    if can_id == 0x000 and len(d) >= 2:
        return {"device": nodes.get(d[1], "all nodes" if d[1] == 0 else f"node {d[1]:02X}h"), "kind": "NMT command",
                "command": NMT_COMMANDS.get(d[0], f"{d[0]:02X}h")}
    if 0x701 <= can_id <= 0x77F and len(d) >= 1:
        return {"device": nodes.get(can_id - 0x700, f"node {can_id - 0x700:02X}h"), "kind": "heartbeat",
                "nmt_state": NMT_STATES.get(d[0], f"{d[0]:02X}h")}
    if 0x081 <= can_id <= 0x0FF:
        return {"device": nodes.get(can_id - 0x080, f"node {can_id - 0x080:02X}h"), "kind": "EMCY", "raw": d.hex().upper()}
    return {"device": "unknown", "kind": "not decoded", "raw": d.hex().upper()}


def decode_bms(can_id, data, extended=True):
    d = bytes(data)
    if not extended:
        return {"device": "unknown", "kind": "11-bit frame on the BMS bus", "raw": d.hex().upper()}
    priority, data_id, dest, src = (can_id >> 24) & 0x1F, (can_id >> 16) & 0xFF, (can_id >> 8) & 0xFF, can_id & 0xFF
    out = {"device": "Daly BMS (emulated, protocol unconfirmed)", "data_id": f"{data_id:02X}h",
           "content": DALY_IDS.get(data_id, "unknown")}
    if priority != 0x18:
        return dict(out, kind="not a Daly frame", raw=d.hex().upper())
    if dest == BMS_ADDRESS and src == BMS_HOST:
        return dict(out, kind="request (STM32 slot to BMS)")
    if not (dest == BMS_HOST and src == BMS_ADDRESS) or len(d) < 8:
        return dict(out, kind="foreign address", raw=d.hex().upper())
    out["kind"] = "reply (BMS to STM32 slot)"
    if data_id == 0x90:
        v, gathered, i, soc = struct.unpack(">HHHH", d)
        out.update(pack_voltage_v=v / 10.0, acquired_voltage_v=gathered / 10.0, current_a=(i - 30000) / 10.0,
                   soc_percent=soc / 10.0)
    elif data_id == 0x91:
        vmax, cmax, vmin, cmin = struct.unpack(">HBHBxx", d)
        out.update(cell_max_mv=vmax, cell_max_no=cmax, cell_min_mv=vmin, cell_min_no=cmin)
    elif data_id == 0x92:
        out.update(temp_max_c=d[0] - 40, temp_max_no=d[1], temp_min_c=d[2] - 40, temp_min_no=d[3])
    elif data_id == 0x93:
        state, charge, discharge, life, remaining = struct.unpack(">BBBBI", d)
        out.update(charge_state=state, charge_mos_on=bool(charge), discharge_mos_on=bool(discharge), bms_life=life,
                   remaining_mah=remaining)
    elif data_id == 0x94:
        out.update(cells=d[0], temperature_sensors=d[1])
    elif data_id == 0x98:
        out.update(failure_raw=d.hex().upper(), any_failure=any(d))
    return out


def decode_arm(bus, can_id, data):
    side = ARM_SIDE.get(bus, bus)
    d = bytes(data).ljust(8, b"\0")
    if can_id == CONFIG_ID:
        motor = d[0] | (d[1] << 8)
        out = {"device": f"Damiao {side}", "kind": "config 7FFh", "motor": motor, "joint": joint_of(side, motor),
               "command": CONFIG_COMMANDS.get(d[2], f"{d[2]:02X}h")}
        if d[2] in (0x33, 0x55):
            out.update(rid=d[3], raw=struct.unpack_from("<I", d, 4)[0])
        return out
    if 0x001 <= can_id <= 0x008:
        motor = can_id
        joint, kind = joint_of(side, motor), MOTOR_TYPES[motor - 1]
        if all(b == 0xFF for b in d[:7]) and d[7] in SPECIAL:
            return {"device": f"Damiao {side}", "kind": "special command", "motor": motor, "joint": joint,
                    "command": SPECIAL[d[7]]}
        p, v, t = ENCODING[kind]
        q = (d[0] << 8) | d[1]
        dq = (d[2] << 4) | (d[3] >> 4)
        kp = ((d[3] & 0xF) << 8) | d[4]
        kd = (d[5] << 4) | (d[6] >> 4)
        tau = ((d[6] & 0xF) << 8) | d[7]
        return {"device": f"Damiao {side}", "kind": "MIT command", "motor": motor, "joint": joint, "motor_type": kind,
                "q_rad": _round(_u(q, -p, p, 16)), "dq_rad_s": _round(_u(dq, -v, v, 12)), "kp": _round(_u(kp, 0, KP_MAX, 12), 3),
                "kd": _round(_u(kd, 0, KD_MAX, 12), 3), "tau_ff_nm": _round(_u(tau, -t, t, 12))}
    if 0x101 <= can_id <= 0x308 and (can_id & 0xFF) and (can_id & 0xFF) <= 8:
        motor, mode = can_id & 0xFF, {1: "position-velocity", 2: "velocity", 3: "position-force"}.get(can_id >> 8)
        return {"device": f"Damiao {side}", "kind": f"{mode} command", "motor": motor, "joint": joint_of(side, motor),
                "raw": d.hex().upper()}
    if 0x011 <= can_id <= 0x018:
        motor = can_id - 0x10
        joint, kind = joint_of(side, motor), MOTOR_TYPES[motor - 1]
        if (d[0] | (d[1] << 8)) == motor and d[2] in (0x33, 0x55):
            return {"device": f"Damiao {side}", "kind": "register reply", "motor": motor, "joint": joint, "rid": d[3],
                    "raw": struct.unpack_from("<I", d, 4)[0]}
        p, v, t = ENCODING[kind]
        code = d[0] >> 4
        return {"device": f"Damiao {side}", "kind": "state feedback", "motor": motor, "joint": joint, "motor_type": kind,
                "id_nibble": d[0] & 0xF, "status": MOTOR_CODES.get(code, f"{code:X}h"), "fault": code >= 0x8,
                "q_rad": _round(_u((d[1] << 8) | d[2], -p, p, 16)), "dq_rad_s": _round(_u((d[3] << 4) | (d[4] >> 4), -v, v, 12)),
                "tau_nm": _round(_u(((d[4] & 0xF) << 8) | d[5], -t, t, 12)), "t_mos_c": d[6], "t_rotor_c": d[7]}
    return {"device": f"Damiao {side}", "kind": "not decoded", "raw": d.hex().upper()}


def decode(bus, can_id, data, extended=False):
    """Engineering values of one frame of a named Build 1 bus, as a dict (kind and device always present)."""
    if bus == "vcan_drive":
        return decode_drive(can_id, data)
    if bus == "vcan_aux":
        return decode_aux(can_id, data)
    if bus == "vcan_bms":
        return decode_bms(can_id, data, extended)
    if bus in ARM_SIDE:
        return decode_arm(bus, can_id, data)
    return {"device": "unknown", "kind": "unknown bus", "raw": bytes(data).hex().upper()}


def parse_candump(line):
    """(time s, bus, can id, data bytes, extended, fd) of one candump -L line, or None."""
    parts = line.split()
    if len(parts) < 3 or not parts[0].startswith("("):
        return None
    ident, sep, rest = parts[2].partition("#")
    if not sep:
        return None
    fd = rest.startswith("#")
    payload = rest[2:] if fd else rest
    if payload.startswith("R"):
        payload = ""
    try:
        return (float(parts[0].strip("()")), parts[1], int(ident, 16), bytes.fromhex(payload), len(ident) == 8, fd)
    except ValueError:
        return None


class Latest:
    """The latest decoded value of each signal, by bus, plus the last raw frame of each ID (selected raw frames)."""

    SIGNAL_KEYS = {
        "SDO write": lambda r: f"{r['index']}:{r['sub']} write", "SDO read reply": lambda r: f"{r['index']}:{r['sub']} read",
    }

    def __init__(self):
        self.values, self.raw = {}, {}

    def add(self, bus, can_id, data, extended=False, t=None):
        record = decode(bus, can_id, data, extended)
        key = self.SIGNAL_KEYS.get(record.get("kind"), lambda r: None)(record)
        if key is None:
            key = record.get("kind", "?")
            if "joint" in record:
                key = f"{record['joint']} {key}"
            elif "data_id" in record:
                key = f"{record['data_id']} {key}"
            elif record.get("kind") == "heartbeat":
                key = f"{record['device']} heartbeat"
        self.values.setdefault(bus, {})[key] = dict(record, t=t)
        self.raw.setdefault(bus, {})[f"{can_id:X}"] = {"data": bytes(data).hex().upper(), "t": t}
        return record

    def summary(self):
        return {"decoded": self.values, "raw_last_by_id": self.raw}
