# SPDX-License-Identifier: MIT
"""Decoders for the frames on the Build 1 virtual CAN buses, written from the sources pinned by the runs:
- vcan_arm_l / vcan_arm_r: Damiao motor protocol as in upstream openarm_can (CanPacketDecoder::parse_motor_state_data,
  create_refresh_command), motor types and IDs as in openarm_ros2 openarm_simple_hardware.hpp (send 0x01..0x08,
  feedback 0x11..0x18; DM8009 j1-j2, DM4340 j3-j4, DM4310 j5-j7 and gripper).
- vcan_drive: CANopen SDO (CiA 301) to the ZLAC8015D emulator, node 1; object names from CiA 402.
- vcan_aux: CTL-017 lift emulator, node 0x10: TPDO1 0x190 '<BBHBBBB' (state, flags, position 0.1 mm, active stop,
  last seq, cmd result, counter) as read by openamrobot_sil stm32_host.py; RPDO1 0x210 seq, op, stop_id (CTL-017 OD v1).
- vcan_bms: Daly BMS protocol, data ID 0x90 '>HHHH' (pack 0.1 V, acquired 0.1 V, current 0.1 A + 30000, SOC 0.1 %).
Anything else is shown as its CiA 301 function code only."""
import struct
LIM = {"DM8009": (12.5, 45, 54), "DM4340": (12.5, 10, 28), "DM4310": (12.5, 30, 10)}
MT = {1: "DM8009", 2: "DM8009", 3: "DM4340", 4: "DM4340", 5: "DM4310", 6: "DM4310", 7: "DM4310", 8: "DM4310"}
OBJ = {0x6040: "controlword", 0x6041: "statusword", 0x6060: "mode", 0x6061: "mode display", 0x606C: "velocity actual",
       0x60FF: "target velocity", 0x6064: "position actual", 0x6077: "torque actual", 0x6083: "accel", 0x6084: "decel"}
OPS = {0: "STOP", 1: "MOVE_TO_STOP", 2: "CLEAR_FAULT", 3: "START_HOMING"}
def u2f(x, lo, hi, bits): return x * (hi - lo) / ((1 << bits) - 1) + lo
def payload(s):
    s = s.decode() if isinstance(s, bytes) else s
    if s.startswith("#"): s = s[2:]          # CAN FD: '##<flags><data>' was stored as '#<flags><data>'
    return bytes.fromhex(s) if len(s) % 2 == 0 else bytes.fromhex(s[:-1])
def damiao_fb(can_id, d):
    j = can_id - 0x10; p, v, t = LIM[MT[j]]
    q = (d[1] << 8) | d[2]; dq = (d[3] << 4) | (d[4] >> 4); tau = ((d[4] & 0xF) << 8) | d[5]
    return j, d[0] >> 4, u2f(q, -p, p, 16), u2f(dq, -v, v, 12), u2f(tau, -t, t, 12), d[6], d[7]
def decode(bus, can_id, data):
    d = payload(data)
    if bus.startswith("vcan_arm"):
        if 0x11 <= can_id <= 0x18 and len(d) >= 8:
            j, err, q, dq, tau, tm, tr = damiao_fb(can_id, d)
            nm = "gripper" if j == 8 else f"j{j}"
            return f"fb {nm} {MT[j]} st{err} q {q:+.3f} rad dq {dq:+.2f} tau {tau:+.2f} N m"
        if 0x01 <= can_id <= 0x08: return f"MIT command to motor {can_id} ({'gripper' if can_id == 8 else 'j%d' % can_id})"
        if can_id == 0x7FF and len(d) >= 3 and d[2] == 0xCC: return f"refresh request to motor {d[0] | d[1] << 8}"
        if can_id == 0x7FF: return "parameter frame (0x7FF)"
        return "other"
    if bus == "vcan_drive":
        if can_id in (0x601, 0x581) and len(d) == 8:
            cs = d[0]; idx = d[1] | d[2] << 8; sub = d[3]; val = struct.unpack("<i", d[4:8])[0]
            o = OBJ.get(idx, f"0x{idx:04X}")
            if can_id == 0x601: return f"SDO write {o} sub {sub} = {val}" if cs & 0xE0 == 0x20 else f"SDO read {o} sub {sub}"
            if cs == 0x60: return f"SDO write ack {o} sub {sub}"
            if cs & 0xE0 == 0x40: return f"SDO read reply {o} sub {sub} = {val}"
            if cs == 0x80: return f"SDO abort {o}"
        if 0x700 <= can_id <= 0x77F: return f"heartbeat node {can_id - 0x700} NMT {d[0] if d else '-'}"
        return "CANopen frame"
    if bus == "vcan_aux":
        if can_id == 0x190 and len(d) == 8:
            st, fl, pos, stop, seq, res, cnt = struct.unpack("<BBHBBBB", d)
            return f"TPDO1 lift state {st} pos {'unknown' if pos == 0xFFFF else '%.1f mm' % (pos / 10)} stop {stop} seq {seq} result {res}"
        if can_id == 0x210 and len(d) >= 3: return f"RPDO1 lift cmd seq {d[0]} {OPS.get(d[1], d[1])} stop {d[2]}"
        if can_id == 0x290: return "TPDO2 lift status 2"
        if can_id == 0x410: return "lift permit frame from the STM32 stand-in"
        if 0x700 <= can_id <= 0x77F: return f"heartbeat node 0x{can_id - 0x700:X} NMT {d[0] if d else '-'}"
        return "CANopen frame"
    if bus == "vcan_bms":
        did = (can_id >> 16) & 0xFF; src = can_id & 0xFF
        if did == 0x90 and src == 0x01 and len(d) == 8:
            v, a, i, soc = struct.unpack(">HHHH", d); return f"SOC reply {v / 10:.1f} V {(i - 30000) / 10:+.1f} A SOC {soc / 10:.1f} %"
        return f"Daly 0x{did:02X} {'request' if src == 0x40 else 'reply'}"
    return ""
