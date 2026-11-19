# SPDX-License-Identifier: MIT
"""Scratch (diag/arm-torque, not for merge): shared readers for the arm torque investigation."""
import glob
import json
import math
import re

DM = {"DM8009": (12.5, 45.0, 54.0), "DM4340": (12.5, 10.0, 28.0), "DM4310": (12.5, 30.0, 10.0)}
TYPES = ["DM8009", "DM8009", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310", "DM4310"]
LINE = re.compile(r"^\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]{3,8})(##?)(\S*)$")


def u2f(x, lo, hi, bits):
    return x / ((1 << bits) - 1) * (hi - lo) + lo


def decode_mit(d, motor):
    p, v, t = DM[TYPES[motor - 1]]
    q = (d[0] << 8) | d[1]
    dq = (d[2] << 4) | (d[3] >> 4)
    kp = ((d[3] & 0xF) << 8) | d[4]
    kd = (d[5] << 4) | (d[6] >> 4)
    tau = ((d[6] & 0xF) << 8) | d[7]
    return {"q": u2f(q, -p, p, 16), "dq": u2f(dq, -v, v, 12), "kp": u2f(kp, 0, 500, 12), "kd": u2f(kd, 0, 5, 12),
            "tau": u2f(tau, -t, t, 12)}


def decode_state(d, motor):
    p, v, t = DM[TYPES[motor - 1]]
    return {"code": d[0] >> 4, "q": u2f((d[1] << 8) | d[2], -p, p, 16), "dq": u2f((d[3] << 4) | (d[4] >> 4), -v, v, 12),
            "tau": u2f(((d[4] & 0xF) << 8) | d[5], -t, t, 12)}


def candump(path):
    """-> list of (t, can_id, bytes) for 8-byte frames of a candump -L file."""
    out = []
    for line in open(path, errors="replace"):
        m = LINE.match(line.strip())
        if not m:
            continue
        stamp, _iface, ident, sep, payload = m.groups()
        if sep == "##":
            payload = payload[1:]
        if payload.startswith("R"):
            continue
        data = bytes.fromhex(payload)
        if len(data) == 8:
            out.append((float(stamp), int(ident, 16), data))
    return out


def arm_can(path):
    """-> {motor: {"cmd": [(t, mit)], "state": [(t, state)]}} for motors 1..8 of one arm bus."""
    motors = {m: {"cmd": [], "state": []} for m in range(1, 9)}
    for t, cid, data in candump(path):
        if 0x001 <= cid <= 0x008:
            if data[:7] == b"\xff" * 7:
                continue   # enable, disable, zero, clear error
            motors[cid]["cmd"].append((t, decode_mit(data, cid)))
        elif 0x011 <= cid <= 0x018:
            motors[cid - 0x10]["state"].append((t, decode_state(data, cid - 0x10)))
    return motors


def at(series, t):
    """The last (t, value) of a time-sorted series at or before t."""
    lo, hi = 0, len(series)
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    return series[lo - 1] if lo else None


def bag_messages(root, topics):
    """{topic: [(t_log_s, msg)]} from every .mcap under root (log time: Gazebo time with --use-sim-time)."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    rows = {t: [] for t in topics}
    for bag in sorted(glob.glob(root + "/**/*.mcap", recursive=True)):
        with open(bag, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            try:
                for _schema, channel, message, ros in reader.iter_decoded_messages(topics=list(topics)):
                    rows[channel.topic].append((message.log_time * 1e-9, ros))
            except Exception as error:  # noqa: BLE001
                print("bag read error", bag, error)
    for v in rows.values():
        v.sort(key=lambda r: r[0])
    return rows


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quantities(path):
    d = json.load(open(path))
    return {q["name"]: q for q in d.get("quantities", [])}, d
