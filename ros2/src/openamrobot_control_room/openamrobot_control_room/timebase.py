# SPDX-License-Identifier: MIT
"""One time base for a recorded run: Gazebo (simulation) time.

The bag is recorded with `ros2 bag record --use-sim-time`, the mission JSON gives phase begin and end in Gazebo seconds,
and REPLAY plays the bag on its own receive times: those three are on Gazebo time already. candump stamps frames with
host wall time. This module maps host time to Gazebo time with pairs that the run itself recorded:

  CTL-017 TPDO1 (vcan_aux, COB-ID 190h) carries an 8-bit counter. The host client publishes each decoded TPDO1 on
  /sil/lift/status with the same `counter` and `sim_clock.now`, the Gazebo time at which it handled the frame. The
  frame with counter c in candump and the status with counter c are the same event: one (host s, Gazebo s) pair.

  /sil/lift/status is published on a 0.05 s Gazebo timer (stm32_host_client_node.py on_status), so a TPDO1 pair
  is 0 to 50 ms of Gazebo time late: a bias the pairs cannot see.

  The control room's own status lines (monitor and vcan monitor, live next to the mission) carry `host_time` (wall
  clock at the tick) next to `sim_time` (the last /clock, 1000 Hz of Gazebo time): a second set of pairs, late by at
  most the executor's queue (wall ms times the real-time factor). Comparing the two maps measures the residual skew.

Between pairs the map is linear; outside the first and last pair it is undefined (None), never extrapolated. The
residual skew is measured leave-one-out: each inner pair is predicted from its neighbours and compared with its
recorded Gazebo time. The host client's handling latency is in every pair (host wall time times the real-time
factor), so it is part of the measured residual. Simulation tooling only; not hardware or safety evidence.
"""
import bisect
import json

TPDO1 = 0x190


def tpdo1_frames(parsed):
    """[(host_t, counter)] of CTL-017 TPDO1 frames, in host time order, from can_decode.parse_candump tuples."""
    from openamrobot_control_room import can_decode as cd
    out = []
    for t, bus, can_id, data, ext, _fd in parsed:
        if can_id != TPDO1 or ext:
            continue
        record = cd.decode(bus, can_id, data, ext)
        if record.get("kind") == "TPDO1 status" and record.get("counter") is not None:
            out.append((t, int(record["counter"])))
    return sorted(out)


def lift_status_samples(messages):
    """[(gazebo_t, counter)] from /sil/lift/status JSON strings, given as (bag_t, text) or (bag_t, dict). The Gazebo time
    is the status's own sim_clock.now when it has one, else the bag's receive time."""
    out = []
    for bag_t, value in messages:
        if isinstance(value, str):
            try:
                value = json.loads(value)
            except ValueError:
                continue
        if not isinstance(value, dict) or value.get("counter") is None:
            continue
        now = (value.get("sim_clock") or {}).get("now")
        out.append((float(now) if now is not None else float(bag_t), int(value["counter"])))
    return sorted(out)


def match(frames, statuses):
    """(pairs, gaps): (host, gazebo) pairs of the same TPDO1 event, and 1 if matching stopped at a gap.

    The host client republishes its status (about 20 Hz against 10 Hz of TPDO1), so only the first status of each
    counter is used: the one written when the frame was handled. The first pair is the first frame with the first
    status's counter; every later frame must be exactly d frames after the previous match, d = (counter - previous
    counter) mod 256. When it is not (a gap in the candump or the bag) matching stops: after a gap a counter is
    ambiguous by whole wraps (25.6 s of Gazebo time) and a wrong choice would place frames at invented times."""
    pairs, j_prev, c_last = [], None, None
    for sim_t, counter in statuses:
        if counter == c_last:
            continue
        c_last = counter
        if j_prev is None:
            j = next((i for i, (_t, c) in enumerate(frames) if c == counter), None)
            if j is None:
                continue
        else:
            j = j_prev + (counter - frames[j_prev][1]) % 256
            if j >= len(frames):
                break                    # the candump ended
            if frames[j][1] != counter:
                return pairs, 1
        host_t = frames[j][0]
        if pairs and not (host_t > pairs[-1][0] and sim_t >= pairs[-1][1]):
            return pairs, 1
        pairs.append((host_t, sim_t))
        j_prev = j
    return pairs, 0


class HostToSim:
    """Piecewise-linear host time to Gazebo time through recorded pairs; None outside them."""

    def __init__(self, pairs):
        self.pairs = sorted(pairs)
        self.hosts = [p[0] for p in self.pairs]

    def __bool__(self):
        return len(self.pairs) >= 2

    def __call__(self, host_t):
        if len(self.pairs) < 2 or host_t < self.hosts[0] or host_t > self.hosts[-1]:
            return None
        k = bisect.bisect_right(self.hosts, host_t)
        if k >= len(self.pairs):
            return self.pairs[-1][1]
        (h0, s0), (h1, s1) = self.pairs[k - 1], self.pairs[k]
        return s0 if h1 == h0 else s0 + (host_t - h0) * (s1 - s0) / (h1 - h0)


def leave_one_out(pairs):
    """Gazebo-time residuals (predicted minus recorded) of every inner pair, predicted from its two neighbours."""
    out = []
    for k in range(1, len(pairs) - 1):
        (h0, s0), (h, s), (h1, s1) = pairs[k - 1], pairs[k], pairs[k + 1]
        if h1 > h0:
            out.append(s0 + (h - h0) * (s1 - s0) / (h1 - h0) - s)
    return out


def _quantile(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(q * (len(ordered) - 1))))]


def skew_summary(residuals):
    """Absolute residuals in Gazebo milliseconds: count, p50, p95, max (None without residuals)."""
    if not residuals:
        return {"n": 0, "p50_ms": None, "p95_ms": None, "max_ms": None}
    absolute = [abs(r) * 1000.0 for r in residuals]
    return {"n": len(absolute), "p50_ms": round(_quantile(absolute, 0.5), 3), "p95_ms": round(_quantile(absolute, 0.95), 3),
            "max_ms": round(max(absolute), 3)}


def mission_status_skew(mission, status_messages):
    """Mission JSON phase begins against the receive times of /baseline1/mission_status (Proposed, #82) in the bag:
    [(phase, json sim_start, bag receive t, difference s)] for each phase whose 'running' status was recorded."""
    first_running = {}
    for bag_t, text in status_messages:
        try:
            value = json.loads(text) if isinstance(text, str) else text
        except ValueError:
            continue
        if isinstance(value, dict) and value.get("state") == "running" and value.get("phase") not in first_running:
            first_running[value.get("phase")] = bag_t
    rows = []
    for phase in (mission or {}).get("phases", []):
        start, seen = phase.get("sim_start"), first_running.get(phase.get("phase"))
        if start is not None and seen is not None:
            rows.append((phase["phase"], float(start), float(seen), round(float(seen) - float(start), 4)))
    return rows


# The map is used only when it is dense and checked: enough pairs, most of the candump covered, a small
# leave-one-out residual.
# Otherwise CAN stays on host time, labelled as such: an unchecked map would put frames at invented Gazebo times.
MIN_PAIRS, MIN_COVERAGE, MAX_P95_MS = 20, 0.9, 50.0


def from_run(candump_lines_by_bus, lift_status):
    """(HostToSim or None, summary, raw HostToSim) of a run: candump -L lines per bus and the /sil/lift/status
    messages of its bag. The first is None when the map fails its checks; the raw map is for comparisons only."""
    from openamrobot_control_room import can_decode as cd
    aux = [p for p in (cd.parse_candump(line) for line in candump_lines_by_bus.get("vcan_aux", [])) if p]
    frames, statuses = tpdo1_frames(aux), lift_status_samples(lift_status)
    pairs, gaps = match(frames, statuses)
    loo = skew_summary(leave_one_out(pairs))
    span = (aux[-1][0] - aux[0][0]) if len(aux) >= 2 else 0.0
    coverage = round((pairs[-1][0] - pairs[0][0]) / span, 4) if len(pairs) >= 2 and span > 0 else 0.0
    summary = {"tpdo1_frames": len(frames), "lift_status": len(statuses), "pairs": len(pairs), "stopped_at_gap": bool(gaps),
               "coverage_of_candump": coverage,
               "host_span_s": round(pairs[-1][0] - pairs[0][0], 3) if len(pairs) >= 2 else None,
               "gazebo_span_s": [round(pairs[0][1], 3), round(pairs[-1][1], 3)] if pairs else None,
               "leave_one_out": loo}
    reasons = []
    if len(pairs) < MIN_PAIRS:
        reasons.append(f"{len(pairs)} pairs, at least {MIN_PAIRS} needed")
    if coverage < MIN_COVERAGE:
        reasons.append(f"the pairs cover {coverage:.0%} of the candump's host time span, at least {MIN_COVERAGE:.0%} needed"
                       + (" (matching stopped at a gap in the candump or the bag)" if gaps else ""))
    if loo["p95_ms"] is not None and loo["p95_ms"] > MAX_P95_MS:
        reasons.append(f"leave-one-out p95 {loo['p95_ms']} ms above {MAX_P95_MS} ms")
    summary["used"] = not reasons
    summary["not_used_because"] = "; ".join(reasons) or None
    raw = HostToSim(pairs)
    return (raw if not reasons else None), summary, raw


def run_time_base(candump_lines_by_bus, lift_status, control_room_lines=(), mission=None, mission_status=()):
    """The time base of a recorded run and its measured skews, for the report and the job log.

    Returns (to_sim or None, info). to_sim maps candump host time to Gazebo time: the control room's own pairs when
    there are enough of them (late by the executor's queue only), else the TPDO1 counter pairs when they pass their
    checks (0 to 50 ms late, the status timer), else None and CAN stays on host time."""
    tpdo_map, tpdo, tpdo_raw = from_run(candump_lines_by_bus, lift_status) if candump_lines_by_bus else (None, None, None)
    cr_pairs = status_line_pairs(control_room_lines)
    cr_map = HostToSim(cr_pairs) if len(cr_pairs) >= MIN_PAIRS else None
    info = {"bag": "Gazebo time (ros2 bag record --use-sim-time: receive times on /clock)",
            "mission_json": "Gazebo time (phase sim_start and sim_end)",
            "replay": "Gazebo time (the bag's receive times, republished on /control_room/replay/clock)",
            "candump": "host wall time (kernel receive stamps)",
            "tpdo1_pairs": tpdo,
            "control_room_pairs": {"n": len(cr_pairs), "leave_one_out": skew_summary(leave_one_out(cr_pairs))}}
    if tpdo_raw and cr_pairs:
        info["tpdo1_map_minus_control_room_pairs"] = compare(tpdo_raw, cr_pairs)
    rows = mission_status_skew(mission, mission_status)
    info["mission_status_minus_mission_json"] = {
        "phases": len(rows), "rows": rows,
        "max_abs_ms": round(max(abs(r[3]) for r in rows) * 1000.0, 3) if rows else None}
    if cr_map:
        info["can_mapped_by"] = f"control room status pairs ({len(cr_pairs)})"
        return cr_map, info
    if tpdo_map:
        info["can_mapped_by"] = f"CTL-017 TPDO1 counter pairs ({tpdo['pairs']}), 0 to 50 ms of Gazebo time late"
        return tpdo_map, info
    info["can_mapped_by"] = None
    info["can_not_mapped_because"] = ("no candump logs" if not candump_lines_by_bus else
                                      (tpdo or {}).get("not_used_because") or "no pairs")
    return None, info


def status_line_pairs(lines):
    """(host, gazebo) pairs from the control room's JSON lines (status.jsonl of the monitor: host_time, sim_time; or
    can.jsonl of the vcan monitor: summary.host_time, summary.sim_time). Lines without either are skipped."""
    pairs = []
    for line in lines:
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict) and "summary" in value and isinstance(value["summary"], dict):
            value = value["summary"]
        if isinstance(value, dict) and value.get("host_time") is not None and value.get("sim_time") is not None:
            pairs.append((float(value["host_time"]), float(value["sim_time"])))
    out = []
    for host_t, sim_t in sorted(pairs):      # Gazebo time stands still while paused: keep the first of a run
        if not out or (host_t > out[-1][0] and sim_t > out[-1][1]):
            out.append((host_t, sim_t))
    return out


def compare(mapping, pairs):
    """Gazebo-time difference of `mapping` against independent pairs (mapping minus pair) where both are defined:
    {"n", "mean_ms", "p50_ms", "p95_ms", "max_ms"} on the signed and absolute differences."""
    diffs = [mapping(h) - s for h, s in pairs if mapping(h) is not None]
    if not diffs:
        return {"n": 0, "mean_ms": None, "p50_ms": None, "p95_ms": None, "max_ms": None}
    summary = skew_summary(diffs)
    summary["mean_ms"] = round(1000.0 * sum(diffs) / len(diffs), 3)
    return summary
