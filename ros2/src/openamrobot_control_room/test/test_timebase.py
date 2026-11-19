# SPDX-License-Identifier: MIT
"""timebase.py without ROS: candump host time to Gazebo time through CTL-017 TPDO1 counters and the control room's
own (host_time, sim_time) pairs, on a synthetic run with a known clock, and the refusal on the sparse fixture."""
import json
import math
import struct

from conftest import package_path
from openamrobot_control_room import report, timebase as tb

HOST0 = 1795000000.0


def clock():
    """A known Gazebo clock: the real-time factor goes from 0.05 to 0.25 and back; host(sim) by integration."""
    sims, hosts, h = [], [], HOST0
    for k in range(0, 60001):           # 60 s of Gazebo time in 1 ms steps
        s = k * 0.001
        sims.append(s)
        hosts.append(h)
        h += 0.001 / (0.15 + 0.10 * math.sin(s / 7.0))
    return sims, hosts


def host_of(s, sims, hosts):
    i = min(int(s / 0.001), len(sims) - 2)
    return hosts[i] + (s - sims[i]) / 0.001 * (hosts[i + 1] - hosts[i])


def synthetic(drop=None):
    """TPDO1 every 0.1 s of Gazebo time on vcan_aux (counter from 58), the host client's status every 0.05 s with the
    counter of the last frame it handled and sim_clock.now = the status time (as stm32_host_client_node.py)."""
    sims, hosts = clock()
    frames, lines, status = [], [], []
    for k in range(0, 590):
        s = 1.0 + 0.1 * k
        counter = (58 + k) % 256
        data = struct.pack("<BBHBBBB", 3, 67, 50, 0, 0, 255, counter)
        frames.append((s, counter))
        if drop is None or not drop[0] <= s < drop[1]:
            lines.append(f"({host_of(s, sims, hosts):.6f}) vcan_aux 190#{data.hex().upper()}")
    for k in range(0, 1170):
        s = 1.025 + 0.05 * k
        handled = [c for fs, c in frames if fs <= s]
        if handled:
            status.append((s, json.dumps({"counter": handled[-1], "sim_clock": {"now": round(s, 3)}})))
    return {"vcan_aux": lines}, status, (sims, hosts)


def test_tpdo1_pairs_recover_the_clock_within_the_status_timer():
    lines, status, (sims, hosts) = synthetic()
    to_sim, summary, _raw = tb.from_run(lines, status)
    assert summary["used"], summary
    assert summary["pairs"] > 500 and not summary["stopped_at_gap"] and summary["coverage_of_candump"] > 0.99
    # Each pair is late by the status timer (0.025 s here: TPDO1 at x.x0, status at x.x25): the bias the doc names.
    for s in (5.0, 20.05, 33.3, 55.5):
        assert abs(to_sim(host_of(s, sims, hosts)) - (s + 0.025)) < 0.002, s
    assert summary["leave_one_out"]["p95_ms"] < 2.0
    assert to_sim(HOST0 - 1.0) is None, "extrapolated before the first pair"


def test_a_gap_in_the_candump_stops_the_map_rather_than_guessing():
    """About one counter wrap of frames missing: after the gap a counter is ambiguous by 25.6 s of Gazebo time. The
    map stops at the gap, covers too little of the candump and is not used: CAN stays on host time."""
    lines, status, (sims, hosts) = synthetic(drop=(20.0, 45.0))
    to_sim, summary, raw = tb.from_run(lines, status)
    assert summary["stopped_at_gap"] and to_sim is None and "cover" in summary["not_used_because"]
    assert raw(host_of(50.0, sims, hosts)) is None, "a frame after the gap was placed on Gazebo time"
    assert abs(raw(host_of(10.0, sims, hosts)) - 10.025) < 0.002


def test_republished_status_is_matched_once():
    frames = [(HOST0 + k, (250 + k) % 256) for k in range(10)]
    statuses = [(0.1 * k, (250 + k // 2) % 256) for k in range(20)]       # every counter twice
    pairs, gaps = tb.match(frames, statuses)
    assert len(pairs) == 10 and gaps == 0 and pairs[0] == (HOST0, 0.0) and pairs[1] == (HOST0 + 1, 0.2)


def test_the_sparse_fixture_is_refused_not_mapped():
    """The recorded fixture keeps only candump windows and 41 of 4622 statuses: the counters cannot be matched safely
    (wraps between windows), so CAN must stay on host time rather than be placed at invented Gazebo times."""
    base = package_path("test", "fixtures", "runs", "grocery_44299879412")
    with open(f"{base}/candump_vcan_aux.log", encoding="utf-8") as f:
        lines = {"vcan_aux": f.read().splitlines()}
    with open(f"{base}/messages.json", encoding="utf-8") as f:
        samples = json.load(f)["/sil/lift/status"]["samples"]
    to_sim, summary, _raw = tb.from_run(lines, [(s["t"], s["value"]) for s in samples])
    assert to_sim is None and not summary["used"] and "pairs" in summary["not_used_because"]


def test_control_room_pairs_and_the_comparison():
    lines, status, (sims, hosts) = synthetic()
    cr_lines = [json.dumps({"host_time": host_of(s, sims, hosts), "sim_time": s}) for s in (2.0 + 0.5 * k for k in range(100))]
    cr_lines.append(json.dumps({"summary": {"host_time": host_of(52.0, sims, hosts), "sim_time": 52.0}}))
    cr_lines.append("not json")
    pairs = tb.status_line_pairs(cr_lines)
    assert len(pairs) == 101
    to_sim, info = tb.run_time_base(lines, status, cr_lines)
    assert info["can_mapped_by"].startswith("control room status pairs")
    assert abs(to_sim(host_of(30.0, sims, hosts)) - 30.0) < 0.002
    diff = info["tpdo1_map_minus_control_room_pairs"]
    assert diff["n"] > 90 and abs(diff["mean_ms"] - 25.0) < 2.0, diff     # the TPDO1 bias, measured


def test_mission_status_skew_and_unmapped_run():
    mission = {"phases": [{"phase": "spawn", "sim_start": 0.5}, {"phase": "nav_to_station", "sim_start": 1.0},
                          {"phase": "grasp", "sim_start": None}]}
    status = [(0.52, json.dumps({"phase": "spawn", "state": "running"})),
              (0.9, json.dumps({"phase": "spawn", "state": "done"})),
              (1.004, json.dumps({"phase": "nav_to_station", "state": "running"}))]
    rows = tb.mission_status_skew(mission, status)
    assert [(r[0], r[3]) for r in rows] == [("spawn", 0.02), ("nav_to_station", 0.004)]
    to_sim, info = tb.run_time_base({}, [], [], mission, status)
    assert to_sim is None and info["can_not_mapped_because"] == "no candump logs"
    assert info["mission_status_minus_mission_json"]["max_abs_ms"] == 20.0


def test_report_places_can_on_gazebo_time_only_when_mapped():
    lines, status, (sims, hosts) = synthetic()
    to_sim, info = tb.run_time_base(lines, status)
    mapped = report.candump_decoded(lines, to_sim=to_sim)
    unmapped = report.candump_decoded(lines)
    assert mapped["time_base"] == "gazebo" and unmapped["time_base"] == "host"
    first_mapped = mapped["series"]["lift TPDO1 position"][0][0]
    assert 1.0 <= first_mapped <= 1.05 and unmapped["series"]["lift TPDO1 position"][0][0] == 0.0
    page = report.render(report.extract([]), None, report.candump_rates(lines, to_sim=to_sim), decoded=mapped, time_base=info)
    assert "Gazebo s (candump host time mapped" in page and "CTL-017 TPDO1 counter pairs" in page
    page = report.render(report.extract([]), None, report.candump_rates(lines), decoded=unmapped,
                         time_base=tb.run_time_base({}, [], [])[1])
    assert "host wall s since the first frame" in page and "not mapped: no candump logs" in page
