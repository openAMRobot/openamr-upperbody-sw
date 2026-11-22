# SPDX-License-Identifier: MIT
"""tools/dds_benchmark: the pure parts of the observer and the comparison (no ROS needed)."""
import json
import random
import struct

from dds_benchmark import compare, observer


def test_harness_domain_is_the_harness_draw():
    # build1_harness.description: random.seed(BASELINE1_SEED) in the test module, then random.randint(101, 200).
    random.seed(20261117)
    assert observer.harness_domain("20261117") == random.randint(101, 200)


def test_discovery_port_and_index_round_trip():
    domain = 119
    assert observer.discovery_port(domain, 0) == 7400 + 250 * 119 + 10
    for index in (0, 1, 31, 32, 80, 119):
        assert observer.participant_index(observer.discovery_port(domain, index), domain) == index
    assert observer.participant_index(observer.discovery_port(domain, 5) + 1, domain) is None    # user unicast (d3)
    assert observer.participant_index(7400 + 250 * 118 + 10, domain) is None                       # other domain
    assert observer.participant_index(observer.discovery_port(domain, 120), domain) is None


def test_parse_ss_maps_ports_to_processes():
    port_0, port_33 = observer.discovery_port(119, 0), observer.discovery_port(119, 33)
    text = "\n".join([
        f'UNCONN 0      0            0.0.0.0:{port_0}      0.0.0.0:*    users:(("python3",pid=100,fd=12))',
        f'UNCONN 0      0            0.0.0.0:{port_0 + 1}  0.0.0.0:*    users:(("python3",pid=100,fd=13))',
        f'UNCONN 0      0            0.0.0.0:{port_33}     0.0.0.0:*    users:(("base_state_adap",pid=200,fd=9),("base_state_adap",pid=200,fd=10))',
        'UNCONN 0      0          127.0.0.53%lo:53         0.0.0.0:*',
    ])
    assert observer.parse_ss(text, 119) == {0: [100], 33: [200]}


def test_process_name():
    assert observer.process_name(["/usr/bin/python3", "/ws/install/x/lib/x/base_state_adapter.py", "--ros-args",
                                  "-r", "__node:=base_state_adapter"]) == "base_state_adapter"
    assert observer.process_name(["/usr/bin/python3", "/ws/install/x/lib/x/rtf_monitor.py"]) == "rtf_monitor.py"
    assert observer.process_name(["/opt/ros/jazzy/lib/nav2_amcl/amcl"]) == "amcl"
    assert observer.process_name([]) == "?"


def test_subscriber_reliability_matches_publishers():
    assert observer.subscriber_reliability(["RELIABLE", "RELIABLE"]) == "RELIABLE"
    assert observer.subscriber_reliability(["RELIABLE", "BEST_EFFORT"]) == "BEST_EFFORT"
    assert observer.subscriber_reliability([]) == "BEST_EFFORT"


def test_delivery_estimate_counts_missing_stamps():
    stamps = [i * 0.2 for i in range(100) if i % 10 != 3]     # a 5 Hz stream with 10 of 100 stamps missing
    est = observer.delivery_estimate(stamps)
    assert est["expected"] == 100 and est["unique_stamps"] == 90 and est["ratio"] == 0.9
    assert abs(est["period_s"] - 0.2) < 1e-6 and abs(est["largest_gap_s"] - 0.4) < 1e-6
    assert observer.delivery_estimate([1.0])["ratio"] is None


def test_latency_helpers():
    clock_sim, clock_wall = [0.0, 1.0, 2.0, 3.0], [10.0, 20.0, 30.0, 40.0]
    assert observer.wall_at(clock_sim, clock_wall, 1.5) == 30.0
    assert observer.wall_at(clock_sim, clock_wall, 5.0) is None
    assert observer.rtf_windows(clock_sim, clock_wall, window_s=20.0) == [{"wall": 30.0, "sim": 2.0, "rtf": 0.1}]
    p = observer.percentiles([1, 2, 3, 4, 100])
    assert p["p50"] == 3 and p["max"] == 100 and p["n"] == 5


def test_header_stamp_reads_cdr():
    raw = bytes([0, 1, 0, 0]) + struct.pack("<iI", 12, 500_000_000) + b"\x00" * 8
    assert observer.header_stamp(raw) == 12.5
    assert observer.header_stamp(b"\x00\x01") is None


def test_peers_xml_lists_every_index():
    xml = observer.peers_xml(119, count=40)
    assert xml.count("<locator>") == 40
    assert f"<port>{observer.discovery_port(119, 39)}</port>" in xml and 'is_default_profile="true"' in xml


def test_console_digest():
    text = "\n".join([
        "[station_marker_detector-34] [INFO] [1.0] [station_marker_detector]: stats age_ms_max=51 messages=50 published=30 seen_2=30",
        "[station_marker_detector-34] [INFO] [2.0] [station_marker_detector]: stats messages=50 seen_0=50",
        "[station_alignment-35] [WARN] [3.0] [station_alignment]: station alignment not aligned: marker not seen: 0 received, 0 fresh",
        "[rmw_cyclonedds_cpp]: Failed to find a free participant index for domain 119",
        "[base_state_adapter.py-15] [WARN] [4.0] [base_state_adapter]: no wheel speed sample for 0.5 s of Gazebo time",
    ])
    d = compare.console_digest(text)
    assert d["detector_station_poses_published"] == 30 and d["detector_detection_messages"] == 100
    assert d["alignment_station_poses_received"] == 0
    assert d["patterns"]["free_participant_index"] == 1 and d["patterns"]["failed_to"] == 1
    assert d["patterns"]["no_wheel_speed_sample"] == 1


def test_phases_and_table(tmp_path):
    cell = tmp_path / "dds-benchmark-fastdds_default-r1-attempt1" / "bench"
    cell.mkdir(parents=True)
    (cell / "phase_table.txt").write_text("| 1 | spawn | x | y | PASS | - | 0.5 | - |\n| 2 | nav_to_station | x | y | FAIL | CONTROL | 3 | r |\n")
    (cell / "processes.csv").write_text("wall_s,pid,name,cpu_percent,rss_mb\n5,0,TOTAL,350,9000\n5,10,amcl,20,100\n10,0,TOTAL,370,9100\n")
    summary = {"rmw": "rmw_fastrtps_cpp", "camera_topic": "/base_camera/color/image_raw",
               "rtf": [{"rtf": 0.1}, {"rtf": 0.2}],
               "participants": [{"count": 44, "max_index": 43}],
               "qos": {"/odom": {"publishers": [{"node": "/base_state_adapter"}]}},
               "late_view": {"_late_probe": {"topics": {"/odom": {"publisher_nodes": []}}}},
               "topics": {"/odom": {"streams": {"": {"unique_stamps": 90, "expected": 100, "ratio": 0.9,
                                                     "latency_wall_s": {"p50": 0.1, "p95": 0.3}, "latency_sim_s": {"p95": 0.02}}}}}}
    (cell / "observer_summary.json").write_text(json.dumps(summary))
    rows = [compare.run_row(str(tmp_path / "dds-benchmark-fastdds_default-r1-attempt1"))]
    assert rows[0]["phases"] == {"rows": 2, "passed": 1, "first_not_passed": "nav_to_station (FAIL)"}
    assert rows[0]["processes"]["total_cpu_mean"] == 360.0 and rows[0]["processes"]["total_mem_peak_mb"] == 9100.0
    assert rows[0]["late_missing"] == {"/odom": ["base_state_adapter"]}
    assert rows[0]["topics"]["/odom"].startswith("90/100 (0.900)")
    assert rows[0]["topics"]["CAMERA"] == "not received"
    text = compare.markdown(rows)
    assert "| dds-benchmark-fastdds_default-r1-attempt1 | rmw_fastrtps_cpp | 1/2 | nav_to_station (FAIL) |" in text
