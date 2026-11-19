# SPDX-License-Identifier: MIT
"""The HTML report from plain message objects (the ROS field names), without ROS or a bag."""
import math
import re
from types import SimpleNamespace as NS

from openamrobot_control_room import bag, report

EM_DASH = chr(0x2014)


def stamp(t):
    return NS(stamp=NS(sec=int(t), nanosec=int((t % 1) * 1e9)))


def pose(x, y, yaw, z=0.0):
    return NS(position=NS(x=x, y=y, z=z), orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))


def messages():
    for k in range(1, 101):
        t = k * 0.5
        yield "/clock", NS(clock=NS(sec=int(t), nanosec=int((t % 1) * 1e9))), k
        yield "/build1/ground_truth/odom", NS(header=stamp(t), pose=NS(pose=pose(0.02 * k, 0.0, 0.0))), k
        yield "/amcl_pose", NS(header=stamp(t), pose=NS(pose=pose(0.02 * k + 0.01, 0.0, 0.02))), k
        yield "/joint_states", NS(header=stamp(t), name=["lift_joint", "openarm_left_joint1"], position=[0.001 * k, 0.01 * k]), k
        yield "/sil/gz/joint_states", NS(header=stamp(t), name=["lift_joint"], position=[0.001 * k]), k
        yield "/build1/rtf_stats", NS(data='{"rtf_last": 0.7}'), k
        yield "/build1/contacts/openarm_left_link7", NS(header=stamp(t), contacts=[
            NS(wrenches=[NS(body_1_wrench=NS(force=NS(x=3.0, y=4.0, z=0.0)))])]), k


def test_extract():
    series = report.extract(messages(), station=(2.0, 0.0, 0.0))
    assert len(series["station_error"]) == 100 and math.isclose(series["station_error"][-1][1], 0.0, abs_tol=1e-9)
    assert math.isclose(series["station_along"][0][1], -1.98, abs_tol=1e-9)
    assert all(math.isclose(v, 0.01, abs_tol=1e-9) for _, v in series["loc_error"])
    assert len(series["lift_js"]) == 100 and len(series["lift_physics"]) == 100
    assert len(series["joints"]["openarm_left_joint1"]) == 100
    assert series["rtf"][0] == (0.5, 0.7)
    assert series["contacts"]["openarm_left_link7"][0][1] == 5.0
    assert series["sim_span"] == (0.5, 50.0)


def test_candump_rates():
    rates = report.candump_rates(bag.synthetic_candump(lines_per_bus=100))
    assert set(rates) == {"vcan_drive", "vcan_aux", "vcan_bms", "vcan_arm_l", "vcan_arm_r"}
    assert rates["vcan_drive"][0][1] == 20.0 and rates["vcan_aux"][0][1] == 40.0


def test_render_single_static_page():
    series = report.extract(messages(), station=(2.0, 0.0, 0.0))
    page = report.render(series, bag.synthetic_mission(), report.candump_rates(bag.synthetic_candump()))
    assert page.startswith("<!DOCTYPE html>") and page.rstrip().endswith("</html>")
    for chart in ("station", "loc", "loc_yaw", "joints_left", "lift", "rtf", "can", "contacts"):
        assert re.search(rf'<svg id="{chart}" class="chart"', page), chart
    assert page.count('class="phase pass"') == 7 and page.count('class="phase fail"') == 1
    assert "retreat_along_heading" in page
    # One self-contained file: nothing is loaded from elsewhere.
    assert not re.search(r'(src|href)="(https?:)?//', page)
    assert EM_DASH not in page


def test_render_without_data_says_so():
    page = report.render(report.extract([]), None)
    assert "No data for this plot in the recording." in page and "No phases in the mission JSON." in page


def test_synthetic_run_without_ros():
    series = report.extract(bag.synthetic_messages(bag.plain_types()), station=bag.SYNTHETIC_STATION)
    assert len(series["station_error"]) == 801 and series["station_error"][-1][1] > 0.0
    assert min(v for _, v in series["station_error"]) < 0.02          # the base reaches the station
    assert max(v for _, v in series["lift_physics"]) == 0.175
    assert len(series["rtf"]) == 81 and len(series["loc_error"]) == 161


def test_decoded_can_and_components_from_a_recorded_run():
    """The decoded CAN sections and the components table from grocery run 44299879412's recorded frames, run_info and
    mission JSON (test/fixtures/runs): every chart has a plotted line, the lift commands of the run are listed."""
    import glob
    import json
    import os
    from conftest import package_path
    base = package_path("test", "fixtures", "runs", "grocery_44299879412")
    lines = {os.path.basename(p)[len("candump_"):-len(".log")]: open(p, encoding="utf-8").readlines()
             for p in glob.glob(os.path.join(base, "candump_*.log"))}
    decoded = report.candump_decoded(lines)
    assert [(c["seq"], c["op"], c["stop"]) for c in decoded["commands"]] == [
        (1, "MOVE_TO_STOP", "L1175"), (2, "MOVE_TO_STOP", "L1175"), (3, "MOVE_TO_STOP", "L1000")]
    series = report.extract(messages(), station=(2.0, 0.0, 0.0))
    with open(os.path.join(base, "messages.json"), encoding="utf-8") as f:
        series["run_info"] = json.load(f)["/build1/run_info"]["samples"][0]["value"]
    with open(os.path.join(base, "mission.json"), encoding="utf-8") as f:
        mission = json.load(f)
    page = report.render(series, mission, decoded=decoded)
    for chart in ("can_drive", "can_lift", "can_arm", "can_bms"):
        match = re.search(rf'<svg id="{chart}" class="chart".*?</svg>', page, re.S)
        assert match and "<polyline" in match.group(0), f"{chart} has no plotted line"
    assert "host test stand-in (openamrobot_sil stm32_host_client), not firmware" in page
    assert "none launched by build1.launch.py" in page and "seq 3 MOVE_TO_STOP L1000" in page
    assert page.count("<tr><td>vcan_") == len(decoded["raw"]) and "<td>190</td>" in page
    assert EM_DASH not in page


def test_report_without_candump_says_so():
    page = report.render(report.extract(messages()), None)
    assert "No candump logs were given" in page
