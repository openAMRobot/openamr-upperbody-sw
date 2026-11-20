# SPDX-License-Identifier: MIT
"""can_decode.py on frames recorded in two real simulation runs (test/fixtures/runs, cut by
.github/workflows/control-room-evidence.yml from the uploaded candump logs), checked against what the same runs
recorded on their ROS status topics and against the pinned driver's own constants. No ROS, no vcan.

Baseline 1 acceptance run 44309817813 (all eight phases passed) and grocery run 44299879412. The status topics are
another path through the same emulators (the host stand-in decodes the same frames), so agreement shows the decoder
reads the frames as the stack does; it says nothing about a real device.
"""
import collections
import json
import os

import pytest

from conftest import package_path
from openamrobot_control_room import can_decode as cd

RUNS = ("baseline1_44309817813", "grocery_44299879412")
# openarm_ros2 4e837e1 (pins.yaml openarm_ros2), openarm_simple_hardware.hpp: kp_, kd_, GRIPPER_KP, GRIPPER_KD.
DRIVER_KP = (70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0, 5.0)
DRIVER_KD = (2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5, 0.1)


def frames(run, bus):
    with open(package_path("test", "fixtures", "runs", run, f"candump_{bus}.log"), encoding="utf-8") as f:
        parsed = [cd.parse_candump(line) for line in f]
    assert parsed and all(parsed), f"{run} {bus}: a line did not parse"
    return [(t, b, i, d, ext, cd.decode(bus, i, d, ext)) for t, b, i, d, ext, _fd in parsed]


def messages(run):
    with open(package_path("test", "fixtures", "runs", run, "messages.json"), encoding="utf-8") as f:
        return json.load(f)


def selection(run):
    with open(package_path("test", "fixtures", "runs", run, "selection.json"), encoding="utf-8") as f:
        return json.load(f)


@pytest.mark.parametrize("run", RUNS)
@pytest.mark.parametrize("bus", cd.ARM_SIDE.keys() | {"vcan_drive", "vcan_aux", "vcan_bms"})
def test_every_recorded_frame_decodes(run, bus):
    decoded = frames(run, bus)
    assert len(decoded) == selection(run)["buses"][bus]["frames_kept"]
    unknown = [(hex(i), r) for _t, _b, i, _d, _e, r in decoded if r["kind"] in ("not decoded", "unknown bus")]
    assert unknown == [], unknown[:5]
    assert all(b == bus for _t, b, *_ in decoded)


@pytest.mark.parametrize("run", RUNS)
def test_drive_frames(run):
    decoded = [r for *_, r in frames(run, "vcan_drive")]
    kinds = collections.Counter((r["kind"], r.get("index")) for r in decoded)
    # The host stand-in's cycle (stm32_host.py DriveHost.step): write 60FFh:1 and :2, read 606Ch:1 and :2.
    assert kinds[("SDO write", "60FFh")] and kinds[("SDO read reply", "606Ch")]
    statuswords = {r["value"] for r in decoded if r["kind"] == "SDO read reply" and r["index"] == "6041h"}
    assert statuswords == {55}
    assert [r["bits"] for r in decoded if r.get("index") == "6041h" and "bits" in r][0] == \
        ["ready", "switched on", "operation enabled", "voltage"]
    assert {r["value"] for r in decoded if r["kind"] == "SDO read reply" and r["index"] == "603Fh"} == {0}
    heartbeats = {r["nmt_state"] for r in decoded if r["kind"] == "heartbeat"}
    assert heartbeats == {"operational"}
    moving = [r for r in decoded if r["kind"] == "SDO write" and r["index"] == "60FFh" and r["value"]]
    assert moving, "the window after the first nonzero target holds a moving command"
    for r in moving:
        assert r["wheel"] in ("left", "right")
        assert r["wheel_m_s"] == pytest.approx(r["rpm"] * 2 * 3.141592653589793 / 60 * 0.098, abs=1e-4)


def test_drive_status_topic_agrees():
    """grocery: the host stand-in published the same statusword and fault code on /sil/drive/status."""
    samples = messages("grocery_44299879412")["/sil/drive/status"]["samples"]
    assert {s["value"]["statusword"] for s in samples} == {55}
    assert {s["value"]["fault_code"] for s in samples} == {0}


@pytest.mark.parametrize("run", RUNS)
def test_lift_frames_follow_the_recorded_commands(run):
    decoded = [r for *_, r in frames(run, "vcan_aux")]
    commands = []
    for r in decoded:
        if r["kind"] == "RPDO1 command" and (not commands or commands[-1][0] != r["seq"]):
            commands.append((r["seq"], r["op"], r["stop_id"]))
    recorded = [s["value"]["data"] for s in messages(run)["/sil/lift/command"]["samples"]]
    # Each /sil/lift/command (0, 1, 2: move to that stop) became one new RPDO1 sequence number with that stop id.
    assert [(op, stop) for _seq, op, stop in commands] == [("MOVE_TO_STOP", value) for value in recorded]
    assert [seq for seq, *_ in commands] == list(range(1, len(recorded) + 1))
    states = {r["state"] for r in decoded if r["kind"] == "TPDO1 status"}
    assert "IDLE" in states and "MOVING" in states, states
    first = next(r for r in decoded if r["kind"] == "TPDO1 status")
    assert first["position_mm"] == 5.0 and first["flags_raw"] == 67
    assert first["flags"] == ["position valid", "at stop", "motor power"]
    permits = [r for r in decoded if r["kind"] == "permit PDO"]
    assert permits and {r["permit"] for r in permits} <= {True, False}
    assert not [r for r in decoded if r["kind"] == "TPDO2 inputs and drive" and r["faults"]]


def test_lift_status_topic_agrees():
    """grocery: the first /sil/lift/status (the host stand-in's decode of TPDO1 and TPDO2) has the same values."""
    first = messages("grocery_44299879412")["/sil/lift/status"]["samples"][0]["value"]
    decoded = [r for *_, r in frames("grocery_44299879412", "vcan_aux")]
    tpdo1 = next(r for r in decoded if r["kind"] == "TPDO1 status")
    tpdo2 = next(r for r in decoded if r["kind"] == "TPDO2 inputs and drive")
    assert (first["position_mm"], first["flags"], first["state"]) == (tpdo1["position_mm"], tpdo1["flags_raw"],
                                                                     cd.LIFT_STATES.index(tpdo1["state"]))
    assert first["inputs"] == 17 and tpdo2["inputs"] == ["stop sensor L1000", "Hall supply good"]


@pytest.mark.parametrize("run", RUNS)
def test_bms_frames_match_the_bms_status(run):
    decoded = [r for *_, r in frames(run, "vcan_bms")]
    replies = [r for r in decoded if r["kind"].startswith("reply")]
    requests = [r for r in decoded if r["kind"].startswith("request")]
    assert replies and requests
    assert {r["data_id"] for r in requests} == {"90h", "93h", "98h"}      # BmsHost.POLLED
    soc = {(r["pack_voltage_v"], r["current_a"], r["soc_percent"]) for r in replies if r["data_id"] == "90h"}
    emulator = messages(run)["/sil/bms/emulator_state"]["samples"]
    assert soc == {(s["value"]["pack_voltage_v"], s["value"]["current_a"], s["value"]["soc_percent"]) for s in emulator}
    assert {r["any_failure"] for r in replies if r["data_id"] == "98h"} == {False}


def test_bms_status_topic_agrees():
    status = messages("grocery_44299879412")["/sil/bms/status"]["samples"]
    decoded = [r for *_, r in frames("grocery_44299879412", "vcan_bms")]
    remaining = {r["remaining_mah"] for r in decoded if r["data_id"] == "93h" and "remaining_mah" in r}
    assert remaining == {s["value"]["remaining_mah"] for s in status} == {67200}


@pytest.mark.parametrize("run", RUNS)
@pytest.mark.parametrize("bus", sorted(cd.ARM_SIDE))
def test_arm_frames(run, bus):
    side = cd.ARM_SIDE[bus]
    decoded = [r for *_, r in frames(run, bus)]
    commands = [r for r in decoded if r["kind"] == "MIT command"]
    feedback = [r for r in decoded if r["kind"] == "state feedback"]
    refresh = [r for r in decoded if r["kind"] == "config 7FFh"]
    assert commands and feedback and refresh
    assert {r["command"] for r in refresh} == {"refresh (state request)"}
    assert {r["motor"] for r in commands} == set(range(1, 9)) == {r["motor"] for r in feedback}
    for r in commands:
        # The unmodified driver sends its default gains; 12-bit encoding truncates by at most one step.
        i = r["motor"] - 1
        assert DRIVER_KP[i] - 500 / 4095 <= r["kp"] <= DRIVER_KP[i] + 1e-9, r
        assert DRIVER_KD[i] - 5 / 4095 <= r["kd"] <= DRIVER_KD[i] + 1e-9, r
        assert r["joint"] == (f"openarm_{side}_joint{r['motor']}" if r["motor"] < 8 else f"openarm_{side}_gripper")
    for r in feedback:
        assert r["id_nibble"] == r["motor"] and r["status"] == "enabled" and not r["fault"], r
        assert -12.5 <= r["q_rad"] <= 12.5
    # One cycle: 8 refresh requests, 8 MIT commands, 16 state replies (the counts of the whole recording).
    counts = selection(run)["buses"][bus]["id_counts"]
    assert counts["7FF"] == 8 * counts["001"] and counts["011"] == pytest.approx(2 * counts["001"], abs=2)


def test_special_and_config_frames():
    assert cd.decode("vcan_arm_l", 0x003, bytes.fromhex("FFFFFFFFFFFFFFFC"))["command"] == "enable"
    reply = cd.decode("vcan_arm_r", 0x012, bytes.fromhex("0200330A01000000"))
    assert reply["kind"] == "register reply" and reply["rid"] == 10 and reply["raw"] == 1
    abort = cd.decode("vcan_drive", 0x581, bytes.fromhex("80FF600111000906"))
    assert abort["kind"] == "SDO abort" and abort["abort_text"] == "sub-index does not exist"
    emcy = cd.decode("vcan_aux", 0x090, bytes.fromhex("07FF91074000" + "0000"))
    assert emcy["fault"] == "F07" and emcy["code"] == "FF07h"


def test_latest_keeps_one_value_per_signal():
    latest = cd.Latest()
    for t, bus, i, d, ext, _r in frames("grocery_44299879412", "vcan_aux"):
        latest.add(bus, i, d, ext, t)
    values = latest.summary()["decoded"]["vcan_aux"]
    assert {"TPDO1 status", "TPDO2 inputs and drive", "permit PDO", "RPDO1 command", "CTL-017 heartbeat",
            "core (STM32 slot) heartbeat"} <= set(values)
    assert set(latest.summary()["raw_last_by_id"]["vcan_aux"]) == {"190", "210", "290", "410", "701", "710"}


def test_fixture_provenance_file():
    assert os.path.exists(package_path("test", "fixtures", "runs", "README.md"))


def test_motor_table_is_the_emulators():
    """The decoder's motor types (they set the q, dq and tau scales) are the emulator's, in id order, for both arms.
    The same order is in the pinned driver (openarm_simple_hardware.hpp, joints 1 to 7 DM8009 DM8009 DM4340 DM4340
    DM4310 DM4310 DM4310, gripper DM4310)."""
    import yaml
    from conftest import REPOSITORY
    path = os.path.join(REPOSITORY, "ros2", "src", "openamrobot_openarm_emulator", "config", "sim_controllers.yaml")
    with open(path, encoding="utf-8") as f:
        config = yaml.safe_load(f)
    tables = [v["ros__parameters"]["motor_types"] for v in config.values()
              if isinstance(v, dict) and "motor_types" in v.get("ros__parameters", {})]
    assert len(tables) == 2 and all(list(t) == list(cd.MOTOR_TYPES) for t in tables), tables
    for kind, (p, v, t) in cd.ENCODING.items():   # openamrobot_openarm_emulator src/codec.cpp encoding_limits
        assert p == 12.5 and (kind, v, t) in (("DM4310", 30.0, 10.0), ("DM4340", 10.0, 28.0), ("DM8009", 45.0, 54.0))
