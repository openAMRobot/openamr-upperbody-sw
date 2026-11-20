# SPDX-License-Identifier: MIT
"""--dry-run sends nothing: no publish, no action client, no goal; every intended send is logged to the session instead.
rclpy and the message packages are replaced by stubs, so this runs without ROS."""
import sys
import types

import pytest


class Recorder:
    def __init__(self):
        self.calls = []


REC = Recorder()


def _stub_modules(monkeypatch):
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        monkeypatch.setitem(sys.modules, name, m)
        return m

    class ActionClient:
        def __init__(self, *a, **k):
            REC.calls.append(("ActionClient", a))

    class Msg:
        def __init__(self, *a, **k):
            pass

    mod("rclpy", init=lambda: None, spin_once=lambda *a, **k: None, create_node=lambda *a, **k: None, shutdown=lambda: None)
    mod("rclpy.action", ActionClient=ActionClient)
    mod("rclpy.parameter", Parameter=Msg)
    mod("rclpy.qos", DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1), QoSProfile=Msg, ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1))
    mod("builtin_interfaces"); mod("builtin_interfaces.msg", Duration=Msg)
    mod("control_msgs"); mod("control_msgs.action", FollowJointTrajectory=Msg, GripperCommand=Msg)
    mod("geometry_msgs"); mod("geometry_msgs.msg", Twist=Msg)
    mod("nav_msgs"); mod("nav_msgs.msg", Odometry=Msg)
    mod("sensor_msgs"); mod("sensor_msgs.msg", JointState=Msg)
    mod("std_msgs"); mod("std_msgs.msg", String=Msg)
    mod("trajectory_msgs"); mod("trajectory_msgs.msg", JointTrajectoryPoint=Msg)
    monkeypatch.delitem(sys.modules, "rosio", raising=False)


class Pub:
    def __init__(self):
        self.n = 0

    def publish(self, msg):
        self.n += 1


@pytest.mark.parametrize("target", ["sim", "robot"])
def test_dry_run_sends_nothing(tmp_path, monkeypatch, target):
    _stub_modules(monkeypatch)
    import common
    import rosio
    monkeypatch.setattr(common, "config_id", lambda *a, **k: (None, "stub", ""))
    REC.calls.clear()
    p = common.base_parser("x", robot_required=[("--cmd-vel-topic", "t")])
    argv = ["--target", target, "--dry-run", "--session-root", str(tmp_path)] + (["--cmd-vel-topic", "/c", "--operator", "op"] if target == "robot" else [])
    a = common.parse(p, argv)
    s = common.Session("t_dry", a)
    io = rosio.Io.__new__(rosio.Io)                 # no node: nothing to talk to
    io.s, io._goals, io._twist_pubs = s, [], []
    pub = Pub()
    io._twist_pubs.append(pub)
    monkeypatch.setattr(common, "confirm", lambda *a, **k: pytest.fail("dry-run must not ask the operator"))
    assert io.drive(pub, "/c", 0.5, 0.0, 5.0, "drive") is None
    assert io.gripper("left", 0.4, 50.0, "grip") == {"sent": False}
    r = io.follow_joint_trajectory("/left_arm_controller/follow_joint_trajectory", ["j"], [([0.1], 2.0)], "arm")
    assert r[0] is None and "dry-run" in r[2]
    io.zero_twist()
    io.cancel_all()
    assert pub.n == 0
    assert REC.calls == []
    assert [x["what"] for x in s.sent] == ["drive", "grip", "arm"]
    assert all(x["dry_run"] for x in s.sent)
    # the caps still apply to what would have been sent
    assert s.sent[0]["payload"]["linear_x"] == 0.2 and s.sent[1]["payload"]["max_effort"] == 5.0
    assert (tmp_path / s.dir.split("/")[-1] / "sent.jsonl").exists()
