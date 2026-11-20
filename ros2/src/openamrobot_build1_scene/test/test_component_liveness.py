# SPDX-License-Identifier: MIT
"""A scene is whole-robot evidence only while every component it started still works at its end.

openamr-upperbody-sw #95 run 44513497097: scn02 passed while host_client_drive_thread_alive was False (the drive thread
of the pinned openamrobot_sil had ended on an unexpected SDO reply, fixed by openamr-platform-sw #169). The value was
recorded without a limit, and a failing scene with a dead drive thread was turned into BLOCKED. rig.run now checks the
liveness of the drive host, the BMS host, the lift host and emulator, and the arm joint states, and fails the scene
when one has stopped; in the faults profile they are recorded, not checked. The rig is driven here with a stand-in of
its topics and a fake wall clock; no ROS graph runs. Functional simulation checks, not safety evidence.
"""

import pytest

from openamrobot_build1_scene import rig as rig_module   # rclpy: the static tests run with ROS 2 Jazzy sourced


class FakeTime:
    """Wall clock that advances only when the rig sleeps."""

    def __init__(self, real):
        self.real, self.t = real, 1000.0

    def monotonic(self):
        return self.t

    def sleep(self, s):
        self.t += s

    def __getattr__(self, name):
        return getattr(self.real, name)


def make_rig(alive, profile="firmware_sil", lift=True, arms=True):
    """A Rig subclass whose topics advance for the components named in `alive`."""

    class Fake(rig_module.Rig):
        def __init__(self, scenario):
            self.log = rig_module.Log(scenario)
            self.devices = True
            self.plant_switches = 0
            self.calls = 0

        def wait_ready(self, **ready):
            self.ready_components = {"lift": lift, "arms": arms}
            return 1.0

        def get(self, topic):
            self.calls += 1
            n = self.calls
            moving = {"/sil/drive/status": ("samples", "drive"), "/sil/bms/status": ("answers", "bms"),
                      "/sil/lift/emulator_state": ("permit_rx_count", "lift")}
            if topic == "/build1/run_info":
                return {"profile": profile}
            if topic == "/build1/rtf_stats":
                return {}
            if topic in moving:
                field, name = moving[topic]
                state = {field: n if name in alive else 7}
                if topic == "/sil/lift/emulator_state":
                    state["permit_gate"] = True
                return state
            return None

        def count(self, topic):
            self.calls += 1
            if topic == "/joint_states":
                return self.calls if "arms" in alive else 3
            return 0

        def close(self):
            pass
    return Fake


ALL = {"drive", "bms", "lift", "arms"}


@pytest.fixture
def run(monkeypatch, tmp_path):
    monkeypatch.setenv("BUILD1_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(rig_module, "time", FakeTime(rig_module.time))

    def go(alive, **kw):
        monkeypatch.setattr(rig_module, "Rig", make_rig(alive, **kw))
        return rig_module.run("scn_liveness", lambda rig, log: None, needs_devices=True, uses_base=True)
    return go


def test_every_component_alive_passes(run):
    result, reason, _ = run(ALL)
    assert result == "PASS", reason


@pytest.mark.parametrize("dead, check", [
    ("drive", "host_client_drive_thread_alive"),
    ("bms", "host_client_bms_alive"),
    ("lift", "lift_host_and_emulator_alive"),
    ("arms", "arm_joint_states_alive"),
])
def test_a_stopped_component_fails_the_scene(run, dead, check):
    """revert (rig.py at 0b39365): a dead drive thread is recorded without a limit and the scene PASSES; the other
    components are not looked at."""
    result, reason, _ = run(ALL - {dead})
    assert result == "FAIL" and check in reason, reason


def test_a_dead_drive_thread_is_a_failure_not_blocked(run):
    """revert: the old finally block turned a FAIL with a dead drive thread into BLOCKED."""
    result, reason, _ = run(ALL - {"drive"})
    assert result == "FAIL" and "BLOCKED" not in reason


def test_in_the_faults_profile_a_silent_component_is_recorded_not_checked(run):
    result, reason, path = run(ALL - {"drive", "lift"}, profile="faults")
    assert result == "PASS", reason
    text = open(path, encoding="utf-8").read()
    assert '"observed_today"' in text and "host_client_drive_thread_alive" in text


def test_components_the_scene_did_not_start_are_not_checked(run):
    result, reason, _ = run(ALL - {"lift", "arms"}, lift=False, arms=False)
    assert result == "PASS", reason


def test_liveness_reads_work_counters_not_timer_published_status():
    """The lift check uses the emulator's count of the host's permit frames, not the count of /sil/lift/status
    messages, which a node timer publishes even after the lift worker thread has ended."""
    import inspect
    src = inspect.getsource(rig_module.Rig.liveness)
    assert '"permit_rx_count"' in src and '"/sil/lift/status"' not in src
    assert 'self.drive_host_alive()' in src and '"answers"' in src
