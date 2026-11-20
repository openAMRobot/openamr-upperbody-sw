# SPDX-License-Identifier: MIT
"""The monitor's arithmetic, without ROS."""
import math

from openamrobot_control_room import state


def test_pose_error():
    error, yaw = state.pose_error((1.03, 2.04, 0.1), (1.0, 2.0, -0.05))
    assert math.isclose(error, 0.05, abs_tol=1e-9) and math.isclose(yaw, 0.15, abs_tol=1e-9)
    assert state.pose_error(None, (0, 0, 0)) == (None, None)
    _, wrapped = state.pose_error((0, 0, math.pi - 0.01), (0, 0, -math.pi + 0.01))
    assert math.isclose(wrapped, 0.02, abs_tol=1e-9)


def test_station_error_along_and_left():
    # Station facing +y: 0.1 m further along +y is +0.1 along the heading, 0.05 m towards -x is +0.05 to its left.
    err = state.station_error((1.95, 1.1, math.pi / 2), (2.0, 1.0, math.pi / 2))
    assert math.isclose(err["along_m"], 0.1, abs_tol=1e-9) and math.isclose(err["left_m"], 0.05, abs_tol=1e-9)
    assert math.isclose(err["distance_m"], math.hypot(0.1, 0.05), abs_tol=1e-9)


def test_action_watch():
    watch = state.ActionWatch()
    watch.update("/navigate_to_pose", [("a", 2)])
    watch.update("/backup", [("b", 4)])
    assert watch.active() == ["navigating (Nav2)"]
    watch.update("/navigate_to_pose", [("a", 4)])
    assert watch.active() == [] and watch.last_terminal == {"/navigate_to_pose": "SUCCEEDED", "/backup": "SUCCEEDED"}
    assert watch.summary()["/backup"] == {"active": 0, "last_result": "SUCCEEDED"}


def test_motion_and_contacts():
    motion = state.Motion()
    motion.update(1.0, (0.0, 0.0, 0.8))
    motion.update(1.5, (0.0, 0.0, 0.85))
    s = motion.summary()
    assert math.isclose(s["speed_mps"], 0.1, abs_tol=1e-6) and math.isclose(s["displacement_m"], 0.05, abs_tol=1e-6)
    assert state.contact_force([[(3.0, 4.0, 0.0)], [(1.0, 0.0, 0.0), (1.0, 0.0, 0.0)]]) == 5.0
    assert state.contact_force([]) == 0.0
    contacts = state.Contacts()
    contacts.update("/build1/contacts/openarm_left_link7", 1, 2.0)
    contacts.update("/build1/contacts/openarm_left_link7", 1, 3.0)
    contacts.update("/build1/contacts/openarm_left_link7", 0, 0.0)
    contacts.update("/build1/contacts/openarm_left_link7", 2, 1.0)
    assert contacts.summary() == {"topics": 1, "events": 2, "in_contact": ["openarm_left_link7"], "max_force_n": 3.0}


def test_unsupported_telemetry_is_unknown_not_zero():
    """A value the run does not provide is None, never a made-up 0: one item pose gives no speed, and without a contact
    sensor topic (BUILD1_CONTACT_SENSORS unset, as in run 44476577635) there is no contact force or event count."""
    motion = state.Motion()
    assert motion.summary() is None
    motion.update(1.0, (0.0, 0.0, 0.8))
    assert motion.summary()["speed_mps"] is None
    motion.update(1.0, (0.0, 0.0, 0.9))      # same time stamp: still no speed
    assert motion.summary()["speed_mps"] is None
    empty = state.Contacts().summary()
    assert empty["topics"] == 0
    assert empty["max_force_n"] is None and empty["events"] is None and empty["in_contact"] is None
    assert "BUILD1_CONTACT_SENSORS" in empty["unknown"]


def test_scalars_keeps_only_scalar_fields():
    assert state.scalars({"a": 1, "b": [1, 2], "c": "x", "d": {"e": 1}, "f": None}) == {"a": 1, "c": "x", "f": None}
    assert state.scalars(None) is None
