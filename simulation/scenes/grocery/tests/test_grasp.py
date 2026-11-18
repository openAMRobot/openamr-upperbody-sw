# SPDX-License-Identifier: MIT
"""The grasp-and-lift experiment: it lifts with the Build 1 settings, it detects a drop, and it never changes the model file.

MuJoCo experiment, not ROS or firmware integrated. Needs GROCERY_ROBOT_MJCF (generate.py output).
"""
import hashlib

import pytest

import pick


def _run(scene_xml, **settings):
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(scene_xml))
    return pick.Pick(m, mujoco.MjData(m), settings=pick.Settings(**settings)).run()


@pytest.fixture(scope="module")
def baseline(scene_xml):
    return _run(scene_xml)


def test_build1_settings_lift_and_hold_the_product(baseline):
    """Friction 1.0, close command 0, the generated gripper gain: the product rises with the hand and stays in the fingers."""
    r = baseline
    assert r["held"], {k: r[k] for k in ("grasp_rise", "product_rise", "slip_z_max")}
    assert r["lift_travel"] == pytest.approx(0.06, abs=2e-3)       # 18 mm/s for 3.33 s
    assert r["product_rise"] > 0.04
    assert r["slip_z_max"] < 0.005
    assert r["normal_force_finger1_held_mean"] > r["static_need_per_finger"]
    assert r["normal_force_finger2_held_mean"] > r["static_need_per_finger"]
    assert r["hand_other_contact_max"] == 0.0                     # only the finger pads touch the product
    assert r["approach_start_offset"] < 0.03


def test_the_lift_follows_the_decided_speed(baseline):
    s = baseline["series"]
    lift = [(t, q) for t, q, ph in zip(s["t"], s["lift_q"], s["phase"]) if ph == "lift"]
    (t0, q0), (t1, q1) = lift[len(lift) // 4], lift[3 * len(lift) // 4]
    assert (q1 - q0) / (t1 - t0) == pytest.approx(0.018, rel=0.03)


def test_low_friction_drops_the_product(scene_xml):
    """Negative control: at friction 0.2 the same grip does not hold the product. Without it, 'held' could be a constant."""
    r = _run(scene_xml, friction=0.2)
    assert not r["held"]
    assert r["product_rise"] < 0.01


def test_the_experiment_does_not_change_the_model_files(scene_xml, robot_mjcf, composed):
    out, manifest = composed
    before = hashlib.sha256((out / manifest["robot_copy"]).read_bytes()).hexdigest()
    _run(scene_xml, friction=0.5, grip_command=0.2, kp_factor=2.0)
    assert hashlib.sha256((out / manifest["robot_copy"]).read_bytes()).hexdigest() == before == manifest["robot_source_sha256"]
    assert hashlib.sha256(robot_mjcf.read_bytes()).hexdigest() == before
