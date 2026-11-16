# SPDX-License-Identifier: MIT
"""Structural parity of the MuJoCo model with the pinned expansion, at the named configurations and lift stops.

Topology, limits, axes, masses, centres of mass, inertias, frames, visual and collision geometry, dynamics, actuators, mimic.
Tolerances: simulation/parity_tolerances.yaml. The mutation tests show that a wrong value fails.
"""
import mujoco
import pytest

from openamr_sim import parity


@pytest.fixture(scope="module")
def results(models, generated, mesh_loader, evidence):
    out = {}
    for name, m in models.items():
        p = parity.MujocoParity(m, generated[name]["expansion"].text)
        rep = p.run(parity.named_configurations(p.u), mesh_loader)
        out[name] = rep
        evidence(f"parity_mujoco_{name}", {"summary": rep.summary(), "failures": rep.failures[:50]})
    return out


@pytest.mark.parametrize("name", ["bp000", "bp050"])
def test_parity(results, name):
    rep = results[name]
    assert rep.checks > 2900
    assert not rep.failures, rep.failures[:10]


@pytest.mark.parametrize("name", ["bp000", "bp050"])
def test_every_parameter_class_is_checked(results, name):
    by = results[name].by_parameter
    for prefix in ("topology.", "limits.", "frames.joint_axis", "inertial.mass", "inertial.com", "inertial.inertia", "frames.position",
                   "frames.rotation", "geometry.mesh_bounds", "geometry.size", "dynamics.damping", "dynamics.frictionloss",
                   "dynamics.armature", "actuators.kp", "mimic.polycoef", "frames.camera_position"):
        assert any(k.startswith(prefix) and v["checks"] > 0 for k, v in by.items()), prefix


def test_configurations_cover_every_lift_stop(generated):
    p = parity.MujocoParity(mujoco.MjModel.from_xml_path(str(generated["bp000"]["xml"])), generated["bp000"]["expansion"].text)
    confs = parity.named_configurations(p.u)
    assert sorted({round(q["lift_joint"], 6) for q in confs.values()}) == [0.0, 0.175, 0.35]
    assert len(confs) == 9


def test_bp050_moves_the_column_50_mm(generated, models):
    for name, dx in (("bp000", 0.0), ("bp050", 0.05)):
        m = models[name]
        d = mujoco.MjData(m)
        mujoco.mj_resetData(m, d)
        mujoco.mj_kinematics(m, d)
        base = d.body("base_link").xpos
        col = d.body("lift_column_base_link").xpos
        ref = models["bp000"]
        d0 = mujoco.MjData(ref)
        mujoco.mj_kinematics(ref, d0)
        assert abs((col[0] - base[0]) - (d0.body("lift_column_base_link").xpos[0] - d0.body("base_link").xpos[0]) - dx) < 1e-9


MUTATIONS = [
    ("inertial.mass", lambda m: m.body_mass.__setitem__(m.body("openarm_left_link3").id, m.body_mass[m.body("openarm_left_link3").id] + 1e-4)),
    ("limits.position", lambda m: m.jnt_range.__setitem__(m.joint("openarm_right_joint4").id, [0.0, 2.44])),
    ("dynamics.armature", lambda m: m.dof_armature.__setitem__(m.jnt_dofadr[m.joint("openarm_left_joint1").id], 0.0)),
    ("dynamics.frictionloss", lambda m: m.dof_frictionloss.__setitem__(m.jnt_dofadr[m.joint("openarm_left_joint5").id], 0.05)),
    ("frames.joint_axis", lambda m: m.jnt_axis.__setitem__(m.joint("openarm_left_joint2").id, [1.0, 0.0, 0.0])),
    ("frames.position", lambda m: m.body_pos.__setitem__(m.body("head_camera_mount_link").id, m.body_pos[m.body("head_camera_mount_link").id] + [0, 0, 1e-5])),
    ("mimic.polycoef", lambda m: m.eq_data.__setitem__((m.equality("openarm_left_finger_joint2_mimic").id, 1), 0.9)),
    ("actuators.kp", lambda m: m.actuator_gainprm.__setitem__((m.actuator("openarm_left_joint1_position").id, 0), 230.0)),
    ("inertial.com", lambda m: m.body_ipos.__setitem__(m.body("lift_carriage_link").id, m.body_ipos[m.body("lift_carriage_link").id] + [1e-5, 0, 0])),
    ("geometry.size", lambda m: m.geom_size.__setitem__(m.geom("left_wheel_link__collision_0").id, [0.098, 0.023, 0])),
]


@pytest.mark.parametrize("parameter,mutate", MUTATIONS, ids=[p for p, _ in MUTATIONS])
def test_mutation_is_detected(generated, mesh_loader, parameter, mutate):
    m = mujoco.MjModel.from_xml_path(str(generated["bp000"]["xml"]))
    mutate(m)
    p = parity.MujocoParity(m, generated["bp000"]["expansion"].text)
    rep = p.run(parity.named_configurations(p.u), mesh_loader)
    assert any(f["parameter"].startswith(parameter) for f in rep.failures), parameter


def test_reference_kinematics_agree_with_pinocchio(generated):
    """The parity reference (urdf_model forward kinematics) agrees with pinocchio's frame placements on the same text."""
    import numpy as np
    import pinocchio as pin
    from openamr_sim.physics import PinocchioGravity
    from openamr_sim.urdf_model import URDF
    text = generated["bp000"]["expansion"].text
    u = URDF(text)
    pg = PinocchioGravity(text)
    data = pg.model.createData()
    for q in parity.named_configurations(u).values():
        pin.framesForwardKinematics(pg.model, data, pg.q_vector(q))
        ref = u.fk(q)
        for f in range(pg.model.nframes):
            name = pg.model.frames[f].name
            if name in u.links and pg.model.frames[f].type == pin.FrameType.BODY:
                assert np.allclose(data.oMf[f].translation, ref[name][:3, 3], atol=1e-9), name
                assert np.allclose(data.oMf[f].rotation, ref[name][:3, :3], atol=1e-9), name
