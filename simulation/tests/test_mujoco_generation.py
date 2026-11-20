# SPDX-License-Identifier: MIT
"""The generated MJCF: what it holds, what it lists as unsupported, and that it is a build product."""
import pathlib
import subprocess

import yaml

from openamr_sim.mjcf import FREE_JOINT

REPO = pathlib.Path(__file__).resolve().parents[2]


def test_actuators_match_the_controller_interfaces(models):
    """One actuator per command interface the controllers claim (arm_controllers.yaml) plus the base wheels (DiffDrive, velocity)."""
    cfg = yaml.safe_load((REPO / "ros2/src/openamrobot_upperbody_control/config/arm_controllers.yaml").read_text())
    expected = set()
    for name, block in cfg.items():
        if name == "controller_manager":
            continue
        p = block["ros__parameters"]
        joints = p.get("joints") or [p["joint"]]
        interfaces = p.get("command_interfaces", ["position"])  # GripperActionController in position_controllers commands position
        expected |= {f"{j}_{i}" for j in joints for i in interfaces}
    expected |= {"left_wheel_joint_velocity", "right_wheel_joint_velocity"}
    for m in models.values():
        assert {m.actuator(a).name for a in range(m.nu)} == expected


def test_counts_and_names(generated, models):
    for name, g in generated.items():
        m, r = models[name], g["report"]
        assert r["counts"]["bodies"] == r["counts"]["urdf_links"] == 67
        assert m.joint(0).name == FREE_JOINT
        assert r["counts"]["equalities"] == 2
        assert r["counts"]["keyframes"] == 9 and r["keyframes"][0] == "L1000_home"
        assert r["counts"]["cameras"] == 4


def test_unsupported_semantics_are_listed(generated):
    for g in generated.values():
        items = {u["item"] for u in g["report"]["unsupported"]}
        for required in ("joint velocity limit", "gazebo plugin", "collision mesh", "drive wheel anisotropic friction", "mimic joint",
                         "self collision", "DAE visual materials", "castor joints", "wheel torque limit", "lift velocity limit",
                         "camera field of view", "ros2_control system"):
            assert required in items, required
        velocity = [u for u in g["report"]["unsupported"] if u["item"] == "joint velocity limit"]
        assert len(velocity) == 19  # 18 OpenArm joints and lift_joint carry <limit velocity>


def test_generated_file_says_do_not_edit(generated):
    for g in generated.values():
        head = g["xml"].read_text().splitlines()[0]
        assert "Do not edit; do not commit." in head and g["report"]["expansion_sha256"] in head


def test_no_generated_backend_file_is_tracked():
    tracked = subprocess.run(["git", "-C", str(REPO), "ls-files"], capture_output=True, text=True, check=True).stdout.split()
    bad = [f for f in tracked if f.endswith((".usd", ".usda", ".usdc", ".mjb")) or
           (f.endswith(".xml") and "openamrobot_bp0" in f) or f.startswith("build/")]
    assert not bad, bad
    ignore = (REPO / ".gitignore").read_text()
    assert "/build/" in ignore.splitlines() or "build/" in ignore.splitlines()


def test_mesh_conversions(generated):
    for g in generated.values():
        assert set(g["report"]["meshes_converted"]) <= {"dae_to_obj", "mirror_applied", "dae_to_obj+mirror_applied"}
        assert g["report"]["meshes_converted"], "the expansion has DAE visuals and mirrored arm meshes"


def test_base_camera_is_the_gemini_336l(models, overrides):
    """C9's `base` stream is the Orbbec Gemini 336L on base_camera_link (platform-sw sensors.xacro), not the legacy camera_link
    (not 2.0 hardware). Its vertical field of view is the datasheet's 68 degrees (Gemini 330 Series Datasheet V1.6, Table 3-4-5)."""
    spec = overrides["cameras"]["base_camera"]
    assert spec["frame"] == "base_camera_color_optical_frame"
    assert spec["fovy"]["status"] == "datasheet" and spec["fovy"]["value"] == 68.0
    assert overrides["sources"][spec["fovy"]["source"]]["version"] == "V1.6"
    for name, m in models.items():
        cam = m.camera("base_camera").id
        assert m.body(m.cam_bodyid[cam]).name == "base_camera_color_optical_frame", name
        assert m.cam_fovy[cam] == 68.0, name
