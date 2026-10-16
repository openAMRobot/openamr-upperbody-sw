"""Shared by the Gazebo launch tests: where a check's sidecar goes (the same evidence directory the A6 checks use)."""
import os
import sys

from ament_index_python.packages import get_package_prefix

DESCRIPTION = "openamrobot_upperbody_description"


def _evidence_common():
    """evidence_common.py is installed with the description package's scripts; its idea of the evidence directory is the
    only one (OPENAMR_EVIDENCE_DIR, else <workspace>/evidence), so the Gazebo checks use it too instead of a copy."""
    scripts = os.path.join(get_package_prefix(DESCRIPTION), "lib", DESCRIPTION)
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    import evidence_common
    return evidence_common


def launch_defaults():
    """{name: value} of what sim.launch.py defaults to that a result depends on (the physics engine), read from the launch file
    itself so a recorded result says what it was measured with and not what a README says. A test that passes the engine
    explicitly records it separately (write_g_sidecar's physics_engine), so this stays "what the launch does by default"."""
    import importlib.util
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("openamrobot_upperbody_gazebo"), "launch", "sim.launch.py")
    spec = importlib.util.spec_from_file_location("sim_launch_module", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {"physics_engine": module.DEFAULT_PHYSICS_ENGINE, "headless": True}


def launch_engine_constants():
    """(default engine, Bullet-Featherstone engine) as sim.launch.py names them, so a test that must pass the second one
    explicitly does not hard-code a plugin name."""
    import importlib.util
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("openamrobot_upperbody_gazebo"), "launch", "sim.launch.py")
    spec = importlib.util.spec_from_file_location("sim_launch_module", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.DEFAULT_PHYSICS_ENGINE, module.BULLET_FEATHERSTONE_ENGINE


def ros_type_name(name):
    """A ROS type name as the graph prints it (pkg/msg/Type); platform-sw's bridge file already writes it that way."""
    return name


def write_g_sidecar(check_id, slug, name, record, expected=None, physics_engine=None):
    """Write the sidecar of a Gazebo check. FAIL when the test did not record everything it is meant to (a test that failed
    part-way leaves holes), which build_evidence combines with the pytest result. `physics_engine` is the engine a test passed
    explicitly (G02); without it the check ran on the launch's default, which `launch_defaults` records."""
    ev = _evidence_common()
    expected = expected or ("controllers", "arm_goal", "platform_sw_topics", "drive", "odom_to_base_frame", "joint_states")
    missing = [key for key in expected if key not in record]
    result = "FAIL" if missing else "PASS"
    details = {"launch_defaults": launch_defaults(), **record}
    details["physics_engine_used"] = physics_engine or details["launch_defaults"]["physics_engine"]
    details["physics_engine_passed_explicitly"] = physics_engine is not None
    if missing:
        details["missing"] = missing
    return ev.write_sidecar(check_id, slug, name, result, details, directory=ev.evidence_dir(ev.share_dir()))


def amend_g_sidecar(check_id, slug, key, value, fail=False):
    """evidence_common.amend_sidecar for a Gazebo check's sidecar, in the directory write_g_sidecar used."""
    ev = _evidence_common()
    return ev.amend_sidecar(check_id, slug, key, value, fail=fail, directory=ev.evidence_dir(ev.share_dir()))


# --- physical joint angles, read from Gazebo's link poses and not from ROS ----------------------------------------------
# /joint_states cannot answer "did the follower finger really move": for a URDF <mimic> joint joint_state_publisher computes
# the value kinematically from the leader. The link poses are what the physics engine actually did.

def _q_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def _q_inv(q):
    return (-q[0], -q[1], -q[2], q[3])


def first_json_message(text):
    """The first JSON object in `text`. `gz topic -e -n 1 --json-output` is asked for ONE message and has printed two (a second one arrives
    before it exits): json.loads then raised "Extra data" and failed G02 for a reason that has nothing to do with the grippers (1 run in 12,
    P17). The first message is the one that was asked for; whatever follows it is ignored."""
    import json
    stripped = text.lstrip()
    if not stripped:
        raise ValueError("gz topic printed no message")
    first, _ = json.JSONDecoder().raw_decode(stripped)
    return first


def gz_link_poses(world="empty", timeout=20):
    """{link name: {"orientation": {x,y,z,w}, ...}} from one message of the world's dynamic pose topic (gz transport, so the
    GZ_PARTITION of the running simulation must be in this process's environment, as the test sets it)."""
    import subprocess
    out = subprocess.run(["gz", "topic", "-e", "-t", f"/world/{world}/dynamic_pose/info", "-n", "1", "--json-output"],
                         capture_output=True, text=True, timeout=timeout).stdout
    return {p["name"]: p for p in first_json_message(out)["pose"]}


def finger_angles(poses, side):
    """(finger_joint1, finger_joint2) in radians, in the URDF's own sign convention (axes -x and +x), from the angle of each
    finger link about x relative to the hand link."""
    import math

    def q(name):
        o = poses[name]["orientation"]
        return (o["x"], o["y"], o["z"], o["w"])

    base = q(f"openarm_{side}_ee_base_link")
    out = []
    for index, sign in ((1, -1.0), (2, 1.0)):
        rel = _q_mul(_q_inv(base), q(f"openarm_{side}_ee_link{index}"))
        out.append(sign * 2.0 * math.atan2(rel[0], rel[3]))
    return tuple(out)
