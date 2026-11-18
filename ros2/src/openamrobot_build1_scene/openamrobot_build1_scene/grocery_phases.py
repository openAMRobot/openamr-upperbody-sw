# SPDX-License-Identifier: MIT
"""Grocery dark store demonstrator: the phases of the pick-and-place mission, their tracker and the reports.

The tracking, the failure classes and the verdict rules are those of Baseline 1 (baseline1_phases): this module only
names the phases of the longer mission. No ROS import: the mission (grocery_demo.py) uses it inside the stack, and
scripts/grocery_demo_report.py and the unit tests use it on a written log.

The demonstrator passes only when every phase passes in one run. Profile sim-oracle (docs/integration/baseline1.md):
the planning scene and the grasp targets come from the simulator's ground truth, declared as `pose_source:
ground_truth`. A pass is integration evidence of the ROS 2 stack, the firmware-SIL path, the controllers and the contact
physics in simulation; it is never a perception, hardware, physical acceptance or safety result.
"""
from contextlib import contextmanager

from openamrobot_build1_scene import baseline1_phases as b1

PROFILE = "sim-oracle"
POSE_SOURCE = "ground_truth"

# name, expected motion, measured feedback. The tolerances are the low/high of each check in the log.
PHASES = (
    ("spawn", "none: the base rests in the parking bay (stations.origin of config/grocery_scene.yaml)",
     "ground truth pose against the station, base speed, odometry against ground truth"),
    ("nav_to_shelf", "Nav2 navigate_to_pose with the station tree (#164) from the parking bay into aisle 1, to stations.work_shelf",
     "Nav2 result, ground truth against the goal (Nav2 tolerance and the 0.04 m / 0.08 rad arm-reach budget)"),
    ("stop_authorize", "base stops; the lift refuses a command without permit and does not move; the permit becomes fresh",
     "ground truth base speed, lift travel without permit, CTL-017 permit state (Baseline 1, unchanged)"),
    ("lift_to_target", "lift L1000 to L1175 and back to L1000, the shelf stop of level 0, with a permit",
     "lift_joint in the physics and on /joint_states, CTL-017 fault bits (Baseline 1, unchanged)"),
    ("arm_approach", "left arm to park, gripper opens, MoveIt approach and straight reach to the target product",
     "MoveIt error codes, servo torque at the end point, finger opening, product displacement before closing"),
    ("grasp", "gripper closes on the product",
     "finger position on the product in the physics and on /joint_states, gripper result"),
    ("lift_object", "hand rises 0.08 m with the product and holds it 3 s; the product stays in the hand",
     "product height gain and drift from ground truth, MoveIt error code and servo torque"),
    ("retreat_with_item", "Nav2 backup 0.30 m from the shelf with the product held, arm unchanged",
     "Nav2 result, displacement along the heading, product height gain and distance to the TCP"),
    ("nav_to_packing", "Nav2 navigate_to_pose with the station tree out of aisle 1 to the packing bench (stations.packing)",
     "Nav2 result, ground truth against the goal (Nav2 tolerance), product still held"),
    ("place", "hand lowers the product into the order tote, gripper opens, hand withdraws 0.10 m",
     "MoveIt Cartesian fraction and error codes, product inside the tote bounds, product settled, gripper open"),
    ("retreat", "both arms to park, Nav2 backup 0.30 m from the bench (Baseline 1 retreat, unchanged)",
     "distance from park, Nav2 result, ground truth displacement along the heading"),
    ("return_to_parking", "Nav2 navigate_to_pose with the station tree back to the parking bay",
     "Nav2 result, ground truth against the parking pose (Nav2 tolerance), base stopped"),
)
PHASE_NAMES = tuple(name for name, _expected, _measured in PHASES)
TITLE = "Grocery dark store demonstrator (sim-oracle profile)"

classify, verdict, classes = b1.classify, b1.verdict, b1.classes


class Tracker(b1.Tracker):
    """Baseline 1's tracker with the phases of this mission."""

    def __init__(self, log, clock):
        super().__init__(log, clock=clock)
        self.records = {name: {"phase": name, "ran": False, "failures": [], "quantities": [], "exception": None,
                               "sim_start": None, "sim_end": None, "note": ""} for name in PHASE_NAMES}

    def result(self):
        return [self.records[name] for name in PHASE_NAMES]

    def skip(self, phase, reason):
        """Record `phase` as not run, with the reason: a later phase that needs an earlier one is not attempted."""
        self.records[phase]["note"] = f"not run: {reason}"


@contextmanager
def _these_phases():
    """baseline1_phases.rows and render read their module's PHASES: this mission's phases while inside, restored after."""
    saved = b1.PHASES
    b1.PHASES = PHASES
    try:
        yield
    finally:
        b1.PHASES = saved


def rows(log_data=None, build_ok=True, env_ok=True, env_reason=""):
    """One row per phase, as baseline1_phases.rows: phase, expected, measured, verdict, classes, Gazebo s, detail."""
    with _these_phases():
        return b1.rows(log_data, build_ok=build_ok, env_ok=env_ok, env_reason=env_reason)


def complete(table):
    return len(table) == len(PHASES) and all(row[3] == "PASS" for row in table)


def render(table):
    with _these_phases():
        text = b1.render(table, title=TITLE)
    return text.replace("Baseline 1: COMPLETE in this run (simulation evidence only)",
                        "Demonstrator: ALL PHASES PASSED in this run (simulation evidence only, sim-oracle profile)").replace(
        "Baseline 1: NOT COMPLETE. Subsystem passes do not make it complete.",
        "Demonstrator: NOT COMPLETE. Partial passes do not make it complete.").replace(
        "they do not make Baseline 1 complete", "they do not make the demonstrator complete")


def evidence_summary(data, log_dir_listing=()):
    """Lines that say what the run declared and recorded: profile, pose source, pins, real-time factor, recordings."""
    data = data or {}
    run = data.get("run", {})
    demo = data.get("grocery", {})
    q = {item["name"]: item["value"] for item in data.get("quantities", [])}
    lines = [f"profile: {demo.get('profile', '-')}",
             f"pose_source: {demo.get('pose_source', '-')}",
             f"scene: {run.get('scene', '-')}, station tree: {demo.get('behavior_tree', '-')}",
             f"run profile: {run.get('profile', '-')}, physics step {run.get('physics_step', '-')}, engine {run.get('physics_engine', '-')}",
             f"seed: {demo.get('seed', '-')}",
             "Gazebo real-time factor over the run: mean {} min {} max {}".format(
                 q.get("gazebo_rtf_mean_whole_run"), q.get("gazebo_rtf_min_whole_run"), q.get("gazebo_rtf_max_whole_run")),
             f"mission Gazebo time: {demo.get('sim_seconds', '-')} s, wall time: {demo.get('wall_seconds', '-')} s"]
    names = sorted(log_dir_listing)
    for label, test in (("ROS bag", lambda n: n == "bag"), ("candump", lambda n: n.startswith("record_candump_")),
                        ("video", lambda n: n.endswith(".mp4")), ("phase subtitles", lambda n: n.endswith(".srt")),
                        ("mission log", lambda n: n == "grocery_demo_mission.json"),
                        ("mission report", lambda n: n == "grocery_demo_report.json"),
                        ("controller and firmware states", lambda n: n == "grocery_states.json")):
        found = [n for n in names if test(n)]
        lines.append(f"{label}: {', '.join(found) if found else 'not recorded'}")
    return lines
