# SPDX-License-Identifier: MIT
"""Integrated Digital Twin Baseline 1: the phases of the whole-robot mission, the tagging of every check to its phase,
the failure classification and the reports. No ROS import: the mission (baseline1.py) uses it inside the stack, and
scripts/baseline1_report.py and the unit tests use it on a written log.

Baseline 1 is complete only when every phase passes in one run of the whole robot. A phase that passes after an
earlier phase failed, or a subsystem scenario that passes on its own, is a partial pass and is reported apart.
Simulation evidence only: never hardware, physical acceptance or safety evidence.
"""
import math
import re
import time

# name, expected motion, measured feedback. The tolerances are the low/high of each check in the log.
PHASES = (
    ("spawn", "none: the base rests at stations.origin of config/scene.yaml",
     "ground truth pose against the station, base speed, odometry against ground truth"),
    ("nav_to_station", "Nav2 navigate_to_pose from origin to stations.work_shelf (about 2.4 m and a 90 deg turn); with "
     "BASELINE1_CANCEL_CHECK=1 first a short goal cancelled mid-way",
     "Nav2 result, ground truth pose against the goal, odometry against ground truth; cancel status, stop and hold"),
    ("stop_authorize", "base stops; the lift refuses a command without permit and does not move; the permit becomes fresh",
     "ground truth base speed, lift travel without permit, CTL-017 permit state"),
    ("lift_to_target", "lift L1000 to L1175 and back to L1000, the grasp height of shelf level 0, with a permit",
     "lift_joint in the physics and on /joint_states, CTL-017 fault bits"),
    ("arm_approach", "left arm leaves the activation pose to park, gripper opens, MoveIt approach and reach to the item",
     "MoveIt error codes, servo torque left at the end point, finger opening, item displacement before closing"),
    ("grasp", "gripper closes on the item",
     "finger position on the item in the physics and on /joint_states, gripper result"),
    ("lift_object", "hand rises 0.08 m with the item, holds 3 s, releases",
     "item height gain and drift from ground truth, MoveIt error code and servo torque"),
    ("retreat", "both arms to park, Nav2 backup 0.30 m away from the shelf",
     "distance from park, Nav2 result, ground truth displacement along the heading"),
)
PHASE_NAMES = tuple(name for name, _expected, _measured in PHASES)

# Failure classes. UNCLASSIFIED is reported as such: a failure is never forced into a class it does not show.
ENV, BUILD, STARTUP, CONTROL, PLANNING, CONTACT, TIMEOUT = ("ENV/INFRA", "BUILD", "STARTUP", "CONTROL", "PLANNING",
                                                            "CONTACT/PHYSICS", "TIMEOUT")
UNCLASSIFIED = "UNCLASSIFIED"
CLASSES = (ENV, BUILD, STARTUP, CONTROL, PLANNING, CONTACT, TIMEOUT)

# grasp() of scenarios.py covers three phases. The first check of each later phase opens it.
SPLIT = {"grasp": ("finger_position_on_item", "finger_driver_reading_on_item", "gripper_reported_"),
         "lift_object": ("grasp_lift", "item_height_")}

# MoveItErrorCodes: CONTROL_FAILED -4, TIMED_OUT -6. Every other code but SUCCESS (1) is a planning outcome.
MOVEIT_CONTROL, MOVEIT_TIMEOUT = -4, -6

_RULES = (  # (class, pattern on the failure text), first match wins
    (ENV, r"BLOCKED|real-time factor .* too slow|machine is too slow|vcan|drive_thread_alive"),
    (TIMEOUT, r"wall limit|timed out|= None\b"),
    (STARTUP, r"not ready after|did not start|no /build1/run_info|did not appear|no /tf |server is missing|"
              r"did not accept the goal|no parameter service"),
    (CONTACT, r"^item_|^finger_|mimic|pushed"),
    (PLANNING, r"^nav_status|_cartesian_fraction"),   # a MoveIt Cartesian path that covers less than the straight line
    (CONTROL, r"_servo_torque_max|_tracking_error|^lift_|lift did not|fault_bits|distance_from_park|park pose|"
              r"settle_time|heading_change|position_error|yaw_error|ground_truth_distance|odometry_minus|"
              r"_status = |motion_without_permit|not refused|permit did not|base_speed|along_heading|drive_configured|"
              r"permit gate"),
)


def classify(text):
    """The failure class of one failed check or exception text."""
    match = re.match(r"(\w+_moveit_error_code) = (-?\d+|None)", text)
    if match:
        code = match.group(2)
        if code == "None":
            return TIMEOUT
        code = int(code)
        return CONTROL if code == MOVEIT_CONTROL else TIMEOUT if code == MOVEIT_TIMEOUT else PLANNING
    for name, pattern in _RULES:
        if re.search(pattern, text):
            return name
    return UNCLASSIFIED


class Tracker:
    """Tags every quantity and failed check of a rig.Log with the running phase. `clock` gives Gazebo seconds."""

    def __init__(self, log, clock=time.monotonic):
        self.log, self.clock = log, clock
        self.records = {name: {"phase": name, "ran": False, "failures": [], "quantities": [], "exception": None,
                               "sim_start": None, "sim_end": None, "note": ""} for name in PHASE_NAMES}
        self.current = None
        self._split = ()
        quantity, require = log.quantity, log.require

        def tagged_quantity(name, *args, **kwargs):
            self._maybe_split(name)
            return self._tagged(lambda: quantity(name, *args, **kwargs))

        def tagged_require(condition, text):
            return self._tagged(lambda: require(condition, text))
        log.quantity, log.require = tagged_quantity, tagged_require

    def _tagged(self, call):
        q0, f0 = len(self.log.data["quantities"]), len(self.log.failures)
        value = call()
        record = self.records.get(self.current)
        for item in self.log.data["quantities"][q0:]:
            item["phase"] = self.current
            if record is not None:
                record["quantities"].append(item)
        if record is not None:
            record["failures"] += [{"text": text, "class": classify(text)} for text in self.log.failures[f0:]]
        return value

    def _maybe_split(self, name):
        for phase in self._split:
            if not self.records[phase]["ran"] and name.startswith(SPLIT[phase]):
                self._begin(phase)

    def _begin(self, phase):
        if self.current is not None and self.records[self.current]["sim_end"] is None:
            self.records[self.current]["sim_end"] = self.clock()
        record = self.records[phase]
        record.update(ran=True, sim_start=self.clock())
        self.current = phase

    def run(self, phase, function, then=()):
        """Run `function` as `phase`; `then` names the later phases it also covers (opened by SPLIT).
        A Blocked-type exception (name "Blocked") is recorded and the mission goes on; any other is recorded and raised."""
        self._split = tuple(then)
        self._begin(phase)
        try:
            function()
        except Exception as error:  # noqa: BLE001 - recorded with its class, then raised unless Blocked
            text = str(error)
            blocked = type(error).__name__ == "Blocked"
            self.records[self.current]["exception"] = {
                "type": type(error).__name__, "text": text,
                "class": ENV if blocked else (TIMEOUT if "wall limit" in text else classify(text))}
            if not blocked:
                raise
        finally:
            self.records[self.current]["sim_end"] = self.clock()
            for later in then:
                if not self.records[later]["ran"]:
                    self.records[later]["note"] = f"not reached: {phase} stopped before it"
            self._split = ()
            self.current = None

    def blocked(self):
        return [r for r in self.records.values() if r["exception"] and r["exception"]["type"] == "Blocked"]

    def result(self):
        return [self.records[name] for name in PHASE_NAMES]


def verdict(record):
    if record["exception"]:
        return "BLOCKED" if record["exception"]["type"] == "Blocked" else "FAIL"
    if not record["ran"]:
        return "NOT RUN"
    if record["failures"]:
        return "FAIL"
    if not any(q.get("ok") is not None for q in record["quantities"]):
        return "FAIL"           # a phase without a single check proves nothing
    return "PASS"


def classes(record):
    found = []
    if record["exception"]:
        found.append(record["exception"]["class"])
    for failure in record["failures"]:
        if failure["class"] not in found:
            found.append(failure["class"])
    if not found and verdict(record) == "FAIL":
        found.append(UNCLASSIFIED)
    return found


def rows(log_data=None, build_ok=True, env_ok=True, env_reason="", launch_reason=""):
    """One row per phase: phase, expected motion, measured, verdict, classes, Gazebo seconds, detail.
    Without a written log the stage that stopped the run classifies every phase: BUILD, ENV/INFRA or STARTUP."""
    phases = {r["phase"]: r for r in (log_data or {}).get("phases", [])}
    out = []
    for index, (name, expected, measured) in enumerate(PHASES):
        if not build_ok:
            out.append([name, expected, measured, "NOT RUN", [BUILD], None, "colcon build failed"])
            continue
        if not env_ok:
            out.append([name, expected, measured, "NOT RUN", [ENV], None, env_reason or "virtual CAN unavailable"])
            continue
        record = phases.get(name)
        if record is None:
            reason = (log_data or {}).get("reason") or launch_reason or "no mission log was written"
            cls = classify(reason) if log_data else STARTUP
            cls = STARTUP if cls in (UNCLASSIFIED, CONTROL) and index == 0 else cls
            out.append([name, expected, measured, "NOT RUN" if index else ("BLOCKED" if cls == ENV else "FAIL"),
                        [cls] if index == 0 else [], None, reason if index == 0 else "the run stopped before this phase"])
            continue
        v = verdict(record)
        duration = None
        if record["sim_start"] is not None and record["sim_end"] is not None:
            duration = record["sim_end"] - record["sim_start"]
        detail = "; ".join([f["text"] for f in record["failures"]][:3]
                           + ([record["exception"]["text"]] if record["exception"] else []) + ([record["note"]] if record["note"] else []))
        out.append([name, expected, _measured(record) or measured, v, classes(record), duration, detail])
    return out


def _measured(record, limit=4):
    checks = [q for q in record["quantities"] if q.get("ok") is not None]
    return ", ".join(f"{q['name']}={q['value']} [{q['low']}, {q['high']}]{'' if q['ok'] else ' OUTSIDE'}"
                     for q in checks[:limit]) + (f" (+{len(checks) - limit} more)" if len(checks) > limit else "")


def partial_passes(table):
    """Phases that passed while the mission as a whole did not: each is a partial pass, never a Baseline 1 pass."""
    first_bad = next((i for i, row in enumerate(table) if row[3] != "PASS"), None)
    if first_bad is None:
        return []
    return [row[0] + (" (after an earlier phase did not pass)" if i > first_bad else "")
            for i, row in enumerate(table) if row[3] == "PASS"]


def complete(table):
    return all(row[3] == "PASS" for row in table) and len(table) == len(PHASES)


def render(table, title="Integrated Digital Twin Baseline 1"):
    lines = [f"### {title}", "", "| # | Phase | Expected motion | Measured feedback (tolerance) | Verdict | Class | Gazebo s | Detail |",
             "|---|---|---|---|---|---|---|---|"]
    for i, (name, expected, measured, v, cls, duration, detail) in enumerate(table, 1):
        lines.append(f"| {i} | {name} | {expected} | {measured} | {v} | {', '.join(cls) or '-'} | "
                     f"{'-' if duration is None else round(duration, 1)} | {(detail or '-').replace('|', '/')} |")
    passes = [row[0] for row in table if row[3] == "PASS"]
    lines += ["", f"Phases passed in this run: {len(passes)} of {len(PHASES)}."]
    partial = partial_passes(table)
    if partial:
        lines += ["", "Partial passes (reported apart; they do not make Baseline 1 complete):"]
        lines += [f"- {name}" for name in partial]
    lines += ["", "Baseline 1: " + ("COMPLETE in this run (simulation evidence only)" if complete(table) else
                                    "NOT COMPLETE. Subsystem passes do not make it complete.")]
    return "\n".join(lines)


def trace_summary(trace, phase_records):
    """Per phase, from the synchronized trace: Gazebo and wall span, base travel and top speed, lift travel, item height
    change, largest odometry minus ground truth, lowest real-time factor."""
    lines = ["phase | samples | gazebo s | wall s | base travel m | base v max m/s | lift travel m | item dz m | "
             "odom-truth max m | map-truth max m | rtf min"]
    for record in phase_records:
        samples = [s for s in trace if s.get("phase") == record["phase"]]
        if not samples:
            lines.append(f"{record['phase']} | 0 | - | - | - | - | - | - | - | - | -")
            continue

        def span(key):
            values = [s[key] for s in samples if s.get(key) is not None]
            return (values[0], values[-1]) if values else (None, None)

        def top(key, fn=max):
            values = [s[key] for s in samples if s.get(key) is not None]
            return round(fn(values), 4) if values else "-"

        (x0, x1), (y0, y1) = span("x"), span("y")
        base = round(math.hypot(x1 - x0, y1 - y0), 4) if None not in (x0, x1, y0, y1) else "-"
        l0, l1 = span("lift")
        z0, z1 = span("item_z")
        sim0, sim1 = span("sim")
        wall0, wall1 = span("wall")
        lines.append(" | ".join(str(v) for v in (
            record["phase"], len(samples), round(sim1 - sim0, 2) if sim0 is not None else "-",
            round(wall1 - wall0, 2) if wall0 is not None else "-", base, top("v"),
            round(l1 - l0, 4) if None not in (l0, l1) else "-", round(z1 - z0, 4) if None not in (z0, z1) else "-",
            top("odom_err"), top("map_err"), top("rtf", min))))
    return lines


def trace_table(trace, every_sim_s=5.0):
    """The trace thinned to one line per `every_sim_s` Gazebo seconds, all channels on one time base."""
    keys = ("sim", "wall", "phase", "x", "y", "yaw", "v", "w", "odom_err", "odom_yaw_err", "map_err", "lift", "lift_js",
            "finger", "item_z", "rtf")
    lines = [" ".join(f"{k:>9}" for k in keys)]
    last = None
    for sample in trace:
        if last is not None and sample["sim"] - last < every_sim_s:
            continue
        last = sample["sim"]
        lines.append(" ".join(f"{_fmt(sample.get(k)):>9}" for k in keys))
    return lines


def _fmt(value):
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)[:9]
