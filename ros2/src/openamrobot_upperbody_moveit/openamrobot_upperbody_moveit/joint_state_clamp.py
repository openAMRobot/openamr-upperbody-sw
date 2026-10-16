"""The simulated start state's float noise, clamped to the joint limit, counted, and nothing else.

Why it exists. MoveIt 2.12.4 refuses a request whose start state is any amount outside a joint's limits (the reply is the generic FAILURE; the reason, "outside bounds by", is only in move_group's log): the
`CheckStartStateBounds` adapter is strict, there is no `FixStartStateBounds` in the installed adapter set, and OMPL's planning context
refuses the same state. Upstream's `home` puts `openarm_*_joint4` exactly on its lower limit (0), and in Gazebo a joint resting on a limit
reads back a hair past it (-2.5e-7 and -9e-8 were seen). That is the simulator's float noise, not a robot state, so the one thing done about
it is to give move_group a joint state with the noise taken off.

What it is NOT. No limit is widened (that would invent vendor data, ADR-009), no pose is moved (`home` stays upstream's), and nothing outside
the clamp's EPSILON is touched: a joint that is further past its limit than that is not float noise, it is passed through unchanged, counted
as `over_epsilon` and logged as an error, because hiding it would hide a real drift. Only the topic move_group listens to is clamped; the
raw /joint_states, which everything else reads, is untouched.

The counters are the point. A clamp that fires constantly, or one that meets values beyond its epsilon, is telling you something is moving
the joint, and the evidence carries `activations`, `episodes`, `max_magnitude` and `over_epsilon` so that is visible.
"""
import json
import math

# 1e-5 rad. Measured in Gazebo here (headless, Bullet-Featherstone), the joint resting on its limit reads back past it by: up to 9.9e-7 rad
# (openarm_left_joint4, 2585 of 3356 messages, from a cold start), 8.0e-7 (the same joint at idle), 2.6e-7 (openarm_right_joint4 after a
# sequence of poses), and earlier sessions saw 2.5e-7 and 9e-8. A 1e-6 epsilon would clear the worst of those by one per cent and trip on the
# next cold start, so this is ten times the largest value seen. It is still ten micro-radians: no plan can tell a joint at its limit from one
# that far inside it. The counters, not the epsilon, are what keeps this honest: a reading beyond the epsilon is never clamped.
EPSILON = 1e-5


def clamp_position(value, lower, upper, epsilon=EPSILON):
    """(clamped value, overshoot, kind). `overshoot` is how far past the limit `value` is (0.0 if inside, inclusive). `kind` is "inside",
    "clamped" (past a limit by no more than `epsilon`: the returned value IS the limit, never beyond it) or "over_epsilon" (further than
    that: the value is returned unchanged, because it is not float noise). NaN and infinities are passed through as "over_epsilon"."""
    if not math.isfinite(value):
        return value, math.inf, "over_epsilon"
    if value < lower:
        overshoot, limit = lower - value, lower
    elif value > upper:
        overshoot, limit = value - upper, upper
    else:
        return value, 0.0, "inside"
    if overshoot <= epsilon:
        return limit, overshoot, "clamped"
    return value, overshoot, "over_epsilon"


class Clamp:
    """The state of one clamp: per-joint limits, and the counters. No ROS here, so the behaviour is unit-tested without a runtime.

    `activations`: joint readings clamped (one per joint per message). `episodes`: runs of consecutive messages in which a joint was clamped
    (a joint resting on its limit for a minute is one episode of many activations). `max_magnitude`: the largest overshoot clamped.
    `over_epsilon`: readings further outside than the epsilon, passed through, with `max_over_epsilon`."""

    def __init__(self, limits, epsilon=EPSILON):
        self.limits = dict(limits)
        self.epsilon = epsilon
        self.messages = 0
        self.activations = 0
        self.episodes = 0
        self.max_magnitude = 0.0
        self.max_joint = None
        self.over_epsilon = 0
        self.max_over_epsilon = 0.0
        self.max_over_epsilon_joint = None
        self.per_joint = {}
        self._active = set()          # joints clamped in the previous message (an episode continues while they are)

    def process(self, names, positions):
        """Clamp one joint state. Returns (positions, events): the new positions list, and one event per joint that was not inside:
        {"joint", "kind", "overshoot", "value", "new_episode"}. A joint with no limits (continuous, wheels) is passed through."""
        self.messages += 1
        out, events, now_active = list(positions), [], set()
        for i, name in enumerate(names):
            limits = self.limits.get(name)
            if limits is None or i >= len(positions):
                continue
            value, overshoot, kind = clamp_position(positions[i], limits[0], limits[1], self.epsilon)
            if kind == "inside":
                continue
            row = self.per_joint.setdefault(name, {"activations": 0, "episodes": 0, "max_magnitude": 0.0, "over_epsilon": 0})
            if kind == "clamped":
                out[i] = value
                new_episode = name not in self._active
                now_active.add(name)
                self.activations += 1
                row["activations"] += 1
                if new_episode:
                    self.episodes += 1
                    row["episodes"] += 1
                if overshoot > self.max_magnitude:
                    self.max_magnitude, self.max_joint = overshoot, name
                row["max_magnitude"] = max(row["max_magnitude"], overshoot)
            else:
                new_episode = False
                self.over_epsilon += 1
                row["over_epsilon"] += 1
                if overshoot > self.max_over_epsilon:
                    self.max_over_epsilon, self.max_over_epsilon_joint = overshoot, name
            events.append({"joint": name, "kind": kind, "overshoot": overshoot, "value": positions[i], "new_episode": new_episode})
        self._active = now_active
        return out, events

    def summary(self):
        return {"epsilon_rad": self.epsilon, "messages": self.messages, "activations": self.activations, "episodes": self.episodes,
                "max_magnitude_rad": self.max_magnitude, "max_magnitude_joint": self.max_joint,
                "over_epsilon": self.over_epsilon, "max_over_epsilon_rad": self.max_over_epsilon,
                "max_over_epsilon_joint": self.max_over_epsilon_joint, "joints": self.per_joint}

    def summary_json(self):
        return json.dumps(self.summary(), sort_keys=True)


def limits_from_urdf(urdf_root):
    """{joint: (lower, upper)} for every joint with position limits: revolute and prismatic joints. Continuous joints have none."""
    out = {}
    for joint in urdf_root.findall("joint"):
        if joint.get("type") in ("revolute", "prismatic"):
            limit = joint.find("limit")
            if limit is not None and limit.get("lower") is not None and limit.get("upper") is not None:
                out[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    return out
