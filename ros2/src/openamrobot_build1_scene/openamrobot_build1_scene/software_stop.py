# SPDX-License-Identifier: MIT
"""The software Stop lifecycle a mission checks before its first goal, on a platform with #90's Stop gate.

Platform #90 starts with the software Stop engaged (fail-closed) and refuses every public navigation goal until an
operator releases it (the UI's Resume). A mission has no UI, so the rig makes that operator step itself, through the
public service only, and checks the lifecycle around it:

1. the gate is present when the installed platform ships it;
2. it starts engaged (LOCKED or RELEASE_REFUSED on /safety/stop_state);
3. the base does not move while it is engaged and before the first goal;
4. /safety/stop_lock_release succeeds;
5. /safety/stop_state then reports RUNNING.

Any miss fails the scenario. No internal flag is changed and nothing is published downstream of the gate. The software
Stop is a functional stop, not the hardwired E-stop. No ROS import here (test/test_software_stop.py); the rig observes.
"""

# openamr_nav_msgs/StopState (openamrobot-interfaces, pinned in config/pins.yaml)
STATE_RUNNING, STATE_LOCKED, STATE_RELEASE_REFUSED = "RUNNING", "LOCKED", "RELEASE_REFUSED"
ENGAGED = (STATE_LOCKED, STATE_RELEASE_REFUSED)
MAX_MOTION_BEFORE_GOAL_M = 0.005   # m of ground-truth travel while engaged; above it something moved the base


def problems(gate_expected, gate_present, initial_state, moved_m, release_ok, release_message, final_state):
    """-> [problem], empty when the lifecycle holds. gate_expected: the installed platform ships safety_stop_gate_node."""
    if not gate_expected:
        return [] if not gate_present else ["a Stop gate answers but the installed platform does not declare one"]
    if not gate_present:
        return ["the installed platform ships safety_stop_gate_node, but /safety/stop_lock_release or "
                "/safety/stop_state is not available"]
    found = []
    if initial_state not in ENGAGED:
        found.append(f"the software Stop did not start engaged: first /safety/stop_state {initial_state!r}")
    if moved_m is None or moved_m > MAX_MOTION_BEFORE_GOAL_M:
        found.append(f"the base moved {moved_m} m while the software Stop was engaged (allowed {MAX_MOTION_BEFORE_GOAL_M} m)")
    if not release_ok:
        found.append(f"/safety/stop_lock_release was refused: {release_message}")
    elif final_state != STATE_RUNNING:
        found.append(f"after the release /safety/stop_state is {final_state!r}, not RUNNING")
    return found
