# D4 refinement: field ownership of the MIT frame

Status: **Proposed**, pending the architecture owner's decision. Ruled acceptable as a refinement of D4 by the CTO
on 17 Nov 2026. Not approved: decisions.yaml is not changed by this page.

Scope: the Build 1 scene (`ros2/src/openamrobot_build1_scene`), profiles `firmware_sil` and `faults`, where the
pinned OpenArm driver (`openarm_hardware/OpenArmHW`) sends each arm motor one Damiao MIT frame `{kp, kd, q, dq, tau}`.
Simulation and driver-path evidence only. Nothing here is a safety function or safety evidence.

## Rule

D4 says one writer per joint. For a joint driven by MIT frames it reads:

1. Exactly one driver writes the whole MIT frame of a motor: `OpenArmHW` in `/controller_manager`.
2. Each field of the frame has exactly one controller:
   - `q` and `dq`: `{side}_arm_controller` (joint_trajectory_controller), through the position interface.
   - `tau`: `{side}_arm_gravity_controller` (JointGroupEffortController), through the effort interface, fed only by
     the `gravity_feedforward` node on `/{side}_arm_gravity_controller/commands`.
   - `kp` and `kd`: the driver's own gains. No controller writes them.

## Condition

JointGroupEffortController keeps its last command for ever. A silent feed-forward would leave the last torque in the
tau field. So `gravity_feedforward` publishes one command per arm every cycle:

- the model torque when all seven joints of that arm have a state younger than `max_state_age_s` (ROS time,
  default 0.2 s);
- tau 0 for that arm when its state is stale, incomplete or never received, with a throttled warning.

This is a functional command timeout of a simulation and driver-path controller. It is not a safety function and
does not replace E-stop, brake, watchdog or motor-enable logic.

A held object enters the model only as an optional point mass (`payload_mass_kg`, `payload_com_xyz` in the frame of
`payload_link`; off by default). Scenario 4 sets it to the scene's item mass at the model's TCP only after the gripper
reports the grasp, and clears it after the release.

## Check

- `config/command_map.yaml` lists the arm joints' effort claim and the one writer of each gravity command topic.
  `test/test_command_map.py` checks the claim against the controller configuration.
- Scenario `scn09_single_writer` checks the running system against that map: one writer per command topic.
- `test/test_gravity.py` checks the condition on the command logic (`FeedForward` in `gravity.py`): zeros on a stale,
  incomplete or never-received state, the model torque on a fresh one, and the payload term.

Not checked by scn09: that zeros reach the tau field on the bus after a joint state loss. No scenario injects that loss.
