<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Timestamps of headerless command topics in a capture (Proposed recommendation, 19 Nov 2026)

Status: Proposed. Nothing here changes `/cmd_vel`, the converter or any contract. The decision belongs to the
software lead (option A, a contract change) and to the embodied-AI owner (option B, a converter rule in
openamrobot-embodied-ai). Simulation evidence only.

## The problem, as measured

- `/cmd_vel` is `geometry_msgs/msg/Twist`: no header. The converter's `read_bag` (embodied-ai `9643da1`) then uses the
  bag receipt time (`timestamp_source: bag`) and refuses any stream whose timestamps are not strictly increasing.
- The Baseline 1 recorder runs `ros2 bag record --use-sim-time`, so receipt times are the recorder's view of `/clock`.
  Gazebo publishes `/clock` at 1 kHz of simulation time and the runner runs at a real-time factor of about 0.2, so
  several commands can arrive within one clock tick and get the same receipt time.
- Result on run 44309791416: `read_bag` refused the capture with
  `Nonmonotonic or duplicate timestamps / clock discontinuity: 'cmd_vel'`.
- Header stamps repeat too. In run 44448676604 (opt-in AI capture, 8/8, real-time factor 0.11 to 0.14) the stamps as
  `read_bag` reads them were not strictly increasing on `/arm/joint_states` (122 times, header),
  `/left_arm_controller/controller_state` (117) and `/right_arm_controller/controller_state` (94), and on `/cmd_vel`
  (1288 times, receipt). `read_bag` refused the capture at the first of them, `arm_joint_states`. These counts do not
  separate equal from decreasing stamps; the binding tool at the head of this PR does.
- Related: action commands that are events (`/sil/lift/command`, 3 messages per run) or that stop when Nav2 is idle
  (`/cmd_vel` during the arm phases) fail the converter's causal lookup, which refuses a command older than
  `max_skew_ns`. The gripper command already has a held stream (`/build1/capture/gripper_command`, Proposed).
  `binding.json` now reports, per candidate stream, how many non-increasing stamps it has.

## Option A: `/cmd_vel` as `geometry_msgs/msg/TwistStamped` (contract change)

What changes: the type of a contract topic. Process: a contract change request naming the entry, one PR per the
rules in AGENTS.md updating the contract and every consumer together, software lead approval.

Files that reference `/cmd_vel` at openamr-platform-sw `931e631` and in this repository (a search, not a reviewed
consumer list; the change request must list every consumer):
- Nav2 (controller server, velocity smoother, collision monitor, behaviours): Nav2 Jazzy keeps `Twist` by default;
  `enable_stamped_cmd_vel: true` switches each server. The default changed to stamped only in Kilted.
- `ros_gz_bridge` entries for `/cmd_vel` (`openamrobot_gazebo/config/gz_bridge.yaml`, the fast and fine profiles of
  this package's `launch_support.py`).
- openamrobot_sil `stm32_host_client_node.py` (the SIL base command input).
- docking (`dock_trigger.py`, config), station alignment (its command-authority test checks for Twist and
  TwistStamped publishers).
- Gate A Teensy micro-ROS firmware and the Gate B STM32 host-to-base contract (I8), and any operator UI teleop.

For: the stamp is set by the producer at acquisition, which is what the converter's header rule expects; matches
upstream's direction (Kilted). Against: many consumers including firmware, close to the 20 Nov pre-release and the
18 Dec release; it does not remove duplicates by itself, because a producer that publishes twice within one 1 ms tick
of simulation time writes equal header stamps too. The measured header repeats on `/arm/joint_states` and the
controller states show this already happens: with option A alone the capture is still refused.

## Option B: a documented receipt-time rule in the converter (suggestion for embodied-ai, not pushed there)

A rule the converter would apply to every stream (receipt time for `timestamp_source: bag`, header stamp for
`header`):
1. Order by the stream's time, then by bag sequence. Equal times are allowed; a decreasing time is still a clock
   discontinuity and is refused.
2. For an action stream, the command in force at grid time t is the last message received at or before t (the last of
   a group of equal stamps). This is the causal rule `align` already uses, applied to groups.
3. The profile declares the clock resolution (`clock_resolution_ns`, 1 000 000 for Gazebo's 1 kHz `/clock`) and the
   converter refuses a grid step that is not much larger than it (for example at least 10 times).
4. The sidecar records the rule and the number of equal-stamp groups per stream, so the data stays auditable.
5. Optional, for event commands: a declared hold (`hold: zero_order`, with a maximum hold time) per action stream,
   instead of `max_skew_ns`, reviewed per channel. Without it, event commands need a held republisher at record time.

For: no contract change, no consumer touched, reversible, covers the headerless topics (`/cmd_vel`,
`/sil/lift/command`) and the measured header repeats alike.
Against: receipt time is arrival at the recorder, not acquisition; under simulation time the error is bounded by one
clock tick plus transport delay, which must be stated in the dataset card.

## Recommendation (Proposed)

Option B for simulation captures now, with the rule written in the converter's documentation and the sidecar, because
it needs no contract change before the release. Keep option A as a candidate contract change request after
18 Dec 2026, decided by the software lead together with the firmware owner (I8). Either way the decision is recorded
in decisions.yaml before a dataset built on it is published.
