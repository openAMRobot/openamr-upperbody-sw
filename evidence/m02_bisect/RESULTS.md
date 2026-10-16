# M02 clamp activations: bisect of the 184-277 -> ~1750 rise (P16 task 3)

**Result: no cause found.** Four candidates and the platform-sw base are ruled out by measurement; the rise is somewhere in the option-B body
composition as a whole and was not bisected below that. The epsilon (1e-5) was not touched.

## What the rise is

Per M02 run (50 Hz `/joint_states`, about 1520 messages, about 30 s of simulated time), per joint, from the clamp's own counters
(`details.clamp.joints` in the run sidecar; the A7 report keeps only the totals):

| tree | activations | where |
|---|---|---|
| old body (mast model, `7daa2ab`) on old platform-sw (`12de3ef`) | 212 | `left_joint4` 44, `right_joint4` 168. **No finger joint ever.** |
| old body on **new** platform-sw (`174731c`) | 188, 216 | the same two joints. **No finger joint ever** |
| new body (`fe5107f`) on new platform-sw | 1796, 1795, 1350 | `right_finger_joint1` and `_joint2` 873, 873, 646 each (the same reading: joint2 is `joint_state_publisher`'s mimic of joint1), `left_finger_*` 8 each at 1.2e-17 rad (rounding), joint4s 8 to 35 |

So it is one physical joint, the **right** finger at its open limit (0; range [-0.7854, 0]), reading 1e-8 to 2.9e-7 rad past it, for about half of
the run; nothing else changed class. The left finger (range [0, 0.7854], same limit value) does not.

## Which candidates are ruled out, and by what

Each variant is the new tree with one change made in a copy of the installed share directory (`scripts/mkovl.py`), two M02 runs each
(the contaminated first attempt is kept beside, see below). The scatter inside one configuration is as large as the whole range between
variants (unchanged tree: 873, 646, 1054 right-finger activations), so only "present or absent" can be read from this table.

| candidate | variant | right finger activations (run 1, run 2) | verdict |
|---|---|---|---|
| (a) SRDF / ACM rebuild (83 pairs, 478 enabled) | **every** link pair disabled (1596 pairs): the ACM cannot be a cause by any value | 162, 882 | **ruled out** (present at both runs) |
| (b) modelled mass 21.96 -> 30.66 kg | `lift_mass_dold_v4_350` 11.7 -> 3.0 kg (modelled mass back to about 21.96) | run 1 invalid (`move_group` SIGSEGV at start, no poses; the idle readings in it: 5654 of 5756 messages), 908 | **ruled out** (present, 98% of idle messages and in the valid run) |
| (c) two plates / 62 mm separation | arm bases 120 mm apart (old mast value); and arm bases at the column axis (x 0.18 -> 0) | 454, 889 ; 1062, 806 | **ruled out** (both present) |
| (d) opposite-signed GripperCommand open positions (ADR-009 point 5) | `open` at +0.001 (left) and -0.001 (right) instead of 0 | 887, 861 | **ruled out as a cause**; also not a change between old and new (the SRDF gripper states and `arm_controllers.yaml` are byte-identical between `7daa2ab` and HEAD) |
| (e) *not on the prompt's list:* platform-sw 935efc7 (wheel radius 0.11 -> 0.100, axle 0.098 above the floor, `base_footprint` root) | old body composed on the new base | 188, 216 (no finger activations) | **ruled out**: the new base does not produce it |
| lift joint (P15) | `lift_joint` made fixed | 3300 to 3500 (P15, in `evidence/reports/README.md`) | ruled out (made worse) |

The old body on the new base is the discriminating run: the rise needs the new body and does not need the new base.

## What was learned about the mechanism (a measurement, not a cause)

`scripts/probe.sh` launches the simulator alone, waits 25 s of simulated time and samples `/joint_states` at rest (600 messages):

| tree | `left_finger_joint1` | `right_finger_joint1` |
|---|---|---|
| old body, old base (`old1`) | +3.6e-5 (inside its range) | -6.0e-5 (inside its range) |
| new body (`head1`) | +2.0e-4 | -4.7e-6 |
| new body (`head2`, same configuration) | +6.4e-4 | +2.7e-8 (**past** the limit) |

In the old body both fingers rest tens of microradians inside their range; in the new body the **right finger rests on its limit** (within
1e-5 rad of it, and in some runs past it), and the left one rests further inside than before. A servo holding a joint at a target that is the
limit, against a static torque that presses it onto the limit, leaves the limit constraint carrying the load, and the soft constraint's
penetration is the 1e-8 to 3e-7 rad that is read. The reading is therefore a resting condition, not a motion effect (it is there with
no poses at all), and it hovers across zero, which is why the count per run scatters from 160 to 1060. **That the torque's sign changed
between the two bodies is a hypothesis consistent with these numbers; it was not measured** (no finger torque or contact force was read),
and nothing here says why the new composition changes it. *(P17 measured the effort in both bodies, below: the sign did not change, and the
hypothesis is refuted.)*

## What is left

The candidates that remain are whatever else the option-B composition changed for the arms' static load, none of which was isolated:
the prismatic joint with `effort 0` and the column's 11.7 kg inertial between the base and the arms (the fixed-lift test of P15 and the
3 kg test above say neither alone is it), the carriage and `arm_profile_link` bodies, the head and wrist camera links, and the way the
URDF-to-SDF conversion lumps the fixed joints and orders the multibody (the arms hang off a different parent chain than they did).
The next experiment is a finger-torque or contact-force readout at rest (`gz topic` on the joint wrench or contacts) in both trees.

## Hygiene note on this data

The first overlay attempt (`m02_results_first_attempt_contaminated.txt`) is **not evidence**: the overlay lacked the package's `lib`
directory (the sidecars were never written) and the failed runs left `gz sim` servers and stack processes running, which loaded
the machine for the runs after them. Those runs were discarded; the table above is from runs after a clean-up, with a pre-run check
that nothing of an earlier run is alive. The M02 launch (not only G02's) leaves its simulator behind after a failed run.

## Finger torque at rest, both trees (P17 task 5)

**One question: why does the right finger rest on its open limit?** Scoped to that, no further bisecting.

**Method.** `scripts/effort_probe.sh` / `effort_reader.py`: the headless simulator (Bullet-Featherstone, nothing commanded), 25 s of simulated
time to settle, then 300 messages (6 s) of `/arm/joint_states` with position, velocity **and effort**. `effort` is the joint state broadcaster's reading
of the `effort` state interface that `gz_ros2_control` declares for every arm joint (the `<ros2_control>` block lists it). Three runs of each tree:
**new** = this branch; **old** = `7daa2ab` (mast model), built as an overlay of the description, control and gazebo packages on the same
platform-sw `174731c`, so the base is the same. Raw data: `effort_new{1,2,3}.json`, `effort_old{1,2,3}.json`.

| joint | tree, run | position mean [rad] | distance to nearest limit [rad] | effort mean [N m] (min .. max) |
|---|---|---|---|---|
| right_finger_joint1 (range -0.7854 .. **0**) | new 1 / 2 / 3 | +2.7e-08 / -8.8e-06 / -4.8e-06 | -2.7e-08 (past) / 8.8e-06 / 4.6e-06 | -1.85e-04 / -1.86e-04 / -1.86e-04 (spread +-5e-06) |
| | old 1 / 2 / 3 | -1.4e-05 / -2.4e-05 / -1.4e-05 | 1.4e-05 / 2.4e-05 / 1.4e-05 | -1.27e-04 / -1.21e-04 / -1.28e-04 (-3.8e-04 .. +8.5e-05) |
| left_finger_joint1 (range **0** .. 0.7854) | new 1 / 2 / 3 | +4.8e-04 / +4.9e-04 / +2.1e-04 | 4.8e-04 / 4.9e-04 / 2.1e-04 | +1.48e-04 / +1.48e-04 / +1.34e-04 |
| | old 1 / 2 / 3 | +8.4e-05 / +4.7e-05 / +5.0e-06 | 8.4e-05 / 4.7e-05 / 5.0e-06 | +1.28e-04 / +1.32e-04 / +1.26e-04 |
| left_joint4 (range **0** .. 2.4435) | new | +1.0e-04 (all three) | 9.8e-05 .. 1.0e-04 | -3.3e-02 |
| | old | -1.7e-07 / -5.7e-08 / -3.9e-08 | **past** the limit by 4.6e-07 .. 7.4e-07 (min reading) | -3.4e-02 / -3.2e-02 / -3.1e-02 |
| right_joint4 | new / old | +9.5e-05 / +4.7e-06 | 9.3e-05 .. 9.5e-05 / 4.5e-06 .. 4.8e-06 | -2.0e-02 / -2.1e-02 |

All other arm joints are 0.17 rad or more from their limits in both trees.

**What the numbers say.**

1. **The hypothesis of `RESULTS.md` above is refuted: the effort has the same sign in both bodies.** Right finger: negative in the old and the new body;
   left finger: positive in both. Each presses its finger from the open limit towards the inside of its range (mirrored, as the joint ranges
   are). The size is 1.5x larger in the new right finger (-1.86e-4 against -1.2e-4 to -1.3e-4) and very much steadier (the old body's
   effort swings from -3.8e-4 to +8.5e-5 in one run, which is also the old body's velocity noise: up to 2e-5 rad/s against 3e-7).
2. **The right finger rests on its open limit in the old body too.** Within 1.4e-5 to 2.4e-5 rad of it in three runs, not "6.0e-5 inside"
   (the one reading of P16's probe). The difference between the bodies is about 1e-5 rad, which is the between-run scatter of the new tree itself
   (9e-6 for this joint). Every one of the four joints that rest on a limit in the new body (both `finger_joint1`, both `joint4`) rests within
   2.5e-5 rad of it in the old body as well, and the old left `joint4` rests *past* its limit in all three runs (by 4.6e-7 to 7.4e-7 at its minimum reading).
3. **Why it is on the limit at all: the open pose is the limit.** `open` is 0.0 in the SRDF and the right finger's range ends at 0
   (`[-0.7854, 0]`; the left one starts at 0), and the gripper controller holds that position; the same is true of `home` and `joint4`
   (upstream's `home` is joint 4 at its lower limit 0). The pose is a limit by construction, in both bodies. That is the whole answer to "why *on*
   the limit"; it was always so.
4. **What moved is which joint straddles, not whether one does.** Old: the `joint4`s sit at or past 0 and the fingers 1e-5 to 1e-4 inside.
   New: the `joint4`s sit 1e-4 inside and the right finger sits at 0 (+2.7e-8 .. -8.8e-6), which is consistent with the clamp's activations moving from
   `joint4` to the finger (a joint that straddles zero is clamped on every other message; the old `joint4`s sat past it by 4e-7 or less). That is
   an inference from positions; the clamp's own counters were not read in these probes.

**What is not answered, said plainly.** Why the new body's *right* finger sits within 1e-5 of the limit while its *left* finger, under a mirrored
effort of similar size (+1.5e-4 against -1.86e-4), sits 2e-4 to 5e-4 inside. Why the same effort moved the left finger and not the right.
What the `effort` number physically is (the servo's command, or the joint's constraint force: the interface does not say, and no contact force
or joint wrench was read, which would take a force-torque sensor in the model). **Cause unknown, measured not guessed.** Not bisected further.

**What this changes elsewhere (for the architecture chat, not decided here).** M07 as specified ("no joint's rest position within a margin of
either limit") FAILS the old body on the same four joints for any margin above about 2.5e-5 rad, so the old body was not "inside" in the sense
M07 tests; the finding is that the zero pose and the open pose are limits, not that the option-B composition put a finger on one. The
margin datum was chosen from the scatter measured here (between-run spread of a rest position up to 2.8e-4 rad, left finger), not tuned to a
verdict; at 1e-3 rad M07 FAILs those four joints in the new tree.
