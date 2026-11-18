# Turn-slip experiment: odometry heading error in turns in place (experiment, Proposed)

Status: experiment on the scratch branch `diag/spin-test`, not for merge. The recommendation below is Proposed and
not applied. Simulation evidence only (Gazebo DART, `firmware_sil`): not hardware, physical acceptance or safety
evidence.

## Question

The grocery demonstrator (#86) saw clean straight driving but turns in place with an odometry to body yaw ratio of
0.877 to 1.109. Three candidate causes in the Gazebo model of the drive wheels and castors:

- A. `fdir1 1 0 0` on the drive wheels is expressed in the spinning wheel's collision frame, so the mu 1.5 / mu2 0.5
  axes might turn with the wheel.
- B. The wheel collision is a flat 46 mm cylinder, not the crowned 49.4 mm tyre, so the contact can sit anywhere
  across the width: effective track 0.400 +- 0.023 m.
- C. Four frictionless fixed castor spheres 0.421 mm above the drive wheel contact plane: DART decides the load share
  and nothing damps yaw.

## Verdict

Cause B dominates. The flat cylinder touches the floor at its two rims, 23 mm either side of the nominal contact. In a
turn in place the two rims need different ground speeds, so one rim slips, and which one sticks decides the body
rate. With the outer rim sticking the body turns at 0.200 / 0.223 = 0.897 of the wheel kinematics (odometry over body
1.115); with the inner rim sticking at 0.200 / 0.177 = 1.130 (odometry over body 0.885). The #86 range 0.877 to 1.109
sits on these two limits. In this experiment the baseline turned between 0.897 and 0.983 of the wheel kinematics.
A single-point contact at the nominal track (B_sphere) removes the contact error entirely; a 4 mm cylinder (B_thin)
bounds it to 1 %. A (friction direction) has no measurable effect. C changes the proportional slip by up to 0.8 % of
the turn but does not remove the 10 % mode.

## Setup

- Repository `openAMRobot/openamr-upperbody-sw`, branch `diag/spin-test` from `integration/baseline1` `024ccb8`.
  Platform: `openAMRobot/openamr-platform-sw` `931e631` (description, SIL, Nav2), as Baseline 1.
- Run: https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44350480199 (one run, eight variant jobs and
  one report job, all completed with success). Workflow `.github/workflows/spin-test.yml`.
- Launch: `build1.launch.py`, profile `firmware_sil` (ZLAC8015D emulator and host test client on `vcan_drive`, 1 ms
  steps, `gz-physics-dartsim-plugin`), headless, no Nav2, no MoveIt, no rendered sensor. Arms moved to park first.
  Real-time factor 0.25 to 0.41 (mean of the whole run per job).
- Variants (`openamrobot_build1_scene/spin_variants.py`) edit the plant URDF text inside the launch only when
  `BUILD1_SPIN_VARIANT` is set. No description package, parameter file or decision was changed.

| Variant | Edit |
|---|---|
| baseline | none |
| A_iso | drive wheels mu2 = mu1 = 1.5, `fdir1` removed (isotropic) |
| A_axial | drive wheels `fdir1 0 0 1` (cylinder axis, invariant under spin), mu1 and mu2 swapped |
| B_sphere | drive wheel collision: sphere r 0.098 at the axle (one contact at the nominal track) |
| B_thin | drive wheel collision: cylinder r 0.098, width 0.004 m |
| C_coplanar | castor spheres lowered 0.421 mm, all six contacts in one plane at rest |
| C_friction | castor spheres mu1 = mu2 = 0.05 |
| ALL | A_iso + B_sphere + C_coplanar |

- Procedure (`openamrobot_build1_scene/spin_test.py`), three repeats of: turn +pi/2, -pi/2, +pi, -pi in place, then a
  2 m straight (forward, back, forward). `/cmd_vel` 0.25 rad/s is Nav2's in-place rate (`rotation_scaling_factor` 0.5
  x `v_angular_max` 0.5, `v_angular_min_in_place` 0.25); the host client rounds each wheel to 5 r/min, which is
  0.2566 rad/s of body rate. Straight `/cmd_vel` 0.2 m/s (`max_vel_x`), 19 r/min, 0.195 m/s. Timed, open loop.
- Per turn: the wheel angles the drive emulator demanded (`/build1/drive/*_wheel_cmd_vel` integrated over Gazebo
  time), the Gazebo wheel joint angles (`/sil/gz/joint_states`), the kinematic yaw r (dR - dL) / track with the
  model's r 0.098 m and track 0.400 m, `/odom` yaw, ground-truth yaw (unwrapped at 20 Hz), and the floor contacts
  from one gz contact sensor on the floor (normal force per drive wheel and of the four castors, force-weighted
  lateral position of each wheel's contact in the base frame, hence the effective track). The residual
  odometry - truth is split into drive chain (odometry - kinematic) and contact (kinematic - truth).

## Results

Summary per variant (12 turns and 3 straights each). Offset and scale: least-squares fit of each part against the
ground-truth yaw over the 12 turns, part = offset + scale x truth. Offset is the direction-independent error per turn.

| Variant | Odom/truth min..max | Summed ratio | Residual offset (rad/turn) | Residual scale | Drive chain offset, scale | Contact offset, scale | Track eff mean (min..max) m | Castor share rest, turns | Straight odom/truth |
|---|---|---|---|---|---|---|---|---|---|
| baseline | 1.017..1.122 | 1.0315 | -0.0012 | +0.0268 | -0.0010, +0.0001 | -0.0002, +0.0267 | 0.3982 (0.3966..0.4024) | 0.060, 0.061 | 1.0004 |
| A_iso | 1.015..1.115 | 1.0304 | -0.0003 | +0.0257 | -0.0006, -0.0006 | +0.0003, +0.0263 | 0.3981 (0.3967..0.4023) | 0.060, 0.061 | 0.9989 |
| A_axial | 1.010..1.116 | 1.0307 | -0.0014 | +0.0266 | -0.0005, +0.0005 | -0.0009, +0.0261 | 0.3981 (0.3967..0.4024) | 0.060, 0.061 | 1.0002 |
| B_sphere | 0.997..1.003 | 1.0004 | +0.0016 | +0.0005 | +0.0016, +0.0005 | -0.0001, -0.0000 | 0.4000 (0.4000..0.4000) | 0.060, 0.061 | 0.9995 |
| B_thin | 1.001..1.010 | 1.0033 | +0.0008 | +0.0029 | +0.0007, -0.0001 | +0.0001, +0.0030 | 0.4000 (0.4000..0.4000) | 0.060, 0.061 | 0.9987 |
| C_coplanar | 1.007..1.115 | 1.0237 | +0.0004 | +0.0186 | +0.0004, +0.0001 | +0.0000, +0.0185 | 0.3988 (0.3963..0.4050) | 0.403, 0.403 | 1.0006 |
| C_friction | 1.017..1.115 | 1.0340 | -0.0005 | +0.0300 | -0.0003, -0.0001 | -0.0002, +0.0302 | 0.3981 (0.3967..0.4023) | 0.060, 0.061 | 1.0000 |
| ALL | 0.996..1.003 | 0.9999 | -0.0011 | -0.0002 | -0.0011, -0.0002 | +0.0001, +0.0000 | 0.4000 (0.4000..0.4000) | 0.576, 0.576 | 0.9995 |

Effect estimate per cause (baseline minus variant):

| Variant | Cause removed | Residual scale removed | Odom/truth spread removed | Mean abs residual removed (rad) |
|---|---|---|---|---|
| A_iso | A | +0.0010 | +0.006 | +0.0027 |
| A_axial | A | +0.0002 | -0.001 | +0.0018 |
| B_sphere | B | +0.0263 | +0.099 | +0.0704 |
| B_thin | B | +0.0239 | +0.096 | +0.0645 |
| C_coplanar | C (clearance) | +0.0082 | -0.003 | +0.0175 |
| C_friction | C (frictionless castors) | -0.0033 | +0.007 | -0.0055 |
| ALL | A+B+C | +0.0269 | +0.098 | +0.0690 |

Baseline, per turn (wheel angles in rad, yaw in rad, normal forces in N during the steady part of the turn):

| Turn | Rep | Demand L, R | Wheels L, R | Kinematic | Odometry | Truth | Odom/truth | Residual | Drive chain | Contact | N left, right, castors | Track eff (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| +pi/2 | 1 | -3.235, +3.235 | -3.234, +3.234 | +1.5848 | +1.5856 | +1.5496 | 1.023 | +0.0360 | +0.0008 | +0.0352 | 399, 403, 52 | 0.3974 |
| -pi/2 | 1 | +3.204, -3.204 | +3.204, -3.204 | -1.5702 | -1.5704 | -1.5349 | 1.023 | -0.0355 | -0.0003 | -0.0352 | 394, 408, 52 | 0.3974 |
| +pi | 1 | -6.431, +6.431 | -6.431, +6.431 | +3.1511 | +3.1506 | +3.0812 | 1.023 | +0.0694 | -0.0005 | +0.0699 | 397, 405, 52 | 0.3974 |
| -pi | 1 | +6.397, -6.397 | +6.398, -6.398 | -3.1349 | -3.1301 | -3.0650 | 1.021 | -0.0650 | +0.0049 | -0.0699 | 397, 405, 52 | 0.3974 |
| +pi/2 | 2 | -3.218, +3.218 | -3.215, +3.215 | +1.5753 | +1.5753 | +1.4128 | 1.115 | +0.1625 | +0.0000 | +0.1625 | 396, 406, 52 | 0.4019 |
| -pi/2 | 2 | +3.216, -3.219 | +3.215, -3.218 | -1.5762 | -1.5856 | -1.4135 | 1.122 | -0.1721 | -0.0093 | -0.1627 | 396, 406, 52 | 0.4024 |
| +pi | 2 | -6.409, +6.419 | -6.409, +6.418 | +3.1426 | +3.1403 | +3.0889 | 1.017 | +0.0514 | -0.0023 | +0.0537 | 396, 406, 52 | 0.3966 |
| -pi | 2 | +6.430, -6.430 | +6.431, -6.431 | -3.1511 | -3.1557 | -3.0965 | 1.019 | -0.0592 | -0.0046 | -0.0546 | 396, 406, 52 | 0.3979 |
| +pi/2 | 3 | -3.225, +3.225 | -3.226, +3.226 | +1.5809 | +1.5804 | +1.5426 | 1.025 | +0.0378 | -0.0005 | +0.0383 | 399, 403, 52 | 0.3975 |
| -pi/2 | 3 | +3.216, -3.216 | +3.215, -3.215 | -1.5753 | -1.5753 | -1.5370 | 1.025 | -0.0383 | -0.0000 | -0.0383 | 394, 408, 52 | 0.3976 |
| +pi | 3 | -6.430, +6.430 | -6.429, +6.430 | +3.1505 | +3.1506 | +3.0807 | 1.023 | +0.0699 | +0.0001 | +0.0698 | 397, 405, 52 | 0.3973 |
| -pi | 3 | +6.429, -6.429 | +6.430, -6.430 | -3.1506 | -3.1506 | -3.0807 | 1.023 | -0.0699 | -0.0000 | -0.0699 | 397, 405, 52 | 0.3974 |

Baseline straights: odometry over truth 1.0014, 0.9996, 1.0002; ground-truth heading change -0.0061, 0.0000,
-0.0050 rad per 2 m. B_sphere, per turn: odometry over truth 0.997 to 1.003, contact part within +-0.0004 rad in all
12 turns; the remaining residual (up to 0.0051 rad) is the drive chain. The full per-turn tables of every variant are
in the report job's summary and the `spin-test-report` artifact of the run.

Wheel normal load split, arms parked, at rest: baseline drive wheels 390.5 N and 412.4 N, castors 50.8 N together
(castor share 0.060 of 853.7 N, the 87.1 kg model); C_coplanar 0.403; ALL 0.576. B_sphere and B_thin keep 0.060.
The share barely changes during turns in any variant. Contact forces were reported by DART in every job.

## What the numbers say

- B, the flat wide contact, carries the error. With the 46 mm cylinder each wheel has two contact points (2.0 per
  sample) and the force-weighted contact sits 2 to 5 mm off the nominal track: rest track 0.3975 m, left contact at
  +0.2022 m, right at -0.1953 m. Turns fall into two regimes. Most under-rotate by 1.7 % to 2.4 % (both rims
  slipping). The +-pi/2 turns of repeat 2 (after the first straight, at x about 2 m) under-rotate by 10.3 %:
  1.4128 / 1.5753 = 0.897, the outer-rim limit 0.200 / 0.223. That mode reproduces to four digits in every
  cylinder variant (A_iso, A_axial, C_coplanar, C_friction) and is gone with B_sphere and B_thin (B_thin: 0.990, its
  own 2 mm rim limit). Which rim sticks depends on how DART shares the load between two rim points, so the same model
  gives a few per cent or ten per cent from turn to turn.
- A has no effect. A_iso and A_axial match the baseline within the repeatability (scale 0.0257 and 0.0266 against
  0.0268). Source reading agrees: gz-physics dartsim (`dartsim/src/SDFFeatures.cc`, gz-physics7) passes `fdir1` with
  no frame unless `gz:expressed_in` is given, so DART 6.13 (`ContactSurface.cpp`, `computeWorldFirstFrictionDir`)
  rotates it with the collision shape, as suspected. But `ContactConstraint::getTangentBasisMatrixODE` builds the
  friction axes from n x fdir1: for a wheel spinning about a horizontal axle the projection of the collision x axis on
  the floor is always the rolling direction, and the basis degenerates only within about 1e-6 rad of vertical. The
  mu 1.5 / 0.5 axes therefore do not turn with the wheel in practice. (DART source read at tag v6.13.2; the DART
  version on the runner was not printed.)
- C is a second-order term. Lowering the castors into the plane (castor share 0.06 to 0.40) lowers the fitted contact
  scale from 0.0267 to 0.0185 but keeps the 10.3 % mode. Castor friction 0.05 raises it to 0.0302. Neither changes the
  sign behaviour.
- Drive chain: odometry minus the Gazebo wheel kinematics is 0.0000 in most turns and +-0.005 rad in some, one 50 Hz
  tick of `base_state_adapter.py` at 0.2566 rad/s (0.0051 rad). It is the same in every variant and is the whole
  residual of B_sphere and ALL. It is not a model cause.
- Straight driving is clean in every variant (odometry over truth 0.9987 to 1.0006 on average), as in #86.

## What this experiment does not explain

- In this open-loop test the residual follows the turn direction (offset fits within +-0.0016 rad per turn in every
  variant): with a cylinder contact (six variants, 72 turns) the body under-rotated in every turn. #86 reported a residual of mostly the same sign whichever way the
  robot turned. That was measured in closed loop (Nav2 stops on odometry) with the arms moving and at other
  positions; this experiment did not reproduce it. The #86 ratio range, both below and above 1, is consistent with
  the two rim limits, but which rim sticks in which turn there was not measured.
- Why the outer rim sticks at x about 2 m and not at x about 0 was not isolated. It is deterministic (same digits in
  four variants) and depends on the contact configuration DART finds, not on the floor (one plane).

## Recommendation for the model owners (Proposed, not applied)

Owner: the `openamrobot_description` owners of `openAMRobot/openamr-platform-sw` (CODEOWNERS), with the wheel and
castor model work in `openAMRobot/audits` #90 (model proposal M2, crowned-tyre contact).

1. Replace the drive wheel collision cylinder (r 0.098, width 0.046) in `robo_urdf.urdf.xacro` with a single contact
   at the tyre centre plane: a sphere of r 0.098 at the axle. Measured here: odometry over truth 0.997 to 1.003, no
   contact error, no change to the load split or straight driving. The sphere reaches 0.098 m sideways from the wheel
   centre plane at axle height, 7 mm beyond the 0.582 m chassis and inside the 0.65 m envelope; if that is not
   acceptable for the obstacle or docking geometry, a 4 mm wide cylinder is the fallback (error bounded to 1 %,
   measured 1.001 to 1.010).
2. Leave `fdir1` as it is: it has no measurable effect in DART.
3. Treat the castor clearance and castor friction (C) with the castor model of audits #90; they are second-order for
   turns.
4. Re-run this experiment and the #86 mission on the changed model before any value goes to decisions.yaml, and add
   the Nav2 closed-loop turn case that #86 measured.

## Not verified

- Hardware: nothing here is measured on a robot. Tyre, floor and castor friction and stiffness are model values.
- One run per variant; repeatability across runs and real-time factors was not measured (rtf 0.25 to 0.41).
- Closed-loop (Nav2) turns, arms out, lift raised, other floor positions and speeds other than 0.25 rad/s and 0.2 m/s.
- The DART version on the runner (source read at v6.13.2). Sphere or thin-cylinder side effects on obstacle contact,
  docking and MoveIt collision checking.
- The commands and the evidence: workflow `.github/workflows/spin-test.yml` at `632f804`; static tests
  `test/test_spin_variants.py` (10 passed locally and in each job).
