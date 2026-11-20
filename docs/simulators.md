# Simulators: MuJoCo and Isaac Sim

The combined model in MuJoCo and in Isaac Sim, generated from the same pinned expansion that Gazebo uses (issue #58). This page says
what each backend models and what it does not, how parity is proved, and how to regenerate everything.

Simulation is development evidence only. Nothing here is hardware evidence, physical acceptance or safety evidence. No safety logic.

## Rules

- **One source.** Every backend file is generated from `combined_robot.xacro` with `hardware_profile:=gazebo` for a named
  configuration (`bp000`, `bp050`). That is the text Gazebo gets. Its SHA-256 is checked against the digital-twin lock before
  anything is written (`simulation/expansion_lock.yaml`, a mirror of `digital-twin/phase1/digital_twin.lock.yaml` in
  openAMRobot/audits at `be17653`). A different hash for the same inputs stops the generator (exit 2). When the description files
  differ from every locked commit, the result counts only if the hash is the locked one for the same configuration, pins and xacro
  build; the status then reads `hash in lock, inputs changed (... re-lock pending)`, never `matches lock`. Otherwise the generator
  stops (exit 3, BLOCKED). On PR #66 head `5adeba3` the status is the second one: re-lock in audits is pending.
- **Lift position is joint state.** It is never an xacro argument. MuJoCo gets one keyframe per lift stop (L1000, L1175, L1350, from
  the SRDF, cross-checked with the shoulder heights of `upperbody_configuration.yaml`). One hash covers every lift position.
- **Generated files are build products.** MJCF, converted meshes, Isaac jobs and USD go to `build/` (git-ignored). They are never
  committed and never edited. A test fails if one is tracked.
- **What URDF cannot carry** lives in one reviewed file next to the joint dynamics profile:
  `ros2/src/openamrobot_upperbody_description/config/sim_dynamics/engine_overrides.yaml` (schema beside it). Every value has a source
  and a locator, or is an engine choice with its method, or is `unknown` with what would supply it. xacro does not read this file, so it
  cannot change the locked expansion (a test checks `xacro --deps`).
- **Parity is not fidelity.** The parity tests assert structure and parameters against the expansion. The physics checks run in one
  backend each, against that backend's own parameters or an independent implementation. No test asserts that two engines behave alike.

## Inputs and pins

| Input | Pin | Where it is pinned |
|---|---|---|
| openamr-upperbody-sw description | locked digest `9dcaa6c2` (the files xacro reads) at PR #66 `6fe5e9c`; PR #66 head `5adeba3` adds a text note to `openarm_v2.yaml`, so its digest is `e4bea9d6` and the expansion hashes are unchanged | `simulation/expansion_lock.yaml` |
| openamr-platform-sw (base description) | `8e7a45428d06b015cdf488ca8e182c450039afbc` | `config/upstream_pins.yaml` |
| openamr-platform-sw (wheel friction, #135) | `07417783bb602c233f7011599dc12e4d81e38a04` | `engine_overrides.yaml` sources |
| enactic/openarm_description | `14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831` | `config/upstream_pins.yaml` |
| enactic/openarm_ros2 (gripper gains) | `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` | `config/upstream_pins.yaml` |
| enactic/openarm_mujoco (comparison only) | `1daab06716b779376dbe7f01fb0548f8b539bb35` | `simulation/sim_pins.yaml` |
| xacro | `ros-jazzy-xacro 2.1.1-1noble.20260519.011123` | lock input; CI installs it exactly |
| MuJoCo, pinocchio, usd-core, trimesh, pycollada, numpy | exact versions | `simulation/requirements.txt` |
| Isaac Sim | 5.1.0, `nvcr.io/nvidia/isaac-sim:5.1.0` | `engine_overrides.yaml` isaac, `simulation/sim_pins.yaml` |

Locked hashes used here: bp000 `5d9a1b51aadd07b0f074beff37cc7ccacab91ec032eee63c39183ae2b79f40ea`,
bp050 `4d21210ed0b2628f9c11be6312e81b33c58246be7ef8231084d3b6ef3c53eb8a` (gazebo profile, PR #66).

## Regenerate

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
python3 -m venv --system-site-packages .venv && .venv/bin/pip install -r simulation/requirements.txt
```

```bash
.venv/bin/python simulation/tools/generate.py --isaac --platform-sw <platform-sw at the pin> --openarm-description <openarm_description at the pin> --out build/sim
```

Output per configuration: `build/sim/<config>/openamrobot_<config>.xml` (robot, floor, light, cameras, keyframes),
`meshes/`, `report_<config>.json` (hash, inputs, counts, derived gains, unsupported list) and `isaac/` (importer URDF and
`isaac_job.json`). View it with `python3 -m mujoco.viewer --mjcf build/sim/bp000/openamrobot_bp000.xml`.

Tests (each checkout at its pin; a missing one fails as BLOCKED):

```bash
OPENAMR_PLATFORM_SW=<pin> OPENAMR_PLATFORM_SW_MAIN=<#135 commit> OPENARM_DESCRIPTION=<pin> OPENARM_ROS2=<pin> OPENARM_MUJOCO=<pin> .venv/bin/python -m pytest -p no:cacheprovider simulation/tests
```

Refresh the lock mirror after audits re-locks (never by hand):

```bash
.venv/bin/python simulation/tools/mirror_lock.py --lock <audits>/digital-twin/phase1/digital_twin.lock.yaml --audits-commit <sha> --clone . --platform-sw <clone> --openarm-description <pin>
```

CI: `.github/workflows/simulators.yml`, job `mujoco` on every pull request (it also runs the CPU Isaac checks: import job and USD checker). Job `isaac` runs only on the GPU runner named by `ISAAC_GPU_RUNNER`; without one it is skipped and the `mujoco` job reports the USD import as BLOCKED (summary and warning annotation).

## What each backend models

| Item | Gazebo (reference, unchanged) | MuJoCo | Isaac Sim |
|---|---|---|---|
| Links and frames | every URDF link (fixed joints lumped by the SDF conversion) | one body per URDF link, same name, never fused | one prim per link (`merge_fixed_joints` false) |
| Joints | URDF types | same names and types; free joint `base_free_joint` at `base_footprint` | importer joints, same names |
| Limits | position, effort, velocity | position range; effort as actuator force range; **no velocity limit** | position (degrees in USD), effort as drive max force |
| Inertials | URDF | mass, CoM, full tensor, never from geometry | importer (`import_inertia_tensor`) |
| Damping, friction | URDF `<dynamics>` (PR #66) | damping, frictionloss | damping in the drive; **joint friction not applied** (PhysX coefficient is unitless) |
| Armature | not applied | from `openarm_v2.yaml` (18 joints) | `physxJoint:armature` |
| Arm and gripper actuators | gz_ros2_control position interface (ideal servo) | position actuators, the real driver's MIT gains (`control_gains.yaml`; gripper kp 5, kd 0.1), clamped to the URDF effort | position drives, same gains in USD units |
| Lift | gz_ros2_control position interface | stiff position servo, gains from the carried mass (0.5 mm static sag), 800 N limit | linear position drive, same gains |
| Wheels | DiffDrive plugin sets wheel velocity | velocity servos, time constant 0.05 s, **no torque limit (unknown)** | velocity drives |
| Castors | fixed spheres (platform-sw calls them a slider approximation) | **passive swivel and roll hinges** (declared difference) | fixed, as the URDF |
| finger_joint2 (SW-09) | URDF mimic; held only with Bullet-Featherstone | equality constraint from the URDF mimic | importer mimic joint (`parse_mimic`) |
| Collision | URDF shapes and meshes | same shapes; convex hull per mesh; floor only (no self collision) | convex hull per mesh; no self collision |
| Visuals | meshes | STL copied; DAE converted to OBJ; mirrored meshes rewritten | importer |
| Contact friction | at the pin, the wheel `<surface>` is dropped (mu 1); #135 gives mu1 1.5, mu2 0.5 | mu 1.5 drive wheels (one coefficient), 1.0 castors and floor | importer defaults |
| Sensors | lidars, IMU, base camera | four cameras on the optical frames; only the base camera (Gemini 336L, datasheet) has a sourced field of view | not imported |
| Step and rate | 1/1500 s, 750 Hz (fine profile) | 1/1500 s; tests write controls at 750 Hz | 1500 steps per second |

### Unsupported semantics (MuJoCo, from `report_<config>.json`)

73 entries per configuration: 19 joint velocity limits, 1 lift velocity limit, 4 Gazebo plugins, 9 Gazebo sensors (7 lidar-type,
1 IMU, 1 camera: the legacy `camera_link` sensor, not represented), 27 Gazebo link tags, 2 ros2_control systems, 2 unknown wheel torque limits, 3 unknown camera
fields of view, and one entry each for collision meshes as convex hulls, anisotropic wheel friction, the soft mimic constraint, self
collision, DAE colours and the castor joints. The Isaac job lists its own five (joint friction, castors, Gazebo items, convex hulls,
wheel torque).

## SW-09: finger_joint2

The single-writer map records that `openarm_*_finger_joint2` has no physics constraint with Gazebo's default engine. Here:

- MuJoCo: an equality constraint per hand, `polycoef = [offset, multiplier, 0, 0, 0]` read from the URDF mimic, solver values from
  openarm_mujoco. No actuator on the follower. Tests: structural (`mimic.*` parity checks) and dynamic (`test_sw09_mimic.py`: the
  follower tracks within 1e-3 rad when the gripper opens; with the constraint switched off it lags by more than 0.3 rad, so the test
  can fail).
- Isaac Sim: the importer's mimic joint. The job asserts `parse_mimic` and the leader, multiplier and offset. `check_usd.py` asserts
  `PhysxMimicJointAPI`, its reference joint and gearing on the USD; its self-test fails a USD without the API. The import itself is
  BLOCKED here.

## Parity tests

`simulation/tests/test_mujoco_parity.py`, tolerances in `simulation/parity_tolerances.yaml`. Each tolerance is at most a tenth of the
precision its source states (a test checks it): masses 1e-6 kg against sources to 1 g, positions 1e-6 m against datums in mm and
upstream origins to 0.1 mm, rotations 1e-6 rad against rpy to 1e-4 rad, limits 1e-9, dynamics 1e-12, mesh bounds 2e-6 m (float32
vertices), gravity torques 1e-6 N*m against the smallest opposing friction 0.04 N*m.

Checked per configuration at nine configurations (three lift stops by three SRDF arm poses, grippers open): topology (bodies, parents,
joints, types), limits, actuators and gains, damping, frictionloss, armature, mimic, masses, CoM, inertia tensors, every link frame,
every joint axis and anchor, camera frames, visual and collision geometry. 2922 checks per configuration, 0 failures locally.
Ten mutation tests change one value each in the compiled model and must fail. The parity reference (the URDF forward kinematics) is
itself checked against pinocchio's frame placements.

## Physics checks (MuJoCo only)

Local results, MuJoCo 4.0.1, bp000 (bp050 alike):

| Check | Expectation | Result |
|---|---|---|
| Static gravity torques | MuJoCo `qfrc_bias` at rest equals pinocchio 4.1.0 generalized gravity on the same expansion, 9 named and 50 seeded configurations | largest difference 6e-14 N*m |
| Base on a flat floor | only wheels touch, both drive wheels touch, normal forces sum to the weight | 854.54 N on 854.54 N; drive wheels and the front castors |
| Castor swivel | driving straight, every castor on the floor trails its mount: 0 rad forward, pi backward, from 90 degrees | front castors within 1e-5 rad |
| Lift travel | ramp 0 to 0.350 m at 0.018 m/s, controls at 750 Hz; error at most the 0.5 mm servo sag; tracking at most sag plus servo lag | final error 0.50 mm, tracking 0.77 mm, peak force 218 N of 800 N, shoulder 1.3495 m above `base_footprint` |
| Both arms hold at 750 Hz | position held with the MIT gains, controls written every two 1/1500 s steps; at rest each actuator torque equals pinocchio's gravity torque within the joint's frictionloss | all 16 actuated joints within frictionloss plus 1e-3 N*m; 2250 updates in 3 s |

### Findings (reported, not changed: no kinematics, mass or limit changes here)

1. **Drive wheel interference.** The drive wheel collision radius is 0.100 m and the axle is 0.098 m above `base_footprint`. The
   castor spheres (radius 0.026 m) end 0.36 mm above the floor plane. On a floor the base rises 1.8 mm and pitches 0.59 degrees onto
   the front castors. The rear castors never touch, so the swivel check covers the front pair only. The same geometry is in Gazebo.
2. **Gazebo at the pin drops the wheel friction.** At platform-sw `8e7a454` the wheel `<surface>` blocks are dropped by the URDF to
   SDF conversion (platform-sw #135 says so and fixes it on main). The combined model still reads the pin.
3. **openarm base_link CoM.** openarm_description gives the arm base_link CoM at x +2.53 mm, z +0.03 mm in the base frame;
   openarm_mujoco gives the opposite signs. The link is welded to the lift carriage, so arm dynamics do not depend on it.
4. **Camera fields of view.** The base camera is the Gemini 336L on `base_camera_color_optical_frame` (the C9 `base` stream), with the
   vertical field of view of its RGB module from the Orbbec Gemini 330 Series Datasheet V1.6, Table 3-4-5: 68 degrees (94 horizontal,
   16:10, 1280 x 800, tolerance 3 degrees). The legacy `camera_link` and its Gazebo sensor (1.047 rad) are not 2.0 hardware and are
   not used. Head and wrist cameras use MuJoCo's 45 degrees until a datasheet is filed.

## The arms against openarm_mujoco

`simulation/tools/compare_openarm_mujoco.py` compiles both models and compares each arm in its base frame: joint ranges, damping,
frictionloss, armature, actuator gains and ranges, every body frame at the zero pose and eight seeded poses, masses, CoM, inertia
tensors, collision geom counts, the mimic constraint and the solver options. Joint ranges, dynamics, frames, masses, inertia tensors
and the mimic constraint agree. The differences (a test keeps this table equal to the tool's output):

- Actuator gains: this model uses the real driver's MIT gains, upstream its own simulator gains.
- Control ranges: upstream rounds the URDF limits to five digits; the gripper's upstream range reaches past the joint limit.
- Collision geoms: upstream splits link5 and the fingers into convex parts; this model has one mesh each (one hull).
- Base_link CoM sign (finding 3). Timestep 1/1500 s here, 1 ms upstream.

<!-- openarm_mujoco differences: begin -->
| Item | Where | This model | openarm_mujoco |
|---|---|---|---|
| actuator kp | openarm_left_joint1 | 70.0 | 230.0 |
| actuator kv | openarm_left_joint1 | 2.75 | 2.7 |
| actuator control range | openarm_left_joint1 | [-3.4907, 1.3963] | [-3.49066, 1.39626] |
| actuator kp | openarm_left_joint2 | 70.0 | 230.0 |
| actuator kv | openarm_left_joint2 | 2.5 | 2.7 |
| actuator kp | openarm_left_joint3 | 70.0 | 190.0 |
| actuator kv | openarm_left_joint3 | 2.0 | 2.2 |
| actuator kp | openarm_left_joint4 | 60.0 | 190.0 |
| actuator kv | openarm_left_joint4 | 2.0 | 2.2 |
| actuator control range | openarm_left_joint4 | [0.0, 2.4435] | [0.0, 2.44346] |
| actuator kp | openarm_left_joint5 | 10.0 | 30.0 |
| actuator kv | openarm_left_joint5 | 0.7 | 1.5 |
| actuator kp | openarm_left_joint6 | 10.0 | 30.0 |
| actuator kv | openarm_left_joint6 | 0.6 | 1.5 |
| actuator kp | openarm_left_joint7 | 10.0 | 30.0 |
| actuator kv | openarm_left_joint7 | 0.5 | 1.5 |
| actuator kp | openarm_left_finger_joint1 | 5.0 | 10.0 |
| actuator kv | openarm_left_finger_joint1 | 0.1 | 0.2 |
| actuator control range | openarm_left_finger_joint1 | [0.0, 0.7854] | [-0.4, 0.7854] |
| centre of mass in arm base frame, m | openarm_left_base_link / openarm_left_base_link | [0.002531, 0.025819, 2.7e-05] | [-0.002531, 0.025819, -2.7e-05] |
| collision geoms | openarm_left_link5 | 1 | 3 |
| collision geoms | openarm_left_ee_link1 | 1 | 4 |
| collision geoms | openarm_left_ee_link2 | 1 | 4 |
| actuator kp | openarm_right_joint1 | 70.0 | 230.0 |
| actuator kv | openarm_right_joint1 | 2.75 | 2.7 |
| actuator control range | openarm_right_joint1 | [-1.3963, 3.4907] | [-1.39626, 3.49066] |
| actuator kp | openarm_right_joint2 | 70.0 | 230.0 |
| actuator kv | openarm_right_joint2 | 2.5 | 2.7 |
| actuator kp | openarm_right_joint3 | 70.0 | 190.0 |
| actuator kv | openarm_right_joint3 | 2.0 | 2.2 |
| actuator kp | openarm_right_joint4 | 60.0 | 190.0 |
| actuator kv | openarm_right_joint4 | 2.0 | 2.2 |
| actuator control range | openarm_right_joint4 | [0.0, 2.4435] | [0.0, 2.44346] |
| actuator kp | openarm_right_joint5 | 10.0 | 30.0 |
| actuator kv | openarm_right_joint5 | 0.7 | 1.5 |
| actuator kp | openarm_right_joint6 | 10.0 | 30.0 |
| actuator kv | openarm_right_joint6 | 0.6 | 1.5 |
| actuator kp | openarm_right_joint7 | 10.0 | 30.0 |
| actuator kv | openarm_right_joint7 | 0.5 | 1.5 |
| actuator kp | openarm_right_finger_joint1 | 5.0 | 10.0 |
| actuator kv | openarm_right_finger_joint1 | 0.1 | 0.2 |
| actuator control range | openarm_right_finger_joint1 | [-0.7854, 0.0] | [-0.7854, 0.4] |
| centre of mass in arm base frame, m | openarm_right_base_link / openarm_right_base_link | [0.002531, -0.025819, 2.7e-05] | [-0.002531, -0.025819, -2.7e-05] |
| collision geoms | openarm_right_link5 | 1 | 3 |
| collision geoms | openarm_right_ee_link1 | 1 | 4 |
| collision geoms | openarm_right_ee_link2 | 1 | 4 |
| timestep s | option | 0.000667 | 0.001 |

270 items compared, 47 differ.
<!-- openarm_mujoco differences: end -->

## Isaac Sim

Status: **BLOCKED** (no GPU in CI or in the session that wrote this). BLOCKED is not PASS. What runs without a GPU: the import job
(`generate.py --isaac`), its tests, and the USD checker's self-test on a small USD built with usd-core.

On a machine with an NVIDIA GPU, Docker with the NVIDIA runtime, ROS 2 Jazzy and the pinned checkouts, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
.venv/bin/python simulation/tools/generate.py --isaac --config bp000 --platform-sw <pin checkout> --openarm-description <pin checkout> --out build/sim
```

```bash
docker run --rm --gpus all -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -v "$PWD":/work -w /work --entrypoint /isaac-sim/python.sh nvcr.io/nvidia/isaac-sim:5.1.0 simulation/tools/isaac_generate_usd.py --job build/sim/bp000/isaac/isaac_job.json
```

```bash
.venv/bin/python simulation/tools/check_usd.py --job build/sim/bp000/isaac/isaac_job.json --platform-sw <pin checkout> --openarm-description <pin checkout> --json build/sim/bp000/isaac/check_usd.json
```

The importer URDF uses absolute mesh paths under the checkouts, so the container must see them at the same paths (keep the checkouts
inside the repository directory, or mount them at the same path). `isaac_generate_usd.py` sets every importer setting of the job and
stops if the installed importer lacks one of `merge_fixed_joints`, `fix_base`, `import_inertia_tensor`, `self_collision`,
`parse_mimic`. It then writes drives (angular stiffness and damping per degree, as USD physics defines them), armature and 1500
physics steps per second, and records the expansion hash in the USD layer. `check_usd.py` re-expands from the pins, checks the job
belongs to that expansion, and compares links, frames, masses, CoM, inertias, joints, limits, drives, armature, mimic and the scene.
For CI, set the repository variable `ISAAC_GPU_RUNNER` to a GPU runner label; the `isaac` job then runs there.

## Prior work

- Issue #58 (Manas) set the scope: MJCF from the xacro, actuators, cameras, a parity test at L1000, L1175, L1350 for bp000 and bp050.
  No comments or linked branches were on the issue on 15 Nov 2026.
- OpenRAL pull request 497 (merged 12 Nov 2026) generates a MuJoCo twin of this robot for navigation from a composed URDF. It was read
  for approach only. No file of it is used here; its licence could not be confirmed from the repository page.
- enactic/openarm_mujoco (Apache-2.0) is the comparison reference and the source of the solver values and mimic constraint values,
  copied as numbers. No file of it is copied or loaded into the generated model.
- PR #66 (sim dynamics, head `5adeba3`) provides the damping, friction and armature profile, the 750 Hz overlay and the 1/1500 s world. This work is
  stacked on it while it is not merged.

## Not verified

- Isaac Sim was not run: no USD exists, so `check_usd.py` has only run on its self-test USD. The importer API names and the
  `PhysxMimicJointAPI` attribute names and gearing sign are from the Isaac Sim 5.x documentation as recalled, not from a run.
- No value is measured. MuJoCo results say the model follows its parameters, not that it follows the robot.
- The lift and wheel servo gains are engine choices with a stated method, not actuator data. Wheel torque limits and castor bearing
  friction are unknown and not applied.
- Self collision is off, as in Gazebo. Arm to arm and arm to body contact is not simulated.
- The anisotropic drive wheel friction of #135 cannot be represented; one coefficient (1.5) is used.
