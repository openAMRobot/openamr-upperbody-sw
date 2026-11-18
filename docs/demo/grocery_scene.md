# Grocery dark store demo: MuJoCo and Isaac Sim

A small grocery dark store (two short aisles, four shelf rows, filler products, a tote with the target product, a packing tote, a
parking spot) around the authoritative robot model, for MuJoCo and for Isaac Sim. Code in `simulation/scenes/grocery/`, CI in
`.github/workflows/grocery-mujoco-isaac.yml`. Capability matrix of all simulators: [simulators.md](simulators.md).

**MuJoCo experiment and demo only: not ROS, not ros2_control, not MoveIt, not Nav2, not firmware integrated.** Nothing here is
hardware, physical acceptance or safety evidence.

## Sources

- Robot: the MJCF and the Isaac import job that `simulation/tools/generate.py` writes from the locked expansion of
  `combined_robot.xacro` (configuration `bp000`). The scene includes the MJCF unchanged; the Isaac stage references the robot USD
  unchanged. Neither generator is modified here.
- Store: `simulation/scenes/grocery/store.yaml`. Every number is `copied` (from the Build 1 Gazebo scene,
  `openamrobot_build1_scene/config/scene.yaml` at #73 head `1a84066`) or `estimated` (demo dressing).
- Target product, as in the Build 1 Gazebo demo: box 0.035 x 0.035 x 0.14 m, 0.15 kg, friction 1.0, in a 0.40 x 0.30 x 0.06 m
  tote (0.6 kg) on the 0.82 m shelf level, 0.16 m to the robot's left. Shelves 1.20 m units, 0.40 m deep, aisle 1.50 m, robot
  footprint front 0.30 m from the shelf front. All of these are estimates in Build 1 too, except the shelf heights (Use_Case_1
  site heights). The Gazebo grocery dark store of #86 (head `d9a5480`, `grocery_scene.yaml`) keeps the same product, tote and
  work-shelf geometry; its store layout differs (2.20 m aisles, four-unit rows), so the two stores are not the same map.

## MuJoCo

```bash
source /opt/ros/jazzy/setup.bash
python3 simulation/tools/generate.py --isaac --config bp000 --platform-sw <pin checkout> --openarm-description <pin checkout> --out build/sim
python3 simulation/scenes/grocery/compose_mujoco.py --robot build/sim/bp000/openamrobot_bp000.xml --out build/grocery/mujoco
GROCERY_ROBOT_MJCF=$PWD/build/sim/bp000/openamrobot_bp000.xml GROCERY_ISAAC_JOB=$PWD/build/sim/bp000/isaac/isaac_job.json \
  python3 -m pytest -p no:cacheprovider simulation/scenes/grocery/tests
python3 simulation/scenes/grocery/grasp_experiment.py --scene build/grocery/mujoco/grocery_scene.xml --out build/grocery/experiment
MUJOCO_GL=egl python3 simulation/scenes/grocery/record_mujoco.py --scene build/grocery/mujoco/grocery_scene.xml --out build/grocery/video
```

`compose_mujoco.py` copies the generated MJCF and its meshes byte for byte and writes `grocery_scene.xml`, which pulls the robot in
with `<include>`. The tests check the copy against the generator output (bytes and SHA-256), check that every robot body, joint,
actuator, geom, equality and keyframe compiles to the same values inside the scene as alone, and fail as BLOCKED when the MJCF was
generated without the lock check. Scene geoms use contype/conaffinity 3/3 so they touch the robot (1/2) and each other; the product
geom has `priority="1"`, so its friction is the pair coefficient against the fingers and the robot model keeps its own values.

### Controller (pick.py)

The script is the only controller: IK (damped least squares, seven arm joints) for a side grasp square to the product face, 0.025 m
above the product centre, approached over 0.08 m; the generated position actuators for arm, gripper and lift. Two additions, both on
the controller side: gravity feed-forward on the arm joints (the model's own bias force, the MuJoCo analogue of the #73
feed-forward; it has no payload term) and the lift setpoint ramp at 18 mm/s (decisions register LIFT), because the generated model
does not enforce the lift speed.

### Grasp-and-lift sweep (MuJoCo experiment, not ROS or firmware integrated)

For the gripper-force question of #73. Each run edits its own in-memory model only; no shared parameter is changed or proposed. Grip
force comes from the position servo: torque = kp x (finger angle on the product - close command). Columns: finger angle and torque at
the end of the pre-lift hold; normal force is the mean per finger while lifted; need is m g / (2 mu); margin = normal / need; slip is
the product's largest downward motion in the grasp frame during lift and hold; creep is its rate in the last hold second.

MuJoCo 4.0.1, step 1/1500 s, elliptic cone, impratio 10, recorded on 18 Nov 2026 (sandbox run; the CI run writes the same table to
its step summary).

| mu | close cmd [rad] | kp [N m/rad] | finger angle [rad] | grip torque [N m] | normal / finger [N] | need [N] | margin | lift travel [m] | hand rise [m] | product rise [m] | slip max [m] | creep [m/s] | held |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.2 | 0 | 5 | 0.2071 | 1.035 | 0 | 3.679 | 0 | 0.05977 | 0.06098 | -0.05253 | 0.1138 | 0.0008992 | NO |
| 0.2 | 0.2 | 5 | 0.2508 | 0.2539 | 0.3015 | 3.679 | 0.08195 | 0.05977 | 0.05987 | 0.001411 | 0.05844 | 0.0009717 | NO |
| 0.2 | 0.25 | 5 | 0.2933 | 0.2165 | 0.2591 | 3.679 | 0.07042 | 0.05977 | 0.06005 | 0.001018 | 0.05903 | 0.0008819 | NO |
| 0.3 | 0 | 5 | 0.2912 | 1.456 | 9.094 | 2.453 | 3.708 | 0.05976 | 0.0507 | 0.04476 | 0.005613 | 0.0004167 | yes |
| 0.3 | 0.2 | 5 | 0.3033 | 0.5164 | 0 | 2.453 | 0 | 0.05977 | 0.06078 | -0.05251 | 0.1134 | 0.001029 | NO |
| 0.3 | 0.25 | 5 | 0.3078 | 0.2891 | 0.2193 | 2.453 | 0.08942 | 0.05977 | 0.05994 | 0.001371 | 0.05855 | 0.0009128 | NO |
| 0.5 | 0 | 5 | 0.2925 | 1.463 | 12.16 | 1.472 | 8.267 | 0.05976 | 0.05029 | 0.04908 | 0.001116 | 0.0002346 | yes |
| 0.5 | 0.2 | 5 | 0.3046 | 0.5229 | 4.176 | 1.472 | 2.838 | 0.05976 | 0.05054 | 0.04624 | 0.004254 | 0.0004611 | yes |
| 0.5 | 0.25 | 5 | 0.3086 | 0.2931 | 0 | 1.472 | 0 | 0.05977 | 0.06078 | -0.0525 | 0.1135 | 0.0007441 | NO |
| 1 | 0 | 5 | 0.293 | 1.465 | 12.2 | 0.7358 | 16.59 | 0.05976 | 0.05026 | 0.04905 | 0.001116 | 0.0002377 | yes |
| 1 | 0.2 | 5 | 0.3047 | 0.5237 | 4.35 | 0.7358 | 5.912 | 0.05976 | 0.05017 | 0.04846 | 0.001682 | 0.0003626 | yes |
| 1 | 0.25 | 5 | 0.3087 | 0.2935 | 2.395 | 0.7358 | 3.255 | 0.05976 | 0.04999 | 0.04601 | 0.003977 | 0.0005101 | yes |
| 0.5 | 0 | 1.25 | 0.3076 | 0.3845 | 3.21 | 1.472 | 2.181 | 0.05976 | 0.05044 | 0.04591 | 0.004485 | 0.0004686 | yes |
| 0.5 | 0 | 2.5 | 0.3017 | 0.7543 | 6.481 | 1.472 | 4.404 | 0.05976 | 0.05068 | 0.04689 | 0.00383 | 0.002608 | yes |
| 0.5 | 0 | 10 | 0.2731 | 2.731 | 21.68 | 1.472 | 14.73 | 0.05976 | 0.05039 | 0.04914 | 0.001064 | 0.0002199 | yes |

What the numbers say, within MuJoCo:

- With the Build 1 settings (mu 1.0, close command 0, kp 5 N m/rad) the fingers stop on the product at 0.293 rad, the grip torque
  is 1.47 N m and the normal force 12.2 N per finger, 16.6 times the static need of 0.74 N. The product rises 49 mm for 60 mm of lift
  travel (hand rise 50 mm: about 10 mm of arm sag under the payload, which the feed-forward does not cover) and slips 1.1 mm.
- 6.2 to 8.6 N of normal force per finger per N m of grip torque in the held runs (a lever of about 0.12 to 0.16 m from the finger
  axis), so normal force ~ 8 x torque is a usable estimate in this model.
- Every run with a measured margin of 2.18 or more held. The runs that lost the product had an estimated margin (8 x torque / need)
  of 1.7 or less (mu 0.3 at 0.52 N m: 1.7; mu 0.5 at 0.29 N m: 1.6), except at mu 0.2 (next point). So in this model the grasp
  needs a static margin of about 2, not 1; the lift start and the soft-contact creep were not separated. For lost runs the angle and torque columns are the
  finger state at the end of the hold, and the normal force is 0 or below 0.31 N.
- At mu 0.2 the full close command (1.04 N m) squeezed the product 24 mm out along the approach axis before the lift (9 mm at close
  command 0.2): the fingers ended at 0.207 rad, past the product width. In the other lost runs the product stayed centred and slipped
  at the lift start: it either stayed standing while the fingers slid up, or toppled in the tote (centre at 0.842 m, on its side).
- Gazebo (DART, #73) reported fingers meeting the item at 0.134 to 0.162 rad, 0.67 to 0.72 N m and 2.7 to 4.7 N with the same
  command. MuJoCo closes on the product at about 0.29 rad, probably because it collides with the convex hull of each finger mesh: the
  raw vertices of the fingertip region give a 35 mm gap only at about 0.19 rad. The two engines therefore give different grip forces for the same position command; the force follows
  the contact geometry, not the command. That supports the #73 conclusion that a measured grip force (or a force-mode gripper) is
  needed before any threshold is set from simulation.

Not a parameter proposal: kp factors and close commands other than the generated values are sweep points only.

### Recording

`record_mujoco.py` writes `grocery_pick.mp4` (1280 x 720, 30 fps, H.264) and six stills. Every frame carries its mode:

| Segment | Mode |
|---|---|
| Establishing shot, robot at the parking spot | kinematic (no stepping) |
| Drive along aisle 1 (3.2 m at 0.45 m/s, below the 0.5 m/s Build 1 ceiling) and turn to the shelf; arm to pre-grasp | **kinematic**: base pose set per frame, wheels turned by distance over radius; arm joints interpolated |
| Approach, close, lift, hold | **controlled**: MuJoCo physics, `pick.py` on the generated actuators, Build 1 settings, real time |
| Pull-back over the store | frozen state, camera only |

Outputs of the sandbox run at `44f8c2b` (binary output branch `viz/grocery-mujoco-output`, commit `6125afa`, not for merge; the
robot MJCF was generated there without the lock check, expansion SHA-256 `5d9a1b51`, the locked value):

- Video (25.2 s, 755 frames, SHA-256 `8d377950...66bd3f`): https://raw.githubusercontent.com/openAMRobot/openamr-upperbody-sw/6125afa50359bfeaa462d61668142ee6ab20944f/grocery/grocery_pick.mp4
- Stills:
  - https://raw.githubusercontent.com/openAMRobot/openamr-upperbody-sw/6125afa50359bfeaa462d61668142ee6ab20944f/grocery/still_1_store.png
  - https://raw.githubusercontent.com/openAMRobot/openamr-upperbody-sw/6125afa50359bfeaa462d61668142ee6ab20944f/grocery/still_2_aisle.png
  - https://raw.githubusercontent.com/openAMRobot/openamr-upperbody-sw/6125afa50359bfeaa462d61668142ee6ab20944f/grocery/still_3_approach.png
  - https://raw.githubusercontent.com/openAMRobot/openamr-upperbody-sw/6125afa50359bfeaa462d61668142ee6ab20944f/grocery/still_4_lift.png
  - https://raw.githubusercontent.com/openAMRobot/openamr-upperbody-sw/6125afa50359bfeaa462d61668142ee6ab20944f/grocery/still_5_held.png
  - https://raw.githubusercontent.com/openAMRobot/openamr-upperbody-sw/6125afa50359bfeaa462d61668142ee6ab20944f/grocery/still_6_overview.png

The CI job uploads the same files as the artifact `grocery-mujoco-isaac`.

## Isaac Sim

`compose_isaac.py` writes `grocery_store.usda` with usd-core (no GPU): the store (UsdGeom cubes and cylinders with
UsdPhysics.CollisionAPI; the tote and the product as rigid bodies with mass), lights, a reference to the robot USD of the #80
importer at the pick station, and sensor prims placed under the robot's link prims:

| Prim | Parent link | Sensor | Note |
|---|---|---|---|
| `base_camera_rtx` (UsdGeom.Camera) | `base_camera_link` | Orbbec Gemini 336L (decisions register CAMERAS) | horizontal FOV 1.047 rad: the Gazebo camera value, not a datasheet value |
| `head_camera_rtx` | `head_camera_link` | Stereolabs ZED Mini | FOV 45 deg as in the generated MJCF |
| `left_wrist_camera_rtx`, `right_wrist_camera_rtx` | wrist camera links | wrist RGB | FOV 45 deg as in the generated MJCF |
| `rtx_lidar_mount` (Xform) | `lidar_link` | RPLIDAR S3 | `isaac_render.py` creates the OmniLidar here; no S3 profile was found in the 5.1 library, `Example_Rotary` stands in |

Cameras look along the link's x axis with the link's z up (USD cameras look along -Z with +Y up). Units: metres and kilograms, Z
up. If the robot USD brings no physics scene, the stage gets one with the job's 1500 steps per second and gravity.

CPU checks (`tests/test_isaac_scene.py`, usd-core): stage units and up axis, collider and rigid-body counts, product and tote mass and
size, one physics scene, exactly one articulation root, the robot USD file unchanged and no property opinion on any robot link, every
sensor at its link origin with the right axes and FOV, and the base camera on `base_camera_link` (not the legacy `camera_link`).
The sensor tests use a **kinematic stand-in** of the robot USD that the test writes from the job's URDF (one Xform per link at its
URDF pose). It checks the composition, not what Isaac Sim imports.

### GPU step: BLOCKED

No GPU here and none on the hosted runners: the robot USD import and the render have never run. BLOCKED is not PASS. On a machine
with an NVIDIA GPU (driver and the NVIDIA Container Toolkit installed), ROS 2 Jazzy for xacro, and the pinned checkouts:

```bash
# 1. MJCF and Isaac import job, with the lock check (as in CI)
source /opt/ros/jazzy/setup.bash
python3 simulation/tools/generate.py --isaac --config bp000 --platform-sw <pin checkout> --openarm-description <pin checkout> --out build/sim
# 2. robot USD with the #80 importer, inside Isaac Sim 5.1.0
docker run --rm --gpus all -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -v "$PWD":/work -w /work --entrypoint /isaac-sim/python.sh \
  nvcr.io/nvidia/isaac-sim:5.1.0 simulation/tools/isaac_generate_usd.py --job build/sim/bp000/isaac/isaac_job.json
python3 simulation/tools/check_usd.py --job build/sim/bp000/isaac/isaac_job.json --platform-sw <pin checkout> --openarm-description <pin checkout>
# 3. the grocery stage (usd-core, no GPU)
python3 simulation/scenes/grocery/compose_isaac.py --robot-usd build/sim/bp000/isaac/openamrobot_bp000.usd \
  --job build/sim/bp000/isaac/isaac_job.json --out build/grocery/isaac/grocery_store.usda
# 4. the render, inside Isaac Sim 5.1.0
docker run --rm --gpus all -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -v "$PWD":/work -w /work --entrypoint /isaac-sim/python.sh \
  nvcr.io/nvidia/isaac-sim:5.1.0 simulation/scenes/grocery/isaac_render.py --stage build/grocery/isaac/grocery_store.usda \
  --out build/grocery/isaac/render
```

Step 4 writes one PNG per robot camera and an overview camera after 2 s of physics, the RTX LiDAR return count and
`render_log.json`. `isaac_render.py` was written against the Isaac Sim 5.1.0 documentation and has never run; any API difference is
recorded in the log as a failure. To run it in CI, set the repository variable `ISAAC_GPU_RUNNER` to a self-hosted GPU runner label
(the `isaac-render` job then stops BLOCKING, and its GPU step still needs the commands above wired in).

## Not verified

- The Isaac robot USD import, the stage with the real robot USD, RTX camera images and LiDAR returns: BLOCKED (no GPU).
- Any ROS, ros2_control, MoveIt, Nav2 or firmware behaviour in MuJoCo or Isaac Sim: not built.
- Contact numbers against hardware: no grip force, pad friction or product friction was measured. The product values are Build 1
  estimates.
- MuJoCo collides with the convex hulls of the finger meshes; how much of the 0.29 rad contact angle that explains was not isolated.
- The recorded drive is kinematic: it shows no wheel, castor or base dynamics.
- Arm sag under the payload (about 10 mm) is a property of this controller (no payload term), not of the robot.
