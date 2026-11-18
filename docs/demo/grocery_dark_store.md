# Grocery dark store demonstrator (Gazebo, sim-oracle profile)

Simulation only. Nothing on this page is hardware, physical acceptance, perception or safety evidence.

## What it shows

The whole OpenAMRobot 2.0 Build 1 robot (base, lift, two OpenArm arms, grippers) runs one pick-and-place order in a
small grocery dark store, in one Gazebo launch, on the same software path as Integrated Digital Twin Baseline 1
(`docs/integration/baseline1.md`): every device on virtual CAN (`firmware_sil`), Nav2 with the station tree of
openamr-platform-sw #164, MoveIt 2, CTL-017 lift with a permit, the unmodified OpenArm driver, Gazebo on simulation time.

The store (`ros2/src/openamrobot_build1_scene/config/grocery_scene.yaml`, 14 m x 10 m):

- two aisles of shelving, 2.2 m wide, between three rows of 1.2 m shelf units (row A, a back-to-back gondola B, row C),
  boards at 0.30, 0.82, 1.09 and 1.36 m, closed plinths that the LiDAR sees, back panels, rack-end bollards (features
  along the aisles for AMCL);
- filler products on the boards, visual only (no collision, not in the map, not in the planning scene);
- the pick unit: the Baseline 1 work shelf, unchanged, as unit 1 of row A. On its 0.82 m level the tray and the target
  product (0.035 x 0.035 x 0.14 m, 0.15 kg, both estimated); on 1.09 m the 1.0 kg payload box;
- a packing bench (top 0.80 m) with an order tote (0.50 x 0.40 x 0.06 m) fixed on it;
- a parking bay at the world origin, where the robot starts and ends, and a charging dock on the west wall (not used).

The mission (`openamrobot_build1_scene/grocery_demo.py`), twelve phases in one run:

| # | Phase | What the robot does | Checks (limits) |
|---|---|---|---|
| 1 | spawn | rests in the parking bay | pose and odometry against ground truth (0.02 m, 0.02 rad); `pose_source` read and required to be `ground_truth` |
| 2 | nav_to_shelf | Nav2 with the station tree from parking into aisle 1, to the work-shelf station | Nav2 SUCCEEDED; Nav2 tolerance (0.35 m, 0.35 rad); arm-reach budget of Baseline 1 (0.04 m, 0.08 rad); odometry |
| 3 | stop_authorize | stops; the lift refuses a command without permit; permit fresh | Baseline 1, unchanged |
| 4 | lift_to_target | L1000 to L1175 and back to L1000 (shelf stop of level 0) | Baseline 1, unchanged |
| 5 | arm_approach | left arm to park, gripper open, MoveIt approach, straight reach | scn04 checks and limits, unchanged |
| 6 | grasp | gripper closes on the product | scn04 checks and limits, unchanged |
| 7 | lift_object | hand rises 0.08 m and holds 3 s; the product is not released | item rise 0.04 to 0.12 m, drift 0.01 m (scn04) |
| 8 | retreat_with_item | Nav2 backup 0.30 m with the product held | Baseline 1 backup checks; item rise 0.04 m; product to TCP 0.05 m |
| 9 | nav_to_packing | Nav2 with the station tree out of aisle 1 to an approach pose 0.60 m in front of the packing station, facing the bench, then Nav2 drive_on_heading straight in (distance from the AMCL pose) | Nav2 SUCCEEDED (both) and Nav2 tolerance; item rise 0.04 m; product to TCP 0.05 m |
| 10 | place | lowers the product into the order tote, opens, withdraws 0.10 m | Cartesian fraction 0.999 and MoveIt success; product centre inside the tote's inner walls; height above the tote floor; speed 0.01 m/s after 1 s; gripper open (scn04 window) |
| 11 | retreat | both arms to park, Nav2 backup 0.30 m | Baseline 1 retreat, unchanged |
| 12 | return_to_parking | Nav2 with the station tree back to the parking bay | Nav2 SUCCEEDED and tolerance; odometry |

Where the same physical requirement applies, the check and its limit are reused unchanged (station 0.04 m and 0.08 rad
at the work shelf, item rise 0.04 m, the scn04 grasp limits). The odometry guard is scn01's 0.10 m per 2 m, scaled to
0.05 m per metre driven since the spawn (odometry is never reset; derived). At the packing bench the requirement is
that the product lands in the tote, so the tote checks decide there and the station is held to Nav2's own tolerance.

## Profile declaration

This demonstrator runs in the **sim-oracle** profile only (`docs/integration/baseline1.md`, Acceptance profiles):

- `pose_source: ground_truth`. The MoveIt planning scene is placed from the simulator's ground truth base pose, and the
  grasp target is the product's ground truth pose. The mission reads `pose_source` from `planning_scene_loader` at run
  time and fails the spawn phase if it is anything else; the value is printed in the mission report.
- Station positioning: the station tree of openamr-platform-sw #164, no marker alignment (#167 is not used).
- A pass is integration evidence of the ROS 2 stack, the firmware-SIL path, the controllers and the contact physics in
  simulation. It is never a perception result: the real robot does not have the ground truth.

## Pinned inputs

| Input | Repository | SHA |
|---|---|---|
| Base of this demonstrator | openAMRobot/openamr-upperbody-sw `integration/baseline1` | branched at `8f8df24`, merged up to `c023dc2` |
| Platform (description, Nav2 and #164 station tree, perception, docking, `openamrobot_sil` with #162 and #166) | openAMRobot/openamr-platform-sw `integration/baseline1` | `e75e16e159d6c68888ba2ade2fa4de72dba13123` (the pin of baseline1-acceptance.yml since 4a67ee7; only openamrobot_station_alignment, not built here, differs from fe28d19) |
| OpenArm driver | enactic/openarm_ros2 | `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` |
| OpenArm CAN | enactic/openarm_can | `bfd4e52194aefe38389fa6e670be3738136facb9` |
| OpenArm description | enactic/openarm_description | `14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831` |
| Gazebo | gz-sim 8.15.0, sdformat 14.9.0 (vendor builds as openamr-platform-sw #152) | see `.github/workflows/grocery-demo.yml` |

ROS 2 Jazzy on Ubuntu 24.04. Seed `BASELINE1_SEED=20261117` (Python random and the ROS domain), `PYTHONHASHSEED`
fixed; Gazebo has no seed argument, so two runs are not guaranteed to be identical.

## Run it locally

Workspace layout as in `.github/workflows/grocery-demo.yml` (this repository, openamr-platform-sw at the SHA above and
the three OpenArm repositories under `ws/src`), with the pinned Gazebo vendor packages installed:

```bash
cd ws
source /opt/ros/jazzy/setup.bash
rosdep install --rosdistro jazzy --from-paths $(colcon list --base-paths src --packages-up-to openamrobot_build1_scene --paths-only) --ignore-src -y -r
colcon build --packages-up-to openamrobot_build1_scene
source install/setup.bash
ros2 run openamrobot_build1_scene setup_vcan_all.sh          # five virtual CAN buses, needs sudo
```

The whole mission, headless, with every recording (as CI does it):

```bash
export BUILD1_LOG_DIR=$PWD/build1_logs BUILD1_CONTACT_SENSORS=1 BASELINE1_STATION_ALIGNMENT=0 BUILD1_PLANNING_POSE_SOURCE=ground_truth
colcon test --packages-select openamrobot_build1_scene --event-handlers console_direct+ \
  --ctest-args -R '^test_grocery_demo_mission$'
python3 src/openamr-upperbody-sw/ros2/src/openamrobot_build1_scene/scripts/grocery_demo_report.py build1_logs
python3 src/openamr-upperbody-sw/ros2/src/openamrobot_build1_scene/scripts/grocery_video.py build1_logs
```

To watch it in the Gazebo window and in RViz: start the scene with the `scene` argument, then run the mission against
it from a second terminal (same workspace sourced, same `ROS_DOMAIN_ID`):

```bash
# terminal 1: the store, Gazebo GUI and RViz (map, costmaps, footprint, planning scene, robot model)
ros2 launch openamrobot_build1_scene build1.launch.py scene:=grocery_scene.yaml profile:=firmware_sil \
  planning_pose_source:=ground_truth headless:=false rviz:=true

# terminal 2: the mission (logs and recordings in BUILD1_LOG_DIR)
export BUILD1_LOG_DIR=$PWD/build1_logs BASELINE1_STATION_ALIGNMENT=0
python3 -c "from openamrobot_build1_scene import rig, grocery_demo; print(rig.run('grocery_demo_mission', grocery_demo.mission, needs_devices=True, nav2=True, moveit=True)[:2])"
```

`scene:=grocery_scene.yaml` is the only change to the Build 1 launch; without it the launch loads `scene.yaml` exactly
as before. The store alone, without the mission: terminal 1 only. `profile:=fast` starts it without devices (no
virtual CAN needed) for a look at the store; the mission needs `firmware_sil`.

The Nav2 map is generated from the scene geometry: every fixed box that the LiDAR plane (0.222 m above the floor) cuts
becomes occupied (shelf posts, plinths, bench legs, walls, the dock backing); boards, products and the tote above it do
not. The launch writes it at start. A copy is committed in `config/maps/grocery_dark_store.{pgm,yaml}`; regenerate it
with `python3 scripts/grocery_map.py` after a change of the scene (the static test compares the two).

## Recordings

`.github/workflows/grocery-demo.yml` (pull request on its own files, or dispatch) uploads `grocery-demo-attemptN`
for 30 days:

| File | Content |
|---|---|
| `grocery_demo.mp4`, `grocery_demo.srt`, `still_*.png` | store camera at 2 Hz of Gazebo time, 960 x 540, played at 10 fps (5x), phase and verdict burned in; a camera path over the store (overview, chase view while driving, close view at the shelf and the bench). A visual record, not evidence |
| `bag/` | ROS bag: `/joint_states`, `/tf`, `/tf_static`, `/odom`, ground truth of the base, product, tray and payload, `/amcl_pose`, `/cmd_vel*`, `/plan`, Nav2 action status and feedback, MoveIt action status, controller states, gripper action status, `/sil/*` drive, lift, BMS and firmware states, lift permit and command, contacts, footprint, mission events (`/grocery_demo/events`, `/grocery_demo/phase`), run info and real-time factor |
| `record_candump_vcan_*.log` | `candump -L` of each virtual CAN bus |
| `grocery_states.json` | controller lists of both controller managers and the drive, lift, BMS and firmware states at the end of every phase |
| `grocery_contacts.json` | contact events per phase, sensor and touching collision (simulation forces, not measurements) |
| `grocery_demo_mission.json` | every check with its limit and phase, events, the synchronized trace (Gazebo time, wall time, base, odometry and AMCL errors, lift, finger, product height, real-time factor) |
| `grocery_demo_report.json` | profile, `pose_source`, scene, seed, run information, phase verdicts and Gazebo times |

The job log prints the phase table, an evidence summary and the trace summary per phase.

## Limitations

- Simulation only: Gazebo DART physics, emulated devices with assumed behaviour (see `docs/build1_scene.md`), estimated
  product, tray, tote, friction and store dimensions (every number in `grocery_scene.yaml` says sourced, derived or
  estimated). Not hardware, physical acceptance, perception or safety evidence.
- sim-oracle profile: the planning scene and the grasp target come from the simulator's ground truth.
- With the product held, the arm reaches 0.79 m ahead of the base and the Nav2 footprint follows it: the robot turns
  0.60 m before the packing station and drives straight in with drive_on_heading (a turn at the station sweeps the
  bench legs, and the station tree's final stage turns in place to correct a lateral offset of a few cm).
- The product is not in the MoveIt planning scene while it is carried and placed (it is not attached to the hand
  either); the arm does not move while the base drives, and the place lowers the hand straight down.
- One product, one order tote, one run. The packing station's 0.10 m gap is chosen so that the arm does not move
  between the stations; it is not from Use_Case_1.
- The work shelf keeps Baseline 1's open space under level 0; the other shelf units have plinths.
- Gazebo has no seed: repeated runs differ; a single pass is not repeatability.

## Evidence

Results are in the pull request that added this page (openamr-upperbody-sw, draft into `integration/baseline1`), with
the run links, the phase table and the artifact list.
