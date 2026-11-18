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

## Station alignment (prepared, off)

openamr-platform-sw #167 (station marker alignment) is Proposed and not adopted into Baseline 1, so it is off here. It is
ready behind one switch, as in Baseline 1: `GROCERY_STATION_ALIGNMENT=1` (workflow env) launches with
`station_alignment:=true`, which places the marker plate on the work shelf, starts the alignment node and sends the
work-shelf goal through `/station_alignment/navigate_to_station`. The packing station is not covered: the node's
standoff (0.713 m) is the shelf parking gap, and the packing gap (0.10 m) needs a second marker and node configuration
that #167 does not have. The packing station's last 0.60 m use `drive_on_heading` instead.

## Pinned inputs

| Input | Repository | SHA |
|---|---|---|
| Base of this demonstrator | openAMRobot/openamr-upperbody-sw `integration/baseline1` | branched at `8f8df24`, merged up to `024ccb8` |
| Platform (description, Nav2 and #164 station tree, perception, docking, `openamrobot_sil` with #162 and #166) | openAMRobot/openamr-platform-sw `integration/baseline1` | `931e631db661b16d1057dfd1ab1d13ea606482e2` (the pin of baseline1-acceptance.yml since d63e9ed) |
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

## Watch it live

The real integrated stack (the same launch, devices on virtual CAN, Nav2, MoveIt 2 and mission as the CI run) with the
Gazebo window, a camera that follows the robot, and RViz, on your own machine. Headless CI is not changed by this.

### Prerequisites

- Ubuntu 24.04 (noble) on x86_64, a desktop session (X11 or Wayland), 8 cores and 16 GB RAM recommended. The CI
  runner (4 cores) reaches a real-time factor of 0.1 to 0.2; a desktop is faster, not real time.
- ROS 2 Jazzy from packages.ros.org: `sudo apt install ros-jazzy-desktop ros-dev-tools` (rviz2 comes with desktop).
- Gazebo Harmonic through the ROS vendor packages, pinned as in CI (gz-sim 8.15.0, sdformat 14.9.0):
  `ros-jazzy-gz-sim-vendor=0.0.13-1noble.20261031.125555`, `ros-jazzy-gz-tools-vendor=0.0.8-1noble.20261025.230735`,
  `ros-jazzy-sdformat-vendor=0.0.14-1noble.20261031.120928` (held with `apt-mark hold`). Check with `gz sim --versions`.
- Rendering: a GPU with OpenGL 3.3 (ogre2) is best. Without one, software rendering works but is slow:
  `export LIBGL_ALWAYS_SOFTWARE=1` before the command; `glxinfo -B` (mesa-utils) shows which renderer is in use.
- Virtual CAN: the `vcan` kernel module (`sudo apt install linux-modules-extra-$(uname -r)`), `iproute2`, `can-utils`,
  and sudo: the launcher runs `sudo modprobe vcan` and `sudo setup_vcan_all.sh` (three classic CAN buses, two CAN FD).
- Sources, pinned: this repository on `demo/grocery-dark-store`, openamr-platform-sw at
  `931e631db661b16d1057dfd1ab1d13ea606482e2`, enactic/openarm_description `14ff67b`, openarm_ros2 `4e837e1`,
  openarm_can `bfd4e52` (config/pins.yaml), in one colcon workspace.
- Build: `cd ~/openamr_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to openamrobot_build1_scene`.
  `--setup` below does all of this.

### The one command

First time (clones the pinned sources into `~/openamr_ws/src`, installs the pinned Gazebo packages and rosdep keys,
builds), from a checkout of this repository:

```bash
bash ros2/src/openamrobot_build1_scene/scripts/run_grocery_demo_gui.sh --setup
```

Every time after that:

```bash
~/openamr_ws/install/openamrobot_build1_scene/lib/openamrobot_build1_scene/run_grocery_demo_gui.sh
```

It checks Ubuntu, ROS and the gz-sim version, builds when the workspace is not built (`--build` forces it), creates the
virtual CAN buses with sudo and runs
`ros2 launch openamrobot_build1_scene grocery_demo_gui.launch.py create_vcan:=false`. Options: `--no-rviz`,
`--no-follow`, `--no-mission` (the store and the robot only), `--video` (also record the store camera as CI does),
`--ws DIR`. The launch file alone takes `follow`, `follow_offset`, `rviz`, `mission`, `mission_delay`, `video`,
`create_vcan` and `log_dir`. The mission starts 20 s after the stack and prints the phase table at the end.

### Moving around

- Gazebo window: left drag rotates (orbit), right drag or scroll zooms, middle drag (or Shift and left drag) pans.
  The camera follows the robot model `openamrobot` (`gz_follow.py`, the GUI's camera tracking). To stop following,
  or to follow by hand: right-click the robot in the Entity Tree (right panel) and choose Follow or Move to; Esc ends it.
- RViz (`rviz/grocery_demo.rviz`, read only: no goal, pose or interactive tools): left drag orbits, scroll zooms, middle
  drag pans; the orbit view follows `base_footprint` (Views panel, Target Frame, set it to `map` to stop following).
  It shows TF, the robot model, map, costmaps, footprint, LiDAR (`/scan_filtered`, `/scan`), odometry, ground truth,
  AMCL pose, Nav2 global and local plans, the item pose, the MoveIt planning scene and planned trajectories.

### Sensor views

- RViz image displays: head camera (`/head_camera/rgbd/image`, on), base depth, left and right wrist cameras and the dock
  camera (off: tick them in the Displays panel); the store camera only with `--video`.
- LiDAR: the two LaserScan displays in RViz; in Gazebo, add the Visualize Lidar plugin (top-right menu) for `/scan`.
- Any topic: `ros2 run rqt_image_view rqt_image_view` or `ros2 topic echo /amcl_pose`.

### Recording and replay

- Every run records into `~/grocery_demo_logs/<date_time>/` (or `log_dir`): a ROS bag (`bag/`, the topics of
  Recordings below, plus `/robot_description`), `record_candump_vcan_*.log` for the five buses, the mission log and
  report JSON, states and contacts.
- More topics, by hand: `ros2 bag record --use-sim-time -o my_bag /tf /tf_static /scan_filtered /head_camera/rgbd/image`.
- Gazebo: the Video Recorder plugin (top-right menu, Video Recorder) records the Gazebo view to MP4; the
  screenshot button saves a PNG.
- Replay a bag with RViz: `ros2 bag play ~/grocery_demo_logs/<date_time>/bag --clock` in one terminal, and in another
  `rviz2 -d ~/openamr_ws/install/openamrobot_build1_scene/share/openamrobot_build1_scene/rviz/grocery_demo.rviz --ros-args -p use_sim_time:=true`.
  The robot model comes from the recorded `/robot_description`; the CAN logs replay into a bus with
  `canplayer -I record_candump_vcan_drive.log`.

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
- Odometry drifts in turns in place (runs 44278919387 and 44299879412, from the 5 s odometry samples): straight
  driving keeps distance within 0.5 % and heading within 0.01 rad per metre, but each turn in place leaves the
  odometry heading about 0.03 rad off the body (per-turn ratio 0.88 to 1.11), mostly in one direction. Over the 6.3 m
  to the packing bench 0.15 rad becomes about 0.9 m of odometry position error. Nav2 drives on AMCL, which stays
  within 0.06 to 0.11 m, so the stations are reached. The wheel and caster contact model is the likely cause
  (Proposed to its owners, not changed here).

## Evidence

Results are in the pull request that added this page (openamr-upperbody-sw, draft into `integration/baseline1`), with
the run links, the phase table and the artifact list.
