# Desktop check of the Digital Twin rc.1, candidate 2
Simulation only. For Ubuntu 24.04 with a real screen, natively or under WSL2 with WSLg. Needs the workspace made by `tools/bootstrap_digital_twin.sh` (default `~/oamr_dt_ws`) and `rc1_desktop_check.sh` from this folder. The script installs nothing and uses no sudo.

## 1. Move the workspace to candidate 2 and rebuild
The manifest still pins the previous pair, so move two repositories by hand:
```bash
cd ~/oamr_dt_ws/src/openamr-platform-sw
git fetch origin 2ac40702deaf3533f9612a1e05d1ea9bb41b996c && git checkout --detach 2ac40702deaf3533f9612a1e05d1ea9bb41b996c
cd ~/oamr_dt_ws/src/openamr-upperbody-sw
git fetch origin 6687f65b8bd03f83ef6c38a7ddaf47eb3bfd18ac && git checkout --detach 6687f65b8bd03f83ef6c38a7ddaf47eb3bfd18ac
cd ~/oamr_dt_ws && source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to openamrobot_build1_scene openamrobot_station_alignment --event-handlers console_cohesion+
```

The build must end without a failed package. If it fails, keep `log/latest_build` and report it; do not delete `build/` or `install/` to make it pass. Then, in every terminal you use:
```bash
source /opt/ros/jazzy/setup.bash
source ~/oamr_dt_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
```

## 2. Run the three modes
One mode at a time, each from an empty evidence folder, for example `mkdir -p ~/c2_evidence && cd ~/c2_evidence`:
```bash
bash /path/to/rc1_desktop_check.sh drive    # profile:=fast, default engine, RViz, scn01_drive
bash /path/to/rc1_desktop_check.sh lift     # no Nav2, MoveIt or sensors, no RViz, scn16_lift_stops
bash /path/to/rc1_desktop_check.sh grasp    # Bullet Featherstone engine, robot at the work shelf, RViz, scn04_grasp
```

Each run checks commits and environment, starts the launch, waits until it is ready, measures the real-time factor, runs the scenario, then waits for Enter. Do the visual checks of section 3 before you press Enter. It then sends one interrupt, times the shutdown and writes `rc1_desktop_<mode>_<time>/report.md`. Every line that starts with `FAIL` goes into your report as it is. `NOT RUN` and `INFO` are not passes.

Do not run `scn03_arms_reach` or any other arm motion in the drive or lift launch: with the default engine it fails (known issue 126). Do not drive in the grasp launch: with the Bullet engine the base does not turn.

## 3. Visual checks only a person can do
Tick a box only when you saw it yourself. If it is not good, write one line saying what you saw.

- [ ] Robot appearance (drive, Gazebo). Good: the robot stands at the origin on a floor with shelf units, a tote, an item and a dock; base, lift column and both arms are complete, with no missing, white or misplaced part.
- [ ] RViz displays (drive). Good: robot model, map, laser scan on shelf posts with no return on the robot's own body, local costmap, footprint octagon, odometry arrow on the robot, planning scene objects in the right place; the Displays panel shows no red error for a display that is switched on.
- [ ] Ground truth display (drive). Tick "Ground truth" in the Displays panel. The sources say its messages carry the frame `world`, which is not in TF, so RViz cannot place them and the display shows an error. Report which you see: that error, or an arrow. Good if an arrow appears: it sits on the robot, on the Odometry arrow, and stays there while the robot drives. An arrow anywhere else is a defect.
- [ ] Navigation path during `scn01_drive` (drive). Good: the "Plan" display draws a path from the robot to a point about 2 m ahead; the robot follows it in RViz and in Gazebo, wheels turning, then turns about 90 degrees on the spot; no jump of the model.
- [ ] Lift during `scn16_lift_stops` (lift, Gazebo). Good: the lift carriage visibly travels the column to several stops and ends at the bottom.
- [ ] Arm and gripper during `scn04_grasp` (grasp, Gazebo). Good: the left gripper opens, the arm reaches the item in the tote, the gripper closes, the hand lifts the item and the item stays in the hand; no arm link passes through the shelf or the tote.
- [ ] RViz in the grasp launch. Nav2 is off there, so the sources predict that the fixed frame `map` does not exist and most displays stay empty. Report what you see; do not judge arm motion from RViz in this launch. The sources name no other fixed frame to select.
- [ ] Usability. Good: both windows open without your help, orbit and zoom answer within about a second, text is readable, and you never had to guess what to do next. Note every point where you hesitated.

## 4. Screenshots and recording
Save them into the report folder of the run. Names: `c2_<mode>_<nn>_<what>.png`.

1. `c2_drive_01_gazebo_start.png`: Gazebo, whole robot and scene, before the scenario.
2. `c2_drive_02_rviz_start.png`: RViz, 3D view and the whole Displays panel.
3. `c2_drive_03_rviz_ground_truth.png`: RViz with "Ground truth" ticked and its status line expanded.
4. `c2_drive_04_rviz_path.png`: RViz while the robot drives, path visible.
5. `c2_grasp_05_gazebo_item_lifted.png`: Gazebo, item held in the left hand.

Recording: `c2_grasp_arm_gripper.mp4`, 30 to 60 s of the Gazebo window from the gripper opening to the lifted item. If you cannot record, say so; do not replace it with more screenshots.

## 5. Controlled Ctrl+C check
The script measures an interrupt sent to the process group. This check is the real key press. In Terminal 1, with the four environment lines:
```bash
ros2 launch openamrobot_build1_scene build1.launch.py profile:=fast rviz:=true
```

Wait until both windows show the robot. Start a stopwatch, press Ctrl+C once, and stop it when the prompt returns. Do not press it a second time for 60 s. Then run `pgrep -af "gz sim"`. Good: the prompt returns within 60 s and the command prints nothing. Report the seconds, any process it lists, and the last 20 lines of the terminal. A non-zero exit of `rviz2` at this point is a known defect (issue 143): report it, it is not a new failure. If something is left, end that one process by its PID, not by a name pattern.

## 6. Relaunch check
Straight after section 5, in the same terminal, run the same launch again. Good: both windows open again, the robot is at the origin, and in a second terminal `ros2 topic info /clock -v` lists one publisher and `ros2 topic echo --once /odom` prints one message. Stop it with one Ctrl+C. The script gives the same evidence between modes: it refuses to start while a `gz sim` process is alive and prints FAIL.

## 7. WSL2 performance: evidence only
The cause of the low frame rate is not established. Collect facts and change nothing.

- Renderer: `report.md` has the `glxinfo -B` renderer line, when `glxinfo` is already installed. `llvmpipe` means software rendering on the processor. A name that starts with `D3D12` followed by a graphics card means the Windows GPU is used. Copy the line as it is.
- `/dev/dxg`, `LIBGL_ALWAYS_SOFTWARE`, `GALLIUM_DRIVER`: copy the values from `report.md`.
- Limits: in Windows, open `%UserProfile%\.wslconfig` and copy the `memory` and `processors` lines of the `[wsl2]` section, or write "no file". Compare with the processor count and memory in `report.md`. Read and report; do not edit the file as part of this check.
- Real-time factor: the number of each mode from `report.md`. The sources give no pass limit for it.
- Frame rate: the script does not measure it, because the sources name no probe for it. If you quote a number, say which window and which tool showed it.

## 8. Report template
```text
Candidate 2 desktop check. Date: ____  Reviewer: ____  Machine: native / WSL2, CPU ____, RAM ____, GPU ____
Commits: platform 2ac40702 PASS/FAIL, upper body 6687f65b PASS/FAIL. Build: PASS/FAIL
Drive: scn01_drive exit __, RESULT ____, real-time factor ____, shutdown ____ s, FAIL lines: ____
Lift: scn16_lift_stops exit __, RESULT ____, real-time factor ____, shutdown ____ s, FAIL lines: ____
Grasp: scn04_grasp exit __, RESULT ____, real-time factor ____, shutdown ____ s, FAIL lines: ____
Visual: appearance __ RViz __ ground truth (error / arrow on robot / arrow elsewhere) path __ lift __ arm and gripper __
Usability notes: ____
Manual Ctrl+C: ____ s to prompt, processes left: ____. Relaunch: PASS/FAIL
WSL2: renderer line ____, /dev/dxg ____, .wslconfig memory ____ processors ____
Not checked, and why: ____   Attached: three report folders, five screenshots, one recording
```
