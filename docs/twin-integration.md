# Full robot twin — Proposed integration, 1 November 2026

This candidate descends from PR #22 (`b45376b00fdfc4e11472818b4acb756de0f795cf`), which already contains the complete PR #14 feature history and six follow-up commits. Its combined model, controller/MoveIt integration and tests are retained. The platform is consumed by reference at `config/upstream_pins.yaml`; the OpenArm pins are unchanged.

After building and sourcing the workspace, launch explicitly in simulation:

```bash
ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=openamrobot_twin_20261101 \
ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=gazebo \
  column:=dold_v4_350 base_plate:=bp000 head_pitch:=hp30 base_tilt:=bt10 \
  physics_engine:=gz-physics-bullet-featherstone-plugin rviz:=true headless:=false
```

Use an isolated container with no robot devices or robot network. Ctrl-C stops this launch. The integration workspace also supplies tracked process-group start/stop helpers and records their exact commands, container configuration and evidence. The historical workspace is never rebuilt or modified.

Bullet Featherstone is the candidate engine for the complete robot. DART did not close either mimic gripper in the reproduced baseline. Valid platform collision surfaces, explicit friction on the ideal floor and 0.125 ms physics steps support base and arm/gripper motion on one engine. Coefficients and rigid contacts are approximations, not measured materials. Gazebo 8.11.0 publishes base joint states on every physics iteration; later Harmonic rate options are not supported by this installed plugin. The actual engine/rendering versions belong in each run's evidence.

Spawn height is 3 mm instead of 150 mm: the approved axle height and tyre radius put the drive contact 2 mm below base_footprint, leaving 1 mm initial clearance. The CAD-contact candidate settles with sub-micrometre tyre penetration and rear castors about 4.2 mm above the ground. All six loaded contacts are therefore NOT verified; the axle/mount/shim configuration needs mechanical reconciliation. The 0.400 m track and existing joint origins remain unchanged.

D8 equipment is sourced through the existing datums and checked against the roadmap model by `tools/check_roadmap_equipment.py --model <roadmap/model/model.js>`. Chest tablet/light, camera housing, static-column second E-stop geometry, lift stop sensors, brackets, rod and flag are included. Fixed/moving attachment ownership is tested. The stop bracket has a sensor aperture instead of an intersecting solid slab. New fixed frames and the deterministic collision matrix are Proposed under [contract #23](https://github.com/openAMRobot/openamr-upperbody-sw/issues/23). The inherited generator derives the matrix from adjacency, clear fixed transforms and upstream declarations; sampling does not authorize extra collision exclusions.

Three catalogue masses are counted once on the moving assembly: Getac 0.590 kg, head camera 0.0629 kg and Falcon light 0.110 kg. Uniform-box inertias remain approximations. The 11.7 kg lift catalogue mass remains on the fixed column because the moving/fixed split is unknown. Base inertials remain legacy nominal values and omit documented equipment; the assembled model mass is not an as-built mass.

Two source conflicts remain visible rather than silently changing D8: the roadmap light is 17 mm high versus 17.5 mm in the FLDL-i150x15 supplier drawing; the roadmap bracket envelope is 40 mm/4 mm sheet versus SICK BEF-WN-M12's 30 mm/1.5 mm sheet. Positions and unknown mount geometry remain provisional pending the mechanical review.

`config/simulation_cameras.yaml` owns the Proposed ideal head/wrist profile under [contract #24](https://github.com/openAMRobot/openamr-upperbody-sw/issues/24): 640x480, 90-degree horizontal field of view, 10 Hz, 0.02–10 m clip. Producers and bridge consume this file. Images and CameraInfo use the existing optical frames, with Gazebo timestamps. These are monocular ideal viewpoints, not delivered wrist calibration or ZED stereo fidelity. Base RGB-D stays in platform-sw. Actual image/occlusion checks cover all four views. The compatible legacy base camera remains available separately.

The lift has no supplied dynamic actuator in this arm-only controller setup. Static URDF height configurations, flag alignment and lift-following geometry tests do not prove lift actuation. No control authority, joint limit, configuration ID or safety logic is changed.

Validation must run on the final clean source combination: installed A6/A7 entry points; platform package checks; world-truth base motion and stops; actual MoveIt arm/gripper execution and mimic motion; collision queries, sensor/TF audits and repeated shutdown. Preserve failures: the original A7 run reproduced MoveIt SIGSEGV during shutdown, despite the inherited harness exiting zero. No new shutdown allowance is added. Delivery remains draft pending these results and owner adoption; physical/as-built and safety acceptance are not claimed.

Final verification follow-up: the ACM has 320 justified pairs (20 upstream, 35 derived adjacent, 265 constant-transform). Tests now assert these generated values and retain the original 26 mm castor-sphere/chassis regression alongside the current CAD-derived 17.5 mm sphere checks. The historical failure fixture still detects four permanent collisions; no collision allowance was added to make a test pass.

On the installed Gazebo 8.11 / gz-physics 7.6 stack, M02 completed all twelve planned arm poses but failed its unchanged 1e-5 rad joint-limit diagnostic gate during startup (maximum 0.000152871 rad in that run). A 2 mm zero-clearance spawn and a 31.25 microsecond step were separately tested and also failed the gate; the latter reached 0.0000585582 rad. Neither experiment is retained. The candidate remains at 3 mm spawn and 0.125 ms steps. Unclaimed joints already receive gz_ros2_control's default hold_joints zero-velocity command; the Bullet solver uses 50 iterations internally and this version exposes no SDF iteration setting. No initial pose bias, joint-limit change, diagnostic suppression, engine patch or new shutdown allowance is introduced. These are unresolved numerical/reliability findings, not passed acceptance tests.

The task runtime separately builds two ABI-compatible components from exact MoveIt2.12.4: the attributed upstream PR3828 TEM destruction-order fix and a local move_group capability-loader lifetime guard. The guard is declared before the main node, so reverse destruction releases that node before plugin code unloads. The unmodified runtime and upstream-only fix both fail the strict teardown regression; the complete fix passes three consecutive mock launches. Full-stack reruns must identify this additional source/prefix, and the change remains Proposed for owner review. No shutdown allowance is relaxed.

A further stock-version Bullet solver experiment explicitly applied200iterations (constructor value and mapped plugin verified). Startup still reached0.000152593rad past the limit. The experiment is not retained: the delivery uses stock gz-physics7.6,50iterations and0.125ms steps. The initial SDF-only trial was ineffective because Gazebo8.11 constructs the engine world without forwarding its physics settings. Diagnostic patches/logs remain in the integration workspace; they are not a second delivery engine.
