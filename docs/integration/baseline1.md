# Integrated Digital Twin Baseline 1: upperbody-sw integration branch

Branch `integration/baseline1`, created 17 Nov 2026 from `main` at `a3a93c6`. Every PR was merged with
`git merge --no-ff` (no rebase, no force-push) at the head SHA verified on GitHub immediately before the merge.
This branch is never merged into `main`: each PR keeps its own review. Simulation evidence only: never hardware,
physical acceptance or safety evidence.

| Order | PR | Branch | Head SHA | Merge commit | Conflicts | Gate status |
|---|---|---|---|---|---|---|
| 1 | #70 | claude/sweet-albattani-km92f2 | `77bbf18` | `537f992` | none | subsystem scenarios: build1-scene-tests on this branch |
| 2 | #75 | fix/build1-lift-and-base (contains #70) | `71efffe` | `cccb00f` | none | as above |
| 3 | #73 | fix/build1-arm-path (contains #70) | `62c38ea` | `2a33336` | six files, below | as above |

Platform input of the acceptance run: openamr-platform-sw `integration/baseline1` at `d36b33a` (main `69847ab` with
#152, #146, #149, #151 and #158; record in that repository's `docs/integration/baseline1.md`), plus `openamrobot_sil`
from openamr-platform-sw #145 `8c5bd52` as in `config/pins.yaml`. `config/pins.yaml` itself is unchanged.

## Conflicts of #73 after #75 and their resolutions

#75 drives the lift by effort without devices; #73 adds the arm gravity feed-forward on the real driver's effort
interface. Both are kept.

- `single_writer.py`: #73's `expected_claims` loop is kept, and its base kind now comes from #75's
  `claimed_interface` (`claimed_by_group`), so `lift_joint/effort` without devices and the arm effort claim with
  devices are both expected.
- `test/test_command_map.py`: #73's `claimed()` helper; both new tests kept. Both pass locally.
- `config/command_map.yaml`: one comment naming the per-kind claims and the lift effort claim.
- `scenarios.py`: the two import lines joined (`arm_frames` from #73, `LIFT_STOP_TOLERANCE_M` from #75).
- `README.md`: lift row from #75, arm row from #73; known issues: gripper and arm text from #73, lift in DART from #75.
- `CHANGELOG.md`: both Unreleased entries kept, #75 first.

## Canonical acceptance: `.github/workflows/baseline1-acceptance.yml`

One launch of the whole robot (`firmware_sil`: every device on virtual CAN, Nav2, MoveIt 2, Gazebo on simulation
time), one mission in eight phases (`openamrobot_build1_scene/baseline1.py`), checks tagged by phase and failures
classified (`openamrobot_build1_scene/baseline1_phases.py`), table printed by `scripts/baseline1_report.py`.

| Phase | Expected motion | Reused checks |
|---|---|---|
| spawn | none, base at `stations.origin` | `rig.wait_ready`, `rig.wait_stopped` |
| nav_to_station | Nav2 `navigate_to_pose` to `stations.work_shelf` | Nav2 goal tolerances of platform `nav2_params.yaml` |
| stop_authorize | base stops; lift refuses without permit; permit becomes fresh | scn02 `_lift_refused` |
| lift_to_target | L1000 to L1175 and back to L1000 (grasp height of level 0) | `rig.lift_move`, scn02 windows |
| arm_approach, grasp, lift_object | park, open, approach, reach, close, lift 0.08 m, hold, release | scn04 `grasp`, unchanged |
| retreat | both arms to park, Nav2 backup 0.30 m | scn03 return move, `rig.wait_stopped` |

Classes: ENV/INFRA, BUILD, STARTUP, CONTROL, PLANNING, CONTACT/PHYSICS, TIMEOUT, and UNCLASSIFIED when no rule
matches. Baseline 1 is complete only when all eight phases pass in one run; partial passes and subsystem scenarios
are reported apart. Deterministic start: `BASELINE1_SEED` seeds Python random and with it the ROS domain,
`PYTHONHASHSEED` is fixed, every node runs on simulation time. Gazebo gets no seed: the launch has no seed argument,
and adding one is a launch contract change.

## Strict acceptance sequence (19 Nov 2026, workflow ready, sequence not started)

One dispatch with `sequence: strict` (the default) runs two missions one after the other (matrix legs 1 and 2,
`max-parallel: 1`, a fresh runner each) on the same pinned commits. Each leg writes `declaration.json`: the commit of
every checkout, the middleware rclpy reports at runtime, `CYCLONEDDS_URI`, profile, station approach and alignment,
seed, build and environment outcome. The `verdict` job runs `test/test_baseline1_strict.py`, then
`scripts/baseline1_strict_verdict.py`, and is PASS only when both legs are 8 of 8, the commits of both legs are
identical, both ran `rmw_cyclonedds_cpp` with `MaxAutoParticipantIndex` 120, both declared station alignment 1 and
sent the goal through `/station_alignment/navigate_to_station`, and both declared sim-oracle
(`planning_pose_source: ground_truth` in the run identity). A pull_request run, or `sequence: single`, is one
validation leg and is reported NOT STRICT. Artifacts per leg and the verdict are kept 90 days.

Gates added 20 Nov 2026 (Alex's acceptance rules; values in `config/baseline1_gates.json`, Proposed;
`openamrobot_build1_scene/baseline1_gates.py`). Each leg must also show:
- both legs from the one dispatch (same run id and attempt);
- mission result PASS and no failed check outside the eight phases (the component liveness checks of `rig.run`
  sit there);
- the drive, battery, lift and arm work counters (`live_*` in every trace sample) advancing through the whole run,
  no gap over 10 s of Gazebo time;
- the Stop checks of `stop_authorize` all run and pass (base settle, speed, yaw rate, no lift motion without permit,
  command refused, permit fresh), and at least one `cancel_*` check run and passed (the cancellation step below);
- contact sensors on (`BUILD1_CONTACT_SENSORS=1`), messages received, and every contact event (per phase, contact
  window and pair) in the allowed list with its reason; any other contact fails the leg. One entry has a window:
  `openarm_left_ee_base_link` or `openarm_left_ee_link1` against `tote::front_collision`, in `arm_approach`, inside the
  window `grasp_reach_last_move` (from the mark `grasp_reach_last_move_start` of the last reach move, #98, through
  `grasp_reach_end` after the 1 s settle), at most 6 N: the residual end-point contact of the base pitch, the software
  lead's exact case on #98 (20 Nov 2026). Without the marks there is no window and the entry accounts for nothing;
- complete evidence: bag metadata, a non-empty candump of each of the five virtual buses, Gazebo start and end of
  every phase, run_info, full pinned hashes of all six checkouts, the declared seed equal to the mission's, settings.

Cancellation step (`baseline1_cancel.py`, values in `config/baseline1_mission_options.json`; position decided by Alex
20 Nov 2026, goal and pass rule awaiting Mohamed and Alex). On only with the declared mission option
`BASELINE1_CANCEL_CHECK=1`, which the acceptance workflow sets and writes to each leg's `declaration.json`
(`cancel_check`; the evidence gate requires it). In the test harness, not in a stop gate. Position `end_of_mission`:
the last step of the retreat phase, after the turn-and-forward retreat, so the odometry guard at the station
(`nav_to_station`) is evaluated before it and no turn follows the cancel (at the start of `nav_to_station` the
following station turn failed the odometry guard, runs 44735544898 and 44736351073). A 0.5 m NavigateToPose goal
straight ahead of the base on open floor (the mission does not need it) on `/navigate_to_pose`, cancelled through the
action's cancel service once ground truth has moved 0.15 m. Checks: goal accepted; cancelled mid-way (0.08 to 0.40 m)
with the base moving (at least 0.02 m/s); cancel request accepted; terminal status CANCELED (5), whatever the stack
returns is recorded; stopped within 4 s and 0.10 m (the scn07 time bound); then over 3 s of Gazebo time ground truth
and odometry under 0.01 m and 0.02 rad and every `/cmd_vel` message, and the last one before the window, zero
(at most 0.001); the last nonzero `/cmd_vel` after the cancel within 4 s. CANCELED is the only pass (ABORTED or
SUCCEEDED fail). On the integration branch `/navigate_to_pose` is bt_navigator itself; platform-sw #90 puts a proxy on
that name, whose cancels the #90 review found ending ABORTED, so the status check is the one to watch when #90 lands.

Settings declared in every leg:
- Middleware: CycloneDDS for initial acceptance (Alex, 19 Nov 2026), with the participant index cap raised to 120 as in
  the DDS benchmark (#89). The robot's middleware beyond acceptance stays the owners' decision.
- Station alignment: openamr-platform-sw #167 on (simulation-only base camera), `station_alignment: 0` is the revert run.
- Profile: sim-oracle (object and planning poses from simulator ground truth); not a perception result.
- Retreat: `forward` (Alex, 20 Nov 2026): Nav2 spin by pi in place, then a 0.30 m forward NavigateToPose with #164's
  station tree, no reverse motion; `BASELINE1_RETREAT: forward` in the workflow, `retreat` in each leg's
  `declaration.json` (the evidence gate requires it). Platform pin with the no-reverse Nav2 configuration:
  openamr-platform-sw #178 (`nav/baseline1-no-reverse-on-175`, 0aa8e14 on 9af88e7), candidate until merged.

Retreat options (`station_departure.py`, values in `config/baseline1_mission_options.json`, `BASELINE1_RETREAT`,
default `forward` since 20 Nov 2026, set by the workflow). Before that the retreat was `retreat()` in `baseline1.py`: after both arms park, a
`nav2_msgs/BackUp` goal on `/backup` (behavior_server's BackUp plugin), 0.30 m at 0.10 m/s, 60 s allowance, through
`cmd_vel_nav`, velocity_smoother (min_velocity[0] -0.20 lets it through) and collision_monitor. With platform-sw #90's
behaviour set (spin and wait only, min_velocity[0] 0; openamr-platform-sw `nav/baseline1-no-reverse` 0aa8e14 for this
branch) `/backup` has no server: `send_goal` waits 120 s, the goal is not accepted, the retreat phase fails (STARTUP).

| Option | Motion | Nav2 today (fb25be1) | #90 behaviour set | Trade-off |
|---|---|---|---|---|
| `backup` | 0.30 m straight back, Nav2 BackUp | allowed | not possible (no server, smoother clamps reverse) | keeps a general, directly callable reverse action in Nav2 |
| `departure` | 0.30 m straight back at 0.10 m/s, harness publisher on `cmd_vel_smoothed`, retreat phase only, 10 s cap | refused: 5 reverse paths enabled | allowed, the only reverse of the run | below the smoother; on #90's chain also below the stop gate, so it needs a contract change request there (a mux input and a departure-only smoother exception) |
| `forward` | Nav2 spin by pi in place, then a 0.30 m forward NavigateToPose with #164's station tree | allowed | allowed, through the normal chain | half turn next to the shelf: 0.141 m measured clearance (#93, closed shelf, base 0.088 m past the station, arms in park), geometry only, carry pose not measured |

Checks per option: `retreat_<option>_allowed` (preconditions), then for `departure` the peak command, time, distance
along the heading (-0.35 to -0.25 m), heading change and peak reverse speed from ground truth; for `forward` the turn
status and heading change (170 to 180 deg), drift in the turn (0.05 m), forward status, distance from the station
(0.25 to 0.35 m) and peak reverse speed (at most 0.02 m/s). `test_station_departure.py` (12 tests) runs every option
against both configurations.

Pins: openamr-platform-sw `960e2e5` (#178, candidate until merged): integration branch `fb25be1` with #169 merged
(`9af88e7`, #175) and the no-reverse Nav2 configuration (0aa8e14 cherry-picked) (#164 at `09f9a85`, BackUp removed from its trees; #167 at
`99ec17f`, then `dccd5e0`, test-only change on top), openamrobot-interfaces `50ed777` (StopState, imported by #167's
node from 99ec17f on).

## Gate status (17 Nov 2026, branch head `80c92e1`, draft PR #77)

Baseline 1: NOT COMPLETE. Acceptance run 44076639872 (platform `d36b33a`, `firmware_sil`, DART, 1 ms step,
real-time factor 0.13 to 0.18 on the hosted runner): 3 of 8 phases passed.

| # | Phase | Verdict | Class | Measured |
|---|---|---|---|---|
| 1 | spawn | PASS | - | pose error 0.0001 m, odometry against ground truth 0.0001 m |
| 2 | nav_to_station | FAIL | CONTROL | Nav2 SUCCEEDED, but the base stopped 0.340 m short of the station (Nav2 xy goal tolerance 0.35 m) after 1.751 m of 2.085 m; odometry minus ground truth 0.110 m (limit 0.10) |
| 3 | stop_authorize | PASS (partial) | - | base speed 0; command without permit refused, no lift motion; permit fresh |
| 4 | lift_to_target | BLOCKED | ENV/INFRA | CTL-017 model fault bits 2 at real-time factor 0.17 (its Hall rate rule needs about 0.55); lift travel 0 |
| 5 | arm_approach | FAIL | PLANNING | park and gripper open passed; `grasp_approach` MoveIt FAILURE (99999): OMPL "Unable to sample any valid states for goal tree", the approach pose is out of reach from where the base stopped |
| 6 | grasp | NOT RUN | - | grasp() returned after the failed approach |
| 7 | lift_object | NOT RUN | - | as above |
| 8 | retreat | PASS (partial) | - | both arms to park by MoveIt (servo torque 0.19 N m); Nav2 backup 0.302 m |

Subsystem scenarios on the same head (build1-scene-tests run 44076639791, reported apart, not Baseline 1):
static and vcan 12 of 12 (with test_baseline1_phases, 25 tests); profiles fast and fine 6 of 6 (scn10 to scn15 PASS);
fast lift stops and base yaw 2 of 2; faults: scn06 and scn08 PASS, scn05 BLOCKED (CTL-017 at real-time factor 0.52,
ENV/INFRA); firmware_sil arms: scn09 PASS, scn03 BLOCKED (CTL-017 at 0.45, ENV/INFRA), scn04 FAIL from the work shelf
spawn (grasp_lift servo torque 0.90 N m of 0.5, CONTROL; item height gain 0.014 m of 0.04, CONTACT/PHYSICS);
firmware_sil drive, lift, cancel: not run, setup-ros could not fetch the ROS apt source (HTTP 404 after an
unauthenticated GitHub API call, ENV/INFRA). OpenArm emulator tests run 44076639712: same setup-ros failure,
ENV/INFRA. Model tests run 44076639604 passed.

What blocks a full pass, in mission order:
1. Station accuracy: Nav2's 0.35 m goal tolerance lets the base stop far enough from the shelf that the arm cannot
   reach the item; the odometry error (0.110 m over 1.75 m) adds to it. #145's `sil.yaml` wheel radius 0.100 against
   the integrated description's 0.098 (#146) is one known contributor.
2. Lift on hosted runners: the CTL-017 model's Hall rate rule runs on the host clock and faults below a real-time
   factor of about 0.55; the whole-robot run reaches 0.13 to 0.18.
3. The grasp itself in `firmware_sil` (scn04): servo torque left at the lift end point and the item not lifted.
4. Platform `openamrobot_sil` is #145, not the integration branch: #141 to #145 are not reconciled with the clock
   contract (#151, #158), so the clock contract is not in the whole-robot run.

Heads that moved after these merges (pinned here at the verified SHA, not re-merged): #73 is now `4b638ee`.

## Acceptance profiles (Proposed, 18 Nov 2026; contract review by the software lead)

Two profiles. A run states its profile in its run information and log, and a result counts only for its own profile.

| Profile | Planning scene objects | Station positioning | Counts as |
|---|---|---|---|
| `sim-oracle` (simulation-oracle integration) | simulator ground truth, declared as `pose_source: ground_truth` | station tree of platform-sw #164 | integration of the ROS 2 stack, firmware-SIL path, controllers and contact physics. Not proof of autonomous perception |
| `perception` (perception-integrated, future) | the robot's own estimate, never ground truth | station tree plus station-relative alignment once adopted (platform-sw #167, extension today) | autonomous perception and positioning in simulation |

Mechanism: `baseline1-acceptance.yml` input `profile` (`sim-oracle` default for this simulation acceptance workflow, `perception`) sets `BUILD1_PLANNING_POSE_SOURCE` (`ground_truth` or `map`), which the launch passes as `planning_pose_source` to the planning scene loader (#73 `1a84066`). The launch file and loader default to `map`. The profile and pose source are printed in the log and the job summary, and the run info and every scenario JSON record `planning_pose_source`. In the `perception` profile the grasp target still comes from ground truth today; a production path takes it from the robot's estimate or perception.

Rules:
- Ground truth is never a default of the launch or the loader. A run uses it only through the `sim-oracle` profile and declares `pose_source: ground_truth`.
- A `sim-oracle` pass is never reported as a perception result.
- The acceptance checks against ground truth (station position 0.04 m, yaw 0.08 rad, item rise 0.04 m) are measurements of the outcome in both profiles, not inputs to the robot, and stay unchanged.
- Partial trajectories fail; no failure is suppressed.

## Results, 18 Nov 2026

Pinned: this branch `6921a29`, openamr-platform-sw integration branch `fe28d19`, `station_alignment` 0. These runs used ground-truth planning-scene placement (`pose_source: ground_truth`, from #73 `b096c7d`), so they belong to the `sim-oracle` profile.

| Run | Phases | Station error | Yaw | Item rise | Result |
|---|---|---|---|---|---|
| [44194529196](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44194529196) | 8/8 | 0.0317 m | 0.021 rad | 0.0797 m | PASS |
| [44194540813](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44194540813) | 7/8 | 0.0447 m | 0.049 rad | 0.0792 m | FAIL station position |
| [44194552457](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44194552457) | 8/8 | 0.0260 m | 0.069 rad | 0.0802 m | PASS |

Artifacts (bag, candump per bus, video, firmware and controller states, phase JSON) are kept until 17 and 18 Dec 2026.

Status: two 8/8 passes on identical commits, not two consecutive passes (the runs ran in parallel and the failed run was dispatched between them). Baseline 1 is not complete.

Extension runs on `aadde12` (`station_alignment` 1, platform-sw #167, simulation-only base camera): [44196268806](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44196268806) 5/8 (alignment ended 0.061 m, Nav2 status 6; reach Cartesian fraction 0.635), [44196280699](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44196280699) Nav2 aborted the first goal (status 6, base did not move), [44196292546](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44196292546) 7/8 (position 0.0046 m, yaw 0.167 rad against 0.08, Nav2 status 6).

Limitations:
- Parking variability is not yet explained; station-tree-only position errors range from 0.026 to 0.080 m across runs.
- The `sim-oracle` planning scene uses information the real robot does not have.
- Grip force, finger friction and item mass are estimates; the gripper is a position servo, so grip torque depends on the finger contact angle. The palm touches the item during the reach (0.4 to 2.6 N).
- Nothing here is bench, physical robot, perception or safety evidence.

Next: a verified parking correction, then the strict sequence: two acceptance runs one after the other on one pinned commit, each in a declared profile.

## #167 verification runs, 18 Nov 2026 (extension, not Baseline 1)

Pinned: this branch `4a67ee7`, openamr-platform-sw integration branch `e75e16e` (#167 at `3e6e203`), profile `sim-oracle` (`pose_source: ground_truth`). Criterion from the parking analysis (#85): marker detected in all 3 runs and each station error at most 0.030 m. Result: not met, 1 of 3.

| Run | Alignment | Phases | Station error | Outcome |
|---|---|---|---|---|
| [44247712314](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44247712314) | on | 8/8 | 0.0035 m | aligned |
| [44247720886](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44247720886) | on | 6/8 | 0.0754 m | measured, then aborted: odom to base_footprint transform not found (unconnected TF trees); reach fraction 0.587 |
| [44247729473](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44247729473) | on | 7/8 | 0.0557 m | marker not seen, 0 poses received |
| [44247710475](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44247710475) | off | 6/8 | 0.0654 m | station tree only; reach fraction 0.603 |

Baseline 1 status unchanged: not complete. The strict consecutive sequence waits for a correction that passes this check.

## #167 verification runs, 18 Nov 2026, second check (extension)

Pinned: this branch `d63e9ed`, openamr-platform-sw integration branch `931e631` (#167 at `c4666a1`), profile `sim-oracle` (`pose_source: ground_truth`, `object_pose_source: ground_truth`, perception stand-in). Criterion: marker detected in all 3 runs, each station error at most 0.030 m. Result: met, 3 of 3, each run 8/8.

| Run | Alignment | Phases | Station error | Yaw |
|---|---|---|---|---|
| [44309791416](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44309791416) | on | 8/8 | 0.0028 m | 0.031 rad |
| [44309804809](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44309804809) | on | 8/8 | 0.0217 m | 0.015 rad |
| [44309817813](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44309817813) | on | 8/8 | 0.0026 m | 0.038 rad |
| [44309784398](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44309784398) | off | 7/8 | 0.0639 m | 0.009 rad |

These are three passes in parallel, not a strict consecutive sequence, and the alignment is not adopted into Baseline 1. They used #167 before the review fixes (aa8fd11). Simulation-oracle evidence only, not perception.

## Diagnostic runs, 20 Nov 2026 (cancellation check, retreat options; diagnostic, not acceptance)

Single legs on `diag/` scratch branches, run name DIAGNOSTIC. Profile sim-oracle, CycloneDDS, alignment on.

| Run | Upperbody | Platform | Retreat | Phases | Cancel status | Stop after cancel | Hold (truth, odom, `/cmd_vel` peak) | Failed checks |
|---|---|---|---|---|---|---|---|---|
| [44735544898](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44735544898) | `d69415c` | `fb25be1` | backup | 7/8 | 5 CANCELED, cancelled at 0.152 m, 0.195 m/s | 0.111 s, 0.0137 m; last nonzero `/cmd_vel` 0.119 s after the cancel | 0.0 m, 0.0 m, 0.0 over 3 s (30 messages) | `nav_odometry_minus_ground_truth` 0.243 m (limit 0.10) |
| [44736351073](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44736351073) | `c0af973` | `0aa8e14` (#90's behaviour set) | forward | 6/8 | 5 CANCELED, cancelled at 0.151 m, 0.185 m/s | 0.135 s, 0.0207 m; last nonzero 0.061 s | 0.0 m, 0.0 m, 0.0 over 3 s (46 messages) | `nav_odometry_minus_ground_truth` 0.236 m; `retreat_peak_reverse_speed` 0.236 m/s (a fault of the measure, fixed below) |

- Cancellation: all 13 `cancel_*` checks passed in both runs. Through `/navigate_to_pose` (bt_navigator, no #90 proxy)
  a public cancel ends CANCELED.
- Odometry after the step: in both runs the odometry was 0.24 m off ground truth at the station (yaw 0.12 rad),
  against 0.062 and 0.075 m (yaw 0.031 and 0.033 rad) in runs 44309791416 and 44309804809 without the step. The
  heading error appears in the 90 deg turn in place that follows the cancelled straight move, as in the turn-slip
  experiment (`diag/spin-test`: a pi/2 turn after a straight under-rotates by 10.3 %). Ground truth at the station was
  fine (0.013 m in the second run). Until owners decide, the step fails the leg through the odometry guard.
- Forward retreat on #90's behaviour set: spin 178.1 deg (status 4), drift 0.014 m, forward goal status 4,
  0.336 m from the station, base speed at most 0.082 m/s, 54 s of Gazebo time (the backup takes 11 s).
  `reverse_speed()` divided pose jitter by sample spacings of a few milliseconds; it now measures over at least
  0.25 s (test added). The no-reverse configuration (`0aa8e14`) also passed the station approach with alignment
  (0.013 m, 0.003 rad).

