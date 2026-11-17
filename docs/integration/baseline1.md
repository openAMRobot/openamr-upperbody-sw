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
