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

## Gate status

Filled in from the first acceptance run on the draft PR of this branch; see the PR body for the run link and the
phase classification.
