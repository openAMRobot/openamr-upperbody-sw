# Reports cited by the PR descriptions

(P14's four reports first, then P15's: the five runs that prove the shutdown fix, the A6 runs of the rebased tree, and three A7 runs of the final code.)

Machine-written, copied byte for byte from the author's workspace `evidence/`; they contain local checkout paths. Each describes the commit in its
`git_hash` field, on a clean tree (`dirty: false`). `WARN` is not `PASS`.

| file | what | commit | result |
|---|---|---|---|
| `20261006T164342Z_dev.json` | A6 suite (`verify_a6.sh`), the report PR-1 cites | `993b000` (A6 head) | 20 PASS, 6 WARN, 0 FAIL |
| `20261012T164313Z_dev.json` | A6 suite on the A7 branch (adds H01, H02; V01 now carries the manual-look findings) | `93a2a49` | 22 PASS, 6 WARN, 0 FAIL, 0 NOT_RUN |
| `20261012T170225Z_a7.json` | A7 regression (`run_a7_regression.sh`), the report PR-2 cites | `93a2a49` | 13 PASS, 1 WARN (V01), 0 FAIL, 0 NOT_RUN |
| `20261012T165237Z_a7.json` | the first A7 run on the same commit: **FAILED** M01 (and AUX) through the process guard, `joint_state_clamp.py` exited 1 at shutdown in one of four launches; all poses passed. Kept beside the passing one | `93a2a49` | 11 PASS, 1 WARN, 2 FAIL |

The two A7 reports ran on one commit with nothing changed between them: one of two runs failed, and both are here.

## P15

**Task 0, five consecutive A7 runs on one commit** (`7daa2ab`: the `joint_state_clamp.py` fix, still on the old model and the old base, before the
rebase). Every run is 13 PASS, 1 WARN (V01), 0 FAIL, 0 NOT_RUN; all 20 M01 launches (four mast IDs x five runs) passed 12 of 12 poses and every
process exited as the guard expects.

| file | run |
|---|---|
| `20261013T093322Z_a7.json` | 1 |
| `20261013T094251Z_a7.json` | 2 |
| `20261013T095220Z_a7.json` | 3 |
| `20261013T100148Z_a7.json` | 4 |
| `20261013T101116Z_a7.json` | 5 |

A `gz sim` server was left running after run 4 (started during G02's launch, re-parented away from it) and stopped by hand by pid; no test saw it.

**The rebased tree with option B adopted** (platform-sw `174731c`):

| file | what | commit | result |
|---|---|---|---|
| `20261013T101652Z_dev.json` | A6 suite, the **first** run: **FAILED** H01 (and AUX with it): option B registers `test_upperbody_tf.py` with `ament_add_ros_isolated_pytest_test`, which H01's registration expression did not know, so a test that runs was reported as one that never runs. Fixed in `1168f45` | `0f1ca1e` | 22 PASS, 4 WARN, 2 FAIL |
| `20261013T102027Z_dev.json` | A6 suite after the fix | `1168f45` | 24 PASS, 4 WARN (T08, E01, W02, V01), 0 FAIL, 0 NOT_RUN |
| `20261013T102944Z_a7.json` | A7 regression, final code, run 1 | `dada509` | 13 PASS, 1 WARN (V01), 0 FAIL, 0 NOT_RUN |
| `20261013T103837Z_a7.json` | the same, run 2 | `dada509` | 13 PASS, 1 WARN, 0 FAIL, 0 NOT_RUN |
| `20261013T104713Z_a7.json` | the same, run 3 | `dada509` | 13 PASS, 1 WARN, 0 FAIL, 0 NOT_RUN |

`dada509` is the last commit that changed code, so the three A7 reports describe the code of the final branch; every later commit here is
documentation or an evidence copy. A `gz sim` server was left running after run 3 (again from G02's launch) and stopped by pid: **two of the eight
A7 runs of this step left a simulator behind**, the P12A finding that the process guard does not see (the simulator is the `ruby` wrapper's child, and
`sim.launch.py` asks the server to stop through `/server_control` on shutdown, which evidently does not always work).

**A number that moved on purpose and has no cause established.** In M02 the clamp's activations went from 184 to 277 per run (P14 and the five runs above,
16 to 22 episodes) to 1707 to 1800 (61 to 88 episodes) on the final code, and the largest overshoot it clamped from 5.4e-07 to 9.3e-07 rad to 1.95e-06
and 3.1e-06 rad (`openarm_left_joint4`). Reproduced in the development overlay: 1725, 1796, 1796 and one run of 397. Nearly all of the activations are
`openarm_right_finger_joint1` and `_joint2` at their open limit reading 2.4e-07 rad past it (829 to 865 messages each), which is float noise of the
same kind the clamp exists for; nothing is beyond the epsilon (1e-05), and the poses still reach their goals (12 of 12, 0.001 rad). The hypothesis that
the lift's own prismatic joint, now an unactuated joint in the physics, is the cause was **tested and rejected**: with `lift_joint` made `fixed` in a
copy the activations are 3300 to 3500 (the left fingers join in). So it is the physics solution of the new model arrangement and the new base, and
it is not understood. **The consequence that matters:** the epsilon was set at ten times the largest overshoot seen (9.9e-07); the largest seen is now 3.1e-06,
a margin of about 3. Nobody should widen the epsilon on this (it is ADR-009's number and the clamp must not launder a real excursion): it is a finding.

Class: `static+mock` for `_dev`, `simulation` for `_a7`. No physical evidence. The frozen expansion set is in `../frozen_a6_993b000/`.

## P16

Two evidence passes, three A6 and three A7 runs each. Pass 1 ran on `9bc327c` (the G02-only orphan check), pass 2 on `aa94fc3` (the same check
on G01 and M02 as well). The last commit that changed code is `63ed6d1`: `aa94fc3` differs from it by report copies only, and the commits after
`aa94fc3` add only reports and documents.

| file | what | commit | result |
|---|---|---|---|
| `20261014T121519Z_dev.json`, `...T121726Z_dev.json`, `...T121931Z_dev.json` | A6 suite, pass 1, runs 1 to 3 | `9bc327c` | 24 PASS, 4 WARN (T08, E01, W02, V01), 0 FAIL, 0 NOT_RUN, each |
| `20261014T122605Z_a7.json`, `...T123243Z_a7.json` | A7, pass 1, runs 1 and 2 | `9bc327c` | 13 PASS, 1 WARN (V01), 0 FAIL, 0 NOT_RUN, each |
| `20261014T123917Z_a7.json` | A7, pass 1, run 3: **FAILED** | `9bc327c` | 11 PASS, 1 WARN, 2 FAIL (M02, and AUX with it): one named pose CONTROL_FAILED (11 of 12) |
| `20261014T124435Z_dev.json`, `...T124640Z_dev.json`, `...T124844Z_dev.json` | A6 suite, pass 2, runs 1 to 3 | `aa94fc3` (code of `63ed6d1`) | 24 PASS, 4 WARN, 0 FAIL, 0 NOT_RUN, each |
| `20261014T125516Z_a7.json`, `...T130148Z_a7.json`, `...T130822Z_a7.json` | A7, pass 2, runs 1 to 3 | `aa94fc3` (code of `63ed6d1`) | 13 PASS, 1 WARN (V01), 0 FAIL, 0 NOT_RUN, each |

**The failed run, and its cause.** During pass 1 run 3 a `gz sim` server of **G01's** launch (partition `g01_65237`) was left running after G01's
teardown (G01's own `parameter_bridge` had exited -6, one of the allowed shutdown behaviours). It ran at about three cores for the rest of the run,
and M02, two tests later, sent a pose whose controller never executed (`left_arm` request, final error 1.5 rad). Nothing detected the orphan: the
check of that commit was on G02 only, and G02's own launch was clean. The check was extended to G01 and to M02's launch (`63ed6d1`) and pass 2
was run on that commit. **This is one failed run in six on the code of the step, with a cause that was seen, not inferred; it is not hidden by
pass 2.** Pass 2's three clean runs say the tree passes when no orphan arises; they do not say the orphan cannot arise.

**What moved against P15, and why.** Between `dada509` and `63ed6d1` nothing the simulator or the model sees changed (the commits are tests,
harness, documents and evidence), and the numbers show it: every check and every count is as in P15 except (1) G01 and G02 (and M02's sidecar)
now carry `gz_server_after_shutdown` (new field); (2) per-run message counts (1511 to 1520 in M02, 4025 to 4389 in M01); (3) the M02 clamp counters:

| run | activations | episodes | largest overshoot |
|---|---|---|---|
| P15, three runs of `dada509` | 1800, 1707, 1783 | 88, 81, 61 | 1.9e-06, 3.1e-06, 1.9e-06 rad |
| pass 2, three runs of `aa94fc3` | 1795, 957, 1832 | 37, 39, 52 | 1.3e-06, 8.6e-07, 6.2e-07 rad |

These moved inside the scatter that P16's bisect measured for an **unchanged** tree (right-finger activations 873, 646, 1054 in three runs
of one configuration; `evidence/m02_bisect/RESULTS.md`), so they are not attributed to anything in the diff: there is nothing in the diff that
could move them, and the scatter itself is the thing that is not understood. The margin to the epsilon (1e-5) is still about 3x on the worst
overshoot seen in any run of this step (4.2e-06 rad, in a variant run of the bisect), not 10x. The epsilon was not widened.

**Orphans seen in this step (natural), for the rate.** G02 launch, first run of the session (before the check existed): 1. G02 launch under
the check: 0 of 12 (and 0 of the 6 G02 launches of the two passes). G01's launch: 1 in pass 1 (above). M02's launch: left servers behind only after failed runs
(8 of 12 runs of the first, contaminated bisect attempt; none in the valid ones). The 25% of P15 was not reproduced; the check on G02 was
shown to work through the whole launch-test path only with a deliberately injected fault (`canary_gz_orphan.sh`).


## P17

The code of this step is `013b524` (the last commit that changed code, other than `test_a7_controller_wait.py` and its CMake line, added after
the passes and run alone). The pushed head differs from it by report copies, measurement data and documents. `WARN` is not `PASS`.

| file | what | commit | result |
|---|---|---|---|
| `20261015T113447Z_dev.json`, `...T113759Z_dev.json`, `...T114111Z_dev.json` | A6 suite, the **first** pass, runs 1 to 3: **FAILED**, all three the same way | `a713533` | 23 PASS, 4 WARN, 1 FAIL (AUX), 0 NOT_RUN: `test_evidence_dir.py` rejected the stop helper's report, which used the system temp directory. Fixed in `013b524` (the report is evidence and lives in the evidence directory) |
| `20261015T114559Z_dev.json`, `...T114920Z_dev.json`, `...T115241Z_dev.json` | A6 suite, runs 1 to 3 | `013b524` | 24 PASS, 4 WARN (T08, E01, W02, V01), 0 FAIL, 0 NOT_RUN, each |
| `20261015T120152Z_a7.json` | A7, run 1: **FAILED** | `013b524` | 11 PASS, 1 WARN (V01), 3 FAIL (G01, M07, AUX), 0 NOT_RUN |
| `20261015T121048Z_a7.json`, `...T121946Z_a7.json` | A7, runs 2 and 3 | `013b524` | 12 PASS, 1 WARN (V01), 2 FAIL (M07, and AUX with it), 0 NOT_RUN, each (corrected 19 Oct, P22: this row said 13 PASS; the two reports have 15 checks, 12 of them PASS, found by `tools/check_evidence_backing.py`) |

**M07 FAILs by design, so no A7 run is green.** It names `openarm_right_finger_joint1` (rests at +2.6e-8 rad, past its open limit), and
`openarm_left_finger_joint1`, `openarm_left_joint4` and `openarm_right_joint4`, which rest 2e-4 to 5e-4 and 1.0e-4 rad inside theirs. AUX fails
with it because the bringup `colcon test` step exits 1 (the same shape as P16's "M02, and AUX with it"). Every other check of the A7 report is
as before.

**The failed A7 run, and what it shows.** In run 1, G01's `gz sim` server ignored the SIGINT, ignored the `/server_control` request (which
timed out) and ignored SIGTERM, and was ended by the stop helper's SIGKILL at 9.2 s. The run FAILed with the helper's report in its message and
**no server was left running** (`details.gz_server_after_shutdown` in the report). This is the hang of `launch/sim.launch.py`'s header, seen live
for the first time with the helper in place: **SIGINT does not prevent it**, it was one in 49 launches since (36 in the orphan count below, 12
here, one M07 run), against about one in 13 before. 36 of the 36 launches of the orphan count stopped on the first step, in 1.32 to 1.44 s; of the 12 launches here, the A7 reports carry the
helper's report for G01, G02 and M07 (eight stopped on the first step in 1.32 to 1.44 s, one was forced), and the M02 launches' helper reports
are in their sidecars, of which only the last run's was kept (stopped, 1.43 s). The cause of the hang is inside gz-sim and was not found.

**M02's two numbers** (gate; `details.clamp_totals.gate` in each A7 report): excursions beyond epsilon 0, 0, 0; largest overshoot
1.54e-06, 1.54e-06 and 1.52e-06 rad (epsilon 1e-5, so about 6.5x). Activations and episodes, labelled `diagnostic_only`: 1839 / 89, 1802 / 66,
1819 / 60. M02 waited 0.0 s for its controllers in the run whose sidecar was kept (they were already `active`).

**Orphans seen by the detector after the handler fix: 0 in 12 runs each of G01, G02 and M02** (`../shutdown_p17/orphan_count_36_runs.json`; one
launch each through `colcon test`, interleaved). The natural rate before the fix was about 1 in 13 per launch. **That is not "fixed"**: 12 clean
runs of a one-in-13 event have a 38% chance of occurring with nothing changed (0.92^12), and the live force-stop above shows the underlying
hang still happens. What the fix changes is that a hung server no longer survives its run and no longer fails the next one. In the same 36 runs
G02 FAILED once (`gz_link_poses` parsed two JSON messages as one; fixed in `aa0a455`, afterwards, and passed in the three A7 runs here).

**The measurement behind the design** (`../shutdown_p17/`): `timing_c.jsonl` six launches of the original service request under launch_testing
(one orphan); `timing_d.jsonl` twelve of the same with the first version of the helper, which followed the process tree and called a
reparented server "gone" (d1 is that case: the driver found the server alive 40 s later); `timing_e.jsonl` and `timing_f.jsonl` twelve launches
each with SIGINT sent to the server, all stopped in 1.32 to 1.54 s. The service request in `c` and `d` stopped in 1.64 to 1.79 s when it worked;
in 4 launches of those 18 it did not (two server segfaults in libgz-sim8 during teardown, two servers that never began it).

## P18

**(P19: M07 was restored, W03 is evidence only; this section describes the state of `bb39c83`.)** M07 was withdrawn and W03 and W04 added (`CHANGELOG.md`, P18). The P17 reports above are **unchanged, byte for byte**; this is how to read them now.

- **Every P17 A7 report carries an M07 FAIL (and AUX with it) that no longer exists as a check.** It was withdrawn because its premise was refuted. They are
  kept as recorded, not rewritten.
- **`20261015T120152Z_a7.json`, the P17 run that failed G01.** Recorded as FAIL because the stop helper's force-stop was then a FAIL. Under the P18 rule it
  is a **W04 WARN**: G01's server ignored SIGINT, the service request and SIGTERM, was ended by SIGKILL at 9.2 s, and nothing was left running. Its other
  checks are as recorded; with M07 gone it would read 13 PASS (G01 would be PASS) / V01 and W04 WARN / 0 FAIL. That reading is derived here, not written
  into the file: the sidecars it came from no longer exist and a rebuilt report would be a different document.

Passes on `bb39c83` (the code of this step; the pushed head adds these reports and documents only):

| file | what | commit | result |
|---|---|---|---|
| `20261018T172411Z_dev.json`, `...T172646Z_dev.json`, `...T172922Z_dev.json` | A6 suite, runs 1 to 3 | `bb39c83` | 24 PASS, 4 WARN (T08, E01, W02, V01), 0 FAIL, 0 NOT_RUN, each; the `config_id`s are those of before P17 again |
| `20261018T173643Z_a7.json` | A7, run 1 | `bb39c83` | 13 PASS, 3 WARN (W03, W04, V01), 0 FAIL, 0 NOT_RUN; AUX clear |
| `20261018T174401Z_a7.json`, `...T175119Z_a7.json` | A7, runs 2 and 3 | `bb39c83` | 14 PASS, 2 WARN (W03, V01), 0 FAIL, 0 NOT_RUN, each; AUX clear |

- **W03** lists `openarm_left_joint4`, `openarm_right_joint4`, `openarm_left_finger_joint1` and `openarm_right_finger_joint1` in all three runs (eight
  poses: `home` and `park` for the joint4s, `open` and `closed` for the fingers), and the same four, eight poses, for the old mast body `7daa2ab`
  (`../w03/old_body_7daa2ab.json` and `new_body_HEAD.json`, byte-identical reports).
- **W04 WARN in run 1: the force-stop path fired live for the second time.** M02's server accepted nothing and went nowhere: SIGINT, the service request
  (after 3 s) and SIGTERM did not end it; SIGKILL at 9.02 s did. The M02 run PASSED its own checks and its survivor check found nothing running. G01 and G02
  of the same run stopped in 1.31 and 1.32 s. In runs 2 and 3 all three launches stopped on the first step. Since the helper existed: 2 force-stops in
  about 61 launches (36 of the orphan count, 12 + 12 here, a P17 M07 run, and the 12 of the P17 passes).
- **M02's two numbers**: 0 excursions beyond epsilon in all three runs; largest overshoot 1.42e-06, 2.37e-06 and 1.76e-06 rad (epsilon 1e-5: margins about
  7.1x, 4.2x and 5.7x). Activations and episodes are `diagnostic_only` and gate nothing.

## P19

Alex Reznichenko's `followups/a6-a7-49d2368` merged into this branch (merge `ce76607`; his 14 commits are unchanged); M07 restored as he wrote it (WARN on the
four upstream-limit joints); W03 is evidence files only; W04 stays and now reads M07's launch as well. Every report below is on the tested code `538a32a`
(the merge head plus the RViz evidence; the pushed head adds only these reports and documents), clean tree. `WARN` is not `PASS`.

**platform-sw pinned at `d619012` (#70), as instructed.** Its `imu_link` is `(0.0544, 0, 0.1210)`, yaw -90 degrees.

| file | what | platform-sw | result |
|---|---|---|---|
| `20261018T191032Z_dev.json`, `...T191402Z_dev.json`, `...T191732Z_dev.json` | A6 suite, runs 1 to 3 | `d619012` | 25 PASS, 3 WARN (E01, W02, V01), 0 FAIL, 0 NOT_RUN, each |
| `20261018T192527Z_a7.json`, `...T193323Z_a7.json`, `...T194118Z_a7.json` | A7, runs 1 to 3 | `d619012` | **12 PASS, 2 WARN (M07, V01), 2 FAIL (M03, AUX), 0 NOT_RUN, each** |

**Why A7 is red on `d619012`, three times alike.** #70's chassis box and the four caster-wheel spheres overlap by 2.4 mm, so `base_link` collides with each
`{fl,fr,bl,br}_wheel_link` in every state: the derivation finds 91 pairs against the 95 in the contract (M03: 3 of 6 tests fail), and the two ACM
regeneration tests fail (AUX). It is the cause `docs/pr/PR14_BODY.md` records for `d619012`, and it is not the merge. The committed matrix is correct for the
base it was derived on; nothing was regenerated or disabled.

**Supplementary, not asked for: the same protocol on `dfd44a5`** (platform-sw `main` after #71, the chassis box clears the caster wheels by 2.6 mm; it contains #70):

| file | what | platform-sw | result |
|---|---|---|---|
| `20261018T194538Z_dev.json`, `...T194915Z_dev.json`, `...T195250Z_dev.json` | A6 suite, runs 1 to 3 | `dfd44a5` | 25 PASS, 3 WARN (E01, W02, V01), 0 FAIL, 0 NOT_RUN, each |
| `20261018T200040Z_a7.json` | A7, run 1 | `dfd44a5` | **14 PASS, 2 WARN (M07, V01), 0 FAIL, 0 NOT_RUN** |
| `20261018T200829Z_a7.json`, `...T201608Z_a7.json` | A7, runs 2 and 3: **FAILED** | `dfd44a5` | 12 PASS, 2 WARN (M07, V01), **2 FAIL (G02, AUX)**, 0 NOT_RUN, each |

G02's physics passed in both failed runs (both grippers reached their goal and the mimic joint followed); its server stopped on the first step in 1.4 s and
nothing survived. What failed is the process guard: `parameter_bridge` (`base_joint_states_bridge`) **died with exit code -11 (SIGSEGV) while the launch was
stopping** (`Proc parameter_bridge-4 exited with code -11`). The guard allows only SIGABRT (-6) for that process, which is a known once-in-a-while
shutdown exit; a SIGSEGV is not allowed and **nothing was added to the allow list**. It did not occur in the 3 A7 runs on `d619012`, the 3 of P18, or run 1 on
`dfd44a5`: 2 of 6 runs of this step, both on `dfd44a5`, and the one difference between the two platform-sw pins is the chassis collision box, which cannot reach a
bridge. The cause was not investigated; I do not know that it is new, rare or related to the gz-sim hang. One bridge also exited -6 (allowed) in run 2.

**W04 in this step: 0 force-stops in 24 launches** (6 A7 runs x G01, G02, M02, M07; all `stopped` on the first step, 1.3 to 1.5 s). The prior rate is 2 in about
61. At that rate 24 launches expect about 0.8; zero is the likelier outcome (about 45%), so **this does not show that the spawner-timeout change (Alex's `1637743`)
helped**, only that no force-stop was seen. The survivor check is unchanged and found nothing in any run. No leftover `gz sim` after any run.

**M02's two numbers, 0 excursions in all six runs.** Largest overshoot, `d619012`: 1.59e-6, 1.66e-6, 2.44e-6 rad; `dfd44a5`: 2.23e-6, 1.71e-6, 3.23e-6 rad; epsilon 1e-5
(not widened).

**M07 is WARN in all six runs** on the four joints `openarm_{left,right}_finger_joint1` and `openarm_{left,right}_joint4`, with all expected joints covered.

**RViz screenshots**: `../renders/rviz_L1000_bp000.png` ... `../renders/rviz_L1350_bp050.png`, real RViz2 on the WSLg display (`../renders/rviz/README.md`).

## P20

**platform-sw pinned at `dfd44a5`** (#70 + #71), by the architecture chat's decision: `d619012` fails M03 deterministically for a defect (#70's chassis box overlaps the
casters) that #71 fixes. Tested code `24c66f5` (the pushed head adds only these reports and documents), clean tree.

| file | what | platform-sw | result |
|---|---|---|---|
| `20261018T213907Z_dev.json`, `...T214244Z_dev.json`, `...T214621Z_dev.json` | A6 suite, runs 1 to 3 | `dfd44a5` | 25 PASS, 3 WARN (E01, W02, V01), 0 FAIL, 0 NOT_RUN, each |
| `20261018T215411Z_a7.json`, `...T220156Z_a7.json`, `...T220947Z_a7.json` | A7, runs 1 to 3 | `dfd44a5` | **14 PASS, 2 WARN (M07, V01), 0 FAIL, 0 NOT_RUN, each** |

- **Which A6 WARN cleared (24/4 to 25/3): T08.** In P18's report (`20261018T172411Z_dev.json`, platform-sw `174731c`) T08 listed six platform-sw links whose collision mesh is their visual STL
  (`base_link`, the four caster links, `lidar_link`); in every report since P19 it lists none. platform-sw #70 (`d619012`, Cuma Ali Karaaslan) replaced those collision meshes with boxes and
  cylinders. Nothing in this repository changed T08. It is the only check whose result differs between the two A6 reports.
- **The `parameter_bridge` SIGSEGV did not occur in these three runs.** Over the 9 A7 runs on `dfd44a5` it occurred in 2 (P19), both G02, both `base_joint_states_bridge`. One bridge exited -6 (allowed) in
  run 1 here. `move_group` exit -11 (allowed, at every stop) appears 7 times in every run, as before. Nothing was added to the allow list.
- **W04: no force-stop in the 12 launches of these runs** (all `stopped` on the first step, 1.33 to 1.43 s); 36 of 36 launches of P19 and P20 together, 1.32 to 1.53 s.
  P17's 36 of 36 before the spawner change were 1.32 to 1.44 s: the timing is the stop helper's and does not distinguish the spawner change.
- **M02**: 0 excursions in all three; largest overshoot 1.61e-6, 1.33e-6, 1.71e-6 rad against 1e-5 (not widened). **M07** WARN on the four joints in all three.

## P21

Tested code `fe2e821`: W04 reads incomplete evidence as NOT_RUN (Mohammad's finding), the M03 caster-clearance regression (platform-sw #71), the `config_id` payload note, the
PR-body statements on W03 and M07 (`CHANGELOG.md`, P21). platform-sw `dfd44a5`, clean tree; the pushed head adds only these reports and documents.

| file | what | platform-sw | result |
|---|---|---|---|
| `20261019T003746Z_dev.json`, `...T004121Z_dev.json`, `...T004454Z_dev.json` | A6 suite, runs 1 to 3 | `dfd44a5` | 25 PASS, 3 WARN (E01, W02, V01), 0 FAIL, 0 NOT_RUN, each |
| `20261019T005239Z_a7.json`, `...T010031Z_a7.json`, `...T010818Z_a7.json` | A7, runs 1 to 3 | `dfd44a5` | **14 PASS, 2 WARN (M07, V01), 0 FAIL, 0 NOT_RUN, each**; AUX 131 tests, 0 failed; H01 and H02 PASS |

- **`parameter_bridge` SIGSEGV: 0 in these 12 launches** (one bridge exit -6, allowed, in run 3). Cumulative (**OBSERVED-NOT-RECORDED**: the -11 exits were read from the console, no report or log here carries them; the reports record only that G02 and AUX failed): **2 of 60** launches since P18 (P18 12, P19 on `d619012` 12, P19 on `dfd44a5` 12, P20 12, P21 12);
  on `dfd44a5` alone **2 of 36**, both in P19, both G02 `base_joint_states_bridge`; A7 runs on `dfd44a5`: 9, of which 2 FAILED (G02, AUX) and 7 passed with 0 FAIL. Not allow-listed.
- **W04**: PASS in all three (12 launches, all `stopped`, 1.33 to 1.43 s). It now reads only `stopped` and `force_stopped` as results; see `CHANGELOG.md`, P21.
- **M02**: 0 excursions in all three; largest overshoot 1.28e-6, 1.99e-6, 2.15e-6 rad against 1e-5 (not widened). **M07** WARN on the four joints in all three.
- **The 1-in-7 search.** No `parameter_bridge` exit -11 in any P16 or P17 report or log: the twelve P16 A7/A6 reports (`20261014T12*` and `T13*`, on `9bc327c` and `aa94fc3`) mention neither -11 nor
  -6 for the bridge, and the 36-run orphan count, the shutdown timings and the bisect logs have none. The only recorded bridge crash before P19 is one SIGABRT (-6) in a P12A G01 log
  (`docs/verification.md`). The claim in P16's report to the architecture chat was unbacked and is recorded as such in `CHANGELOG.md`.


## P23

Tested code `77374a0`: the generators no longer write that the lift column has no collision geometry, `datums.yaml` cites the register's CAMERAS at P-03 rev19.4 for `base_link`'s 0.098 m, and the datums revision is `2026-10-20.a` (so every `config_id` differs from P21's: all 14 in the A6 reports, all 9 in the A7 reports). platform-sw `dfd44a5`, clean tree, a clean rebuild first; the pushed head adds only these reports and documents.
The workspace's other checkouts are in `../p23/workspace_checkouts.txt`: `openamrobot-manipulation` is `795143e` for every run on this workstation (P15 to P23), not the `095e6c7` of Alex's container.

| file | what | platform-sw | result |
|---|---|---|---|
| `20261020T104401Z_dev.json`, `...T104733Z_dev.json`, `...T105107Z_dev.json` | A6 suite, runs 1 to 3 | `dfd44a5` | 25 PASS, 3 WARN (E01, W02, V01), 0 FAIL, 0 NOT_RUN, each |
| `20261020T105921Z_a7.json` | A7, run 1: **FAILED** | `dfd44a5` | 13 PASS, 1 WARN (V01), 2 FAIL (M07, AUX), 0 NOT_RUN |
| `20261020T110705Z_a7.json`, `...T111449Z_a7.json` | A7, runs 2 and 3 | `dfd44a5` | 14 PASS, 2 WARN (M07, V01), 0 FAIL, 0 NOT_RUN, each |

- **Run 1 failed on the `parameter_bridge` SIGSEGV, the third occurrence, and the first in M07's launch** (the two of P19 were in G02's). M07's physics were as in the passing runs: no failing joint, the same four WARN joints. What failed is `test_m07_processes_started_and_exited_cleanly`, and AUX with it because the bringup `colcon test` step exited 1; both are recorded in the report. The exit code is not: the last lines the driver kept are `../p23/a7_run1_console_tail.txt` ("Proc parameter_bridge-4 exited with code -11"). The stop took 1.42 s and nothing survived. W04 PASS in all three runs; M02 0 excursions in all three, largest overshoot 1.99e-6, 1.54e-6 and 3.71e-6 rad against 1e-5 (not widened). The failed run is kept beside the two clean ones, and no run was repeated to replace it. Nothing was added to the allow list.
- **Rates of this bridge exit are OBSERVED-NOT-RECORDED**, here and everywhere: the JSON of an A7 report does not carry exit codes, so "2 of 60" (P21 section above), and now 3 of 72 launches since P18 and 3 of 48 on `dfd44a5`, were read from the console. The reports record which checks failed in which runs. Option A (recording each process's exit code in the stop-helper sidecar) would make the next occurrence data; it would not recover the past ones, and it was not done.
- **Which reports carry the false column statement.** The generators' limitation "the lift column has no collision geometry (its lateral extent is not sourced): the arms can plan through the column body" (and, in the A6 reports, the V01 item 1 note "nothing is drawn for the column") is in 56 of the reports above. It was true of the 32 reports of P15 to P18 (`20261013T101652Z_dev.json` to `20261018T175119Z_a7.json`), produced on code without the column boxes. It is **false in the 24 reports of P19 to P21, `20261018T191032Z_dev.json` to `20261019T010818Z_a7.json`**, produced on code with `b45d8f6`'s boxes (tested code `538a32a`, `24c66f5` and `fe2e821`). It was a generator string and not a finding: nothing was measured that said the arms can plan through the column. None of the reports is edited; the six reports of this section carry the corrected limitation.

## P24

Tested code `15109f9`: every launch test records the name and exit code of each process of its launch in its sidecar (`process_exits`), and the A7 report lists them per launch. Nothing allow-listed. platform-sw `dfd44a5`, datums revision `2026-10-20.a`, clean tree, a clean rebuild first; the pushed head adds only these reports and documents.

| file | what | platform-sw | result |
|---|---|---|---|
| `20261020T115041Z_dev.json`, `...T115416Z_dev.json`, `...T115751Z_dev.json` | A6 suite, runs 1 to 3 | `dfd44a5` | 25 PASS, 3 WARN (E01, W02, V01), 0 FAIL, 0 NOT_RUN, each |
| `20261020T120539Z_a7.json`, `...T121325Z_a7.json`, `...T122112Z_a7.json` | A7, runs 1 to 3 | `dfd44a5` | 14 PASS, 2 WARN (M07, V01), 0 FAIL, 0 NOT_RUN, each |

- **The `parameter_bridge` did not crash in these runs.** The A7 reports carry `process_exits` for 7 launches each (G01, G02, M01 at three lift stops, M02, M07); the four simulator launches of a run have two `parameter_bridge` processes (platform-sw's and ours), all 24 recorded with exit code 0 in the 12 simulator launches. The only other non-zero exits are the documented, allowed ones: `move_group` -11 in the launches that have one (M01 at its three stops, M02) and the `gazebo` wrapper -2 in the four simulator launches. So the rate **recorded** is 0 of 12 launches. It is derived by `tools/check_evidence_backing.py` (kind `launch_exits`), not written by hand.
- **The three earlier occurrences remain OBSERVED-NOT-RECORDED**: P19 runs 2 and 3 (G02) and P23 run 1 (M07). They cannot be recovered, and the reports of P19 to P23 have no `process_exits`. The observed and the recorded figures are not added together.
- **Run level, from the failures the reports record:** of the 15 A7 runs on `dfd44a5` of P19 to P24, 3 failed on this crash, 20 percent (3 of the 12 of P19 to P23, 25 percent). The cause of those three is the console's -11, not the JSON.

## P25

Tested code `ecdfcf2` (the placeholder boxes get one named material, in the URDF and for Gazebo); the reports name `1165166`, which differs from it only by `evidence/p25/`. platform-sw `dfd44a5`, datums revision `2026-10-20.a`, clean tree, the description package rebuilt first (the new test is registered in its CMakeLists). The same machine, the same upstream pins and the same procedure as P24, so the P24 reports are the baseline.

| file | what | platform-sw | result |
|---|---|---|---|
| `20261021T084244Z_dev.json`, `...T084621Z_dev.json`, `...T085000Z_dev.json` | A6 suite, runs 1 to 3 | `dfd44a5` | 25 PASS, 3 WARN (E01, W02, V01), 0 FAIL, 0 NOT_RUN, each |
| `20261021T085748Z_a7.json`, `...T090537Z_a7.json`, `...T091325Z_a7.json` | A7, runs 1 to 3 | `dfd44a5` | 14 PASS, 2 WARN (M07, V01), 0 FAIL, 0 NOT_RUN, each |

- **Nothing moved.** `../p25/after_material_vs_p24.txt` compares the six runs with the six of P24 (`15109f9`) check by check: every check has the same result in all six, the upstream commits, the datums revision and the provenance counts are identical, and M03 has no differing value (95 disabled pairs, the same sampled counts). What differs, and why: the 14 `config_id`s (the URDF text is an input of each, so a changed colour changes them: expected); the test counts (AUX 298 to 307 and H02 46 to 47: the nine tests of `test_p25_placeholder_material.py`, which the harness has no check ID for and so counts under AUX as unclaimed `P25`, 0 failures); and run-to-run noise (process ids, clamp message counts, `right_arm_moved_rad`; M02's worst final error 0.00118 to 0.00129 rad against 0.00112 to 0.00126 in P24, gate 0.01, three runs each: the ranges overlap and nothing is read into the 0.00003 at the top).
- **Process exits (recorded).** 24 `parameter_bridge` processes in the 12 simulator launches of the three A7 runs: none ended in -11; one ended in -6 (G01, run 3), which the guard allows (`launch_guard.py`, the shutdown abort of the bridge, seen in P16). `move_group` -11 (allowed, at every stop) appears 4 times in every run, as before.
- **No orphan.** `pgrep -af "^gz sim"` after each A7 run, and a sweep of my own processes by environment (private domains 101 to 200, the test partitions), found nothing to kill after any of the three.
- **Run level, now:** of the 18 A7 runs on `dfd44a5` of P19 to P25, 3 failed on the bridge crash, 17 percent; none of the six of P24 and P25 did.

## P27

Three A6 and three A7 runs on `26f539c` (the M07 non-finite fix, Om's two tests, the merge-marker fix), clean tree, platform-sw `dfd44a5`. Details in `../p27/README.md`.

| file | run | result |
|---|---|---|
| `20261021T131959Z_dev.json` | A6 1 | 25 PASS, 3 WARN, 0 FAIL |
| `20261021T132342Z_dev.json` | A6 2 | 25 PASS, 3 WARN, 0 FAIL |
| `20261021T132726Z_dev.json` | A6 3 | 25 PASS, 3 WARN, 0 FAIL |
| `20261021T133515Z_a7.json` | A7 1 | 14 PASS, 2 WARN (M07, V01), 0 FAIL |
| `20261021T134303Z_a7.json` | A7 2 | 14 PASS, 2 WARN (M07, V01), 0 FAIL |
| `20261021T135052Z_a7.json` | A7 3 | 14 PASS, 2 WARN (M07, V01), 0 FAIL |


## P28

Three A6 and three A7 runs on `a14a3ff` (the lift column on the DOLD datasheet's sections, the carriage as the inner stage, the adapter plate, the re-derived matrix), clean tree, platform-sw `dfd44a5`, datums revision `2026-10-28.a`. Details in `../p28/README.md`.

| file | run | result |
|---|---|---|
| `20261028T002351Z_dev.json` | A6 1 | 25 PASS, 3 WARN, 0 FAIL |
| `20261028T002738Z_dev.json` | A6 2 | 25 PASS, 3 WARN, 0 FAIL |
| `20261028T003126Z_dev.json` | A6 3 | 25 PASS, 3 WARN, 0 FAIL |
| `20261028T003921Z_a7.json` | A7 1 | 14 PASS, 2 WARN (M07, V01), 0 FAIL |
| `20261028T004705Z_a7.json` | A7 2 | **12 PASS, 2 WARN, 2 FAIL (G02, AUX)** |
| `20261028T005455Z_a7.json` | A7 3 | 14 PASS, 2 WARN (M07, V01), 0 FAIL |

- **A7 run 2: G02 and AUX.** G02 failed because `parameter_bridge-3` exited -11 (the shutdown SIGSEGV, not allow-listed, tracked upstream by the drafts in `docs/issues/`); recorded in the report's `process_exits`. AUX failed on `test_zz_a_reading_beyond_the_epsilon_is_not_hidden` (`test_joint_state_clamp_loop.py:146`, `42 != 41`): the test reads the clamp's activation counter, switches the injected value, and asserts the counter did not move, so one more in-epsilon reading clamped between the two reads fails it. It is a race in an existing test, AUX passed in the nine A7 runs of P24, P25 and P27 and in the other two A7 runs of P28 (the P26 and earlier runs were not re-counted); I did not investigate or change it (outside this step). Extracts: `../p28/a7_run2_failures.txt`.
- **M03 (all three runs, PASS):** 99 disabled pairs (20 upstream, 38 `Adjacent` in total, 61 `ConstantTransform`), 567 of 666 enabled; no unjustified pair, no wrong label, no constant pair colliding at any stop.
- **Process exits (recorded):** 24 `parameter_bridge` processes in the 12 simulator launches of the three A7 runs; one ended in -11 (run 2, G02).

## P29

Three A7 runs on `c84cfbd` (the AUX race fixed in the test, `4f2c3b4`; docs only after that), clean tree, platform-sw `dfd44a5`. A6 was not re-run: nothing it covers changed. Details in `../p29/README.md`.

| file | run | result |
|---|---|---|
| `20261028T090816Z_a7.json` | A7 1 | 14 PASS, 2 WARN (M07, V01), 0 FAIL |
| `20261028T091706Z_a7.json` | A7 2 | 14 PASS, 2 WARN (M07, V01), 0 FAIL |
| `20261028T092556Z_a7.json` | A7 3 | 14 PASS, 2 WARN (M07, V01), 0 FAIL |

- **Root cause of the P28 AUX failure.** `test_zz_a_reading_beyond_the_epsilon_is_not_hidden` compared the clamp's `activations` read before the injected value was switched with `activations` read after it. The publisher keeps sending in-epsilon noise until the switch lands and each such message is a legitimate clamp, so one in flight when the first number was read made the two differ by one (42 against 41). Nothing beyond the epsilon was counted as a clamp. It was a race in the test, not a clamp fault.
- **Fix (`4f2c3b4`, test only).** The baseline is the snapshot in which the first beyond-epsilon reading has been counted, and `max_magnitude_rad` is checked within one snapshot. No clamp, epsilon or margin changed.
- **30 of 30.** The test alone, 30 iterations of the clamp-on launch test, the testcase read from the xunit each time: 30 pass, none skipped (`../p29/clamp_loop_30.txt`).
- **Two mutations, both caught.** A clamp that also counts every beyond-epsilon reading as a clamp fails (`85 != 43`); a clamp that counts only the first one fails (`0.001 != 2.5e-07`); the real clamp passes (`../p29/mutation_check.txt`). They were run with `launch_test` directly: under `colcon test` the install's `PYTHONPATH` hook wins and a mutation placed on `PYTHONPATH` does nothing.
- **A7 x3: 14 / 2 / 0, each.** No check changed result against the clean P28 run; M03 99 disabled, 567 of 666 enabled in all three. The AUX failure did not occur.
- **Process exits (recorded):** 24 `parameter_bridge` processes in the 12 simulator launches of the three runs: none ended in -11 (`../p29/run_exit_codes.txt`). Over P24 to P29, the 60 launches whose exit codes are recorded have one -11 (P28 run 2, G02, `parameter_bridge-3`, which is `platform_bridge` of openamr-platform-sw and not this branch's `base_joint_states_bridge`). The observed console figures (up to P23) are a different basis and are not added to it.
- **Run level, now:** of the 27 A7 runs on `dfd44a5` of P19 to P29, 4 had a failed check on the bridge crash, 15 percent (3 observed, 1 recorded; kept apart in the PR body). The reports of this section were first committed in `evidence/p29/reports/` and moved here in P30, together with the PR body and ledger figures that derive from them.
