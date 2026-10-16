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
| `20261015T121048Z_a7.json`, `...T121946Z_a7.json` | A7, runs 2 and 3 | `013b524` | 13 PASS, 1 WARN (V01), 2 FAIL (M07, and AUX with it), 0 NOT_RUN, each |

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
