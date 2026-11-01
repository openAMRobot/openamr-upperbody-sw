# P27 evidence

The three reviewer findings on PR #14 at `ba5b4ea` (M07 non-finite samples, Om's two description tests, the red `repository-quality` check), and the runs of the fix.

| file | what |
|---|---|
| `m07_nan_repro.py` | Mohammad's two M07 reproductions (50 NaNs per expected joint; 49 finite readings plus one NaN), run through the real helpers; no simulator |
| `m07_nan_before.txt` | at `ba5b4ea`: both print **PASS** (the plain one-NaN-last variant and a NaN-first variant too) |
| `m07_nan_after.txt` | with `a7_support.evaluate_m07`: **FAIL** for all three |
| `merge_marker_scan.txt` | the organisation's merge-marker command over the tree of `ba5b4ea` (one hit: the pytest separator at `evidence/p24/pytest_process_exits.txt:14`) and over this commit's tree (none) |
| `pytest_p27.txt` | the focused tests after the change (M07 non-finite and coverage; `evidence_dir`, `build_evidence`, `merge_markers`) |
| `m07_mutation_check.txt` | the regression against a copy of `a7_support.py` with the finite filter disabled: 13 of the 42 fail (both reproductions, mixed, infinities, the 50-sample count); against the real file all pass |
| `run_exit_codes.txt` | exit status of the six runs below, and the count of `gz sim` processes left after each A7 run |

**The runs**, all on `26f539c` (code and tests; clean tree, platform-sw `dfd44a5`), in `../reports/`:

| file | run | result |
|---|---|---|
| `20261021T131959Z_dev.json`, `20261021T132342Z_dev.json`, `20261021T132726Z_dev.json` | A6 `verify_a6.sh` x3 | 25 PASS, 3 WARN (E01, W02, V01), 0 FAIL |
| `20261021T133515Z_a7.json`, `20261021T134303Z_a7.json`, `20261021T135052Z_a7.json` | A7 `run_a7_regression.sh` x3 | 14 PASS, 2 WARN (M07, V01), 0 FAIL |

In each A7 report M07 is WARN for exactly the four upstream-limit joints, `failing_joints` empty, `invalid_readings` empty, 600 valid of 600 raw readings for every expected joint, and no `parameter_bridge` exit other than 0
(`gazebo` -2 and `move_group` -11 are the allowed shutdown exits). An earlier set of the same six runs on `8b8454f` gave the same counts; it was superseded, not hidden: `classify` was then tightened (WARN only at the inherited zero limit)
and the runs were repeated on the final code. Its reports are not committed.

Not covered by these runs: a real NaN from the simulator (none occurred); the non-finite path is exercised by `test_m07_non_finite.py` on synthetic readings, and by the mutation check described in the PR comment.
