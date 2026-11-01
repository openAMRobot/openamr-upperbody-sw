# P24 evidence

| file | what |
|---|---|
| `pytest_process_exits.md` | the tests of the recording, including the canary: a synthetic -11 planted in a sidecar comes out of the A7 report as a rate that `check_evidence_backing.py` derives and backs; other numbers are a MISMATCH |
| `evidence_backing_canary.txt` | the planted faults of the backing check, including the synthetic `launch_exits` -11 case |
| `evidence_backing_run.txt` | the PR body and the newest reports; `0 of 12` launches with a `parameter_bridge` ending in -11 is BACKED from `process_exits`, the three earlier occurrences are OBSERVED-NOT-RECORDED |
| `decision_currency_run.txt` | the currency check: 0 STALE |
| `pytest_tools.txt` | the tests of the two tools |

The A7 reports themselves are `../reports/20261020T120539Z_a7.json` and the two after it; their `process_exits` is the data.

`pytest_process_exits.md` is a `.md` file on purpose and its text is the unedited pytest output. Its last line is pytest's own summary separator, a long run of `=`, and the organisation's
`repository-quality` workflow (`openAMRobot/.github`, `repository-quality-reusable.yml@main`, step "Reject unresolved merge markers") fails on any tracked line that starts with seven `=`, except in
`*.md`, `*.patch` and `*.diff`. The workflow is not ours to change; the test `test_merge_markers.py` runs the same command locally. (P27, run 39575711096.)
