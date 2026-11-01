# P23 evidence

| file | what |
|---|---|
| `a7_run1_console_tail.txt` | the last lines the driver kept of the one failed A7 run on `77374a0` (`20261020T105921Z_a7.json`): `Proc parameter_bridge-4 exited with code -11`. A partial record; the full launch log was not kept |
| `workspace_checkouts.txt` | the checkouts of the workspace the runs used: `openamrobot-manipulation` `795143e` (one reflog entry, the clone of 5 Oct), `095e6c7` is not in it; GitHub's compare of the two (two docs-only commits) |
| `decision_currency_run.txt` | the currency check on the real list and documents after the `datums.yaml` re-citation: 0 STALE |
| `decision_currency_canary.txt`, `evidence_backing_canary.txt` | the planted faults, each caught |
| `evidence_backing_run.txt` | every count and rate in the PR body and the newest reports: BACKED, RECORDED, OBSERVED-NOT-RECORDED, or acknowledged UNBACKED with its reason |
| `pytest_tools.txt` | the tests of the two tools |

Each output file starts with the command, the repository commit and the register commit it read. `decision_currency_run.txt` needs the design folder on the Windows side, which is not in this repository.
