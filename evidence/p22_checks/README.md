# P22: outputs of the two prototype checks

Console output of `tools/check_decision_currency.py` and `tools/check_evidence_backing.py` (run and `--canary`) and of `tools/tests`, taken from the clean tree at
`8ad3bf6` on 19 Oct 2026. Each file starts with the command, the repository commit and, for the currency check, the commit of the register that was read
(`openAMRobot/.github` `decisions.yaml` @ `9109d3e1a8fc07151e500e83b32b7ce6ccab6af6`, in force at P-03 rev19.6). The register was cloned read-only to a scratch directory; the design
folder is the author's `10_architecture` directory on the Windows side, which is not part of this repository, so `decision_currency_run.txt` can only be reproduced where both exist.

| file | what it shows |
|---|---|
| `decision_currency_run.txt` | the real list against the real documents: 28 CURRENT, 3 HISTORICAL, **1 STALE** (`datums.yaml` line 30 cites FRAMES-REP105 at rev19.0; the register records rev19.4), exit 1 |
| `decision_currency_canary.txt` | six planted faults, each caught, and the clean sentence passing; one of the six is a copy of a real document with a real citation re-dated. The real list is not clean, which the file says |
| `evidence_backing_run.txt` | the PR body and the newest A6 and A7 reports: every claim BACKED, RECORDED or acknowledged UNBACKED (67 of them, each with its reason), exit 0 |
| `evidence_backing_canary.txt` | planted unbacked numbers in the body, in a report's prose, in the ledger, and in a report-index row, each caught; acknowledged and clean cases pass |
| `pytest_tools.txt` | 33 tests of the two tools |

`evidence_backing_run.txt` is the list of numbers in the PR body that no file under `evidence/` shows. It is not a statement that they are wrong.
