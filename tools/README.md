# tools

Scripts that are not part of a ROS package.

## Two prototype checks (P22)

Both are **run by hand**. Neither is called by `verify_a6.sh`, `run_a7_regression.sh`, a workflow or any other repository's CI, and neither
touches a file it reads. Python 3 with PyYAML (`python3-yaml`) is enough; no ROS is needed.

### `check_decision_currency.py`: does a document cite the revision the register records?

```bash
git clone https://github.com/openAMRobot/.github /tmp/harness        # or any checkout; the commit it prints is the one that was read
python3 tools/check_decision_currency.py --register /tmp/harness/decisions.yaml \
        --design-root /path/to/design/10_architecture                  # or --repo-only
python3 tools/check_decision_currency.py --register /tmp/harness/decisions.yaml --canary
python3 tools/check_decision_currency.py --register /tmp/harness/decisions.yaml --discover
```

`docs/decision_citations.yaml` lists (document, decision ID, revision cited), each with a verbatim `quote` so the list cannot drift from the text. The
check compares the revision with the `source.document` the register records for that decision. States: `CURRENT`, `STALE`, `AHEAD`, `SUPERSEDED`,
`HISTORICAL` (kept on purpose, with a reason), `UNKNOWN_*`, `QUOTE_*`, `UNLISTED` (a living document names a hyphenated decision ID next to a revision and
the list has no entry). Exit 0 clean, 1 a failure, 2 unreadable input, 3 design documents not checked.

What it does **not** prove: that the value a document states is the register's value (the register's own `tools/check_decisions.py` scans for
contradicting phrases), or anything about a citation that names no revision. Citations that name an item number and no register ID
(`P-03 rev19.0 item 19 D2`) are not covered.

`--canary` plants faults in temporary copies (a stale citation, an unknown decision, a quote that is not in the document, a list that disagrees with the
quote, an unlisted citation, and a real document of this repository with one citation re-dated) and exits 0 only if every one is caught.

### `check_evidence_backing.py`: does every count and rate resolve to a file under `evidence/`?

```bash
python3 tools/check_evidence_backing.py                  # docs/pr/PR14_BODY.md and the newest committed A6 and A7 reports
python3 tools/check_evidence_backing.py -v               # the backed claims too
python3 tools/check_evidence_backing.py --report evidence/reports/<new>.json
python3 tools/check_evidence_backing.py --skeleton       # ledger entries for what nothing backs yet
python3 tools/check_evidence_backing.py --canary
```

A claim is a verdict count (`25 PASS / 3 WARN / 0 FAIL`), a rate (`2 of 36`, `about 1 in 13`), a count (`826 tests`) or a percentage. It is `BACKED` (a
committed report has it, or `docs/pr/claim_backing.yaml` says how to derive it from committed reports), `RECORDED` (a written record under `evidence/`
says it: weaker, and labelled so), `UNBACKED` with an acknowledged reason, `OBSERVED-NOT-RECORDED` (seen, never recorded, and labelled so in the text), or `UNBACKED` without a reason, which fails. The table rows of
`evidence/reports/README.md` are read for one thing: the report files named in a row must have the verdict counts the row states.

What it does **not** prove: that a number is right (a record written by us is only as good as we were), and it does not see numbers that are not counts of
the nouns it knows. Exit 0 clean, 1 a failure, 2 unreadable input.

### Tests

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest tools/tests -q -p no:cacheprovider
```

They use small registers and trees of their own; they do not read the real `decisions.yaml`.

### Wiring them in, if someone wants to (not done)

Run `--canary` first in the same job (a check that cannot fail is not a check) and pin the register to a harness commit (the same SHA as the caller's
`harness_ref`). Neither tool should be added to another repository's workflow. Since P23 the currency check is clean on the real list (0 STALE): the
`datums.yaml` citation that P22 left red was re-cited with the revision bump to `2026-10-20.a`.

Since P24 an A7 report carries `process_exits` (the exit code of every process of every launch), and the ledger kind `launch_exits` derives "N of M launches with a `parameter_bridge` ended in -11" from it: BACKED for runs since P24, nothing for earlier reports.

The backing check has a fourth status besides BACKED, RECORDED and UNBACKED: `OBSERVED-NOT-RECORDED`, for a number someone saw (a console line) and nothing
recorded. It is allowed only while the text on the claim's line says OBSERVED-NOT-RECORDED.

## `compare_option_b_wrapper.sh`

The byte-level comparison of option B's wrapper against `upstream/main` that the PR body cites.
