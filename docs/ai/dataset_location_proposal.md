<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Where OpenAMRobot datasets live on the Hugging Face Hub (Proposed, 19 Nov 2026)

Status: Proposed, for the owners to accept, change or reject. The licence is the founder's decision; the
organisation, naming and validation are for the embodied-AI owner and the software lead. Nothing is published.

Synthetic and simulated data is not real robot data. A dataset recorded in Gazebo or MuJoCo says so in its name, its
card and its sidecar, and is never presented as a recording of the physical robot.

## Organisation

- Proposed organisation: `openamrobot` (the GitHub organisation's name in lower case; the embodied-ai converter's own
  fixture already uses `dataset_id: openamrobot/...`). Fallback: `botshare`.
- Not verified: whether `huggingface.co/openamrobot` (or `openAMRobot`, `botshare`) exists or is free. huggingface.co
  was not reachable from the session that wrote this page, and a web search found no such organisation page. The
  founder checks this on huggingface.co and creates or claims the organisation; admins mirror the GitHub
  organisation's maintainers.

## Repository naming (one dataset repository per dataset line)

`openamrobot/<origin>-<source>-<task>-<contract>`, lower case, for example:

| Dataset line | Repository |
|---|---|
| Gazebo Baseline 1 captures | `openamrobot/sim-gazebo-baseline1-c9v1` |
| MuJoCo scripted episodes | `openamrobot/sim-mujoco-<task>-c9v1` |
| Real robot (from 2027) | `openamrobot/robot-<site>-<task>-c9v1` |

`<origin>` is `sim` or `robot`, so the origin is visible before anyone opens the card. `<contract>` is the C9 contract
version the episodes follow. A new contract version is a new repository, not a new revision.

## Revisions

- LeRobot 0.6.1 (the converter's pin) can tag a pushed dataset with its format version (`CODEBASE_VERSION`, `v3.0`; `push_to_hub(tag_version=...)`) and,
  when no revision is given, resolves the newest compatible format tag (and refuses a repository with no such tag). It treats every tag or branch that parses as a
  PEP 440 version (`1.0`, `v1.2.0`, `2026.11.19`) as a format version.
- So dataset release tags must not parse as a version. Proposed: `release-YYYYMMDD-N` (for example
  `release-20261119-1`), one immutable tag per dataset version, never moved. The LeRobot `v3.0` tag stays as LeRobot
  writes it.
- The release entry in embodied-ai `datasets/registry.yaml` pins the commit, not only the tag:
  `storage_uri: hf://datasets/openamrobot/<repo>@<40-hex commit>`, with `sha256` of the release manifest. The
  registry's `dataset_id` (no slash allowed) is `<repo>-<release tag>`; the converter profile's `dataset_id` is the Hub
  repository id and its `dataset_revision` is the release tag.

## Licence (founder's decision)

| Option | Effect | Fit |
|---|---|---|
| CC-BY-4.0 | Reuse, also commercial, with attribution | Matches the licence map for documentation; credits contributors by licence |
| CDLA-Permissive-2.0 | Data-specific permissive licence, results of use unrestricted | Permissive; less familiar to robotics users |
| CC0-1.0 | No conditions | Widest reuse; attribution only by convention |

Share-alike licences (CC-BY-SA, ODbL) are not proposed: they would bind users who build commercial robots on the data.
Before any publication: the licences of every model, mesh and texture rendered in the scene (NOTICE.md) are checked
against the chosen dataset licence, and each contributor of episodes agrees to the licence of the dataset their
episodes go into. The external contributor who generated the six MuJoCo episodes is credited by name in the dataset
card, `CITATION.cff` and the sidecar's contributor list, with their agreement to the licence recorded first.

## Provenance (every episode sidecar and the dataset card)

- `origin`: the converter's enum (`synthetic`, `simulation`, `robot`). Proposed reading: `simulation` for Gazebo and
  MuJoCo episodes, `synthetic` only for generated fixtures. In every case: not real robot data.
- Generator: tool and version (for example `oa-convert-bag` at its commit), the policy or script that produced the
  motion (Baseline 1 mission, a scripted MuJoCo trajectory, teleoperation).
- Commit SHAs (40 hex): openamr-upperbody-sw, openamr-platform-sw, openamrobot-interfaces, openamrobot-embodied-ai, and
  the pinned OpenArm repositories (`pins.txt` of the run).
- Simulator and version: Gazebo (gz-sim 8.15.0, physics engine, step) or MuJoCo (version, timestep); real-time factor
  of the run.
- Capture profile: `profile_id`, its reviewed `sha256` (`capture_contract_digest`) and `approval_ref`; the C9 component
  contract path and sha256.
- Declarations of the run: acceptance profile (`sim-oracle` or `perception`), `planning_pose_source` and
  `object_pose_source` as `/build1/run_info` declares them. A `ground_truth` source is a simulation oracle and the card
  says so.
- The run id and artifact name the episode came from.

The registry schema (`schemas/dataset_registry.schema.json`, `additionalProperties: false`) holds only part of this;
the rest goes into the sidecar and the dataset card unless the embodied-AI owner extends the schema.

## Units

Radians for every angular channel, metres for the lift, m/s and rad/s for the base (decision on #79, 17 Nov 2026:
radians everywhere, recorded datasets included). The accepted C9 v1 LeRobot profile and the six MuJoCo episodes use
degrees: a dataset is either converted to radians at export (the converter supports degree to radian) with the source
unit kept in the sidecar, or the C9 v1 profile is amended. Which one is a decision for the owner of #79 together with
the embodied-AI owner; until then no degree dataset is published under a radian contract name.

## Validation before a release tag

1. The converter's own gates, unchanged: `validate_profile` with a registered, approved capture profile, `read_bag`,
   `align` and the C10 dataset QA (`oa-dataset-qa`). A refusal is a failed release, never relaxed.
2. The LeRobot loader at the converter's pin (`lerobot==0.6.1`): `LeRobotDataset(repo_id, revision=<release tag>)`
   opens every episode, frame counts and fps match the sidecar, every camera feature decodes.
3. MuJoCo replay: the recorded joint commands replayed in the MuJoCo model at the dataset's commit reproduce the
   recorded joint states within a tolerance the embodied-AI owner sets per joint group (none is proposed here).
4. The registry entry (`quality_status: passed`) lands in a reviewed embodied-ai PR, then the tag is created.

None of these steps is physical, safety or release acceptance of the robot.
