# Working state: focused operator-UI handoff proposal

Prepared 28 November 2026 against `main` at
`1ab45a62957d238fc293eee08a8fd9158acebb01`.
This branch introduces this focused record because STATE.md is absent at that
base. It is not a summary of whole-project readiness or acceptance.

## Proposed documentation handoff

`docs/architecture/upperbody_ui_handoff.md` records existing UI/control-room
reuse, model/configuration source ownership, URDF-derived lift bounds and units,
resource closure, reconnect behavior and the separate telemetry/control/camera
gates. README links the document. No runtime source or release input changes.

The user explicitly requested a new PR for presently possible work. This draft
is preparation only: UI model implementation and #64 merge still wait for the
rc.1 tag and owner sequence. UI #55/#62/#68 and upperbody #84 remain the reuse
points; no competing viewer or control room is added.

Validation scope: Markdown/source-link and diff checks only, reported in the PR.
No ROS, browser/model rendering, camera, MoveIt, physical or safety validation
has been executed for this documentation proposal. Owner adopt/adapt/reject
and any runtime contract decisions remain pending.

AI disclosure: Codex authored the handoff, README pointer and this focused record
from the linked repository sources and owner instructions. No dependency,
licence, CODEOWNERS or safety implementation changes.
