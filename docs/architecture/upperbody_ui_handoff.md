# Upperbody model handoff to the operator UI

Status: **Proposed, documentation only.** This is a preparation handoff for
owner review, not an accepted runtime interface or a new implementation.
The combined-model UI work starts after the rc.1 tag in the agreed integration
sequence. This document changes no release selection or publication decision.

## Scope and existing work

The browser operator interface belongs in `openamrobot-ui`. This repository
owns the upperbody model and its integration inputs. Reuse the existing work:

| Work | Reuse boundary |
|---|---|
| [UI integration inventory #68](https://github.com/openAMRobot/openamrobot-ui/pull/68) | Integration order and ownership; refresh moving heads before implementation. |
| [UI upperbody view #55](https://github.com/openAMRobot/openamrobot-ui/pull/55) | Read-only model and joint-state components; extend with its owner rather than opening another viewer. |
| [UI twin operator view #62](https://github.com/openAMRobot/openamrobot-ui/pull/62) | Existing twin launch and operator-status integration; coordinate resource/profile changes there. |
| [Upperbody control room #84](https://github.com/openAMRobot/openamr-upperbody-sw/pull/84) | Read-only simulation monitoring and replay/reporting; do not introduce a competing control room here. |
| [UI bridge contract #63](https://github.com/openAMRobot/openamrobot-ui/issues/63) and [implementation #64](https://github.com/openAMRobot/openamrobot-ui/pull/64) | Every new ROS caller changes its exact allowlist entry and classification test in the same consuming PR. |

The preparation assessment inspected UI #55 at
`0d4caf5dc64b1f58d5f452f5dcf565d910a0bf43`, UI #62 at
`796acfe464482f0c96fbaaaf35c36de1ebec896b`, and control room #84 at
`dda35832ff5f4edd893ca589af13a88d35d258cc`. These are review references,
not release pins, accepted implementations or evidence of current deployment.

## Model inputs: source files, not new wire fields

| Input | Existing source / handoff requirement |
|---|---|
| Combined description | [combined_robot.xacro](../../ros2/src/openamrobot_upperbody_description/urdf/combined_robot.xacro); use the expansion selected by the deployment. |
| Configuration axes and geometry | [upperbody_configuration.yaml](../../ros2/src/openamrobot_upperbody_description/config/upperbody_configuration.yaml); unresolved inputs remain unresolved. |
| Configuration identity algorithm | [print_config_id.py](../../ros2/src/openamrobot_upperbody_description/scripts/print_config_id.py); reuse the existing identity rather than inventing a browser hash. |
| Launch/profile semantics | [bringup README](../../ros2/src/openamrobot_upperbody_bringup/README.md); distinguish reference viewing, mock, simulation and physical profiles. |
| Planning semantics | [MoveIt README](../../ros2/src/openamrobot_upperbody_moveit/README.md); preserve declared limitations and placeholder status. |

Configuration arguments are `column`, `base_plate`, `head_pitch`, `base_tilt`,
`arm_package`, `hardware_profile`, `enable_planar_base`, `datums_file` and
`controllers_file`. The consuming implementation must record the selected
values and use the owning source for their meanings. This list does not
choose a new configuration or authorize a different runtime profile.

Description Mode may load an explicitly labelled reference model without a
robot. Live Mode must identify the reported model separately. A reference
checkout, source default or the apparent geometry is not evidence of the
running configuration. If a runtime identity producer is unavailable, display
unknown; agree its interface with the owner before adding a caller.

## Lift limits and display units

- Read `lift_joint` type, axis and lower/upper bounds from the selected expanded
  URDF. Do not hard-code a stroke in the UI or use shoulder-height stops as
  motion limits.
- Require finite, ordered bounds for the bounded prismatic display. If the
  model omits valid bounds, show them as unavailable rather than inventing
  defaults. Continuous rotational joints have different limit semantics.
- Label prismatic position in metres/mm and rotational position in
  radians/degrees. Do not apply angular formatting to a lift.
- Preserve a reported out-of-range value and mark it diagnostically; a display
  must not rewrite telemetry or become actuator-limit enforcement.
- Named lift stops are configuration-linked calibration annotations. Their
  absence does not prevent basic model display, and proximity to one does not
  establish calibration, readiness or permission to move.

## Resource closure and model-source lifecycle

The model owner supplies the selected source/dependency revisions and the
resource inventory. The UI/backend owner decides how to serve the required
installed package resources; this handoff defines no HTTP endpoint or schema.
Resources include meshes and referenced materials/textures, with their source
and required notices preserved. Do not treat control-plugin filenames as
browser geometry resources.

Extend the existing model-loading path. A platform-only package map does not
establish that upperbody and OpenArm resources resolve. Unknown packages,
path traversal and arbitrary filesystem access must be refused. Missing
resources must be visible; a partly rendered robot is not fully loaded.

On robot/model source changes, discard the previous live model and ignore late
callbacks from the old source. A disconnect must not leave a reference model
presented as live telemetry. Verify late-subscriber and static-transform
behavior against the selected transport/profile rather than assuming it.

## Read-only telemetry is a separate work package

Existing joint positions can support a model display, but they do not prove
controller health, ownership, readiness or calibration. Before extending the
readiness panel, the producer and software owners agree topic/type/version,
frame and units, clock domain, freshness/QoS and missing/invalid meanings.
Distinguish receipt time from source time and preserve partial/stale states.

The inspected UI #55 uses a published model and joint states. The accepted #64
read list does not yet include `/robot_description`; add the exact subscription
and its classification/negative tests with the consuming model change.
Coordinate the separate #62 bridge profile too. This is a proposed consumer
delta, not permission to change the accepted boundary through documentation.

## Controls and cameras: gates, not implied capabilities

Manipulation controls wait for the accepted/integrated
[I1 interfaces #36](https://github.com/openAMRobot/openamrobot-interfaces/pull/36)
and defined runtime controller ownership. Wire definitions alone do not supply
a running planning/execution server, ownership enforcement or a gripper
implementation. The UI uses the common manipulation layer, never vendor SDKs
or direct controller commands. Backend owners retain command arbitration and
base/manipulation exclusion; no enforcement logic is proposed here.

Camera-only views can reuse the existing UI component after agreeing the
stream identity, source, transport, frame, calibration and timestamps. Camera
frames in a model do not establish image publishers. Do not guess head/wrist
topic names from dataset roles or a camera label. Preserve explicit unknown
and unavailable states. HTTP video transport is a separate access boundary
from the ROS bridge; authentication work remains
[UI #71](https://github.com/openAMRobot/openamrobot-ui/issues/71).

Perception overlays wait for an accepted producer contract and valid
image/result time association, TF and calibration. Simulator ground truth is
labelled separately from perception. Object selection proposes a target for
approved planning/preview/execution; it never directly commands an actuator.

## Proposed downstream acceptance evidence

| Control | Required observation |
|---|---|
| Two fixtures with different lift bounds | Each uses its own URDF values; reverting to a fixed range fails. |
| Missing/non-finite/reversed lift bounds | Explicit unavailable bounds; no fallback configuration invented. |
| Reported position outside bounds | Raw value retained and flagged; no telemetry clamp. |
| Missing mesh/material/texture | Visible resource failure; no false fully-loaded state. |
| Robot A to B, then late A callback | A cannot overwrite B's model or state. |
| Left-arm-only update | Right wrist remains unchanged in the visual model. |
| Read-only model page exercised | No command publication, service/action request or parameter mutation. |
| New allowlist permission reverted | Consumer test fails rather than leaving a silent blank view. |
| Missing readiness producer | Unknown, never inferred ready from joint movement. |
| Wrong image/result frame or time | Overlay unavailable/stale; no execution-ready target. |

These are planned acceptance criteria, not test results. Attach exact source
and configuration revisions, nonzero test counts, JUnit, resource errors and
browser/probe summaries when implementation is ready. Mock fixtures, simulation
and status displays are not physical or safety acceptance.

## Owner decisions and sequence

1. UI and model owners choose the remaining #55/#62 delta and model/resource
   ownership. Keep this proposal draft until adopt/adapt/reject is recorded.
2. After the rc.1 tag and the agreed integration start, extend the combined-model
   view first. This document does not start that implementation early.
3. Add read-only telemetry only after agreeing its producers and semantics.
4. Add camera-only views against agreed streams; overlays remain gated.
5. Add manipulation controls after I1 integration and runtime ownership gates.

No new runtime endpoint, configuration ID, readiness threshold, contract schema,
release pin or hardware-control behavior is introduced by this handoff.
