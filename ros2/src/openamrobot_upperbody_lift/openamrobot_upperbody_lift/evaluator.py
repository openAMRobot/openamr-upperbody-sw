# SPDX-License-Identifier: MIT
"""Decide, from /base/aux_status observations alone, whether lift_joint may be published. No ROS here: unit-tested without a runtime.

The rule (issue #45, I8 rev 3, CAN3 lift proposal section 9): the lift state is usable only when ALL of these hold, and otherwise nothing is
published. There is never a held, default or extrapolated value.

  link fresh          a status message arrived within `stale_after_s` (0.6 s: /base/aux_status is a 5 Hz heartbeat, stale after 0.6 s, measured
                      from LOCAL receipt time; the header stamp is diagnostic only)
  progressing         (boot_id, sequence) moves forward. A repeated or backward sequence does not refresh anything, so a transient-local replay
                      of the last sample, or a frozen publisher, cannot keep the state alive
  resynchronised      after start-up, a boot_id change, a sequence jump of more than `max_sequence_gap`, any staleness or any failed check,
                      `resync_frames` (3, decision D8) consecutive progressing messages that pass every check are needed before it is published
  measurement fresh   lift_status_age_ms in the message is at most `max_lift_status_age_ms` (CAN3 status freshness, decision D15: 300 ms).
                      A new message can carry an old lift measurement
  position valid      lift_position_valid is true and recovery_required is false (proposal 9.3), and the position is finite and inside the
                      stroke by at most `range_tolerance_m`
  contract            contract_version is one the host knows

Position units: metres from the L1000 stop, 0 to 0.350 (P-03 item 17, matches the URDF). `position_offset_m` is added to the reported value;
it is 0.0 until Evans and the fixture confirm the calibration offset (TODO(Evans)).
"""
import enum
import math
from dataclasses import dataclass

STROKE_M = 0.350                       # lift_joint upper limit in the URDF (DOLD V4 350 mm stroke)
STALE_AFTER_S = 0.6                    # /base/aux_status: 5 Hz heartbeat, stale after 0.6 s (I8 rev 3, frozen 6 Nov)
MAX_LIFT_STATUS_AGE_MS = 300           # CAN3 proposal 9.1 / decision D15; TODO(Evans) confirm for the relayed block
RESYNC_FRAMES = 3                      # decision D8 / D15: three fresh progressing frames
MAX_SEQUENCE_GAP = 3                   # D15 uses 3 for the CAN status counter; same starting value here, TODO(Evans) for the aux_status sequence
RANGE_TOLERANCE_M = 0.002              # stop tolerance of decision D3 (plus or minus 2 mm); TODO(Evans) confirm
NO_FRAME_AGE_MS = 0xFFFFFFFF           # lift_status_age_ms: no LiftStatus frame received since boot
ACCEPTED_CONTRACT_VERSIONS = (1,)      # I8 rev 3: uint16 starting at 1; TODO(Evans) confirm with the published package
SEQUENCE_MODULUS = 1 << 32
LIFT_STATE_FAULT = 6                   # CAN3 proposal 5.3: 0 INIT, 1 UNHOMED, 2 HOMING, 3 IDLE, 4 MOVING, 5 STOPPING, 6 FAULT


class Reason(enum.Enum):
    OK = "ok"
    NO_MESSAGE = "no /base/aux_status message received"
    STALE_LINK = "/base/aux_status is stale"
    STALE_MEASUREMENT = "lift measurement is stale (lift_status_age_ms)"
    NOT_PROGRESSING = "/base/aux_status sequence is not progressing"
    RESYNC = "re-establishing progression"
    RESTARTED = "base controller restarted (boot_id changed)"
    POSITION_INVALID = "lift_position_valid is false"
    RECOVERY_REQUIRED = "lift recovery is required"
    NOT_FINITE = "lift position is not a finite number"
    OUT_OF_RANGE = "lift position is outside the stroke"
    CONTRACT = "unknown contract_version"


@dataclass(frozen=True)
class Observation:
    """The lift block of one /base/aux_status message, in the fields the evaluator needs. Built by relay_node.observation_from_msg."""
    contract_version: int
    boot_id: int
    sequence: int
    position_m: float
    position_valid: bool
    status_age_ms: int
    recovery_required: bool = False
    fault_present: bool = False
    lift_state: int = 0


@dataclass(frozen=True)
class Decision:
    publish: bool
    reason: Reason
    position_m: float = math.nan
    detail: str = ""

    @property
    def text(self):
        return self.reason.value + (f": {self.detail}" if self.detail else "")


def forward_distance(previous, current):
    """How far `current` is ahead of `previous` modulo 2**32, in 1 .. 2**31 - 1 if it is ahead, 0 if equal, None if it is behind."""
    delta = (current - previous) % SEQUENCE_MODULUS
    if delta == 0:
        return 0
    return delta if delta < SEQUENCE_MODULUS // 2 else None


class LiftEvaluator:
    def __init__(self, stale_after_s=STALE_AFTER_S, max_lift_status_age_ms=MAX_LIFT_STATUS_AGE_MS, resync_frames=RESYNC_FRAMES,
                 range_tolerance_m=RANGE_TOLERANCE_M, position_offset_m=0.0, stroke_m=STROKE_M,
                 accepted_contract_versions=ACCEPTED_CONTRACT_VERSIONS, max_sequence_gap=MAX_SEQUENCE_GAP):
        self.stale_after_s = stale_after_s
        self.max_lift_status_age_ms = max_lift_status_age_ms
        self.resync_frames = max(1, int(resync_frames))
        self.range_tolerance_m = range_tolerance_m
        self.position_offset_m = position_offset_m
        self.stroke_m = stroke_m
        self.max_sequence_gap = max_sequence_gap
        self.accepted_contract_versions = tuple(accepted_contract_versions)
        self.last_receipt = None           # local time of the last message that PROGRESSED (not a replay)
        self.boot_id = None
        self.sequence = None
        self.rebaseline = False
        self.run = 0                       # consecutive progressing messages that passed every check
        self.last = Decision(False, Reason.NO_MESSAGE)
        self.clamped = 0                   # readings within the tolerance outside the stroke, published at the limit

    # -- one message ---------------------------------------------------------------------------------------------------------------
    def on_status(self, obs, now):
        """Evaluate one message received at local time `now` (seconds, monotonic). Returns the Decision; publish only if `.publish`."""
        decision = self._evaluate(obs, now)
        self.last = decision
        return decision

    def _evaluate(self, obs, now):
        if self.last_receipt is not None and now - self.last_receipt > self.stale_after_s:
            self._lose_progression()                     # a gap longer than the stale time: trust nothing until progression is re-established
        if self.boot_id is not None and obs.boot_id != self.boot_id:
            self._lose_progression()
            self.rebaseline = False
            self.boot_id, self.sequence, self.last_receipt = obs.boot_id, obs.sequence, now
            return Decision(False, Reason.RESTARTED, detail=f"boot_id {obs.boot_id}")
        if self.sequence is None:
            self.boot_id, self.sequence, self.last_receipt = obs.boot_id, obs.sequence, now
        else:
            ahead = forward_distance(self.sequence, obs.sequence)
            if self.rebaseline and obs.sequence != self.sequence:
                ahead, self.rebaseline = 1, False        # after a stale gap any NEW sequence starts again; the same one is still a replay
            if not ahead:                                # repeated or behind: not a fresh frame, and it must not refresh the link
                self.run = 0
                return Decision(False, Reason.NOT_PROGRESSING, detail=f"sequence {obs.sequence} after {self.sequence}")
            self.sequence, self.last_receipt = obs.sequence, now
            if ahead > self.max_sequence_gap:            # a jump: frames were lost or the publisher restarted; progression starts again
                self.run = 0
        problem = self._problem(obs)
        if problem is not None:
            self.run = 0
            return problem
        position = obs.position_m + self.position_offset_m
        self.run += 1
        if self.run < self.resync_frames:
            return Decision(False, Reason.RESYNC, detail=f"{self.run} of {self.resync_frames} good frames")
        if position < 0.0 or position > self.stroke_m:   # inside the stop tolerance: the URDF limit is a hard bound for MoveIt's start state
            self.clamped += 1
            position = min(max(position, 0.0), self.stroke_m)
        return Decision(True, Reason.OK, position_m=position)

    def _problem(self, obs):
        """The first reason this one message cannot be trusted, or None."""
        if obs.contract_version not in self.accepted_contract_versions:
            return Decision(False, Reason.CONTRACT, detail=f"{obs.contract_version}, accepted {list(self.accepted_contract_versions)}")
        if obs.status_age_ms == NO_FRAME_AGE_MS or obs.status_age_ms > self.max_lift_status_age_ms:
            age = "no LiftStatus frame since boot" if obs.status_age_ms == NO_FRAME_AGE_MS else f"{obs.status_age_ms} ms"
            return Decision(False, Reason.STALE_MEASUREMENT, detail=f"{age}, limit {self.max_lift_status_age_ms} ms")
        if obs.recovery_required:
            return Decision(False, Reason.RECOVERY_REQUIRED)
        if not obs.position_valid:
            return Decision(False, Reason.POSITION_INVALID)
        if not math.isfinite(obs.position_m):
            return Decision(False, Reason.NOT_FINITE)
        position = obs.position_m + self.position_offset_m
        if position < -self.range_tolerance_m or position > self.stroke_m + self.range_tolerance_m:
            return Decision(False, Reason.OUT_OF_RANGE, detail=f"{position:.4f} m, stroke 0 to {self.stroke_m:g} m")
        return None

    def _lose_progression(self):
        """Staleness: the last sequence is KEPT, so a replay of it (a frozen publisher, a transient-local re-delivery) is still a replay."""
        self.rebaseline = self.sequence is not None
        self.run = 0

    # -- time passing without a message --------------------------------------------------------------------------------------------------
    def poll(self, now):
        """The state now, when no message has just arrived. Stale (or no message yet) is never publishable."""
        if self.last_receipt is None:
            return Decision(False, Reason.NO_MESSAGE)
        age = now - self.last_receipt
        if age > self.stale_after_s:
            self._lose_progression()
            self.last = Decision(False, Reason.STALE_LINK, detail=f"{age:.2f} s since the last message, limit {self.stale_after_s:g} s")
        return self.last
