# SPDX-License-Identifier: MIT
"""The rig commands the lift only on a permit the CTL-017 emulator holds after the grant (run 44364910928).

Recorded vcan_aux of the run (job 135016191561, wall time):
    1795025930.633018 410 00 1126 ac03   permit flag 0, seq 940 (stop_authorize withdraws the permit)
    1795025930.892456 210 02 01 01       command seq 2, move to stop 1 (lift_move)
    1795025930.892738 190 ... 02 80 ..   last_seq 2, result 80h: refused, no permit
    1795025930.922848 410 01 1126 ad03   permit flag 1, seq 941 (the new permit, 30 ms late)
then 60 s of keep-alives with seq 2, result 80h, the lift at stop 0, no fault bits. The rig had read permit_fresh from
the state it had cached before the withdrawal. The test plays that order against Rig.lift_move with a stand-in of
the rig's topics; no ROS graph runs. The permit gate itself is not modelled beyond refusing without a valid permit.
"""
from openamrobot_build1_scene.rig import Rig   # rclpy: the static tests run with ROS 2 Jazzy sourced
EMULATOR = "/sil/lift/emulator_state"


class Log:
    def __init__(self):
        self.events = []

    def event(self, text):
        self.events.append(text)

    def quantity(self, *args, **kwargs):
        pass


class RecordedOrder:
    """The topics lift_move reads, in the order of the run: a cached state from before the withdrawal, then one state
    published before the stand-in sent the new permit, then states with the new permit."""

    def __init__(self):
        self.devices = True
        self.log = Log()
        self.valid = False                     # the emulator saw the flag-0 frame: no valid permit
        self.last = {EMULATOR: {"permit_fresh": True}}   # cached before the withdrawal
        self.counts = {EMULATOR: 10}
        self.pending = []
        self.commands = []

    def lift_permit(self, on):
        if on:
            self.pending = [False, True, True, True, True, True]   # fresh as the next states will report it

    def count(self, topic):
        return self.counts[topic]

    def get(self, topic):
        return self.last[topic]

    def _publish(self):
        if self.pending:
            fresh = self.pending.pop(0)
            self.valid = fresh
            self.last[EMULATOR] = {"permit_fresh": fresh}
            self.counts[EMULATOR] += 1

    def wait(self, condition, sim_s, what="", step=0.02):
        for _ in range(int(sim_s / 0.05)):     # one emulator state per 0.05 s of Gazebo time
            if condition():
                return True
            self._publish()
        return bool(condition())

    def __getattr__(self, name):
        """Every other rig method is the real one, run on this stand-in."""
        return getattr(Rig, name).__get__(self)

    def lift_go(self, stop, sim_s):
        self.commands.append((stop, self.valid))
        return self.valid


def test_the_lift_command_waits_for_the_permit_given_after_the_grant():
    rig = RecordedOrder()
    Rig.lift_move(rig, 1, 60.0)
    assert rig.commands == [(1, True)], "the command went out without a valid permit (run 44364910928)"


def test_no_command_when_the_permit_never_becomes_fresh():
    rig = RecordedOrder()
    rig.lift_permit = lambda on: setattr(rig, "pending", [False] * 100 if on else [])
    assert Rig.lift_move(rig, 1, 60.0) is False
    assert rig.commands == [] and any("did not become fresh" in e for e in rig.log.events)


def test_a_stale_fresh_state_already_on_its_way_at_the_grant_is_not_taken():
    """The first state after the call can be one the emulator published before the grant, while it still held the old
    permit (review of #94). Taking it would command the lift on a permit that is withdrawn a moment later; the wait
    needs the second state. revert of the +2 rule to +1 or to the cached state: the command goes out invalid."""
    rig = RecordedOrder()
    rig.lift_permit = lambda on: setattr(rig, "pending", [True, False, False, True, True] if on else [])
    original_publish = rig._publish

    def publish():
        stale = rig.pending[:1] == [True] and rig.counts[EMULATOR] == 10
        original_publish()
        if stale:
            rig.valid = False              # in flight from before the withdrawal: the emulator holds no valid permit
    rig._publish = publish
    Rig.lift_move(rig, 1, 60.0)
    assert rig.commands == [(1, True)]


def test_the_permit_is_withdrawn_when_the_move_ends_raises_or_fails():
    """Cancellation and recovery: lift_move gives the permit back in every outcome, so no later step runs on it."""
    for outcome in ("arrived", "refused", "raised"):
        rig = RecordedOrder()
        grants = []
        rig.lift_permit = lambda on, rig=rig, grants=grants: (grants.append(on), setattr(
            rig, "pending", [False, True, True, True] if on else []))

        def lift_go(stop, sim_s, rig=rig, outcome=outcome):
            rig.commands.append((stop, rig.valid))
            if outcome == "raised":
                raise RuntimeError("cancelled")
            return outcome == "arrived"
        rig.lift_go = lift_go
        try:
            result = Rig.lift_move(rig, 1, 60.0)
        except RuntimeError:
            result = "raised"
        assert grants == [True, False], (outcome, grants)
        assert rig.commands == [(1, True)], outcome
        assert result == {"arrived": True, "refused": False, "raised": "raised"}[outcome]


# ------------------------------------------------------------------ the same wait elsewhere (review of #94, 20 Nov)
class StaleState(RecordedOrder):
    """A permit change whose first state after the call is one in flight from before it."""

    def __init__(self, cached, sequence):
        super().__init__()
        self.last = {EMULATOR: {"permit_fresh": cached}}
        self.sequence = list(sequence)

    def change(self):
        self.pending = list(self.sequence)


def test_the_withdrawal_wait_reads_only_states_after_the_change():
    """stop_authorize and scn02 withdraw the permit and then check the refusal. A cached or in-flight 'not fresh' from
    before must not end the wait: only the second state after the call counts."""
    rig = StaleState(cached=False, sequence=[False, True, False, False])
    rig.change()
    seen = []
    original = rig._publish
    rig._publish = lambda: (original(), seen.append(rig.counts[EMULATOR]))
    assert Rig.wait_permit_state(rig, False, 3.0) is True
    assert rig.counts[EMULATOR] >= 10 + 2 and rig.last[EMULATOR]["permit_fresh"] is False
    assert len(seen) >= 3, "returned on the cached or first state after the call"


def test_the_fresh_again_wait_reads_only_states_after_the_change():
    """scn05: after permit_silent is cleared, a cached 'fresh' from before the silence is not the new permit."""
    rig = StaleState(cached=True, sequence=[True, False, True, True])
    rig.change()
    assert Rig.wait_permit_state(rig, True, 3.0) is True
    assert rig.counts[EMULATOR] >= 10 + 3, "took the state in flight from before the change"


def test_a_wait_without_new_states_fails():
    rig = StaleState(cached=True, sequence=[])
    assert Rig.wait_permit_state(rig, True, 0.5) is False
