# SPDX-License-Identifier: MIT
"""The relay's decision, no ROS: fresh -> published in metres; stale, invalid, replayed, restarted, out of range -> not published, with a reason."""
import math

import pytest
from conftest import FakeClock, obs, settled

from openamrobot_upperbody_lift.evaluator import LiftEvaluator, Reason, forward_distance


@pytest.fixture
def clock():
    return FakeClock()


def test_fresh_valid_status_is_published_in_metres_after_resync(clock):
    ev = LiftEvaluator()
    first = ev.on_status(obs(1, 0.175), clock())
    assert not first.publish and first.reason is Reason.RESYNC          # decision D8: three frames before the position is trusted
    clock.advance(0.2)
    assert not ev.on_status(obs(2, 0.175), clock()).publish
    clock.advance(0.2)
    third = ev.on_status(obs(3, 0.175), clock())
    assert third.publish and third.reason is Reason.OK and third.position_m == pytest.approx(0.175)


@pytest.mark.parametrize("metres", [0.0, 0.175, 0.350])      # L1000, L1175, L1350: the calibration stops, in metres from L1000
def test_the_three_stops_are_published_unchanged(clock, metres):
    ev = LiftEvaluator()
    _, decision = settled(ev, clock, position_m=metres)
    assert decision.publish and decision.position_m == pytest.approx(metres, abs=1e-9)


def test_a_position_offset_is_added(clock):
    ev = LiftEvaluator(position_offset_m=0.01)
    _, decision = settled(ev, clock, position_m=0.1)
    assert decision.position_m == pytest.approx(0.11)


def test_stale_link_is_not_published_and_nothing_is_held(clock):
    ev = LiftEvaluator()
    sequence, decision = settled(ev, clock)
    assert decision.publish
    clock.advance(0.5)                                  # 0.7 s since the last message in total: over 0.6 s
    polled = ev.poll(clock())
    assert not polled.publish and polled.reason is Reason.STALE_LINK
    assert math.isnan(polled.position_m)                # no held value
    assert not ev.poll(clock()).publish                 # and it stays that way


def test_just_under_the_stale_time_is_still_fresh(clock):
    ev = LiftEvaluator()
    _, decision = settled(ev, clock)
    assert decision.publish
    clock.advance(0.35)                                 # 0.2 (settled's last advance) + 0.35 = 0.55 s
    assert ev.poll(clock()).publish


def test_after_a_stale_gap_three_good_frames_are_needed_again(clock):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    clock.advance(1.0)
    ev.poll(clock())
    decisions = []
    for i in range(3):
        decisions.append(ev.on_status(obs(sequence + i), clock()))
        clock.advance(0.2)
    assert [d.publish for d in decisions] == [False, False, True]


def test_a_late_message_after_a_gap_is_not_published_at_once(clock):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    clock.advance(2.0)                                  # no poll in between: the gap is detected on arrival
    decision = ev.on_status(obs(sequence), clock())
    assert not decision.publish and decision.reason is Reason.RESYNC


def test_invalid_position_is_not_published(clock):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    decision = ev.on_status(obs(sequence, valid=False), clock())
    assert not decision.publish and decision.reason is Reason.POSITION_INVALID


def test_recovery_required_is_not_published_even_if_valid(clock):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    decision = ev.on_status(obs(sequence, recovery=True), clock())
    assert not decision.publish and decision.reason is Reason.RECOVERY_REQUIRED


def test_after_invalid_the_position_returns_only_after_three_good_frames(clock):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    assert not ev.on_status(obs(sequence, valid=False), clock()).publish
    results = [ev.on_status(obs(sequence + 1 + i), clock()).publish for i in range(3)]
    assert results == [False, False, True]


def test_an_old_lift_measurement_in_a_new_message_is_not_published(clock):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    decision = ev.on_status(obs(sequence, age_ms=301), clock())
    assert not decision.publish and decision.reason is Reason.STALE_MEASUREMENT
    assert ev.on_status(obs(sequence + 1, age_ms=0xFFFFFFFF), clock()).reason is Reason.STALE_MEASUREMENT      # no frame since boot


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_non_finite_position_is_not_published(clock, value):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    assert ev.on_status(obs(sequence, position_m=value), clock()).reason is Reason.NOT_FINITE


@pytest.mark.parametrize("value", [-0.05, 0.36, 1.0])
def test_a_position_outside_the_stroke_is_not_published(clock, value):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    assert ev.on_status(obs(sequence, position_m=value), clock()).reason is Reason.OUT_OF_RANGE


@pytest.mark.parametrize("value,expected", [(-0.0015, 0.0), (0.3515, 0.350)])
def test_a_reading_within_the_stop_tolerance_is_clamped_to_the_urdf_limit(clock, value, expected):
    ev = LiftEvaluator()
    _, decision = settled(ev, clock, position_m=value)
    assert decision.publish and decision.position_m == expected and ev.clamped == 1


def test_a_replayed_or_repeated_sequence_does_not_keep_the_state_alive(clock):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    last = sequence - 1
    for _ in range(5):                                  # a frozen publisher: the same sequence, five times, one second apart
        clock.advance(0.2)
        decision = ev.on_status(obs(last), clock())
        assert not decision.publish and decision.reason is Reason.NOT_PROGRESSING
    assert not ev.poll(clock()).publish                 # still stale by time: replays did not refresh the receipt time


def test_a_step_back_is_not_progress(clock):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock, start=100)
    assert ev.on_status(obs(sequence - 5), clock()).reason is Reason.NOT_PROGRESSING


def test_a_base_controller_restart_blocks_until_progression_is_re_established(clock):
    ev = LiftEvaluator()
    settled(ev, clock)
    first = ev.on_status(obs(1, boot_id=8), clock())
    assert not first.publish and first.reason is Reason.RESTARTED
    results = []
    for i in range(2, 5):
        clock.advance(0.2)
        results.append(ev.on_status(obs(i, boot_id=8), clock()).publish)
    assert results == [False, False, True]


def test_a_sequence_jump_over_the_gap_limit_needs_resync(clock):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    decision = ev.on_status(obs(sequence + 50), clock())
    assert not decision.publish and decision.reason is Reason.RESYNC


def test_a_lost_frame_inside_the_gap_limit_does_not_break_progression(clock):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    assert ev.on_status(obs(sequence + 2), clock()).publish                  # one message lost, counted as progression


def test_sequence_wraps_at_32_bits(clock):
    ev = LiftEvaluator()
    _, decision = settled(ev, clock, start=(1 << 32) - 2)
    assert decision.publish
    assert forward_distance((1 << 32) - 1, 0) == 1 and forward_distance(5, 4) is None and forward_distance(5, 5) == 0


def test_an_unknown_contract_version_is_not_published(clock):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    assert ev.on_status(obs(sequence, version=2), clock()).reason is Reason.CONTRACT


def test_no_message_yet_is_not_published(clock):
    ev = LiftEvaluator()
    decision = ev.poll(clock())
    assert not decision.publish and decision.reason is Reason.NO_MESSAGE
    assert "no /base/aux_status" in decision.text


def test_every_refusal_has_a_reason_text(clock):
    ev = LiftEvaluator()
    sequence, _ = settled(ev, clock)
    for decision in (ev.on_status(obs(sequence, valid=False), clock()), ev.on_status(obs(sequence + 1, age_ms=999), clock())):
        assert decision.text and decision.reason is not Reason.OK
