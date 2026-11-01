# SPDX-License-Identifier: MIT
"""The controller wait honours ONE deadline (PR #14 review, Mohamed). No ROS graph: a fake clock, a fake client and a fake spin.

The old wait gave every request a fresh 2 s and did not check the overall deadline before returning success, so with `timeout_s=1.0` an
all-active answer that arrived at 1.5 s returned `(True, ..., 1.5)` and M02 sent poses after its readiness deadline. Pinned here:
  - an all-active answer that arrives just after the deadline is a False;
  - one that arrives before the deadline is still a True;
  - no wait (service discovery, request, spin, sleep) runs past the deadline: the fake clock never passes it by more than one spin step.
"""
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import a7_support  # noqa: E402

NAMES = a7_support.ARM_CONTROLLERS


class FakeClock:
    """time.monotonic / time.sleep for a7_support: sleeping advances the clock, nothing really waits."""

    def __init__(self):
        self.now = 1000.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        assert seconds >= 0
        self.now += seconds


class FakeFuture:
    def __init__(self, clock, ready_at, result):
        self.clock, self.ready_at, self._result = clock, ready_at, result

    def done(self):
        return self.clock.now >= self.ready_at

    def result(self):
        return self._result if self.done() else None


class FakeClient:
    """Answers every request after `latency_s`; the controllers are all active from `active_from` (clock time) on."""

    def __init__(self, clock, latency_s, active_from):
        self.clock, self.latency_s, self.active_from = clock, latency_s, active_from

    def wait_for_service(self, timeout_sec):
        assert timeout_sec >= 0
        return True

    def call_async(self, request):
        ready_at = self.clock.now + self.latency_s
        state = "active" if ready_at >= self.active_from else "inactive"
        controllers = [types.SimpleNamespace(name=n, state=state) for n in NAMES]
        return FakeFuture(self.clock, ready_at, types.SimpleNamespace(controller=controllers))


class FakeNode:
    def __init__(self, client):
        self.client = client

    def create_client(self, srv_type, name):
        return self.client

    def destroy_client(self, client):
        pass


@pytest.fixture
def clock(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(a7_support, "time", types.SimpleNamespace(monotonic=clock.monotonic, sleep=clock.sleep))
    import rclpy
    monkeypatch.setattr(rclpy, "spin_once", lambda node, timeout_sec=None: clock.sleep(timeout_sec if timeout_sec is not None else 0.05))
    return clock


def wait(clock, timeout_s, latency_s, active_after_s):
    started = clock.now
    client = FakeClient(clock, latency_s=latency_s, active_from=started + active_after_s)
    ok, states, waited = a7_support.wait_for_controllers_active(FakeNode(client), timeout_s=timeout_s)
    return ok, states, waited, clock.now - started


def test_all_active_just_after_the_deadline_is_not_ok(clock):
    """Mohamed's reproduction: timeout 1.0 s, the first all-active answer at 1.5 s. The old code returned (True, ..., 1.5)."""
    ok, states, waited, elapsed = wait(clock, timeout_s=1.0, latency_s=1.5, active_after_s=1.5)
    assert not ok, f"ok after the deadline: waited {waited} s of a 1.0 s budget"


def test_all_active_just_after_the_deadline_at_the_real_budget_is_not_ok(clock):
    ok, _, waited, _ = wait(clock, timeout_s=a7_support.CONTROLLERS_ACTIVE_WITHIN_S, latency_s=0.3,
                            active_after_s=a7_support.CONTROLLERS_ACTIVE_WITHIN_S + 0.1)
    assert not ok, f"ok after the {a7_support.CONTROLLERS_ACTIVE_WITHIN_S} s deadline (waited {waited} s)"


def test_all_active_before_the_deadline_is_ok(clock):
    ok, states, waited, _ = wait(clock, timeout_s=1.0, latency_s=0.2, active_after_s=0.6)
    assert ok and set(states) == set(NAMES) and waited <= 1.0


def test_no_wait_runs_past_the_deadline(clock):
    _, _, _, elapsed = wait(clock, timeout_s=1.0, latency_s=5.0, active_after_s=0.0)
    assert elapsed <= 1.0 + 0.05 + 1e-9, f"the wait ran {elapsed:.2f} s on a 1.0 s budget"
