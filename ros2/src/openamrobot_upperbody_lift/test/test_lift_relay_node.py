# SPDX-License-Identifier: MIT
"""The relay node against a fake /base/aux_status publisher (own ROS_DOMAIN_ID, localhost). The node runs on an injected clock, so
"stale" is made by moving that clock, not by sleeping."""
import math
import threading
import time

import pytest
import rclpy
from conftest import FakeClock
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from openamrobot_upperbody_lift.relay_node import LiftRelay
from openamrobot_upperbody_lift_msgs.msg import BaseAuxStatusStandIn

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Rig:
    def __init__(self):
        self.clock = FakeClock()
        self.relay = LiftRelay(BaseAuxStatusStandIn, clock=self.clock)
        self.peer = rclpy.create_node("fake_base_and_listener")
        self.pub = self.peer.create_publisher(BaseAuxStatusStandIn, "/base/aux_status", QOS)
        self.joints, self.diagnostics = [], []
        self.peer.create_subscription(JointState, "/joint_states", self.joints.append, 10)
        self.peer.create_subscription(DiagnosticStatus, "/lift_relay/status", self.diagnostics.append, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.relay)
        self.executor.add_node(self.peer)
        self.sequence = 0
        self.spin_until(lambda: self.pub.get_subscription_count() >= 1, 10.0)

    def spin_until(self, predicate, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not predicate():
            self.executor.spin_once(timeout_sec=0.02)
        return predicate()

    def spin_for(self, seconds=0.3):
        self.spin_until(lambda: False, seconds)

    def send(self, position=0.175, valid=True, age_ms=100, boot_id=7, advance=True, recovery=False):
        if advance:
            self.sequence += 1
        msg = BaseAuxStatusStandIn(contract_version=1, boot_id=boot_id, sequence=self.sequence, lift_state=3, lift_position_m=position,
                                   lift_position_valid=valid, lift_status_age_ms=age_ms, lift_active_stop_id=1,
                                   lift_recovery_required=recovery)
        before = len(self.diagnostics)
        self.pub.publish(msg)
        self.spin_until(lambda: len(self.diagnostics) > before, 5.0)
        self.spin_for(0.1)

    def settle(self, **kwargs):
        for _ in range(3):
            self.clock.advance(0.2)
            self.send(**kwargs)

    def close(self):
        self.executor.shutdown()
        self.relay.destroy_node()
        self.peer.destroy_node()


@pytest.fixture
def rig():
    rclpy.init()
    r = Rig()
    yield r
    r.close()
    rclpy.shutdown()


def test_fresh_status_publishes_lift_joint_in_metres(rig):
    rig.settle(position=0.175)
    assert rig.joints, "no lift_joint published"
    last = rig.joints[-1]
    assert list(last.name) == ["lift_joint"] and last.position[0] == pytest.approx(0.175, abs=1e-6)
    assert rig.diagnostics[-1].level == DiagnosticStatus.OK


@pytest.mark.parametrize("metres", [0.0, 0.175, 0.35])
def test_the_three_stops(rig, metres):
    rig.settle(position=metres)
    assert rig.joints[-1].position[0] == pytest.approx(metres, abs=1e-6)


def test_stale_status_stops_publication_and_raises_a_diagnostic(rig):
    rig.settle()
    count = len(rig.joints)
    assert count >= 1
    rig.clock.advance(0.7)                                    # no message for 0.7 s
    before = len(rig.diagnostics)
    assert rig.spin_until(lambda: len(rig.diagnostics) > before, 5.0)      # the timer reports it
    rig.spin_for(0.3)
    assert len(rig.joints) == count, "lift_joint was published while stale"
    status = rig.diagnostics[-1]
    assert status.level == DiagnosticStatus.ERROR and "stale" in status.message
    assert {v.key: v.value for v in status.values}["reason"] == "STALE_LINK"


def test_invalid_position_is_not_published(rig):
    rig.settle()
    count = len(rig.joints)
    rig.clock.advance(0.2)
    rig.send(valid=False, position=0.2)
    assert len(rig.joints) == count
    assert rig.diagnostics[-1].level == DiagnosticStatus.ERROR and "lift_position_valid" in rig.diagnostics[-1].message


def test_nan_position_is_not_published(rig):
    rig.settle()
    count = len(rig.joints)
    rig.clock.advance(0.2)
    rig.send(position=math.nan)
    assert len(rig.joints) == count


def test_old_measurement_in_a_new_message_is_not_published(rig):
    rig.settle()
    count = len(rig.joints)
    rig.clock.advance(0.2)
    rig.send(age_ms=450)
    assert len(rig.joints) == count and "lift_status_age_ms" in rig.diagnostics[-1].message


def test_nothing_is_published_before_any_message(rig):
    rig.clock.advance(1.0)
    rig.spin_for(1.0)
    assert not rig.joints
    assert rig.diagnostics and rig.diagnostics[-1].level == DiagnosticStatus.ERROR


def test_a_replayed_message_does_not_refresh_a_stale_state(rig):
    rig.settle()
    count = len(rig.joints)
    rig.clock.advance(0.7)
    rig.send(advance=False)                                   # same sequence again (for example a transient-local replay)
    assert len(rig.joints) == count


def test_recovery_after_stale_needs_three_frames(rig):
    rig.settle()
    rig.clock.advance(0.7)
    rig.spin_for(0.6)
    count = len(rig.joints)
    rig.send()
    rig.send()
    assert len(rig.joints) == count
    rig.send()
    assert len(rig.joints) == count + 1
