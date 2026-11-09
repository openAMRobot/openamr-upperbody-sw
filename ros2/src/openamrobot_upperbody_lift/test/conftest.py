# SPDX-License-Identifier: MIT
"""Shared helpers. Node tests run in their own ROS_DOMAIN_ID, localhost only, so they cannot see another run's graph."""
import os
import random

os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["ROS_DOMAIN_ID"] = str(random.randint(101, 232))

from openamrobot_upperbody_lift.evaluator import Observation  # noqa: E402


class FakeClock:
    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def obs(sequence, position_m=0.175, valid=True, age_ms=100, boot_id=7, version=1, recovery=False, fault=False):
    return Observation(contract_version=version, boot_id=boot_id, sequence=sequence, position_m=position_m, position_valid=valid,
                       status_age_ms=age_ms, recovery_required=recovery, fault_present=fault, lift_state=3)


def settled(evaluator, clock, start=1, count=3, **kwargs):
    """Feed `count` good 5 Hz frames from `start`; returns the next sequence number and the last decision."""
    decision = None
    for i in range(count):
        decision = evaluator.on_status(obs(start + i, **kwargs), clock())
        clock.advance(0.2)
    return start + count, decision
