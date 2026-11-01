# SPDX-License-Identifier: MIT
"""Wall-bounded simulation-time waits. Pure Python so a frozen /clock is testable."""
import math
import time


def spin_sim_seconds(seconds, *, sim_now, spin_once, wall_deadline, on_tick=None,
                     frozen_after_s=5.0, wall_now=time.monotonic):
    """Wait the entire simulated duration, or fail explicitly before the outer test timeout.

    Neither slow progress nor a frozen clock can extend the absolute wall deadline.
    The frozen-clock bound is a harness watchdog, not a robot acceptance tolerance.
    """
    started = last_progress = wall_now()
    start = previous = sim_now()
    while True:
        wall, sim = wall_now(), sim_now()
        if not math.isfinite(sim) or sim < previous:
            raise AssertionError(f"simulation clock is invalid or moved backwards: {previous} -> {sim}")
        if sim > previous:
            last_progress = wall
        previous = sim
        elapsed, advanced = wall - started, sim - start
        diagnostic = (f"requested={seconds:.3f} sim s, advanced={advanced:.6f} sim s, "
                      f"wall={elapsed:.3f} s, RTF={advanced / elapsed if elapsed else 0:.6f}")
        if wall >= wall_deadline:
            raise AssertionError(f"simulation wait exhausted the G01 wall-clock budget: {diagnostic}")
        if advanced >= seconds:
            return
        if wall - last_progress >= frozen_after_s:
            raise AssertionError(f"simulation clock frozen for {wall - last_progress:.3f} wall s: {diagnostic}")
        if on_tick is not None:
            on_tick()
        spin_once()
