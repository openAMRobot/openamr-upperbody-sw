#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run against a live `ros2 launch openamrobot_upperbody_bringup robot_sim.launch.py` (same ROS_DOMAIN_ID and GZ_PARTITION) and print the
measured numbers as JSON: nodes, publisher counts, controller states, /clock rate, real-time factor, 1 m drive and 90 deg turn against
ground truth, the lift through L1000 / L1175 / L1350 and both arms to `raised` and back to `park`."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_smoke_support as s  # noqa: E402


def main():
    m = s.Smoke()
    out = {"controllers": m.controller_states()}
    m.wait_until(lambda: len(m.odom) > 5 and len(m.clock_stamps) > 5, 60)
    out["nodes"], topics = m.graph()
    out["publishers"] = dict(sorted(topics.items()))
    out["clock_msgs_per_wall_s"], out["clock_sim_per_wall_s"] = m.clock_rate()
    out["real_time_factor_gz_stats"] = s.real_time_factor()
    out["drive"] = m.drive_forward()
    out["rotate"] = m.rotate()
    out["lift"] = m.lift_through_stops()
    out["arms"] = m.arms_to("raised") + m.arms_to("park")
    print(json.dumps(out, indent=1, default=str))


if __name__ == "__main__":
    main()
