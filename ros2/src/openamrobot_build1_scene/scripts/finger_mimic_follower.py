#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The owner of openarm_*_finger_joint2 when the physics engine has no mimic constraint (conflict SW-09).

The model gives each second finger a mimic of the first, with multiplier 1 and offset 0. Bullet Featherstone applies
that constraint. DART does not, and then nothing owns the joint. This node gives it one writer: it reads the position
of finger_joint1 from the physics and sends it as the position target of finger_joint2, to the position controller
that urdf/build1_robot.urdf.xacro adds for this case. A servo, not a constraint: the second finger lags the first by
the bridge and controller delay. Simulation only. Not a safety function.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

SOURCE = "/build1/ground_truth/finger_joint_states"


class FingerMimicFollower(Node):
    def __init__(self):
        super().__init__("finger_mimic_follower")
        self.targets = {side: self.create_publisher(Float64, f"/build1/fingers/{side}_finger2_cmd_pos", 10) for side in ("left", "right")}
        self.create_subscription(JointState, SOURCE, self.on_state, 10)

    def on_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        for side, publisher in self.targets.items():
            leader = positions.get(f"openarm_{side}_finger_joint1")
            if leader is not None:
                publisher.publish(Float64(data=float(leader)))


def main():
    rclpy.init()
    node = FingerMimicFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
