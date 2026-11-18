# SPDX-License-Identifier: MIT
"""Recordings of the grocery dark store demonstrator, next to the mission log in BUILD1_LOG_DIR.

- the Baseline 1 recorders (baseline1_record.Recorders): a ROS bag and a candump of every virtual CAN bus, with more
  topics in the bag (mission events and phase, contacts, object poses, Nav2 status and feedback, gripper actions);
- a store camera for the video: one camera model in the world, moved along a camera path over the store with gz-sim's
  set_pose service (an overview, then a chase view behind the robot while it drives, a close view at the shelf and
  at the bench). Frames at 2 Hz of Gazebo time, 960 x 540, written as PPM with their Gazebo stamp;
- contact events per phase from the gz contact sensors (BUILD1_CONTACT_SENSORS=1).

A recorder never commands the robot, and a recording is never a check: one that cannot start is written to the log as
an event and the mission goes on. The video is a visual record only, never evidence. Simulation only.
"""
import json
import math
import os
import subprocess
import threading
import time

from openamrobot_build1_scene import baseline1_record as b1rec
from openamrobot_build1_scene import diagnostics as dx

EVENTS_TOPIC, PHASE_TOPIC = "/grocery_demo/events", "/grocery_demo/phase"
CAMERA_TOPIC = "/grocery_demo/camera/image"
CAMERA_NAME = "store_camera"
CAMERA_RATE_HZ = 2.0
CAMERA_SIZE = (960, 540)
BAG_TOPICS = b1rec.BAG_TOPICS + (
    EVENTS_TOPIC, PHASE_TOPIC, "/build1/ground_truth/payload/pose", "/build1/planning_scene/objects",
    "/build1/planning_scene/exclude", "/navigate_to_pose/_action/feedback", "/local_costmap/published_footprint",
    "/rosout",
)
BAG_REGEX = b1rec.BAG_REGEX + r"|/build1/contacts/.*"
CAMERA_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <sensor name="{name}" type="camera">
        <always_on>true</always_on>
        <update_rate>{rate}</update_rate>
        <topic>{topic}</topic>
        <camera>
          <horizontal_fov>1.2</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>40</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""

# Camera path (world frame, metres). The overview sees the whole 14 m x 10 m store from beyond the south wall.
OVERVIEW = ((0.0, -8.8, 6.2), (0.6, -0.4, 0.2))
CHASE_BACK, CHASE_UP, CHASE_AHEAD = 2.4, 3.0, 0.8
CLOSE_BACK, CLOSE_SIDE, CLOSE_UP = 0.7, 1.4, 1.9
HAND_AHEAD, HAND_LEFT, HAND_Z = 0.75, 0.16, 0.92
DRIVE_PHASES = ("nav_to_shelf", "retreat_with_item", "nav_to_packing", "retreat", "return_to_parking")
CLOSE_PHASES = ("stop_authorize", "lift_to_target", "arm_approach", "grasp", "lift_object", "place")


def look_pose(eye, target):
    """(x, y, z, qx, qy, qz, qw) of a camera at `eye` whose +x looks at `target` (roll 0; positive pitch looks down)."""
    dx_, dy_, dz_ = (target[i] - eye[i] for i in range(3))
    yaw = math.atan2(dy_, dx_)
    pitch = math.atan2(-dz_, math.hypot(dx_, dy_))
    cp, sp, cy, sy = math.cos(pitch / 2), math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2)
    return (eye[0], eye[1], eye[2], -sp * sy, sp * cy, cp * sy, cp * cy)


def camera_target(phase, robot, finished=False):
    """(eye, look-at) of the camera path for `phase` and the robot pose (x, y, yaw). Pure geometry."""
    if robot is None or finished or phase in (None, "spawn"):
        return OVERVIEW
    x, y, yaw = robot
    hx, hy = math.cos(yaw), math.sin(yaw)
    lx, ly = -hy, hx
    if phase in CLOSE_PHASES:
        hand = (x + HAND_AHEAD * hx + HAND_LEFT * lx, y + HAND_AHEAD * hy + HAND_LEFT * ly, HAND_Z)
        eye = (x - CLOSE_BACK * hx + CLOSE_SIDE * lx, y - CLOSE_BACK * hy + CLOSE_SIDE * ly, CLOSE_UP)
        return eye, hand
    return (x - CHASE_BACK * hx, y - CHASE_BACK * hy, CHASE_UP), (x + CHASE_AHEAD * hx, y + CHASE_AHEAD * hy, 0.5)


class CameraPath:
    """Moves the store camera along the camera path with `gz service .../set_pose`, in its own thread. Smoothed, so a
    phase change or a turn of the robot is a glide, not a jump. Stops trying after five failed calls in a row."""
    PERIOD_WALL_S = 0.4
    SMOOTHING = 0.25

    def __init__(self, rig, tracker, log, world="build1"):
        self.rig, self.tracker, self.log, self.world = rig, tracker, log, world
        self.eye, self.look = OVERVIEW
        self.running, self.finished, self.failures, self.calls, self.ok = True, False, 0, 0, 0
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _robot(self):
        try:
            return self.rig.truth()
        except Exception:  # noqa: BLE001 - no ground truth yet: the overview
            return None

    def _set_pose(self, pose):
        x, y, z, qx, qy, qz, qw = pose
        request = (f'name: "{CAMERA_NAME}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
                   f"orientation: {{x: {qx:.5f}, y: {qy:.5f}, z: {qz:.5f}, w: {qw:.5f}}}")
        try:
            done = subprocess.run(["gz", "service", "-s", f"/world/{self.world}/set_pose", "--reqtype", "gz.msgs.Pose",
                                   "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", request],
                                  capture_output=True, text=True, timeout=10.0, check=False)
            return done.returncode == 0 and "true" in done.stdout
        except (OSError, subprocess.TimeoutExpired):
            return False

    def _loop(self):
        while self.running:
            eye, look = camera_target(self.tracker.current, self._robot(), self.finished)
            a = self.SMOOTHING
            self.eye = tuple(self.eye[i] + a * (eye[i] - self.eye[i]) for i in range(3))
            self.look = tuple(self.look[i] + a * (look[i] - self.look[i]) for i in range(3))
            self.calls += 1
            if self._set_pose(look_pose(self.eye, self.look)):
                self.ok += 1
                self.failures = 0
            else:
                self.failures += 1
                if self.failures >= 5:
                    self.log.event("camera path: set_pose failed five times in a row; the camera stays where it is")
                    return
            time.sleep(self.PERIOD_WALL_S)

    def stop(self):
        self.running = False
        self.thread.join(timeout=15.0)
        self.log.event(f"camera path: {self.ok} of {self.calls} set_pose calls succeeded")


class ContactEvents:
    """Contact messages of the gz contact sensors, aggregated per phase, sensor and touching collision: message count,
    contact points and the largest force magnitude. Needs BUILD1_CONTACT_SENSORS=1 (diagnostics.py); else empty."""

    def __init__(self, rig, tracker):
        self.rig, self.tracker = rig, tracker
        self.lock = threading.Lock()
        self.table = {}
        self.available = False
        try:
            from ros_gz_interfaces.msg import Contacts
        except ImportError:
            return
        links = dx.contact_links() + [dx.world_contact_link(model) for model in dx.WORLD_SENSOR_MODELS]
        for link in links:
            rig.node.create_subscription(Contacts, dx.ros_contact_topic(link), lambda m, l=link: self._on(l, m), 50)
        self.available = True

    def _on(self, sensor, msg):
        phase = self.tracker.current or "between_phases"
        for contact in msg.contacts:
            pair = f"{contact.collision1.name} | {contact.collision2.name}"
            force = 0.0
            for wrench in contact.wrenches:
                f = wrench.body_1_wrench.force
                force = max(force, math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z))
            with self.lock:
                row = self.table.setdefault((phase, sensor, pair), {"messages": 0, "points": 0, "max_force_n": 0.0})
                row["messages"] += 1
                row["points"] += len(contact.positions)
                row["max_force_n"] = max(row["max_force_n"], force)

    def rows(self):
        with self.lock:
            return [{"phase": p, "sensor": s, "pair": c, **v} for (p, s, c), v in sorted(self.table.items())]


class Recorders(b1rec.Recorders):
    """Baseline 1's recorders with this demonstrator's bag topics and its store camera."""

    def start(self, rig):
        os.makedirs(self.directory, exist_ok=True)
        bag = os.path.join(self.directory, "bag")
        import shutil
        shutil.rmtree(bag, ignore_errors=True)
        self._start("bag", ["ros2", "bag", "record", "--use-sim-time", "-o", bag, "--topics", *BAG_TOPICS, "--regex", BAG_REGEX])
        for bus in b1rec.VCAN:
            self._start(f"candump_{bus}", ["candump", "-L", bus])
        if os.environ.get("GROCERY_VIDEO", "1") != "0":
            self._camera()

    def _camera(self):
        sdf = CAMERA_SDF.format(name=CAMERA_NAME, rate=CAMERA_RATE_HZ, topic=CAMERA_TOPIC, w=CAMERA_SIZE[0], h=CAMERA_SIZE[1])
        x, y, z, qx, qy, qz, qw = look_pose(*OVERVIEW)
        roll, pitch, yaw = 0.0, math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))), math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        spawn = subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", "build1", "-name", CAMERA_NAME, "-string", sdf,
                                "-x", str(x), "-y", str(y), "-z", str(z), "-R", str(roll), "-P", str(pitch), "-Y", str(yaw)],
                               capture_output=True, text=True, timeout=120, check=False)
        self.log.event(f"recording video: store camera spawn exit {spawn.returncode}")
        if spawn.returncode != 0:
            return
        self._start("camera_bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                                      f"{CAMERA_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image", "--ros-args", "-p", "use_sim_time:=true"])
        self.frames = os.path.join(self.directory, "frames")
        os.makedirs(self.frames, exist_ok=True)

    def subscribe_frames(self, rig):
        """As Baseline 1: each image a PPM named by its Gazebo stamp; here from the store camera topic."""
        if self.frames is None:
            return
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        directory = self.frames

        def on_image(msg):
            if msg.encoding not in ("rgb8", "bgr8"):
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            rows = bytes(msg.data)
            if msg.step != msg.width * 3:
                rows = b"".join(rows[r * msg.step:r * msg.step + msg.width * 3] for r in range(msg.height))
            if msg.encoding == "bgr8":
                rows = bytes(b for i in range(0, len(rows), 3) for b in (rows[i + 2], rows[i + 1], rows[i]))
            with open(os.path.join(directory, f"{stamp:010.3f}.ppm"), "wb") as f:
                f.write(f"P6 {msg.width} {msg.height} 255\n".encode() + rows)
        rig.node.create_subscription(Image, CAMERA_TOPIC, on_image, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))


def write_json(directory, name, data):
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, name)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=1, sort_keys=True, default=str)
    return path
