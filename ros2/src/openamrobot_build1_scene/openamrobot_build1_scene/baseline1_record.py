# SPDX-License-Identifier: MIT
"""Baseline 1 recordings next to the mission log, in BUILD1_LOG_DIR: a ROS bag of the run, a candump of every virtual
CAN bus and the frames of one fixed observer camera (encoded to a video by the workflow).

The recorders run as child processes of the mission, so they share its ROS domain and Gazebo partition. A recorder
that cannot start is written to the log as an event and the mission goes on: a recording is a record of the run,
never a check. The video is a visual record only, never evidence.
"""
import os
import shutil
import signal
import subprocess
import time

# Everything the phase table is computed from, plus the command chain, localization, controllers and the device states.
BAG_TOPICS = (
    "/clock", "/tf", "/tf_static", "/odom", "/build1/ground_truth/odom", "/sil/gz/odom", "/amcl_pose",
    "/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed", "/collision_monitor_state", "/plan", "/local_plan",
    "/joint_states", "/sim/joint_states", "/sil/gz/joint_states", "/arm/joint_states",
    "/build1/ground_truth/finger_joint_states", "/build1/ground_truth/item/pose", "/build1/ground_truth/tote/pose",
    "/sil/drive/status", "/sil/drive/emulator_state", "/sil/lift/status", "/sil/lift/emulator_state",
    "/sil/bms/status", "/sil/bms/emulator_state", "/sil/fw/status", "/sil/lift/permit", "/sil/lift/command",
    "/build1/rtf_stats", "/build1/run_info", "/scan_filtered", "/baseline1/mission_status",   # Proposed (#82)
    "/navigate_to_pose/_action/status", "/navigate_to_pose/_action/feedback", "/backup/_action/status",
    "/move_action/_action/status", "/execute_trajectory/_action/status",
)
BAG_REGEX = r"/.*_controller/(controller_state|state|joint_trajectory|commands)$|/.*gripper.*/_action/status$"
VCAN = ("vcan_drive", "vcan_aux", "vcan_bms", "vcan_arm_l", "vcan_arm_r")

# A fixed camera that sees the base from the origin to the work shelf. 1 Hz of Gazebo time, 480 x 270: a small load
# on a runner that already runs at a low real-time factor.
CAMERA_RATE_HZ = 1.0
CAMERA_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="baseline1_observer">
    <static>true</static>
    <link name="link">
      <sensor name="baseline1_observer" type="camera">
        <always_on>true</always_on>
        <update_rate>{rate}</update_rate>
        <topic>/baseline1/observer/image</topic>
        <camera>
          <horizontal_fov>1.2</horizontal_fov>
          <image><width>480</width><height>270</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>30</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""
CAMERA_POSE = (3.2, -1.1, 1.9, 0.0, 0.42, 3.1416)   # x y z roll pitch yaw: right of the drive line, looking at it


class Recorders:
    def __init__(self, log, directory=None):
        self.log = log
        self.directory = directory or os.environ.get("BUILD1_LOG_DIR", os.path.join(os.getcwd(), "build1_logs"))
        self.processes = []
        self.frames = None

    def _start(self, name, command, **kwargs):
        if shutil.which(command[0]) is None:
            self.log.event(f"recording {name}: {command[0]} not found, not recorded")
            return None
        out = open(os.path.join(self.directory, f"record_{name}.log"), "w", encoding="utf-8")
        try:
            process = subprocess.Popen(command, stdout=out, stderr=subprocess.STDOUT, start_new_session=True, **kwargs)
        except OSError as error:
            self.log.event(f"recording {name}: {error}")
            return None
        self.processes.append((name, process, out))
        self.log.event(f"recording {name}: pid {process.pid}")
        return process

    def start(self, rig):
        os.makedirs(self.directory, exist_ok=True)
        bag = os.path.join(self.directory, "bag")
        shutil.rmtree(bag, ignore_errors=True)
        self._start("bag", ["ros2", "bag", "record", "--use-sim-time", "-o", bag, "--topics", *BAG_TOPICS,
                            "--regex", BAG_REGEX])
        for bus in VCAN:
            self._start(f"candump_{bus}", ["candump", "-L", bus])
        if os.environ.get("BASELINE1_VIDEO", "1") != "0":
            self._camera()

    def _camera(self):
        sdf = CAMERA_SDF.format(rate=CAMERA_RATE_HZ)
        x, y, z, roll, pitch, yaw = CAMERA_POSE
        spawn = subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", "build1", "-name", "baseline1_observer",
                                "-string", sdf, "-x", str(x), "-y", str(y), "-z", str(z), "-R", str(roll), "-P", str(pitch),
                                "-Y", str(yaw)], capture_output=True, text=True, timeout=120, check=False)
        self.log.event(f"recording video: camera spawn exit {spawn.returncode}")
        if spawn.returncode != 0:
            return
        self._start("camera_bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                                      "/baseline1/observer/image@sensor_msgs/msg/Image[gz.msgs.Image",
                                      "--ros-args", "-p", "use_sim_time:=true"])
        self.frames = os.path.join(self.directory, "frames")
        os.makedirs(self.frames, exist_ok=True)

    def subscribe_frames(self, rig):
        """Write each observer image as a PPM with its Gazebo stamp in the file name (no image library needed)."""
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
        rig.node.create_subscription(Image, "/baseline1/observer/image", on_image,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def stop(self):
        for _name, process, _out in self.processes:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 20.0
        for name, process, out in self.processes:
            try:
                process.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                self.log.event(f"recording {name}: killed after 20 s")
            out.close()
        if self.frames is not None:
            self.log.event(f"recording video: {len(os.listdir(self.frames))} frames")


class ContactLog:
    """The contacts of the run, from the gz contact sensors of diagnostics.py (BUILD1_CONTACT_SENSORS=1: the arm links,
    and the item, tote and fixtures of the world). It observes and commands nothing; a gz contact sensor changes no
    force. record() returns what the strict verdict checks against config/baseline1_gates.json
    (baseline1_gates.contacts): every contact event is in the allowed list with its reason, or the run fails."""

    def __init__(self, rig, log):
        from openamrobot_build1_scene import diagnostics as dx
        self.rig, self.log, self.rows, self.messages = rig, log, [], {}
        self.sensors_on = dx.enabled(os.environ)
        self.links = dx.contact_links() if self.sensors_on else []
        self.world = [dx.world_contact_link(model) for model in dx.WORLD_SENSOR_MODELS] if self.sensors_on else []
        if not self.sensors_on:
            log.event("contacts: BUILD1_CONTACT_SENSORS is not 1, no contact sensors in this run")
            return
        import threading
        from ros_gz_interfaces.msg import Contacts
        self.lock = threading.Lock()
        for link in self.links + self.world:
            rig.node.create_subscription(Contacts, dx.ros_contact_topic(link), lambda m, l=link: self._on(l, m), 50)
        log.event(f"contacts: {len(self.links)} arm link sensors, world sensors {self.world}")

    def _on(self, sensor, msg):
        from openamrobot_build1_scene import diagnostics as dx
        t = self.rig.sim_time()
        rows = []
        for contact in msg.contacts:
            one, two = contact.collision1.name, contact.collision2.name
            if sensor.startswith("world_"):
                a, b = one, two                      # both full collision names (model::link::collision)
            else:
                a, b = sensor, (two if sensor in one else one)
            for i in range(max(1, len(contact.positions))):
                fn = None
                if i < len(contact.wrenches) and i < len(contact.normals):
                    force, normal = contact.wrenches[i].body_1_wrench.force, contact.normals[i]
                    fn, _ft = dx.normal_tangential((force.x, force.y, force.z), (normal.x, normal.y, normal.z))
                rows.append({"t": t, "a": a, "b": b, "fn": fn})
        with self.lock:
            self.messages[sensor] = self.messages.get(sensor, 0) + 1
            self.rows += rows

    def record(self, phase_records):
        from openamrobot_build1_scene import baseline1_gates as gates
        spans = [(r["phase"], r["sim_start"], r["sim_end"]) for r in phase_records
                 if r.get("sim_start") is not None and r.get("sim_end") is not None]

        def phase_of(t):
            return next((name for name, start, end in spans if start <= t <= end), None)
        if not self.sensors_on:
            return {"sensors_on": False, "sensor_links": [], "messages": {}, "events": []}
        with self.lock:
            rows, messages = list(self.rows), dict(self.messages)
        return {"sensors_on": True, "sensor_links": self.links + self.world, "messages": messages,
                "events": gates.contact_events(rows, phase_of)}
