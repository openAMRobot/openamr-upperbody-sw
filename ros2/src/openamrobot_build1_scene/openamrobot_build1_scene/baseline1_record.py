# SPDX-License-Identifier: MIT
"""Baseline 1 recordings next to the mission log, in BUILD1_LOG_DIR: a ROS bag of the run, a candump of every virtual
CAN bus and the frames of one fixed observer camera (encoded to a video by the workflow).

The recorders run as child processes of the mission, so they share its ROS domain and Gazebo partition. A recorder
that cannot start is written to the log as an event and the mission goes on: a recording is a record of the run,
never a check. The video is a visual record only, never evidence.

With BASELINE1_AI_CAPTURE=1 (opt-in, Proposed; default off) the bag also gets what a C9 dataset capture needs: see
AI_CAPTURE_TOPICS below and docs/ai/capture_profile_proposal.md. It changes what is recorded, never a check.
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
    "/build1/rtf_stats", "/build1/run_info", "/scan_filtered",
    "/navigate_to_pose/_action/status", "/navigate_to_pose/_action/feedback", "/backup/_action/status",
    "/move_action/_action/status", "/execute_trajectory/_action/status",
)
BAG_REGEX = r"/.*_controller/(controller_state|state|joint_trajectory|commands)$|/.*gripper.*/_action/status$"
VCAN = ("vcan_drive", "vcan_aux", "vcan_bms", "vcan_arm_l", "vcan_arm_r")

# AI capture (Proposed, opt-in: BASELINE1_AI_CAPTURE=1; default off, so a Baseline 1 acceptance run records as before).
# Adds what a C9 capture needs and the default bag lacks: the four robot cameras of build1_robot.urdf.xacro part 3
# (native rates 6, 6, 5, 5 Hz of Gazebo time, not throttled: the converter refuses a reused camera frame, so the export
# grid can be at most the slowest camera rate), their camera_info, a held gripper command (GripperCommandHold) and the
# arm emulators' status, whose mit_q is the gripper motor's last commanded position as received over CAN (motor side).
# Hidden topics (/_action/ status) are included, so the action status topics listed above are recorded too.
AI_CAPTURE_ENV = "BASELINE1_AI_CAPTURE"
CAMERA_TOPICS = (
    "/head_camera/rgbd/image", "/head_camera/rgbd/camera_info",
    "/base_camera/rgbd/image", "/base_camera/rgbd/camera_info",
    "/left_wrist_camera/image", "/left_wrist_camera/camera_info",
    "/right_wrist_camera/image", "/right_wrist_camera/camera_info",
)
GRIPPER_COMMAND_TOPIC = "/build1/capture/gripper_command"    # Proposed topic name, simulation capture only
GRIPPER_JOINTS = {"left": "openarm_left_finger_joint1", "right": "openarm_right_finger_joint1"}
GRIPPER_HOLD_RATE_HZ = 20.0
EMULATOR_STATUS_TOPICS = ("/sim/left_arm_emulator/status", "/sim/right_arm_emulator/status")
AI_CAPTURE_TOPICS = CAMERA_TOPICS + (GRIPPER_COMMAND_TOPIC,) + EMULATOR_STATUS_TOPICS
# Fast DDS discovery for the recorder process only. Under ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST (build1_harness.py)
# Fast DDS 2.x sends discovery to participant indices 0 to 31; two late participants (index 32 or above) never meet.
# That is why base_state_adapter's /odom is missing from the 18 Nov bags (PR #89, measured with 40 probes). The recorder
# gets explicit initial peers for indices 0 to 119; the rest of the stack is unchanged. CycloneDDS ignores the file.
RECORDER_PEER_INDICES = 120


def ai_capture_enabled(environ=None):
    return (os.environ if environ is None else environ).get(AI_CAPTURE_ENV, "0") == "1"


def bag_command(bag, capture):
    topics = BAG_TOPICS + (AI_CAPTURE_TOPICS if capture else ())
    return ["ros2", "bag", "record", "--use-sim-time", "-o", bag, *(["--include-hidden-topics"] if capture else []),
            "--topics", *topics, "--regex", BAG_REGEX]


def discovery_port(domain, index):
    """RTPS unicast metatraffic port of participant `index` in `domain` (PB 7400, DG 250, PG 2, d1 10)."""
    return 7400 + 250 * int(domain) + 10 + 2 * int(index)


def recorder_peers_profile(domain, count=RECORDER_PEER_INDICES):
    locators = "\n".join(f"          <locator><udpv4><address>127.0.0.1</address><port>{discovery_port(domain, i)}</port>"
                         "</udpv4></locator>" for i in range(count))
    return f"""<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <participant profile_name="baseline1_recorder_peers" is_default_profile="true">
      <rtps>
        <builtin>
          <initialPeersList>
{locators}
          </initialPeersList>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>
"""


def recorder_environment(environ, directory):
    """Environment of the bag recorder: unchanged unless AI capture is on and the middleware is Fast DDS."""
    env = dict(environ)
    if not ai_capture_enabled(environ) or env.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp") != "rmw_fastrtps_cpp":
        return env
    path = os.path.join(directory, "recorder_fastdds_peers.xml")
    with open(path, "w", encoding="utf-8") as f:
        f.write(recorder_peers_profile(env.get("ROS_DOMAIN_ID", "0")))
    env["FASTRTPS_DEFAULT_PROFILES_FILE"] = env["FASTDDS_DEFAULT_PROFILES_FILE"] = path
    return env


class GripperCommandHold:
    """The last gripper goal of each side, held: what each GripperActionController is commanded to at a given time.

    position_controllers/GripperActionController keeps the position it read at activation until its first goal, then
    holds each goal's command.position. Before the first goal of a side, the hold value is the first measured finger
    position the capture saw (`start`), a stand-in for the activation position; `source` says which. Read only of the
    action: it never sends a goal. ROS free; the publisher lives in Recorders.attach_gripper_hold."""

    def __init__(self, joints=None):
        self.joints = dict(joints or GRIPPER_JOINTS)
        self.position = {side: None for side in self.joints}
        self.source = {side: None for side in self.joints}
        self.last_stamp_ns = None

    def start(self, measured):
        """measured: joint name -> position (rad) of one joint state sample."""
        for side, joint in self.joints.items():
            if self.position[side] is None and joint in measured:
                self.position[side], self.source[side] = float(measured[joint]), "measured_at_capture_start"

    def goal(self, side, position):
        self.position[side], self.source[side] = float(position), "goal"

    def sample(self, stamp_ns):
        """(names, positions) at `stamp_ns`, or None when a side has no value yet or the stamp does not advance."""
        if any(v is None for v in self.position.values()):
            return None
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return None
        self.last_stamp_ns = stamp_ns
        sides = list(self.joints)
        return [self.joints[s] for s in sides], [self.position[s] for s in sides]

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
        self.capture = ai_capture_enabled()
        self.gripper_hold = None

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
        if self.capture:
            self.log.event(f"recording: AI capture on ({AI_CAPTURE_ENV}=1, Proposed): cameras, camera_info, "
                           f"{GRIPPER_COMMAND_TOPIC}, arm emulator status, hidden topics; recorder discovery peers 0 to "
                           f"{RECORDER_PEER_INDICES - 1}")
            self.attach_gripper_hold(rig)
        self._start("bag", bag_command(bag, self.capture), env=recorder_environment(os.environ, self.directory))
        for bus in VCAN:
            self._start(f"candump_{bus}", ["candump", "-L", bus])
        if os.environ.get("BASELINE1_VIDEO", "1") != "0":
            self._camera()

    def attach_gripper_hold(self, rig):
        """Publish the held gripper command on GRIPPER_COMMAND_TOPIC at GRIPPER_HOLD_RATE_HZ of Gazebo time (Proposed).

        sensor_msgs/JointState, header stamped with Gazebo time, both finger joints, position in rad. The rig reports
        each GripperCommand goal it sends (Rig.gripper_goal_listeners); nothing else is read or sent."""
        from sensor_msgs.msg import JointState
        hold = self.gripper_hold = GripperCommandHold()
        publisher = rig.node.create_publisher(JointState, GRIPPER_COMMAND_TOPIC, 10)
        rig.gripper_goal_listeners.append(hold.goal)

        def tick():
            measured = rig.get("/arm/joint_states")
            if measured is not None:
                hold.start(dict(zip(measured.name, measured.position)))
            now = rig.node.get_clock().now()
            sample = hold.sample(now.nanoseconds)
            if sample is None:
                return
            msg = JointState()
            msg.header.stamp = now.to_msg()
            msg.name, msg.position = sample
            publisher.publish(msg)
        rig.node.create_timer(1.0 / GRIPPER_HOLD_RATE_HZ, tick)

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
