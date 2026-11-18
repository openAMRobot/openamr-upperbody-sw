# SPDX-License-Identifier: MIT
"""The control room monitor node: subscribes to the Digital Twin's topics and publishes summaries under /control_room/.

Read-only by construction: the node is created without parameter services, every publisher goes through
readonly.guard_publisher_topic (a topic outside /control_room/ raises), and the node creates no service client, no
action client and no parameter client. Its timer runs on host steady time, so the status keeps coming while Gazebo is
paused (the status then shows the sim time standing still).

Published, all derived from other nodes' topics:
  /control_room/status                       std_msgs/String JSON, 2 Hz: everything below plus the SIL device states
  /control_room/localization/position_error  std_msgs/Float64 m: AMCL (/amcl_pose) against Gazebo ground truth
  /control_room/localization/yaw_error       std_msgs/Float64 rad
  /control_room/localization/tf_position_error std_msgs/Float64 m: map > base_footprint (/tf) against ground truth
  /control_room/station_error                std_msgs/Float64 m: ground truth against the station parameter
  /control_room/nav_goal                     geometry_msgs/PoseStamped: the last pose of Nav2's /plan
  /control_room/lift/position                std_msgs/Float64 m: lift_joint of /joint_states
  /control_room/item/speed                   std_msgs/Float64 m/s: from /build1/ground_truth/item/pose
  /control_room/contacts/max_force           std_msgs/Float64 N: largest contact force of the last status period
Simulation evidence only; nothing here is a check or safety evidence.
"""
import json

from openamrobot_control_room import readonly, state

SIL_JSON_TOPICS = ("/sil/drive/status", "/sil/drive/emulator_state", "/sil/lift/status", "/sil/lift/emulator_state",
                   "/sil/bms/status", "/sil/bms/emulator_state", "/sil/fw/status", "/build1/rtf_stats", "/build1/run_info")
CONTACT_PREFIX = "/build1/contacts/"


class ControlRoomMonitor:
    """Wraps an rclpy Node; `node_factory` makes it (tests pass their own)."""

    def __init__(self, node):
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry, Path
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock as ClockMsg
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64, String
        import tf2_ros

        self.node = node
        self.String, self.Float64, self.PoseStamped = String, Float64, PoseStamped
        station = node.declare_parameter("station", [float("nan")] * 3).value
        self.station = tuple(station) if len(station) == 3 and all(v == v for v in station) else None
        self.station_name = node.declare_parameter("station_name", "").value
        self.base_frame = node.declare_parameter("base_frame", "base_footprint").value
        self.map_frame = node.declare_parameter("map_frame", "map").value
        self.data = {"sim_time": None, "truth": None, "amcl": None, "tf_error": None, "goal": None, "nav_feedback": None,
                     "lift_m": None, "lift_physics_m": None, "joint_names": 0, "fingers": None, "mission": None,
                     "permit": None, "json": {}, "scan": None}
        self.actions = state.ActionWatch()
        self.item = state.Motion()
        self.contacts = state.Contacts()
        self.period_force = 0.0
        self.contact_subs = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

        def pub(topic, kind, qos=10):
            return node.create_publisher(kind, readonly.guard_publisher_topic(topic), qos)
        self.pub = {"status": pub("/control_room/status", String),
                    "loc": pub("/control_room/localization/position_error", Float64),
                    "loc_yaw": pub("/control_room/localization/yaw_error", Float64),
                    "loc_tf": pub("/control_room/localization/tf_position_error", Float64),
                    "station": pub("/control_room/station_error", Float64),
                    "goal": pub("/control_room/nav_goal", PoseStamped, latched),
                    "lift": pub("/control_room/lift/position", Float64),
                    "item": pub("/control_room/item/speed", Float64),
                    "force": pub("/control_room/contacts/max_force", Float64)}
        sub = node.create_subscription
        sub(ClockMsg, "/clock", self.on_clock, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        sub(Odometry, "/build1/ground_truth/odom", self.on_truth, 10)
        sub(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        sub(Path, "/plan", self.on_plan, 10)
        sub(JointState, "/joint_states", self.on_joint_states, 10)
        sub(JointState, "/sil/gz/joint_states", self.on_physics_joints, 10)
        sub(JointState, "/build1/ground_truth/finger_joint_states", self.on_fingers, 10)
        sub(PoseStamped, "/build1/ground_truth/item/pose", self.on_item, 10)
        sub(Bool, "/sil/lift/permit", self.on_permit, 10)
        for topic in SIL_JSON_TOPICS:
            qos = latched if topic == "/build1/run_info" else 10
            sub(String, topic, lambda m, t=topic: self.on_json(t, m), qos)
        sub(String, state.MISSION_STATUS_TOPIC, self.on_mission, latched)
        for action in state.ACTIONS:
            sub(GoalStatusArray, f"{action}/_action/status", lambda m, a=action: self.on_status(a, m), 10)
        try:
            from nav2_msgs.action import NavigateToPose
            sub(NavigateToPose.Impl.FeedbackMessage, "/navigate_to_pose/_action/feedback", self.on_nav_feedback, 10)
        except ImportError:
            pass
        try:
            from sensor_msgs.msg import LaserScan
            sub(LaserScan, "/scan_filtered", self.on_scan, qos_profile_sensor_data)
        except ImportError:
            pass
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node, spin_thread=False)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        node.create_timer(0.5, self.tick, clock=steady)
        node.create_timer(2.0, self.discover_contacts, clock=steady)

    # ---------------------------------------------------------------- callbacks (read only)
    def on_clock(self, msg):
        self.data["sim_time"] = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_truth(self, msg):
        p = msg.pose.pose
        self.data["truth"] = (p.position.x, p.position.y, state.yaw_of(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def on_amcl(self, msg):
        p = msg.pose.pose
        c = msg.pose.covariance
        self.data["amcl"] = {"pose": (p.position.x, p.position.y,
                                      state.yaw_of(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)),
                             "sigma_x": max(c[0], 0.0) ** 0.5, "sigma_y": max(c[7], 0.0) ** 0.5, "sigma_yaw": max(c[35], 0.0) ** 0.5}
        error, yaw = state.pose_error(self.data["amcl"]["pose"], self.data["truth"])
        if error is not None:
            self.pub["loc"].publish(self.Float64(data=float(error)))
            self.pub["loc_yaw"].publish(self.Float64(data=float(yaw)))

    def on_plan(self, msg):
        if msg.poses:
            goal = msg.poses[-1]
            out = self.PoseStamped()
            out.header = msg.header
            out.pose = goal.pose
            q = goal.pose.orientation
            new = (round(goal.pose.position.x, 3), round(goal.pose.position.y, 3), round(state.yaw_of(q.x, q.y, q.z, q.w), 3))
            if new != self.data["goal"]:
                self.data["goal"] = new
                self.pub["goal"].publish(out)

    def on_nav_feedback(self, msg):
        fb = msg.feedback
        self.data["nav_feedback"] = {"distance_remaining_m": round(fb.distance_remaining, 3),
                                     "recoveries": int(fb.number_of_recoveries)}

    def on_joint_states(self, msg):
        names = list(msg.name)
        self.data["joint_names"] = len(names)
        if "lift_joint" in names and names.index("lift_joint") < len(msg.position):
            self.data["lift_m"] = msg.position[names.index("lift_joint")]
            self.pub["lift"].publish(self.Float64(data=float(self.data["lift_m"])))

    def on_physics_joints(self, msg):
        names = list(msg.name)
        if "lift_joint" in names and names.index("lift_joint") < len(msg.position):
            self.data["lift_physics_m"] = msg.position[names.index("lift_joint")]

    def on_fingers(self, msg):
        self.data["fingers"] = {n: round(p, 4) for n, p in zip(msg.name, msg.position)}

    def on_item(self, msg):
        p = msg.pose.position
        t = self.data["sim_time"] or (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.item.update(t, (p.x, p.y, p.z))

    def on_permit(self, msg):
        self.data["permit"] = bool(msg.data)

    def on_json(self, topic, msg):
        try:
            self.data["json"][topic] = json.loads(msg.data)
        except ValueError:
            self.data["json"][topic] = {"unparsed": msg.data[:200]}

    def on_mission(self, msg):
        try:
            self.data["mission"] = json.loads(msg.data)
        except ValueError:
            self.data["mission"] = {"unparsed": msg.data[:200]}

    def on_status(self, action, msg):
        self.actions.update(action, [(bytes(s.goal_info.goal_id.uuid).hex(), s.status) for s in msg.status_list])

    def on_scan(self, msg):
        finite = sum(1 for v in msg.ranges if v == v and v not in (float("inf"), float("-inf")))
        self.data["scan"] = {"beams": len(msg.ranges), "with_return": finite}

    def on_contacts(self, topic, msg):
        wrenches = [[(w.body_1_wrench.force.x, w.body_1_wrench.force.y, w.body_1_wrench.force.z) for w in c.wrenches]
                    for c in msg.contacts]
        force = state.contact_force(wrenches)
        self.contacts.update(topic, len(msg.contacts), force)
        self.period_force = max(self.period_force, force)

    def discover_contacts(self):
        """Contact sensor topics exist only with BUILD1_CONTACT_SENSORS=1; subscribe to each one that appears."""
        try:
            from ros_gz_interfaces.msg import Contacts
        except ImportError:
            return
        for topic, types in self.node.get_topic_names_and_types():
            if topic.startswith(CONTACT_PREFIX) and topic not in self.contact_subs and "ros_gz_interfaces/msg/Contacts" in types:
                self.contact_subs[topic] = self.node.create_subscription(
                    Contacts, topic, lambda m, t=topic: self.on_contacts(t, m), 10)

    # ---------------------------------------------------------------- summaries
    def tf_error(self):
        if self.data["truth"] is None:
            return None
        try:
            from rclpy.time import Time
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:  # noqa: BLE001 - no transform yet
            return None
        q = tf.transform.rotation
        pose = (tf.transform.translation.x, tf.transform.translation.y, state.yaw_of(q.x, q.y, q.z, q.w))
        return state.pose_error(pose, self.data["truth"])

    def snapshot(self):
        d = self.data
        amcl = d["amcl"]
        loc = state.pose_error(amcl["pose"], d["truth"]) if amcl else (None, None)
        tf_err = self.tf_error()
        station = state.station_error(d["truth"], self.station)
        js = d["json"]
        mission = d["mission"]
        return {
            "sim_time": d["sim_time"],
            "rtf": state.scalars(js.get("/build1/rtf_stats")),
            "run_info": state.scalars(js.get("/build1/run_info")),
            "mission": {"source": state.MISSION_STATUS_TOPIC + " (Proposed)" if mission else "none: the mission publishes no "
                        "status topic yet; observed activity below is inferred from action status topics",
                        "status": mission, "observed_activity": self.actions.active()},
            "actions": self.actions.summary(),
            "nav": {"goal_from_plan": d["goal"], "feedback": d["nav_feedback"]},
            "localization": {"amcl_minus_truth_m": _r(loc[0]), "amcl_yaw_minus_truth_rad": _r(loc[1]),
                             "tf_map_minus_truth_m": _r(tf_err[0]) if tf_err else None,
                             "amcl_sigma": {k: _r(v) for k, v in amcl.items() if k != "pose"} if amcl else None,
                             "truth": [_r(v) for v in d["truth"]] if d["truth"] else None},
            "station": dict({"name": self.station_name, "pose": self.station},
                            **({k: _r(v) for k, v in station.items()} if station else {})),
            "lift": {"joint_states_m": _r(d["lift_m"]), "physics_m": _r(d["lift_physics_m"]), "permit_topic": d["permit"],
                     "ctl017_status": state.scalars(js.get("/sil/lift/status")),
                     "ctl017_emulator": state.scalars(js.get("/sil/lift/emulator_state"))},
            "drive": {"status": state.scalars(js.get("/sil/drive/status")),
                      "emulator": state.scalars(js.get("/sil/drive/emulator_state"))},
            "bms": {"status": state.scalars(js.get("/sil/bms/status")), "emulator": state.scalars(js.get("/sil/bms/emulator_state"))},
            "firmware": state.scalars(js.get("/sil/fw/status")),
            "grasp": {"fingers": d["fingers"], "left_gripper": self.actions.last_terminal.get("/left_gripper_controller/gripper_cmd"),
                      "right_gripper": self.actions.last_terminal.get("/right_gripper_controller/gripper_cmd")},
            "item": self.item.summary(),
            "contacts": self.contacts.summary(),
            "scan": d["scan"],
            "joint_states_names": d["joint_names"],
            "disclaimer": "Simulation monitor, read only. Not hardware, physical acceptance or safety evidence.",
        }

    def tick(self):
        snap = self.snapshot()
        self.pub["status"].publish(self.String(data=json.dumps(snap, sort_keys=True, default=str)))
        loc = snap["localization"]["tf_map_minus_truth_m"]
        if loc is not None:
            self.pub["loc_tf"].publish(self.Float64(data=float(loc)))
        if snap["station"].get("distance_m") is not None:
            self.pub["station"].publish(self.Float64(data=float(snap["station"]["distance_m"])))
        if snap["item"]:
            self.pub["item"].publish(self.Float64(data=float(snap["item"]["speed_mps"])))
        self.pub["force"].publish(self.Float64(data=float(self.period_force)))
        self.period_force = 0.0


def _r(value, digits=4):
    return None if value is None else round(float(value), digits)


def make_node(name="control_room_monitor", **kwargs):
    """An rclpy Node without parameter services: nobody can set this node's parameters at run time."""
    from rclpy.node import Node
    return Node(name, start_parameter_services=False, **kwargs)


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = make_node()
    ControlRoomMonitor(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
