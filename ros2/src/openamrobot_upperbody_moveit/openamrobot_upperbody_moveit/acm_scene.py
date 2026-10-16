"""A MoveIt planning scene of the combined model for collision checks that ignore the SRDF: what M03, the matrix derivation and the planning-time
measurement share. Needs moveit_py, so it runs in a process of its own that leaves with os._exit (a MoveItPy that is torn down segfaults).

    with open_scene("bp000") as scene:
        scene.set_default_state()
        colliding = scene.colliding(pairs)             # the subset of `pairs` that collide in the current state, with the SRDF's list ignored

Collision is checked with an allowed-collision matrix of the scene's own that allows everything except the pairs under test, so nothing the SRDF
says is consulted. The contact map is not readable from Python (the `Contact` type is not bound in moveit_py: `CollisionResult.contacts` raises
"Unregistered type"), so the colliding pairs of a set are found by group testing: one check with only that set forbidden and, on a hit, halving.
"""
import xml.etree.ElementTree as ET

_KEEP_ALIVE = []            # a MoveItPy that is garbage collected segfaults; the worker keeps it until os._exit


def colliding_pairs(check, pairs):
    """Every pair of `pairs` that collides in the state `check` looks at, by halving. `check(subset)` is True when any pair of the subset
    collides. One call when nothing collides."""
    if not pairs or not check(pairs):
        return []
    if len(pairs) == 1:
        return list(pairs)
    middle = len(pairs) // 2
    return colliding_pairs(check, pairs[:middle]) + colliding_pairs(check, pairs[middle:])


class Scene:
    def __init__(self, base_plate=None, srdf_file=None, node_name="acm_scene", hardware_profile="mock"):
        from moveit.core import collision_detection as cd
        from moveit.core.robot_state import RobotState
        from moveit.planning import MoveItPy
        from openamrobot_upperbody_moveit.moveit_config import moveit_config
        self.cd = cd
        cfg = moveit_config(base_plate, hardware_profile, srdf_file=srdf_file).to_dict()
        cfg["planning_scene_monitor_options"] = {
            "name": "planning_scene_monitor", "robot_description": "robot_description", "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": 0.0}            # 0: do not wait for a /joint_states that nothing publishes here
        cfg["planning_pipelines"] = {"pipeline_names": ["ompl"]}
        cfg["plan_request_params"] = {"planning_attempts": 1, "planning_pipeline": "ompl", "max_velocity_scaling_factor": 0.1,
                                      "max_acceleration_scaling_factor": 0.1, "planning_time": 1.0}
        self.config = cfg
        self.urdf_text = cfg["robot_description"]
        self.urdf_root = ET.fromstring(self.urdf_text)
        self.link_names = [link.get("name") for link in self.urdf_root.findall("link")]
        self.moveit_py = MoveItPy(node_name=f"{node_name}_{base_plate or 'baseline'}", config_dict=cfg)
        _KEEP_ALIVE.append(self.moveit_py)
        self.robot_model = self.moveit_py.get_robot_model()
        self.state = RobotState(self.robot_model)
        self.request = cd.CollisionRequest()        # no contacts: see the module docstring
        self._context = None
        self.scene = None

    def __enter__(self):
        self._context = self.moveit_py.get_planning_scene_monitor().read_only()
        self.scene = self._context.__enter__()
        return self

    def __exit__(self, *exc):
        return self._context.__exit__(*exc)

    def set_default_state(self):
        self.state.set_to_default_values()
        self.state.update()

    def set_joint_positions(self, positions):
        self.state.joint_positions = {**self.state.joint_positions, **positions}
        self.state.update()

    def forbidding(self, subset):
        acm = self.cd.AllowedCollisionMatrix(self.link_names, True)          # everything allowed ...
        for a, b in subset:
            acm.set_entry(a, b, False)                                      # ... except the pairs under test
        return acm

    def any_collision(self, subset):
        result = self.cd.CollisionResult()
        self.scene.check_collision(self.request, result, self.state, self.forbidding(subset))
        return result.collision

    def colliding(self, pairs):
        return colliding_pairs(self.any_collision, list(pairs))


def open_scene(base_plate=None, srdf_file=None, node_name="acm_scene", hardware_profile="mock"):
    """A Scene to use as a context manager (see the module docstring). `base_plate`: bp000 | bp050 (default: option B's baseline)."""
    return Scene(base_plate, srdf_file=srdf_file, node_name=node_name, hardware_profile=hardware_profile)
