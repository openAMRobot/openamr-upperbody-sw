"""How joint_state_clamp.py ends: exit 0 on a shutdown, an error still an error.

P14 ran A7 twice on an unchanged commit and got 2 FAIL, then 0 FAIL. The cause was this node, which exited 1 at shutdown: rclpy raised
ExternalShutdownException out of `spin` (every SIGTERM, found by sending one: 30 of 30), and the executor's `take_message` raised a RuntimeError
when the signal landed between the wait set and the take (1 of 30 SIGINTs). A crash of a node this repo owns is fixed, not allow-listed.

Two layers. The decision, with no ROS runtime to race: `run()` with a `spin` that raises what rclpy raises. And the process itself, a real
signal to a real node that is being flooded with messages, which is the only thing that ever showed the defect."""
import importlib.util
import os
import random
import signal
import subprocess
import sys
import time

import pytest
from ament_index_python.packages import get_package_prefix
from rclpy.executors import ExternalShutdownException

SCRIPT = os.path.join(get_package_prefix("openamrobot_upperbody_moveit"), "lib", "openamrobot_upperbody_moveit", "joint_state_clamp.py")

URDF = ('<robot name="r"><link name="a"/><link name="b"/><joint name="j1" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/>'
        '<limit lower="0" upper="1" effort="1" velocity="1"/></joint></robot>')
FLOOD = ("import rclpy, time\nfrom sensor_msgs.msg import JointState\nrclpy.init(); n = rclpy.create_node('flood')\n"
         "p = n.create_publisher(JointState, '/joint_states', 10); m = JointState(name=['j1'], position=[-2.5e-7])\n"
         "while rclpy.ok():\n    p.publish(m); time.sleep(0.0005)\n")


@pytest.fixture(scope="module")
def clamp_script():
    spec = importlib.util.spec_from_file_location("joint_state_clamp_script", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def raising(exception):
    def spin(node):
        raise exception
    return spin


# --- the decision ---------------------------------------------------------------------------------------------------------------------------
def test_a_spin_that_returns_is_exit_0(clamp_script):
    assert clamp_script.run(object(), spin=lambda node: None, context_ok=lambda: False) == 0


@pytest.mark.parametrize("exception", [KeyboardInterrupt(), ExternalShutdownException()], ids=["KeyboardInterrupt", "ExternalShutdownException"])
def test_the_ways_a_shutdown_ends_a_spin_are_exit_0(clamp_script, exception):
    assert clamp_script.run(object(), spin=raising(exception), context_ok=lambda: True) == 0


def test_the_take_message_race_with_a_closed_context_is_a_shutdown(clamp_script):
    race = RuntimeError("Unable to convert call argument '0' to Python object")
    assert clamp_script.run(object(), spin=raising(race), context_ok=lambda: False) == 0


def test_the_same_error_with_a_live_context_is_not_a_shutdown_and_is_not_swallowed(clamp_script):
    with pytest.raises(RuntimeError, match="Unable to convert"):
        clamp_script.run(object(), spin=raising(RuntimeError("Unable to convert call argument '0' to Python object")), context_ok=lambda: True)


def test_any_other_exception_is_not_swallowed_whatever_the_context(clamp_script):
    for ok in (True, False):
        with pytest.raises(ValueError):
            clamp_script.run(object(), spin=raising(ValueError("a bug")), context_ok=lambda: ok)


# --- the process ----------------------------------------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def flooded_domain():
    """A private ROS domain on localhost, and a publisher that floods /joint_states so the signal lands while messages are being taken."""
    saved = {k: os.environ.get(k) for k in ("ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE")}
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(101, 200))
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    flood = subprocess.Popen([sys.executable, "-c", FLOOD], stderr=subprocess.DEVNULL)
    yield
    flood.terminate()
    flood.wait(timeout=10)
    for key, value in saved.items():
        if value is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = value


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM], ids=["SIGINT", "SIGTERM"])
def test_a_signal_to_the_running_node_is_exit_0_with_its_counters_printed(flooded_domain, sig):
    for attempt in range(5):
        process = subprocess.Popen([sys.executable, SCRIPT, "--ros-args", "-p", f"robot_description:={URDF}"],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        time.sleep(1.5 + random.random() * 0.5)
        process.send_signal(sig)
        output, _ = process.communicate(timeout=20)
        assert process.returncode == 0, f"attempt {attempt}: exit {process.returncode} on {sig.name}:\n{output[-2000:]}"
        assert "final counters" in output and "Traceback" not in output, output[-2000:]
