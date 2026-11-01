import os, time, unittest
import launch, launch_testing.actions, pytest
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
@pytest.mark.launch_test
def generate_test_description():
    sim = os.path.join(get_package_share_directory("openamrobot_upperbody_gazebo"), "launch", "sim.launch.py")
    return launch.LaunchDescription([launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(sim),
            launch_arguments={"headless": "true", "physics_engine": "gz-physics-bullet-featherstone-plugin"}.items()),
            launch_testing.actions.ReadyToTest()]), {}
class T(unittest.TestCase):
    def test_wait(self):
        time.sleep(float(os.environ.get("READY", "25")))
        open(os.environ["MARK"], "w").write(repr(time.time()))
