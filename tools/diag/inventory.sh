#!/usr/bin/env bash
# Diagnostic only. Launches the default profile headless and prints the runtime inventory to the job log.
set +u
source /opt/ros/jazzy/setup.bash; source install/setup.bash
setsid ros2 launch openamrobot_build1_scene build1.launch.py headless:=true create_vcan:=false > launch.log 2>&1 &
LP=$!
ok=0
for i in $(seq 1 60); do
  if timeout 8 ros2 topic echo --once /clock > /dev/null 2>&1; then ok=1; break; fi; sleep 5
done
echo "INV clock_seen=$ok after_tries=$i"
sleep 90
echo "=====NODES"; ros2 node list | sort
echo "=====TOPICS"; ros2 topic list -t | sort
echo "=====ACTIONS"; ros2 action list -t | sort
echo "=====SERVICE_COUNT"; ros2 service list | wc -l
echo "=====CONTROLLERS"; timeout 20 ros2 control list_controllers 2>&1
echo "=====CONTROLLERS_SIM"; timeout 20 ros2 control list_controllers -c /sim/controller_manager 2>&1
echo "=====HW"; timeout 20 ros2 control list_hardware_components 2>&1 | head -60
echo "=====NODEINFO"
for n in $(ros2 node list | sort -u); do echo "@@NODE $n"; timeout 15 ros2 node info "$n" 2>&1 | grep -v -E "parameter|/get_type_description|rcl_interfaces|lifecycle_msgs" ; done
echo "=====TF"; (cd /tmp && timeout 40 ros2 run tf2_tools view_frames > /dev/null 2>&1; cat /tmp/frames_*.gv 2>/dev/null)
echo "=====URDF"
timeout 30 ros2 topic echo --once --full-length --field data /robot_description > /tmp/rd.txt 2>/dev/null
python3 - <<'PY'
import yaml
t = open('/tmp/rd.txt').read().split('\n---')[0]
try: s = yaml.safe_load(t)
except Exception: s = t
open('/tmp/robot.urdf', 'w').write(s if isinstance(s, str) else t)
print('URDF bytes', len(s) if isinstance(s, str) else -1)
PY
pip install --quiet --break-system-packages trimesh pycollada 2>&1 | tail -1
python3 "$1/model_plot.py" /tmp/robot.urdf /tmp/model.png 2>&1 | tail -150
echo "=====PNG_BEGIN"; base64 -w 160 /tmp/model.png 2>/dev/null; echo "=====PNG_END"
echo "=====LAUNCH_TAIL"; tail -15 launch.log
kill -INT -- "-$LP" 2>/dev/null; sleep 20; kill -KILL -- "-$LP" 2>/dev/null
exit 0
