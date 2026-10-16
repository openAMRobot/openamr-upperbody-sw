#!/usr/bin/env bash
# effort_probe.sh TAG : headless sim (Bullet-Featherstone) on a private domain and partition, read /arm/joint_states at rest (position, velocity,
# effort), stop it, reap by partition. env: OVERLAYS (colon list of ament prefixes put first), SCRATCH, N (messages, default 300), LAUNCH_ARGS.
set +u
S=${SCRATCH:?set SCRATCH}
TAG=$1
HERE=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source /home/slash/openamr_ws/install/setup.bash >/dev/null 2>&1
[ -n "${OVERLAYS:-}" ] && export AMENT_PREFIX_PATH="$OVERLAYS:$AMENT_PREFIX_PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=$((101 + RANDOM % 100)) GZ_PARTITION=effort_probe_$$_$RANDOM
mkdir -p $S/effort
setsid ros2 launch openamrobot_upperbody_gazebo sim.launch.py headless:=true physics_engine:=gz-physics-bullet-featherstone-plugin ${LAUNCH_ARGS:-} > $S/effort/$TAG.launch.log 2>&1 &
LPID=$!
timeout 220 python3 $HERE/effort_reader.py ${N:-300} /arm/joint_states > $S/effort/$TAG.out 2>&1
kill -INT -$LPID 2>/dev/null; sleep 8; kill -TERM -$LPID 2>/dev/null; sleep 2; kill -KILL -$LPID 2>/dev/null
for d in /proc/[0-9]*; do p=${d#/proc/}; part=$(tr '\0' '\n' < $d/environ 2>/dev/null | grep ^GZ_PARTITION= | cut -d= -f2); [ "$part" = "$GZ_PARTITION" ] && kill -KILL $p 2>/dev/null; done
grep EFFORT_JSON $S/effort/$TAG.out | sed 's/^EFFORT_JSON //' > $S/effort/$TAG.json
echo "$TAG: $(wc -c < $S/effort/$TAG.json) bytes of result"
