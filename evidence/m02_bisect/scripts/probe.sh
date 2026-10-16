#!/usr/bin/env bash
# probe.sh TAG : launch the sim headless (Bullet-Featherstone) in its own session and domain, sample /joint_states at rest, stop it, reap by partition.
# env: WS_ROOT (default main ws), OVERLAYS (colon list of ament prefixes), LAUNCH_ARGS, EXTRA_SOURCE
set +u
S=${SCRATCH:-/tmp/m02_bisect}
TAG=$1
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
WS_ROOT=${WS_ROOT:-/home/slash/openamr_ws}; source $WS_ROOT/install/setup.bash >/dev/null 2>&1
[ -n "${OVERLAYS:-}" ] && export AMENT_PREFIX_PATH="$OVERLAYS:$AMENT_PREFIX_PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=$((101 + RANDOM % 100)) GZ_PARTITION=m02_probe_$$_$RANDOM
LOG=$S/probe_$TAG.log
setsid ros2 launch openamrobot_upperbody_gazebo sim.launch.py headless:=true physics_engine:=gz-physics-bullet-featherstone-plugin ${LAUNCH_ARGS:-} > $LOG 2>&1 &
LPID=$!
timeout 200 python3 $S/probe_reader.py ${N:-600} ${TOPIC:-/joint_states} > $S/probe_$TAG.out 2>&1
# stop: ask the server, then the launch group, then reap anything of this partition by pid
gz service -s /server_control --reqtype gz.msgs.ServerControl --reptype gz.msgs.Boolean --timeout 3000 --req 'stop: true' >/dev/null 2>&1
kill -INT -$LPID 2>/dev/null; sleep 6; kill -TERM -$LPID 2>/dev/null; sleep 2; kill -KILL -$LPID 2>/dev/null
for d in /proc/[0-9]*; do p=${d#/proc/}; part=$(tr '\0' '\n' < $d/environ 2>/dev/null | grep ^GZ_PARTITION= | cut -d= -f2); [ "$part" = "$GZ_PARTITION" ] && kill -KILL $p 2>/dev/null; done
grep PROBE_JSON $S/probe_$TAG.out | sed 's/^PROBE_JSON //' > $S/probe_$TAG.json
python3 - $TAG <<'PY'
import json,sys
S="${SCRATCH:-/tmp/m02_bisect}"
tag=sys.argv[1]
try:
    d=json.load(open(f"{S}/probe_{tag}.json"))
except Exception as e:
    print(tag,"NO RESULT",e); sys.exit()
j=d["joints"]
f=lambda k: j.get(k)
def row(k):
    v=j.get(k)
    return f"{k.replace('openarm_','')}: mean={v['mean']:+.3e} min={v['min']:+.3e} max={v['max']:+.3e}" if v else f"{k}: -"
print(tag, "msgs", d["messages"]); 
for k in ("openarm_left_finger_joint1","openarm_right_finger_joint1","openarm_left_joint4","openarm_right_joint4"): print("   ",row(k))
PY
