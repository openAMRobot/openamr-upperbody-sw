#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# rc1_desktop_check.sh: desktop diagnostic for the OpenAMRobot 2.0 Digital Twin rc.1, candidate 2.
#
# Usage:
#   bash rc1_desktop_check.sh MODE [WORKSPACE]
#   bash rc1_desktop_check.sh -h
#
#   MODE       drive | lift | grasp
#                drive  profile:=fast rviz:=true (default engine, DART)            scenario scn01_drive
#                lift   profile:=fast nav2:=false moveit:=false sensors:=false
#                       extra_sensors:=false                                       scenario scn16_lift_stops
#                grasp  profile:=fast station:=work_shelf sensors:=false nav2:=false moveit:=true
#                       physics_engine:=gz-physics-bullet-featherstone-plugin rviz:=true
#                                                                                  scenario scn04_grasp
#   WORKSPACE  default: $HOME/oamr_dt_ws (the default of tools/bootstrap_digital_twin.sh)
#
# Before you run it, in the same terminal (quick-start.md, section 4):
#   source /opt/ros/jazzy/setup.bash
#   source <WORKSPACE>/install/setup.bash
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
#
# What it does: checks the commits under WORKSPACE/src and the environment, reports CPU, memory and GPU facts,
# starts the launch of MODE in its own process group, waits for /clock, /robot_description and TF, lists the
# controllers, measures the real-time factor over 20 s, runs the scenario, waits for your visual checks, sends
# ONE SIGINT to the process group and measures how long the processes take to end. It writes report.md and the
# logs into ./rc1_desktop_<MODE>_<UTC time>/ under the current directory.
#
# What it does not do: it installs nothing, uses no sudo, changes no system configuration, sources nothing and
# never sends a second signal without asking. A failed check prints FAIL. NOT RUN and INFO are not PASS.
# Ctrl+C while it runs skips the remaining steps and goes to the shutdown step; the launch is not left behind
# silently.
#
# There is no mode for scn03_arms_reach and none for arm motion with the default engine in profile:=fast: that
# combination fails (openamr-upperbody-sw issue 126). With the Bullet Featherstone engine the base does not turn,
# so the grasp mode never drives.
#
# Optional environment: RC1_READY_TIMEOUT (seconds to wait for the stack, default 300, the value of the CI probe).
# Exit code: 0 when no check failed, 1 when at least one check printed FAIL, 2 for a usage error.
# Simulation only. Nothing here is hardware, physical acceptance or safety evidence.
set -u

EXPECTED_PLATFORM=2ac40702deaf3533f9612a1e05d1ea9bb41b996c
EXPECTED_UPPERBODY=6687f65b8bd03f83ef6c38a7ddaf47eb3bfd18ac
PACKAGE=openamrobot_build1_scene
DART_ENGINE=gz-physics-dartsim-plugin
BULLET_ENGINE=gz-physics-bullet-featherstone-plugin
# Gazebo vendor builds rc.1 was qualified on (tools/bootstrap_digital_twin.sh).
TESTED_GZ_SIM_VENDOR=0.0.13-1noble.20261031.125555
READY_TIMEOUT="${RC1_READY_TIMEOUT:-300}"
RTF_WINDOW_S=20
SHUTDOWN_TIMEOUT_S=60

usage() { sed -n '3,41p' "$0" | sed 's/^# \{0,1\}//'; }

case "${1:-}" in
  -h|--help) usage; exit 0 ;;
  drive)
    LAUNCH_ARGS=(profile:=fast rviz:=true)
    SCENARIO=scn01_drive; SCENARIO_TIMEOUT=600; EXPECTED_ENGINE=$DART_ENGINE ;;
  lift)
    LAUNCH_ARGS=(profile:=fast nav2:=false moveit:=false sensors:=false extra_sensors:=false)
    SCENARIO=scn16_lift_stops; SCENARIO_TIMEOUT=600; EXPECTED_ENGINE=$DART_ENGINE ;;
  grasp)
    LAUNCH_ARGS=(profile:=fast station:=work_shelf sensors:=false nav2:=false moveit:=true "physics_engine:=$BULLET_ENGINE" rviz:=true)
    SCENARIO=scn04_grasp; SCENARIO_TIMEOUT=1200; EXPECTED_ENGINE=$BULLET_ENGINE ;;
  *) usage >&2; exit 2 ;;
esac
MODE=$1
WORKSPACE="${2:-$HOME/oamr_dt_ws}"
WORKSPACE="$(realpath -m "$WORKSPACE")"

OUT="$PWD/rc1_desktop_${MODE}_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$OUT/scenario" || { echo "FAIL: cannot create $OUT"; exit 1; }

NAMES=(); STATUSES=(); DETAILS=(); FAILS=0
INTERRUPTED=0
LAUNCH_PID=""; LAUNCH_GROUP_OK=0; SCEN_PID=""

# check NAME STATUS DETAIL. STATUS: PASS, FAIL, INFO (a fact, no criterion in the sources), "NOT RUN".
check() {
  NAMES+=("$1"); STATUSES+=("$2"); DETAILS+=("$3")
  [ "$2" = "FAIL" ] && FAILS=$((FAILS + 1))
  printf '%-8s %s: %s\n' "$2" "$1" "$3"
}
section() { printf '\n=== %s\n' "$*"; }
now() { date +%s.%N; }
elapsed() { awk -v a="$1" -v b="$(now)" 'BEGIN { printf "%.1f", b - a }'; }
have() { command -v "$1" > /dev/null 2>&1; }
group_pids() { pgrep -g "$1" 2> /dev/null; }
# PID, elapsed time and name of every process still in the group.
group_listing() {
  local pids
  pids=$(group_pids "$1" | paste -sd, -)
  [ -n "$pids" ] && ps -o pid=,etime=,comm= -p "$pids" 2> /dev/null
}

on_interrupt() {
  INTERRUPTED=1
  printf '\n*** Ctrl+C received: the remaining steps are skipped, the shutdown step follows.\n'
}
trap on_interrupt INT

# ---------------------------------------------------------------------------------------------------------------
section "1. Repository commits under $WORKSPACE/src"
compare_commit() {  # NAME DIRECTORY EXPECTED
  local head dirty
  if ! head=$(git -C "$2" rev-parse HEAD 2> /dev/null); then
    check "$1 commit" FAIL "no git checkout at $2"; return
  fi
  dirty=$(git -C "$2" status --porcelain 2> /dev/null | wc -l)
  if [ "$head" = "$3" ]; then
    check "$1 commit" PASS "$head (uncommitted changes: $dirty files)"
  else
    check "$1 commit" FAIL "found $head, expected $3 (uncommitted changes: $dirty files)"
  fi
  [ "$dirty" -eq 0 ] || check "$1 working tree" FAIL "$dirty changed or untracked files: the build is not the plain commit"
}
{
  for git_dir in "$WORKSPACE"/src/*/.git "$WORKSPACE"/src/*/*/.git; do
    [ -e "$git_dir" ] || continue
    repo=$(dirname "$git_dir")
    printf '%s %s\n' "$(git -C "$repo" rev-parse HEAD 2> /dev/null || echo unreadable)" "${repo#"$WORKSPACE"/}"
  done
} | tee "$OUT/commits.txt"
[ -s "$OUT/commits.txt" ] || echo "no git checkout found under $WORKSPACE/src"
compare_commit "openamr-platform-sw" "$WORKSPACE/src/openamr-platform-sw" "$EXPECTED_PLATFORM"
compare_commit "openamr-upperbody-sw" "$WORKSPACE/src/openamr-upperbody-sw" "$EXPECTED_UPPERBODY"

# ---------------------------------------------------------------------------------------------------------------
section "2. Environment"
if [ "${ROS_DISTRO:-}" = "jazzy" ]; then check "ROS_DISTRO" PASS "jazzy"
else check "ROS_DISTRO" FAIL "'${ROS_DISTRO:-unset}', expected jazzy (source /opt/ros/jazzy/setup.bash)"; fi

ROS2_OK=0
if have ros2; then
  ROS2_OK=1
  prefix=$(ros2 pkg prefix "$PACKAGE" 2> /dev/null || true)
  [ -n "$prefix" ] && prefix=$(realpath -m "$prefix")
  case "$prefix" in
    "$WORKSPACE"/install*) check "workspace sourced" PASS "$PACKAGE from $prefix" ;;
    "") check "workspace sourced" FAIL "$PACKAGE not found (source $WORKSPACE/install/setup.bash)" ;;
    *) check "workspace sourced" FAIL "$PACKAGE comes from $prefix, not from $WORKSPACE/install" ;;
  esac
else
  check "workspace sourced" FAIL "ros2 is not on PATH"
fi

if [ "${RMW_IMPLEMENTATION:-}" = "rmw_cyclonedds_cpp" ]; then check "RMW_IMPLEMENTATION" PASS "rmw_cyclonedds_cpp"
else check "RMW_IMPLEMENTATION" FAIL "'${RMW_IMPLEMENTATION:-unset}', expected rmw_cyclonedds_cpp"; fi
# quick-start.md sets CYCLONEDDS_URI, the manifest README leaves it out: reported, no verdict.
check "CYCLONEDDS_URI" INFO "${CYCLONEDDS_URI:-unset} (quick-start.md sets MaxAutoParticipantIndex 120)"

if have gz; then
  gz_text=$(timeout 30 gz sim --version 2>&1 | tr '\n' ' ')
  gz_version=$(printf '%s' "$gz_text" | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n 1)
  case "$gz_version" in
    8.*) check "gz sim version" PASS "$gz_version (Harmonic is 8.x)" ;;
    *) check "gz sim version" FAIL "expected 8.x (Harmonic), got: ${gz_text:-no output}" ;;
  esac
else
  check "gz sim version" FAIL "gz is not on PATH"
fi
if have dpkg-query; then
  vendor=$(dpkg-query -W -f='${Version}' ros-jazzy-gz-sim-vendor 2> /dev/null || true)
  check "ros-jazzy-gz-sim-vendor" INFO "installed ${vendor:-not found}; rc.1 was qualified on $TESTED_GZ_SIM_VENDOR"
fi

# ---------------------------------------------------------------------------------------------------------------
section "3. Machine and graphics (read only)"
{
  echo "kernel: $(uname -r)"
  echo "os: $(grep -s '^PRETTY_NAME=' /etc/os-release | cut -d= -f2- | tr -d '"')"
  echo "cpus (nproc): $(nproc 2> /dev/null || echo unknown)"
  echo "cpu model: $(grep -m1 'model name' /proc/cpuinfo 2> /dev/null | cut -d: -f2- | sed 's/^ //')"
  grep -E '^(MemTotal|MemAvailable|SwapTotal|SwapFree):' /proc/meminfo 2> /dev/null
  if grep -qi microsoft /proc/version 2> /dev/null; then echo "wsl: yes (WSL_DISTRO_NAME=${WSL_DISTRO_NAME:-unset})"; else echo "wsl: no"; fi
  if [ -e /dev/dxg ]; then echo "/dev/dxg: present"; else echo "/dev/dxg: absent"; fi
  echo "DISPLAY=${DISPLAY:-unset}"
  echo "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-unset}"
  echo "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-unset}"
  echo "GALLIUM_DRIVER=${GALLIUM_DRIVER:-unset}"
  echo "MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME:-unset}"
} > "$OUT/machine.txt" 2>&1
cat "$OUT/machine.txt"
CPUS=$(nproc 2> /dev/null || echo unknown)
MEM_GB=$(awk '/^MemTotal:/ { printf "%.1f", $2 / 1048576 }' /proc/meminfo 2> /dev/null)
check "cpu and memory" INFO "$CPUS processors, ${MEM_GB:-unknown} GiB memory"
if grep -qi microsoft /proc/version 2> /dev/null; then IS_WSL=yes; else IS_WSL=no; fi
if [ -e /dev/dxg ]; then DXG=present; else DXG=absent; fi
check "WSL2 and /dev/dxg" INFO "WSL: $IS_WSL, /dev/dxg: $DXG"
check "GL environment" INFO "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-unset} GALLIUM_DRIVER=${GALLIUM_DRIVER:-unset}"
RENDERER="not measured"
if have glxinfo; then
  timeout 20 glxinfo -B > "$OUT/glxinfo.txt" 2>&1
  RENDERER=$(grep -m1 'OpenGL renderer string' "$OUT/glxinfo.txt" | cut -d: -f2- | sed 's/^ //')
  [ -n "$RENDERER" ] || RENDERER="glxinfo printed no renderer line (see glxinfo.txt)"
  check "glxinfo renderer" INFO "$RENDERER"
else
  check "glxinfo renderer" "NOT RUN" "glxinfo is not installed; this script installs nothing"
fi
if have nvidia-smi; then timeout 20 nvidia-smi -L > "$OUT/nvidia-smi.txt" 2>&1; cat "$OUT/nvidia-smi.txt"; fi

# ---------------------------------------------------------------------------------------------------------------
section "4. Launch ($MODE)"
LAUNCH_CMD=(ros2 launch "$PACKAGE" build1.launch.py "${LAUNCH_ARGS[@]}")
echo "command: ${LAUNCH_CMD[*]}"
leftover=$(pgrep -af "gz sim" 2> /dev/null || true)
if [ "$ROS2_OK" -ne 1 ]; then
  check "launch started" FAIL "ros2 is not on PATH: nothing was launched"
elif [ -n "$leftover" ]; then
  printf '%s\n' "$leftover" | tee "$OUT/simulator_before_launch.txt"
  check "launch started" FAIL "a 'gz sim' process is already running (listed above); nothing was launched and nothing was stopped. See troubleshooting.md, Shutdown"
elif [ "$INTERRUPTED" -ne 0 ]; then
  check "launch started" "NOT RUN" "interrupted by the user"
else
  LAUNCH_T0=$(now)
  # setsid: the launch gets its own process group, so the one interrupt below reaches every process of it,
  # as Ctrl+C in a terminal does (the construct of the clean-install workflow).
  # env --default-signal=INT: bash starts a background command of a script with SIGINT ignored, and the launch and
  # its children would inherit that. A launch typed in a terminal has the default disposition, so it is restored.
  if env --default-signal=INT true 2> /dev/null; then
    RESTORE_INT=(env --default-signal=INT)
  else
    RESTORE_INT=()
    check "SIGINT disposition" INFO "this env has no --default-signal: the launch starts with SIGINT ignored at exec, as in the CI workflow; the shutdown time may differ from a terminal Ctrl+C"
  fi
  setsid "${RESTORE_INT[@]}" "${LAUNCH_CMD[@]}" > "$OUT/launch.log" 2>&1 < /dev/null &
  LAUNCH_PID=$!
  sleep 2
  launch_pgid=$(ps -o pgid= -p "$LAUNCH_PID" 2> /dev/null | tr -d ' ')
  if [ "$launch_pgid" = "$LAUNCH_PID" ]; then
    LAUNCH_GROUP_OK=1
    check "launch started" PASS "process group $LAUNCH_PID, log launch.log"
  elif [ -z "$launch_pgid" ]; then
    check "launch started" FAIL "the launch ended within 2 s: $(tail -n 3 "$OUT/launch.log" | tr '\n' ' ')"
  else
    check "launch started" FAIL "the launch (pid $LAUNCH_PID) is in group $launch_pgid, not in its own group: this script will not signal it. Stop it yourself with Ctrl+C semantics: kill -INT $LAUNCH_PID"
  fi
fi

# ---------------------------------------------------------------------------------------------------------------
section "5. Readiness: /clock advances, /robot_description, TF with base_link (limit ${READY_TIMEOUT} s)"
READY=0
if [ "$LAUNCH_GROUP_OK" -eq 1 ] && [ "$INTERRUPTED" -eq 0 ]; then
  # The probe of the clean-install workflow: more than 20 /clock messages spanning more than 5 s of simulation
  # time, a robot description of more than 1000 characters, and base_link in /tf or /tf_static.
  python3 - "$LAUNCH_PID" "$READY_TIMEOUT" <<'PY' 2>&1 | tee "$OUT/probe.log"
import os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

launch_pid, limit = int(sys.argv[1]), float(sys.argv[2])
rclpy.init()
node = Node('rc1_desktop_probe')
seen = {'clock': [], 'description': 0, 'frames': set()}
node.create_subscription(Clock, '/clock', lambda m: seen['clock'].append(m.clock.sec + m.clock.nanosec * 1e-9), 10)
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(String, '/robot_description', lambda m: seen.__setitem__('description', len(m.data)), latched)
def frames(m):
    for t in m.transforms:
        seen['frames'].update((t.header.frame_id, t.child_frame_id))
node.create_subscription(TFMessage, '/tf', frames, 100)
node.create_subscription(TFMessage, '/tf_static', frames,
                         QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))

def state():
    clock = seen['clock']
    return (len(clock) > 20 and clock[-1] - clock[0] > 5.0, seen['description'] > 1000, 'base_link' in seen['frames'])

start = time.monotonic()
alive = True
try:
    while time.monotonic() - start < limit and not all(state()):
        rclpy.spin_once(node, timeout_sec=0.5)
        try:
            os.kill(launch_pid, 0)
        except OSError:
            alive = False
            break
except (KeyboardInterrupt, Exception) as error:   # rclpy ends its context on SIGINT
    print(f'PROBE stopped early: {type(error).__name__} {error}')
clock_ok, description_ok, tf_ok = state()
first, last = (seen['clock'][0], seen['clock'][-1]) if seen['clock'] else (None, None)
verdict = lambda ok: 'PASS' if ok else 'FAIL'
print(f"PROBE_CLOCK {verdict(clock_ok)} {len(seen['clock'])} messages, simulation time {first} to {last}")
print(f"PROBE_DESCRIPTION {verdict(description_ok)} {seen['description']} characters")
print(f"PROBE_TF {verdict(tf_ok)} {len(seen['frames'])} frames, base_link {'base_link' in seen['frames']}")
print('PROBE frames', sorted(seen['frames']))
print(f"PROBE_WALL {time.monotonic() - start:.1f} s in the probe; launch process alive: {alive}")
sys.exit(0 if clock_ok and description_ok and tf_ok else 1)
PY
  for item in CLOCK DESCRIPTION TF; do
    line=$(grep -m1 "^PROBE_${item} " "$OUT/probe.log" | cut -d' ' -f2-)
    case "$line" in
      PASS*) check "probe ${item,,}" PASS "${line#PASS }" ;;
      FAIL*) check "probe ${item,,}" FAIL "${line#FAIL } (limit ${READY_TIMEOUT} s)" ;;
      *) check "probe ${item,,}" FAIL "the probe printed no result: see probe.log (rclpy import or node start failed?)" ;;
    esac
  done
  if grep -q '^PROBE_CLOCK PASS' "$OUT/probe.log" && grep -q '^PROBE_DESCRIPTION PASS' "$OUT/probe.log" && grep -q '^PROBE_TF PASS' "$OUT/probe.log"; then
    READY=1
    check "time from launch to ready" INFO "$(elapsed "$LAUNCH_T0") s of wall time (includes the 2 s start check; no limit other than ${READY_TIMEOUT} s)"
  fi
else
  for item in clock description tf; do check "probe $item" "NOT RUN" "no launch under the control of this script"; done
fi

# ---------------------------------------------------------------------------------------------------------------
section "6. Controllers"
if [ "$LAUNCH_GROUP_OK" -eq 1 ] && [ "$INTERRUPTED" -eq 0 ]; then
  if timeout 60 ros2 control list_controllers > "$OUT/controllers.txt" 2>&1; then
    cat "$OUT/controllers.txt"
    check "controllers" INFO "$(grep -c . "$OUT/controllers.txt") lines, $(grep -cw active "$OUT/controllers.txt") with the word active (controllers.txt). Listed before the scenario: spawners may still be working"
  else
    tail -n 5 "$OUT/controllers.txt"
    check "controllers" "NOT RUN" "'ros2 control list_controllers' is unavailable or failed (controllers.txt); tolerated, the scenario checks its own controllers"
  fi
else
  check "controllers" "NOT RUN" "no launch under the control of this script"
fi

# ---------------------------------------------------------------------------------------------------------------
section "7. Real-time factor over ${RTF_WINDOW_S} s of wall time, from /clock"
RTF="not measured"
if [ "$READY" -eq 1 ] && [ "$INTERRUPTED" -eq 0 ]; then
  python3 - "$RTF_WINDOW_S" <<'PY' 2>&1 | tee "$OUT/rtf.txt"
import json, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

window = float(sys.argv[1])
rclpy.init()
node = Node('rc1_desktop_rtf')
samples, text = [], {}
node.create_subscription(Clock, '/clock', lambda m: samples.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)),
                         QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
# The launch's own monitor (rtf_monitor.py): its statistics and the latched identity of the run.
node.create_subscription(String, '/build1/rtf_stats', lambda m: text.__setitem__('stats', m.data), 10)
node.create_subscription(String, '/build1/run_info', lambda m: text.__setitem__('info', m.data),
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
wait_until = time.monotonic() + 30.0
try:
    while not samples and time.monotonic() < wait_until:
        rclpy.spin_once(node, timeout_sec=0.2)
    while samples and time.monotonic() - samples[0][0] < window:
        rclpy.spin_once(node, timeout_sec=0.2)
except (KeyboardInterrupt, Exception) as error:   # rclpy ends its context on SIGINT
    print(f'RTF stopped early: {type(error).__name__} {error}')
if len(samples) >= 2 and samples[-1][0] > samples[0][0]:
    wall, sim = samples[-1][0] - samples[0][0], samples[-1][1] - samples[0][1]
    print(f"RTF_MEASURED {sim / wall:.3f} simulation {sim:.3f} s in wall {wall:.3f} s, {len(samples)} clock messages")
else:
    print(f"RTF_FAILED {len(samples)} clock messages received")
print('RTF_STATS', text.get('stats', 'none received on /build1/rtf_stats'))
print('RUN_INFO', text.get('info', 'none received on /build1/run_info'))
try:
    info = json.loads(text.get('info', '{}'))
    print('RUN_PROFILE', info.get('profile'))
    print('RUN_ENGINE', info.get('physics_engine'))
except ValueError:
    pass
PY
  line=$(grep -m1 '^RTF_MEASURED ' "$OUT/rtf.txt" | cut -d' ' -f2-)
  if [ -n "$line" ]; then
    RTF=${line%% *}
    check "real-time factor" INFO "$line. Stack idle, before the scenario. The sources give no pass limit for it"
  else
    check "real-time factor" FAIL "no measurement: $(grep -m1 '^RTF_' "$OUT/rtf.txt")"
  fi
  run_engine=$(grep -m1 '^RUN_ENGINE ' "$OUT/rtf.txt" | cut -d' ' -f2-)
  run_profile=$(grep -m1 '^RUN_PROFILE ' "$OUT/rtf.txt" | cut -d' ' -f2-)
  if [ "$run_engine" = "$EXPECTED_ENGINE" ] && [ "$run_profile" = "fast" ]; then
    check "run identity" PASS "profile $run_profile, engine $run_engine (/build1/run_info)"
  else
    check "run identity" FAIL "profile '${run_profile:-unknown}', engine '${run_engine:-unknown}'; expected fast and $EXPECTED_ENGINE"
  fi
else
  check "real-time factor" "NOT RUN" "the stack was not ready"
  check "run identity" "NOT RUN" "the stack was not ready"
fi

# ---------------------------------------------------------------------------------------------------------------
section "8. Scenario $SCENARIO (limit ${SCENARIO_TIMEOUT} s, the value of the clean-install workflow)"
SCEN_RC="not run"; RESULT_LINE="none"
if [ "$READY" -eq 1 ] && [ "$INTERRUPTED" -eq 0 ]; then
  echo "Watch the Gazebo window now$([ "$MODE" = lift ] || echo ' and the RViz window')."
  setsid timeout "$SCENARIO_TIMEOUT" ros2 run "$PACKAGE" run_scenario.py "$SCENARIO" --log-dir "$OUT/scenario" \
    > >(tee "$OUT/scenario.log") 2>&1 < /dev/null &
  SCEN_PID=$!
  wait "$SCEN_PID"; SCEN_RC=$?
  if [ "$INTERRUPTED" -ne 0 ] && kill -0 "$SCEN_PID" 2> /dev/null; then
    # The scenario client only, in its own group; the launch is not touched here.
    if [ "$(ps -o pgid= -p "$SCEN_PID" 2> /dev/null | tr -d ' ')" = "$SCEN_PID" ]; then kill -TERM -- "-$SCEN_PID" 2> /dev/null
    else kill -TERM "$SCEN_PID" 2> /dev/null; fi
    wait "$SCEN_PID" 2> /dev/null
    SCEN_RC="interrupted by the user"
  fi
  sleep 1
  RESULT_LINE=$(grep -a "BUILD1 $SCENARIO RESULT" "$OUT/scenario.log" | tail -n 1)
  [ -n "$RESULT_LINE" ] || RESULT_LINE="none"
  case "$SCEN_RC" in
    0) case "$RESULT_LINE" in
         *"RESULT PASS"*) check "scenario $SCENARIO" PASS "exit code 0; $RESULT_LINE" ;;
         *) check "scenario $SCENARIO" FAIL "exit code 0 but no RESULT PASS line (found: $RESULT_LINE)" ;;
       esac ;;
    1) check "scenario $SCENARIO" FAIL "exit code 1 (FAIL); $RESULT_LINE" ;;
    3) check "scenario $SCENARIO" FAIL "exit code 3 (BLOCKED, which is not PASS); $RESULT_LINE" ;;
    2) check "scenario $SCENARIO" FAIL "exit code 2: the scenario name is unknown to this build" ;;
    124) check "scenario $SCENARIO" FAIL "exit code 124: no end within ${SCENARIO_TIMEOUT} s; RESULT line: $RESULT_LINE" ;;
    *) check "scenario $SCENARIO" FAIL "exit code '$SCEN_RC'; RESULT line: $RESULT_LINE" ;;
  esac
  grep -a "FAILED CHECK" "$OUT/scenario.log" | head -n 10
else
  check "scenario $SCENARIO" "NOT RUN" "the stack was not ready or the run was interrupted"
fi

# ---------------------------------------------------------------------------------------------------------------
section "9. Visual checks"
if [ "$LAUNCH_GROUP_OK" -eq 1 ] && [ "$INTERRUPTED" -eq 0 ]; then
  if [ -t 0 ]; then
    echo "The twin is still running. Do the visual checks of procedure.md and take the screenshots now."
    echo "Press Enter when they are done. The script then sends ONE SIGINT to process group $LAUNCH_PID."
    read -r _ || true
  else
    echo "Standard input is not a terminal: no pause for visual checks."
  fi
fi

# ---------------------------------------------------------------------------------------------------------------
section "10. Shutdown: one SIGINT to the process group, limit ${SHUTDOWN_TIMEOUT_S} s"
trap '' INT   # a further Ctrl+C must not abort the measurement or the report
SHUTDOWN="not run"
ask() {  # QUESTION -> 0 when the user types y
  local answer=""
  [ -t 0 ] || { echo "$1 [no terminal: answered no]"; return 1; }
  read -r -p "$1 [y/N] " answer || true
  [ "$answer" = "y" ] || [ "$answer" = "Y" ]
}
if [ "$LAUNCH_GROUP_OK" -eq 1 ]; then
  if [ -z "$(group_pids "$LAUNCH_PID")" ]; then
    SHUTDOWN="the launch had already ended before the interrupt"
    check "shutdown" FAIL "$SHUTDOWN: see the end of launch.log"
  else
    group_listing "$LAUNCH_PID" > "$OUT/processes_before_interrupt.txt"
    T0=$(now)
    kill -INT -- "-$LAUNCH_PID" 2> /dev/null
    gone=0
    while awk -v a="$T0" -v b="$(now)" -v l="$SHUTDOWN_TIMEOUT_S" 'BEGIN { exit !(b - a < l) }'; do
      if [ -z "$(group_pids "$LAUNCH_PID")" ]; then gone=1; break; fi
      sleep 0.5
    done
    if [ "$gone" -eq 1 ]; then
      SHUTDOWN="all processes of the group gone $(elapsed "$T0") s after one SIGINT"
      check "shutdown" PASS "$SHUTDOWN"
    else
      SHUTDOWN="processes still alive ${SHUTDOWN_TIMEOUT_S} s after one SIGINT"
      echo "Still alive (pid, elapsed time, name):"
      group_listing "$LAUNCH_PID" | tee "$OUT/processes_after_timeout.txt"
      check "shutdown" FAIL "$SHUTDOWN: $(awk '{ print $3 }' "$OUT/processes_after_timeout.txt" | sort | uniq -c | awk '{ printf "%s x%s ", $2, $1 }')"
      if ask "Send SIGTERM to the remaining processes of group $LAUNCH_PID?"; then
        kill -TERM -- "-$LAUNCH_PID" 2> /dev/null
        for _ in $(seq 1 30); do [ -z "$(group_pids "$LAUNCH_PID")" ] && break; sleep 0.5; done
        SHUTDOWN="$SHUTDOWN; SIGTERM sent on request"
      fi
      if [ -n "$(group_pids "$LAUNCH_PID")" ]; then
        echo "Still alive:"; group_listing "$LAUNCH_PID"
        if ask "Send SIGKILL to the remaining processes of group $LAUNCH_PID?"; then
          kill -KILL -- "-$LAUNCH_PID" 2> /dev/null
          sleep 2
          SHUTDOWN="$SHUTDOWN; SIGKILL sent on request"
        fi
      fi
      if [ -n "$(group_pids "$LAUNCH_PID")" ]; then
        group_listing "$LAUNCH_PID" > "$OUT/processes_left_running.txt"
        SHUTDOWN="$SHUTDOWN; $(wc -l < "$OUT/processes_left_running.txt") processes LEFT RUNNING (processes_left_running.txt)"
        echo "Processes of group $LAUNCH_PID were left running. To end them yourself: kill -TERM -- -$LAUNCH_PID"
      fi
    fi
    echo "interrupt lines in the launch log: $(grep -c "user interrupted\|SIGINT" "$OUT/launch.log")"
  fi
  # Processes that left the group cannot be seen through it. The check of troubleshooting.md, report only.
  sleep 1
  outside=$(pgrep -af "gz sim" 2> /dev/null || true)
  if [ -n "$outside" ]; then
    printf '%s\n' "$outside" | tee "$OUT/simulator_after_shutdown.txt"
    check "no simulator left" FAIL "'gz sim' processes exist after the shutdown (listed above, not stopped by this script)"
  else
    check "no simulator left" PASS "pgrep -af 'gz sim' finds nothing"
  fi
else
  check "shutdown" "NOT RUN" "no launch under the control of this script"
fi

# Gazebo's own render log, when this Gazebo version wrote one. The path is not named in the project sources.
[ -f "$HOME/.gz/rendering/ogre2.log" ] && cp "$HOME/.gz/rendering/ogre2.log" "$OUT/gz_ogre2.log" 2> /dev/null

# ---------------------------------------------------------------------------------------------------------------
section "Report"
{
  echo "# rc.1 candidate 2 desktop check: $MODE"
  echo
  echo "- Date (UTC): $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "- Workspace: \`$WORKSPACE\`"
  echo "- Launch: \`${LAUNCH_CMD[*]}\`"
  echo "- Scenario: \`$SCENARIO\`, exit code: $SCEN_RC"
  echo "- RESULT line: \`$RESULT_LINE\`"
  echo "- Real-time factor (idle, ${RTF_WINDOW_S} s): $RTF"
  echo "- Shutdown: $SHUTDOWN"
  echo "- Machine: $CPUS processors, ${MEM_GB:-unknown} GiB, WSL: $IS_WSL, /dev/dxg: $DXG, renderer: $RENDERER"
  echo "- Checks that printed FAIL: $FAILS"
  echo
  echo "| Check | Status | Detail |"
  echo "|---|---|---|"
  for i in "${!NAMES[@]}"; do
    printf '| %s | %s | %s |\n' "${NAMES[$i]}" "${STATUSES[$i]}" "$(printf '%s' "${DETAILS[$i]}" | tr '|\n' '/ ')"
  done
  echo
  echo "## Not verified by this script"
  echo
  echo "- Anything visual: robot appearance, RViz displays, motion. See procedure.md."
  echo "- That the install tree was built from the commits listed above. The script compares the checkouts only."
  echo "- A pass limit for the real-time factor: the sources give none, so the number is reported without a verdict."
  echo "- The frame rate of the Gazebo and RViz windows: the sources name no probe for it. The cause of a low frame rate is not established."
  echo "- Controller states have no pass criterion here: the list is recorded as printed."
  echo "- glxinfo shows the default OpenGL renderer of this session, not proof of what Gazebo or RViz selected."
  echo "- Simulation only: no hardware, physical acceptance or safety evidence."
  echo
  echo "## Files"
  echo
  for f in "$OUT"/* "$OUT"/scenario/*; do [ -f "$f" ] && echo "- \`${f#"$OUT"/}\`"; done
} > "$OUT/report.md"
echo "report: $OUT/report.md"
echo "checks that printed FAIL: $FAILS"
[ "$FAILS" -eq 0 ]
