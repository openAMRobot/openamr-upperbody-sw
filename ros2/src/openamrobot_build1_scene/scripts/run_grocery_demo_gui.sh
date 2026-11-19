#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Watch the grocery dark store demonstrator live on this machine: Gazebo window, RViz and the mission, one command.
#
#   run_grocery_demo_gui.sh [--preflight] [--setup [--yes]] [--ws DIR] [--build] [--no-rviz] [--no-follow]
#                           [--no-mission] [--video] [--record] [--headless] [--duration S] [--log-dir DIR] [--force]
#
#   --preflight   read only: PASS/WARN/FAIL for OS, display, GPU and OpenGL, ROS 2, Gazebo pins, workspace pins,
#                 vcan and sudo, disk, RAM and packages, each with the exact fix command (printed, never run)
#   --setup       clone the pinned sources into DIR/src (user space only), then list the apt changes it would make
#                 and ask before running them (no terminal and no --yes: it prints them and stops)
#   --yes         with --setup: run the listed apt changes without asking
#   --ws DIR      colcon workspace (default ~/openamr_ws)
#   --build       colcon build even when the workspace is already built
#   --no-rviz, --no-follow, --no-mission   leave out RViz, the follow camera, the mission
#   --video       also record the store camera frames as CI does (costs real-time factor)
#   --record      ROS bag of the session for replay in RViz (log dir/live_bag) and, on X11 with ffmpeg, a screen video
#   --headless    (or --no-gui) Gazebo server only, no RViz, no follow camera: smoke runs without a desktop
#   --duration S  stop and clean up after S wall seconds (smoke runs); default: run until Ctrl-C
#   --log-dir DIR logs, bags and recordings (default ~/grocery_demo_logs/<date_time>)
#   --force       start even when the preflight reports FAIL
# Order: preflight, build when needed, the virtual CAN buses it lacks (sudo, once), the stack; then it waits for
# simulation time, /tf, /scan and /odom before RViz, the follow camera and the mission. It watches Gazebo and RViz and
# on exit or Ctrl-C stops every process it started and deletes only the vcan interfaces it created.
# docs/demo/grocery_dark_store.md, "Watch it live". Simulation only; not hardware or safety evidence.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${HOME}/openamr_ws"; SETUP=0; YES=0; BUILD=0; PREFLIGHT=0; FORCE=0; RECORD=0; DURATION=0; LOG_DIR=""
RVIZ=true; FOLLOW=true; MISSION=true; VIDEO=0; GAZEBO_GUI=true; UPPERBODY_REF=""; READY_TIMEOUT=900
while [ $# -gt 0 ]; do
  case "$1" in
    --ws) WS="$2"; shift ;;
    --setup) SETUP=1 ;;
    --yes|-y) YES=1 ;;
    --build) BUILD=1 ;;
    --preflight|--dry-run) PREFLIGHT=1 ;;
    --no-rviz) RVIZ=false ;;
    --no-follow) FOLLOW=false ;;
    --no-mission) MISSION=false ;;
    --video) VIDEO=1 ;;
    --record) RECORD=1 ;;
    --headless|--no-gui) GAZEBO_GUI=false; RVIZ=false; FOLLOW=false ;;
    --duration) DURATION="$2"; shift ;;
    --log-dir) LOG_DIR="$2"; shift ;;
    --ready-timeout) READY_TIMEOUT="$2"; shift ;;
    --upperbody-ref) UPPERBODY_REF="$2"; shift ;;
    --force) FORCE=1 ;;
    -h|--help) sed -n '3,26p' "$0"; exit 0 ;;
    *) echo "unknown option $1 (see --help)"; exit 2 ;;
  esac
  shift
done

# Pins: the grocery-demo workflow's (.github/workflows/grocery-demo.yml) and config/pins.yaml. The preflight reads them
# from here; test_live_launcher.py checks them against the workflow.
UPPERBODY_BRANCH=demo/grocery-dark-store
PLATFORM_SW_SHA=931e631db661b16d1057dfd1ab1d13ea606482e2
OPENARM_DESCRIPTION_SHA=14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831
OPENARM_ROS2_SHA=4e837e1d0dae692ff67b560b69d8d281d7a8d4ed
OPENARM_CAN_SHA=bfd4e52194aefe38389fa6e670be3738136facb9
GZ_SIM_VENDOR=0.0.13-1noble.20261031.125555
GZ_TOOLS_VENDOR=0.0.8-1noble.20261025.230735
SDFORMAT_VENDOR=0.0.14-1noble.20261031.120928
GZ_SIM_VERSION=8.15.0
SDFORMAT_VERSION=14.9.0
PACKAGE=openamrobot_build1_scene
ROS_SETUP="${GROCERY_DEMO_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
SYS_NET="${GROCERY_DEMO_SYS_NET:-/sys/class/net}"  # overridden only by test_live_launcher.py
# The five buses and their MTU, as setup_vcan_all.sh: classic CAN 16 for the base, CAN FD 72 for the arms.
VCAN_IFACES="vcan_drive vcan_bms vcan_aux vcan_arm_l vcan_arm_r"
vcan_mtu() { case "$1" in vcan_arm_l|vcan_arm_r) echo 72 ;; *) echo 16 ;; esac; }
# Packages --setup installs after asking (rosdep adds the keys of the workspace).
APT_PACKAGES="python3-can can-utils iproute2 ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-navigation2 \
ros-jazzy-nav2-bringup ros-jazzy-moveit ros-jazzy-joint-state-publisher ros-jazzy-rviz2 ros-jazzy-xacro \
ros-jazzy-launch-testing-ament-cmake ros-jazzy-ament-cmake-pytest ffmpeg mesa-utils"
VENDOR_PACKAGES="ros-jazzy-gz-sim-vendor=${GZ_SIM_VENDOR} ros-jazzy-gz-tools-vendor=${GZ_TOOLS_VENDOR} \
ros-jazzy-sdformat-vendor=${SDFORMAT_VENDOR}"

say() { echo "[grocery demo] $*"; }
fail() { echo "[grocery demo] STOP: $*" >&2; exit 1; }
preflight() {
  local extra=(); [ "$GAZEBO_GUI" = true ] || extra=(--headless)
  python3 "$HERE/live_demo_preflight.py" --ws "$WS" --launcher "$HERE/run_grocery_demo_gui.sh" "${extra[@]}"
}

if [ "$PREFLIGHT" = "1" ]; then
  preflight; exit $?
fi

# ------------------------------------------------------------------------------------------------ --setup
confirm() {  # confirm QUESTION: yes with --yes, ask on a terminal, no otherwise
  [ "$YES" = "1" ] && return 0
  [ -t 0 ] || return 1
  local answer; read -r -p "[grocery demo] $1 [y/N] " answer || return 1
  case "$answer" in y|Y|yes|YES) return 0 ;; *) return 1 ;; esac
}

clone() {  # clone REPO REF DIR: a detached checkout of REF; refuses to move a checkout with local changes
  local repo="$1" ref="$2" dir="$3" head
  if [ ! -d "$dir/.git" ]; then git clone --quiet --no-checkout "https://github.com/$repo.git" "$dir" || fail "git clone $repo"; fi
  if [ -n "$(git -C "$dir" status --porcelain --untracked-files=no 2>/dev/null)" ]; then
    fail "$dir has local changes; commit or stash them (git -C $dir stash), then run --setup again"
  fi
  git -C "$dir" fetch --quiet origin "$ref" || fail "git fetch $repo $ref"
  git -C "$dir" checkout --quiet --detach FETCH_HEAD || fail "git checkout $repo $ref"
  head=$(git -C "$dir" rev-parse HEAD)
  case "$ref" in [0-9a-f]*) [ ${#ref} -eq 40 ] && [ "$head" != "$ref" ] && fail "$repo is at $head, pinned $ref" ;; esac
  echo "$repo $head" >> "$WS/grocery_demo_pins.txt"
  say "$repo at ${head:0:12}"
}

missing_packages() {  # the apt packages of APT_PACKAGES and VENDOR_PACKAGES that are absent or at another version
  local spec name want have
  for spec in $VENDOR_PACKAGES $APT_PACKAGES; do
    name="${spec%%=*}"; want="${spec#*=}"; [ "$want" = "$spec" ] && want=""
    have=$(dpkg-query -W -f='${Version} ${db:Status-Abbrev}' "$name" 2>/dev/null | awk '$2 ~ /^.i/ {print $1}')
    if [ -z "$have" ] || { [ -n "$want" ] && [ "$have" != "$want" ]; }; then echo "$spec"; fi
  done
}

if [ "$SETUP" = "1" ]; then
  mkdir -p "$WS/src/upstream"; : > "$WS/grocery_demo_pins.txt"
  # This repository: the commit the launcher was started from when it runs from a checkout, else --upperbody-ref or the
  # demo branch as it is now (its hash is printed and written to grocery_demo_pins.txt).
  self=$(git -C "$HERE" rev-parse --show-toplevel 2>/dev/null || true)
  if [ -n "$self" ] && git -C "$self" remote get-url origin 2>/dev/null | grep -q 'openamr-upperbody-sw' \
     && [ "$(cd "$self" && pwd -P)" = "$(cd "$WS/src/openamr-upperbody-sw" 2>/dev/null && pwd -P)" ]; then
    say "openAMRobot/openamr-upperbody-sw: using this checkout as it is ($(git -C "$self" rev-parse --short HEAD))"
    echo "openAMRobot/openamr-upperbody-sw $(git -C "$self" rev-parse HEAD)" >> "$WS/grocery_demo_pins.txt"
  else
    ref="$UPPERBODY_REF"
    [ -z "$ref" ] && [ -n "$self" ] && git -C "$self" remote get-url origin 2>/dev/null | grep -q 'openamr-upperbody-sw' \
      && ref=$(git -C "$self" rev-parse HEAD)
    clone openAMRobot/openamr-upperbody-sw "${ref:-$UPPERBODY_BRANCH}" "$WS/src/openamr-upperbody-sw"
  fi
  clone openAMRobot/openamr-platform-sw "$PLATFORM_SW_SHA" "$WS/src/openamr-platform-sw"
  clone enactic/openarm_description "$OPENARM_DESCRIPTION_SHA" "$WS/src/upstream/openarm_description"
  clone enactic/openarm_ros2 "$OPENARM_ROS2_SHA" "$WS/src/upstream/openarm_ros2"
  clone enactic/openarm_can "$OPENARM_CAN_SHA" "$WS/src/upstream/openarm_can"
  say "sources pinned: $WS/grocery_demo_pins.txt"

  [ -f "$ROS_SETUP" ] || fail "ROS 2 Jazzy not found ($ROS_SETUP): install it first (docs: Watch it live, Prerequisites), or use the container route"
  missing=$(missing_packages | tr '\n' ' ')
  plan=()
  if [ -n "$missing" ]; then
    vendors=$(echo "$missing" | tr ' ' '\n' | grep -- '-vendor=' | tr '\n' ' ')
    others=$(echo "$missing" | tr ' ' '\n' | grep -v -- '-vendor=' | grep . | tr '\n' ' ')
    plan+=("sudo apt-get update")
    [ -n "$vendors" ] && plan+=("sudo apt-get install -y --allow-downgrades $vendors" \
      "sudo apt-mark hold ros-jazzy-gz-sim-vendor ros-jazzy-gz-tools-vendor ros-jazzy-sdformat-vendor")
    [ -n "$others" ] && plan+=("sudo apt-get install -y $others")
  fi
  uname_r=$(uname -r)
  [ -d /sys/module/vcan ] || modinfo vcan >/dev/null 2>&1 || plan+=("sudo apt-get install -y linux-modules-extra-$uname_r")
  set +u
  # shellcheck source=/dev/null
  . "$ROS_SETUP"
  set -u
  rosdep update --rosdistro jazzy >/dev/null 2>&1 || say "rosdep update failed (network?); the rosdep step below may be incomplete"
  paths=$(cd "$WS" && colcon list --base-paths src --packages-up-to "$PACKAGE" --paths-only 2>/dev/null | tr '\n' ' ')
  rosdep_plan=$(cd "$WS" && rosdep install --rosdistro jazzy --simulate --from-paths $paths --ignore-src -r 2>/dev/null \
    | grep -E '^\s*sudo' | sed 's/^ *//' || true)
  while IFS= read -r line; do [ -n "$line" ] && plan+=("$line"); done <<< "$rosdep_plan"
  if [ ${#plan[@]} -eq 0 ]; then
    say "packages: nothing to install, nothing changed"
  else
    say "--setup would change this machine's packages with:"
    for line in "${plan[@]}"; do echo "    $line"; done
    say "alternative without changing this machine's packages: the container route (docs/demo/grocery_dark_store.md, Container route)"
    if confirm "run these commands now?"; then
      for line in "${plan[@]}"; do say "$line"; bash -c "$line" || fail "'$line' failed"; done
    else
      say "nothing installed. Run the commands above yourself, or --setup --yes to let the launcher run them"
      exit 3
    fi
  fi
fi

# ------------------------------------------------------------------------------------------------ checks and build
say "preflight (read only)"
preflight_out=$(preflight); preflight_rc=$?
echo "$preflight_out" | grep -E '^(WARN|FAIL)|^     fix|^preflight:' || true
if [ "$preflight_rc" != "0" ] && [ "$FORCE" != "1" ]; then
  fail "the preflight reports FAIL (above); fix it, or --force to start anyway"
fi
[ -f "$ROS_SETUP" ] || fail "ROS 2 Jazzy not found ($ROS_SETUP)"
[ -d "$WS/src/openamr-upperbody-sw" ] || fail "no workspace in $WS: run once with --setup (or --ws DIR)"
set +u
# shellcheck source=/dev/null
. "$ROS_SETUP"
set -u
if [ "$BUILD" = "1" ] || [ ! -f "$WS/install/$PACKAGE/share/$PACKAGE/launch/grocery_demo_gui.launch.py" ]; then
  say "colcon build --packages-up-to $PACKAGE"
  (cd "$WS" && colcon build --packages-up-to "$PACKAGE" --event-handlers console_cohesion+) || fail "colcon build failed"
fi
set +u
# shellcheck source=/dev/null
. "$WS/install/setup.bash"
set -u
LIB="$(ros2 pkg prefix "$PACKAGE")/lib/$PACKAGE"

LOG_DIR="${LOG_DIR:-$HOME/grocery_demo_logs/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$LOG_DIR"
echo "$preflight_out" > "$LOG_DIR/preflight.txt"
# Everything this run starts carries GROCERY_DEMO_RUN_ID; cleanup stops exactly those processes. Its own GZ_PARTITION
# keeps its gz-transport topics and services apart from any other Gazebo on this machine.
GROCERY_DEMO_RUN_ID="grocery_demo_$$_$(date +%s)"; export GROCERY_DEMO_RUN_ID
export GZ_PARTITION="$GROCERY_DEMO_RUN_ID"
say "run $GROCERY_DEMO_RUN_ID, logs in $LOG_DIR"
say "pins: platform-sw ${PLATFORM_SW_SHA:0:7}, gz-sim $GZ_SIM_VERSION, sdformat $SDFORMAT_VERSION (preflight.txt has the checks)"

# ------------------------------------------------------------------------------------------------ cleanup
CREATED_VCAN=""; STACK_PID=""; VIEWS_PID=""; MISSION_PID=""; BAG_PID=""; SCREEN_PID=""; TAIL_PID=""; CLEANED=0
run_pids() {  # PIDs of this user's processes that carry this run's GROCERY_DEMO_RUN_ID (one grep over /proc)
  local f p
  grep -l -s -z -x "GROCERY_DEMO_RUN_ID=$GROCERY_DEMO_RUN_ID" /proc/[0-9]*/environ 2>/dev/null | while read -r f; do
    p="${f#/proc/}"; p="${p%/environ}"
    [ "$p" = "$$" ] || [ ! -O "/proc/$p" ] || echo "$p"
  done
}
alive() { [ -n "$1" ] && kill -0 "$1" 2>/dev/null; }
stop_group() {  # stop_group NAME PID: SIGINT to its process group, then TERM, then KILL
  local name="$1" pid="$2"
  alive "$pid" || return 0
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
  for _ in $(seq 1 40); do alive "$pid" || break; sleep 0.5; done
  if alive "$pid"; then kill -TERM -- "-$pid" 2>/dev/null; sleep 3; fi
  if alive "$pid"; then kill -KILL -- "-$pid" 2>/dev/null; fi
  echo "stopped $name ($pid)" >> "$LOG_DIR/cleanup.txt"
}
cleanup() {
  [ "$CLEANED" = "1" ] && return; CLEANED=1
  trap '' INT TERM
  say "stopping (cleanup)"
  : > "$LOG_DIR/cleanup.txt"
  stop_group "screen video" "$SCREEN_PID"; stop_group "ros2 bag" "$BAG_PID"; stop_group mission "$MISSION_PID"
  stop_group "RViz and follow camera" "$VIEWS_PID"; stop_group "stack (Gazebo, Nav2, MoveIt, devices)" "$STACK_PID"
  [ -n "$TAIL_PID" ] && kill "$TAIL_PID" 2>/dev/null
  # run_pids also lists its own short-lived grep and subshell: keep only what still exists a moment later.
  candidates=$(run_pids); sleep 0.3; left=""
  for p in $candidates; do [ -d "/proc/$p" ] && left="$left $p"; done
  if [ -n "$left" ]; then
    # shellcheck disable=SC2086
    kill -KILL $left 2>/dev/null; echo "killed leftovers:$left" >> "$LOG_DIR/cleanup.txt"
  fi
  for iface in $CREATED_VCAN; do
    if ip link show "$iface" >/dev/null 2>&1; then
      if $SUDO ip link delete "$iface" 2>/dev/null; then echo "deleted $iface (created by this run)" >> "$LOG_DIR/cleanup.txt"
      else say "could not delete $iface: sudo ip link delete $iface"; fi
    fi
  done
  [ -n "${KEPT_VCAN:-}" ] && echo "kept (existed before this run): $KEPT_VCAN" >> "$LOG_DIR/cleanup.txt"
  sed 's/^/[grocery demo]   /' "$LOG_DIR/cleanup.txt"
  (cd "$(dirname "$LOG_DIR")" && tar czf "$LOG_DIR.logs.tar.gz" --exclude='*/bag' --exclude='*/live_bag' --exclude='*.mp4' \
    "$(basename "$LOG_DIR")" 2>/dev/null) && say "log bundle (no bags, no video): $LOG_DIR.logs.tar.gz"
}
trap cleanup EXIT
trap 'say "interrupted"; exit 130' INT
trap 'exit 143' TERM

# ------------------------------------------------------------------------------------------------ virtual CAN
SUDO=""; [ "$(id -u)" = "0" ] || SUDO="sudo"
KEPT_VCAN=""; to_create=""
for iface in $VCAN_IFACES; do
  if [ -d "$SYS_NET/$iface" ]; then
    mtu=$(cat "$SYS_NET/$iface/mtu")
    [ "$mtu" = "$(vcan_mtu "$iface")" ] || fail "$iface exists with MTU $mtu, needs $(vcan_mtu "$iface"); it is not ours to change: sudo ip link set $iface down && sudo ip link set $iface mtu $(vcan_mtu "$iface") && sudo ip link set $iface up"
    ip link show "$iface" 2>/dev/null | grep -q '[<,]UP[,>]' \
      || fail "$iface exists but is down; it is not ours to change: sudo ip link set $iface up"
    KEPT_VCAN="$KEPT_VCAN $iface"
  else
    to_create="$to_create $iface"
  fi
done
if [ -n "$to_create" ]; then
  say "virtual CAN: creating$to_create (sudo); kept as they are:${KEPT_VCAN:- none}"
  [ -z "$SUDO" ] || sudo -v || fail "sudo is needed to create the virtual CAN buses"
  [ -d /sys/module/vcan ] || $SUDO modprobe vcan || fail "modprobe vcan failed: sudo apt-get install -y linux-modules-extra-$(uname -r)"
  $SUDO modprobe can_raw 2>/dev/null || true
  for iface in $to_create; do
    $SUDO ip link add dev "$iface" type vcan || fail "cannot create $iface"
    CREATED_VCAN="$CREATED_VCAN $iface"
    $SUDO ip link set "$iface" mtu "$(vcan_mtu "$iface")" || fail "cannot set MTU $(vcan_mtu "$iface") on $iface"
    $SUDO ip link set "$iface" up || fail "cannot bring $iface up"
  done
  echo "$CREATED_VCAN" > "$LOG_DIR/vcan_created.txt"
else
  say "virtual CAN: all five exist, nothing created"
fi

# ------------------------------------------------------------------------------------------------ start
# A background job of a non-interactive bash starts with SIGINT and SIGQUIT ignored, and an ignored signal survives
# exec: without `env --default-signal` ros2 launch, ros2 bag and ffmpeg could not be stopped gracefully with SIGINT.
# An array, not a function: `function &` would fork a subshell, and $! would not be the new session's leader.
DETACH=(setsid env "--default-signal=INT,QUIT")
launch() {  # launch VAR LOG ARGS...: ros2 launch in its own session (PID = process group id), PID into VAR
  local var="$1" log="$2"; shift 2
  "${DETACH[@]}" ros2 launch "$PACKAGE" grocery_demo_gui.launch.py create_vcan:=false log_dir:="$LOG_DIR" video:="$VIDEO" "$@" \
    > "$log" 2>&1 < /dev/null &
  printf -v "$var" '%s' "$!"
}
run_has() { [ "$(count_cmd "$1")" != "0" ]; }
count_cmd() { local n=0 p; for p in $(run_pids); do tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q "$1" && n=$((n + 1)); done; echo "$n"; }
gz_count() { count_cmd 'gz sim'; }
gui_up() { gz topic -l 2>/dev/null | grep -qx '/gui/camera/pose'; }

say "stack: ros2 launch $PACKAGE grocery_demo_gui.launch.py stack:=true gazebo_gui:=$GAZEBO_GUI (log: $LOG_DIR/stack.log)"
launch STACK_PID "$LOG_DIR/stack.log" stack:=true gazebo_gui:="$GAZEBO_GUI" rviz:=false follow:=false mission:=false
START=$(date +%s)
say "waiting for simulation time, /tf, /scan and /odom (up to $READY_TIMEOUT s)"
python3 "$LIB/live_demo_wait.py" --timeout "$READY_TIMEOUT" > "$LOG_DIR/ready.log" 2>&1 &
WAIT_PID=$!
gz_seen=0  # the most `gz sim` processes of this run seen so far; sampled from the start, so an early death counts
while alive "$WAIT_PID"; do
  alive "$STACK_PID" || { kill "$WAIT_PID" 2>/dev/null; tail -n 30 "$LOG_DIR/stack.log"; fail "the stack exited before it was ready (Gazebo did not start?): $LOG_DIR/stack.log"; }
  n=$(gz_count); [ "$n" -gt "$gz_seen" ] && gz_seen=$n
  sleep 1
done
wait "$WAIT_PID"; ready_rc=$?
cat "$LOG_DIR/ready.log"
[ "$ready_rc" = "0" ] || fail "the stack is not ready (above); logs in $LOG_DIR"
n=$(gz_count); [ "$n" -gt "$gz_seen" ] && gz_seen=$n
say "ready after $(( $(date +%s) - START )) s wall; Gazebo processes of this run: $n"

if [ "$RECORD" = "1" ]; then
  # The topics RViz (rviz/grocery_demo.rviz) and the control room need, plus the mission phases. No camera images (size).
  "${DETACH[@]}" ros2 bag record --use-sim-time -o "$LOG_DIR/live_bag" --topics /clock /tf /tf_static /robot_description \
    /map /global_costmap/costmap /local_costmap/costmap /local_costmap/published_footprint /scan /scan_filtered /odom \
    /build1/ground_truth/odom /amcl_pose /plan /local_plan /build1/ground_truth/item/pose /joint_states \
    /monitored_planning_scene /display_planned_path /grocery_demo/phase /grocery_demo/events /build1/rtf_stats \
    /sil/drive/status /sil/lift/status /sil/bms/status /sil/fw/status > "$LOG_DIR/live_bag.log" 2>&1 < /dev/null &
  BAG_PID=$!
  say "recording: ROS bag $LOG_DIR/live_bag"
  if [ "$GAZEBO_GUI" = "true" ] && [ -n "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ] \
     && ffmpeg -hide_banner -devices 2>/dev/null | grep -q x11grab; then
    "${DETACH[@]}" ffmpeg -hide_banner -loglevel warning -y -f x11grab -framerate 15 -i "$DISPLAY" -c:v libx264 -preset veryfast \
      -pix_fmt yuv420p "$LOG_DIR/screen.mp4" > "$LOG_DIR/screen.log" 2>&1 < /dev/null &
    SCREEN_PID=$!
    say "recording: screen video $LOG_DIR/screen.mp4 (ffmpeg x11grab, the whole screen)"
  else
    say "no screen video (needs X11 and ffmpeg with x11grab): use the Gazebo Video Recorder plugin or the desktop's recorder"
  fi
fi

if [ "$RVIZ" = "true" ] || [ "$FOLLOW" = "true" ]; then
  say "views: RViz $RVIZ, follow camera $FOLLOW (log: $LOG_DIR/views.log)"
  launch VIEWS_PID "$LOG_DIR/views.log" stack:=false rviz:="$RVIZ" follow:="$FOLLOW" mission:=false
fi
if [ "$MISSION" = "true" ]; then
  say "mission: twelve phases (log: $LOG_DIR/mission.log)"
  launch MISSION_PID "$LOG_DIR/mission.log" stack:=false rviz:=false follow:=false mission:=true mission_delay:=0.0
  tail -n +1 -F "$LOG_DIR/mission.log" 2>/dev/null | grep --line-buffered -E '^[^ ]*grocery demo mission:|^ *(Phase|phase)|\| *(PASS|FAIL|BLOCKED)' &
  TAIL_PID=$!
fi

# ------------------------------------------------------------------------------------------------ watch
if [ "$DURATION" != "0" ]; then say "running; stops by itself after $DURATION s (Ctrl-C stops earlier)"
else say "running. Ctrl-C stops everything this run started"; fi
gui_seen=0; gui_missing=0; rviz_seen=0; rviz_reported=0; follow_reported=0; mission_reported=0
RC=0; t0=$(date +%s); tick=0
while true; do
  sleep 5; tick=$((tick + 1))
  if ! alive "$STACK_PID"; then say "the stack exited (Gazebo closed or crashed): $LOG_DIR/stack.log"; RC=1; break; fi
  n=$(gz_count)
  if [ "$gz_seen" -gt 0 ] && [ "$n" = "0" ]; then say "Gazebo is no longer running: $LOG_DIR/stack.log"; RC=1; break; fi
  [ "$n" -gt 0 ] && gz_seen=$n
  if [ "$GAZEBO_GUI" = "true" ] && [ $((tick % 3)) = 0 ]; then
    if gui_up; then gui_seen=1; gui_missing=0
    elif [ "$gui_seen" = "1" ]; then
      gui_missing=$((gui_missing + 1))
      [ "$gui_missing" = "2" ] && say "the Gazebo window is gone (closed or crashed; the server runs on): $LOG_DIR/stack.log"
    fi
  fi
  if [ "$RVIZ" = "true" ] && [ "$rviz_reported" = "0" ] && [ $((tick % 2)) = 0 ]; then
    if run_has rviz2; then rviz_seen=1
    elif [ "$rviz_seen" = "1" ] || [ "$tick" -ge 24 ]; then
      rviz_reported=1; say "RViz is not running (closed or crashed): $LOG_DIR/views.log; the run goes on, Ctrl-C to stop"
    fi
  fi
  if [ "$FOLLOW" = "true" ] && [ "$follow_reported" = "0" ] && grep -q '^.*gz_follow: ' "$LOG_DIR/views.log" 2>/dev/null; then
    follow_reported=1; say "$(grep -o 'gz_follow: .*' "$LOG_DIR/views.log" | tail -n 1)"
  fi
  if [ -n "$MISSION_PID" ] && ! alive "$MISSION_PID" && [ "$mission_reported" = "0" ]; then
    mission_reported=1; wait "$MISSION_PID" 2>/dev/null
    verdict=$(grep -o 'grocery demo mission: [A-Z]*' "$LOG_DIR/mission.log" | tail -n 1)
    say "mission process ended: ${verdict:-no verdict line (see the log)}; phase table in $LOG_DIR/mission.log, report JSON in $LOG_DIR"
    say "The windows stay open; Ctrl-C to stop"
  fi
  if [ "$DURATION" != "0" ] && [ $(( $(date +%s) - t0 )) -ge "$DURATION" ]; then say "duration $DURATION s reached"; break; fi
done
exit "$RC"
