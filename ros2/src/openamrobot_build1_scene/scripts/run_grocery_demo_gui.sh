#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Watch the grocery dark store demonstrator live on this machine: Gazebo window, RViz and the mission, one command.
#
#   run_grocery_demo_gui.sh [--ws DIR] [--setup] [--build] [--no-rviz] [--no-follow] [--no-mission] [--video]
#
#   --ws DIR     colcon workspace (default ~/openamr_ws)
#   --setup      first time: clone the pinned sources into DIR/src, install the pinned Gazebo vendor packages and
#                rosdep keys (sudo apt)
#   --build      colcon build even when the workspace is already built
# Then it checks the machine (Ubuntu 24.04, ROS 2 Jazzy, gz-sim 8.15.0), builds when needed, creates the five virtual
# CAN buses (sudo) and runs: ros2 launch openamrobot_build1_scene grocery_demo_gui.launch.py create_vcan:=false ...
# docs/demo/grocery_dark_store.md, "Watch it live". Simulation only; not hardware or safety evidence.
set -euo pipefail

WS="${HOME}/openamr_ws"; SETUP=0; BUILD=0; ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --ws) WS="$2"; shift ;;
    --setup) SETUP=1 ;;
    --build) BUILD=1 ;;
    --no-rviz) ARGS+=("rviz:=false") ;;
    --no-follow) ARGS+=("follow:=false") ;;
    --no-mission) ARGS+=("mission:=false") ;;
    --video) ARGS+=("video:=1") ;;
    -h|--help) sed -n '3,14p' "$0"; exit 0 ;;
    *) echo "unknown option $1 (see --help)"; exit 2 ;;
  esac
  shift
done

# Pins: the grocery-demo workflow's (.github/workflows/grocery-demo.yml) and config/pins.yaml.
UPPERBODY_BRANCH=demo/grocery-dark-store
PLATFORM_SW_SHA=931e631db661b16d1057dfd1ab1d13ea606482e2
OPENARM_DESCRIPTION_SHA=14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831
OPENARM_ROS2_SHA=4e837e1d0dae692ff67b560b69d8d281d7a8d4ed
OPENARM_CAN_SHA=bfd4e52194aefe38389fa6e670be3738136facb9
GZ_SIM_VENDOR=0.0.13-1noble.20261031.125555
GZ_TOOLS_VENDOR=0.0.8-1noble.20261025.230735
SDFORMAT_VENDOR=0.0.14-1noble.20261031.120928
PACKAGE=openamrobot_build1_scene

say() { echo "[grocery demo] $*"; }
fail() { echo "[grocery demo] STOP: $*" >&2; exit 1; }

. /etc/os-release
[ "${VERSION_ID:-}" = "24.04" ] || fail "Ubuntu 24.04 needed, this is ${PRETTY_NAME:-unknown}"
[ -f /opt/ros/jazzy/setup.bash ] || fail "ROS 2 Jazzy not found in /opt/ros/jazzy (docs: Watch it live, prerequisites)"
set +u; source /opt/ros/jazzy/setup.bash; set -u

clone() {  # clone REPO SHA_OR_BRANCH DIR
  if [ ! -d "$3/.git" ]; then git clone --quiet "https://github.com/$1.git" "$3"; fi
  git -C "$3" fetch --quiet origin "$2" && git -C "$3" checkout --quiet FETCH_HEAD
  say "$1 at $(git -C "$3" rev-parse --short HEAD)"
}

if [ "$SETUP" = "1" ]; then
  mkdir -p "$WS/src/upstream"
  clone openAMRobot/openamr-upperbody-sw "$UPPERBODY_BRANCH" "$WS/src/openamr-upperbody-sw"
  clone openAMRobot/openamr-platform-sw "$PLATFORM_SW_SHA" "$WS/src/openamr-platform-sw"
  clone enactic/openarm_description "$OPENARM_DESCRIPTION_SHA" "$WS/src/upstream/openarm_description"
  clone enactic/openarm_ros2 "$OPENARM_ROS2_SHA" "$WS/src/upstream/openarm_ros2"
  clone enactic/openarm_can "$OPENARM_CAN_SHA" "$WS/src/upstream/openarm_can"
  say "pinned Gazebo vendor packages and the stack (sudo apt)"
  sudo apt-get update
  sudo apt-get install -y --allow-downgrades "ros-jazzy-gz-sim-vendor=${GZ_SIM_VENDOR}" "ros-jazzy-gz-tools-vendor=${GZ_TOOLS_VENDOR}" \
    "ros-jazzy-sdformat-vendor=${SDFORMAT_VENDOR}"
  sudo apt-mark hold ros-jazzy-gz-sim-vendor ros-jazzy-gz-tools-vendor ros-jazzy-sdformat-vendor
  sudo apt-get install -y python3-can can-utils iproute2 "linux-modules-extra-$(uname -r)" ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-moveit ros-jazzy-joint-state-publisher ros-jazzy-rviz2 \
    ros-jazzy-launch-testing-ament-cmake ros-jazzy-ament-cmake-pytest ffmpeg
  (cd "$WS" && rosdep update --rosdistro jazzy && \
    rosdep install --rosdistro jazzy --from-paths $(colcon list --base-paths src --packages-up-to "$PACKAGE" --paths-only) --ignore-src -y -r)
fi
[ -d "$WS/src/openamr-upperbody-sw" ] || fail "no workspace in $WS: run once with --setup (or --ws DIR)"

version=$(gz sim --versions 2>/dev/null || true)
[ "$version" = "8.15.0" ] || say "WARNING: gz-sim ${version:-not found}, the pinned build is 8.15.0 (--setup installs it)"
if [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then fail "no display: this launcher opens the Gazebo window and RViz"; fi
if ! glxinfo -B 2>/dev/null | grep -qi "renderer"; then say "glxinfo not available: cannot tell GPU from software rendering"; fi

if [ "$BUILD" = "1" ] || [ ! -f "$WS/install/$PACKAGE/share/$PACKAGE/launch/grocery_demo_gui.launch.py" ]; then
  say "colcon build --packages-up-to $PACKAGE"
  (cd "$WS" && colcon build --packages-up-to "$PACKAGE" --event-handlers console_cohesion+)
fi
set +u; source "$WS/install/setup.bash"; set -u

say "five virtual CAN buses (sudo: modprobe vcan and ip link)"
sudo modprobe vcan; sudo modprobe can_raw || true
sudo sh "$(ros2 pkg prefix "$PACKAGE")/lib/$PACKAGE/setup_vcan_all.sh"

say "ros2 launch $PACKAGE grocery_demo_gui.launch.py create_vcan:=false ${ARGS[*]:-}"
exec ros2 launch "$PACKAGE" grocery_demo_gui.launch.py create_vcan:=false "${ARGS[@]}"
