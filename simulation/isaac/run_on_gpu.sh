#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Isaac Sim run kit: generate the USD of the combined model from the locked expansion, check it, run the benchmark scenarios and
# collect the evidence to send back. Run from the repository root on a Linux machine with an NVIDIA GPU, Docker and the NVIDIA
# Container Toolkit. Read simulation/isaac/RUN_ON_GPU.md first.
#
#   bash simulation/isaac/run_on_gpu.sh            # everything
#   bash simulation/isaac/run_on_gpu.sh --dry-run  # print the plan and the pins, run nothing
#   bash simulation/isaac/run_on_gpu.sh --cpu-only # only the steps without a GPU (generation, job, checker self-test)
#
# Two containers, both pinned: ros:jazzy-ros-base (xacro at the lock's build, the generators, check_usd, collect) and the Isaac Sim
# image of simulation/sim_pins.yaml. The working tree is mounted at the same absolute path in both, so the absolute mesh paths of the
# importer URDF resolve in Isaac. Output: build/isaac_kit/ and build/isaac_kit_evidence_<UTC>.tar.gz (the file to send back).
# Simulation evidence only: never hardware, physical acceptance or safety evidence.
set -euo pipefail

MODE="${1:-all}"
REPO="$(pwd)"
KIT="$REPO/build/isaac_kit"
ROS_IMAGE="ros:jazzy-ros-base-noble"
[ -f simulation/sim_pins.yaml ] || { echo "run from the openamr-upperbody-sw repository root" >&2; exit 2; }

grep_value() {  # the first line `key: value` of a YAML file (no host Python or PyYAML needed)
  sed -n "s/^ *$2: *\"\{0,1\}\([^\"#]*\)\"\{0,1\}.*/\1/p" "$1" | head -n 1 | tr -d ' '
}

ISAAC_IMAGE="$(grep_value simulation/sim_pins.yaml container_image)"
XACRO="$(grep_value simulation/expansion_lock.yaml xacro_package_version)"
PIN_FILE=ros2/src/openamrobot_upperbody_description/config/upstream_pins.yaml
PLATFORM="$(grep_value "$PIN_FILE" openamr_platform_sw)"
OPENARM_DESC="$(grep_value "$PIN_FILE" openarm_description)"
CONFIGS="bp000 bp050"

echo "== Isaac run kit"
echo "repository      $REPO at $(git rev-parse HEAD 2>/dev/null || echo 'no git')"
echo "isaac image     $ISAAC_IMAGE"
echo "ros image       $ROS_IMAGE (xacro $XACRO)"
echo "platform-sw     $PLATFORM"
echo "openarm_desc    $OPENARM_DESC"
echo "output          $KIT"
if [ "$MODE" = "--dry-run" ]; then
  cat <<EOF
plan:
  1 preflight      nvidia-smi, docker, NVIDIA runtime, free disk  -> $KIT/preflight.json
  2 sources        git clone platform-sw and openarm_description at the pins into $KIT/src
  3 generate       $ROS_IMAGE: generate.py --isaac (MJCF and the Isaac import job), benchmark_mujoco.py -> $KIT/sim, $KIT/benchmark_mujoco.json
  4 import         $ISAAC_IMAGE: isaac_generate_usd.py per configuration ($CONFIGS) -> USD and isaac_import_log.json
  5 scenarios      $ISAAC_IMAGE: isaac_scenarios.py (bp000) -> $KIT/benchmark_isaac.json, $KIT/isaac_rest.png
  6 check          $ROS_IMAGE: check_usd.py per configuration -> check_usd.json; collect_result.py -> isaac_result.json
  7 bundle         $KIT/../isaac_kit_evidence_<UTC>.tar.gz
EOF
  exit 0
fi

mkdir -p "$KIT/src" "$KIT/logs"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"

# 1 preflight ---------------------------------------------------------------------------------------------------------------
{
  echo "{"
  echo "  \"utc\": \"$STAMP\","
  echo "  \"repository_head\": \"$(git rev-parse HEAD 2>/dev/null || echo unknown)\","
  echo "  \"repository_dirty\": $([ -z "$(git status --porcelain 2>/dev/null)" ] && echo false || echo true),"
  echo "  \"host\": \"$(uname -srm)\","
  echo "  \"docker\": \"$(docker --version 2>/dev/null | tr -d '"')\","
  echo "  \"nvidia_smi\": \"$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null | tr '\n' ';' | tr -d '"')\","
  echo "  \"free_disk_gb\": $(df -BG --output=avail "$REPO" | tail -n 1 | tr -dc '0-9'),"
  echo "  \"isaac_image\": \"$ISAAC_IMAGE\", \"ros_image\": \"$ROS_IMAGE\", \"xacro\": \"$XACRO\""
  echo "}"
} > "$KIT/preflight.json"
cat "$KIT/preflight.json"
if [ "$MODE" != "--cpu-only" ]; then
  nvidia-smi -L >/dev/null 2>&1 || { echo "BLOCKED: nvidia-smi finds no GPU" >&2; exit 3; }
  docker info 2>/dev/null | grep -qi nvidia || echo "warning: docker info lists no nvidia runtime; --gpus all needs the NVIDIA Container Toolkit" >&2
fi

# 2 sources -----------------------------------------------------------------------------------------------------------------
fetch() {  # fetch <url> <dir> <sha>
  if [ ! -d "$2/.git" ]; then git clone --quiet "$1" "$2"; fi
  git -C "$2" fetch --quiet origin "$3" 2>/dev/null || true
  git -C "$2" checkout --quiet --detach "$3"
  [ "$(git -C "$2" rev-parse HEAD)" = "$3" ] || { echo "FAIL: $2 is not at $3" >&2; exit 2; }
}
fetch https://github.com/openAMRobot/openamr-platform-sw "$KIT/src/openamr-platform-sw" "$PLATFORM"
fetch https://github.com/enactic/openarm_description "$KIT/src/openarm_description" "$OPENARM_DESC"

# 3 generate (CPU) ------------------------------------------------------------------------------------------------------------
cpu() {  # cpu <command>: in the ROS image with xacro at the locked build and the pinned Python packages
  docker run --rm -v "$REPO":"$REPO" -w "$REPO" -e HOME=/tmp "$ROS_IMAGE" bash -lc "
    set -euo pipefail
    apt-get update -qq >/dev/null
    apt-get install -y -qq --no-install-recommends 'ros-jazzy-xacro=$XACRO' python3-venv git >/dev/null
    git config --global --add safe.directory '*'
    source /opt/ros/jazzy/setup.bash
    [ -x '$KIT/venv/bin/python' ] || python3 -m venv --system-site-packages '$KIT/venv'
    '$KIT/venv/bin/pip' install --quiet -r simulation/requirements.txt
    dpkg-query -W -f='\${Package} \${Version}\n' ros-jazzy-xacro > '$KIT/logs/xacro_version.txt'
    $1
    chmod -R a+rwX '$KIT'"
}
cpu "'$KIT/venv/bin/python' simulation/tools/generate.py --isaac --platform-sw '$KIT/src/openamr-platform-sw' \
       --openarm-description '$KIT/src/openarm_description' --out '$KIT/sim' && \
     '$KIT/venv/bin/python' simulation/tools/benchmark_mujoco.py --platform-sw '$KIT/src/openamr-platform-sw' \
       --openarm-description '$KIT/src/openarm_description' --out '$KIT/mujoco_bench' && \
     cp '$KIT/mujoco_bench/benchmark_mujoco.json' '$KIT/benchmark_mujoco.json' && \
     '$KIT/venv/bin/python' -c 'import json, sys; sys.path.insert(0, \"simulation\"); from openamr_sim import benchmark as bm; json.dump(bm.load_scenarios(), open(\"$KIT/scenarios.json\", \"w\"), indent=1)'" \
  2>&1 | tee "$KIT/logs/generate.log"

if [ "$MODE" = "--cpu-only" ]; then
  cpu "'$KIT/venv/bin/python' simulation/isaac/collect_result.py --kit '$KIT' --mujoco '$KIT/benchmark_mujoco.json' || true" \
    2>&1 | tee "$KIT/logs/collect.log"
  echo "cpu-only: done. Isaac steps not run: BLOCKED."
  exit 0
fi

# 4 and 5 Isaac (GPU) -------------------------------------------------------------------------------------------------------
mkdir -p ~/docker/isaac-sim/cache/main ~/docker/isaac-sim/cache/computecache ~/docker/isaac-sim/logs ~/docker/isaac-sim/config \
         ~/docker/isaac-sim/data ~/docker/isaac-sim/pkg
chmod -R a+rwX ~/docker/isaac-sim 2>/dev/null || true
isaac() {  # isaac <python script and args>: in the pinned Isaac Sim image, headless
  docker run --rm --gpus all --network=host -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=N \
    -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
    -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
    -v "$REPO":"$REPO" -w "$REPO" --entrypoint /isaac-sim/python.sh "$ISAAC_IMAGE" "$@"
}
set +e
for c in $CONFIGS; do
  isaac simulation/tools/isaac_generate_usd.py --job "$KIT/sim/$c/isaac/isaac_job.json" 2>&1 | tee "$KIT/logs/isaac_import_$c.log"
done
isaac simulation/isaac/isaac_scenarios.py --job "$KIT/sim/bp000/isaac/isaac_job.json" --spec "$KIT/scenarios.json" \
  --out "$KIT/benchmark_isaac.json" --screenshot "$KIT/isaac_rest.png" 2>&1 | tee "$KIT/logs/isaac_scenarios.log"
set -e

# 6 check and collect (CPU) -------------------------------------------------------------------------------------------------
checks=""
for c in $CONFIGS; do
  checks="$checks '$KIT/venv/bin/python' simulation/tools/check_usd.py --job '$KIT/sim/$c/isaac/isaac_job.json' \
    --platform-sw '$KIT/src/openamr-platform-sw' --openarm-description '$KIT/src/openarm_description' \
    --json '$KIT/sim/$c/isaac/check_usd.json' || true;"
done
cpu "$checks '$KIT/venv/bin/python' simulation/isaac/collect_result.py --kit '$KIT' --mujoco '$KIT/benchmark_mujoco.json' || true" \
  2>&1 | tee "$KIT/logs/check_and_collect.log"

# 7 bundle --------------------------------------------------------------------------------------------------------------------
BUNDLE="$REPO/build/isaac_kit_evidence_$STAMP.tar.gz"
tar -czf "$BUNDLE" -C "$REPO/build" \
  isaac_kit/preflight.json isaac_kit/isaac_result.json isaac_kit/isaac_comparison.md isaac_kit/benchmark_isaac.json \
  isaac_kit/benchmark_mujoco.json isaac_kit/logs $(cd "$REPO/build" && ls isaac_kit/isaac_rest.png 2>/dev/null) \
  $(cd "$REPO/build" && ls isaac_kit/sim/*/isaac/isaac_import_log.json isaac_kit/sim/*/isaac/check_usd.json isaac_kit/sim/*/isaac/isaac_job.json 2>/dev/null) \
  2>/dev/null || true
echo "== send back: $BUNDLE ($(du -h "$BUNDLE" | cut -f1)) and its sha256:"
sha256sum "$BUNDLE"
python3 -c "import json; r = json.load(open('$KIT/isaac_result.json')); print('overall:', r['status'])" 2>/dev/null || true
