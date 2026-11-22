# Isaac Sim run kit

Generates the Isaac Sim USD of the combined model from the same locked expansion that Gazebo and MuJoCo use, checks it, runs the
backend benchmark scenarios in Isaac Sim and packs the evidence into one file. CI runs every step that needs no GPU; the steps
below need an NVIDIA GPU. Until a run's evidence is reviewed, the Isaac result is BLOCKED, never PASS.

Simulation evidence only: not hardware, physical acceptance or safety evidence.

## What you need

| Item | Expected | How to check |
|---|---|---|
| OS | Ubuntu 22.04 or 24.04, x86_64 (the Isaac Sim container is Linux only) | `lsb_release -a` |
| GPU | NVIDIA RTX with 16 GB or more VRAM (NVIDIA lists an RTX 4080 as the minimum for Isaac Sim 5.1.0) | `nvidia-smi` |
| Driver | NVIDIA tested Isaac Sim 5.1.0 with 580.65.06 on Linux; record whatever you have | `nvidia-smi --query-gpu=driver_version --format=csv` |
| RAM | 32 GB or more | `free -g` |
| Disk | 60 GB free (image about 20 GB, caches, build output under 1 GB) | `df -h .` |
| Docker and NVIDIA Container Toolkit | `docker run --rm --gpus all ubuntu nvidia-smi` prints your GPU | |
| Network | pulls from nvcr.io, Docker Hub (ros:jazzy-ros-base-noble), packages.ros.org, PyPI, github.com | |

Optional system check, about two minutes:

```bash
docker run --rm --gpus all --entrypoint bash nvcr.io/nvidia/isaac-sim:5.1.0 ./isaac-sim.compatibility_check.sh --/app/quitAfter=10 --no-window
```

It should print `System checking result: PASSED`.

## Pins

| Input | Value | Where it is pinned |
|---|---|---|
| Isaac Sim | 5.1.0, `nvcr.io/nvidia/isaac-sim:5.1.0`, importer `isaacsim.asset.importer.urdf` | `simulation/sim_pins.yaml`, `engine_overrides.yaml` |
| Expansion | bp000 `5d9a1b51aadd07b0f074beff37cc7ccacab91ec032eee63c39183ae2b79f40ea`, bp050 `4d21210ed0b2628f9c11be6312e81b33c58246be7ef8231084d3b6ef3c53eb8a` | `simulation/expansion_lock.yaml` |
| xacro | the same installation as CI: the hash-pinned PyPI wheel `xacro==2.1.1` from `simulation/requirements-xacro.txt` (upperbody-sw #104, Proposed); until #104 is in this branch, the lock's apt build `ros-jazzy-xacro 2.1.1-1noble.20261025.230251` | `simulation/expansion_lock.yaml`, `simulation/requirements-xacro.txt` |
| platform-sw, openarm_description | the commits of `upstream_pins.yaml`; the script clones them | `ros2/src/openamrobot_upperbody_description/config/upstream_pins.yaml` |
| Python packages | exact versions | `simulation/requirements.txt` |

## Run

From a clean checkout of openamr-upperbody-sw at the commit you were asked to run:

```bash
git clone https://github.com/openAMRobot/openamr-upperbody-sw && cd openamr-upperbody-sw
git checkout <commit>
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
bash simulation/isaac/run_on_gpu.sh --dry-run
bash simulation/isaac/run_on_gpu.sh
```

Running the script (without `--dry-run`) accepts the NVIDIA Isaac Sim EULA: the Isaac container is started with `ACCEPT_EULA=Y`
(and `PRIVACY_CONSENT=N`). Read the EULA of the image before you run it. The script takes no argument, `--dry-run` or `--cpu-only`;
any other argument prints the usage and exits 2 without running anything.

The dry run prints the pins and the plan and runs nothing. The full run takes about 20 to 40 minutes the first time (shader cache),
less afterwards. It writes everything to `build/isaac_kit/` and prints the bundle to send back with its sha256.

Steps of the script:

0. delete every output of an earlier run in `build/isaac_kit/` (`benchmark_isaac.json`, `isaac_result.json`,
   `isaac_comparison.md`, `isaac_rest.png` and, per configuration, `isaac_import_log.json`, `check_usd.json` and the USD files);
   if one cannot be deleted (owned by root), the script stops with BLOCKED;
1. preflight: GPU, driver, Docker and disk into `preflight.json`;
2. clone platform-sw and openarm_description at the pins;
3. in the ROS container: MJCF and the Isaac import job for bp000 and bp050 (`generate.py --isaac`; stops if the expansion is not the
   locked one), the MuJoCo benchmark, and `scenarios.json`;
4. in the Isaac container: `simulation/tools/isaac_generate_usd.py` per configuration (USD and `isaac_import_log.json`; stops unless
   the importer writes exactly one articulation);
5. in the Isaac container: `simulation/isaac/isaac_scenarios.py` on bp000 (`benchmark_isaac.json`, screenshot `isaac_rest.png`);
6. in the ROS container: `simulation/tools/check_usd.py` per configuration (`check_usd.json`) and
   `simulation/isaac/collect_result.py` (`isaac_result.json`, `isaac_comparison.md`);
7. the bundle `build/isaac_kit_evidence_<UTC>.tar.gz`.

## Expected output

Exit status of the script: `0` PASS, `1` FAIL, `3` BLOCKED, the status of `collect_result.py`. BLOCKED includes `--cpu-only`, no
GPU, a collector crash and a missing `isaac_result.json`; `2` is a usage or setup error. Each Isaac-side output carries the run's
stamp (`run_stamp`, the `utc` of `preflight.json`); `collect_result.py --stamp` reports any output with another stamp as BLOCKED,
so a container that dies before writing can never leave an earlier run's files counted as this run.

- `isaac_result.json` `status`: `PASS` when every check passed. The checks per configuration: `usd_generated`, `one_articulation`,
  `isaac_version` (the build starts with 5.1.0), `check_usd` (0 failures: links, joints, limits in degrees, masses, inertia, drives,
  armature, mimic, physics scene against the expansion), and for bp000 `dof_count` (21: 14 arm and 4 finger revolute joints, 2
  wheels, the lift; the castors are fixed joints in the URDF) and `benchmark.ran`.
- `isaac_comparison.md`: the seven benchmark scenarios, Isaac against MuJoCo (and against Gazebo when a Gazebo result is passed to
  `collect_result.py --gazebo`). The tolerances are those of `simulation/benchmark/scenarios.yaml`. A FAIL row there is a finding
  to explain; it does not change `status`.
- `isaac_rest.png`: the robot standing after the rest scenario (view from front left).

## Send back

The bundle `build/isaac_kit_evidence_<UTC>.tar.gz` and the sha256 the script printed, plus:

- the exact commit you ran (`git rev-parse HEAD`) and whether the tree was clean (also in `preflight.json`);
- the console output of the run if anything failed (the logs are in the bundle under `logs/`);
- optionally a short screen recording of the Isaac Sim viewer on the USD (`build/isaac_kit/sim/bp000/isaac/openamrobot_bp000.usd`).

Do not edit any generated file. Do not change any value to make a check pass: a failing check is the result.

## Comparison metrics

| Scenario | Metrics |
|---|---|
| rest | base height, pitch and roll after 3 s |
| joint_step | final and holding error of openarm_left_joint4 after a 1.0 rad move; base displacement |
| arm_reach | end-effector (`openarm_left_ee_base_link`) position in base_footprint; largest joint error |
| lift_move | final lift position, largest tracking error on the 0.018 m/s ramp, base pitch change |
| straight_drive | distance, lateral drift and yaw change after 0.20 m/s for 10 s |
| in_place_turn | yaw turned over yaw commanded, centre drift, one commanded revolution |
| object_contact | rest height, slip and speed of a dropped 1 kg box |

## Troubleshooting

| Symptom | Cause and what to do |
|---|---|
| `docker: could not select device driver "" with capabilities: [[gpu]]` | NVIDIA Container Toolkit missing or Docker not restarted after installing it |
| pull of nvcr.io fails with `unauthorized` | `docker login nvcr.io` with user `$oauthtoken` and an NGC API key |
| `BLOCKED: ... no lock entry` or exit 3 in step 3 | the checkout or a pin differs from the lock; run the commit you were given, unchanged |
| `apt-get install ros-jazzy-xacro=...` fails | packages.ros.org no longer serves the lock's build; this only happens before #104 is in the branch. Report it, do not install another build |
| `pip install --require-hashes ... requirements-xacro.txt` fails with a hash mismatch | PyPI served a different file; report it, do not edit the hash |
| `expected one articulation, the importer wrote N` | importer behaviour differs from 5.1.0; send the log, this is a result |
| `importer has no setting [...]` | the image is not 5.1.0 |
| permission denied writing `build/isaac_kit` | the Isaac container runs as another user; the script makes the folder writable, rerun after `sudo chown -R $USER build` |
| first start hangs for many minutes | shader compilation on first start; the caches under `~/docker/isaac-sim` make later runs faster |
| Isaac crashes at start (segmentation fault) | check the driver against NVIDIA's requirements page for 5.1.0 and run the compatibility check above |
| screenshot missing | rendering failed headless; the run still counts, note it when sending back |
