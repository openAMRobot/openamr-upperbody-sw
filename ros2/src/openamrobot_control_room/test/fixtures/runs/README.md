# Recorded run fixtures

Cut from the uploaded `build1_logs` of two simulation runs of this repository by
`.github/workflows/control-room-evidence.yml` (run 44442060716, tool `tools/extract_run_fixtures.py`, tar.gz
sha256 `468b10b96c6d1dc4db1eeca3996825ab5e99d42eace21593b64fda6004df43ef`). Nothing is edited or invented.

| Directory | Workflow run | Branch and commit | Result |
|---|---|---|---|
| `baseline1_44309817813` | Baseline 1 acceptance 44309817813 | integration/baseline1 d63e9ed, platform-sw 931e631 | all eight phases PASS |
| `grocery_44299879412` | Grocery dark store demo 44299879412 | demo/grocery-dark-store 6170e6a | mission FAIL |

Both runs: profile `firmware_sil`, `stm32: host` (the STM32 slot is the host test stand-in, not firmware),
emulated ZLAC8015D, CTL-017, Daly BMS and Damiao motors on Linux vcan, Gazebo physics (DART). `selection.json`
names the candump windows and bag messages that were kept. candump stamps are host wall time; bag times are the
recorder's simulation time (`ros2 bag record --use-sim-time`).

Simulation records only: not hardware, physical acceptance or safety evidence.
