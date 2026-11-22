# simulation

MuJoCo and Isaac Sim representations of the combined model, generated from the pinned gazebo expansion (issue #58).
Everything is in [docs/simulators.md](../docs/simulators.md): what each backend models, the commands, the tests and the findings.

| Path | Content |
|---|---|
| `openamr_sim/expand.py` | Expansion of a named configuration and its check against `expansion_lock.yaml` |
| `openamr_sim/mjcf.py` | MJCF generator |
| `openamr_sim/parity.py` | Structural parity with the expansion; tolerances in `parity_tolerances.yaml` |
| `openamr_sim/physics.py` | MuJoCo physics checks and pinocchio gravity torques |
| `openamr_sim/openarm_compare.py` | Arm comparison with enactic/openarm_mujoco |
| `openamr_sim/isaac.py` | Isaac import job and the USD checker |
| `tools/` | `generate.py`, `isaac_generate_usd.py` (inside Isaac Sim), `check_usd.py`, `compare_openarm_mujoco.py`, `mirror_lock.py` |
| `tests/` | pytest suite, run by `.github/workflows/simulators.yml` |

The backend values that URDF cannot carry are in
`ros2/src/openamrobot_upperbody_description/config/sim_dynamics/engine_overrides.yaml`. Generated files go to `build/` and are
never committed or edited. `models/`, `scenarios/` and `worlds/` are unchanged placeholders.
