# G01 wall time and simulation time

G01 keeps its 90-second CTest budget, default DART engine, 0.125 ms world
step, camera streams, sensor rates and every existing assertion. The full
Bullet acceptance remains a separate run; G01 is not substituted for it.

## Diagnosis (1 November 2026)

Official #26 at `a0b2b279276d1aaec8afd637096069ba0ce59ac7` completed
[hosted G01 in 45.18 seconds](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/41272973424/job/124749352946).
Its three-simulation-second arm goal took 4.84 wall seconds. #27 at
`aca3ea2283fe7d7c95d8fde0359a9be0ecbaa6ea`, with platform
`a336cb042e385999db5f1335ae58826369784610`, passed checks 1–4 and
[timed out at 90.03 seconds](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/41287166518/job/124790700048).
That arm goal took 26.67 wall seconds. The buffered log cannot establish
clock progress after check 4; it does not prove a frozen clock or a failed
base assertion. Neither a DDS discovery warning nor an image SHM allocation
error appears in that G01 log.

A local instrumented copy of the unchanged #27 test measured:

| Check | Wall seconds | Simulated seconds | Sim/wall factor |
|---|---:|---:|---:|
| 1 controllers, including startup | 18.441 | 3.231 | 0.175 |
| 2 arm plus settling | 15.243 | 3.509 | 0.230 |
| 3 topic discovery | 1.007 | 0.245 | 0.243 |
| 4 base plus settling | 15.427 | 3.500 | 0.227 |
| 5 TF observation | 20.137 | 5.003 | 0.248 |
| 6 joint-state observation | 8.053 | 2.000 | 0.248 |

ROS clock samples every 250 ms showed continuous progress after startup,
with a steady interval factor of 0.239. Native Gazebo world statistics
also advanced: 139,864 iterations at 17.483 simulated seconds. The current
0.125 ms step requires 8,000 iterations per simulated second, versus
#26's 1,000 at 1 ms. This is a substantially larger stepping workload,
in addition to the complete camera workload. The local reproducer is slow,
not stalled. These measurements do not isolate renderer CPU time from
physics/update/transport time, so they do not establish rendering as the
sole cause. The identified harness problem is seven redundant simulated
seconds of serial observation, costing 28.19 wall seconds locally.

## Harness repair

TF and all three joint-state streams are subscribed after all controllers
are active, before either motion. Every received sample is retained until
its check. Checks 5 and 6 still require at least five and two simulated
seconds respectively, waiting for any remainder. Duplicate/out-of-order
TF stamps, rates, publisher identities, duplicate joints, joint ownership,
description and motion assertions are unchanged. Observation now includes
the actual arm and base motions. No stream is disabled or downsampled.

The first corrected local CTest run completed all nine G01 cases in
49.00 seconds. Both observation windows covered 7.278 simulated seconds;
364 odom transforms measured 50.0 Hz. Checks 5 and 6 took 0.021 and 0.048
wall seconds to evaluate the already collected samples. This is a harness
timing improvement, not a simulation speedup or a physical acceptance claim.

Simulation-time waits have an absolute wall deadline and fail after five
wall seconds without clock progress. The assertion phase ends at 84 wall
seconds from launch-description generation to leave teardown time inside
the existing 90-second CTest bound. Other waits retain their existing
limits, capped by the same remaining budget. A failed drive wait sends a
zero command in `finally`. No shutdown exception allowance is added.

`G01_TIMING` start/progress/end lines are flushed with per-check wall time,
simulation-clock endpoints, sim/wall factor and maximum observed clock
gap. Progress is written every five wall seconds while spinning. The
sidecar is also checkpointed; incomplete measurements remain FAIL.
Clock gaps are gaps observed by the test executor, including time it was
not spinning; a zero factor during a short evaluation phase is not a
frozen-clock verdict. The explicit clock watchdog diagnoses stalled waits.

## Regression and reproduction

From a built, sourced workspace:

```bash
python3 -m pytest -q src/openamr-upperbody-sw/ros2/src/openamrobot_upperbody_gazebo/test/test_g01_clock_guard.py
ctest --test-dir build/openamrobot_upperbody_gazebo -V -R '^test_test_g01_gazebo_smoke.py$'
ros2 run openamrobot_upperbody_description verify_a6.sh
ros2 run openamrobot_upperbody_bringup run_a7_regression.sh
```

The 12 focused cases pass. In isolated copies, restoring the original
unbounded loop makes the frozen-clock subprocess regression fail at its
two-second outer guard; restoring serial waits causes two window tests
to fail; removing the drive `finally` causes the zero-command regression
to fail. These controlled failures are regression evidence, not passing
simulation runs. Invalid/backwards clocks, slow advancing clocks, full
duration, bad early TF/joint samples and expired motion commands are covered.

Exact-head local A6/A7 and hosted A7 results belong in #27's Tests section
and workflow artifacts. Local timings alone do not satisfy hosted acceptance.
