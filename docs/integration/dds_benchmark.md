# DDS benchmark of the Baseline 1 workload (Proposed)

Status: **Proposed**. Not adopted; the benchmark is not dispatched. The middleware configuration of Baseline 1 is unchanged; nothing
here changes an acceptance threshold. Owners who decide: the software lead (middleware, contracts) and the navigation
owner (odometry, Nav2). Simulation evidence only: never hardware, physical acceptance, perception or safety evidence.

Branch `diag/dds-benchmark` from `integration/baseline1` at `d63e9ed`. New files only:
`.github/workflows/dds-benchmark.yml`, `tools/dds_benchmark/observer.py`, `tools/dds_benchmark/compare.py`,
`tools/tests/test_dds_benchmark.py` and this page. `baseline1-acceptance.yml`, the mission and the harness are not
changed.

## 1. Symptoms that started this

| Run | Workflow, commit | Symptom |
|---|---|---|
| 44247729473 | Baseline 1, #167 alignment on, `4a67ee7` | the alignment node received 0 station poses while the detector published them |
| 44247720886 | Baseline 1, #167 alignment on, `4a67ee7` | `odom` > `base_footprint` missing from the alignment node's TF buffer while base_state_adapter published it |
| #167 workflow | openamr-platform-sw | 30 of 214 camera images delivered between processes |
| 44242589231 attempts 1 and 2 | grocery demo, `8c77978` | `/odom` stood still for the whole run; AMCL never updated; the base drove on a frozen odom frame |
| 44299879412 | grocery demo, `6170e6a` | phase 9 `nav_odometry_minus_ground_truth` 0.907 m against a 0.56 m guard |

## 2. The /odom freeze of 44242589231 (attempts 1 and 2)

### What the logs and the bag show

Sources: the job logs of both attempts and the bag of attempt 2 (artifact `grocery-demo-attempt2`; the attempt 1
artifact is no longer listed), read by `tools/odom_freeze_diag/analyse.py` on the scratch branch `diag/odom-freeze`.

- **base_state_adapter was alive and publishing.** Started with pid 26670 at launch, logged its start line, finished
  cleanly at shutdown with the same pid. The mission's readiness check `ready: odometry` needs a received `/odom`,
  and Nav2 ran, which needs `odom` > `base_footprint`.
- **/odom was frozen at the spawn pose, not absent.** At the end of attempt 2 the mission's
  `nav_odometry_minus_ground_truth` is 2.6288 m and the ground truth final position is (2.376, 1.124): its distance
  from the origin is 2.629 m. The yaw difference 1.66017 rad equals the ground truth final yaw 1.660 rad. Attempt 1
  gives the same picture (2.70904 m, 1.66563 rad). `return_to_parking` succeeded in 1.1 s with zero settle time: Nav2
  believed the base was already at the origin.
- **The wheels turned.** Dead reckoning from the Gazebo wheel joint velocities (`/sil/gz/joint_states`, r 0.098 m,
  separation 0.400 m) follows ground truth within 0.05 m for the first 70 s of simulation time; the base drove 7.39 m
  and turned 45.0 rad in ground truth.
- **base_state_adapter integrated zero wheel speed from the start.** Its tick publishes on every 20 ms of Gazebo time
  once `/clock` runs, and integrates zero when no `/sil/base/odom` sample is fresh. It warns "no wheel speed sample"
  only after it has had at least one sample. No such warning appears in either attempt: its subscription to
  `/sil/base/odom` never received a sample.
- **The host client was alive.** stm32_host_client received `/cmd_vel` (the base moved over CAN1), and the mission
  received its `/sil/drive/status` (`ready: drive configured over CAN1`).
- **Clock gating did not hold the adapter**: it published `/odom` and the transform, which it does only with a
  non-zero `/clock`. Process start order: both nodes start within 30 ms of each other (pids 26668, 26670).
- **No middleware error is logged**: no `Failed to` other than shutdown, no SHM, RTPS, port or participant message in
  either attempt.
- **The bag recorder lost whole participants.** A topic of the recorder's list that it never discovers has no entry in
  the bag at all (a discovered topic without messages has count 0). Per run:

| Run | Not in the bag although in the recorder's list and published | Publishing process |
|---|---|---|
| 44242589231 attempt 1, attempt 2 | `/odom`, `/base/joint_states`, `odom` > `base_footprint` on `/tf`; `/sil/drive/status`, `/sil/lift/status`, `/sil/bms/status` | base_state_adapter; stm32_host_client |
| 44278919387 | `/sil/drive/status`, `/sil/lift/status`, `/sil/bms/status`, `/sil/base/odom` | stm32_host_client |
| 44299879412 | `/odom`, `/base/joint_states`, `odom` > `base_footprint` | base_state_adapter |
| 44247720886 | `/odom` | base_state_adapter (the alignment node also lacked its transform) |
| 44247729473 | `/sil/drive/status`, `/sil/lift/status`, `/sil/bms/status` | stm32_host_client |

Every bag misses at least one Python node's participant entirely. The recorder starts after readiness, as one of the
last participants on the host.

### Most likely cause: the LOCALHOST discovery range caps the participants that can find each other

`build1_harness.py` sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` for every scenario and mission test. In ROS 2 Jazzy:

- rmw_fastrtps (`rmw_fastrtps_shared_cpp/src/participant.cpp`, branch jazzy) clears the multicast discovery locators,
  replaces the built-in transports by SHM plus UDP with `maxInitialPeersRange = 32`, and adds `127.0.0.1` as the only
  initial peer. Fast DDS expands a peer without a port to the discovery ports of participant indices 0 to 31 only.
  A participant announces itself only to those ports and to participants that announced themselves to it. Two
  participants that both have an index of 32 or more never announce to each other and never match. Fast DDS logs
  nothing.
- rmw_cyclonedds (`rmw_cyclonedds_cpp/src/rmw_node.cpp`, branch jazzy) sets `MaxAutoParticipantIndex` 32 for the same
  range: a participant beyond it is not created ("Failed to find a free participant index").
- Fast DDS 2.14 applies `FASTDDS_BUILTIN_TRANSPORTS` only when the built-in transports are in use
  (`RTPSParticipantImpl.cpp`); under LOCALHOST rmw_fastrtps turns them off, so `LARGE_DATA` has no effect there.

The Baseline 1 and grocery launches start about 40 ROS processes, plus the test process, the recorder, the camera
bridge and controller spawners. Participant indices go by the order in which participants bind their discovery port.
The C++ Nav2 servers create theirs within about a second; the Python nodes take about five seconds (base_state_adapter
logs its first line 4.7 s after its process starts). The late indices therefore go mostly to Python nodes, and which
two of them end up both at 32 or above changes from run to run. That matches every symptom above: the adapter missing
the host client (the freeze), the alignment node missing the detector or the adapter, and the late recorder missing
one or two of these nodes in every run, while the test process (index near 0) always sees everything.

The mechanism was measured on a hosted `ubuntu-24.04` runner (scratch branch `diag/odom-freeze`, workflow
`diag-discovery-cap.yml`, Fast DDS cells of run 44316085344, Cyclone cell of run 44314480882): 40 small participants, each publishing a test topic of its own, no robot
stack, started one after the other after the observer (index 0), with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` as
the harness sets it.

| Setting | Members at index 1 to 31 | Members at index 32 to 40 | Fresh participant after 40 s |
|---|---|---|---|
| Fast DDS 2.14.7, rmw_fastrtps_cpp 8.4.5, unchanged | see all 39 others | see the 31 low members, none of each other | sees 33 nodes; `/cap/p31`, `/cap/p35`, `/cap/p39` without publisher |
| same, `discovery_cap` raised (XML initial peers 0 to 119) | see all 39 | see all 39 | sees 42 nodes, every publisher |
| Cyclone DDS 0.10.5, rmw_cyclonedds_cpp 2.2.4, unchanged | created | not created: "Failed to find a free participant index for domain 119" | not created |

Fast DDS logged nothing in the unchanged case.

This is a cause in the middleware configuration, not in the load: RTF and CPU decide only the order of the indices.
The camera loss (30 of 214 images) is a different symptom (partial delivery between matched participants) and is
not explained by it; the benchmark measures it.

### The diagnostic that would confirm it in the stack

A snapshot of the participant index of every process (`ss -u -a -n -p`: discovery port
`7400 + 250 * domain + 10 + 2 * index`) taken while the stack runs, in a run that freezes: base_state_adapter and
stm32_host_client both at index 32 or above, and in a run that does not, at least one of them below 32. The benchmark
observer records this snapshot every 30 s in every cell (`participants` in `observer_summary.json`). The confirming
intervention is one cell with `discovery_cap=raised`: the gaps disappear if the cap is the cause. That changes the
middleware configuration of that cell, so it waits for the owners.

## 3. The 0.907 m odometry deviation of 44299879412

Wheel slip in the simulation, not odometry loss.

- `/odom` and `/sil/base/odom` kept their rate through phase 9: the mission's odometry lines show `/odom` and
  `/sil/base/odom` counts rising by about 250 per 5 s of simulation time (50 Hz) with ages under 0.4 s.
- Dead reckoning from the Gazebo wheel joint velocities alone (no CAN, no DDS, no adapter) gives the same error as
  dead reckoning from `/sil/base/odom`: 0.945 m and 0.906 m at 178.8 s, yaw error +0.19 rad in both. The deviation is
  already in the wheel motion against the body motion.
- It is a heading error: the distance along the path agrees within 1 %. The wheels imply 20.32 rad of rotation while
  the body turned 19.84 rad (2.4 % less); 44278919387 shows 3.7 % and 44242589231 attempt 2 3.6 %. The error grows
  during the in-place turns at the work shelf (yaw error 0.09 rad after the turn at 37 to 41 s) and the 4 to 5 m
  westward leg of phase 9 turns 0.15 rad into about 0.6 to 0.9 m of lateral error.
- 44278919387 (not frozen) failed the same guard in phase 9 with 0.744 m against 0.564 m.

The guard is derived from 0.10 m per 2 m driven and does not model heading error from turning slip; that is an
observation for the navigation owner, not a proposal to change it. The track (0.400 m) is a placeholder in the
description (decision DRIVE-TRACK open); an effective track 2 to 4 % wider than the kinematic one in DART is what the
numbers show.

## 4. Benchmark design

### Workload

The Baseline 1 mission exactly as `baseline1-acceptance.yml` at `d63e9ed` runs it: same apt source, Gazebo vendor
pins, platform pin `931e631`, `config/pins.yaml`, seed 20261117, profile sim-oracle, station approach 1, the
`test_baseline1_mission` launch test. Input `station_alignment` 1 (default, measures the camera and the station pose)
or 0 (Baseline 1 as accepted). Every cell runs on `ubuntu-24.04` hosted runners, the type Baseline 1 uses.

### Settings

| Setting | Environment | Expected under the unchanged harness (LOCALHOST) |
|---|---|---|
| `fastdds_default` | `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` | as Baseline 1 today |
| `fastdds_large_data` | plus `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` | same transports as default (section 2); a second sample of the default |
| `cyclonedds` | `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | participants beyond index 32 are not created; the mission likely fails at STARTUP |

`ros-jazzy-rmw-cyclonedds-cpp` is installed in the cyclonedds cell only (Eclipse Cyclone DDS: Eclipse Public License
2.0 or Eclipse Distribution License 1.0; rmw_cyclonedds: Apache-2.0). `python3-psutil` (BSD-3-Clause) for the
sampler and `pytest` 8.3.3 (MIT, in a throw-away venv of the plan job) are the other benchmark-only dependencies.

Input `discovery_cap` (default `unchanged`). `raised` lifts the cap to 120 in that dispatch only: a Fast DDS XML
profile with explicit `127.0.0.1` initial peers for indices 0 to 119 of the harness's domain (rmw_fastrtps keeps a
profile's initial peers and adds its own), and `CYCLONEDDS_URI` with `MaxAutoParticipantIndex` 120 (read after the
rmw's own fragment). For the owners' decision; not to be dispatched before it.

### What each cell measures

The observer (`tools/dds_benchmark/observer.py`) starts 10 s before the mission in the harness's domain (the harness's
first draw of `random.randint(101, 200)` after `random.seed(BASELINE1_SEED)`: 119) with the same discovery range, so it
takes participant index 0 and sees the whole graph. It publishes nothing: no rosout, no parameter services, no command
topics.

| Metric | How |
|---|---|
| QoS of the key topics | every publisher and subscriber of `/odom`, `/tf`, `/scan`, `/joint_states`, `/sil/base/odom`, the camera (`/base_camera/color/image_raw` with alignment, `/base_camera/rgbd/image` without), `/station_alignment/station_pose`, `/apriltag/detections`: reliability, durability, history, depth, liveliness, from the graph every 30 s |
| Late view | 120 s after the first `/odom`: `ros2 topic info --verbose --no-daemon --spin-time 15` for each key topic, and a fresh participant spun 15 s; publishers the observer sees and the late participant does not are listed |
| Participants | `ss -u -a -n -p` every 30 s: participant index of every process (its discovery port), count and highest index |
| CPU, memory | psutil every 5 s: CPU % and RSS per process, total CPU and used memory (`processes.csv`) |
| Delivery | observer subscriptions with the publishers' reliability (reliable when every publisher is reliable, else best effort), volatile, depth 200. Received unique stamps against the count the publisher's stamp sequence implies (span / median period + 1), per stream (`/tf` per parent > child). An estimate of the publisher-side count, not a publisher counter. The #167 detector's own `published=` counts (publisher side of the station pose) and the alignment node's `received` count are read from the mission console |
| Latency | header stamp to receipt: in simulation time (the last `/clock` the observer had) and in wall time (receipt against the wall time at which `/clock` reached the stamp); p50, p95, max |
| RTF over time | from the observer's `/clock`: simulation seconds per wall second in 30 s windows |
| Mission | `baseline1_report.py` phase table, as in the acceptance run |
| Console | counts and examples of middleware error lines, Cyclone's participant index message, base_state_adapter's stale warnings |

The observer is one more participant and one more subscriber of every key topic (including the camera). It shifts the
other participants' indices by one and adds load; that is the same in every cell, and the benchmark is not an
acceptance run.

The `compare` job downloads each cell's artifact (without bags and frames) and prints one table to the job summary
and `comparison.json`. Each cell also prints its own row.

### How to dispatch

```bash
gh workflow run dds-benchmark.yml -R openAMRobot/openamr-upperbody-sw --ref diag/dds-benchmark \
  -f station_alignment=1 -f settings=fastdds_default,fastdds_large_data,cyclonedds -f repeats=1 -f discovery_cap=unchanged
```

`repeats=2` runs each setting twice (hosted runners vary from run to run). Expected runtime per cell: 25 to 45 min
when the apt mirrors are fast (setup about 8 min, build 1 min, mission 10 to 30 min at RTF near 0.1), up to 2 h on a
slow mirror day as on 18 Nov 2026; the cells run in parallel.

## 5. How the results will be judged

The owners judge; the table only reports. Readings agreed in advance:

1. **Discovery cap.** If in any cell a pair of communicating processes is missing from each other (late view, station
   pose received 0, odom transform missing, stale `/odom`), and both have participant index 32 or above in that
   cell's snapshot, while no pair with an index below 32 is affected, section 2 is supported. A missing link with one
   end below 32 contradicts it.
2. **Fast DDS default against LARGE_DATA.** Expected identical within runner variance. A difference beyond that means
   the transports are not what section 2 says and the source reading is wrong.
3. **CycloneDDS.** Expected to fail at STARTUP with the participant index message. If it runs the whole mission, the
   cap did not bind (fewer than 33 participants or a different configuration), recorded as such.
4. **Delivery and latency.** Compared per topic across cells: a ratio below 1.0 on a reliable stream to the index-0
   observer, or a camera ratio that differs between transports, is a transport finding independent of the cap.
5. **Load.** RTF median and minimum, total CPU and memory, and the top processes per cell; a setting that changes RTF
   by more than the spread between repeats is a load finding.
6. **Mission.** The phase table per cell is reported, never used to make Baseline 1 complete.

A middleware change for Baseline 1 (discovery range, cap, RMW or transports) is a separate contract change request
after this review.

## 6. Verification so far

- Unit tests of the observer and comparison (no ROS): `python3 -m pytest -q -p no:cacheprovider
  tools/tests/test_dds_benchmark.py`, 11 passed locally; with `tools/dds_benchmark` removed, collection fails.
- `actionlint` 1.7.7 on the workflow: no findings.
- Discovery-cap probe, runs 44316085344 and 44314480882 (section 2): three cells, as in the table. The same run is the smoke run of
  the observer on Fast DDS and Cyclone DDS (index 0, participant snapshots, QoS, received counts, late probe,
  `processes.csv`). Its graph reports history `UNKNOWN` and depth 0 for remote endpoints: discovery does not carry them,
  so the QoS table has reliability, durability and liveliness from the graph only.

### Not verified

- The benchmark has not been dispatched; no cell result exists.
- The participant index of base_state_adapter and stm32_host_client in the frozen runs was not recorded. The cap is
  measured on the probe, not in the stack; that it caused the freeze is the most likely explanation, not a measured one.
- The observer has run on the probe, not yet next to the Baseline 1 stack; its image and `/tf` paths are untested there.
- Whether the twist the host client published in 44242589231 was non-zero (its topics are not in that bag).
- The `discovery_cap=raised` profile has not run in the stack.
- Physics cause of the turning slip (contact, caster, DART friction) is not isolated.
