# Parking variability at the work-shelf station (analysis, 18 Nov 2026)

Diagnosis of the final station error of Baseline 1 from finished CI runs only. No acceptance run was dispatched for
it. Simulation evidence only: never a site, robot, physical or safety tolerance. The reach budget (0.04 m, 0.08 rad,
`station_approach.md`) is unchanged here.

## Verdict

- The station error is mostly a **systematic offset, not scatter**: in 21 runs with a full decomposition the base
  stopped short of the shelf in 20 and to the right of the station in 20. Mean error vector 47.7 mm (along -39.5 mm,
  left -26.7 mm); scatter around it 24.4 mm rms. The offset alone is larger than the 40 mm budget; 80 % of the mean
  square error is the offset.
- **Localisation dominates.** AMCL map pose against ground truth: mean 31.9 mm, rms 37.7 mm. Nav2's own residual
  against its goal: mean 24.5 mm, rms 26.7 mm. Both point the same way (short and to the right), so they add. With the
  localisation error removed 19 of 21 runs would be within 40 mm; with the Nav2 residual removed 16 of 21.
- **Why it varies:** at the station AMCL scores each particle on about 8 LiDAR beams, 4 of them on the shelf's 40 mm posts, and its
  likelihood is within 2 % over about -65 to +55 mm along the heading. The scan cannot pin the base to a few
  centimetres, so the final error is whatever the drive left (correlation 0.82 with the error at the start of the
  final approach). The final approach does not remove it.
- RTF, AMCL covariance, AMCL update count, arrival heading, time in the final approach and the CPU proxy do not
  explain the error (|r| at most 0.37, n = 21; |r| above 0.43 is the 5 % significance level).
- The station goal itself is exact (0 mm).
- Recommended correction: measure the final pose against the shelf (openamr-platform-sw #167 marker route) once its
  marker detection is reliable. In the 3 existing runs where the marker was detected, the station error was 19 to
  24 mm. Initial localisation before departure is not a factor: AMCL starts on the exact spawn pose.

## Runs

`baseline1-acceptance.yml` on `integration/baseline1`, all completed runs listed through the REST API (48 on the
workflow, 18 Nov 2026). Selected: `station_approach` 1 (the #164 `navigate_to_station.xml`), `station_alignment` 0
or absent (the input exists from 29adcb8), and the same Nav2 and AMCL configuration (openamr-platform-sw pins
182ee1b, 9dc0ab7 and fe28d19, AMCL `update_min_d` / `update_min_a` 0.05 m / 0.05 rad; their `nav2_params.yaml` differ
only in the bt_navigator server wait). The scene in the LiDAR plane is the same in all of them (the #167 marker plate is
above the LiDAR plane and absent with alignment 0).

- 24 runs: the 12 of `station_approach.md`; 16b673e 44128319717, 44128353096, 44128368373, 44128399817,
  44128412993; 2d462e8 44134830867; fe28d19 44186562560 (7bf19ce), 44194523618, 44194529196, 44194540813,
  44194552457 (6921a29), 44196259900 (aadde12).
- Full vector decomposition from the bag: 21 of them. The bags of 44110449067, 44128368373 and 44196259900 have no
  `/odom` and no odom > base_footprint on `/tf` (the recorder missed them); for those the job log's magnitudes are used.
- Excluded: 44128383301 (Nav2 never moved, 2.085 m: a start-up failure, not parking); 44100053272, 44100120091
  (AMCL 0.25 m / 0.2 rad, decomposed but kept apart); every `station_alignment` 1 run (29adcb8, the two of 6921a29,
  three of aadde12), summarised separately below; runs without the station tree.

## Method

- Job logs (`gh api repos/openAMRobot/openamr-upperbody-sw/actions/jobs/<id>/logs`): the mission quantities, the
  Nav2 controller and behaviour server events, `rtf_monitor`, control-loop warnings.
- Bags and `baseline1_mission.json` of the uploaded artifacts, analysed by the diagnostics workflow on branch
  `diag/parking-variability` (run 44225841420 at 10af235; scripts in `tools/parking_diag/` there; it downloads the
  artifacts and reruns nothing). At the end of `nav_to_station`, in the station frame (along: + towards the shelf;
  left: + to the robot's left):
  total = ground truth - goal = (Nav2 belief - goal) + (ground truth - Nav2 belief), exactly, per axis. Nav2 belief is
  map > base_footprint from `/tf`, ordered by availability (AMCL future-dates map > odom by 1.0 s).
- Check: the total reproduces the logged `nav_ground_truth_position_error` to 0.1 mm in all 23 bags. The two parts
  reproduce the logged magnitudes exactly where AMCL had not updated in the last 0.6 s, and within 11 mm where it had
  (the mission samples a fraction of a second after the last trace sample, and an AMCL update in between moves the
  belief). The split between (a) and (b) is uncertain by that much in those runs; the total is exact.
- Offline (no CI): `amcl_landscape.py` ray-casts the scene's LiDAR-plane geometry from the station, applies the
  simulation body filter (LiDAR -35 to +31 deg) and evaluates Nav2's likelihood-field score (60 beams, sigma_hit 0.2 m,
  z_hit 0.5, z_rand 0.5, distance capped at 2.0 m) on the map that `scene.py` generates. It reproduces the filtered
  returns at the station (38 against 32 to 41 measured).

## Decomposition (21 runs)

| Component | along mean / sd (mm) | left mean / sd (mm) | xy mean (mm) | xy rms (mm) | same sign |
|---|---|---|---|---|---|
| (a) Nav2 residual: belief against the goal (station goal checker 25 mm) | -17.7 / 15.4 | -9.5 / 9.4 | 24.5 | 26.7 | along 18/21, left 20/21 |
| (b) Localisation: ground truth against AMCL map pose | -21.8 / 18.4 | -17.2 / 18.4 | 31.9 | 37.7 | along 20/21, left 18/21 |
| (c) Station goal definition against the shelf | 0 | 0 | 0 | 0 | |
| **Total: ground truth against the station** | **-39.5 / 17.8** | **-26.7 / 16.7** | **49.5** | **53.3** | along 20/21, left 20/21 |

- Sum of squares over the 21 runs (mm^2): total 59704 = Nav2 14985 + localisation 29808 + cross term 14911. The
  cross term is positive because the two biases point the same way.
- Run-to-run variance per axis (mm^2): along: Nav2 225, localisation 323, total 302; left: Nav2 85, localisation 324,
  total 264. Localisation carries most of the variability; the Nav2 part partly compensates it (negative covariance:
  Nav2 drives to where AMCL says the goal is).
- Heading: total yaw error mean +20.7 mrad (sd 24.3), of which Nav2 +18.8 and localisation +1.9. Within the 80 mrad
  budget; not the problem.
- Within 40 mm: 7 of 21 as measured; 16 of 21 with (a) set to zero; 19 of 21 with (b) set to zero; 13 of 21 with
  only the mean of (a) removed; 19 of 21 with both means removed (residual scatter 23.8 mm rms).

### (a) Nav2 residual

- The station goal checker (0.025 m, stateful) accepts the first pose inside the circle. The approach comes from
  behind at `v_linear_min` 0.03 m/s, so the base stops near the near edge: along -17.7 mm on average.
- In 10 of 21 runs the residual exceeds 25 mm (up to 53 mm): the belief moved after acceptance, through an AMCL
  update (the last one 0.02 to 1.8 s before the measurement) or the base coasting under the velocity smoother. The bag
  analysis does not separate the two.
- 7 of 21 runs abort the final approach once with "Failed to make progress" (12 s within 0.05 m): in those the
  graceful controller turned 2.5 to 3.6 rad in total during the final approach (0.2 to 1.1 rad in the other runs) and
  spent 2.9 to 16.9 s with its belief within 5 cm of the goal; a 1 s wait and a 0.035 m replan end it. Cost about 12 s of Gazebo time; mean error 54.7 mm against 47.0 mm.

### (b) Localisation

- Before departure AMCL has one pose, seeded from `initial_pose` with zero covariance (sigma 0, 0, 0) on the exact spawn
  (spawn error 0.15 mm). The map pose is within 1 mm of ground truth at departure. There is no convergence step to
  improve.
- The error builds during the drive (53 to 57 AMCL updates over the 2.4 m and the 90 deg turn). Firmware odometry
  arrives 65.8 mm to the right of ground truth (heading error 33.5 mrad from the turn); AMCL removes most of it and
  ends 17.2 mm to the left of the truth, 21.8 mm ahead.
- At the start of the final approach (0.32 to 0.35 m out) the localisation error is already 38.2 mm on average; at
  the stop 31.9 mm. The final approach (6 to 24 updates) moves it by 22.8 mm on average without removing it.
- At the station the body-filtered scan has 32 to 41 returns. Offline: 16 from the four work-shelf posts (40 mm
  square), 15 from the two obstacle boxes 2.5 to 4.5 m away, 7 from aisle posts further out; with `max_beams` 60 of
  360 AMCL scores 8 of them (4 on the work-shelf posts). The generated map draws each post as 2 x 2
  cells (100 mm square, centroid 20 mm outward in both axes). Offline, the likelihood-field score stays within 2 % of
  its maximum over along -65 to +55 mm and left -10 to +55 mm of the true pose. The flat side is to the left of the
  truth, which is where AMCL ends (belief left of truth in 18 of 21 runs).
- AMCL's reported 1-sigma at the stop (56 / 110 / 71 mm, mrad) says the same: the filter does not claim more than it has.

### (c) Station goal definition

`scene.py` derives `stations.work_shelf` from the same `scene.yaml` the world is generated from: shelf centre y -3.000,
half depth 0.200, `parking_gap` 0.300, footprint front 0.415 gives y -2.085 and yaw -pi/2, the goal every run sends.
Ground truth is referenced to the spawn (spawn error 0.15 mm), and map and world share their origin. The goal adds no
error. The map rasterisation of the posts is a map difference, counted in (b).

### (d) Run-to-run factors

Pearson correlation with the station error (xy), n = 21; |r| above 0.43 is significant at 5 %.

| Factor | Range | r |
|---|---|---|
| localisation error at the start of the final approach | 7 to 75 mm | **+0.82** |
| largest localisation error during the drive | 20 to 87 mm | **+0.73** |
| path: furthest the base swung to -x in the turn | -97 to -35 mm | **-0.67** |
| CPU proxy: "Control loop missed its desired rate" warnings | 109 to 708 | +0.37 |
| RTF during the navigation | 0.12 to 0.27 | -0.29 |
| AMCL 1-sigma y at the start of the final approach | 97 to 150 mm | -0.34 |
| AMCL 1-sigma x at the start of the final approach | 53 to 483 mm | -0.26 |
| odometry lateral error at the stop | 51 to 79 mm | -0.41 |
| time in the final approach (Gazebo) | 6.3 to 22.1 s | +0.16 |
| heading error at the start of the final approach | 0 to 136 mrad | +0.12 |
| AMCL updates in the final approach | 6 to 24 | +0.05 |
| age of the last AMCL update at the stop | 0.02 to 3.5 s | +0.01 |
| no-progress abort in the final approach | 0 or 1 | +0.19 |

- The path swing is a symptom, not a cause: Nav2 steers the true base by its belief, so a belief error during the turn
  shows as a swing of the truth.
- No CPU load is recorded in these runs; the control-loop warnings are the only proxy, and they follow the RTF (the
  two runs at RTF 0.12 and 0.13 have 708 and 737). Lower RTF does not make the parking measurably worse.
- AMCL covariance does not predict the error (the sign is even reversed); the filter is equally unsure in good and
  bad runs.

## Station alignment runs (station_alignment 1, kept apart)

openamr-platform-sw #167 (Proposed, opt-in). The marker was detected in 3 of the 9 runs with a navigation result
(44155457759, 44155471421, 44155485209): station error 23.7, 19.4 and 22.6 mm. Its first measured residual in those
runs was 26 to 55 mm, and every corrected goal moved the base towards the shelf and to its left, the same offset as
above, measured independently. Not detected in 4 ("marker not seen: 0 samples in 6.0 s": 44155397328, 44155429472,
44155442866, 44196268806); Nav2 status 6 in 2 (44196280699, 44196292546). The two of 6921a29 did not build the #167
package (fixed at aadde12).

## Recommendations (40 mm budget unchanged, AMCL not tuned for this scene)

1. **Measure the final pose against the shelf: the #167 marker route.** It removes (a) and (b) together; the existing
   data shows 19 to 24 mm whenever the marker was seen. The open item is detection reliability (4 of 9 not seen in the
   6 s window, 2 Nav2 aborts), for the #167 owner. Until then Baseline 1 with alignment 0 cannot hold the 40 mm budget
   reliably in this scene: its offset alone is 47.7 mm.
2. **Final approach behaviour (Nav2 side, owner decision in openamr-platform-sw):** stop at the goal rather than at
   the near edge of the 25 mm circle, and keep the progress checker from aborting the final approach while the
   graceful controller turns in place. Expected effect from the data: about 18 mm of the along offset and the 12 s
   aborts in a third of the runs. Alone it gives 13 of 21 within 40 mm: necessary, not sufficient.
3. **More LiDAR-plane structure at the station is a world choice for the owners, not a fix here.** Offline, a 1.20 x
   0.30 m plate on the shelf front raises the beams AMCL uses from 8 to 19 but the score still stays within 2 % over
   about +-50 mm along and +-80 mm left; a 0.025 m map does not help either. With sigma_hit 0.2 m and 60 beams the
   likelihood is too flat to resolve 40 mm here, and changing those is AMCL tuning, excluded.
4. **Not recommended:** a localisation convergence step before departure (AMCL already starts exact; the error is
   built during the drive); offsetting the station goal by the measured bias (tuned to this route and scene, hides
   the cause); further repeated acceptance runs to describe the variability (21 decomposed runs already show the
   offset with 20 of 21 of the same sign).

## Verifying the correction with few runs

- Recommendation 1, after the detection fix: one head, 3 `station_alignment` 1 runs dispatched in parallel. Pass:
  the marker detected in all 3 (`samples` > 0 in the alignment report) and each station error at most 30 mm. With the 3
  earlier detected runs (mean 21.9 mm, sd 2.2 mm) a 30 mm ceiling keeps the one-sided 95 % prediction bound under
  40 mm. One missed detection fails the fix; do not add runs to dilute it.
- Recommendation 2, if adopted: the same 3 runs with alignment 0 on that head, decomposed by the diagnostics workflow
  (no mission rerun). Pass: Nav2 along mean within +-5 mm of 0 (now -17.7 mm) and no "Failed to make progress" in the
  final approach.
- Every later acceptance run can be fed to the diagnostics workflow; the decomposition needs no extra mission run.

## Per run

Total = Nav2 + AMCL per axis (mm, station frame). Yaw: total, signed. RTF: Gazebo over wall time during the
navigation. Runs marked (log) have no odometry in the bag: magnitudes from the job log.

| 44105673151 | a3eee67 | -61 / -40 / 73 | -16 / +0 | -45 / -40 | +25 | 18.8 | 8 | 1 | 0.22 |
| 44107726177 | a3eee67 | -67 / -44 / 80 | -28 / -1 | -38 / -43 | +33 | 6.9 | 6 | 0 | 0.12 |
| 44110378874 | 2c0b7c1 | -41 / -15 / 44 | -29 / -14 | -13 / -1 | +10 | 7.1 | 10 | 0 | 0.22 |
| 44110449067 | 2c0b7c1 | -32 / -39 / 50 | xy 20 (log) | xy 38 (log) | 23 (abs) | n/a | n/a | 0 | 0.13 |
| 44114692549 | d1c88be | -52 / -46 / 69 | -51 / -15 | -1 / -32 | -16 | 20.3 | 22 | 1 | 0.23 |
| 44114738411 | d1c88be | -23 / -13 / 27 | +15 / -5 | -39 / -7 | -7 | 9.8 | 13 | 0 | 0.21 |
| 44128319717 | 16b673e | -32 / -11 / 34 | -26 / -7 | -5 / -4 | +30 | 7.1 | 11 | 0 | 0.22 |
| 44128353096 | 16b673e | -62 / -21 / 65 | -25 / -11 | -36 / -9 | +10 | 6.5 | 10 | 0 | 0.15 |
| 44128368373 | 16b673e | -1 / -17 / 17 | xy 20 (log) | xy 16 (log) | 3 (abs) | n/a | n/a | 0 | 0.22 |
| 44128399817 | 16b673e | -56 / -65 / 86 | -15 / -1 | -42 / -63 | +2 | 6.3 | 6 | 0 | 0.22 |
| 44128412993 | 16b673e | -54 / -19 / 57 | +14 / -3 | -68 / -16 | +49 | 20.5 | 24 | 1 | 0.23 |
| 44130637433 | fc1fe22 | -43 / +1 / 43 | -25 / -8 | -18 / +9 | +64 | 19.4 | 17 | 1 | 0.22 |
| 44130671011 | fc1fe22 | -39 / -45 / 60 | -19 / -2 | -19 / -43 | +13 | 6.7 | 10 | 0 | 0.22 |
| 44130685870 | fc1fe22 | -34 / -41 / 53 | -14 / -24 | -20 / -17 | +36 | 7.1 | 12 | 0 | 0.22 |
| 44130700211 | fc1fe22 | -16 / -17 / 23 | -26 / -1 | +10 / -16 | +18 | 7.1 | 7 | 0 | 0.22 |
| 44130714431 | fc1fe22 | -57 / -21 / 61 | -22 / -8 | -35 / -13 | -14 | 9.8 | 10 | 0 | 0.25 |
| 44130728617 | fc1fe22 | -52 / -47 / 70 | -27 / -39 | -25 / -8 | -17 | 22.1 | 22 | 1 | 0.19 |
| 44134830867 | 2d462e8 | -38 / -31 / 49 | -31 / -11 | -7 / -19 | +22 | 7.3 | 10 | 0 | 0.22 |
| 44186562560 | 7bf19ce | +2 / -15 / 15 | +5 / -17 | -2 / +2 | +11 | 7.4 | 12 | 0 | 0.23 |
| 44194523618 | 6921a29 | -29 / -5 / 29 | -20 / -12 | -8 / +7 | +27 | 9.7 | 9 | 0 | 0.17 |
| 44194529196 | 6921a29 | -23 / -22 / 32 | -6 / -3 | -17 / -19 | +21 | 6.7 | 9 | 0 | 0.23 |
| 44194540813 | 6921a29 | -37 / -26 / 45 | -15 / -1 | -22 / -25 | +49 | 19.8 | 19 | 1 | 0.27 |
| 44194552457 | 6921a29 | -18 / -19 / 26 | -8 / -17 | -9 / -2 | +69 | 18.9 | 12 | 1 | 0.24 |
| 44196259900 | aadde12 | -8 / -34 / 35 | xy 17 (log) | xy 28 (log) | 17 (abs) | n/a | n/a | 0 | 0.22 |

## Not verified

- No new run was made: the recommendations are not yet shown on a run; the verification plan above is how.
- The offline likelihood model reimplements Nav2's likelihood field from its parameters; it is not AMCL itself, it uses
  the scene's geometry, not Gazebo's ray caster, and ignores the robot's own body beyond the filter.
- Why AMCL's residual has the same sign in nearly every run is not shown beyond the flat, one-sided likelihood at the
  station; AMCL's random sequence and the run timing may contribute.
- Whether the Nav2 residual above 25 mm comes from AMCL updates or from coasting after acceptance.
- Three runs have magnitudes only (no odometry in their bags).
- Real hardware, a real LiDAR and a real shelf: nothing here applies to them.
