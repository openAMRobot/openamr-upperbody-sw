# tools/sim2real: bench-week sim-to-real scripts (Proposed)

The protocol is [docs/sim_to_real_protocol.md](../../docs/sim_to_real_protocol.md). Read its safety section before running anything with `--target robot`.

What the scripts are: five rclpy scripts that send goals and Twists through controllers that already exist, record what was commanded and what
was measured, and write one session directory per run; a comparison script; and blank CSV sheets for the measurements a person takes with a tape
or calipers. They never bypass a stop. With `--target robot` they ask the operator to type `yes` before every motion, and on Ctrl+C they cancel
their goals and send zero velocity. `--dry-run` sends nothing and logs what it would have sent.

Dependencies: ROS 2 Jazzy (rclpy, control_msgs, nav_msgs, geometry_msgs), numpy and PyYAML. `compare.py` needs only PyYAML.

## Simulation reference

One simulator at a time, private domain and partition:

```bash
export ROS_DOMAIN_ID=45 GZ_PARTITION=sim2real
source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash
ros2 launch openamrobot_upperbody_gazebo sim.launch.py headless:=true base_plate:=bp000          # DART (default)
# wait until `ros2 control list_controllers` shows the six controllers active, then, in this order:
python3 tools/sim2real/t4_arm_tracking.py  --target sim --session-root sessions/sim     # ends with both arms in park
python3 tools/sim2real/t1_lift.py          --target sim --session-root sessions/sim --park-first
python3 tools/sim2real/t2_base_straight.py --target sim --session-root sessions/sim
python3 tools/sim2real/t3_base_rotate.py   --target sim --session-root sessions/sim
python3 tools/sim2real/t5_gripper.py       --target sim --session-root sessions/sim --physics gz-physics-dartsim-plugin
```

For T5 also run on Bullet-Featherstone (DART does not move the grippers on main, #101):
`sim.launch.py ... physics_engine:=gz-physics-bullet-featherstone-plugin`, then `t5_gripper.py --target sim --physics gz-physics-bullet-featherstone-plugin`.

In `sim` the scripts also read Gazebo ground truth (`gz model -m openamrobot -p` / `-l <link>`, the `- Pose` block). Odometry is recorded and is
never the reference.

## Robot (bench week)

Names that this repository does not define for the real robot are required arguments with no default. Confirm each one on the bench before you
write it on the command line (protocol, table "Interfaces"):

```bash
python3 tools/sim2real/t4_arm_tracking.py  --target robot --operator <name> --session-root sessions/robot
python3 tools/sim2real/t1_lift.py          --target robot --operator <name> --lift-state-topic <topic|none> --session-root sessions/robot
python3 tools/sim2real/t2_base_straight.py --target robot --operator <name> --cmd-vel-topic <topic> --odom-topic <topic> --session-root sessions/robot
python3 tools/sim2real/t3_base_rotate.py   --target robot --operator <name> --cmd-vel-topic <topic> --odom-topic <topic> --session-root sessions/robot
python3 tools/sim2real/t5_gripper.py       --target robot --operator <name> --session-root sessions/robot
python3 tools/sim2real/t5_gripper.py       --target robot --operator <name> --fractions 1,0.75,0.5,0.25,0 --session-root sessions/robot   # the #79 table
```

Run each one with `--dry-run` first: it prints the session directory, and `sent.jsonl` in it lists every goal and Twist it would have sent.
`t1_lift.py --target robot` sends nothing at all: the operator moves the lift through the authoritative lift controller and presses Enter at the
start and end of each move.

Copy `templates/*.csv` into a `manual/` directory for the session and fill them in at the bench.

## Compare

```bash
python3 tools/sim2real/compare.py --sim sessions/sim/* --robot sessions/robot/* --manual manual/ --out report.md
```

`report.md` lists every banded metric with the sim value, the robot value, the bench value, the delta and the verdict (PASS, FAIL, MISSING), then
every metric. Exit 0 all PASS, 1 a FAIL, 2 something MISSING. The bands are `bands.yaml`: **all Proposed**, owner to accept.

## Session layout

```
<session-root>/<UTC stamp>_<test>_<target>/
  session.yaml      target, dry_run, operator, argv, config_id (print_config_id.py) and its command, git SHA and dirty flag of each repository,
                    ROS_DOMAIN_ID, GZ_PARTITION, caps, every clamp applied, the AD-30 park check
  sent.jsonl        every goal / Twist sent (or, with --dry-run, that would have been sent)
  metrics.csv       test, metric, value, unit, source: what compare.py reads
  <test>.csv        the per-step table of the test
  joint_states.csv  /joint_states (position, velocity, effort) of the joints the test touches; odom.csv for T2 and T3
```

## Tests

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest tools/sim2real/tests -q -p no:cacheprovider
```

No ROS needed: `compare.py` on fixture sessions (pass, fail, missing file), the speed-cap clamping, and `--dry-run` sending nothing (rclpy
replaced by stubs).
