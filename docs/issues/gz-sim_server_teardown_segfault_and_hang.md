<!-- Draft of the issue for https://github.com/gazebosim/gz-sim/issues/new . NOT FILED: the workspace has no GitHub credential (no gh, no token),
     so it is written to be pasted by someone who has one. When it exists, put its URL in docs/pr/PR14_BODY.md ("Known gaps") and here. -->

# Server stop: segfault in a libgz-sim8 thread during teardown, and a hang that ignores SIGTERM (gz-sim 8.15.0, gz_ros2_control, Bullet-Featherstone / DART)

## Environment

- Ubuntu 24.04.5 (WSL2), ROS 2 Jazzy: `ros-jazzy-gz-sim-vendor 0.0.13` (gz-sim **8.15.0**), `ros-jazzy-gz-physics-vendor 0.0.9`, `ros-jazzy-ros-gz-sim 1.0.24`,
  `ros-jazzy-gz-ros2-control 1.2.20`.
- A headless server (`gz sim -r -s -v 2`) with one robot model loaded by `ros_gz_sim create`, `gz_ros2_control` as a system plugin (a controller manager
  inside the server, five controllers active), two `ros_gz_bridge` `parameter_bridge` nodes and a few ROS nodes beside it. Physics: gz-physics
  Bullet-Featherstone (and DART in the last case below).
- `ros_gz_sim`'s launch file starts it as `/bin/sh -c "ruby .../gz sim ..."`, so a launch only ever signals the shell, never the server.

## What happens

The server is asked to stop, either with `gz service -s /server_control --reqtype gz.msgs.ServerControl --reptype gz.msgs.Boolean --req 'stop: true'`
(answered `data: true`) or with SIGINT sent to the server process. Normally it is gone in 1.3 to 1.8 s. Sometimes it is not:

1. **Segfault in a thread of libgz-sim8 during teardown.** Console output of the server (the process then lingers for seconds):

   ```
   Stack trace (most recent call last) in thread 12221:
   #5    Object "[0xffffffffffffffff]", at 0xffffffffffffffff, in
   #4    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x7bca1f329ecb, in
   #3    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x7bca1f29cb83, in
   #2    Object "/lib/x86_64-linux-gnu/libstdc++.so.6", at 0x7bca1a4ecdb3, in
   #1    Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8", at 0x7bca195608f0, in
   #0    Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8", at 0x7bca1956aa7c, in
   Segmentation fault (Address not mapped to object [(nil)])
   ```

   A null dereference in a `std::thread` body inside libgz-sim8 (no symbols, so no function name; the raw console log of these two launches was not kept).
2. **Silent hang.** The service answers `data: true`, and then nothing: none of the teardown output a normal stop prints (the `controller_manager`
   destructor lines) ever appears, the process stays alive, and it **ignores SIGTERM**. Only SIGKILL ends it. When the launch gives up on the shell
   (SIGTERM at 5 s) the server is left running as an orphan, holding its transport partition and about three cores.
3. **The same hang after SIGINT.** With SIGINT sent to the server first, then the `/server_control` request after 3 s, then SIGTERM, one G01 launch in
   49 still needed SIGKILL (9.2 s); the service request timed out in that run (`Service call timed out`).

## Counts (headless, one launch each, same machine, nothing else running)

| stop mechanism | launches | clean (1.3 to 1.8 s) | segfault in teardown | silent hang, SIGTERM ignored |
|---|---|---|---|---|
| `/server_control` request only | 18 | 14 | 2 | 2 (one also had a `parameter_bridge` abort "corrupted double-linked list" at the same moment) |
| SIGINT to the server | 24 | 24 | 0 | 0 |
| SIGINT, then request after 3 s, then SIGTERM / SIGKILL | 49 | 48 | 0 | 1 |

From the console logs of 154 earlier launches of the same stack (`/server_control` only): 14 server segfaults, and about 10 more launches where the server
was still there after the launch's 5 s escalation with no stack trace. Per-launch data: `evidence/shutdown_p17/timing_*.jsonl` in
https://github.com/Tushargoel230/openamr-upperbody-sw (branch `feat/a7-gazebo-moveit`).

**Not reproduced on a bare server:** `gz sim -r -s -v 2 --physics-engine gz-physics-bullet-featherstone-plugin empty.sdf` (no ROS, no model, no plugins of
ours), 6 s of running, then the same `/server_control` request: 30 of 30 stopped in 0.30 to 0.37 s, no segfault, no hang (and SIGINT: 30 of 30, in 0.05 s)
(`evidence/shutdown_p17/scripts/barerepro.py`, `evidence/shutdown_p17/logs/bare_empty_world_service_stop_30.txt`). So it needs something of the loaded
stack; `gz_ros2_control` is the obvious suspect and is not proven to be the cause.

## Reproduction

```bash
git clone https://github.com/Tushargoel230/openamr-upperbody-sw && cd openamr-upperbody-sw
git checkout 3bf4e1b          # the last commit whose sim.launch.py stops the server with the /server_control request only
# build the five packages under ros2/src with the platform-sw base (openamr-platform-sw, main) in a Jazzy workspace, then:
cd evidence/shutdown_p17/scripts          # shut_test.py + shutdrv.py + loop.sh (paths inside are this machine's)
N=24 PFX=r ./loop.sh                      # per launch: starts the stack under launch_testing, lets it run 25 s, stops it, times every process
```

`shutdrv.py` prints one JSON line per launch: the exit time of the `gz sim` processes after the stop, and any survivor. With the service request only, 4 of 18 launches went wrong
and about 1 in 6 of the earlier 154; the scripts are ad hoc and have this machine's paths in them, so adapt `loop.sh` and `shutdrv.py` before running.

## Expected

The server stops in about 1.5 s every time, or at least dies of SIGTERM when it does not.

## Notes

- The workaround in the project that found this: send SIGINT to the server pid, repeat with the service request after 3 s, SIGTERM and SIGKILL after 8 s, and
  report any server that needed a signal. That hides nothing from the project's tests but does not remove the hang.
- Happy to run anything on this stack; no debugger (gdb) is installed on the machine that saw it, so there is no backtrace with symbols.
