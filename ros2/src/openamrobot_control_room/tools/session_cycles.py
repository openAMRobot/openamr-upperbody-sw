#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Start and stop a session of processes several times in one job; fail on anything a stop leaves behind.

    session_cycles.py SPEC.yaml [--summary FILE]

SPEC.yaml:
    cycles: 3
    stop_signals: [INT, TERM, INT]        # one per cycle (repeated if shorter): the signal of that cycle's stop
    processes:                            # started in this order, stopped in the reverse order
      - name: control_room
        command: [ros2, launch, openamrobot_control_room, control_room.launch.py, rviz:=false]
        ready: {topic: /control_room/status, type: std_msgs/msg/String}   # or {line: "regex on its output"}
        ready_timeout: 120
        stop_timeout: 60

Each process runs in its own session; the stop sends the cycle's signal to its process group (as Ctrl-C does to a
terminal's foreground group) and waits for the leader. Every process started gets CONTROL_ROOM_SESSION=<id> in its
environment, inherited by everything it starts. After each cycle the check fails on:
  - a live process that carries the marker (an orphan of the stop; sudo drops the environment, so the name list below
    is the second net), or a new live process whose command line names a known stack program;
  - a ROS 2 daemon that did not exist before the cycle (a stale daemon left by a ros2 command of the session);
  - a vcan interface that did not exist before the cycle (created by the session and not deleted);
  - a process that never became ready (the component did not run: a cycle with nothing running is a failure).
Leftovers are killed after they are reported, so the next cycle starts clean. Exit 0 only if every cycle passed.
Simulation tooling only; not hardware, physical acceptance or safety evidence.
"""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
import uuid

MARKER = "CONTROL_ROOM_SESSION"
# Programs of the stack and the control room: a new live one after a stop is a leftover even without the marker.
KNOWN = ("control_room_monitor", "vcan_monitor", "control_room_replay", "foxglove_bridge", "rosbridge_websocket",
         "rviz2", "gz sim", "ruby", "ros2 launch", "ros2 bag", "_ros2_daemon", "controller_manager", "ros2_control_node",
         "move_group", "parameter_bridge", "bt_navigator", "controller_server", "planner_server", "amcl",
         "live_demo_wait", "gz_follow", "candump")
DAEMON = "_ros2_daemon"


# ---------------------------------------------------------------------------------------------- the process table
def processes(proc_root="/proc"):
    """{pid: {"cmd": str, "env": {..}, "ppid": int}} of every process whose files this user can read."""
    out = {}
    for entry in os.listdir(proc_root):
        if not entry.isdigit():
            continue
        base = os.path.join(proc_root, entry)
        try:
            with open(os.path.join(base, "cmdline"), "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            continue
        env = {}
        try:
            with open(os.path.join(base, "environ"), "rb") as f:
                for item in f.read().split(b"\0"):
                    key, sep, value = item.partition(b"=")
                    if sep:
                        env[key.decode(errors="replace")] = value.decode(errors="replace")
        except OSError:
            pass                       # another user's process (root via sudo): only its command line is read
        ppid = 0
        try:
            with open(os.path.join(base, "stat"), encoding="utf-8", errors="replace") as f:
                ppid = int(f.read().rsplit(")", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            pass
        if cmd:
            out[int(entry)] = {"cmd": cmd, "env": env, "ppid": ppid}
    return out


def leftovers(before, after, session_id, own_pids=()):
    """Processes of `after` that the session left: carrying its marker, or new and named like a stack program."""
    found = {}
    for pid, info in after.items():
        if pid in own_pids:
            continue
        if info["env"].get(MARKER) == session_id:
            found[pid] = dict(info, why="carries the session marker")
        elif pid not in before and any(name in info["cmd"] for name in KNOWN):
            found[pid] = dict(info, why="new process of a known stack program")
    return found


def new_daemons(before, after):
    return {pid: info for pid, info in after.items() if DAEMON in info["cmd"] and pid not in before}


def vcan_interfaces(sys_net="/sys/class/net"):
    """Names of the vcan interfaces (their sysfs type is 280, ARPHRD_CAN, and there is no device behind them)."""
    names = []
    for name in sorted(os.listdir(sys_net)) if os.path.isdir(sys_net) else []:
        try:
            with open(os.path.join(sys_net, name, "type"), encoding="utf-8") as f:
                kind = f.read().strip()
        except OSError:
            continue
        if kind == "280" and not os.path.exists(os.path.join(sys_net, name, "device")):
            names.append(name)
    return names


def cycle_verdict(record):
    """(ok, reasons) of one cycle record."""
    reasons = []
    for name, ready in record["ready"].items():
        if not ready.get("ok"):
            reasons.append(f"{name} never became ready ({ready.get('detail')}): the component did not run")
    for name, stop in record["stopped"].items():
        if not stop.get("exited"):
            reasons.append(f"{name} did not exit within {stop.get('timeout')} s of SIG{record['signal']}")
    for pid, info in record["leftovers"].items():
        reasons.append(f"orphan {pid} ({info['why']}): {info['cmd'][:160]}")
    for pid, info in record["daemons"].items():
        reasons.append(f"stale ROS 2 daemon {pid}: {info['cmd'][:160]}")
    for name in record["vcan_left"]:
        reasons.append(f"vcan interface {name} created during the cycle and not deleted")
    return not reasons, reasons


# ---------------------------------------------------------------------------------------------- running a cycle
READY_PROBE = r'''
import sys, time, rclpy
from rosidl_runtime_py.utilities import get_message
topic, kind, timeout = sys.argv[1], get_message(sys.argv[2]), float(sys.argv[3])
rclpy.init(); node = rclpy.create_node("session_cycles_ready_probe", start_parameter_services=False)
got = []
node.create_subscription(kind, topic, lambda m: got.append(1), 10)
deadline = time.monotonic() + timeout
while not got and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node(); rclpy.try_shutdown()
print(f"{topic}: {len(got)} message(s)"); sys.exit(0 if got else 1)
'''


def start(spec, session_id, log_dir, index):
    env = dict(os.environ, **{MARKER: session_id})
    log = open(os.path.join(log_dir, f"cycle{index}_{spec['name']}.log"), "w", encoding="utf-8")
    # A new session: the process group can be signalled as a terminal would; SIGINT is not ignored in it.
    command = [os.path.expandvars(str(part)) for part in spec["command"]]      # $VAR from the job's environment
    proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            env=env, start_new_session=True,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
    return proc, log


def wait_ready(spec, proc, log_path):
    ready = spec.get("ready") or {}
    timeout = float(spec.get("ready_timeout", 120))
    deadline = time.monotonic() + timeout
    if "line" in ready:
        pattern = re.compile(ready["line"])
        while time.monotonic() < deadline:
            with open(log_path, encoding="utf-8", errors="replace") as f:
                if any(pattern.search(line) for line in f):
                    return {"ok": True, "detail": f"line /{ready['line']}/"}
            if proc.poll() is not None:
                return {"ok": False, "detail": f"exited with {proc.returncode} before /{ready['line']}/"}
            time.sleep(1.0)
        return {"ok": False, "detail": f"no line /{ready['line']}/ in {timeout} s"}
    if "topic" in ready:
        # The probe runs without the marker: it is the checker's, not the session's.
        env = {k: v for k, v in os.environ.items() if k != MARKER}
        try:
            probe = subprocess.run([sys.executable, "-c", READY_PROBE, ready["topic"], ready["type"], str(timeout)],
                                   capture_output=True, text=True, env=env, timeout=timeout + 30)
        except subprocess.TimeoutExpired:
            return {"ok": False, "detail": f"probe of {ready['topic']} timed out"}
        detail = (probe.stdout.strip() or probe.stderr.strip()[-300:])
        return {"ok": probe.returncode == 0 and proc.poll() is None, "detail": detail}
    return {"ok": proc.poll() is None, "detail": "running (no readiness condition)"}


def stop(proc, sig, timeout):
    try:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=timeout)
        return {"exited": True, "returncode": proc.returncode, "timeout": timeout}
    except subprocess.TimeoutExpired:
        return {"exited": False, "returncode": None, "timeout": timeout}


def kill_all(pids):
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass


def run(spec, log_dir, settle=3.0):
    cycles = int(spec.get("cycles", 3))
    signals = spec.get("stop_signals") or ["INT"]
    os.makedirs(log_dir, exist_ok=True)
    records = []
    for index in range(1, cycles + 1):
        sig_name = signals[(index - 1) % len(signals)]
        session_id = f"cycle{index}_{uuid.uuid4().hex[:8]}"
        before, vcan_before = processes(), set(vcan_interfaces())
        record = {"cycle": index, "signal": sig_name, "session": session_id, "ready": {}, "stopped": {}}
        started = []
        for item in spec["processes"]:
            proc, log = start(item, session_id, log_dir, index)
            started.append((item, proc, log))
            record["ready"][item["name"]] = wait_ready(item, proc, log.name)
            print(f"cycle {index}: {item['name']} ready: {record['ready'][item['name']]}", flush=True)
            if not record["ready"][item["name"]]["ok"]:
                break
        for item, proc, log in reversed(started):
            record["stopped"][item["name"]] = stop(proc, getattr(signal, f"SIG{sig_name}"), float(item.get("stop_timeout", 60)))
            log.close()
        time.sleep(settle)
        after = processes()
        record["leftovers"] = leftovers(before, after, session_id, own_pids={os.getpid()})
        record["daemons"] = new_daemons(before, after)
        record["vcan_left"] = sorted(set(vcan_interfaces()) - vcan_before)
        ok, reasons = cycle_verdict(record)
        record["ok"], record["reasons"] = ok, reasons
        print(f"cycle {index} (SIG{sig_name}): {'PASS' if ok else 'FAIL'}" + "".join(f"\n  {r}" for r in reasons), flush=True)
        kill_all(set(record["leftovers"]) | set(record["daemons"]))
        for name in record["vcan_left"]:
            subprocess.run((["sudo", "-n"] if os.geteuid() else []) + ["ip", "link", "delete", name], capture_output=True)
        for key in ("leftovers", "daemons"):
            record[key] = {str(pid): {"cmd": info["cmd"][:300], "why": info.get("why", "")} for pid, info in record[key].items()}
        records.append(record)
    return records


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("spec")
    parser.add_argument("--log-dir", default="session_cycles")
    parser.add_argument("--summary", help="JSON file: every cycle's record")
    args = parser.parse_args(argv)
    import yaml
    with open(args.spec, encoding="utf-8") as f:
        spec = yaml.safe_load(f)
    records = run(spec, args.log_dir)
    if args.summary:
        with open(args.summary, "w", encoding="utf-8") as f:
            json.dump(records, f, indent=1, sort_keys=True)
    passed = sum(1 for r in records if r["ok"])
    print(f"session cycles: {passed} of {len(records)} passed")
    return 0 if records and passed == len(records) else 1


if __name__ == "__main__":
    sys.exit(main())
