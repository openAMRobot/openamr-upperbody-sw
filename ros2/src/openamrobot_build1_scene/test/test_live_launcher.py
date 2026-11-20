# SPDX-License-Identifier: MIT
"""The live launcher (scripts/run_grocery_demo_gui.sh) and its helpers, without ROS, Gazebo or a display.

Every external command the launcher and the preflight call (sudo, apt-get, apt-mark, modprobe, ip, git, dpkg-query,
rosdep, colcon, ros2, gz, glxinfo) is a stub in a temporary bin directory that writes its arguments to a call log;
the virtual CAN interfaces live in a fake sysfs directory. That proves what the scripts would run and in which order,
not that Gazebo, RViz or the mission work: those need the desktop acceptance test (docs/demo/grocery_dark_store.md).
"""
import os
import re
import signal
import subprocess
import sys
import textwrap
import time

import pytest
import yaml

from conftest import REPOSITORY, package_path

sys.path.insert(0, package_path("scripts"))
import live_demo_preflight as preflight  # noqa: E402
import live_demo_wait as wait  # noqa: E402
import gz_follow  # noqa: E402

LAUNCHER = package_path("scripts", "run_grocery_demo_gui.sh")
VCAN = ("vcan_drive", "vcan_bms", "vcan_aux", "vcan_arm_l", "vcan_arm_r")
MUTATING = re.compile(r"^(apt-get (install|update|remove)|apt (install|remove)|apt-mark hold|modprobe|"
                      r"ip link (add|delete|set)|git (clone|fetch|checkout)|rosdep install(?!.*--simulate))")

STUB = r"""#!/usr/bin/env bash
# stub: log the call, then behave just enough
echo "$(basename "$0") $*" >> "$STUB_LOG"
name=$(basename "$0")
case "$name" in
  sudo)
    [ "$1" = "-n" ] && [ "$2" = "true" ] && exit 1
    [ "$1" = "-v" ] && exit 0
    exec "$@" ;;
  dpkg-query) exit 1 ;;
  apt-mark) exit 0 ;;
  apt-get|apt|modprobe) exit 0 ;;
  modinfo) exit 0 ;;
  glxinfo) printf 'direct rendering: Yes\nOpenGL renderer string: llvmpipe (LLVM 19.1.1, 256 bits)\nOpenGL core profile version string: 4.5 (Core Profile) Mesa 25.0.7\n' ;;
  lspci|nvidia-smi) exit 1 ;;
  rosdep)
    case "$*" in *--simulate*) echo "#[apt] Installation commands:"; echo "  sudo -H apt-get install -y ros-jazzy-fake-key" ;; esac ;;
  colcon)
    case "$1" in list) echo "src/fake" ;; build) mkdir -p install ;; esac ;;
  git)
    dir=""; if [ "$1" = "-C" ]; then dir="$2"; shift 2; fi
    case "$1" in
      clone) mkdir -p "${@: -1}/.git" ;;
      fetch) echo "${@: -1}" > "$dir/.git/FETCHED" ;;
      checkout) exit 0 ;;
      status) exit 0 ;;
      rev-parse)
        [ "$2" = "--show-toplevel" ] && exit 1
        [ -f "$dir/.git/FETCHED" ] || exit 128
        ref=$(cat "$dir/.git/FETCHED"); [ ${#ref} -eq 40 ] && echo "$ref" || echo 1111111111111111111111111111111111111111 ;;
      remote) exit 1 ;;
    esac ;;
  ip)
    # ip link show IFACE | ip link add dev IFACE type vcan | ip link set IFACE ... | ip link delete IFACE
    case "$2" in
      show) [ -d "$FAKE_NET/$3" ] && echo "9: $3: <NOARP,UP,LOWER_UP> mtu $(cat "$FAKE_NET/$3/mtu")" || exit 1 ;;
      add) mkdir -p "$FAKE_NET/$4"; echo 2060 > "$FAKE_NET/$4/mtu" ;;
      set) [ "$4" = "mtu" ] && echo "$5" > "$FAKE_NET/$3/mtu"; exit 0 ;;
      delete) rm -rf "${FAKE_NET:?}/$3" ;;
    esac ;;
  gz)
    case "$1 $2" in "sim --versions") echo 8.15.0 ;; "sdf --versions") echo 14.9.0 ;; "topic -l") exit 0 ;; esac ;;
  ros2)
    case "$1 $2" in
      "pkg prefix") echo "$FAKE_PREFIX" ;;
      "launch openamrobot_build1_scene")
        # the stack: one process that looks like `gz sim`; STUB_GZ_LIFETIME seconds, then the "Gazebo" dies
        case "$*" in *stack:=true*)
          (exec -a "gz sim -r world.sdf" sleep "${STUB_GZ_LIFETIME:-1000}") &
          wait; [ -n "${STUB_GZ_LIFETIME:-}" ] && exec sleep 1000 ;;
        esac
        exec sleep 1000 ;;
      "bag record") exec sleep 1000 ;;
    esac ;;
esac
exit 0
"""

STUBBED = ("sudo", "dpkg-query", "apt-mark", "apt-get", "apt", "modprobe", "modinfo", "glxinfo", "lspci", "nvidia-smi",
           "rosdep", "colcon", "git", "ip", "gz", "ros2")


@pytest.fixture()
def machine(tmp_path):
    """A stub machine: bin/ with the stubs first on PATH, a fake sysfs for vcan, a fake ROS and a fake install."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    for name in STUBBED:
        (bindir / name).write_text(STUB)
        (bindir / name).chmod(0o755)
    net = tmp_path / "net"
    net.mkdir()
    ros = tmp_path / "ros_setup.bash"
    ros.write_text("export ROS_DISTRO_STUB=jazzy\n")
    os_release = tmp_path / "os-release"
    os_release.write_text('ID=ubuntu\nVERSION_ID="24.04"\nPRETTY_NAME="Ubuntu 24.04 stub"\n')
    prefix = tmp_path / "prefix"
    lib = prefix / "lib" / "openamrobot_build1_scene"
    lib.mkdir(parents=True)
    (lib / "live_demo_wait.py").write_text("print('live_demo_wait: ready (stub)')\n")
    ws = tmp_path / "ws"
    log = tmp_path / "calls.log"
    log.write_text("")
    env = {key: value for key, value in os.environ.items() if not key.startswith(("ROS_", "GZ_", "AMENT", "COLCON"))}
    env.update(PATH=f"{bindir}:{env['PATH']}", STUB_LOG=str(log), FAKE_NET=str(net), FAKE_PREFIX=str(prefix),
               GROCERY_DEMO_SYS_NET=str(net), GROCERY_DEMO_ROS_SETUP=str(ros), GROCERY_DEMO_OS_RELEASE=str(os_release),
               HOME=str(tmp_path / "home"))
    env.pop("DISPLAY", None)
    env.pop("WAYLAND_DISPLAY", None)
    (tmp_path / "home").mkdir()

    class Machine:
        pass
    m = Machine()
    m.tmp, m.env, m.ws, m.net, m.log = tmp_path, env, ws, net, log
    m.calls = lambda: [line for line in log.read_text().splitlines() if line]

    def built():
        launch = ws / "install" / "openamrobot_build1_scene" / "share" / "openamrobot_build1_scene" / "launch"
        launch.mkdir(parents=True, exist_ok=True)
        (launch / "grocery_demo_gui.launch.py").write_text("")
        (ws / "install" / "setup.bash").write_text("")
        (ws / "src" / "openamr-upperbody-sw").mkdir(parents=True, exist_ok=True)
    m.built = built
    return m


def snapshot(path, skip=("calls.log",)):
    """(relative path, mtime) of every file under path, except the stub call log."""
    return sorted((os.path.relpath(os.path.join(d, f), path), os.path.getmtime(os.path.join(d, f)))
                  for d, _dirs, files in os.walk(path) for f in files if f not in skip)


# ------------------------------------------------------------------------------------------------ preflight

def test_preflight_changes_nothing_and_prints_fix_commands(machine):
    machine.built()
    before = snapshot(machine.tmp)
    done = subprocess.run(["bash", LAUNCHER, "--preflight", "--ws", str(machine.ws)], env=machine.env,
                          capture_output=True, text=True, timeout=120, stdin=subprocess.DEVNULL)
    out = done.stdout
    assert done.returncode == 1, out + done.stderr                    # stubs report nothing installed: FAIL
    for name in ("OS", "display", "ROS 2 Jazzy", "gz-sim-vendor", "sdformat", "colcon", "workspace", "vcan module",
                 "vcan interfaces", "disk", "RAM", "packages"):
        assert re.search(rf"^(PASS|WARN|FAIL) {re.escape(name)}\b", out, re.M), name
    assert "fix: sudo apt-get install -y --allow-downgrades ros-jazzy-gz-sim-vendor=0.0.13-1noble.20261031.125555" in out
    assert re.search(r"^FAIL display .*no DISPLAY", out, re.M)
    assert "nothing on this machine was changed" in out
    calls = machine.calls()
    assert not [c for c in calls if MUTATING.match(c)], [c for c in calls if MUTATING.match(c)]
    assert all(c in ("sudo -n true",) for c in calls if c.startswith("sudo")), calls
    assert not any(c.startswith("ros2 ") for c in calls), calls
    assert snapshot(machine.tmp) == before, "the preflight wrote or changed a file"
    assert any(c.startswith("dpkg-query") for c in calls)  # the stubs were used


def test_preflight_parsers():
    gl = preflight.parse_glxinfo("direct rendering: Yes\nOpenGL vendor string: NVIDIA Corporation\n"
                                 "OpenGL renderer string: NVIDIA GeForce RTX 4070/PCIe/SSE2\n"
                                 "OpenGL core profile version string: 4.6.0 NVIDIA 550.120\n")
    assert gl["core"] == (4, 6) and not preflight.is_software(gl["renderer"])
    assert preflight.is_software("llvmpipe (LLVM 19.1.1, 256 bits)")
    assert preflight.classify_display({"DISPLAY": ":0"})[0] == "x11"
    assert preflight.classify_display({"WAYLAND_DISPLAY": "wayland-0", "DISPLAY": ":0"})[0] == "wayland"
    assert preflight.classify_display({"WAYLAND_DISPLAY": "wayland-0"}, wsl_marker=True)[0] == "wslg"
    assert preflight.classify_display({})[0] == "none"
    pins = preflight.parse_pins(open(LAUNCHER, encoding="utf-8").read())
    assert pins["PLATFORM_SW_SHA"] == "931e631db661b16d1057dfd1ab1d13ea606482e2" and pins["GZ_SIM_VERSION"] == "8.15.0"


def test_preflight_counts_held_packages_and_sources_ros_with_bash(monkeypatch):
    """Session run 44590676440 (pinned vendor builds installed and held, as --setup leaves them): the preflight said
    "not installed" for the held packages (dpkg status hi) and "gz-sim not found" (setup.bash sourced by /bin/sh)."""
    out = ("ros-jazzy-gz-sim-vendor 0.0.13-1noble.20261031.125555 hi \nros-jazzy-rviz2 14.1.6-1noble ii \n"
           "ros-jazzy-old 1.0 rc \n")
    assert preflight.parse_dpkg(out) == {"ros-jazzy-gz-sim-vendor": "0.0.13-1noble.20261031.125555",
                                          "ros-jazzy-rviz2": "14.1.6-1noble"}
    commands = []

    def fake_run(command, timeout=20, shell=False):
        commands.append((command, shell))
        return 0, "8.15.0\n" if "gz sim" in str(command) else "14.9.0\n" if "gz sdf" in str(command) else ""
    monkeypatch.setattr(preflight, "run", fake_run)
    report = preflight.Report()
    preflight.check_gazebo(report, {"GZ_SIM_VERSION": "8.15.0", "SDFORMAT_VERSION": "14.9.0"}, "/opt/ros/jazzy/setup.bash", True)
    gz = [c for c, _shell in commands if "--versions" in str(c)]
    assert gz and all(isinstance(c, list) and c[0] == "bash" for c in gz), gz


def test_setup_counts_a_held_package_as_installed():
    """--setup holds the vendor builds; dpkg then reports them `hi`. The launcher's own installed-version filter must
    take them, or every --setup lists them as missing again."""
    import re
    import subprocess
    text = open(LAUNCHER, encoding="utf-8").read()
    program = re.search(r"Status-Abbrev\}' \"\$name\" 2>/dev/null \| awk '([^']+)'", text).group(1)
    out = subprocess.run(["awk", program], input="0.0.13-1noble.20261031.125555 hi \n", capture_output=True, text=True).stdout
    assert out.strip() == "0.0.13-1noble.20261031.125555", program
    gone = subprocess.run(["awk", program], input="1.0 rc \n", capture_output=True, text=True).stdout
    assert gone.strip() == "", "a removed package (rc) counted as installed"


def test_launcher_pins_equal_the_workflow_and_pins_yaml():
    pins = preflight.parse_pins(open(LAUNCHER, encoding="utf-8").read())
    workflow = yaml.safe_load(open(os.path.join(REPOSITORY, ".github", "workflows", "grocery-demo.yml"), encoding="utf-8"))
    env = workflow["env"]
    for key in ("PLATFORM_SW_SHA", "GZ_SIM_VENDOR", "GZ_TOOLS_VENDOR", "SDFORMAT_VENDOR", "GZ_SIM_VERSION", "SDFORMAT_VERSION"):
        assert pins[key] == str(env[key]), key
    yaml_pins = yaml.safe_load(open(package_path("config", "pins.yaml"), encoding="utf-8"))["pins"]
    for key in ("openarm_description", "openarm_ros2", "openarm_can"):
        assert pins[f"{key.upper()}_SHA"] == yaml_pins[key]["sha"], key
    assert preflight.pins_yaml(package_path("config", "pins.yaml"))["openarm_can"] == yaml_pins["openarm_can"]["sha"]


def test_launcher_vcan_mtus_equal_setup_vcan_all():
    script = open(package_path("scripts", "setup_vcan_all.sh"), encoding="utf-8").read()
    base = re.search(r'^BASE="([^"]+)"', script, re.M).group(1).split()
    arms = re.search(r'^ARMS="([^"]+)"', script, re.M).group(1).split()
    expected = {**{i: 16 for i in base}, **{i: 72 for i in arms}}
    assert preflight.VCAN_MTU == expected
    for iface, mtu in expected.items():
        done = subprocess.run(["bash", "-c", f'eval "$(grep "^vcan_mtu()" "{LAUNCHER}")"; vcan_mtu {iface}'],
                              capture_output=True, text=True, check=True)
        assert done.stdout.strip() == str(mtu), iface


# ------------------------------------------------------------------------------------------------ --setup

def test_setup_asks_before_any_package_change(machine):
    done = subprocess.run(["bash", LAUNCHER, "--setup", "--ws", str(machine.ws)], env=machine.env, capture_output=True,
                          text=True, timeout=120, stdin=subprocess.DEVNULL)
    out = done.stdout + done.stderr
    assert done.returncode == 3, out
    assert "would change this machine's packages" in out and "nothing installed" in out
    assert "sudo apt-get install -y --allow-downgrades ros-jazzy-gz-sim-vendor=0.0.13-1noble.20261031.125555" in out
    assert "sudo -H apt-get install -y ros-jazzy-fake-key" in out          # rosdep's plan, simulated only
    assert "Container route" in out
    calls = machine.calls()
    assert not [c for c in calls if c.startswith(("sudo", "apt-get", "apt-mark", "modprobe"))], calls
    assert not [c for c in calls if c.startswith("rosdep install") and "--simulate" not in c], calls
    pins = (machine.ws / "grocery_demo_pins.txt").read_text()
    for sha in ("931e631db661b16d1057dfd1ab1d13ea606482e2", "14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831",
                "4e837e1d0dae692ff67b560b69d8d281d7a8d4ed", "bfd4e52194aefe38389fa6e670be3738136facb9"):
        assert sha in pins


def test_setup_with_yes_runs_the_listed_commands(machine):
    done = subprocess.run(["bash", LAUNCHER, "--setup", "--yes", "--ws", str(machine.ws), "--preflight"], env=machine.env,
                          capture_output=True, text=True, timeout=120, stdin=subprocess.DEVNULL)
    # --preflight wins over --setup: read only
    assert not [c for c in machine.calls() if c.startswith(("sudo apt-get", "apt-get install"))]
    machine.log.write_text("")
    done = subprocess.run(["bash", LAUNCHER, "--setup", "--yes", "--ws", str(machine.ws), "--force", "--headless",
                           "--no-mission", "--duration", "1", "--log-dir", str(machine.tmp / "logs")],
                          env=machine.env, capture_output=True, text=True, timeout=180, stdin=subprocess.DEVNULL)
    calls = machine.calls()
    assert any(c.startswith("sudo apt-get install -y --allow-downgrades ros-jazzy-gz-sim-vendor=0.0.13") for c in calls), calls
    assert "sudo apt-mark hold ros-jazzy-gz-sim-vendor ros-jazzy-gz-tools-vendor ros-jazzy-sdformat-vendor" in calls
    assert any(c.startswith("sudo -H apt-get install -y ros-jazzy-fake-key") for c in calls), calls
    del done


# ------------------------------------------------------------------------------------------------ run and cleanup

def start(machine, *args, lifetime=None):
    env = dict(machine.env)
    if lifetime is not None:
        env["STUB_GZ_LIFETIME"] = str(lifetime)
    return subprocess.Popen(["bash", LAUNCHER, "--ws", str(machine.ws), "--force", "--log-dir", str(machine.tmp / "logs"),
                             *args], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            stdin=subprocess.DEVNULL, start_new_session=True)


def run_processes(run_id):
    """PIDs of processes that carry exactly this run's GROCERY_DEMO_RUN_ID."""
    marker = f"GROCERY_DEMO_RUN_ID={run_id}".encode()
    found = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            with open(f"/proc/{pid}/environ", "rb") as f:
                if marker in f.read().split(b"\0"):
                    found.append(pid)
        except OSError:
            pass
    return found


def run_id(output):
    match = re.search(r"run (grocery_demo_\d+_\d+), logs in", output)
    assert match, output
    return match.group(1)


def finish(proc, timeout):
    """communicate(), and on a timeout kill the launcher's whole session so that no test leaves processes behind."""
    try:
        return proc.communicate(timeout=timeout)[0]
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        raise


def precreate(machine, iface, mtu):
    (machine.net / iface).mkdir()
    (machine.net / iface / "mtu").write_text(f"{mtu}\n")


def test_run_creates_only_missing_vcan_and_deletes_only_those(machine):
    machine.built()
    precreate(machine, "vcan_aux", 16)
    proc = start(machine, "--headless", "--no-mission", "--duration", "6")
    out = finish(proc, 120)
    assert proc.returncode == 0, out
    assert "ready (stub)" in out and "duration 6 s reached" in out
    calls = machine.calls()
    added = [c.split()[4] for c in calls if c.startswith("ip link add")]
    deleted = [c.split()[3] for c in calls if c.startswith("ip link delete")]
    assert sorted(added) == sorted(set(VCAN) - {"vcan_aux"}) == sorted(deleted), calls
    assert not [c for c in calls if c.startswith("ip link") and "vcan_aux" in c and not c.startswith("ip link show")], calls
    assert sorted(os.listdir(machine.net)) == ["vcan_aux"]
    assert (machine.net / "vcan_aux" / "mtu").read_text().strip() == "16"
    for iface in set(VCAN) - {"vcan_aux"}:
        assert any(c == f"ip link set {iface} mtu {preflight.VCAN_MTU[iface]}" for c in calls), iface
    assert any("stack:=true" in c and "gazebo_gui:=false" in c for c in calls), calls
    assert not any("rviz:=true" in c for c in calls), "--headless must not open RViz"
    assert run_processes(run_id(out)) == [], "processes of the run survived the cleanup"
    cleanup = (machine.tmp / "logs" / "cleanup.txt").read_text()
    assert re.search(r"kept \(existed before this run\):\s+vcan_aux$", cleanup, re.M), cleanup
    assert (machine.tmp / "logs.logs.tar.gz").exists()


def test_existing_vcan_with_wrong_mtu_is_not_changed(machine):
    machine.built()
    precreate(machine, "vcan_arm_l", 16)
    proc = start(machine, "--headless", "--no-mission", "--duration", "5")
    out = finish(proc, 60)
    assert proc.returncode == 1 and "it is not ours to change" in out, out
    assert not [c for c in machine.calls() if c.startswith(("ip link add", "ip link set", "ip link delete"))]
    assert (machine.net / "vcan_arm_l" / "mtu").read_text().strip() == "16"


def test_ctrl_c_stops_everything_and_deletes_created_vcan(machine):
    machine.built()
    proc = start(machine, "--headless", "--no-mission")
    deadline = time.monotonic() + 60
    lines = []
    while time.monotonic() < deadline:
        line = proc.stdout.readline()
        lines.append(line)
        if "Ctrl-C stops everything" in line:
            break
    if not any("Ctrl-C stops everything" in line for line in lines):
        os.killpg(proc.pid, signal.SIGKILL)
        pytest.fail("".join(lines))
    rid = run_id("".join(lines))
    mine = run_processes(rid)
    assert len(mine) >= 2, mine                          # the stack stub and its `gz sim` child run
    # The stack was started in the background of a non-interactive bash, which ignores SIGINT for its jobs; the
    # launcher must give it SIGINT back (env --default-signal), or ros2 launch could not be stopped gracefully.
    stacks = [pid for pid in mine
              if "stack:=true" in open(f"/proc/{pid}/cmdline", "rb").read().decode(errors="replace")]
    assert stacks, "no stack process"
    for pid in stacks:
        ignored = int(re.search(r"^SigIgn:\s+([0-9a-f]+)", open(f"/proc/{pid}/status").read(), re.M).group(1), 16)
        assert not ignored & (1 << (signal.SIGINT - 1)), f"the stack ({pid}) ignores SIGINT"
    os.killpg(proc.pid, signal.SIGINT)                    # as the terminal does for Ctrl-C
    out = finish(proc, 90)
    assert proc.returncode == 130, "".join(lines) + out
    assert "interrupted" in out and "stopping (cleanup)" in out
    assert os.listdir(machine.net) == [], os.listdir(machine.net)
    assert run_processes(rid) == [], "processes of the run survived Ctrl-C"


def test_dead_gazebo_is_detected_and_cleaned_up(machine):
    machine.built()
    proc = start(machine, "--headless", "--no-mission", lifetime=8)
    out = finish(proc, 120)
    assert proc.returncode == 1, out
    assert "Gazebo is no longer running" in out, out
    assert os.listdir(machine.net) == []
    assert run_processes(run_id(out)) == []


# ------------------------------------------------------------------------------------------------ helpers

def test_follow_camera_uses_the_gz_gui8_interfaces():
    """/gui/track is a topic in gz-gui8 (CameraTracking subscribes to it); success is read back from
    /gui/currently_tracked, because /gui/follow answers true even before the model exists."""
    source = open(package_path("scripts", "gz_follow.py"), encoding="utf-8").read()
    assert gz_follow.TRACK_TOPIC == "/gui/track" and gz_follow.STATUS_TOPIC == "/gui/currently_tracked"
    assert not re.search(r'"service",\s*"-s",\s*(TRACK_TOPIC|"/gui/track")', source)
    assert re.search(r'"topic",\s*"-t",\s*TRACK_TOPIC,\s*"-m",\s*"gz.msgs.CameraTrack"', source)
    request = gz_follow.track_request("openamrobot", (-2.5, 0.0, 2.0))
    assert request == 'track_mode: FOLLOW, follow_target: {name: "openamrobot"}, follow_offset: {x: -2.5, y: 0.0, z: 2.0}'
    following = textwrap.dedent('''\
        track_mode: FOLLOW
        follow_target {
          name: "openamrobot"
        }
        follow_offset {
          x: -2.5
          z: 2
        }''')
    assert gz_follow.is_following(following, "openamrobot")
    assert not gz_follow.is_following(following.replace("openamrobot", "other"), "openamrobot")
    assert not gz_follow.is_following("track_mode: NONE\n", "openamrobot")


def test_readiness_needs_advancing_sim_time_and_every_topic():
    state = wait.Readiness(("/tf", "/scan", "/odom"), min_sim=2.0)
    assert state.missing()[0] == "/clock"
    state.clock(10.0)
    for topic in ("/tf", "/scan", "/odom"):
        state.message(topic)
    assert not state.ready() and state.missing()[0].startswith("/clock advancing")
    state.clock(12.5)
    assert state.ready()


def test_launch_offers_the_parts_the_launcher_starts():
    launch = open(package_path("launch", "grocery_demo_gui.launch.py"), encoding="utf-8").read()
    for name in ("stack", "gazebo_gui"):
        assert f'DeclareLaunchArgument("{name}"' in launch, name
    assert 'if value("stack") == "true":' in launch
    script = open(LAUNCHER, encoding="utf-8").read()
    assert "live_demo_wait.py" in script and script.index("live_demo_wait.py") < script.index("rviz:=\"$RVIZ\"")
    assert script.index("live_demo_wait.py") < script.index("mission:=true")
    assert "trap cleanup EXIT" in script
    cmake = open(package_path("CMakeLists.txt"), encoding="utf-8").read()
    for name in ("live_demo_preflight.py", "live_demo_wait.py"):
        assert f"scripts/{name}" in cmake, name
