#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Preflight of the live grocery demonstrator: what this machine has, compared with what the launcher needs.

    live_demo_preflight.py [--ws DIR] [--launcher PATH] [--headless]

Read only: it runs no package manager, no modprobe, no ip link, no git fetch or checkout, and writes no file. It
reads /etc/os-release, /proc, /sys, dpkg-query, apt-mark showhold, git rev-parse and status, glxinfo, ffmpeg -devices
and `gz sim --versions` (in a subshell that sources /opt/ros/jazzy/setup.bash); `sudo -n true` only when the virtual
CAN buses need creating, to learn whether sudo would ask for a password (it never prompts). Every line is PASS, WARN or FAIL; a WARN or FAIL line is followed by the exact
command that fixes it, printed and never run. Exit 0 when there is no FAIL, 1 otherwise.

The pins are read from run_grocery_demo_gui.sh (the launcher is the single place they are written for the live demo);
config/pins.yaml is compared for the OpenArm repositories. Simulation tooling only: a PASS here says the machine can
start the simulation, nothing about the robot, hardware or safety.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = "openamrobot_build1_scene"
VCAN = ("vcan_drive", "vcan_bms", "vcan_aux", "vcan_arm_l", "vcan_arm_r")
VCAN_MTU = {"vcan_drive": 16, "vcan_bms": 16, "vcan_aux": 16, "vcan_arm_l": 72, "vcan_arm_r": 72}
# Debian packages the live demo needs at run time (the launcher's --setup installs them after asking).
REQUIRED_PACKAGES = ("ros-jazzy-ros-gz", "ros-jazzy-gz-ros2-control", "ros-jazzy-navigation2", "ros-jazzy-nav2-bringup",
                     "ros-jazzy-moveit", "ros-jazzy-joint-state-publisher", "ros-jazzy-rviz2", "ros-jazzy-xacro",
                     "python3-can", "can-utils", "iproute2")
OPTIONAL_PACKAGES = {"mesa-utils": "glxinfo, to tell a GPU from software rendering",
                     "ffmpeg": "--record screen video and --video", "xvfb": "headless smoke runs (CI)"}
SOFTWARE_RENDERERS = ("llvmpipe", "softpipe", "swrast", "software rasterizer", "svga3d", "microsoft basic render")
MIN_GL = (3, 3)  # ogre2, the gz-sim 8 default render engine


class Report:
    def __init__(self):
        self.lines = []

    def add(self, status, name, detail, fix=None):
        self.lines.append((status, name, detail, fix))

    def text(self):
        out, shown = [], set()
        for status, name, detail, fix in self.lines:
            out.append(f"{status:<4} {name:<22} {detail}")
            if fix and status != "PASS":
                for line in fix if isinstance(fix, (list, tuple)) else [fix]:
                    out.append(f"     fix: {line}" if line not in shown else "     fix: as above")
                    shown.add(line)
        counts = {s: sum(1 for line in self.lines if line[0] == s) for s in ("PASS", "WARN", "FAIL")}
        out.append(f"preflight: {counts['PASS']} PASS, {counts['WARN']} WARN, {counts['FAIL']} FAIL "
                   "(read only: nothing on this machine was changed)")
        return "\n".join(out)

    def failed(self):
        return any(line[0] == "FAIL" for line in self.lines)


def run(command, timeout=20, shell=False):
    """(exit status, stdout) of a read-only command; 127 when it is missing."""
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=timeout, shell=shell, check=False)
        return done.returncode, done.stdout
    except FileNotFoundError:
        return 127, ""
    except subprocess.TimeoutExpired:
        return 124, ""


def read(path, default=""):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return default


# ---------------------------------------------------------------- pure parsers (unit tested without a machine)

def parse_pins(launcher_text):
    """NAME=value assignments of the launcher's pin block."""
    return dict(re.findall(r"^([A-Z0-9_]+_(?:SHA|VENDOR|VERSION|BRANCH))=([^\s#]+)", launcher_text, re.M))


def parse_os_release(text):
    return dict(re.findall(r'^(\w+)="?([^"\n]*)"?$', text, re.M))


def classify_display(env, wsl_marker=False):
    """'wslg', 'wayland', 'x11' or 'none', and a description."""
    session = env.get("XDG_SESSION_TYPE", "")
    if wsl_marker and (env.get("WAYLAND_DISPLAY") or env.get("DISPLAY")):
        return "wslg", f"WSLg (DISPLAY={env.get('DISPLAY', '')}, WAYLAND_DISPLAY={env.get('WAYLAND_DISPLAY', '')})"
    if env.get("WAYLAND_DISPLAY") or session == "wayland":
        xwayland = f", XWayland DISPLAY={env['DISPLAY']}" if env.get("DISPLAY") else ", no XWayland DISPLAY"
        return "wayland", f"Wayland ({env.get('WAYLAND_DISPLAY', 'session')}{xwayland})"
    if env.get("DISPLAY"):
        return "x11", f"X11 (DISPLAY={env['DISPLAY']})"
    return "none", "no DISPLAY and no WAYLAND_DISPLAY"


def parse_glxinfo(text):
    """{'renderer', 'vendor', 'core', 'compat', 'direct'} from `glxinfo -B`; versions as (major, minor) or None."""
    def field(name):
        match = re.search(rf"^\s*{name}:\s*(.+)$", text, re.M | re.I)
        return match.group(1).strip() if match else ""

    def version(name):
        match = re.search(rf"^\s*{name}:\s*(\d+)\.(\d+)", text, re.M | re.I)
        return (int(match.group(1)), int(match.group(2))) if match else None
    return {"renderer": field("OpenGL renderer string"), "vendor": field("OpenGL vendor string"),
            "core": version("OpenGL core profile version string"), "compat": version("OpenGL version string"),
            "direct": field("direct rendering")}


def is_software(renderer):
    return any(word in renderer.lower() for word in SOFTWARE_RENDERERS)


def meminfo_gib(text, key):
    match = re.search(rf"^{key}:\s+(\d+) kB", text, re.M)
    return int(match.group(1)) / 1024 / 1024 if match else None


def apt_install(packages, held=False):
    line = "sudo apt-get install -y " + " ".join(packages)
    return line.replace("install -y", "install -y --allow-downgrades") if held else line


# ---------------------------------------------------------------- checks

def check_os(report, os_release_path):
    info = parse_os_release(read(os_release_path))
    name = info.get("PRETTY_NAME", "unknown")
    if info.get("ID") == "ubuntu" and info.get("VERSION_ID") == "24.04":
        report.add("PASS", "OS", name)
    else:
        report.add("FAIL", "OS", f"{name}: ROS 2 Jazzy binaries and the pinned Gazebo packages are for Ubuntu 24.04",
                   ["use Ubuntu 24.04, or the container route (docs/demo/grocery_dark_store.md, Container route)"])
    machine = os.uname().machine
    if machine in ("x86_64", "amd64"):
        report.add("PASS", "architecture", machine)
    else:
        report.add("WARN", "architecture", f"{machine}: CI and the pinned vendor builds are x86_64; arm64 is untested",
                   ["run on an x86_64 machine for the tested combination"])
    if os.path.exists("/.dockerenv") or "container" in os.environ:
        report.add("WARN", "container", "running inside a container: the vcan module and the GPU come from the host",
                   ["on the host: sudo modprobe vcan", "start the container with --network host and the GPU flags of "
                    "the Container route (docs/demo/grocery_dark_store.md)"])


def check_display(report, env, headless):
    wsl = "microsoft" in read("/proc/version").lower()
    kind, detail = classify_display(env, wsl_marker=wsl or os.path.isdir("/mnt/wslg"))
    if kind == "none":
        if headless:
            report.add("PASS", "display", f"{detail} (headless run asked: no window is needed)")
        else:
            report.add("FAIL", "display", f"{detail}: the Gazebo window and RViz need a desktop session",
                       ["log in to a desktop session (X11 or Wayland) and run from a terminal there",
                        "or, without a screen: run_grocery_demo_gui.sh --headless (no windows)"])
    elif kind == "wayland":
        report.add("WARN", "display", f"{detail}: Gazebo's Qt and ogre2 window is most tested on X11",
                   ["if the Gazebo window stays black or crashes: export QT_QPA_PLATFORM=xcb (runs it on XWayland)",
                    "or choose 'Ubuntu on Xorg' on the login screen"])
    elif kind == "wslg":
        report.add("WARN", "display", f"{detail}: WSLg works with a GPU driver for WSL; vcan needs a WSL kernel with it",
                   ["wsl --update (Windows side), then in WSL: glxinfo -B should not say llvmpipe"])
    else:
        report.add("PASS", "display", detail)
    return kind


def check_gpu(report, env, display_kind):
    software_forced = env.get("LIBGL_ALWAYS_SOFTWARE") == "1"
    if display_kind == "none":
        report.add("WARN", "OpenGL", "not checked: no display", ["run the preflight from the desktop session"])
        return
    status, out = run(["glxinfo", "-B"])
    if status == 127:
        gpus = [line for line in run(["lspci"])[1].splitlines() if re.search(r"VGA|3D|Display", line)]
        nvidia = run(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"])
        dri = sorted(os.listdir("/dev/dri")) if os.path.isdir("/dev/dri") else []
        seen = "; ".join(gpus[:2]) or "no GPU in lspci"
        if nvidia[0] == 0:
            seen += f"; nvidia-smi: {nvidia[1].strip()}"
        seen += f"; /dev/dri: {' '.join(dri) or 'none'}"
        report.add("WARN", "OpenGL", f"glxinfo missing, renderer unknown ({seen})", [apt_install(["mesa-utils"])])
        return
    if status != 0:
        report.add("FAIL", "OpenGL", "glxinfo -B could not open the display or create a GL context",
                   ["check that this terminal belongs to the desktop session (echo $DISPLAY)",
                    "NVIDIA: sudo ubuntu-drivers install, then reboot"])
        return
    gl = parse_glxinfo(out)
    version = gl["core"] or gl["compat"]
    shown = f"{gl['renderer'] or 'unknown renderer'}, OpenGL core {'.'.join(map(str, version)) if version else '?'}"
    if version and version < MIN_GL:
        report.add("FAIL", "OpenGL", f"{shown}: ogre2 needs OpenGL {MIN_GL[0]}.{MIN_GL[1]}",
                   ["install the vendor GPU driver (NVIDIA: sudo ubuntu-drivers install)",
                    "or force Mesa's software renderer: export LIBGL_ALWAYS_SOFTWARE=1 (slow)"])
    elif is_software(gl["renderer"]):
        forced = " (LIBGL_ALWAYS_SOFTWARE=1 is set)" if software_forced else ""
        report.add("WARN", "OpenGL", f"{shown}: software rendering{forced}; Gazebo and RViz run, slowly",
                   ["for GPU rendering: unset LIBGL_ALWAYS_SOFTWARE and install the GPU driver "
                    "(NVIDIA: sudo ubuntu-drivers install; Intel/AMD: Mesa from Ubuntu is enough)",
                    "NVIDIA hybrid laptops: __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia before the command"])
    else:
        report.add("PASS", "OpenGL", f"{shown}, hardware")
    if software_forced:
        report.add("WARN", "software rendering", "LIBGL_ALWAYS_SOFTWARE=1 is set in this shell",
                   ["unset LIBGL_ALWAYS_SOFTWARE (only keep it when there is no usable GPU)"])
    else:
        report.add("PASS", "software rendering", "fallback available: export LIBGL_ALWAYS_SOFTWARE=1 if the GPU fails")


def check_ros(report, env, ros_setup):
    if not os.path.isfile(ros_setup):
        report.add("FAIL", "ROS 2 Jazzy", f"{ros_setup} missing",
                   ["ROS 2 apt source: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html",
                    "sudo apt-get install -y ros-jazzy-desktop ros-dev-tools"])
        return False
    report.add("PASS", "ROS 2 Jazzy", f"installed ({ros_setup})")
    distro = env.get("ROS_DISTRO", "")
    if distro == "jazzy":
        report.add("PASS", "ROS 2 sourced", "ROS_DISTRO=jazzy in this shell")
    elif distro:
        report.add("FAIL", "ROS 2 sourced", f"ROS_DISTRO={distro} in this shell: two distributions would mix",
                   ["open a new terminal without other ROS setup files sourced (check ~/.bashrc)"])
    else:
        report.add("PASS", "ROS 2 sourced", "not sourced in this shell; the launcher sources it itself")
    for tool in ("colcon", "rosdep"):
        if shutil.which(tool):
            report.add("PASS", tool, shutil.which(tool))
        else:
            report.add("FAIL", tool, "missing", [apt_install(["ros-dev-tools"])])
    return True


def dpkg_versions(packages):
    status, out = run(["dpkg-query", "-W", "-f=${Package} ${Version} ${db:Status-Abbrev}\\n", *packages])
    versions = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[2].startswith("ii"):
            versions[parts[0]] = parts[1]
    return versions


def check_gazebo(report, pins, ros_setup, have_ros):
    vendors = {"ros-jazzy-gz-sim-vendor": pins.get("GZ_SIM_VENDOR"), "ros-jazzy-gz-tools-vendor": pins.get("GZ_TOOLS_VENDOR"),
               "ros-jazzy-sdformat-vendor": pins.get("SDFORMAT_VENDOR")}
    installed = dpkg_versions(list(vendors))
    wrong = [f"{name}={want}" for name, want in vendors.items() if installed.get(name) != want]
    fix = [apt_install(wrong, held=True), "sudo apt-mark hold " + " ".join(vendors)]
    for name, want in vendors.items():
        have = installed.get(name, "not installed")
        report.add("PASS" if have == want else "FAIL", name.replace("ros-jazzy-", ""), f"{have} (pinned {want})", fix)
    _, held = run(["apt-mark", "showhold"])
    unheld = [name for name in vendors if name not in held.split()]
    if unheld and not wrong:
        report.add("WARN", "vendor hold", f"not held: {' '.join(unheld)} (an apt upgrade would move them)",
                   ["sudo apt-mark hold " + " ".join(unheld)])
    if not have_ros:
        return
    for tool, want in (("sim", pins.get("GZ_SIM_VERSION")), ("sdf", pins.get("SDFORMAT_VERSION"))):
        status, out = run(f"set +u; . '{ros_setup}' >/dev/null 2>&1; gz {tool} --versions", shell=True, timeout=60)
        have = out.strip().splitlines()[-1] if out.strip() else "not found"
        name = "gz-sim" if tool == "sim" else "sdformat"
        if have == want:
            report.add("PASS", name, have)
        else:
            report.add("FAIL", name, f"{have} (pinned {want})",
                       [apt_install([f"{n}={v}" for n, v in vendors.items()], held=True)])


def check_packages(report):
    installed = dpkg_versions(list(REQUIRED_PACKAGES) + list(OPTIONAL_PACKAGES))
    missing = [name for name in REQUIRED_PACKAGES if name not in installed]
    if missing:
        report.add("FAIL", "packages", f"missing: {' '.join(missing)}", [apt_install(missing)])
    else:
        report.add("PASS", "packages", f"{len(REQUIRED_PACKAGES)} required packages installed")
    for name, why in OPTIONAL_PACKAGES.items():
        if name not in installed:
            report.add("WARN", name, f"not installed ({why})", [apt_install([name])])


def git_head(path):
    status, out = run(["git", "-C", path, "rev-parse", "HEAD"])
    return out.strip() if status == 0 else None


def git_dirty(path):
    status, out = run(["git", "-C", path, "status", "--porcelain", "--untracked-files=no"])
    return status == 0 and bool(out.strip())


def pins_yaml(path):
    """{name: sha} of config/pins.yaml without a YAML library (the file is flat)."""
    pins, current = {}, None
    for line in read(path).splitlines():
        key = re.match(r"^  (\w+):\s*$", line)
        if key:
            current = key.group(1)
        sha = re.match(r'^    sha:\s*"?([0-9a-f]{40})"?', line)
        if sha and current:
            pins[current] = sha.group(1)
    return pins


def check_workspace(report, ws, pins, pins_yaml_path):
    src = os.path.join(ws, "src")
    if not os.path.isdir(src):
        report.add("FAIL", "workspace", f"{src} missing",
                   [f"run_grocery_demo_gui.sh --setup --ws {ws} (clones the pinned sources; asks before any apt change)"])
        return
    report.add("PASS", "workspace", ws)
    repos = {"openamr-upperbody-sw": ("src/openamr-upperbody-sw", None),
             "openamr-platform-sw": ("src/openamr-platform-sw", pins.get("PLATFORM_SW_SHA")),
             "openarm_description": ("src/upstream/openarm_description", pins.get("OPENARM_DESCRIPTION_SHA")),
             "openarm_ros2": ("src/upstream/openarm_ros2", pins.get("OPENARM_ROS2_SHA")),
             "openarm_can": ("src/upstream/openarm_can", pins.get("OPENARM_CAN_SHA"))}
    for name, (relative, want) in repos.items():
        path = os.path.join(ws, relative)
        head = git_head(path)
        if head is None:
            report.add("FAIL", name, f"{path} is not a git checkout",
                       [f"run_grocery_demo_gui.sh --setup --ws {ws}"])
            continue
        dirty = " with local changes" if git_dirty(path) else ""
        if want is None:
            report.add("WARN" if dirty else "PASS", name, f"{head[:12]}{dirty} (the demo branch, not pinned by hash)",
                       [f"git -C {path} stash  # or commit the changes"] if dirty else None)
        elif head == want and not dirty:
            report.add("PASS", name, f"{head[:12]} = pin")
        elif head == want:
            report.add("WARN", name, f"{head[:12]} = pin{dirty}", [f"git -C {path} stash"])
        else:
            report.add("FAIL", name, f"{head[:12]}, pinned {want[:12]}",
                       [f"git -C {path} fetch origin {want} && git -C {path} checkout --detach {want}"])
    yaml_pins = pins_yaml(pins_yaml_path)
    for key, launcher_key in (("openarm_description", "OPENARM_DESCRIPTION_SHA"), ("openarm_ros2", "OPENARM_ROS2_SHA"),
                              ("openarm_can", "OPENARM_CAN_SHA")):
        if yaml_pins.get(key) and yaml_pins[key] != pins.get(launcher_key):
            report.add("FAIL", "pins.yaml", f"{key}: pins.yaml {yaml_pins[key][:12]}, launcher {pins.get(launcher_key, '?')[:12]}",
                       ["report it to the work-package owner: the launcher and config/pins.yaml disagree"])
    if yaml_pins:
        report.add("PASS", "pins.yaml", "OpenArm pins equal the launcher's; platform-sw is pinned by the grocery-demo "
                   f"workflow ({pins.get('PLATFORM_SW_SHA', '?')[:7]}), not by pins.yaml's platform_sw_main")
    built = os.path.join(ws, "install", PACKAGE, "share", PACKAGE, "launch", "grocery_demo_gui.launch.py")
    if os.path.isfile(built):
        report.add("PASS", "build", f"{PACKAGE} installed in {ws}/install")
    else:
        report.add("WARN", "build", "not built yet (the launcher builds it, about 10 to 30 minutes)",
                   [f"cd {ws} && source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to {PACKAGE}"])


def check_vcan(report):
    release = os.uname().release
    loaded = os.path.isdir("/sys/module/vcan")
    status, _ = run(["modinfo", "vcan"])
    available = loaded or status == 0
    sys_net = os.environ.get("GROCERY_DEMO_SYS_NET", "/sys/class/net")
    existing = {name: read(f"{sys_net}/{name}/mtu").strip() for name in VCAN if os.path.isdir(f"{sys_net}/{name}")}
    root = os.geteuid() == 0
    _, groups = run(["id", "-nG"])
    sudoer = bool({"sudo", "admin", "wheel"} & set(groups.split()))
    if loaded:
        report.add("PASS", "vcan module", "loaded")
    elif available:
        report.add("PASS", "vcan module", "available, not loaded (the launcher loads it with sudo modprobe vcan)")
    else:
        report.add("FAIL", "vcan module", f"not available for kernel {release}",
                   [apt_install([f"linux-modules-extra-{release}"]), "WSL: a WSL kernel built with CONFIG_CAN_VCAN"])
    wrong = [f"{name} mtu {mtu} (needs {VCAN_MTU[name]})" for name, mtu in existing.items() if mtu != str(VCAN_MTU[name])]
    if wrong:
        report.add("FAIL", "vcan interfaces", "exist with the wrong MTU: " + ", ".join(wrong) +
                   " (the launcher does not change interfaces it did not create)",
                   [f"sudo ip link set {w.split()[0]} down && sudo ip link set {w.split()[0]} mtu "
                    f"{VCAN_MTU[w.split()[0]]} && sudo ip link set {w.split()[0]} up" for w in wrong])
    missing = [name for name in VCAN if name not in existing]
    need_sudo = bool(missing or not loaded) and not root
    detail = f"present: {' '.join(existing) or 'none'}; to create: {' '.join(missing) or 'none'}"
    # `sudo -n true` only asks whether sudo works without a password; it never prompts and runs nothing else.
    passwordless = need_sudo and run(["sudo", "-n", "true"], timeout=5)[0] == 0
    if not need_sudo:
        report.add("PASS", "vcan interfaces", detail + "; no sudo needed")
    elif passwordless:
        report.add("PASS", "vcan interfaces", detail + "; sudo works without a password; the launcher deletes only the "
                   "interfaces it created when it exits")
    elif sudoer:
        report.add("WARN", "vcan interfaces", detail + "; the launcher will ask for your sudo password once and deletes "
                   "only the interfaces it created when it exits", None)
    else:
        report.add("FAIL", "vcan interfaces", detail + f"; sudo needed and {os.environ.get('USER', 'this user')} is not in "
                   "the sudo group", ["ask an administrator to run: sudo modprobe vcan && sudo sh setup_vcan_all.sh "
                                      "(scripts/ of this package), then run the launcher again"])


def check_resources(report, ws):
    target = ws if os.path.isdir(ws) else os.path.dirname(os.path.abspath(ws)) or "/"
    while not os.path.isdir(target):
        target = os.path.dirname(target)
    free_gib = shutil.disk_usage(target).free / 1024 ** 3
    built = os.path.isdir(os.path.join(ws, "install"))
    need = 5 if built else 20
    status = "PASS" if free_gib >= need else ("WARN" if free_gib >= need / 2 else "FAIL")
    report.add(status, "disk", f"{free_gib:.1f} GiB free at {target} ({need} GiB recommended: "
               f"{'logs and bags' if built else 'build plus logs and bags'})",
               [f"free space on {target} (each run writes a bag of about 1 GiB to ~/grocery_demo_logs)"])
    meminfo = read("/proc/meminfo")
    total, free = meminfo_gib(meminfo, "MemTotal"), meminfo_gib(meminfo, "MemAvailable")
    if total is None:
        report.add("WARN", "RAM", "unknown (/proc/meminfo not readable)")
    else:
        status = "PASS" if total >= 15 else ("WARN" if total >= 7.5 else "FAIL")
        report.add(status, "RAM", f"{total:.1f} GiB total, {free:.1f} GiB available (16 GiB recommended)",
                   ["close other programs; with 8 GiB use --no-rviz", "colcon build --parallel-workers 2 if the build runs out of memory"])
    cores = os.cpu_count() or 0
    report.add("PASS" if cores >= 8 else "WARN", "CPU", f"{cores} cores (8 recommended; the 4-core CI runner reaches "
               "real-time factor 0.1 to 0.2)", ["expect a slow run: the mission takes longer, nothing else changes"])


def check_running(report):
    status, out = run(["pgrep", "-a", "-f", r"gz sim|rviz2|ros2 launch openamrobot_build1_scene"])
    lines = [line for line in out.splitlines() if "live_demo_preflight" not in line]
    if lines:
        report.add("WARN", "running processes", f"{len(lines)} Gazebo, RViz or launch processes already run: "
                   + "; ".join(line[:70] for line in lines[:3]),
                   ["stop them first: pkill -f 'gz sim'; pkill rviz2 (only if they are yours and not needed)"])
    else:
        report.add("PASS", "running processes", "no other Gazebo, RViz or build1 launch")


def check_gui_config(report):
    path = os.path.expanduser("~/.gz/sim/8/gui.config")
    if not os.path.exists(path):
        report.add("PASS", "Gazebo GUI config", "none yet: gz-sim copies its default (with CameraTracking) on first start")
    elif 'filename="CameraTracking"' in read(path):
        report.add("PASS", "Gazebo GUI config", f"{path} has CameraTracking (the follow camera)")
    else:
        report.add("WARN", "Gazebo GUI config", f"{path} has no CameraTracking plugin: the follow camera cannot work",
                   [f"mv {path} {path}.bak  # gz-sim restores its default configuration on the next start"])


def check_recording(report, display_kind):
    status, out = run(["ffmpeg", "-hide_banner", "-devices"])
    if status == 127:
        report.add("WARN", "screen video", "ffmpeg missing: --record writes the ROS bag only", [apt_install(["ffmpeg"])])
    elif "x11grab" not in out:
        report.add("WARN", "screen video", "this ffmpeg has no x11grab: --record writes the ROS bag only",
                   [apt_install(["ffmpeg"]) + "  # Ubuntu's ffmpeg has x11grab"])
    elif display_kind != "x11":
        report.add("WARN", "screen video", f"{display_kind}: x11grab records X11 screens only; --record writes the bag, "
                   "record the screen with the desktop's recorder", ["GNOME: Ctrl+Shift+Alt+R, or OBS Studio"])
    else:
        report.add("PASS", "screen video", "ffmpeg x11grab available: --record also records the screen")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--ws", default=os.path.expanduser("~/openamr_ws"))
    parser.add_argument("--launcher", default=os.path.join(HERE, "run_grocery_demo_gui.sh"))
    parser.add_argument("--pins-yaml", default=None)
    parser.add_argument("--headless", action="store_true", help="no window is needed (server only)")
    parser.add_argument("--os-release", default=os.environ.get("GROCERY_DEMO_OS_RELEASE", "/etc/os-release"))
    parser.add_argument("--ros-setup", default=os.environ.get("GROCERY_DEMO_ROS_SETUP", "/opt/ros/jazzy/setup.bash"))
    args = parser.parse_args(argv)
    pins = parse_pins(read(args.launcher))
    if not pins:
        print(f"FAIL launcher              no pins found in {args.launcher}")
        return 1
    pins_yaml_path = args.pins_yaml or next(
        (p for p in (os.path.join(HERE, "..", "config", "pins.yaml"),
                     os.path.join(HERE, "..", "..", "share", PACKAGE, "config", "pins.yaml"),
                     os.path.join(args.ws, "src", "openamr-upperbody-sw", "ros2", "src", PACKAGE, "config", "pins.yaml"))
         if os.path.isfile(p)), "")
    report = Report()
    env = dict(os.environ)
    check_os(report, args.os_release)
    display_kind = check_display(report, env, args.headless)
    check_gpu(report, env, display_kind if not args.headless else "none")
    have_ros = check_ros(report, env, args.ros_setup)
    check_gazebo(report, pins, args.ros_setup, have_ros)
    check_packages(report)
    check_workspace(report, args.ws, pins, pins_yaml_path)
    check_vcan(report)
    check_resources(report, args.ws)
    check_running(report)
    check_gui_config(report)
    check_recording(report, display_kind)
    print(report.text(), flush=True)
    return 1 if report.failed() else 0


if __name__ == "__main__":
    sys.exit(main())
