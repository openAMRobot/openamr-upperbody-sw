# SPDX-License-Identifier: MIT
"""scripts/openarm_can_setup.sh against a fake `ip`: the commands it issues and the faults it refuses. No CAN hardware, no root."""
import os
import stat
import subprocess

import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, os.pardir, os.pardir, "scripts",
                      "openarm_can_setup.sh")

GOOD = """3: {i}: <NOARP,UP,LOWER_UP,ECHO> mtu 72 qdisc pfifo_fast state UP mode DEFAULT group default qlen 64
    link/can  promiscuity 0 minmtu 0 maxmtu 0
    can <FD,BERR-REPORTING> state ERROR-ACTIVE restart-ms 0
\t  bitrate 1000000 sample-point 0.750
\t  dbitrate 5000000 dsample-point 0.750
"""

FAKE_IP = """#!/usr/bin/env bash
echo "$*" >> "$FAKE_IP_LOG"
if [ "$1" = "-details" ]; then
  i=$4
  f="$FAKE_IP_DIR/$i.txt"
  if [ -f "$f" ]; then cat "$f"; exit 0; fi
  echo "Device \\"$i\\" does not exist." >&2; exit 1
fi
exit 0
"""


@pytest.fixture
def env(tmp_path):
    fake = tmp_path / "ip"
    fake.write_text(FAKE_IP)
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    log = tmp_path / "log"
    log.write_text("")
    e = dict(os.environ, IP=str(fake), FAKE_IP_LOG=str(log), FAKE_IP_DIR=str(tmp_path))
    e["_log"] = str(log)
    e["_dir"] = str(tmp_path)
    return e


def run(env, *args):
    e = {k: v for k, v in env.items() if not k.startswith("_")}
    return subprocess.run(["bash", SCRIPT, *args], env=e, capture_output=True, text=True, timeout=30)


def put(env, iface, text=None):
    with open(os.path.join(env["_dir"], f"{iface}.txt"), "w") as f:
        f.write(GOOD.format(i=iface) if text is None else text)


def calls(env):
    return open(env["_log"]).read().splitlines()


def test_dry_run_prints_the_four_commands_per_arm_and_touches_nothing(env):
    r = run(env, "--dry-run")
    assert r.returncode == 0, r.stderr
    assert calls(env) == []
    out = r.stdout
    for iface in ("can1", "can0"):
        assert f"link set {iface} type can bitrate 1000000 sample-point 0.75 dbitrate 5000000 dsample-point 0.75 dsjw 2 fd on restart-ms 0" in out
        assert f"link set {iface} txqueuelen 64" in out
    assert out.index("can1") < out.index("can0")  # left first


def test_interface_names_are_parameters(env):
    r = run(env, "--dry-run", "--left", "can5", "--right", "can6", "--txqueuelen", "128")
    assert "link set can5 type can" in r.stdout and "link set can6 type can" in r.stdout
    assert "can1" not in r.stdout and "txqueuelen 128" in r.stdout


def test_one_arm_only(env):
    r = run(env, "--dry-run", "--arm", "right", "--right", "can7")
    assert "can7" in r.stdout and "can1" not in r.stdout and "can0" not in r.stdout


def test_check_passes_on_a_good_interface_and_changes_nothing(env):
    put(env, "can0"), put(env, "can1")
    r = run(env, "--check")
    assert r.returncode == 0, r.stderr
    assert not any(c.startswith("link set") for c in calls(env))
    assert "ok: can1" in r.stdout and "ok: can0" in r.stdout


@pytest.mark.parametrize("bad, message", [
    (lambda t: t.replace("<FD,", "<"), "not in CAN FD mode"),
    (lambda t: t.replace("dbitrate 5000000", "dbitrate 2000000"), "data bitrate"),
    (lambda t: t.replace("bitrate 1000000", "bitrate 500000"), "nominal bitrate"),
    (lambda t: t.replace("restart-ms 0", "restart-ms 100"), "restart-ms"),
    (lambda t: t.replace("UP,LOWER_UP", "LOWER_UP"), "not UP"),
    (lambda t: t.replace("ERROR-ACTIVE", "BUS-OFF"), "BUS-OFF"),
    (lambda t: t.replace("link/can", "link/ether"), "not a CAN interface"),
])
def test_check_refuses_each_fault_with_a_clear_message(env, bad, message):
    put(env, "can1", bad(GOOD.format(i="can1")))
    r = run(env, "--check", "--arm", "left")
    assert r.returncode == 1
    assert message in r.stderr


def test_missing_interface_is_named(env):
    r = run(env, "--check", "--arm", "left")
    assert r.returncode == 1 and "'can1' not found" in r.stderr


@pytest.mark.parametrize("args, message", [
    (["--left", "can0"], "each arm needs its own CAN FD bus"),
    (["--left", "bad name"], "not a valid interface name"),
    (["--left", "x;reboot"], "not a valid interface name"),
    (["--arm", "middle"], "--arm must be"),
    (["--bitrate", "fast"], "not a positive integer"),
    (["--dbitrate", "500000"], "below the nominal bitrate"),
    (["--bogus"], "unknown argument"),
    (["--left"], "needs an interface name"),
])
def test_bad_arguments_are_refused_before_any_ip_call(env, args, message):
    r = run(env, "--dry-run", *args)
    assert r.returncode == 1 and message in r.stderr
    assert calls(env) == []


def test_changing_without_root_is_refused_with_a_hint(env):
    if os.geteuid() == 0:
        pytest.skip("running as root")
    r = run(env, "--arm", "left")
    assert r.returncode == 1 and "needs root" in r.stderr
    assert calls(env) == []


def test_down_dry_run(env):
    r = run(env, "--down", "--dry-run", "--arm", "left")
    assert r.stdout.strip() == "+ " + env["IP"] + " link set can1 down"


def test_help_exits_zero(env):
    r = run(env, "--help")
    assert r.returncode == 0 and "SocketCAN FD" in r.stdout
