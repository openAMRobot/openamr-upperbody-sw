# SPDX-License-Identifier: MIT
"""The Isaac Sim run kit (simulation/isaac/), checked without a GPU: the script's pins, the job's benchmark values, the result
collector's verdicts, and that the Isaac-side runner parses. The Isaac run itself stays BLOCKED until a GPU run's evidence exists."""
import ast
import json
import os
import pathlib
import shutil
import subprocess
import sys

import pytest
import yaml

from openamr_sim import benchmark as bm
from openamr_sim import expand as ex
from openamr_sim import isaac

SIM = pathlib.Path(__file__).resolve().parents[1]
KIT = SIM / "isaac"
sys.path.insert(0, str(KIT))
import collect_result  # noqa: E402


def test_run_script_parses_and_uses_the_pinned_image_and_xacro_build():
    subprocess.run(["bash", "-n", str(KIT / "run_on_gpu.sh")], check=True)
    out = subprocess.run(["bash", str(KIT / "run_on_gpu.sh"), "--dry-run"], cwd=SIM.parent, capture_output=True, text=True, check=True).stdout
    pins = yaml.safe_load((SIM / "sim_pins.yaml").read_text())
    lock_build = {e["inputs"]["xacro_package_version"] for e in ex.load_lock()["entries"]}.pop()
    assert f"isaac image     {pins['isaac_sim']['container_image']}" in out
    assert f"(xacro {lock_build})" in out
    assert f"platform-sw     {ex.pins()['openamr_platform_sw']}" in out
    assert f"openarm_desc    {ex.pins()['openarm_description']}" in out


def test_isaac_runner_parses_and_imports_isaac_only_inside_main():
    tree = ast.parse((KIT / "isaac_scenarios.py").read_text())
    top = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
    names = {a.name for n in top for a in n.names} | {n.module for n in top if isinstance(n, ast.ImportFrom)}
    assert not any(str(m).startswith(("isaacsim", "omni", "pxr")) for m in names), names


def test_job_carries_the_benchmark_values_of_the_expansion(expansions, tmp_path):
    j = isaac.build_job(expansions["bp000"], tmp_path)
    assert j["diffdrive"] == bm.diffdrive_params(expansions["bp000"].text)
    assert j["lift_velocity_limit"] == pytest.approx(0.018)


@pytest.fixture
def kit(expansions, tmp_path):
    job_dir = tmp_path / "sim" / "bp000" / "isaac"
    j = isaac.build_job(expansions["bp000"], job_dir)
    return tmp_path, job_dir, j


def test_collector_without_a_gpu_run_is_blocked(kit):
    root, _, _ = kit
    r = collect_result.collect(root)
    assert r["status"] == "BLOCKED"
    assert {c["check"]: c["status"] for c in r["checks"]} == {"bp000.usd_generated": "BLOCKED", "benchmark.ran": "BLOCKED"}
    assert all(row["verdicts"]["isaac"] == "BLOCKED" for row in r["comparison"])
    assert (root / "isaac_result.json").is_file()


def _fake_gpu_run(root, job_dir, job, roots, dofs, results):
    (job_dir / job["usd"]).write_text("#usda 1.0\n")
    (job_dir / "isaac_import_log.json").write_text(json.dumps({"result": "written", "articulation_roots": roots,
                                                                 "isaac_sim_build": "5.1.0-rc.19"}))
    (job_dir / "check_usd.json").write_text(json.dumps({"summary": {"checks": 120, "failures": 0}, "failures": []}))
    (root / "benchmark_isaac.json").write_text(json.dumps({"schema": bm.SCHEMA, "backend": "isaac", "configuration": "bp000",
                                                           "results": results, "meta": {"dof_count": dofs, "errors": {}}}))


def test_collector_passes_a_complete_run_and_fails_a_split_or_short_one(kit):
    root, job_dir, job = kit
    expected = len(collect_result.moving_joints(job_dir, job))
    assert expected == 21  # bp000 gazebo expansion: 14 arm and 4 finger revolute, 2 continuous wheels, the prismatic lift
    results = {"rest": {"base_height_m": 0.0, "base_pitch_deg": 0.5, "base_roll_deg": 0.0}}
    mujoco = root / "mujoco.json"
    mujoco.write_text(json.dumps({"results": results}))
    _fake_gpu_run(root, job_dir, job, ["/openamrobot/base_footprint"], expected, results)
    r = collect_result.collect(root, mujoco=str(mujoco))
    assert r["status"] == "PASS", r["checks"]
    assert r["configurations"]["bp000"]["usd_sha256"]
    rows = {(x["scenario"], x["metric"]): x for x in r["comparison"]}
    assert rows[("rest", "base_pitch_deg")]["verdicts"]["isaac"] == "PASS"
    _fake_gpu_run(root, job_dir, job, ["/a", "/b"], expected, results)
    assert collect_result.collect(root)["status"] == "FAIL"
    _fake_gpu_run(root, job_dir, job, ["/openamrobot/base_footprint"], expected - 1, results)
    r = collect_result.collect(root)
    assert r["status"] == "FAIL" and any(c["check"] == "bp000.dof_count" and c["status"] == "FAIL" for c in r["checks"])


def test_run_script_installs_xacro_the_same_way_as_ci():
    """When the hash-pinned wheel file exists (upperbody-sw #104), the kit installs xacro from it with --require-hashes,
    exactly as CI does, and does not install the apt build; before that it falls back to the lock's apt build."""
    text = (KIT / "run_on_gpu.sh").read_text()
    assert "--require-hashes --no-deps --ignore-installed -r simulation/requirements-xacro.txt" in text
    assert "if [ -f simulation/requirements-xacro.txt ]" in text
    pip_branch = text.split("if [ -f simulation/requirements-xacro.txt ]; then", 1)[1].split("else", 1)[0]
    assert "ros-jazzy-xacro=" not in pip_branch


# run_on_gpu.sh end to end without a GPU: docker, git and nvidia-smi are faked, the real collect_result.py runs. The fake Isaac
# containers write what the Isaac-side scripts write (or crash); the fake ROS container writes a minimal job per configuration,
# stubs check_usd.py (a report only when the USD exists, as the real checker needs the USD) and runs the real collector.
FAKE_DOCKER = r'''#!@PYTHON@
import json, os, pathlib, shlex, subprocess, sys
args = sys.argv[1:]
if not args or args[0] != "run":
    print("Docker version 0.0 (fake)" if args[:1] == ["--version"] else "Runtimes: nvidia")
    sys.exit(0)
env, workdir, i = {}, None, 1
while i < len(args) and args[i].startswith("-"):
    flag = args[i]
    if flag in ("--rm",):
        i += 1
        continue
    if flag.startswith("--") and "=" in flag:
        i += 1
        continue
    value = args[i + 1]
    if flag == "-e" and "=" in value:
        k, v = value.split("=", 1)
        env[k] = v
    if flag == "-w":
        workdir = value
    i += 2
image, rest = args[i], args[i + 1:]
stamp = env.get("OPENAMR_RUN_STAMP")
mode = os.environ["FAKE_ISAAC"]

def opt(tokens, name):
    return tokens[tokens.index(name) + 1]

if "isaac-sim" in image:
    if mode == "crash":
        print("Segmentation fault (fake)", file=sys.stderr)
        sys.exit(139)
    script = rest[0]
    if script.endswith("isaac_generate_usd.py"):
        job_path = pathlib.Path(opt(rest, "--job"))
        job = json.loads(job_path.read_text())
        (job_path.parent / job["usd"]).write_text("#usda 1.0\n")
        roots = ["/a", "/b"] if mode == "split" else ["/openamrobot/base_footprint"]
        (job_path.parent / "isaac_import_log.json").write_text(json.dumps(
            {"result": "written", "articulation_roots": roots, "isaac_sim_build": "5.1.0-rc.19", "run_stamp": stamp}))
    else:
        out = pathlib.Path(opt(rest, "--out"))
        out.write_text(json.dumps({"schema": "openamr.benchmark.v1", "backend": "isaac", "configuration": "bp000",
                                   "results": {"rest": {"base_height_m": 0.0}}, "run_stamp": stamp,
                                   "meta": {"dof_count": 2, "errors": {}}}))
    sys.exit(0)

kit = pathlib.Path(workdir) / "build" / "isaac_kit"
command = rest[-1]
if "generate.py" in command:
    for cfg in ("bp000", "bp050"):
        d = kit / "sim" / cfg / "isaac"
        d.mkdir(parents=True, exist_ok=True)
        (d / "model.urdf").write_text(
            '<robot name="r"><link name="a"/><link name="b"/><link name="c"/>'
            '<joint name="j1" type="revolute"><parent link="a"/><child link="b"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>'
            '<joint name="j2" type="continuous"><parent link="b"/><child link="c"/></joint></robot>')
        (d / "isaac_job.json").write_text(json.dumps({"configuration": cfg, "expansion_sha256": "0" * 64,
                                                      "urdf": "model.urdf", "usd": "openamrobot.usd"}))
    (kit / "benchmark_mujoco.json").write_text(json.dumps({"results": {"rest": {"base_height_m": 0.0}}}))
    sys.exit(0)
tokens = shlex.split(command.replace(";", " ; ").replace("||", " || ").replace("\n", " ; "))
rc = 0
for n, tok in enumerate(tokens):
    if tok.endswith("check_usd.py"):
        job = pathlib.Path(opt(tokens[n:], "--job"))
        if not (job.parent / json.loads(job.read_text())["usd"]).is_file():
            rc = 1
            continue
        pathlib.Path(opt(tokens[n:], "--json")).write_text(json.dumps(
            {"summary": {"checks": 10, "failures": 0}, "failures": [], "run_stamp": stamp}))
    if tok.endswith("collect_result.py"):
        tail = tokens[n + 1:]
        stop = next((k for k, t in enumerate(tail) if t in (";", "||", "&&")), len(tail))
        if os.environ.get("FAKE_COLLECTOR") == "crash":
            print("Traceback (most recent call last): RuntimeError (fake collector crash)", file=sys.stderr)
            rc = 1
        else:
            rc = subprocess.run([sys.executable, tok] + tail[:stop], cwd=workdir, env={**os.environ, **env}).returncode
        if tail[stop:stop + 2] == ["||", "true"]:
            rc = 0
sys.exit(rc)
'''

FAKE_GIT = r'''#!/usr/bin/env bash
# clone, fetch and checkout of the pinned sources only touch a directory; everything else is the real git
case "$1 $2 $4" in
  "clone "*) mkdir -p "${@: -1}/.git"; exit 0 ;;
esac
if [ "$1" = "-C" ] && [ -d "$2/.git" ] && [ ! -f "$2/.git/HEAD" ]; then
  case "$3" in
    fetch) exit 0 ;;
    checkout) echo "${@: -1}" > "$2/.git/PIN"; exit 0 ;;
    rev-parse) cat "$2/.git/PIN"; exit 0 ;;
  esac
fi
exec @GIT@ "$@"
'''


@pytest.fixture
def gpu_host(tmp_path):
    """A copy of the repository files the kit reads, with docker, git and nvidia-smi faked on PATH."""
    repo = tmp_path / "repo"
    shutil.copytree(SIM, repo / "simulation", ignore=shutil.ignore_patterns("__pycache__", "build"))
    pins = "ros2/src/openamrobot_upperbody_description/config/upstream_pins.yaml"
    (repo / pins).parent.mkdir(parents=True)
    shutil.copy(SIM.parent / pins, repo / pins)
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    for name, text in {"docker": FAKE_DOCKER.replace("@PYTHON@", sys.executable),
                       "git": FAKE_GIT.replace("@GIT@", shutil.which("git")),
                       "nvidia-smi": "#!/usr/bin/env bash\necho 'GPU 0: fake'\n"}.items():
        (bin_dir / name).write_text(text)
        (bin_dir / name).chmod(0o755)
    home = tmp_path / "home"
    home.mkdir()

    def run(*args, isaac="pass", collector="real"):
        env = {**os.environ, "PATH": f"{bin_dir}:{os.environ['PATH']}", "HOME": str(home),
               "FAKE_ISAAC": isaac, "FAKE_COLLECTOR": collector}
        return subprocess.run(["bash", str(repo / "simulation/isaac/run_on_gpu.sh"), *args], cwd=repo, env=env,
                              capture_output=True, text=True, timeout=120, check=False)
    run.kit = repo / "build" / "isaac_kit"
    return run


def test_run_script_exits_0_only_for_a_pass_of_this_run(gpu_host):
    r = gpu_host()
    assert r.returncode == 0, r.stdout + r.stderr
    assert "overall: PASS" in r.stdout
    result = json.loads((gpu_host.kit / "isaac_result.json").read_text())
    preflight = json.loads((gpu_host.kit / "preflight.json").read_text())
    assert result["status"] == "PASS" and result["run_stamp"] == preflight["utc"]


def test_run_script_exits_1_for_a_fail(gpu_host):
    r = gpu_host(isaac="split")
    assert r.returncode == 1, r.stdout + r.stderr
    assert "overall: FAIL" in r.stdout


def test_run_script_exits_3_when_isaac_crashes(gpu_host):
    r = gpu_host(isaac="crash")
    assert r.returncode == 3, r.stdout + r.stderr
    assert "overall: BLOCKED" in r.stdout


def test_run_script_is_blocked_when_the_collector_crashes(gpu_host):
    r = gpu_host(collector="crash")
    assert r.returncode == 3, r.stdout + r.stderr
    assert "BLOCKED" in r.stdout and "overall: PASS" not in r.stdout
    assert not (gpu_host.kit / "isaac_result.json").exists()


def test_run_script_cpu_only_exits_3(gpu_host):
    r = gpu_host("--cpu-only")
    assert r.returncode == 3, r.stdout + r.stderr
    assert "BLOCKED" in r.stdout


def test_stale_outputs_of_an_earlier_good_run_never_give_pass(gpu_host):
    assert gpu_host().returncode == 0  # an earlier good run leaves its outputs in build/isaac_kit
    r = gpu_host(isaac="crash")
    assert r.returncode == 3, r.stdout + r.stderr
    assert "overall: PASS" not in r.stdout
    assert json.loads((gpu_host.kit / "isaac_result.json").read_text())["status"] == "BLOCKED"
    assert not list(gpu_host.kit.glob("sim/*/isaac/isaac_import_log.json"))
    assert not (gpu_host.kit / "benchmark_isaac.json").exists()


def test_run_script_rejects_an_unknown_argument(gpu_host):
    r = gpu_host("--dryrun")
    assert r.returncode == 2, r.stdout + r.stderr
    assert "usage:" in r.stderr and "unknown argument: --dryrun" in r.stderr
    assert not gpu_host.kit.exists()


def test_collector_blocks_outputs_of_another_run(tmp_path):
    job_dir = tmp_path / "sim" / "bp000" / "isaac"
    job_dir.mkdir(parents=True)
    (job_dir / "model.urdf").write_text('<robot name="r"><link name="a"/><link name="b"/>'
                                        '<joint name="j" type="continuous"><parent link="a"/><child link="b"/></joint></robot>')
    (job_dir / "isaac_job.json").write_text(json.dumps({"configuration": "bp000", "expansion_sha256": "0" * 64,
                                                         "urdf": "model.urdf", "usd": "r.usd"}))
    _fake_gpu_run(tmp_path, job_dir, {"usd": "r.usd"}, ["/r"], 1, {"rest": {"base_height_m": 0.0}})
    for f in (job_dir / "isaac_import_log.json", job_dir / "check_usd.json", tmp_path / "benchmark_isaac.json"):
        f.write_text(json.dumps({**json.loads(f.read_text()), "run_stamp": "20261120T000000Z"}))
    assert collect_result.collect(tmp_path, stamp="20261120T000000Z")["status"] == "PASS"
    r = collect_result.collect(tmp_path, stamp="20261122T120000Z")
    assert r["status"] == "BLOCKED" and r["run_stamp"] == "20261122T120000Z"
    assert {c["check"]: c["status"] for c in r["checks"]} == {"bp000.usd_generated": "BLOCKED", "benchmark.ran": "BLOCKED"}
