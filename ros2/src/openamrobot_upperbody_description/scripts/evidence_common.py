"""Shared by the A6 evidence scripts and tests: where evidence goes, what the checkout is, which expansions are checked.

Not a command. Imported by print_config_id.py, check_urdf_matrix.py, build_evidence.py and the tests.
"""
import json
import os
import re
import subprocess
import sys

PKG = "openamrobot_upperbody_description"
SHA256_RE = re.compile(r"^sha256:[0-9a-f]{64}$")      # openamrobot-interfaces C9 `sha256` definition
EVIDENCE_ENV = "OPENAMR_EVIDENCE_DIR"
UPSTREAM_DIR = os.path.join("src", "upstream")
# Sidecars are `<check id>_<slug>.json`; build_evidence.py reads them and verify_a6.sh clears them before a run.
SIDECAR_RE = re.compile(r"^[A-Z]\d{2}_[A-Za-z0-9_]+\.json$")


class EvidenceError(RuntimeError):
    """Something the evidence needs cannot be determined. Raised instead of guessing."""


def share_dir():
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(PKG)


def package_share(name):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(name)


def workspace_root(share=None):
    """The colcon workspace: the parent of the nearest ancestor of the package share path that is named `install`.

    Deliberately not os.path.realpath: with --symlink-install, files under install/ point back into src/, and the
    directory itself does not.
    """
    path = os.path.abspath(share or share_dir())
    while True:
        parent = os.path.dirname(path)
        if parent == path:
            raise EvidenceError(
                f"cannot find the workspace from {share or share_dir()!r}: no ancestor named 'install'. "
                f"Set {EVIDENCE_ENV} to say where evidence goes.")
        if os.path.basename(path) == "install":
            return parent
        path = parent


def evidence_dir(share=None, environ=None):
    """`$OPENAMR_EVIDENCE_DIR` if set, else `<workspace>/evidence`. Created if missing. Never a temp directory:
    evidence a reviewer or CI cannot find is not evidence."""
    environ = os.environ if environ is None else environ
    path = environ.get(EVIDENCE_ENV) or os.path.join(workspace_root(share), "evidence")
    os.makedirs(path, exist_ok=True)
    return path


def write_sidecar(check_id, slug, name, result, details, directory=None):
    """Write `<directory>/<check_id>_<slug>.json` ({id, name, result, details}) and return its path."""
    assert result in ("PASS", "FAIL", "WARN", "SKIP"), result
    filename = f"{check_id}_{slug}.json"
    assert SIDECAR_RE.match(filename), filename
    path = os.path.join(directory or evidence_dir(), filename)
    with open(path, "w") as f:
        json.dump({"id": check_id, "name": name, "result": result, "details": details}, f, indent=2)
        f.write("\n")
    return path


def amend_sidecar(check_id, slug, key, value, fail=False, directory=None):
    """Add `details[key] = value` to a sidecar a test wrote earlier (a finding made after the launch was torn down, when the class that wrote
    the sidecar has finished) and set its result to FAIL when `fail`. It never turns a FAIL into a PASS. Returns the path, or None when there
    is no such sidecar (a test that died before writing one: the failing test already says so)."""
    path = os.path.join(directory or evidence_dir(), f"{check_id}_{slug}.json")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        doc = json.load(f)
    doc["details"][key] = value
    if fail:
        doc["result"] = "FAIL"
    return write_sidecar(doc["id"], slug, doc["name"], doc["result"], doc["details"], directory=os.path.dirname(path))


def read_sidecars(directory):
    """{check id: sidecar dict} for every sidecar in the directory."""
    out = {}
    for filename in sorted(os.listdir(directory)):
        if SIDECAR_RE.match(filename):
            with open(os.path.join(directory, filename)) as f:
                doc = json.load(f)
            out[doc["id"]] = {**doc, "file": filename}
    return out


def clear_sidecars(directory, only=None):
    """Delete the sidecars in `directory`; `only(filename)` narrows it (the two profiles' sidecars must not clear each other's:
    the Gazebo checks are G*, everything else belongs to the A6 report)."""
    for filename in os.listdir(directory):
        if SIDECAR_RE.match(filename) and (only is None or only(filename)):
            os.remove(os.path.join(directory, filename))


# --- the expansions every check runs over ---------------------------------------------------------------------------

# The four configuration IDs of option B (upperbody_configuration.yaml `configuration_axes`), in the order the xacro takes them.
CONFIG_KEYS = ("column", "base_plate", "head_pitch", "base_tilt")
ROW_KEYS = CONFIG_KEYS + ("arm_package", "hardware_profile")
ARM_PACKAGES = ("openarm", "so101", "none")


def load_upperbody(share=None):
    """Option B's configuration (config/upperbody_configuration.yaml, Alex's, merged on main): the axes, the baseline, the
    column data, the geometry and the lift calibration stops. The one source for every ID this repo accepts."""
    import yaml
    with open(os.path.join(share or share_dir(), "config", "upperbody_configuration.yaml")) as f:
        return yaml.safe_load(f)


def column_is_buildable(upperbody, column):
    """False for a column whose supplier data is null (tl3_400 today): its configuration does not expand."""
    data = upperbody["column"][column]
    return data["retracted_length"] is not None and data["stroke"] is not None


def baseline_row(upperbody, arm_package="openarm", hardware_profile="mock"):
    return {**upperbody["baseline"], "arm_package": arm_package, "hardware_profile": hardware_profile}


def expansion_matrix(upperbody):
    """T01's matrix as rows of ROW_KEYS: one factor at a time around option B's baseline, because the full product
    (2 x 2 x 5 x 3 x 3 x 3) is not what any check needs and its size would hide what the matrix is for.
      every non-baseline base_plate x arm_package on `mock`, and the baseline base_plate x arm_package  (2 x 3 = 6)
      every non-baseline head_pitch, and every non-baseline base_tilt, `openarm` on `mock`             (4 + 2 = 6)
      `openarm` on `gazebo` and `real` at the baseline                                                 (2)
    = 14 expansions, as before. Only buildable columns appear (a null-data column is refused, not expanded)."""
    base = upperbody["baseline"]
    axes = upperbody["configuration_axes"]
    column = base["column"]
    assert column_is_buildable(upperbody, column), f"the baseline column {column} has null supplier data"
    rows = [{**base, "base_plate": plate, "arm_package": arm, "hardware_profile": "mock"}
            for plate in [base["base_plate"]] + [p for p in axes["base_plate"] if p != base["base_plate"]]
            for arm in ARM_PACKAGES]
    for key in ("head_pitch", "base_tilt"):
        rows += [{**base, key: value, "arm_package": "openarm", "hardware_profile": "mock"}
                 for value in axes[key] if value != base[key]]
    rows += [baseline_row(upperbody, "openarm", profile) for profile in ("gazebo", "real")]
    return rows


def planar_extras(upperbody):
    """Not in T01's matrix: the planar flag at the baseline, per arm package. T02 runs them too, because the W02 reasoning
    ("check_urdf accepts the planar joint") would otherwise rest on a claim nobody ran."""
    return [baseline_row(upperbody, arm, "mock") for arm in ARM_PACKAGES]


def profile_extras(upperbody):
    """Not in T01's matrix: `openarm` on `gazebo` and `real` at the first non-baseline base_plate (bp050), so T02 has run
    check_urdf on the other profiles at a second column position too."""
    base = upperbody["baseline"]
    other = next(p for p in upperbody["configuration_axes"]["base_plate"] if p != base["base_plate"])
    return [{**baseline_row(upperbody, "openarm", profile), "base_plate": other} for profile in ("gazebo", "real")]


def row_label(row):
    """column/base_plate/head_pitch/base_tilt/arm_package/hardware_profile: one string per expansion."""
    return "/".join(row[k] for k in ROW_KEYS)


def row_mappings(row, enable_planar_base="false", datums_file=None):
    mappings = {k: row[k] for k in ROW_KEYS}
    mappings["enable_planar_base"] = enable_planar_base
    if datums_file:
        mappings["datums_file"] = datums_file
    return mappings


def lift_joint_position(upperbody, column, stop):
    """lift_joint position (m) of a calibration stop: the stop's shoulder height above the floor, minus the shoulder height at
    lift 0 (base-plate top + retracted length + the mount's z above the column top, all from option B's config). Stops are
    CALIBRATION IDs: joint positions with a calibration, never an input of the configuration id."""
    geo = upperbody["geometry"]
    at_zero = geo["base_plate_top_z"] + upperbody["column"][column]["retracted_length"] + geo["arm_mount"]["z_above_column_top"]
    return upperbody["lift_calibration_stops"][stop] - at_zero


def expand_urdf(share, row, enable_planar_base="false", datums_file=None):
    """Expanded URDF text of combined_robot.xacro (the installed package's) for one row of ROW_KEYS."""
    import xacro
    return xacro.process_file(os.path.join(share, "urdf", "combined_robot.xacro"),
                              mappings=row_mappings(row, enable_planar_base, datums_file)).toxml()


# --- what the checkout is -------------------------------------------------------------------------------------------

def _git(path, *args):
    return subprocess.run(["git", "-C", path, *args], capture_output=True, text=True)


def git_head(path):
    """Full commit hash of the checkout at `path`, or None if it is not one."""
    result = _git(path, "rev-parse", "HEAD")
    return result.stdout.strip() if result.returncode == 0 else None


def git_dirty(path):
    """(dirty, files). Compiled Python caches are generated by running the tests, not authored, so they do not count."""
    result = _git(path, "status", "--porcelain", "--untracked-files=all")
    files = [line[3:] for line in result.stdout.splitlines()
             if "__pycache__/" not in line and not line.endswith(".pyc")]
    return bool(files), files


def upstream_checkout(workspace, name):
    return os.path.join(workspace, UPSTREAM_DIR, name)


GIT_SOURCE = "git checkout"
METADATA_SOURCE = "package metadata (config/upstream_pins.yaml)"


HEX40_RE = re.compile(r"^[0-9a-f]{40}$")      # a full commit hash: no prefixes, so equality is the only comparison


def all_pins(pins_file=None):
    """{name: value} for every entry of config/upstream_pins.yaml, as YAML read it (so a number stays a number)."""
    import yaml
    pins_file = pins_file or os.path.join(share_dir(), "config", "upstream_pins.yaml")
    if not os.path.isfile(pins_file):
        return {}
    with open(pins_file) as f:
        return dict((yaml.safe_load(f) or {}).get("pins") or {})


def pinned_commit_in_metadata(name="openarm_description", pins_file=None):
    """The commit config/upstream_pins.yaml pins for `name`, or None. Every pin there is a quoted 40-hex string (C02)."""
    return all_pins(pins_file).get(name)


def upstream_commit_and_source(workspace, name="openarm_description", pins_file=None):
    """(commit, where it came from). The git checkout wins. With no checkout at all, the pin in the package metadata is
    used and labelled as such. A checkout with local changes is not 'the pinned commit' and raises: it is never replaced
    by the metadata, because that would hide the change."""
    path = upstream_checkout(workspace, name)
    head = git_head(path) if os.path.isdir(path) else None
    if head is None:
        pinned = pinned_commit_in_metadata(name, pins_file)
        if pinned is None:
            raise EvidenceError(f"no git checkout of {name} at {path} and no pin in config/upstream_pins.yaml: "
                                "pass the pinned commit explicitly")
        return pinned, METADATA_SOURCE
    dirty, files = git_dirty(path)
    if dirty:
        raise EvidenceError(f"{path} has local changes ({', '.join(files[:3])}): its HEAD is not the pinned commit")
    return head, GIT_SOURCE


def upstream_commit(workspace, name="openarm_description", pins_file=None):
    return upstream_commit_and_source(workspace, name, pins_file)[0]


def pin_vs_checkout(workspace, name="openarm_description", pins_file=None):
    """(result, details) for one pin of C02: the commit config/upstream_pins.yaml pins for `name` against the HEAD of its
    checkout.

    PASS when they are equal. FAIL when they differ (both hashes named), when the checkout has local changes (its HEAD is
    then not what anyone pinned), when the pin file has no pin for `name` but a checkout exists, or when the pin is not a
    full 40-hex string (a number YAML read from an all-digit hash, a 7-character prefix): that is checked with or without
    a checkout, because the pin is also what print_config_id.py falls back to. SKIP only when there is no checkout to
    compare with: then the config id uses the pin, so the two cannot disagree, and the check says so. That is the one SKIP
    this tree is allowed to have. It compares the pin with the checkout, never with upstream's tip, so a pin held behind
    upstream on purpose is not drift; moving the pin and the checkout is a change made in one commit.
    """
    path = upstream_checkout(workspace, name)
    pinned = pinned_commit_in_metadata(name, pins_file)
    head = git_head(path) if os.path.isdir(path) else None
    if pinned is not None and not isinstance(pinned, str):   # YAML read an all-digit hash as a number: quote it
        return "FAIL", {"reason": f"the pin for {name} in config/upstream_pins.yaml is {pinned!r}, not a string: quote it",
                        "pin": pinned, "checkout_head": head}
    if pinned is not None and not HEX40_RE.match(pinned):
        return "FAIL", {"reason": f"the pin for {name} in config/upstream_pins.yaml is {pinned!r}, not a full 40-hex "
                                  "commit hash: a prefix cannot be compared exactly and would give the fallback config "
                                  "id a different value from the checkout's",
                        "pin": pinned, "checkout_head": head}
    if head is None:
        return "SKIP", {"reason": f"no git checkout of {name} at {path}, so there is nothing to compare the pin with. "
                                  "This is the only legitimate SKIP in this tree: without a checkout the config id "
                                  "falls back to the pin, which therefore cannot disagree with it",
                        "pin": pinned}
    if pinned is None:
        return "FAIL", {"reason": f"config/upstream_pins.yaml has no pin for {name}, but a checkout exists at {path}",
                        "checkout_head": head}
    dirty, files = git_dirty(path)
    if dirty:
        return "FAIL", {"reason": f"{path} has local changes ({', '.join(files[:3])}): its HEAD is not a pinned commit",
                        "pin": pinned, "checkout_head": head}
    if head != pinned:
        return "FAIL", {"reason": f"config/upstream_pins.yaml pins {name} at {pinned} but the checkout is at {head}: a "
                                  "config id computed without the checkout would not match the one computed with it. "
                                  "Move the pin and the checkout together, in one commit",
                        "pin": pinned, "checkout_head": head}
    return "PASS", {"pin": pinned, "checkout_head": head, "checkout": path}


def pins_vs_checkouts(workspace, pins_file=None, required=("openarm_description",)):
    """(result, details) for C02 as a whole: pin_vs_checkout for every entry of the pin file, plus the `required` names
    (openarm_description is what the config id is built from, so a file with no pin for it fails when its checkout is
    there). FAIL if any pin FAILs, each reason named. SKIP only if no pin had a checkout to compare with. Otherwise PASS,
    with the pins that had no checkout listed under `not_compared`: a pin nothing checks is a pin that drifts, so the
    report says which ones those were instead of letting a PASS cover them."""
    names = list(dict.fromkeys([*all_pins(pins_file), *required]))
    per_pin = {name: pin_vs_checkout(workspace, name, pins_file) for name in names}
    rows = {name: {"result": result, **details} for name, (result, details) in per_pin.items()}
    failed = {name: row["reason"] for name, row in rows.items() if row["result"] == "FAIL"}
    not_compared = [name for name, row in rows.items() if row["result"] == "SKIP"]
    details = {"pins": rows, "compared": [n for n, r in rows.items() if r["result"] == "PASS"], "not_compared": not_compared}
    if failed:
        return "FAIL", {**details, "reason": "; ".join(f"{name}: {why}" for name, why in failed.items())}
    if len(not_compared) == len(rows):
        return "SKIP", {**details, "reason": "; ".join(f"{n}: {rows[n]['reason']}" for n in not_compared)}
    return "PASS", details


def pinned_commit_in_baseline(workspace, name="openarm_description"):
    """The commit openamrobot-manipulation's baseline .repos pins for `name`, or None if that file is not there."""
    import yaml
    path = os.path.join(workspace, "src", "openamrobot-manipulation", "integration", "openarm_v2_fake_baseline",
                        "openarm_v2.dev.repos")
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        return yaml.safe_load(f)["repositories"][name]["version"]


if __name__ == "__main__":
    sys.exit("evidence_common.py is a library; see print_config_id.py and verify_a6.sh")
