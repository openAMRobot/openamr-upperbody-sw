# SPDX-License-Identifier: MIT
"""S6: the base Gazebo sensors of the combined model against the platform's sensor profiles (platform-sw #138).

  - every profile key that config/base_sensor_model.yaml names exists in the platform profile of the pinned platform-sw;
  - for each base sensor, the SDF values of the expansion are compared with the values the profile gives, and the rows that
    differ must be exactly the listed divergences (a closed one or a new one fails the test);
  - every Gazebo sensor of the combined robot is accounted for: a base sensor here, an upper-body camera in sensor_profiles.yaml,
    or a listed legacy sensor;
  - every effect Gazebo cannot represent is a documented parameter with a source or the status unknown, and a TODO for the
    sensor-fault layer;
  - the file copies no profile number: the gazebo rows hold keys, never values.
Structural only (xacro expansion, no Gazebo run): it proves which numbers the sensors carry, not how a simulated sensor behaves.
"""
import copy
import math
import os
import re

import pytest
from ament_index_python.packages import get_package_share_directory

from conftest import SHARE, expand, load_yaml

MODEL_FILE = os.path.join(SHARE, "config", "base_sensor_model.yaml")
UPPER_PROFILES_FILE = os.path.join(SHARE, "config", "sensor_profiles.yaml")
PLATFORM_PROFILES_FILE = os.path.join(get_package_share_directory("openamrobot_description"), "config", "sensor_profiles.yaml")
STATUSES = {"datasheet", "measured", "estimated", "unknown"}
GAP = re.compile(r"RG-SEN-\d{3}")
DERIVE = {
    "radians": math.radians,
    "half_angle_radians": lambda deg: math.radians(deg / 2.0),
    "samples_per_turn": lambda deg: round(360.0 / deg),
}
REL_TOL = {"half_angle_radians": 5e-3}        # the model writes 0.0175 for 0.017453 rad (documented in the row)


def _fill(obj, frame):
    if isinstance(obj, str):
        return obj.replace("{frame}", frame)
    if isinstance(obj, list):
        return [_fill(o, frame) for o in obj]
    if isinstance(obj, dict):
        return {k: _fill(v, frame) for k, v in obj.items()}
    return obj


def model():
    data = load_yaml(MODEL_FILE)
    data["sensors"] = {frame: _fill(copy.deepcopy(entry), frame) for frame, entry in data["sensors"].items()}
    return data


def platform():
    return load_yaml(PLATFORM_PROFILES_FILE)


def profile_mapping(prof, key):
    """'lidar_link.scan.range_min' -> the value mapping; 'imu_link.timestamp_skew' (no channel level) also works."""
    frame, *rest = key.split(".")
    node = prof["frames"][frame]["channels"]
    for part in rest:
        node = node[part]
    assert isinstance(node, dict) and "value" in node, f"{key} is not a value mapping"
    return node


def expected(row, prof):
    if row["from"] is None:
        return None
    v = profile_mapping(prof, row["from"])["value"]
    if v is None:
        return None
    if "derive" in row:
        v = DERIVE[row["derive"]](v)
    return v * row.get("scale", 1)


def sensor_element(root, frame, name):
    found = [s for g in root.findall("gazebo") if g.get("reference") == frame for s in g.findall("sensor")
             if s.get("name") == name]
    assert len(found) <= 1, f"{len(found)} sensors {name} on {frame}"
    return found[0] if found else None


def sdf_value(sensor, path):
    e = sensor.find(path)
    return None if e is None else float(e.text.strip())


def same(a, b, rel):
    if a is None or b is None:
        return a is None and b is None
    return math.isclose(a, b, rel_tol=rel, abs_tol=0.0)


def mismatches(root, frame, entry, prof):
    s = sensor_element(root, frame, entry["sensor"]) if entry["sensor"] else None
    if s is None:
        return {("*", "missing")}
    assert s.get("type") == entry["type"], f"{frame}: type {s.get('type')}, expected {entry['type']}"
    out = set()
    for row in entry["gazebo"]:
        got = sdf_value(s, row["sdf"])
        if not same(got, expected(row, prof), REL_TOL.get(row.get("derive"), 1e-9)):
            out.add((row["sdf"], got))
    return out


def all_from_keys(m):
    for frame, entry in m["sensors"].items():
        for kind in ("gazebo", "not_modelled"):
            for row in entry[kind]:
                if row.get("from"):
                    yield f"{frame}.{kind}", row["from"]


# --- the file -----------------------------------------------------------------------------------------------------------

def test_every_profile_key_exists_in_the_platform_profile_of_the_pin():
    prof = platform()
    keys = list(all_from_keys(model()))
    assert len(keys) > 40
    for where, key in keys:
        profile_mapping(prof, key)                                      # raises with the key if it is gone or renamed
    assert set(model()["sensors"]) <= set(prof["frames"]), "a base sensor frame without a platform profile entry"


def test_the_gazebo_rows_hold_keys_and_never_a_profile_number():
    for frame, entry in model()["sensors"].items():
        for row in entry["gazebo"]:
            assert set(row) <= {"sdf", "from", "scale", "derive", "note"}, f"{frame} {row['sdf']}: {sorted(row)}"
            assert row.get("derive") in (None, *DERIVE), row
            assert row["from"] is not None or row.get("note"), f"{frame} {row['sdf']}: no key and no note why"


@pytest.mark.parametrize("frame", list(load_yaml(MODEL_FILE)["sensors"]))
def test_the_base_sensor_differs_from_the_profile_exactly_where_listed(frame):
    m = model()
    entry = m["sensors"][frame]
    listed = {(d["sdf"], d["model"]) for d in entry["divergences"]}
    for d in entry["divergences"]:
        assert GAP.fullmatch(d["gap"]), d
    found = mismatches(expand(), frame, entry, platform())
    assert found == listed, (f"\n{frame}: divergences found {sorted(found, key=str)}\nlisted {sorted(listed, key=str)}\n"
                             f"update config/base_sensor_model.yaml (and {m['tracking']}) when platform-sw changes a sensor")


def test_a_changed_profile_value_is_seen_as_a_divergence(tmp_path):
    """The comparison reads the profile: a different scan rate in the profile makes the model's 10 Hz a divergence."""
    prof = platform()
    profile_mapping(prof, "lidar_link.scan.scan_rate")["value"] = 20.0
    entry = model()["sensors"]["lidar_link"]
    assert ("update_rate", 10.0) in mismatches(expand(), "lidar_link", entry, prof)
    assert ("update_rate", 10.0) not in mismatches(expand(), "lidar_link", entry, platform())


def test_the_gemini_field_of_view_is_the_depth_one_90_by_65():
    prof = platform()
    rows = {r["sdf"]: r for r in model()["sensors"]["base_camera_link"]["gazebo"]}
    assert expected(rows["camera/horizontal_fov"], prof) == pytest.approx(math.radians(90.0))
    assert profile_mapping(prof, "base_camera_link.depth.fov_vertical")["value"] == 65.0
    w, h = expected(rows["camera/image/width"], prof), expected(rows["camera/image/height"], prof)
    # Gazebo's vertical angle from the horizontal one and the aspect: 64.0 deg, the note's figure
    assert math.degrees(2 * math.atan(math.tan(math.radians(45.0)) * h / w)) == pytest.approx(64.0, abs=0.05)


def test_the_lidar_noise_is_not_taken_from_the_accuracy_figure():
    prof = platform()
    assert profile_mapping(prof, "lidar_link.scan.accuracy_0_8m")["value"] == 30.0
    rows = {r["sdf"]: r for r in model()["sensors"]["lidar_link"]["gazebo"]}
    assert rows["lidar/noise/stddev"]["from"] == "lidar_link.scan.range_noise_stddev"
    assert expected(rows["lidar/noise/stddev"], prof) is None


# --- every sensor of the combined robot ---------------------------------------------------------------------------------

def test_every_gazebo_sensor_of_the_combined_robot_is_accounted_for():
    m = model()
    root = expand(arm_package="openarm")
    found = {(g.get("reference"), s.get("name")) for g in root.findall("gazebo") for s in g.findall("sensor")}
    base = {(frame, e["sensor"]) for frame, e in m["sensors"].items() if e["sensor"]}
    legacy = {(e["link"], e["sensor"]) for e in m["legacy_sensors"]}
    upper = {(link, f"{link[:-len('_link')]}_sensor") for link in load_yaml(UPPER_PROFILES_FILE)["frames"]}
    assert found - base - legacy - upper == set(), "sensors with no profile entry"
    assert base <= found, f"listed base sensors missing from the expansion: {sorted(base - found)}"
    assert legacy <= found, "a legacy sensor is gone: remove it from legacy_sensors"


# --- effects Gazebo does not represent ----------------------------------------------------------------------------------

def _own_value_problems(where, row):
    status = row.get("status")
    if status not in STATUSES:
        return [f"{where}: status {status!r}"]
    out = []
    if not row.get("unit"):
        out.append(f"{where}: no unit")
    if status == "unknown":
        if row.get("value") is not None or row.get("source") is not None:
            out.append(f"{where}: unknown with a value or a source")
        if not row.get("needed") or not row.get("owner"):
            out.append(f"{where}: unknown without needed and owner")
    else:
        if not isinstance(row.get("value"), (int, float)) or isinstance(row.get("value"), bool):
            out.append(f"{where}: {status} without a number")
        src = " ".join(str(row.get("source")).split())
        if not (re.search(r"SHA-256 [0-9a-f]{64}", src) and re.search(r"(?i)\brev\b|document", src)):
            out.append(f"{where}: {status} without a document, revision and SHA-256")
    if row.get("proposed_for_platform_profile") is not True:
        out.append(f"{where}: a value of its own must be marked proposed_for_platform_profile")
    return out


def test_every_not_modelled_effect_is_a_documented_parameter_for_the_fault_layer():
    problems, n = [], 0
    for frame, entry in model()["sensors"].items():
        for row in entry["not_modelled"]:
            n += 1
            where = f"{frame}: {row.get('effect')}"
            if not GAP.fullmatch(str(row.get("gap"))):
                problems.append(f"{where}: no RG-SEN gap id")
            if not str(row.get("fault_layer", "")).startswith("TODO("):
                problems.append(f"{where}: no TODO for the fault layer")
            if row.get("from"):
                if "value" in row:
                    problems.append(f"{where}: both a profile key and a value of its own")
            else:
                problems += _own_value_problems(where, row)
    assert n >= 25
    assert not problems, "\n" + "\n".join(problems)


def test_the_imu_datasheet_rows_are_the_filed_ds_000347_rev_1_9():
    m, prof = load_yaml(MODEL_FILE), platform()
    sha = re.search(r"[0-9a-f]{64}", prof["documents"]["icm_42688_p_ds"]).group(0)
    rows = [r for r in m["sensors"]["imu_link"]["not_modelled"] if r.get("status") == "datasheet"]
    assert {(r["value"], r["unit"]) for r in rows} == {
        (0.5, "dps"), (0.005, "dps/degC"), (20.0, "mg"), (0.15, "mg/degC"), (0.028, "dps rms"), (0.70, "mg rms")}
    for r in rows:
        assert "DS-000347 rev 1.9" in r["source"] and sha in " ".join(r["source"].split())
    # the noise densities are reused from the platform profile, not restated
    assert profile_mapping(prof, "imu_link.gyro.noise_density")["value"] == 2.8
    assert profile_mapping(prof, "imu_link.accel.noise_density")["value"] == 70.0


def test_the_head_camera_effects_gazebo_cannot_represent_are_unknown_values_for_the_fault_layer():
    head = load_yaml(UPPER_PROFILES_FILE)["frames"]["head_camera_link"]["channels"]
    for channel, key in (("color", "latency"), ("color", "dropout"), ("depth", "latency"), ("depth", "invalid_fraction")):
        m = head[channel][key]
        assert m["value"] is None and m["status"] == "unknown" and m["needed"] and m["owner"], (channel, key)
        assert "TODO(sensor-fault layer)" in " ".join(m["note"].split()), (channel, key)
