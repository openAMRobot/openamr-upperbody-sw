# SPDX-License-Identifier: MIT
"""S5: the Gazebo camera sensors of the upper body come from config/sensor_profiles.yaml and from nothing else.

  - every camera link and optical frame that carries a sensor exists in the expansion, at the parent the contract says;
  - every sensor value is the profile's value (checked on the real profile, and on a profile with other numbers, where the
    sensor must follow);
  - every profile value has a source, or the status unknown with a null value, the document still needed and an owner;
  - a camera whose profile values are null gets no sensor (the wrist cameras: the part is not fixed, BOM ARM-004);
  - the sensors change no kinematics, mass or joint limit: an expansion with the sensors and one without differ only in <gazebo>.
"""
import copy
import math
import os
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

from conftest import ARM_PACKAGES, SHARE, expand, expand_xml, load_yaml, tree_facts

PROFILES_FILE = os.path.join(SHARE, "config", "sensor_profiles.yaml")
XACRO_FILE = os.path.join(SHARE, "urdf", "camera_sensors.xacro")
STATUSES = {"datasheet", "measured", "estimated", "unknown"}
SOURCED = {"datasheet", "measured", "estimated"}
WRISTS = ["left_wrist_camera_link", "right_wrist_camera_link"]
HEAD = "head_camera_link"
COLOUR_NEEDS = ["fov_horizontal", "width", "height", "frame_rate", "range_min", "range_max"]


def profiles():
    return load_yaml(PROFILES_FILE)


def sensors_of(root):
    """{link: <sensor> element} for every camera sensor of the upper body (the base camera is platform-sw's)."""
    out = {}
    for g in root.findall("gazebo"):
        ref = g.get("reference")
        for s in g.findall("sensor"):
            if ref and ref.endswith("_camera_link") and ref != "base_camera_link":
                assert ref not in out, f"two sensors on {ref}"
                out[ref] = s
    return out


def text(element, path):
    found = element.find(path)
    assert found is not None, f"no <{path}>"
    return found.text.strip()


def walk_values(profile):
    """Every value mapping of the profile: (path, mapping)."""
    for frame, entry in profile["frames"].items():
        for channel, values in entry["channels"].items():
            for name, mapping in values.items():
                yield f"{frame}.{channel}.{name}", mapping


# --- the profile file ---------------------------------------------------------------------------------------------------

def test_every_profile_value_has_a_source_or_is_unknown():
    problems = []
    n = 0
    for path, m in walk_values(profiles()):
        n += 1
        status = m.get("status")
        if status not in STATUSES:
            problems.append(f"{path}: status {status!r}")
        elif status == "unknown":
            if m.get("value") is not None or m.get("source") is not None:
                problems.append(f"{path}: unknown but has a value or a source")
            if not m.get("needed") or not m.get("owner"):
                problems.append(f"{path}: unknown without needed and owner")
        else:
            if not isinstance(m.get("value"), (int, float)) or isinstance(m.get("value"), bool):
                problems.append(f"{path}: {status} without a number")
            if not (isinstance(m.get("source"), str) and len(m["source"].split()) >= 4):
                problems.append(f"{path}: {status} without a source document")
        if not m.get("unit"):
            problems.append(f"{path}: no unit")
    assert n > 30
    assert not problems, "\n" + "\n".join(problems)


def test_every_source_names_a_document_revision_and_a_url_or_hash():
    for path, m in walk_values(profiles()):
        if m["status"] in SOURCED:
            src = " ".join(m["source"].split())
            assert re.search(r"https?://", src), f"{path}: no URL in the source"
            assert re.search(r"(?i)\b(rev(ision)?|version)\b|read \d+ \w+ 20\d\d", src), f"{path}: no revision or read date"


def test_profile_keys_are_camera_links_of_the_model_with_their_optical_frames():
    root = expand(arm_package="openarm")
    links, _, child_of = tree_facts(root)
    for link, entry in profiles()["frames"].items():
        assert link in links, f"{link} is not a link of the combined robot"
        optical = entry["optical_frame"]
        assert optical in links, f"{optical} is not a link"
        assert child_of[optical] == link, f"{optical} is not a child of {link}"


def test_wrist_camera_part_is_recorded_as_unknown():
    p = profiles()
    for link in WRISTS:
        assert p["frames"][link]["part"] is None
        assert all(m["status"] == "unknown" and m["value"] is None for _, m in walk_values({"frames": {link: p["frames"][link]}}))


def test_head_camera_values_are_the_zed_mini_datasheet():
    c = profiles()["frames"][HEAD]
    assert c["part"] == "ZED-121210"
    colour, depth = c["channels"]["color"], c["channels"]["depth"]
    assert colour["fov_horizontal"]["value"] == 102.0
    assert colour["fov_vertical"]["value"] == 57.0
    assert (colour["width"]["value"], colour["height"]["value"]) == (1280, 720)
    assert (depth["range_min"]["value"], depth["range_max"]["value"]) == (0.1, 15.0)
    assert depth["range_ideal_max"]["value"] == 9.0
    # no accuracy figure is used as noise
    assert colour["noise_stddev"]["status"] == "unknown" and depth["noise_stddev"]["status"] == "unknown"


# --- the sensors follow the profile -------------------------------------------------------------------------------------

@pytest.mark.parametrize("arm_package", ARM_PACKAGES)
def test_head_sensor_exists_and_takes_every_value_from_the_profile(arm_package):
    root = expand(arm_package=arm_package)
    sensors = sensors_of(root)
    assert HEAD in sensors
    entry = profiles()["frames"][HEAD]
    c, d = entry["channels"]["color"], entry["channels"]["depth"]
    s = sensors[HEAD]
    assert s.get("type") == "rgbd_camera"
    assert float(text(s, "update_rate")) == c["frame_rate"]["value"]
    assert float(text(s, "camera/horizontal_fov")) == pytest.approx(math.radians(c["fov_horizontal"]["value"]))
    assert int(text(s, "camera/image/width")) == c["width"]["value"]
    assert int(text(s, "camera/image/height")) == c["height"]["value"]
    assert float(text(s, "camera/clip/near")) == c["range_min"]["value"]
    assert float(text(s, "camera/clip/far")) == c["range_max"]["value"]
    assert float(text(s, "camera/depth_camera/clip/near")) == d["range_min"]["value"]
    assert float(text(s, "camera/depth_camera/clip/far")) == d["range_max"]["value"]
    assert text(s, "topic") == entry["gz_topic"]
    assert text(s, "gz_frame_id") == entry["optical_frame"]
    # unknown noise: no noise element, so nothing is invented
    assert s.find("camera/noise") is None


def test_every_sensor_is_on_an_existing_link_and_publishes_in_an_existing_frame():
    root = expand(arm_package="openarm")
    links, _, child_of = tree_facts(root)
    for link, s in sensors_of(root).items():
        assert link in links
        optical = text(s, "gz_frame_id")
        assert optical in links and child_of[optical] == link
        assert text(s, "pose") == "0 0 0 0 0 0"
        assert s.find("pose").get("relative_to") == link


@pytest.mark.parametrize("arm_package", ARM_PACKAGES)
def test_wrist_cameras_get_no_sensor_while_their_values_are_unknown(arm_package):
    assert not set(sensors_of(expand(arm_package=arm_package))) & set(WRISTS)


def _with_profile(tmp_path, edit, **mappings):
    data = profiles()
    edit(data)
    path = tmp_path / "profiles.yaml"
    path.write_text(yaml.safe_dump(data))
    return ET.fromstring(expand_xml(sensor_profiles_file=str(path), **mappings))


def test_a_different_profile_gives_a_different_head_sensor_with_no_value_from_elsewhere(tmp_path):
    def edit(d):
        ch = d["frames"][HEAD]["channels"]
        ch["color"]["fov_horizontal"]["value"] = 80.0
        ch["color"]["width"]["value"] = 640
        ch["color"]["height"]["value"] = 360
        ch["color"]["frame_rate"]["value"] = 15.0
        ch["color"]["range_min"]["value"] = 0.3
        ch["color"]["range_max"]["value"] = 7.5
        ch["depth"]["range_min"]["value"] = 0.4
        ch["depth"]["range_max"]["value"] = 6.5
    s = sensors_of(_with_profile(tmp_path, edit))[HEAD]
    assert float(text(s, "update_rate")) == 15.0
    assert float(text(s, "camera/horizontal_fov")) == pytest.approx(math.radians(80.0))
    assert (int(text(s, "camera/image/width")), int(text(s, "camera/image/height"))) == (640, 360)
    assert (float(text(s, "camera/clip/near")), float(text(s, "camera/clip/far"))) == (0.3, 7.5)
    assert (float(text(s, "camera/depth_camera/clip/near")), float(text(s, "camera/depth_camera/clip/far"))) == (0.4, 6.5)


def test_a_filled_wrist_profile_gives_a_wrist_sensor_and_a_noise_value_gives_a_noise_element(tmp_path):
    """The wrist sensors are not special-cased: they appear when, and only when, the profile says what they are."""
    def edit(d):
        for link in WRISTS:
            ch = d["frames"][link]["channels"] = copy.deepcopy(d["frames"][link]["channels"])
            for k, v in dict(fov_horizontal=70.0, width=640, height=480, frame_rate=30.0, range_min=0.05, range_max=2.0,
                             noise_stddev=0.01).items():
                ch["color"][k] = {"value": v, "unit": "x", "source": "test fixture, not a datasheet", "status": "estimated"}
    root = _with_profile(tmp_path, edit)
    sensors = sensors_of(root)
    for link in WRISTS:
        s = sensors[link]
        assert s.get("type") == "camera"
        assert float(text(s, "camera/horizontal_fov")) == pytest.approx(math.radians(70.0))
        assert float(text(s, "camera/noise/stddev")) == 0.01
        assert text(s, "gz_frame_id") == link.replace("_link", "_optical_frame")
    # no arms, no wrist cameras, whatever the profile says
    assert not set(sensors_of(_with_profile(tmp_path, edit, arm_package="none"))) & set(WRISTS)


def test_a_null_value_the_sensor_needs_removes_the_sensor(tmp_path):
    for need in COLOUR_NEEDS:
        def edit(d, need=need):
            d["frames"][HEAD]["channels"]["color"][need] = {"value": None, "unit": "x", "source": None, "status": "unknown",
                                                           "needed": "test", "owner": "test"}
        assert HEAD not in sensors_of(_with_profile(tmp_path, edit)), need


def test_a_depth_noise_value_stops_the_expansion_instead_of_being_dropped(tmp_path):
    def edit(d):
        d["frames"][HEAD]["channels"]["depth"]["noise_stddev"] = {"value": 0.01, "unit": "m", "source": "fixture", "status": "estimated"}
    with pytest.raises(Exception, match="depth noise_stddev"):
        _with_profile(tmp_path, edit)


def test_the_xacro_has_no_sensor_number_of_its_own():
    """Every number in an element of the sensor comes through ${...}; the only literals are the zero pose, the mean and the booleans."""
    src = re.sub(r"<!--.*?-->|<\?xml[^>]*\?>", "", open(XACRO_FILE).read(), flags=re.S)
    body = re.sub(r"\$\{[^}]*\}", "", src)                       # drop the expressions
    numbers = re.findall(r">\s*(-?\d[\d. ]*)\s*<", body)
    assert sorted(n.strip() for n in numbers) == ["0 0 0 0 0 0", "0.0"], numbers      # the sensor pose and the noise mean
    attrs = re.findall(r'="(-?\d[\d.]*)"', body)
    assert not attrs, attrs


# --- no change to the model ---------------------------------------------------------------------------------------------

def _without_sensors(root):
    """The expansion with the camera <gazebo> sensor blocks removed (and nothing else)."""
    root = copy.deepcopy(root)
    for g in list(root.findall("gazebo")):
        if g.find("sensor") is not None and (g.get("reference") or "").endswith("_camera_link") and g.get("reference") != "base_camera_link":
            root.remove(g)
    return ET.tostring(root)


@pytest.mark.parametrize("arm_package", ARM_PACKAGES)
def test_the_sensors_change_nothing_but_the_gazebo_blocks(arm_package, tmp_path):
    """Same links, joints, origins, limits, inertials and ros2_control with the shipped profile and with every value null."""
    def all_null(d):
        for _, m in walk_values(d):
            m.update(value=None, source=None, status="unknown", needed="test", owner="test")
    plain = _with_profile(tmp_path, all_null, arm_package=arm_package)
    assert not sensors_of(plain)
    shipped = expand(arm_package=arm_package)
    assert _without_sensors(shipped) == ET.tostring(plain)


def test_the_sensors_are_in_every_hardware_profile_and_do_not_touch_ros2_control():
    for profile in ("mock", "gazebo", "real"):
        root = expand(hardware_profile=profile)
        assert HEAD in sensors_of(root)
        assert HEAD not in {j.get("name") for r in root.findall("ros2_control") for j in r.findall("joint")}
