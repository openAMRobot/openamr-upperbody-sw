# SPDX-License-Identifier: MIT
"""The pins, the documents and the wording rules of the Build 1 scene."""
import glob
import os
import re

import yaml

from conftest import PACKAGE_DIR, REPOSITORY, load_yaml, package_path

DOC = os.path.join(REPOSITORY, "docs", "build1_scene.md")
WORKFLOW = os.path.join(REPOSITORY, ".github", "workflows", "build1-scene-tests.yml")


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def own_text_files():
    files = [DOC, WORKFLOW]
    for pattern in ("*.md", "*.txt", "*.xml", "config/*", "launch/*", "scripts/*", "test/*.py", "urdf/*", "traces/*", "rviz/*",
                    "openamrobot_build1_scene/*.py"):
        files += glob.glob(os.path.join(PACKAGE_DIR, pattern))
    return sorted(path for path in files if os.path.isfile(path))


def test_pins_are_full_hashes_and_say_how_each_input_is_used():
    pins = load_yaml("config", "pins.yaml")["pins"]
    assert {"upperbody_sw_main", "upperbody_sw_pr66", "upperbody_sw_pr65", "platform_sw_main", "platform_sw_pr141", "platform_sw_pr139",
            "platform_sw_pr105", "audits", "openarm_ros2", "openarm_can", "openarm_description"} <= set(pins)
    for name, pin in pins.items():
        assert re.fullmatch(r"[0-9a-f]{40}", pin["sha"]), name
        assert pin["use"].strip(), name
    assert pins["upperbody_sw_main"]["sha"].startswith("b15bcb6")
    assert pins["platform_sw_main"]["sha"].startswith("0741778")
    assert pins["platform_sw_pr141"]["sha"].startswith("8bc5d1a") and pins["platform_sw_pr139"]["sha"].startswith("4a44f7d")
    assert pins["upperbody_sw_pr66"]["sha"].startswith("6fe5e9c") and pins["upperbody_sw_pr65"]["sha"].startswith("e3b5cb8")


def test_arm_driver_pins_equal_those_of_the_emulator_package():
    pins = load_yaml("config", "pins.yaml")["pins"]
    with open(os.path.join(PACKAGE_DIR, "..", "openamrobot_openarm_emulator", "config", "sil_pins.yaml"), encoding="utf-8") as f:
        emulator = yaml.safe_load(f)["pins"]
    for name in ("openarm_ros2", "openarm_can", "openarm_description"):
        assert pins[name]["sha"] == emulator[name], name


def test_the_workflow_reads_the_pins_file_and_fails_on_blocked():
    workflow = read(WORKFLOW)
    assert "config/pins.yaml" in workflow and "ubuntu-24.04" in workflow and "jazzy" in workflow
    assert "modprobe vcan" in workflow and "setup_vcan_all.sh" in workflow
    assert "sparse-checkout: ros2/src/openamrobot_sil" in workflow, "only openamrobot_sil of the open platform PR may be built"
    assert '[ "${TEST_RC:-1}" = "0" ] && [ "${COUNT_RC:-1}" = "0" ]' in workflow
    assert not re.search(r"[0-9a-f]{40}", re.sub(r"uses: \S+@[0-9a-f]{40}", "", workflow)), "pins belong in pins.yaml"
    names = set(re.findall(r"test_(scn\d\d_\w+)\.py", " ".join(os.listdir(package_path("test")))))
    listed = set()
    for group in re.findall(r'tests: "\^test_scn\(([^)]*)\)\$"', workflow):
        listed |= {f"scn{part}" for part in group.split("|")}
    assert names == listed, f"scenario tests and workflow groups differ: {sorted(names ^ listed)}"


def test_every_scenario_test_is_registered_with_a_timeout():
    cmake = read(package_path("CMakeLists.txt"))
    registered = dict(re.findall(r"(scn\d\d_\w+):(\d+)", cmake))
    files = set(re.findall(r"test_(scn\d\d_\w+)\.py", " ".join(os.listdir(package_path("test")))))
    assert files == set(registered)
    assert all(int(seconds) >= 600 for seconds in registered.values())
    for name in ("scene", "world_and_map", "launch_support", "command_map", "footprint", "odometry", "candump", "docs_and_pins"):
        assert os.path.isfile(package_path("test", f"test_{name}.py")) and name in cmake
    assert "RESOURCE_LOCK build1_vcan" in cmake


def test_emulator_limits_are_stated_where_results_depend_on_them():
    for path in (DOC, package_path("README.md"), package_path("openamrobot_build1_scene", "scenarios.py"),
                 package_path("config", "expected_reactions.yaml")):
        text = read(path)
        assert "Manual 1.03" in text and "CANopen Routine 1.08" in text and "V4.2" in text, path
        assert "2019 protocol copy" in text, path
        assert re.search(r"assumed power-up state", text), path
        assert "V1.02" not in text, f"{path}: the older statement about the drive manual was replaced"


def test_fault_reference_is_labelled_expected_not_certified():
    reference = read(package_path("config", "expected_reactions.yaml"))
    assert "NOT certified" in reference and "simulation-only" in reference.lower().replace("simulation only", "simulation-only")
    reactions = load_yaml("config", "expected_reactions.yaml")["reactions"]
    assert set(reactions) == {"drive_bus_loss", "bms_bus_loss", "aux_bus_loss", "arm_bus_loss", "stale_sensors", "arm_power_cut"}
    for name, entry in reactions.items():
        assert entry["expected"] and entry["source"].strip() and entry["fault"].strip(), name
    rig = read(package_path("openamrobot_build1_scene", "rig.py"))
    assert "not certified safety behaviour" in rig


def test_readme_has_the_single_writer_table_and_every_conflict():
    readme = read(package_path("README.md"))
    for conflict in ("SW-01", "SW-03", "SW-06", "SW-08", "SW-09", "SW-10"):
        row = next((line for line in readme.splitlines() if line.startswith(f"| {conflict} ")), None)
        assert row, f"{conflict} has no row in the README"
        assert row.count("|") == 6 and "test" in row.lower()
    command_map = load_yaml("config", "command_map.yaml")
    for joint in ("left_wheel_joint", "lift_joint", "finger_joint2"):
        assert joint in readme
    for topic in ("/odom", "/joint_states", "/cmd_vel", "/tf", "/planning_scene", "/sil/lift/gz_cmd_vel"):
        assert topic in command_map["topics"] and f"`{topic}`" in readme, topic
    for profile in ("geometry", "fast", "fine", "firmware_sil", "faults"):
        assert f"`{profile}`" in readme
    assert "Gazebo owns the rigid-body dynamics of every joint" in readme
    assert "fixed joints" in readme, "the README must report that the castors are fixed"


def test_the_document_says_modelled_assumed_and_not_modelled():
    doc = read(DOC)
    for heading in ("## Modelled", "## Assumed", "## Not modelled", "## Reality gaps this scene touches"):
        assert heading in doc
    assert "reality_gap_register" in doc and "be17653" in doc
    assert "Simulation only" in doc and "never safety evidence" in doc
    assert "estimated" in doc and "sourced" in doc


def test_checklist_and_rviz_config_cover_the_same_things():
    readme = read(package_path("README.md"))
    for item in ("TF tree", "Joint states", "Footprint", "Odometry", "Scan", "Depth", "Camera topics", "Collision objects",
                 "Planning scene", "Costmaps"):
        assert f"| {item} |" in readme, item
    rviz = yaml.safe_load(read(package_path("rviz", "build1.rviz")))
    displays = rviz["Visualization Manager"]["Displays"]
    names = {display["Name"] for display in displays}
    assert {"TF", "RobotModel", "Footprint", "Odometry", "Ground truth", "LaserScan", "Base depth", "Head depth", "Dock camera",
            "Left wrist camera", "Right wrist camera", "PlanningScene", "MotionPlanning", "Local costmap", "Global costmap"} <= names
    assert rviz["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"
    tools = [tool["Class"] for tool in rviz["Visualization Manager"]["Tools"]]
    assert not any("Goal" in tool or "Pose" in tool for tool in tools), "RViz must not be a writer of a command topic"


def test_plain_wording_rules():
    """No em dash, no en dash. The application is named Use Case 1 only. No price and no contact data."""
    for path in own_text_files():
        text = read(path)
        assert "\u2014" not in text, f"{path}: em dash"
        assert "\u2013" not in text, f"{path}: en dash"
        assert not re.search(r"[\w.+-]+@[\w-]+\.(com|ai|org|de|net)\b", text.replace("info@botshare.ai", "")), f"{path}: contact data"
        if not path.endswith(".sh"):
            assert not re.search(r"(EUR|USD|\u20ac|\$)\s?\d", text), f"{path}: a price"
        for word in ("hospital", "ward", "customer", "partner"):
            assert not re.search(rf"\b{word}\b", text, re.I) or os.path.basename(path) == "test_docs_and_pins.py", f"{path}: {word}"
        assert "UC1" not in re.sub(r"UC1-\d+|UC1:|\"UC1\"|'UC1'|UC1 (item|site|,)", "", text) or path.endswith(("scene.yaml", ".py")), path


def test_every_code_file_has_the_licence_header():
    for path in own_text_files():
        if path.endswith((".md", ".log", "package.xml", ".gitignore")):
            continue
        head = "\n".join(read(path).splitlines()[:3])
        assert "SPDX-License-Identifier: MIT" in head, path
    assert "<license>MIT</license>" in read(package_path("package.xml"))
