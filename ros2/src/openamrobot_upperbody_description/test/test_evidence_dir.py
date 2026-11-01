# SPDX-License-Identifier: MIT
"""Where evidence goes. S4 defaulted to the system temp directory, and evidence a reviewer or CI cannot find is not
evidence. The default is now `<workspace>/evidence`, found from the package share path; `OPENAMR_EVIDENCE_DIR` overrides;
when neither can be determined it is an error, never a temp directory.
"""
import os

import pytest

import evidence_common as ev
from conftest import SCRIPTS, SHARE, write_sidecar


def _parent_of_the_install_ancestor(path):
    """The expected workspace, worked out from the path alone and independently of evidence_common: the parent of the nearest path component
    named `install`. It does not look for, or require, any sibling directory (a `src/` in particular)."""
    parts = os.path.abspath(path).split(os.sep)
    index = len(parts) - 1 - parts[::-1].index("install")
    return os.sep.join(parts[:index]) or os.sep


def test_default_is_the_workspace_evidence_directory_found_from_the_share_path():
    """Found from the SHARE PATH, wherever the sources sit. This test used to also assert `<workspace>/src` exists: that is how some colcon
    workspaces are laid out, not this repository's (its sources are under ros2/src, and a checkout that is built in place has no root-level
    src/). The contract is install -> workspace -> evidence, and nothing about the source tree."""
    directory = ev.evidence_dir(SHARE, environ={})
    workspace = _parent_of_the_install_ancestor(SHARE)
    assert directory == os.path.join(workspace, "evidence")
    assert os.path.isdir(os.path.join(workspace, "install"))
    assert os.path.isdir(directory)


@pytest.mark.parametrize("root,sources", [
    ("checkout", "checkout/ros2/src/pkg"),                       # this repository built from its root: no root-level src/ anywhere
    ("checkout/ros2", "checkout/ros2/src/pkg"),                  # built from ros2/: src/ is a sibling of install/
    ("ws", "ws/src/repo/ros2/src/pkg"),                          # a conventional workspace with this repository cloned into src/
])
def test_the_evidence_directory_does_not_depend_on_where_the_sources_sit(tmp_path, root, sources):
    """A checkout with no root-level src/ (the first case) must resolve and create `<root>/evidence`; it cannot regress to demanding a src/."""
    (tmp_path / sources).mkdir(parents=True)
    share = tmp_path / root / "install" / "pkg" / "share" / "pkg"
    share.mkdir(parents=True)
    if root == "checkout":
        assert not (tmp_path / "checkout" / "src").exists()
    directory = ev.evidence_dir(str(share), environ={})
    assert directory == str(tmp_path / root / "evidence") and os.path.isdir(directory)
    assert _parent_of_the_install_ancestor(str(share)) == str(tmp_path / root)


def test_the_env_var_overrides_and_the_directory_is_created(tmp_path):
    target = tmp_path / "somewhere" / "else"
    assert ev.evidence_dir(SHARE, environ={ev.EVIDENCE_ENV: str(target)}) == str(target)
    assert target.is_dir()


@pytest.mark.parametrize("layout", ["ws/install/pkg/share/pkg", "ws/install/share/pkg", "ws/install/pkg/share/pkg/deeper"])
def test_the_workspace_is_the_parent_of_the_install_directory_in_isolated_and_merged_layouts(tmp_path, layout):
    share = tmp_path / layout
    share.mkdir(parents=True)
    assert ev.workspace_root(str(share)) == str(tmp_path / "ws")
    assert ev.evidence_dir(str(share), environ={}) == str(tmp_path / "ws" / "evidence")


def test_symlinked_files_under_install_do_not_move_the_workspace(tmp_path):
    # --symlink-install links files back into src/; the directory path must not be resolved through them
    source = tmp_path / "ws" / "src" / "repo" / "pkg"
    source.mkdir(parents=True)
    share = tmp_path / "ws" / "install" / "pkg" / "share" / "pkg"
    share.mkdir(parents=True)
    (share / "config").symlink_to(source)
    assert ev.workspace_root(str(share / "config")) == str(tmp_path / "ws")


def test_with_no_install_ancestor_and_no_env_var_it_is_an_error_not_a_temp_directory(tmp_path):
    with pytest.raises(ev.EvidenceError, match=ev.EVIDENCE_ENV):
        ev.evidence_dir(str(tmp_path / "opt" / "ros" / "share" / "pkg"), environ={})


def test_the_t04_sidecar_helper_writes_to_the_evidence_directory_not_a_temp_one(monkeypatch, tmp_path):
    monkeypatch.setenv(ev.EVIDENCE_ENV, str(tmp_path))
    path = write_sidecar("T99", "probe", "probe", "PASS", {})
    assert os.path.dirname(path) == str(tmp_path)


def test_no_source_in_tests_or_scripts_asks_for_the_system_temp_directory_for_evidence():
    needle = "get" + "tempdir"
    here = os.path.dirname(os.path.abspath(__file__))
    offenders = []
    for directory in (here, SCRIPTS):
        for name in sorted(os.listdir(directory)):
            path = os.path.join(directory, name)
            if name.endswith(".py") and name != os.path.basename(__file__):
                with open(path) as f:
                    if needle in f.read():
                        offenders.append(name)
    assert not offenders, f"{offenders} use the system temp directory; evidence belongs in the evidence directory"


def test_sidecar_names_and_clearing_touch_only_sidecars(tmp_path):
    ev.write_sidecar("T99", "probe", "probe", "WARN", {"k": 1}, directory=str(tmp_path))
    for keep in ("S1.json", "20261005T000000Z_dev.json", "REPO_SYNC_2026-10-05.md", "colcon_build.log"):
        (tmp_path / keep).write_text("keep")
    assert set(ev.read_sidecars(str(tmp_path))) == {"T99"}
    ev.clear_sidecars(str(tmp_path))
    assert sorted(os.listdir(tmp_path)) == sorted(["S1.json", "20261005T000000Z_dev.json", "REPO_SYNC_2026-10-05.md", "colcon_build.log"])
    with pytest.raises(AssertionError):
        ev.write_sidecar("T99", "bad name!", "x", "PASS", {}, directory=str(tmp_path))
    with pytest.raises(AssertionError):
        ev.write_sidecar("T99", "x", "x", "MAYBE", {}, directory=str(tmp_path))
