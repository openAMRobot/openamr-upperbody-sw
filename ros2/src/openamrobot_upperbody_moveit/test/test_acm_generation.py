# SPDX-License-Identifier: MIT
"""The checked-in matrix IS the contract, and since P12A it is DERIVED, not sampled: config/acm/acm_generation.yaml records what it was derived by and
its fingerprint, and these tests read both back. Before P12A the matrix came from the Setup Assistant's sampler, which has no seed and gave a different
set of `Never` pairs on every run (473 against 468 at 3000 trials, 455, 456 and 458 at 10000); regenerating it was therefore not a verification step.
A derivation has no sampler, so regenerating must give the contract back every time, and that is tested by doing it."""
import hashlib
import os
import random
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix

from conftest import SHARE, SRDF_FILE, load_yaml, read_text, srdf_root
from openamrobot_upperbody_moveit.acm_split import acm_fingerprint, acm_pairs, diff_acm

DERIVE = os.path.join(get_package_prefix("openamrobot_upperbody_moveit"), "lib", "openamrobot_upperbody_moveit", "derive_acm.py")
META = os.path.join(SHARE, "config", "acm", "acm_generation.yaml")


def run_derive(*args, timeout=300):
    env = {**os.environ, "ROS_DOMAIN_ID": str(random.randint(201, 232)), "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST", "PYTHONDONTWRITEBYTECODE": "1"}
    return subprocess.run([sys.executable, DERIVE, *args], capture_output=True, text=True, timeout=timeout, env=env)


def test_the_checked_in_matrix_is_the_one_whose_fingerprint_is_on_record():
    """A change to the matrix must come with a new fingerprint in acm_generation.yaml: a visible change in a review."""
    meta = load_yaml("acm/acm_generation.yaml")
    pairs = acm_pairs(srdf_root())
    assert acm_fingerprint(pairs) == meta["contract"]["fingerprint"], \
        "the SRDF's collision matrix is not the one on record: either this is a deliberate change of the contract (derive_acm.py --write) or the " \
        "matrix was edited by hand"
    assert meta["contract"]["pairs"] == {"total": len(pairs), **{r: sum(1 for x in pairs.values() if x == r) for r in set(pairs.values())}}
    assert meta["contract"]["file"] == "config/openamrobot.srdf"


def test_the_record_says_the_matrix_is_derived_with_no_sampling_and_gives_the_three_reasons():
    meta = load_yaml("acm/acm_generation.yaml")
    assert meta["deterministic"] is True and "no sampling, no seed" in meta["method"]
    assert set(meta["rules"]) == {"upstream_declared", "Adjacent", "ConstantTransform", "everything_else"}
    assert "sampling falsifies" in meta["rules"]["everything_else"]
    pairs = acm_pairs(srdf_root())
    counts = meta["counts"]
    assert counts["disabled_total"] == len(pairs) == counts["upstream_declared"] + counts["adjacent_derived"] + counts["constant_transform"]
    assert counts["constant_transform"] == sum(1 for r in pairs.values() if r == "ConstantTransform")
    assert "Never" not in pairs.values() and "tool" not in meta, "the Setup Assistant's sampled matrix is gone"


def test_no_generation_setting_mentions_a_sample_count_or_a_seed_as_something_the_matrix_depends_on():
    text = read_text(META).lower()
    assert "trials" not in text and "collisions_updater" not in text.replace("no longer", "")


def test_the_fingerprint_follows_the_matrix_and_nothing_else():
    base = {("a", "b"): "ConstantTransform", ("a", "c"): "Adjacent"}
    assert acm_fingerprint(base) == acm_fingerprint(dict(reversed(list(base.items())))), "file order is not part of the matrix"
    assert acm_fingerprint(base) != acm_fingerprint({**base, ("b", "c"): "Adjacent"})
    assert acm_fingerprint(base) != acm_fingerprint({**base, ("a", "b"): "Adjacent"})
    assert diff_acm(base, {**base, ("b", "c"): "Adjacent", ("a", "b"): "Adjacent"}) == (
        {("b", "c"): "Adjacent"}, {}, {("a", "b"): ("ConstantTransform", "Adjacent")})


def test_regenerating_gives_the_contract_back_every_time(tmp_path):
    """The stability question of P12A, answered by running it: no sampler, so two derivations are the same matrix and it is the contract."""
    contract = acm_pairs(srdf_root())
    fingerprints = []
    for run in (1, 2):
        proposal = tmp_path / f"proposal{run}.srdf"
        done = run_derive("--srdf", SRDF_FILE, "--proposal", str(proposal))
        assert done.returncode == 0, done.stdout[-1500:] + done.stderr[-800:]
        assert "0 only in the derivation, 0 only in the contract, 0 relabelled" in done.stdout, done.stdout[-1500:]
        derived = acm_pairs(ET.parse(proposal).getroot())
        assert derived == contract
        fingerprints.append(acm_fingerprint(derived))
    assert fingerprints[0] == fingerprints[1] == load_yaml("acm/acm_generation.yaml")["contract"]["fingerprint"]


def test_a_proposal_leaves_the_contract_and_its_record_alone_and_only_write_replaces_them(tmp_path):
    """On COPIES of the SRDF and its record: never the repository's own files."""
    srdf, meta = tmp_path / "openamrobot.srdf", tmp_path / "acm_generation.yaml"
    shutil.copy(SRDF_FILE, srdf)
    shutil.copy(META, meta)
    contract = read_text(str(srdf))
    block_line = '    <disable_collisions link1="base_link" link2="bl_wheel_link" reason="ConstantTransform"/>\n'
    assert block_line in contract
    srdf.write_text(contract.replace(block_line, ""))                       # a contract that has lost a pair the derivation finds
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()           # noqa: E731
    before = (digest(srdf), digest(meta))
    proposal = tmp_path / "proposal.srdf"
    done = run_derive("--srdf", str(srdf), "--meta", str(meta), "--proposal", str(proposal))
    assert done.returncode == 0 and "1 only in the derivation" in done.stdout, done.stdout[-1500:]
    assert (digest(srdf), digest(meta)) == before, "a proposal changed the contract"
    assert acm_pairs(ET.parse(proposal).getroot()) == acm_pairs(ET.fromstring(contract))
    done = run_derive("--srdf", str(srdf), "--meta", str(meta), "--write")
    assert done.returncode == 0 and "THE CONTRACT CHANGED" in done.stdout, done.stdout[-1500:]
    assert acm_pairs(ET.parse(srdf).getroot()) == acm_pairs(ET.fromstring(contract)), "--write is how the contract is replaced"
    assert acm_fingerprint(acm_pairs(ET.parse(srdf).getroot())) in meta.read_text()
