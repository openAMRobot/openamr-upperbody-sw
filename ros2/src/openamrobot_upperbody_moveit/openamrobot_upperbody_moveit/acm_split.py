# SPDX-License-Identifier: MIT
"""The kinematic tree of the combined model, and the collision matrix as a set of pairs: what the matrix derivation (acm_derive.py), M03 and the
contract fingerprint are built on.

(Until P12A this module also split the matrix into "inherited" and "ours" by link provenance. That split rested on a false premise, that the
`Never` pairs were upstream's, and is gone: upstream declares 20 pairs, all `Adjacent`, and every `Never` pair was written by our own sampler.)

Between two links of a kinematic tree there is exactly one path. If no joint on it can move (every one is `fixed` and none is a mimic), the
relative transform of the two links is a constant, so their collision status is the same in every configuration and one evaluation settles it for
all of them. Pure Python, no ROS runtime.
"""
import hashlib
import os
import xml.etree.ElementTree as ET


def pair_key(a, b):
    return tuple(sorted((a, b)))


def urdf_tree(urdf_root):
    """{child link: (parent link, joint name, movable)}. A joint is movable unless it is `fixed` and has no <mimic>: a mimic follows another
    joint, so the link it moves is not at a constant pose, however the joint is typed."""
    tree = {}
    for joint in urdf_root.findall("joint"):
        movable = joint.get("type") != "fixed" or joint.find("mimic") is not None
        tree[joint.find("child").get("link")] = (joint.find("parent").get("link"), joint.get("name"), movable)
    return tree


def path_to_root(tree, link):
    path = [link]
    while path[-1] in tree:
        path.append(tree[path[-1]][0])
    return path


def movable_joints_between(tree, a, b):
    """Names of the movable joints on the one path between links `a` and `b` ([] means their relative transform is constant)."""
    up_a, up_b = path_to_root(tree, a), path_to_root(tree, b)
    common = next((l for l in up_a if l in up_b), None)
    if common is None:
        raise ValueError(f"{a} and {b} are not in the same kinematic tree")
    joints = []
    for up in (up_a, up_b):
        for link in up[:up.index(common)]:
            joints.append(tree[link])
    return sorted(name for _, name, movable in joints if movable)


def parse_urdf(text):
    return ET.fromstring(text)


def acm_pairs(srdf_root):
    """{pair key: reason} for every disable_collisions of an SRDF (parsed). A pair listed twice is an error: each pair is disabled once."""
    pairs = {}
    for d in srdf_root.findall("disable_collisions"):
        key = pair_key(d.get("link1"), d.get("link2"))
        if key in pairs:
            raise ValueError(f"{key[0]} <-> {key[1]} is disabled twice")
        pairs[key] = d.get("reason")
    return pairs


def acm_fingerprint(pairs):
    """sha256 of the matrix as a set of (pair, reason) lines, sorted: the same matrix gives the same hash whatever the file's order, comments
    or whitespace, and any added, removed or relabelled pair changes it. This is what `the checked-in ACM is the contract` is checked against."""
    text = "".join(f"{a} {b} {reason}\n" for (a, b), reason in sorted(pairs.items()))
    return "sha256:" + hashlib.sha256(text.encode()).hexdigest()


def diff_acm(old, new):
    """What differs between two matrices ({pair: reason}): ({pair: reason} only in new, {pair: reason} only in old, {pair: (old, new)})."""
    added = {p: r for p, r in new.items() if p not in old}
    removed = {p: r for p, r in old.items() if p not in new}
    relabelled = {p: (old[p], new[p]) for p in old.keys() & new.keys() if old[p] != new[p]}
    return added, removed, relabelled
