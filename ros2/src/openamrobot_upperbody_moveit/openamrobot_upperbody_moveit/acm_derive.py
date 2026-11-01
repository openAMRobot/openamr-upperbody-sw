# SPDX-License-Identifier: MIT
"""The collision matrix, derived: a pair is disabled only for a reason that can be shown, and sampling is not one.

P12A replaced the matrix the Setup Assistant sampled (458 `Never` pairs that were only ever "no collision seen in N samples") with a matrix
that disables a pair for exactly three reasons, each computed from the model or read from upstream and none from a random sampler:

  upstream-declared   the pair is listed in upstream's own SRDF (`enactic/openarm_ros2` @ `4e837e1`): 20 pairs, all `Adjacent`, copied verbatim
                      into section 1 of our SRDF and diffed against the pinned checkout by test_srdf_import.py
  Adjacent            a structural neighbour: both links carry collision geometry, no other geometry-bearing link lies between them on the tree
                      path (frames and geometry-less links do not count), and at most one movable joint does: contact there is governed by the
                      joint's limits and by the assembly, not by the matrix
  ConstantTransform   certified by construction: no movable joint between the two links, so their relative transform is a constant and ONE
                      evaluation settles every configuration. Only pairs found CLEAR in that evaluation are disabled; a constant pair that
                      collides and is neither Adjacent nor upstream-declared would be a permanent collision of the model and is reported, not hidden

Everything else is enabled, including every pair whose only credential would be that a sampler saw no collision: a false `Never` hides a real
self-collision from the planner, a missing one costs CPU. Because nothing here samples, deriving the matrix twice gives the same matrix.

Pure Python over the URDF tree: the one evaluation that needs a collision checker is passed in (`collides`), so these functions are tested with no
ROS runtime and the script that runs them (scripts/derive_acm.py) supplies moveit_py's.
"""
import itertools

from openamrobot_upperbody_moveit.acm_split import movable_joints_between, pair_key, path_to_root, urdf_tree

ADJACENT, CONSTANT = "Adjacent", "ConstantTransform"
REASONS = (ADJACENT, CONSTANT)
UPSTREAM = "upstream-declared"


def geometry_links(urdf_root):
    """Links that carry a <collision> element: the only ones that can appear in the matrix."""
    return {link.get("name") for link in urdf_root.findall("link") if link.find("collision") is not None}


def links_between(tree, a, b):
    """The links strictly between `a` and `b` on the tree path (including the common ancestor when it is neither of them)."""
    up_a, up_b = path_to_root(tree, a), path_to_root(tree, b)
    common = next(link for link in up_a if link in up_b)
    middle = up_a[1:up_a.index(common)] + up_b[1:up_b.index(common)]
    return middle + ([common] if common not in (a, b) else [])


def structural_adjacent(urdf_root):
    """{pair} of structural neighbours: both links have collision geometry, no other geometry-bearing link between them on the tree path, and at
    most one movable joint on it. (Two links under one geometry-bearing link through two movable joints, a gripper's two fingers, are not
    structural neighbours by this rule; upstream declares that pair itself.)"""
    tree, geometry = urdf_tree(urdf_root), geometry_links(urdf_root)
    out = set()
    for a, b in itertools.combinations(sorted(geometry), 2):
        if not any(link in geometry for link in links_between(tree, a, b)) and len(movable_joints_between(tree, a, b)) <= 1:
            out.add(pair_key(a, b))
    return out


def constant_pairs(urdf_root):
    """{pair} of geometry-bearing links with no movable joint between them: their relative transform is constant."""
    tree, geometry = urdf_tree(urdf_root), geometry_links(urdf_root)
    return {pair_key(a, b) for a, b in itertools.combinations(sorted(geometry), 2) if not movable_joints_between(tree, a, b)}


def derive(urdf_root, upstream, collides):
    """-> (matrix {pair: reason}, report). `upstream` is {pair: reason} as upstream declares it (kept as it is); `collides(pairs)` returns the set of
    those pairs that collide in the ONE evaluation (the default state), which is all a constant pair needs.

    report: {"upstream": [...], "adjacent": [...] (derived, not upstream), "constant_transform": [...] (clear), "constant_and_colliding": [...]
    (constant, not disabled by any reason: a permanent collision, which must be looked at)}."""
    adjacent, constant = structural_adjacent(urdf_root), constant_pairs(urdf_root)
    matrix = {pair_key(*p): reason for p, reason in upstream.items()}
    derived_adjacent = sorted(adjacent - set(matrix))
    for pair in derived_adjacent:
        matrix[pair] = ADJACENT
    candidates = sorted(constant - set(matrix))
    colliding = set(collides(candidates)) if candidates else set()
    clear = [p for p in candidates if p not in colliding]
    for pair in clear:
        matrix[pair] = CONSTANT
    return matrix, {"upstream": sorted(pair_key(*p) for p in upstream), "adjacent": derived_adjacent, "constant_transform": clear,
                    "constant_and_colliding": sorted(colliding)}


def justifications(pair, urdf_root, upstream_pairs, adjacent=None, constant=None):
    """The set of reasons that can be shown for disabling `pair`: any of UPSTREAM, ADJACENT, CONSTANT. Empty means it should not be disabled."""
    adjacent = structural_adjacent(urdf_root) if adjacent is None else adjacent
    constant = constant_pairs(urdf_root) if constant is None else constant
    key = pair_key(*pair)
    out = set()
    if key in {pair_key(*p) for p in upstream_pairs}:
        out.add(UPSTREAM)
    if key in adjacent:
        out.add(ADJACENT)
    if key in constant:
        out.add(CONSTANT)
    return out
