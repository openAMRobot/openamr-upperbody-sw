# SPDX-License-Identifier: MIT
"""T13: datums.yaml and the xacro agree, in both directions, with no hand-maintained list of references.

Every datum is in exactly one of three states, anything else FAILs:
  - referenced by the xacro,
  - listed under `report_only:` (reported, never feeds the model),
  - listed under `pending_reference:` as `<datum>: <step that will consume it>`.
A `pending_reference` entry whose consumer now exists FAILs too: it is referenced AND pending, so the step that adds
the consumer must delete the entry in the same commit.
"""
import re

import pytest

from conftest import load_datums, xacro_files

_Q = r"""['"]"""
_NAME = r"([A-Za-z0-9_]+)"
_COMMENT = re.compile(r"<!--.*?-->", re.S)


class UnparseableReference(Exception):
    """A datums[...] access whose key is not a string literal: it cannot be checked statically."""


def referenced_datums(xacro_text):
    """Datum keys referenced by one xacro file, found by parsing its text.

    Recognised: `datums['datums']['key']` and the same through any alias, where an alias is
      - a macro/call argument bound to `${datums}`  (<xacro:openamr_upper_sensors datums="${datums}" .../>), or
      - a property bound to `${<root>['datums']}`    (<xacro:property name="dt" value="${datums['datums']}"/>).
    A key that is not a string literal raises UnparseableReference instead of being skipped.
    """
    text = _COMMENT.sub("", xacro_text)
    roots = {"datums"} | set(re.findall(rf"\b{_NAME}\s*=\s*[\"']\$\{{\s*datums\s*\}}[\"']", text))
    keys, tables = set(), set()
    for root in roots:
        tables_re = rf"\b{re.escape(root)}\s*\[\s*{_Q}datums{_Q}\s*\]"
        for m in re.finditer(tables_re + rf"\s*\[\s*{_Q}([^'\"]+){_Q}\s*\]", text):
            keys.add(m.group(1))
        if re.search(tables_re + rf"\s*\[\s*(?!{_Q})", text):
            raise UnparseableReference(f"{root}['datums'][<non-literal>]")
        # `<xacro:property name="dt" value="${datums['datums']}"/>` (any attribute order)
        for prop in re.finditer(r"<xacro:property\b[^>]*>", text):
            tag = prop.group(0)
            name = re.search(r"\bname\s*=\s*\"(\w+)\"", tag)
            value = re.search(r"\bvalue\s*=\s*\"\$\{\s*" + tables_re + r"\s*\}\"", tag)
            if name and value:
                tables.add(name.group(1))
    for table in tables:
        for m in re.finditer(rf"(?<![\w.]){re.escape(table)}\s*\[\s*{_Q}([^'\"]+){_Q}\s*\]", text):
            keys.add(m.group(1))
        if re.search(rf"(?<![\w.]){re.escape(table)}\s*\[\s*(?!{_Q})", text):
            raise UnparseableReference(f"{table}[<non-literal>]")
    return keys


def referenced_in_package():
    keys = set()
    for path in xacro_files():
        with open(path) as f:
            keys |= referenced_datums(f.read())
    return keys


def reference_problems(datum_names, referenced, report_only, pending):
    """All T13 violations as readable strings (empty list = pass)."""
    datum_names, referenced = set(datum_names), set(referenced)
    report_only, pending = list(report_only), dict(pending)
    problems = []
    for key in sorted(referenced - datum_names):
        problems.append(f"{key}: referenced by the xacro but not defined in datums.yaml")
    for key in sorted(set(report_only) - datum_names):
        problems.append(f"{key}: listed under report_only but not defined in datums.yaml")
    for key in sorted(set(pending) - datum_names):
        problems.append(f"{key}: listed under pending_reference but not defined in datums.yaml")
    for key in sorted(datum_names):
        states = [s for s, hit in (("referenced by the xacro", key in referenced),
                                   ("report_only", key in report_only),
                                   ("pending_reference", key in pending)) if hit]
        if not states:
            problems.append(f"{key}: in none of referenced / report_only / pending_reference")
        elif len(states) > 1:
            extra = ""
            if key in pending and key in referenced:
                extra = f" ({pending[key]} has run: delete the pending_reference entry)"
            problems.append(f"{key}: in more than one state: {' + '.join(states)}{extra}")
    return problems


def test_t13_datums_and_xacro_agree():
    doc = load_datums()
    problems = reference_problems(doc["datums"], referenced_in_package(),
                                  doc.get("report_only", []), doc.get("pending_reference", {}))
    assert not problems, "\n" + "\n".join(problems)


def test_t13_extractor_finds_the_known_references():
    # Guards against the parser silently matching nothing.
    keys = referenced_in_package()
    assert {"column_axis_x_bp000", "lift_mass_dold_v4_350", "lift_mass_tl3_400", "lift_column_section",
            "left_wrist_camera_pose", "right_wrist_camera_pose"} <= keys
    assert "base_link_height_above_floor" not in keys, "it is report_only: a decision record asserted against the expansion by T04"


def test_t13_wrist_camera_poses_are_consumed_since_s4_and_the_head_camera_pose_is_gone():
    """S4 references the wrist camera poses, so it must have deleted their pending_reference entries. The head camera is
    option B's (P15): its placeholder datum is deleted, and nothing may bring it back."""
    doc = load_datums()
    referenced = referenced_in_package()
    assert "head_camera_pose" not in doc["datums"]
    for key in ("left_wrist_camera_pose", "right_wrist_camera_pose"):
        assert key in referenced, f"{key} should be consumed by upper_sensors.xacro"
        assert key not in doc.get("pending_reference", {}), f"{key} is referenced: delete its pending_reference entry"


# --- the checker and the extractor themselves ---------------------------------------------------------------

DATUMS = {"a", "b", "c", "d"}


def test_t13_checker_passes_when_every_datum_is_in_exactly_one_state():
    assert not reference_problems(DATUMS, {"a"}, ["b"], {"c": "S4", "d": "S5"})


@pytest.mark.parametrize("referenced,report_only,pending,needle", [
    ({"a", "zz"}, ["b"], {"c": "S4", "d": "S4"}, "zz: referenced by the xacro but not defined"),
    ({"a"}, ["b", "zz"], {"c": "S4", "d": "S4"}, "zz: listed under report_only but not defined"),
    ({"a"}, ["b"], {"c": "S4", "d": "S4", "zz": "S4"}, "zz: listed under pending_reference but not defined"),
    ({"a"}, ["b"], {"c": "S4"}, "d: in none of"),                                  # unreferenced, unlisted
    ({"a"}, ["b", "c"], {"c": "S4", "d": "S4"}, "c: in more than one state: report_only + pending_reference"),
    ({"a", "b"}, ["b"], {"c": "S4", "d": "S4"}, "b: in more than one state: referenced by the xacro + report_only"),
    ({"a", "c"}, ["b"], {"c": "S4", "d": "S4"}, "S4 has run: delete the pending_reference entry"),
], ids=["undefined-reference", "undefined-report-only", "undefined-pending", "unreferenced-unlisted",
        "report-only-and-pending", "referenced-and-report-only", "pending-but-already-referenced"])
def test_t13_checker_fails(referenced, report_only, pending, needle):
    problems = reference_problems(DATUMS, referenced, report_only, pending)
    assert any(needle in p for p in problems), problems


XACRO = """
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- datums['datums']['in_a_comment'] must not count -->
  <xacro:macro name="m" params="d">
    <xacro:property name="dt" value="${d['datums']}"/>
    <xacro:property name="x" value="${dt['via_alias']}"/>
    <xacro:property name="y" value="${d['datums']['via_param']['value']}"/>
  </xacro:macro>
  <xacro:property name="datums" value="${xacro.load_yaml(f)}"/>
  <xacro:property name="z" value="${datums['datums']['direct']['value']}"/>
  <xacro:m d="${datums}"/>
  <xacro:property value="${datums['datums']}" name="alias2"/>
  <xacro:property name="w" value="${alias2['reordered']}"/>
  <xacro:property name="o" value="${mast_ids['mast_ids']['not_a_datum']}"/>
</robot>
"""


def test_t13_extractor_on_synthetic_xacro():
    assert referenced_datums(XACRO) == {"via_alias", "via_param", "direct", "reordered"}


@pytest.mark.parametrize("snippet", [
    '<xacro:property name="k" value="${datums[\'datums\'][key]}"/>',
    '<xacro:property name="dt" value="${datums[\'datums\']}"/><xacro:property name="k" value="${dt[key]}"/>',
])
def test_t13_extractor_refuses_a_non_literal_key(snippet):
    with pytest.raises(UnparseableReference):
        referenced_datums(snippet)
