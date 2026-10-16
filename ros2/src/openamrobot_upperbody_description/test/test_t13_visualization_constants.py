"""T13 (DATUMS.md, "Visualization constants are not datums"): the numeric literals the xacro keeps outside datums.yaml.

A number the xacro holds as a literal `xacro:property` is either a datum (then it lives in datums.yaml, and T13's other
tests apply) or a visualization constant. A visualization constant must satisfy the three narrow rules:

  1. it affects only visual geometry: every use sits inside a <visual>, and none inside a <collision>, an <inertial> or
     a <joint>, and none feeds another property (that would launder it past this check);
  2. it is a named property, and the comment immediately above it says it is a visualization constant;
  3. the evidence report lists the set (sidecar T13_visualization_constants.json).

A literal property that is not documented as one fails: this is also the literal-number audit for properties. Numbers
written inline in an attribute (the 1/2 of a box centre, structural zeros) are outside this check, as DATUMS.md allows.
"""
import re
import xml.etree.ElementTree as ET

import pytest

from conftest import write_sidecar, xacro_files

XACRO_NS = "{http://www.ros.org/wiki/xacro}"
NUMBER = re.compile(r"^\s*[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?\s*$")
PHRASE = re.compile(r"visuali[sz]ation constant", re.I)
FORBIDDEN_ANCESTORS = {"collision", "inertial", "joint"}


def _parse(text):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.fromstring(text, parser=parser)


def _parents(root):
    return {child: parent for parent in root.iter() for child in parent}


def literal_properties(root):
    """[(property element, the comment directly above it or '')] for every xacro:property whose value is a number."""
    parents = _parents(root)
    out = []
    for prop in root.iter(f"{XACRO_NS}property"):
        if not NUMBER.match(prop.get("value", "")):
            continue
        siblings = list(parents[prop])
        previous = siblings[siblings.index(prop) - 1] if siblings.index(prop) else None
        comment = previous.text if previous is not None and previous.tag is ET.Comment else ""
        out.append((prop, comment or ""))
    return out


def uses(root, name):
    """[(element, ancestors' tags)] for every attribute value or text that mentions the property, definition excluded."""
    parents = _parents(root)
    pattern = re.compile(rf"(?<![\w.]){re.escape(name)}(?![\w])")
    out = []
    for element in root.iter():
        if element.tag is ET.Comment or (element.tag == f"{XACRO_NS}property" and element.get("name") == name):
            continue
        attributes = [v for k, v in element.attrib.items() if not (element.tag == f"{XACRO_NS}property" and k == "name")]
        if any(pattern.search(v) for v in attributes):
            chain, node = [], element
            while node in parents:
                node = parents[node]
                chain.append(node.tag)
            out.append((element, [element.tag] + chain))
    return out


def constant_problems(xacro_text, filename="<xacro>"):
    """(problems, constants) for one xacro file: T13's visualization-constant rules."""
    root = _parse(xacro_text)
    problems, constants = [], []
    for prop, comment in literal_properties(root):
        name = prop.get("name")
        if not PHRASE.search(comment):
            problems.append(f"{filename}: property {name} = {prop.get('value')} is a literal number outside datums.yaml and its "
                            f"comment does not say it is a visualization constant: make it a datum, or document it (DATUMS.md)")
            continue
        found = uses(root, name)
        for element, tags in found:
            local = [t.replace(XACRO_NS, "xacro:") for t in tags]
            if "visual" not in tags:
                problems.append(f"{filename}: visualization constant {name} used outside a <visual> (in <{local[0]}>)")
            if FORBIDDEN_ANCESTORS & set(tags):
                bad = sorted(FORBIDDEN_ANCESTORS & set(tags))
                problems.append(f"{filename}: visualization constant {name} reaches <{'>/<'.join(bad)}> (in <{local[0]}>)")
        constants.append({"name": name, "value": float(prop.get("value")), "file": filename,
                          "comment": " ".join(comment.split()), "uses": len(found)})
    return problems, constants


def package_constants():
    problems, constants = [], []
    for path in xacro_files():
        with open(path) as f:
            p, c = constant_problems(f.read(), path.rsplit("/", 1)[-1])
        problems += p
        constants += c
    return problems, constants


# --- the real package ------------------------------------------------------------------------------------------------

def test_t13_visualization_constants_obey_the_three_rules():
    problems, constants = package_constants()
    write_sidecar("T13", "visualization_constants", "visualization constants: visual-only, named, commented, listed",
                  "FAIL" if problems else "PASS", {
                      "constants": constants, "count": len(constants), "failures": problems,
                      "rule": "DATUMS.md: affects only visual geometry; named property whose comment says so; listed here",
                  })
    assert not problems, "\n" + "\n".join(problems)


def test_t13_the_known_visualization_constant_is_found_and_used():
    _, constants = package_constants()
    assert [(c["name"], c["value"]) for c in constants] == [("openamr_sensor_marker_edge", 0.04)]
    assert constants[0]["uses"] >= 1                                     # the audit found where it is used


# --- the audit on synthetic xacro: it must be able to fail ------------------------------------------------------------

HEAD = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
GOOD = HEAD + """
  <!-- Visualization constant: how big the marker is drawn. -->
  <xacro:property name="edge" value="0.04"/>
  <link name="l"><visual><geometry><box size="${edge} ${edge} ${edge}"/></geometry></visual></link>
</robot>"""


def _problems(body):
    return constant_problems(HEAD + body + "</robot>")[0]


def test_t13_audit_accepts_a_documented_visual_only_constant():
    problems, constants = constant_problems(GOOD)
    assert not problems and [c["name"] for c in constants] == ["edge"] and constants[0]["uses"] == 1


@pytest.mark.parametrize("use,needle", [
    ('<link name="l"><collision><geometry><box size="${edge} 1 1"/></geometry></collision></link>', "outside a <visual>"),
    ('<link name="l"><inertial><mass value="${edge}"/></inertial></link>', "reaches <inertial>"),
    ('<joint name="j" type="fixed"><origin xyz="${edge} 0 0"/></joint>', "reaches <joint>"),
    ('<link name="l"><visual><geometry><box size="1 1 1"/></geometry></visual>'
     '<inertial><origin xyz="0 0 ${edge}"/></inertial></link>', "reaches <inertial>"),
    ('<xacro:property name="derived" value="${edge * 2}"/>', "outside a <visual>"),            # laundering through a property
    ('<link name="l"><visual><geometry><box size="1 1 1"/></geometry></visual>'
     '<collision><geometry><cylinder radius="${edge}" length="1"/></geometry></collision></link>', "outside a <visual>"),
])
def test_t13_audit_fails_a_use_that_reaches_something_physical(use, needle):
    problems = _problems('<!-- A visualization constant. -->\n<xacro:property name="edge" value="0.04"/>' + use)
    assert any(needle in p for p in problems), problems


@pytest.mark.parametrize("comment", ["", "<!-- the marker edge -->", "<!-- A drawing aid, not a hardware dimension. -->"])
def test_t13_audit_fails_a_literal_property_whose_comment_does_not_say_what_it_is(comment):
    problems = _problems(comment + '<xacro:property name="edge" value="0.04"/>')
    assert any("does not say it is a visualization constant" in p for p in problems), problems


def test_t13_audit_ignores_expression_properties_and_does_not_confuse_similar_names():
    problems, constants = constant_problems(HEAD + """
      <xacro:property name="computed" value="${1 / 2}"/>
      <xacro:property name="text" value="not a number"/>
      <!-- Visualization constant. -->
      <xacro:property name="edge" value="0.04"/>
      <link name="l"><visual><geometry><box size="${edge} ${edge_other} 1"/></geometry></visual>
        <collision><geometry><box size="${edge_other} 1 1"/></geometry></collision></link>
    </robot>""")
    assert not problems and [c["name"] for c in constants] == ["edge"]
