"""T06: a bad value for any arg fails expansion with a message listing the accepted values."""
import pytest
import xacro

from conftest import ARM_PACKAGES, AXES, COMBINED, HARDWARE_PROFILES, expand_xml, read_text

BAD = [
    ("column", "dold_v4_999", AXES["column"]),
    ("column", "", AXES["column"]),
    ("base_plate", "bp100", AXES["base_plate"]),    # one of ADR-010's five rev18.7 positions: there are two now
    ("base_plate", "BP000", AXES["base_plate"]),
    ("head_pitch", "hp40", AXES["head_pitch"]),
    ("head_pitch", "25", AXES["head_pitch"]),
    ("base_tilt", "bt20", AXES["base_tilt"]),
    ("arm_package", "openarm2", ARM_PACKAGES),
    ("arm_package", "OpenArm", ARM_PACKAGES),
    ("hardware_profile", "sim", HARDWARE_PROFILES),
    ("hardware_profile", "", HARDWARE_PROFILES),
    ("enable_planar_base", "yes", ["true", "false"]),
    ("enable_planar_base", "True", ["true", "false"]),
]


@pytest.mark.parametrize("arg,value,accepted", BAD, ids=[f"{a}={v!r}" for a, v, _ in BAD])
def test_t06_bad_value_errors_with_accepted_list(arg, value, accepted):
    with pytest.raises(xacro.XacroException) as err:
        expand_xml(**{arg: value})
    msg = str(err.value)
    assert f"Invalid xacro arg {arg}:={value}" in msg
    assert "Accepted values: " + ", ".join(accepted) in msg


def test_t06_missing_datums_file_errors_naming_the_path():
    with pytest.raises(xacro.XacroException) as err:
        expand_xml(datums_file="/nonexistent/datums.yaml")
    assert "/nonexistent/datums.yaml" in str(err.value)


def test_t06_the_retired_mast_id_argument_is_not_declared_and_changes_nothing():
    assert 'name="mast_id"' not in read_text(COMBINED)
    assert expand_xml(mast_id="mast_1450") == expand_xml()


def test_t06_a_null_datum_that_is_read_stops_the_expansion_and_names_it(tmp_path):
    """The hard-fail: lift_mass_dold_v4_350 made null (as lift_mass_tl3_400 is) is not replaced by a guess."""
    import yaml
    from conftest import load_datums
    datums = load_datums()
    datums["datums"]["lift_mass_dold_v4_350"].update(value=None, source="placeholder", status="placeholder")
    del datums["datums"]["lift_mass_dold_v4_350"]["density_band"]
    datums["revision"] = "2099-01-01.a"
    path = tmp_path / "datums_null_lift_mass.yaml"
    path.write_text(yaml.safe_dump(datums))
    with pytest.raises(xacro.XacroException) as err:
        expand_xml(datums_file=str(path))
    assert "datum lift_mass_dold_v4_350 is null" in str(err.value)
