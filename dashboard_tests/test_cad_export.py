import io
import re
import zipfile
import pytest
from config import defaults, loads
from cad_export import parameters, equations, finalize, bundle, EQUATIONS_FILE


def test_importer_text_has_unique_quoted_variables_explicit_units_and_references():
    c=defaults();text=equations(c)
    text.encode('ascii')
    definitions={}
    for line in text.splitlines():
        match=re.fullmatch(r'"([A-Za-z][A-Za-z0-9_]*)" = (.+)',line)
        assert match
        name,rhs=match.groups()
        assert name not in definitions
        references=re.findall(r'"([A-Za-z][A-Za-z0-9_]*)"',rhs)
        if references:
            assert all(ref in definitions for ref in references)
        else:
            assert re.fullmatch(r'-?\d+(?:\.\d+)? (mm|deg)',rhs)
        definitions[name]=rhs
    assert definitions['ThighLength_L']=='65 mm'
    assert definitions['FootRadius_L']=='6 mm'
    assert definitions['WaistHeight']=='4 mm'
    assert len(definitions)==10
    assert not any(name.startswith(('Pose','Reference','Servo','Platform')) or 'Offset' in name or 'Diameter' in name for name in definitions)
    assert '\r\n' in text
    assert '@Sketch' not in text  # Never invent identifiers in a user's CAD.


def test_effective_symmetry_and_independent_right_geometry_are_exported():
    c=defaults();c['R_thigh']=99
    assert next(r for r in parameters(c) if r['variable']=='ThighLength_R')['value']==65
    c['symmetry']=False
    assert next(r for r in parameters(c) if r['variable']=='ThighLength_R')['value']==99


def test_snapshot_is_immutable_from_later_edits_and_package_is_consistent():
    c=defaults();snapshot=finalize(c)
    c['L_thigh']=90
    assert snapshot['config']['L_thigh']==65
    assert finalize(defaults())['revision']==snapshot['revision']
    with zipfile.ZipFile(io.BytesIO(bundle(snapshot))) as archive:
        assert archive.read(EQUATIONS_FILE).decode('ascii')==snapshot['text']
        assert loads(archive.read('biped_config.json').decode())==snapshot['config']
        assert 'center-to-center' in archive.read('parameter_map.csv').decode()
        assert snapshot['revision'] in archive.read('START_HERE.txt').decode()
        assert 'Tools > Equations > Import' in archive.read('START_HERE.txt').decode()


def test_flat_foot_uses_flat_dimensions_and_invalid_configuration_is_rejected():
    c=defaults();c['foot_model']='Flat sole'
    text=equations(c)
    assert '"FootLength_L" = 75 mm' in text
    assert 'FootRadius' not in text
    c['L_thigh']=float('nan')
    with pytest.raises(ValueError): finalize(c)


def test_optional_cad_cross_sections_are_explicit_and_do_not_alter_model():
    c=defaults()
    dims={'ThighWidth':20.,'ThighThickness':8.,'ShinWidth':18.,'ShinThickness':7.,'WaistHeight':40.}
    snapshot=finalize(c,dims)
    assert snapshot['config']==c
    assert '"ThighWidth" = 20 mm' in snapshot['text']
    assert '"WaistHeight" = 40 mm' in snapshot['text']
    assert snapshot['config']['platform_t']==4
    assert snapshot['revision']!=finalize(c)['revision']
    dims['ThighWidth']=100
    assert snapshot['dimensions']['ThighWidth']==20
    assert len(snapshot['parameters'])==14
    with zipfile.ZipFile(io.BytesIO(bundle(snapshot))) as archive:
        import json
        assert json.loads(archive.read('cad_dimensions.json'))==snapshot['dimensions']
    for invalid in ({'ThighWidth':0},{'Unknown':4},{'ShinThickness':float('inf')}):
        with pytest.raises(ValueError): finalize(c,invalid)
