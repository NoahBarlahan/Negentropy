from copy import deepcopy
import pytest
from config import defaults, validate
from design_goals import optimize
from gait import preview
from model import build


def test_height_payload_pose_and_mass_locations_are_derived_without_mutation():
    source=defaults();snapshot=deepcopy(source)
    report=optimize(source,180.,100.,20.,15.,800.)
    assert source==snapshot
    c=validate(report['applied'])
    assert build(c)['height']*1000==pytest.approx(180,abs=.01)
    assert c['payload_mass']==c['payload_target']==100
    assert c['L_knee_servo_com_z']==-c['L_thigh']
    assert c['L_shin_print_com_z']==-c['L_shin']/2
    assert c['L_thigh']==c['R_thigh']
    assert c['L_hip']!=source['L_hip']
    assert c['stall']==source['stall'] and c['derating']==source['derating']
    assert c['L_thigh_print_mass']==source['L_thigh_print_mass']
    assert preview(c)['complete']


def test_payload_increases_load_screening_and_impossible_goals_preserve_source():
    c=defaults();snapshot=deepcopy(c)
    empty=optimize(c,157.,0.,20.,15.,800.)
    loaded=optimize(c,157.,100.,20.,15.,800.)
    assert loaded['chosen']['peak']>empty['chosen']['peak']
    failed=optimize(c,157.,1000.,20.,15.,800.)
    assert failed['applied'] is None
    assert failed['failures']['Mass']==18
    assert c==snapshot


def test_invalid_goals_and_incompatible_model_are_rejected():
    with pytest.raises(ValueError): optimize(defaults(),float('nan'),0,20,15,800)
    c=defaults();c['foot_model']='Flat sole'
    with pytest.raises(ValueError,match='Rounded nub'): optimize(c,150,0,20,15,800)
