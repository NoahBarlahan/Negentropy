from copy import deepcopy
import pytest
from config import defaults, validate
from design_goals import optimize, optimize_walking
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


def test_height_only_search_builds_nonzero_motion_and_preserves_constraints():
    c=defaults();c['payload_mass']=100.;c['payload_target']=100.
    source=deepcopy(c)
    report=optimize_walking(c,180.)
    chosen=report['chosen'];applied=report['applied']
    assert c==source and applied is not None
    assert build(applied)['height']*1000==pytest.approx(180,abs=.01)
    assert applied['payload_mass']==applied['payload_target']==100
    assert applied['max_mass']==c['max_mass']
    assert applied['stall']==c['stall'] and applied['derating']==c['derating']
    assert applied['reach']>0 and applied['lift']>=3
    assert chosen['period']>=4 and chosen['knee_margin']>=10
    assert report['motion']['complete'] and len(report['motion']['frames'])==81
    assert report['motion']['max_joint_speed_deg_s']<=45.0001
    frames=report['motion']['frames']
    assert frames[-1]['robot']['config']['pelvis_x']>frames[0]['robot']['config']['pelvis_x']


def test_height_only_failure_does_not_relax_mass_or_payload_requirement():
    c=defaults();c['max_mass']=100.
    before=deepcopy(c)
    assert optimize_walking(c,157.)['applied'] is None
    assert c==before
    c=defaults();c['payload_target']=200.
    assert optimize_walking(c,157.)['applied'] is None
