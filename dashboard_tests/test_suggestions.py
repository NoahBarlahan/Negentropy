from copy import deepcopy
import numpy as np
import pytest
from config import defaults
from model import evaluate,FEASIBLE
from suggestions import suggest,undo,PRIORITIES,CHANGED_KEYS
from conftest import flat_defaults as defaults


def test_guess_applies_only_documented_fields_and_passes_mechanics():
    c=defaults();before=deepcopy(c)
    report=suggest(c,count=3)
    assert c==before
    assert report['chosen'] is not None
    assert report['target_mm']==pytest.approx(152.4)
    assert report['tested']==16 # three grid points plus current length per axis
    assert report['feasible']>0
    proposed=report['applied']
    assert evaluate(proposed)['status']==FEASIBLE
    assert all(c[k]==proposed[k] for k in c if k not in CHANGED_KEYS)
    assert proposed['pelvis_z']==report['chosen']['hip height (mm)']
    assert proposed['L_thigh']==proposed['R_thigh']
    assert proposed['L_shin']==proposed['R_shin']
    assert np.isfinite(report['chosen']['peak screening utilization'])
    assert report['shortlist'][0] is report['chosen']


@pytest.mark.parametrize('priority',PRIORITIES)
def test_priority_order_is_explainable_and_sorted(priority):
    report=suggest(defaults(),count=3,priority=priority)
    order=report['order']
    scores=[tuple(round(float(row[key]),10) for key in order) for row in report['shortlist']]
    assert scores==sorted(scores)


def test_restrictions_are_not_relaxed_when_no_guess_passes():
    c=defaults();c['max_mass']=300
    report=suggest(c,count=2)
    assert report['chosen'] is None and report['applied'] is None
    assert report['failures']['Mass']==report['tested']
    c=defaults();c['L_foot_angle']=-10
    report=suggest(c,count=2)
    assert report['applied'] is None
    assert report['failures']['Stance']==report['tested']


def test_undo_preserves_later_requirement_edits_and_rejects_geometry_edits():
    report=suggest(defaults(),count=2)
    applied=dict(report['applied']);applied['max_mass']=900
    restored=undo(applied,report)
    assert restored['max_mass']==900
    assert all(restored[k]==report['source'][k] for k in CHANGED_KEYS)
    applied['L_thigh']+=1
    with pytest.raises(ValueError,match='since been edited'): undo(applied,report)


def test_search_bounds_and_current_candidate_preservation():
    c=defaults()
    report=suggest(c,(65,65),(65,65),count=2,priority=PRIORITIES[2])
    assert report['tested']==1
    assert report['chosen']['length change (mm)']==0
    for bounds in [(90,45),(0,20),(5,float('nan'))]:
        with pytest.raises(ValueError): suggest(c,bounds,count=2)
    with pytest.raises(ValueError): suggest(c,count=1)
