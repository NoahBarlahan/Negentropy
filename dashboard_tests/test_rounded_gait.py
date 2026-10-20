from copy import deepcopy
import json
import numpy as np
import pytest
from config import defaults,MASS_ITEMS,loads,dumps
from model import build,evaluate,reach_samples,rotation,FEASIBLE,INFEASIBLE,NOT_EVALUATED,within_limits
from gait import preview,animation,center_ik,sphere_center
from suggestions import suggest
from plots import view3d
from units import length_figure


def test_rounded_radius_contact_and_orientation_invariance():
    c=defaults();a=build(c)
    c['L_foot_angle']=130;c['L_foot_l']=160;c['L_foot_w']=100
    b=build(c)
    np.testing.assert_allclose(a['vertices'],b['vertices'])
    np.testing.assert_allclose(a['com'],b['com'])
    for l in b['legs'].values():
        assert len(l['sole'])==1 and l['tilt'] is None
        assert l['sole'][0,2]==pytest.approx(l['foot'][2]-l['radius'])
    e=evaluate(c)
    assert len(e['contact']['points'])==2 and not len(e['contact']['polygon'])
    assert e['contact']['margin'] is None
    assert e['contact']['support_distance']>0
    assert e['stance']['status']==INFEASIBLE and not e['rows']


def test_rounded_point_equilibrium_has_correct_reactions_but_no_stability_claim():
    c=defaults()
    for key,*_ in MASS_ITEMS:c[key+'_mass']=0
    c['platform_mass']=1000;c['max_mass']=1500
    e=evaluate(c)
    assert e['stance']['status']==FEASIBLE
    np.testing.assert_allclose(e['stance']['forces'],[4.903325,4.903325],atol=1e-9)
    np.testing.assert_allclose(e['stance']['moment_residual'],0,atol=1e-10)
    assert e['checks']['Dynamic walking balance']==NOT_EVALUATED
    assert e['status']==NOT_EVALUATED


def test_v1_import_preserves_flat_geometry_and_rejects_unknown_keys():
    from conftest import flat_defaults
    c=flat_defaults();c.pop('foot_model');c.pop('L_nub_radius');c.pop('R_nub_radius');c['version']=1
    migrated=loads(json.dumps(c))
    assert migrated['foot_model']=='Flat sole' and migrated['version']==2
    assert migrated['L_foot_com_z']==-3
    assert loads(dumps(migrated))==migrated
    with pytest.raises(ValueError):loads(json.dumps(dict(c,surprise=True)))


def test_rounded_suggestion_uses_qualified_gravity_screen_not_static_balance():
    r=suggest(defaults(),count=3)
    assert r['chosen'] is not None
    assert 'Stance' not in r['required']
    assert 'dynamic balance not evaluated' in r['qualification']
    assert 'Externally supported' in r['chosen']['torque basis']
    assert r['chosen']['Contact geometry']==FEASIBLE


def test_trajectory_clearance_continuity_rolling_limits_and_no_mutation():
    c=defaults();original=deepcopy(c)
    result=preview(c)
    assert c==original
    assert result['complete'] and result['failure'] is None
    assert len(result['frames'])==81
    angles=np.array([f['angles_deg'] for f in result['frames']])
    assert np.max(np.abs(np.diff(angles,axis=0)))<10
    for frame in result['frames']:
        assert not frame['contact']['errors']
        assert max(frame['rolling_error'].values(),default=0)<1e-7
        for side,leg in frame['robot']['legs'].items():
            assert within_limits(c,side,leg['angles'])
            assert leg['sole'][0,2]>=-1e-7
    last=result['frames'][-1]['robot'];first=result['frames'][0]['robot']
    assert last['frames']['pelvis'][0][0]-first['frames']['pelvis'][0][0]==pytest.approx(.08)
    assert max(f['robot']['legs']['L']['sole'][0,2] for f in result['frames'])==pytest.approx(.015)
    assert any(f['phases']['L']=='swing' for f in result['frames'])
    assert any(f['phases']['R']=='swing' for f in result['frames'])


def test_timing_changes_speed_not_geometry():
    a=preview(defaults(),cycles=1,samples=20,period=1.6)
    b=preview(defaults(),cycles=1,samples=20,period=3.2)
    np.testing.assert_allclose([f['angles_deg'] for f in a['frames']],[f['angles_deg'] for f in b['frames']],atol=1e-8)
    assert a['max_joint_speed_deg_s']==pytest.approx(2*b['max_joint_speed_deg_s'])


def test_center_ik_includes_shin_offsets():
    c=defaults();c.update(L_foot_ox=5,L_foot_oz=-2)
    r=build(c);hip=r['legs']['L']['hip'];q=r['legs']['L']['angles']
    center=sphere_center(c,'L',hip,q)
    solved=center_ik(c,'L',hip,center)
    assert solved is not None
    np.testing.assert_allclose(sphere_center(c,'L',hip,solved),center,atol=1e-10)
    for sample in reach_samples(c):
        if sample['side']=='L':
            assert sample['target reference']=='nub center'
            for q in sample['solutions']:
                actual=rotation(np.radians(c['pitch'])).T@(sphere_center(c,'L',hip,q)-hip)
                np.testing.assert_allclose(actual[[0,2]],[sample['x (mm)']/1000,sample['z (mm)']/1000],atol=1e-10)


def test_unreachable_motion_stops_instead_of_clamping():
    result=preview(defaults(),lift_mm=120,cycles=1,samples=20)
    assert not result['complete'] and result['failure']
    assert len(result['frames'])<21
    for frame in result['frames']:assert not frame['contact']['errors']
    with pytest.raises(ValueError):preview(dict(defaults(),foot_model='Flat sole'))
    with pytest.raises(ValueError):preview(defaults(),period=0)


def test_animation_frames_coordinates_and_rounded_static_plot():
    result=preview(defaults(),cycles=1,samples=16)
    a=animation(result,'mm');b=animation(result,'in')
    assert len(a.frames)==17
    assert len(a.frames[0].data)==7
    assert 'Plotly.addFrames' in a.to_html(include_plotlyjs=False,auto_play=False)
    np.testing.assert_allclose(np.array(b.frames[5].data[0].x)*25.4,a.frames[5].data[0].x)
    figure=length_figure(view3d(evaluate(defaults())),'in')
    assert figure.layout.scene.xaxis.title.text=='Forward x (in)'
    assert sum(t.name=='Rounded nub' for t in figure.data)==2
