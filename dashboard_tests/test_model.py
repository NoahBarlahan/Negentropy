import json
import numpy as np
import pytest
from config import defaults, effective, FIELDS, MASS_ITEMS, dumps, loads
from model import (fk,ik,rotation,build,evaluate,contacts,stance,Mass,gravity_moment,
                   torque_rows,within_limits,support_polygon,tipping_margin,height_survey,
                   FEASIBLE,INFEASIBLE,NOT_EVALUATED,G)
from sweep import compare,CRITERIA
from conftest import flat_defaults as defaults


def test_schema_keys_unique_and_platform_geometry_independent_of_mass_com():
    assert len(FIELDS)==len({f.key for f in FIELDS})
    c=defaults();assert c['platform_z']==27 and c['platform_com_z']==0
    r=build(c)
    assert r['height']*1000 == pytest.approx(c['pelvis_z']+29,abs=.01)


@pytest.mark.parametrize('hip,knee,k,e',[
    (0,0,[0,0,-.1],[0,0,-.2]),
    (0,np.pi/2,[0,0,-.1],[.1,0,-.1]),
    (np.pi/2,-np.pi/2,[.1,0,0],[.1,0,-.1]),
    (0,np.pi,[0,0,-.1],[0,0,0]),
])
def test_known_fk(hip,knee,k,e):
    actual=fk(.1,.1,hip,knee)
    np.testing.assert_allclose(actual[0],k,atol=1e-12)
    np.testing.assert_allclose(actual[1],e,atol=1e-12)


def test_ik_roundtrips_both_branches():
    for hip in np.linspace(-1.2,1.2,7):
        for knee in np.linspace(-2,2,9):
            _,end=fk(.07,.055,hip,knee)
            sols=ik(.07,.055,end[0],end[2])
            assert len(sols)==2
            for q in sols:
                np.testing.assert_allclose(fk(.07,.055,*q)[1],end,atol=1e-11)


def test_unreachable_inner_outer_and_folded_singularity():
    assert ik(.1,.05,.3,0)==[]
    assert ik(.1,.05,0,-.01)==[]
    for q in ik(.1,.1,0,0):
        np.testing.assert_allclose(fk(.1,.1,*q)[1],np.zeros(3),atol=1e-12)
    c=defaults();c.update(L_hip=0,L_knee=0)
    assert evaluate(c)['singular']==['L','R']


def test_limits_and_unmodified_height_requirement():
    c=defaults();c['L_hip']=100
    assert not within_limits(c,'L',np.radians([100,40]))
    assert evaluate(c)['checks']['Joint limits']==INFEASIBLE
    c=defaults();before=build(c)['height'];c['min_height']=180
    assert build(c)['height']==before
    assert evaluate(c)['checks']['Current overall geometric height']==INFEASIBLE


def test_symmetry_mass_and_rotation():
    r=build(defaults())
    assert r['mass']==pytest.approx(.437)
    assert r['com'][1]==pytest.approx(0,abs=1e-14)
    assert len([m for m in r['masses'] if 'servo' in m.name])==4
    c=defaults();c.update(pelvis_x=30,pelvis_y=-10,pelvis_z=200,pitch=30,mode='Pelvis fixed')
    transformed=build(c)
    c0=defaults();c0.update(pelvis_z=0,mode='Pelvis fixed')
    expected=np.array([.03,-.01,.2])+rotation(np.pi/6)@build(c0)['com']
    np.testing.assert_allclose(transformed['com'],expected)


def test_manual_mass_is_not_scaled_with_length():
    c=defaults();a=build(c);c['L_thigh']=100;c['L_shin']=100;b=build(c)
    assert a['mass']==b['mass']
    assert c['L_thigh_print_com_z']==-32.5


def test_independent_gravity_moment_and_carrier_exclusion():
    m=Mass('test','shin',.2,np.array([.04,0,-.1]))
    assert gravity_moment([m],np.zeros(3))==pytest.approx(.0784532)
    c=defaults();c['mode']='Pelvis fixed';a=torque_rows(build(c))
    c['L_knee_servo_mass']=500;b=torque_rows(build(c))
    assert b[1]['static demand (N·m)']==a[1]['static demand (N·m)']
    assert b[0]['static demand (N·m)']>a[0]['static demand (N·m)']


def test_default_stance_equilibrium_and_subtree_free_bodies():
    e=evaluate(defaults());s=e['stance'];r=e['robot'];ct=e['contact']
    assert s['status']==FEASIBLE
    assert all(s['forces']>=0)
    np.testing.assert_allclose(s['force_residual'],0,atol=1e-10)
    np.testing.assert_allclose(s['moment_residual'],0,atol=1e-10)
    assert sum(s['forces'])==pytest.approx(r['mass']*G)
    assert sum(f for f,owner in zip(s['forces'],ct['owners']) if owner=='L')==pytest.approx(r['mass']*G/2)
    # Independently sum each distal free body's external moments and actuator.
    for row in s['rows']:
        assert row['signed actuator (N·m)']-row['gravity moment (N·m)']+row['ground moment (N·m)']==pytest.approx(0,abs=1e-12)


def test_payload_changes_stance_loading_without_distal_mass_double_count():
    c=defaults();a=evaluate(c);c.update(payload_mass=300,payload_com_x=15)
    b=evaluate(c)
    assert b['stance']['status']==FEASIBLE
    assert b['rows'][1]['static demand (N·m)']>a['rows'][1]['static demand (N·m)']
    assert a['rows'][0]['gravity moment (N·m)']==b['rows'][0]['gravity moment (N·m)']
    assert sum(b['stance']['forces'])-sum(a['stance']['forces'])==pytest.approx(.3*G)


def test_contact_failures_and_pelvis_fixed_not_equilibrium():
    c=defaults();c['pelvis_z']-=5
    e=evaluate(c);assert e['stance']['status']==INFEASIBLE and not e['rows']
    c=defaults();c['pelvis_z']+=5
    assert evaluate(c)['stance']['status']==INFEASIBLE
    c=defaults();c['contacts']='Left'
    assert 'unselected' in evaluate(c)['stance']['reason']
    c=defaults();c.update(mode='Pelvis fixed',pelvis_z=250)
    e=evaluate(c);assert e['stance']['status']==NOT_EVALUATED and len(e['rows'])==4


def test_tilted_feet_edge_contact_not_rectangle_and_degenerate_unavailable():
    c=defaults();c['L_foot_angle']=-10
    r=build(c);c['pelvis_z']-=min(l['sole'][:,2].min() for l in r['legs'].values())*1000
    e=evaluate(c)
    assert len(e['contact']['points'])==4 # two lowest edges
    assert all('edge' in detail for detail in e['contact']['details'])
    assert e['stance']['status']==NOT_EVALUATED
    assert not e['rows']


def test_opposing_tilted_edges_can_form_real_area_support():
    c=defaults();c.update(symmetry=False,L_foot_angle=-10,R_foot_angle=-30)
    r=build(c);c['pelvis_z']-=min(l['sole'][:,2].min() for l in r['legs'].values())*1000
    e=evaluate(c)
    assert len(e['contact']['points'])==4
    assert len(e['contact']['polygon'])==4
    assert e['stance']['status']==FEASIBLE
    np.testing.assert_allclose(e['stance']['moment_residual'],0,atol=1e-10)


def test_single_support_with_raised_leg_and_lateral_payload():
    c=defaults();c.update(symmetry=False,contacts='Left',R_hip=0,R_knee=90,
                          payload_mass=300,payload_com_y=50)
    e=evaluate(c)
    assert not e['contact']['errors']
    assert e['contact']['owners']==['L']*4
    assert e['stance']['status']==FEASIBLE
    assert e['stance']['forces'].sum()==pytest.approx(e['robot']['mass']*G)
    assert abs(e['stance']['structural_roll_moments']['L'])>0
    np.testing.assert_allclose(e['stance']['moment_residual'],0,atol=1e-10)


def test_suspended_torque_matches_potential_energy_derivative():
    c=defaults();c.update(mode='Pelvis fixed',symmetry=False)
    base=build(c);h=1e-6
    for index,joint in enumerate(('hip','knee')):
        plus=dict(c);minus=dict(c)
        plus[f'L_{joint}']+=np.degrees(h)
        minus[f'L_{joint}']-=np.degrees(h)
        up=sum(m.kg*G*m.position[2] for m in build(plus)['masses'])
        down=sum(m.kg*G*m.position[2] for m in build(minus)['masses'])
        assert torque_rows(base)[index]['signed actuator (N·m)']==pytest.approx((up-down)/(2*h),abs=1e-9)


def test_support_polygon_margin_and_tipping():
    p=support_polygon([[-1,-1],[-1,1],[1,-1],[1,1]])
    assert tipping_margin(p,[0,0])==pytest.approx(1)
    assert tipping_margin(p,[2,0])==pytest.approx(-1)
    assert support_polygon([[0,0],[1,0],[2,0]]).size==0
    c=defaults();c.update(payload_mass=1500,payload_com_x=200)
    e=evaluate(c);assert e['stance']['status']==INFEASIBLE and not e['rows']


def test_height_survey_flat_foot_constraint():
    survey=height_survey(defaults());assert survey['count']>0
    assert 100<survey['max']<250
    c=defaults();c['symmetry']=False
    assert height_survey(c) is None


def test_sweep_protocol_fixed_masses_packaging_ranking():
    c=defaults();rows=compare(c,[25,65],[65],CRITERIA)
    assert rows[0]['Packaging']==INFEASIBLE
    assert rows[0]['mass (g)']==rows[1]['mass (g)']
    assert rows[1]['Stance']==FEASIBLE
    assert rows[1]['selected checks']==FEASIBLE
    assert rows[1]['rank']==1
    assert c==defaults() # exploration never mutates UI config
    c['L_foot_mass']=80
    assert all(row['mass (g)']==pytest.approx(553) for row in compare(c,[65],[65],CRITERIA))


def test_json_roundtrip_defaults_and_independent_values():
    c=defaults();c.update(L_thigh=82,R_thigh=93,symmetry=False)
    assert loads(dumps(c))==c
    assert effective(dict(c,symmetry=True))['R_thigh']==82


@pytest.mark.parametrize('key,value',[('L_thigh',0),('battery_mass',-1),('pitch',float('nan')),('stall',True),('version',3),('symmetry',1),('contacts','Floor')])
def test_invalid_configuration(key,value):
    c=defaults();c[key]=value
    with pytest.raises(ValueError): dumps(c)


def test_invalid_schema_json_and_all_zero_masses():
    for text in ['{}','[]','not JSON','{"version":1,"version":1}']:
        with pytest.raises(ValueError): loads(text)
    c=defaults();c['L_hip_min']=100
    with pytest.raises(ValueError): dumps(c)
    c=defaults()
    for m in MASS_ITEMS: c[m[0]+'_mass']=0
    with pytest.raises(ValueError): build(c)
    with pytest.raises(ValueError): fk(-.1,.1,0,0)
    with pytest.raises(ValueError): ik(.1,.1,float('inf'),0)
