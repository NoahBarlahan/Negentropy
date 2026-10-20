from pathlib import Path
import pytest
from streamlit.testing.v1 import AppTest
from config import defaults,dumps

APP=Path(__file__).resolve().parents[1]/'app.py'


def test_cad_finalize_download_is_mm_and_snapshot_stays_frozen_after_edit():
    app=AppTest.from_file(str(APP),default_timeout=60).run()
    app.selectbox(key='length_unit').select('in').run()
    app.button(key='finalize_cad').click().run()
    assert not app.exception
    snapshot=app.session_state['cad_snapshot']
    assert '"ThighLength_L" = 65 mm' in snapshot['text']
    assert snapshot['config']==defaults()
    assert len(app.get('download_button'))>=3
    app.number_input(key='L_thigh__n').set_value(3.).run()
    assert not app.exception
    assert app.session_state['cad_snapshot']['config']['L_thigh']==65
    assert any('Downloads still contain the frozen snapshot' in w.value for w in app.warning)
    app.button(key='finalize_cad').click().run()
    assert app.session_state['cad_snapshot']['config']['L_thigh']==pytest.approx(76.2)
    assert '"ThighLength_L" = 76.2 mm' in app.session_state['cad_snapshot']['text']
    assert app.session_state['cad_snapshot']['revision']!=snapshot['revision']
    app.number_input(key='cad_dimension_ThighWidth').set_value(1.).run()
    app.number_input(key='cad_dimension_WaistHeight').set_value(2.).run()
    app.button(key='finalize_cad').click().run()
    assert not app.exception
    assert '"ThighWidth" = 25.4 mm' in app.session_state['cad_snapshot']['text']
    assert '"WaistHeight" = 50.8 mm' in app.session_state['cad_snapshot']['text']
    assert app.session_state['cfg']['platform_t']==4
    app.selectbox(key='length_unit').select('mm').run()
    assert not app.exception
    assert app.number_input(key='cad_dimension_ThighWidth').value==pytest.approx(25.4)
    assert app.number_input(key='cad_dimension_WaistHeight').value==pytest.approx(50.8)


def test_main_goals_optimize_in_inches_and_undo():
    app=AppTest.from_file(str(APP),default_timeout=60).run()
    original=dict(app.session_state['cfg'])
    app.selectbox(key='length_unit').select('in').run()
    assert app.number_input(key='goal_height').value==pytest.approx(157.1600407/25.4)
    app.number_input(key='goal_height').set_value(7.)
    app.number_input(key='goal_payload').set_value(100.)
    next(b for b in app.button if b.label=='Optimize my design').click().run()
    assert not app.exception
    report=app.session_state['goal_report']
    assert report['applied'] is not None
    assert app.session_state['cfg']['payload_mass']==100
    from model import build
    assert build(app.session_state['cfg'])['height']*1000==pytest.approx(177.8,abs=.01)
    app.button(key='undo_design').click().run()
    assert not app.exception
    assert app.session_state['cfg']==original


def test_sync_both_directions_load_reset_and_ik():
    app=AppTest.from_file(str(APP),default_timeout=60).run()
    assert not app.exception
    app.slider(key='L_thigh__s').set_value(75.).run()
    assert not app.exception
    assert app.number_input(key='L_thigh__n').value==75
    assert app.session_state['cfg']['L_thigh']==75
    app.number_input(key='L_thigh__n').set_value(82.).run()
    assert app.slider(key='L_thigh__s').value==82
    config=defaults();config['L_thigh']=72
    app.text_area(key='config_text').set_value(dumps(config))
    app.button(key='load_config').click().run()
    assert not app.exception
    assert app.slider(key='L_thigh__s').value==72
    assert app.number_input(key='L_thigh__n').value==72
    assert app.session_state['cfg']==config
    app.button(key='reset').click().run()
    assert app.slider(key='L_thigh__s').value==65
    app.button(key='L_apply_ik').click().run()
    assert not app.exception
    assert app.number_input(key='L_hip__n').value==pytest.approx(-20,abs=1e-5)
    assert app.slider(key='L_knee__s').value==pytest.approx(40,abs=1e-5)


def test_invalid_load_preserves_state_and_invalid_live_limits_recover():
    app=AppTest.from_file(str(APP),default_timeout=60).run()
    app.text_area(key='config_text').set_value('{}')
    app.button(key='load_config').click().run()
    assert not app.exception
    assert app.session_state['cfg']==defaults()
    assert app.session_state['load_error']
    app.number_input(key='L_hip_min__n').set_value(100.).run()
    assert not app.exception
    assert any('Invalid configuration' in e.value for e in app.error)
    app.number_input(key='L_hip_min__n').set_value(-75.).run()
    assert not app.exception
    app.checkbox(key='option_symmetry').uncheck().run()
    assert app.slider(key='R_thigh__s').value==65


def test_sweep_button_and_result_invalidation():
    app=AppTest.from_file(str(APP),default_timeout=60).run()
    app.number_input(key='sweep_count').set_value(2)
    app.button(key='run_sweep').click().run()
    assert not app.exception
    assert len(app.session_state['sweep_rows'])==4
    assert app.slider(key='sweep_count_slider').value==2
    app.number_input(key='thigh_lo').set_value(50).run()
    assert app.slider(key='thigh_range').value==(50,90)
    app.slider(key='shin_range').set_value((55,95)).run()
    assert app.number_input(key='shin_lo').value==55
    assert app.number_input(key='shin_hi').value==95
    app.slider(key='L_shin__s').set_value(70.).run()
    assert not app.exception
    assert 'sweep_rows' not in app.session_state


def test_inches_switch_preserves_design_and_converts_edits_load_placement_and_sweep():
    app=AppTest.from_file(str(APP),default_timeout=60).run()
    original=dict(app.session_state['cfg'])
    app.selectbox(key='length_unit').select('in').run()
    assert not app.exception
    assert app.session_state['cfg']==original
    assert app.slider(key='L_thigh__s').value==pytest.approx(65/25.4)
    assert '(in)' in app.slider(key='L_thigh__s').label
    assert any(m.value=='6.187 in' for m in app.metric)
    app.number_input(key='L_thigh__n').set_value(3.).run()
    assert not app.exception
    assert app.slider(key='L_thigh__s').value==3
    assert app.session_state['cfg']['L_thigh']==pytest.approx(76.2)
    app.slider(key='L_shin__s').set_value(2.5).run()
    assert app.number_input(key='L_shin__n').value==2.5
    assert app.session_state['cfg']['L_shin']==pytest.approx(63.5)
    app.selectbox(key='length_unit').select('mm').run()
    assert app.slider(key='L_thigh__s').value==pytest.approx(76.2)
    app.selectbox(key='length_unit').select('in').run()
    app.button(key='place_ground').click().run()
    assert not app.exception
    assert app.number_input(key='pelvis_z__n').value==pytest.approx(app.session_state['cfg']['pelvis_z']/25.4)
    app.text_area(key='config_text').set_value(dumps(defaults()))
    app.button(key='load_config').click().run()
    assert app.slider(key='L_thigh__s').value==pytest.approx(65/25.4)
    assert app.session_state['cfg']==defaults()
    app.number_input(key='thigh_lo').set_value(2.).run()
    assert app.session_state['sweep_mm_thigh'][0]==pytest.approx(50.8)
    app.number_input(key='sweep_count').set_value(2)
    app.button(key='run_sweep').click().run()
    assert not app.exception
    rows=app.session_state['sweep_rows']
    assert min(r['thigh (mm)'] for r in rows)==pytest.approx(50.8)
    app.selectbox(key='length_unit').select('mm').run()
    assert not app.exception
    assert app.session_state['sweep_rows']==rows
    assert app.number_input(key='thigh_lo').value==pytest.approx(50.8)
    app.selectbox(key='length_unit').select('in').run()
    app.button(key='reset').click().run()
    assert app.session_state['cfg']==defaults()
    assert app.number_input(key='L_thigh__n').value==pytest.approx(65/25.4)


def test_good_guess_button_apply_in_inches_report_is_snapshot_and_undo():
    app=AppTest.from_file(str(APP),default_timeout=60).run()
    app.selectbox(key='length_unit').select('in').run()
    app.number_input(key='sweep_count').set_value(2).run()
    original=dict(app.session_state['cfg'])
    app.button(key='calculate_guess').click().run()
    assert not app.exception
    report=app.session_state['guess_report']
    assert report['applied'] is not None
    assert app.session_state['cfg']==report['applied']
    assert app.slider(key='L_thigh__s').value==pytest.approx(report['chosen']['thigh (mm)']/25.4)
    app.number_input(key='max_mass__n').set_value(900.).run()
    assert report['applied']['max_mass']==800
    assert any('changed since this calculation' in w.value for w in app.warning)
    # Invalid live requirements remain editable even with a stored guess.
    app.number_input(key='min_height__n').set_value(9.).run()
    assert not app.exception
    app.number_input(key='min_height__n').set_value(4.).run()
    app.button(key='undo_guess').click().run()
    assert not app.exception
    from suggestions import CHANGED_KEYS
    assert all(app.session_state['cfg'][k]==original[k] for k in CHANGED_KEYS)
    assert app.session_state['cfg']['max_mass']==900
    app.number_input(key='max_mass__n').set_value(300.).run()
    before=dict(app.session_state['cfg'])
    app.button(key='calculate_guess').click().run()
    assert not app.exception
    assert app.session_state['guess_report']['applied'] is None
    assert app.session_state['cfg']==before


def test_rounded_controls_and_walking_animation_export_and_stale_inputs():
    app=AppTest.from_file(str(APP),default_timeout=60).run()
    assert not app.exception
    assert app.selectbox(key='option_foot_model').value=='Rounded nub'
    assert app.slider(key='L_nub_radius__s').value==6
    assert all(s.key!='L_foot_angle__s' for s in app.slider)
    app.selectbox(key='length_unit').select('in').run()
    app.button(key='build_preview').click().run()
    assert not app.exception
    assert app.session_state['motion_result']['complete']
    assert len(app.session_state['motion_result']['frames'])==81
    assert len(app.get('iframe'))==1
    assert 'Plotly.addFrames' in app.get('iframe')[0].proto.srcdoc
    assert app.session_state['cfg']==defaults()
    app.number_input(key='motion_period__n').set_value(2.).run()
    assert app.slider(key='motion_period__s').value==2
    assert any('old animation is hidden' in info.value for info in app.info)
    app.selectbox(key='option_foot_model').select('Flat sole').run()
    assert not app.exception
    assert app.slider(key='L_foot_angle__s').value==-20
    assert app.button(key='build_preview').disabled
