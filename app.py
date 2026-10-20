"""Run: python -m streamlit run app.py"""
from pathlib import Path
import csv
import io
import numpy as np
import streamlit as st
from streamlit.components.v1 import html as component_html
import plotly.express as px
from config import FIELDS, MASS_ITEMS, defaults, effective, dumps, loads, validate
from model import (build, evaluate, ik, within_limits, reach_samples, height_survey,
                   packaging, distal_masses, G, FEASIBLE, INFEASIBLE, NOT_EVALUATED)
from plots import view3d, orthographic
from sweep import compare, CRITERIA, design_criteria
from gait import preview, animation
from units import mm_per_unit, length_label, format_length, length_rows, length_figure
from suggestions import suggest, undo, PRIORITIES, CHANGED_KEYS
from design_goals import optimize
from cad_export import finalize, bundle, EQUATIONS_FILE, HELP_URL, CAD_FIELDS, EXPORT_SCHEMA

st.set_page_config(page_title="Biped Studio · MG996R", page_icon="🤖", layout="wide")
st.markdown("""<style>
.block-container {padding-top:2rem;max-width:1600px}
[data-testid="stMetric"] {background:rgba(100,116,139,.10);padding:16px;border-radius:12px}
.stCaption {line-height:1.5}
</style>""", unsafe_allow_html=True)


if "length_unit" not in st.session_state:
    st.session_state.length_unit="mm"
FIELD_BY_KEY={f.key:f for f in FIELDS}


def field_factor(f):
    return mm_per_unit(st.session_state.length_unit) if "(mm)" in f.label else 1.


def sync_fields(config):
    for f in FIELDS:
        value=float(config[f.key])/field_factor(f)
        st.session_state[f.key+"__s"] = value
        st.session_state[f.key+"__n"] = value


def seed(config):
    st.session_state.cfg = config
    sync_fields(config)
    for key in ("symmetry", "mode", "contacts", "working_basis", "provenance", "foot_model"):
        st.session_state["option_"+key] = config[key]
    st.session_state.pop("sweep_rows", None)
    scale=mm_per_unit(st.session_state.length_unit)
    st.session_state.goal_height=min(450.,max(80.,float(build(config)["height"]*1000)))/scale
    st.session_state.goal_payload=float(config["payload_mass"])
    st.session_state.goal_step=float(config["reach"])/scale
    st.session_state.goal_clearance=float(config["lift"])/scale
    st.session_state.goal_mass_limit=float(config["max_mass"])
    st.session_state.goal_unit=st.session_state.length_unit


if "cfg" not in st.session_state:
    seed(defaults())
elif st.session_state.cfg.get('version')==1 or "goal_height" not in st.session_state:
    seed(validate(st.session_state.cfg))


def commit(key, kind):
    val = st.session_state[key+"__"+kind]
    f=FIELD_BY_KEY[key]
    st.session_state.cfg[key] = min(f.high,max(f.low,val*field_factor(f)))
    st.session_state[key+"__"+("n" if kind=="s" else "s")] = val
    st.session_state.pop("sweep_rows",None)


def change_length_unit():
    old=mm_per_unit(st.session_state.get("goal_unit","mm"))
    new=mm_per_unit(st.session_state.length_unit)
    for key in ("goal_height","goal_step","goal_clearance"):
        if key in st.session_state: st.session_state[key]*=old/new
    for name in CAD_FIELDS:
        key='cad_dimension_'+name
        if st.session_state.get(key) is not None: st.session_state[key]*=old/new
    st.session_state.goal_unit=st.session_state.length_unit
    sync_fields(st.session_state.cfg)
    for name in ("thigh","shin"):
        canonical=st.session_state.get("sweep_mm_"+name,st.session_state.get(name+"_range",(45.,90.)))
        st.session_state["sweep_mm_"+name]=canonical
        shown=tuple(float(v)/mm_per_unit(st.session_state.length_unit) for v in canonical)
        st.session_state[name+"_range"]=shown
        st.session_state[name+"_lo"],st.session_state[name+"_hi"]=shown


def option(key):
    st.session_state.cfg[key] = st.session_state["option_"+key]
    st.session_state.pop("sweep_rows",None)


def reset():
    seed(defaults())
    st.session_state.pop("goal_report",None)
    st.session_state.pop("goal_error",None)
    st.session_state.pop("load_error",None)


def load_config():
    try:
        file=st.session_state.get("config_file")
        text=file.getvalue().decode("utf-8-sig") if file is not None else st.session_state.get("config_text", "")
        seed(loads(text))
        st.session_state.pop("load_error",None)
        st.session_state.load_ok=True
    except (ValueError, UnicodeError) as exc:
        st.session_state.load_error=str(exc)


def apply_ik(side):
    c=effective(st.session_state.cfg)
    sols=ik(c[f"{side}_thigh"]/1000,c[f"{side}_shin"]/1000,c[f"{side}_target_x"]/1000,c[f"{side}_target_z"]/1000)
    branch=st.session_state.get(side+"_branch","Positive knee")
    if not sols:
        st.session_state.ik_message=(False, "Unreachable target: outside the two-link annulus. Inputs were preserved.")
        return
    q=sols[0 if branch=="Positive knee" else 1]
    if not within_limits(c,side,q):
        st.session_state.ik_message=(False,"Selected branch violates a joint limit. Try the other branch or change the explicit limits. Inputs were preserved.")
        return
    for joint,val in zip(("hip","knee"),np.degrees(q)):
        key=f"{side}_{joint}"
        st.session_state.cfg[key]=float(val)
        st.session_state[key+"__s"]=float(val)
        st.session_state[key+"__n"]=float(val)
    st.session_state.pop("sweep_rows",None)
    st.session_state.ik_message=(True,"IK applied to joint angles only. Verify the resulting sole tilt and contact geometry.")


def place_on_ground():
    try:
        robot=build(st.session_state.cfg)
        selected={"Both":["L","R"],"Left":["L"],"Right":["R"],"None":[]}[robot["config"]["contacts"]]
        if not selected:
            raise ValueError("Select a contact before placing the robot.")
        lowest=min(robot["legs"][s]["sole"][:,2].min() for s in selected)
        z=st.session_state.cfg["pelvis_z"]-float(lowest)*1000
        if not -100 <= z <= 500:
            raise ValueError("Required pelvis height is outside the editable range.")
        st.session_state.cfg["pelvis_z"]=z
        st.session_state["pelvis_z__s"]=z/mm_per_unit(st.session_state.length_unit)
        st.session_state["pelvis_z__n"]=z/mm_per_unit(st.session_state.length_unit)
        st.session_state.pop("sweep_rows",None)
        st.session_state.placement_message="Translated pelvis z only; the lowest selected sole point is now on ground. Other contacts may remain inconsistent."
    except ValueError as exc:
        st.session_state.placement_message=str(exc)


def calculate_guess():
    try:
        report=suggest(st.session_state.cfg,
            st.session_state.get("sweep_mm_thigh",(45.,90.)),
            st.session_state.get("sweep_mm_shin",(45.,90.)),
            int(st.session_state.get("sweep_count",6)),
            st.session_state.get("guess_priority",PRIORITIES[0]))
        if report["applied"] is not None:
            seed(dict(report["applied"]))
        st.session_state.guess_report=report
        st.session_state.pop("guess_error",None)
    except ValueError as exc:
        st.session_state.guess_error=str(exc)


def undo_guess():
    try:
        report=st.session_state.guess_report
        seed(undo(st.session_state.cfg,report))
        st.session_state.guess_report={**report,"undone":True}
        st.session_state.pop("guess_error",None)
    except ValueError as exc:
        st.session_state.guess_error=str(exc)


def optimize_goals():
    try:
        scale=mm_per_unit(st.session_state.length_unit)
        report=optimize(st.session_state.cfg,
            st.session_state.goal_height*scale,st.session_state.goal_payload,
            st.session_state.goal_step*scale,st.session_state.goal_clearance*scale,
            st.session_state.goal_mass_limit)
        if report['applied'] is not None:
            seed(dict(report['applied']))
            st.session_state.pop('motion_result',None)
        st.session_state.goal_report=report
        st.session_state.pop('goal_error',None)
    except ValueError as exc:
        st.session_state.goal_error=str(exc)


def undo_design():
    report=st.session_state.goal_report
    if st.session_state.cfg!=report['applied']:
        st.session_state.goal_error='The design has been edited since optimization. Optimize again before undoing.'
        return
    seed(dict(report['source']))
    st.session_state.pop('goal_report',None)


def cad_dimension_values():
    scale=mm_per_unit(st.session_state.length_unit)
    return {name:float(st.session_state['cad_dimension_'+name])*scale for name in CAD_FIELDS
            if st.session_state.get('cad_dimension_'+name) is not None}


def finalize_cad():
    try:
        st.session_state.cad_snapshot=finalize(st.session_state.cfg,cad_dimension_values())
        st.session_state.pop('cad_error',None)
    except ValueError as exc:
        st.session_state.cad_error=str(exc)


with st.sidebar:
    st.markdown("## Biped Studio")
    st.caption("4 × MG996R · two pitch joints per leg")
    st.selectbox("Length units",["mm","in"],format_func=lambda u:"Millimeters (mm)" if u=="mm" else "Inches (in)",key="length_unit",on_change=change_length_unit)
    st.caption("Change your goals below, then optimize the design.")
    with st.expander("Advanced settings"):
        st.selectbox('Foot contact model',['Rounded nub','Flat sole'],key='option_foot_model',on_change=option,args=('foot_model',))
        st.selectbox("Good-guess priority",PRIORITIES,key="guess_priority")
        st.button("Calculate & apply a good guess",on_click=calculate_guess,key="calculate_guess",width="stretch")
        st.caption("Adjusts both legs’ thigh/shin lengths and pelvis height; switches to grounded mode. Uses the search ranges in Dimension comparison. All requirements stay fixed.")
        if st.session_state.get("guess_error"):
            st.error(st.session_state.guess_error)
        if st.session_state.get("guess_report",{}).get("applied") and not st.session_state.guess_report.get("undone"):
            st.button("Undo last good guess",on_click=undo_guess,key="undo_guess",width="stretch")
        st.toggle("Show explanations", value=False, key="explanations")
        st.checkbox("Left/right symmetry", key="option_symmetry",on_change=option,args=("symmetry",))
        st.selectbox("Support mode",["Grounded pose","Pelvis fixed"],key="option_mode",on_change=option,args=("mode",))
        st.selectbox("Selected ground contacts",["Both","Left","Right","None"],key="option_contacts",on_change=option,args=("contacts",))
        st.button("Place lowest selected sole on ground",on_click=place_on_ground,key="place_ground",width="stretch")
        st.caption("Explicit placement changes only pelvis z. Grounded mode never levels a foot or moves a joint for you.")
        if st.session_state.get("placement_message"):
            st.info(st.session_state.placement_message)
    st.button("Reset defaults",on_click=reset,key="reset",width="stretch")
    with st.expander("Save / load configuration"):
        st.file_uploader("JSON file",type="json",key="config_file")
        st.text_area("Or paste configuration JSON",key="config_text",height=100)
        st.button("Apply configuration",on_click=load_config,key="load_config")
        if st.session_state.get("load_error"):
            st.error(st.session_state.load_error)
        if st.session_state.pop("load_ok",False):
            st.success("Configuration loaded and controls synchronized.")
        try:
            exported=dumps(st.session_state.cfg)
            st.download_button("Save configuration JSON",exported,"biped_config.json","application/json",key="save_config")
        except ValueError as exc:
            st.caption(f"Correct the inputs before saving: {exc}")

st.caption("BIPED STUDIO / 4 × MG996R")
st.title("What should your robot do?")
st.write("Set your goals. Let the dashboard choose the leg lengths and starting pose.")
unit=st.session_state.length_unit
scale=mm_per_unit(unit)
with st.form('design_goals'):
    a,b=st.columns(2)
    with a:
        st.number_input(f"Robot height ({unit})",min_value=80./scale,max_value=450./scale,
                        step=1./scale,key='goal_height',help='Ground to the top of the modeled robot, excluding the payload envelope.')
        st.number_input('Payload to carry (g)',min_value=0.,max_value=2000.,step=10.,key='goal_payload')
    with b:
        st.number_input(f"Step length ({unit})",min_value=0.,max_value=160./scale,
                        step=1./scale,key='goal_step',help='Forward travel per alternating footfall; a full gait cycle travels twice this distance.')
        st.number_input(f"Foot clearance ({unit})",min_value=0.,max_value=120./scale,
                        step=1./scale,key='goal_clearance')
    with st.expander('Weight budget'):
        st.number_input('Maximum total mass including payload (g)',min_value=10.,max_value=5000.,step=10.,key='goal_mass_limit')
    st.form_submit_button('Optimize my design',on_click=optimize_goals,type='primary',width='stretch')
st.caption('Automatically chooses symmetric thigh/shin lengths, hip/knee angles and grounded hip height. Hardware, body layout and component masses use the assumptions in Advanced inputs.')
if st.session_state.get('goal_error'): st.error(st.session_state.goal_error)
report=st.session_state.get('goal_report')
if report:
    if report['applied'] is None:
        st.warning('No design passed these goals. Your existing model is unchanged. Try a lower payload, shorter step, or a different height.')
        with st.expander('Why no design passed'):
            st.dataframe([{'Constraint':k,'Candidates rejected':v} for k,v in report['failures'].items()],hide_index=True)
    else:
        best=report['chosen']
        st.success(f"Design calculated · {report['passing']} of {report['tested']} candidates passed the screening checks.")
        cols=st.columns(3)
        cols[0].metric('Thigh length',format_length(best['config']['L_thigh'],unit))
        cols[1].metric('Shin length',format_length(best['config']['L_shin'],unit))
        cols[2].metric('Modeled mass',f"{best['mass']:.0f} g")
        if st.session_state.cfg!=report['applied']:
            st.info('Advanced inputs have changed since optimization. Optimize again to update this result.')
        st.button('Undo optimized design',on_click=undo_design,key='undo_design')
    with st.expander('How the dashboard chooses your design'):
        st.write('1. Add your payload to the mass model. Try 18 combinations of thigh/shin proportion (45:55, 50:50, 55:45) and knee bend (25° to 75°). Solve link lengths to match your overall height within 1 mm and place the feet on the ground.')
        st.write('2. Check the height, weight budget, joint limits, approximate servo packaging and requested reach. Test a full walking cycle at 17 time samples for reach and ground clearance.')
        st.write('3. Estimate joint torque from gravity plus a vertical load equal to the entire robot weight on each stance leg. Adding the absolute moments gives a conservative screening load case. Choose the passing candidate with the lowest peak demand against the configured servo allowance.')
        st.write('The knee servo is placed at the knee; printed-link centers of mass are estimated at mid-link. Component masses, foot radius, platform dimensions and measured offsets stay fixed. Update these assumptions from your CAD in Advanced inputs. The search finds the best of 18 candidates, not a global optimum.')
        st.caption('The load case does not solve dynamic equilibrium, acceleration, traction, lateral balance or servo heating. A passing design and the walking animation do not establish that the robot can walk unsupported.')


c=st.session_state.cfg
unit=st.session_state.length_unit
factor=mm_per_unit(unit)
def dimension(mm):
    return format_length(mm,unit)
def display_rows(rows):
    return length_rows(rows,unit)
try:
    result=evaluate(c)
except ValueError as exc:
    st.error(f"Invalid configuration: {exc}")
    result=None

st.divider()
overview,motion,cad,inputs,loading,comparison,guide=st.tabs(["Robot & balance","Walking preview","CAD / SolidWorks","Advanced inputs","Static loading","Dimension comparison","Model & walkthrough"])

with cad:
    st.subheader('Finalize dimensions for SolidWorks')
    st.write('Export just the dimensions you will use to build the waist, legs and feet.')
    st.caption('Exports the current model. After editing goals, press Optimize my design first; manual dimension changes in Advanced inputs are included immediately.')
    if result:
        current=effective(c)
        cols=st.columns(3)
        cols[0].metric('Current overall height',dimension(result['robot']['height']*1000))
        cols[1].metric('Left thigh / shin',dimension(current['L_thigh'])+' / '+dimension(current['L_shin']))
        cols[2].metric('Hip spacing',dimension(current['spacing']))
    with st.expander('Waist & printed link dimensions'):
        st.caption('Optional CAD dimensions. Blank waist values use the current platform dimensions. Blank link widths/thicknesses are omitted. These values do not change the dashboard geometry or mass calculations.')
        left,right=st.columns(2)
        for index,(name,label) in enumerate(CAD_FIELDS.items()):
            with (left if index%2==0 else right):
                st.number_input(label+f' ({unit})',min_value=.1/factor,max_value=500./factor,
                                value=None,step=1./factor,key='cad_dimension_'+name,
                                placeholder='Optional')
        st.caption('Thigh and shin width/thickness apply to both legs. Waist height starts as platform thickness; enter your body height if you are designing an enclosure. Link cross-sections need your CAD choices; the optimizer does not size them for strength.')
    st.button('Finalize current dimensions',on_click=finalize_cad,key='finalize_cad',type='primary')
    if st.session_state.get('cad_error'): st.error(st.session_state.cad_error)
    snapshot=st.session_state.get('cad_snapshot')
    if snapshot and snapshot.get('export_schema')!=EXPORT_SCHEMA:
        st.info('The export has been simplified. Finalize again to replace the previous parameter list.')
        snapshot=None
    if snapshot:
        st.success(f"CAD snapshot {snapshot['revision']} · {snapshot['created']}")
        if c!=snapshot['config'] or cad_dimension_values()!=snapshot['dimensions']:
            st.warning('The current model has changed. Downloads still contain the frozen snapshot above. Finalize again to export the new dimensions.')
        a,b=st.columns(2)
        with a:
            st.download_button('Download SolidWorks equations (.txt)',snapshot['text'],EQUATIONS_FILE,
                               'text/plain',key='download_cad_txt',width='stretch')
        with b:
            st.download_button('Download CAD package (.zip)',bundle(snapshot),
                               f"negentropy_cad_{snapshot['revision']}.zip",'application/zip',
                               key='download_cad_bundle',width='stretch')
        st.caption('The ZIP includes the equations, setup guide, parameter map, dashboard configuration and CAD dimensions. All equations use explicit mm, including when you display inches. Pose, offsets and servo dimensions are excluded.')
        with st.expander('Preview the Notepad file'):
            st.code(snapshot['text'],language=None)
        with st.expander('Parameter names and physical references'):
            st.dataframe([{k:v for k,v in row.items() if k!='equation'} for row in snapshot['parameters']],hide_index=True,width='stretch')
    st.markdown('''**Use it in SolidWorks**

1. Save the text file in a permanent folder beside your CAD project.
2. Open **Tools → Equations → Import**, select the file and enable **Link to File** if available.
3. Drive sketch dimensions with `="ThighLength_L"`, `="ShinLength_L"`, `="WaistWidth"` or `="HipSpacing"`. Use `="FootRadius_L"` for a nub radius, or `=2*"FootRadius_L"` for its diameter.
4. Import/link the same file in each part that uses these values. To update later, replace the file at the same path and rebuild.
''')
    st.link_button('SolidWorks import and linking instructions',HELP_URL)
    st.info('Thigh and shin lengths are measured between joint references. With zero attachment offset, the shin ends at the rounded foot center. The file supplies global variables; you build and connect the sketches, features and mates.')
    with st.expander('Measurements still needed from your hardware'):
        st.write('Servo mounting holes, shaft offset, horns/splines, bearings, fasteners, bracket thickness and print fits need your measured dimensions. Pose angles use model conventions; check your CAD mate directions. Finalizing saves a snapshot and does not certify strength or walking performance.')


def explain_guess():
    report=st.session_state.get("guess_report")
    if not report:
        st.info("Change your requirements, masses or geometry, then press Calculate & apply a good guess. Rounded-foot suggestions screen geometry, contact clearance, reach and supported-pelvis gravity loads; they do not claim dynamic balance or feasible grounded loading.")
        return
    if report.get("undone"):
        st.info("The last guess was undone. The calculation record below is retained for reference.")
    elif report["applied"] is not None:
        st.success(f"Applied a calculated starting point: {report['feasible']} of {report['tested']} candidates passed the applicable design checks.")
        st.caption(report.get('qualification','Static screening; dynamic balance not evaluated.'))
        if c!=report["applied"]:
            st.warning("Inputs have changed since this calculation. The explanation describes the recorded design; calculate again for your current inputs.")
    else:
        st.warning(f"No passing candidate among {report['tested']} tested. No inputs were changed.")
    st.write(f"**Search:** thigh {dimension(report['thigh_range'][0])}–{dimension(report['thigh_range'][1])}; shin {dimension(report['shin_range'][0])}–{dimension(report['shin_range'][1])}. {report['count']} grid samples per axis, plus current left-leg lengths when inside those bounds.")
    st.write("**1. Calculate each candidate’s geometry.** Keep your joint angles, foot attachment, platform and masses. Try a common thigh/shin pair on both legs. Compute the feet with forward kinematics, then translate pelvis height until the lowest sole reaches the ground. Other selected contacts must also agree.")
    st.latex(r"h_{hip}=L_1\cos(p+q_1)+L_2\cos(p+q_1+q_2)-\Delta z_{sole}")
    st.caption("Here Δzₛₒₗₑ is the lowest sole point relative to the two-link endpoint, including attachment offsets and rigid foot rotation. Total height comes from the complete rendered envelope, not hip height alone.")
    st.write("**2. Reject candidates that fail a requirement.** Check overall height, mass, payload, joint limits, swing targets, approximate packaging, contact geometry and gravity torque allowance. Flat-sole candidates also require static equilibrium. Rounded-foot candidates do not require static equilibrium: unsupported dynamic balance and stance loads are outside this recommendation.")
    st.dataframe([{"Check":key,"Candidates failing or not evaluated":value} for key,value in report["failures"].items()],hide_index=True)
    st.caption("One candidate can fail several checks. If all candidates fail, expand the search bounds or address the listed constraints. Changing length cannot level a rigid sole at fixed joint angles or reduce a manually entered mass.")
    st.write("**3. Rank the passing candidates.** Priority: "+report["priority"]+". Compare these quantities in order; later quantities break ties: "+" → ".join(length_label(k,unit) for k in report["order"])+".")
    st.write(f"The preferred height is the midpoint of your required band: {dimension(report['target_mm'])}. Height error = |predicted overall height − preferred height|. Torque utilization = worst screening torque / assumed working allowance. Length change = sum of absolute changes to the four leg lengths.")
    if report["chosen"] is not None:
        best=report["chosen"]
        st.write(f"**Chosen result:** overall height {dimension(best['overall height (mm)'])}; target error {dimension(best['height target error (mm)'])}; worst screening torque {best['objective worst screening (N·m)']:.4f} N·m; utilization {best['peak screening utilization']:.1%}.")
        before=effective(report["source"])
        st.dataframe(display_rows([{"Input":key,"Before (mm)":before[key],"Applied (mm)":report["applied"][key]} for key in CHANGED_KEYS if key!="mode"]),hide_index=True)
        st.caption(f"Support mode: {report['source']['mode']} → Grounded pose. The chosen lengths are applied to both legs, including independently edited legs.")
        st.write("**Top passing alternatives**")
        columns=["thigh (mm)","shin (mm)","overall height (mm)","height target error (mm)","peak screening utilization","length change (mm)"]
        st.dataframe(display_rows([{key:row[key] for key in columns} for row in report["shortlist"]]),hide_index=True)
    st.info("This is a finite-search engineering starting point. Manual masses/CoMs stay fixed. Rounded-foot torque screening assumes an externally supported pelvis, including the base pose and six discrete swing endpoints; it does not estimate actual walking/stance demands. Flat-sole screening also includes its assumed static ground reactions. Heating, full collision clearance and dynamic stability remain unevaluated.")


with overview:
    with st.expander("How the calculated good guess was derived",expanded="guess_report" in st.session_state):
        explain_guess()


def numeric(f):
    scale=field_factor(f)
    label=length_label(f.label,unit)
    step=0.001 if scale!=1 else f.step
    low,high=f.low/scale,f.high/scale
    for suffix in ("__s","__n"):
        if f.key+suffix not in st.session_state:
            st.session_state[f.key+suffix]=float(c[f.key])/scale
    left,right=st.columns([3,1])
    with left:
        st.slider(label,min_value=low,max_value=high,step=step,key=f.key+"__s",on_change=commit,args=(f.key,"s"),format="%.3f" if scale!=1 else None)
    with right:
        st.number_input(label+" — numeric",min_value=low,max_value=high,step=step,key=f.key+"__n",label_visibility="collapsed",on_change=commit,args=(f.key,"n"),format="%.3f" if scale!=1 else "%.4f" if f.key=="stall" else "%.2f")
    changed="Edited value" if c[f.key]!=f.default else "Default"
    st.caption(f"Input · {changed} · Default basis: {f.source}")
    if st.session_state.explanations:
        st.info(f.explanation)


def group(name):
    for f in FIELDS:
        if f.group==name:
            if c['foot_model']=='Rounded nub' and f.key.startswith(('L_','R_')) and f.key[2:] in ('foot_l','foot_w','foot_t','foot_angle'):
                continue
            if c['foot_model']=='Flat sole' and f.key.endswith('_nub_radius'):
                continue
            numeric(f)


with inputs:
    st.write(f"Inputs use {unit}, g, degrees and N·m. Derived results never resize the model to satisfy requirements. Sourced reference specifications retain their original units.")
    geo,pose,mass,act,req=st.tabs(["Geometry","Pose & IK","Mass distribution","Actuators","Requirements"])
    with geo:
        st.info('Rounded nub: shin endpoint + rigid attachment offset = sphere center. Radius determines the bottom contact point. There is no ankle joint or sole-angle constraint; one point per foot is used, with no assumed rubber contact patch.' if c['foot_model']=='Rounded nub' else 'Flat sole: foot orientation follows the shin plus its fixed attachment angle. Legacy configurations retain this model.')
        a,b=st.columns(2)
        with a:
            with st.expander("Platform, servo envelope & spacing",expanded=True): group("Geometry")
        with b:
            for side in ("L","R"):
                if side=="R" and c["symmetry"]:
                    st.info("Right geometry is derived from left while symmetry is enabled. Stored independent right values are preserved for later editing.")
                else:
                    with st.expander(f"{side} leg geometry",expanded=side=="L"): group(f"{side} geometry")
    with pose:
        a,b=st.columns(2)
        with a:
            with st.expander("Pelvis frame in world",expanded=True): group("Pose")
        with b:
            for side in ("L","R"):
                if side=="R" and c["symmetry"]:
                    st.caption("Right pose/limits/IK target follow left under symmetry.")
                    continue
                with st.expander(f"{side} angles, limits & inverse kinematics",expanded=side=="L"):
                    group(f"{side} pose")
                    st.selectbox("Knee branch",["Positive knee","Negative knee"],key=side+"_branch")
                    st.button(f"Apply {side} inverse kinematics",key=side+"_apply_ik",on_click=apply_ik,args=(side,))
        if "ik_message" in st.session_state:
            ok,message=st.session_state.ik_message
            (st.success if ok else st.error)(message)
        st.caption("Targets are in the pelvis frame from the relevant hip, before the fixed attachment offset. Folded equal links at target (0, 0) have infinitely many hip solutions; the solver returns a representative, subject to limits.")
    with mass:
        st.info("Manual lumped masses and local CoM coordinates stay fixed when link lengths change. Hip servo cases ride on the pelvis; knee servo cases ride on the thighs. All four servos are counted once. No density scaling is enabled.")
        fields={f.key:f for f in FIELDS}
        for key,label,body,*_ in MASS_ITEMS:
            if key.startswith("R_") and c["symmetry"]:
                continue
            with st.expander(f"{label} · carrier: {body}"):
                numeric(fields[key+"_mass"])
                for axis in "xyz": numeric(fields[key+"_com_"+axis])
        if c["symmetry"]: st.caption("All right-leg masses and local CoMs follow the left-leg inputs.")
    with act:
        group("Actuators")
        st.text_area("Basis for working torque assumption",key="option_working_basis",on_change=option,args=("working_basis",))
        st.warning("Stall torque is not continuous usable torque. The default 25% fraction is unvalidated. This tool does not calculate heating, continuous duty or dynamic walking loads.")
        if abs(c["voltage"]-6)>1e-6 and abs(c["voltage"]-4.8)>1e-6:
            st.warning("No manufacturer stall rating is sourced at this voltage. Enter a justified torque reference; the tool does not interpolate.")
        elif abs(c["stall"]-(1.0787315 if c["voltage"]==6 else .9218251))>0.001:
            st.info("Entered stall reference differs from TowerPro's rating at the selected voltage. Treat it as a user assumption and document the basis.")
    with req:
        group("Requirements")
        st.text_area("Configuration provenance / measurement notes",key="option_provenance",on_change=option,args=("provenance",))

if result:
    r,ct=result["robot"],result["contact"]
    with overview:
        cols=st.columns(4)
        cols[0].metric("Derived · overall geometric height",dimension(r['height']*1000))
        cols[1].metric("Derived · hip midpoint height",dimension(r['frames']['pelvis'][0][2]*1000))
        cols[2].metric("Derived · total modeled mass",f"{r['mass']*1000:.1f} g")
        margin=ct["margin"] if c["mode"]=="Grounded pose" and not ct["errors"] else None
        cols[3].metric("Derived · geometric tipping margin",dimension(margin*1000) if margin is not None else "Not evaluated")
        if c['foot_model']=='Rounded nub':
            st.warning('Rounded contacts provide a point or line, with zero support area. A static-equilibrium failure does not prove dynamic walking is impossible. Dynamic balance is not evaluated by this dashboard.')
            if ct['support_distance'] is not None:
                st.caption('CoM projection distance from the active support point/line: '+dimension(ct['support_distance']*1000)+'. This is not a dynamic stability score.')
        st.caption("Overall height is the z-span of rendered physical envelopes and link references. Hip height is world z above ground. Payload is a mass point without an envelope.")
        (st.success if result["status"]==FEASIBLE else st.error if result["status"]==INFEASIBLE else st.info)(f"Static pose checks: {result['status']}")
        st.caption(result["stance"]["reason"])
        if c["mode"]=="Pelvis fixed":
            st.info("Externally supported pelvis: foot motion and gravity loading can be explored, but this mode does not establish free-standing equilibrium. Ground marks show geometry only.")
        for error in ct["errors"]: st.warning(error)
        if result["singular"]: st.warning("Near singular straight/folded knee: "+", ".join(result["singular"])+". IK sensitivity is high.")
        v1,v2,v3=st.columns(3)
        show_masses=v1.checkbox("Show individual mass centers",value=True)
        show_workspace=v2.checkbox("Show sampled endpoint workspace")
        show_dimensions=v3.checkbox("Show dimensions & angle arcs",value=True)
        st.plotly_chart(length_figure(view3d(result,show_masses,show_workspace,show_dimensions),unit),width="stretch",key="robot3d")
        st.plotly_chart(length_figure(orthographic(result),unit),width="stretch",key="orthographic")
        st.dataframe([{"Requirement / check":k,"Status":v} for k,v in result["checks"].items()],hide_index=True,width="stretch")
        st.caption("Torque status uses the editable working allowance and screening multiplier. Packaging is approximate; dynamic stability, lateral control and thermal capacity are not evaluated.")
        st.write("**Current foot geometry**")
        st.dataframe(display_rows([{"leg":s,"sole pitch (deg)":l["tilt"],"lowest sole z (mm)":l["sole"][:,2].min()*1000} for s,l in r["legs"].items()]),hide_index=True)
        st.caption("; ".join(ct["details"]) or "No selected active sole vertices.")
        st.write("**Foot-reference reach: three discrete pelvis-fixed targets per leg**")
        samples=reach_samples(c)
        st.dataframe(display_rows([{k:v for k,v in s.items() if k!="solutions"} for s in samples]),hide_index=True)
        st.caption("Each target is tested from the current endpoint. Both IK branches are checked; intermediate motion, contact and collision clearance are not established.")
        with st.expander("Achievable heights & endpoint reach"):
            survey=height_survey(c)
            if survey is None:
                st.info("Coupled asymmetric achievable-height survey: not evaluated. Current geometry and per-leg reach remain evaluated.")
            elif not survey["count"]:
                st.info("No symmetric flat-foot sample meets the selected contact geometry and joint limits.")
            else:
                st.write(f"{survey['count']} sampled symmetric grounded geometric poses: overall height {dimension(survey['min'])}–{dimension(survey['max'])}; hip height {dimension(survey['hip_min'])}–{dimension(survey['hip_max'])}.")
                overlap=survey["max"]>=c["min_height"] and survey["min"]<=c["max_height"]
                st.caption(f"Sampled range overlaps requested height band: {overlap}. Sampling: {survey['protocol']}. This does not establish equilibrium, torque feasibility or a continuous range.")
            for side in ("L","R"):
                cc=r["config"];a,b=cc[f"{side}_thigh"],cc[f"{side}_shin"]
                st.caption(f"{side} unconstrained endpoint radius: |L₁−L₂| = {dimension(abs(a-b))} to L₁+L₂ = {dimension(a+b)}. Joint-limit workspace is sampled in the 3D overlay.")
        with st.expander("Transformed mass inventory"):
            st.dataframe(display_rows([{"mass":m.name,"carrier":m.body,"g":m.kg*1000,**{a:m.position[i]*1000 for i,a in enumerate(("world x (mm)","world y (mm)","world z (mm)"))}} for m in r["masses"]]),hide_index=True)
        with st.expander("Assumptions & model scope"):
            st.write("Level ground; rigid bodies; quasi-static gravity; adequate friction. Ground reaction model uses only vertical forces and zero horizontal prestress. The load-sharing solver minimizes peak vertex force with global equilibrium. Fixed hip/knee axes transmit structural moments in the blocked directions. The solution is one assumed distribution, not a stiffness prediction.")
            st.write("Rounded nubs contribute one point each; their orientation does not change ground clearance. Static vertical reactions are shown only if exact force/moment balance is possible, without implying a robust margin. Tilted flat soles contribute only their lowest edge; degenerate flat-sole cases remain unevaluated. Ground tolerance: 0.05 mm. Placement is explicit.")
            st.write("A side-view balance check does not establish lateral balance. Four pitch joints provide no powered lateral weight shift. A support-region check does not establish dynamic walking stability.")
            st.write("Packaging uses simple servo/foot spacing thresholds; it does not check all collisions, brackets, horns, cable routes, PLA strength or printability. CAD remains necessary.")
            for issue in packaging(r): st.write("• "+issue)

    with loading:
        st.write("**"+("Grounded stance" if c["mode"]=="Grounded pose" else "Suspended legs · pelvis externally fixed")+"**")
        st.write(result["stance"]["reason"] if c["mode"]=="Grounded pose" else "The external fixture supports the pelvis in all directions. Each joint balances only its distal subtree's gravity; this is a swing-leg load case.")
        if result["rows"]:
            st.dataframe(result["rows"],hide_index=True,width="stretch")
            st.caption(f"Allowable = {c['stall']:.4f} × {c['derating']:.2f} = {c['stall']*c['derating']:.4f} N·m. Basis: {c['working_basis']}")
            st.caption("Utilization = |static torque| × screening multiplier / assumed allowance. Demand is evaluated even if a configured angle exceeds its limit; the joint-limit check remains infeasible.")
            st.latex(r"\tau_{a,j}=\sum_{i\in distal(j)}m_i g(x_i-x_j)-\sum_{k\in contact(leg)}F_{z,k}(x_k-x_j)")
            st.caption("Signed torque is about −y, consistent with positive input angles. Distal hip subtree includes knee case, thigh, shin and foot. Distal knee subtree includes shin and foot; knee case rides on thigh.")
            first=result["rows"][0];joint=r["legs"]["L"]["hip"]
            st.write("**Worked example · left hip with current inputs**")
            st.dataframe([{"distal mass":m.name,"mass (kg)":m.kg,"x lever (m)":m.position[0]-joint[0],"mg·lever (N·m)":m.kg*G*(m.position[0]-joint[0])} for m in distal_masses(r,"L","hip")],hide_index=True)
            st.write(f"Gravity sum {first['gravity moment (N·m)']:.6f} − ground moment {first['ground moment (N·m)']:.6f} = signed actuator torque {first['signed actuator (N·m)']:.6f} N·m. Absolute static demand {first['static demand (N·m)']:.6f}; multiplied screening demand {first['screening demand (N·m)']:.6f} N·m.")
        else:
            st.info("Stance torque unavailable: "+result["stance"]["status"]+". "+result["stance"]["reason"])
        ground=result["stance"]
        if "forces" in ground:
            st.write("**Assumed contact load sharing**")
            st.dataframe(display_rows([{"leg":s,"x (mm)":p[0]*1000,"y (mm)":p[1]*1000,"vertical reaction (N)":f} for s,p,f in zip(ct["owners"],ct["points"],ground["forces"]) ]),hide_index=True)
            for side in ("L","R"):
                force=sum(f for owner,f in zip(ct["owners"],ground["forces"]) if owner==side)
                st.caption(f"{side} support reaction: {force:.4f} N ({100*force/(r['mass']*G):.1f}% of total weight).")
            st.code(f"Net force residual (N): {ground['force_residual']}\nNet moment residual (N·m): {ground['moment_residual']}")
            st.write("Required structural hip roll reaction moments about +x (N·m):",ground["structural_roll_moments"])
            st.caption("These are carried by the fixed-axis brackets/bearings. Their strength is not evaluated; no hip-roll actuator is modeled.")
        st.latex(r"\sum F_z=Mg,\quad \sum x_k F_{z,k}=Mg x_{CoM},\quad \sum y_k F_{z,k}=Mg y_{CoM},\quad F_{z,k}\geq0")
        st.write("Global reactions carry the entire robot weight, including platform, battery and payload. Joint moments then use the distal free body with those reactions; body weight is not added again to that subtree. Fixed-axis joints transmit structural roll moments without a powered roll degree of freedom. Horizontal ground forces are set to zero, so other equilibrium distributions may exist outside this model.")

with comparison:
    st.write("Compare a finite grid of thigh/shin lengths. Manual masses and local mass centers stay fixed; update them from CAD to refine candidates.")
    st.info("Protocol: replace both legs' link lengths, keep current angles and offsets, explicitly translate pelvis z until the lowest sole touches ground, then check the selected contacts. Test 3 additional pelvis-fixed swing endpoints per leg: lift, forward reach, and combined lift + reach. Search both IK branches within limits for each endpoint.")
    st.write("Objective: minimize the largest screening torque across the stance and swing endpoint samples. Each swing endpoint uses the branch with the lowest peak limb demand. This is a limited comparison, not a globally optimal design or a walking trajectory.")
    def sync_range(name,kind):
        if kind=="slider":
            lo,hi=st.session_state[name+"_range"]
            st.session_state[name+"_lo"],st.session_state[name+"_hi"]=lo,hi
        else:
            lo,hi=st.session_state[name+"_lo"],st.session_state[name+"_hi"]
            if lo>hi:
                hi=lo
                st.session_state[name+"_hi"]=hi
            st.session_state[name+"_range"]=(lo,hi)
        st.session_state["sweep_mm_"+name]=tuple(float(v)*factor for v in st.session_state[name+"_range"])
    def sync_count(source):
        dest="sweep_count" if source=="sweep_count_slider" else "sweep_count_slider"
        st.session_state[dest]=st.session_state[source]
    a,b,d=st.columns(3)
    for name,col in (("thigh",a),("shin",b)):
        if name+"_range" not in st.session_state:
            st.session_state[name+"_range"]=(45./factor,90./factor)
            st.session_state[name+"_lo"]=45./factor
            st.session_state[name+"_hi"]=90./factor
        if "sweep_mm_"+name not in st.session_state:
            st.session_state["sweep_mm_"+name]=tuple(v*factor for v in st.session_state[name+"_range"])
        with col:
            st.slider(name.title()+f" sweep range ({unit})",min_value=20./factor,max_value=160./factor,step=.01 if unit=="in" else 1.,key=name+"_range",on_change=sync_range,args=(name,"slider"),format="%.3f" if unit=="in" else "%.1f")
            lo_col,hi_col=st.columns(2)
            lo_col.number_input(name.title()+f" minimum ({unit})",min_value=20./factor,max_value=160./factor,step=.01 if unit=="in" else 1.,key=name+"_lo",on_change=sync_range,args=(name,"numeric"),format="%.3f" if unit=="in" else "%.1f")
            hi_col.number_input(name.title()+f" maximum ({unit})",min_value=20./factor,max_value=160./factor,step=.01 if unit=="in" else 1.,key=name+"_hi",on_change=sync_range,args=(name,"numeric"),format="%.3f" if unit=="in" else "%.1f")
    if "sweep_count" not in st.session_state:
        st.session_state.sweep_count=6
        st.session_state.sweep_count_slider=6
    with d:
        st.slider("Samples per axis",min_value=2,max_value=15,step=1,key="sweep_count_slider",on_change=sync_count,args=("sweep_count_slider",))
        count=st.number_input("Samples per axis — numeric",min_value=2,max_value=15,step=1,key="sweep_count",on_change=sync_count,args=("sweep_count",))
        st.caption("Assumed default: 6 × 6 candidates. All ranges are editable; max 225 candidates.")
    tr,sr=st.session_state.sweep_mm_thigh,st.session_state.sweep_mm_shin
    selected=st.multiselect("Criteria required for candidate feasibility",CRITERIA,default=design_criteria(c),key="criteria")
    if c['foot_model']=='Rounded nub':
        st.warning('Rounded-foot comparison: the default filters omit static Stance. Torque uses externally supported-pelvis gravity screening, not ground reactions or dynamic loads. Passing candidates are geometry/load-screen candidates; walking balance remains not evaluated.')
    if st.button("Run dimension comparison",key="run_sweep",disabled=result is None):
        with st.spinner("Evaluating geometry, contact equilibrium and endpoint loads…"):
            st.session_state.sweep_rows=compare(c,np.linspace(*tr,int(count)),np.linspace(*sr,int(count)),selected)
            st.session_state.sweep_signature=(dumps(c),tr,sr,count,tuple(selected))
    if st.session_state.get("sweep_rows"):
        if st.session_state.sweep_signature != (dumps(c),tr,sr,count,tuple(selected)):
            st.warning("Comparison settings changed. Run again to refresh; results below retain their original protocol and selected criteria.")
        rows=st.session_state.sweep_rows
        shown_rows=display_rows(rows)
        st.plotly_chart(px.scatter(shown_rows,x=f"thigh ({unit})",y=f"shin ({unit})",color="selected checks",
            color_discrete_map={FEASIBLE:"#10b981",INFEASIBLE:"#ef4444",NOT_EVALUATED:"#94a3b8"},
            hover_data=[f"overall height ({unit})","objective worst screening (N·m)","rank"],title="Candidate feasibility under selected checks"),width="stretch")
        st.dataframe(shown_rows,hide_index=True,width="stretch")
        buffer=io.StringIO(newline="");writer=csv.DictWriter(buffer,fieldnames=list(shown_rows[0]));writer.writeheader();writer.writerows(shown_rows)
        st.download_button("Export comparison CSV",buffer.getvalue(),"biped_comparison.csv","text/csv",key="export_csv")
        st.download_button("Save comparison source configuration",st.session_state.sweep_signature[0],"comparison_source.json","application/json")
        ranked=sorted((r for r in rows if r["rank"] is not None),key=lambda x:x["rank"])
        if ranked:
            best=ranked[0]
            st.success(f"Lowest sampled objective among candidates passing selected checks: thigh {dimension(best['thigh (mm)'])}, shin {dimension(best['shin (mm)'])}, worst screening torque {best['objective worst screening (N·m)']:.4f} N·m.")
        else:
            st.info("No fully evaluated candidate passing the selected checks is available to rank.")

with motion:
    st.subheader('Rounded-foot walking preview')
    st.warning('Kinematic preview, not a prediction of real-world walking. Body motion is prescribed; there is no dynamic balance controller, friction/impact model, or powered lateral weight shift. The real robot may fall even when all preview frames are reachable.')
    st.write('The pelvis advances at constant height and speed. Legs alternate half a cycle apart. Each nub follows no-slip sagittal rolling during stance, then a smooth lifted path to its next touchdown. Both legs use your current lengths, radii, offsets and limits; the animation generates new joint angles without changing your saved pose.')
    st.caption(f"Each alternating step advances the body by {dimension(c['reach'])}; each foot repeats after a stride of {dimension(2*c['reach'])}. Peak swing clearance is {dimension(c['lift'])}. Change Forward reach and Desired foot-reference lift under Design inputs → Requirements. Those values specify motion targets, not guaranteed walking capability.")
    def sync_motion(key,kind):
        other='n' if kind=='s' else 's'
        st.session_state[key+'__'+other]=st.session_state[key+'__'+kind]
    def motion_input(key,label,lo,hi,value,step):
        for kind in ('s','n'):
            if key+'__'+kind not in st.session_state:
                st.session_state[key+'__'+kind]=value
        a,b=st.columns([3,1])
        a.slider(label,min_value=lo,max_value=hi,step=step,key=key+'__s',on_change=sync_motion,args=(key,'s'))
        v=b.number_input(label+' — numeric',min_value=lo,max_value=hi,step=step,key=key+'__n',on_change=sync_motion,args=(key,'n'),label_visibility='collapsed')
        return v
    period=motion_input('motion_period','Full cycle duration (s)',.4,10.,1.6,.05)
    duty=motion_input('motion_duty','Stance fraction of each leg cycle',.5,.8,.6,.01)
    cycles=st.selectbox('Cycles to preview',[1,2,3,4],index=1,key='motion_cycles')
    st.caption('Assumed preview defaults: 1.6 s cycle, 60% stance, two cycles. More than 50% stance creates double-contact intervals. Timing controls only the animation and calculated kinematic speeds; actuator response is not simulated.')
    signature=(dumps(c),period,duty,cycles) if result else None
    if st.button('Build walking preview',key='build_preview',disabled=result is None or c['foot_model']!='Rounded nub'):
        with st.spinner('Solving rounded-foot rolling and swing trajectories…'):
            try:
                st.session_state.motion_result=preview(c,period=period,duty=duty,cycles=int(cycles))
                st.session_state.motion_source=signature
                st.session_state.pop('motion_error',None)
            except ValueError as exc:
                st.session_state.motion_error=str(exc)
    if c['foot_model']!='Rounded nub':
        st.info('Select Rounded nub in the sidebar to build this preview. Imported version-1 configurations retain their original flat soles until you switch explicitly.')
    if st.session_state.get('motion_error'):
        st.error(st.session_state.motion_error)
    if st.session_state.get('motion_result'):
        if signature!=st.session_state.motion_source:
            st.info('Inputs or timing changed. Build the preview again to update the motion; the old animation is hidden.')
        else:
            generated=st.session_state.motion_result
            if generated['failure']:
                st.error(generated['failure'])
                st.caption('Any animation below stops at the last valid frame. No unreachable frame is replaced by a plausible-looking pose. Try a shorter step, lower lift or a different explicit hip height.')
            if generated['frames']:
                a,b,d=st.columns(3)
                a.metric('Valid sampled poses',str(len(generated['frames'])))
                b.metric('Prescribed forward speed',dimension(generated['speed_mm_s'])+'/s')
                d.metric('Peak sampled joint speed',f"{generated['max_joint_speed_deg_s']:.1f} deg/s")
                # Streamlit's native Plotly renderer does not register go.Frame
                # sequences. A self-contained document runs Plotly.addFrames,
                # enabling real Play/Pause/scrubbing without any CDN dependency.
                movie=animation(generated,unit).to_html(include_plotlyjs=True,full_html=False,auto_play=False,
                    config={'responsive':True,'displaylogo':False})
                component_html('<style>body{margin:0;background:#0e1117}</style>'+movie,height=690,scrolling=False)
                st.caption('Press Play; drag the frame slider to inspect individual poses. Red is CoM/projection; green is the actual contact point or line. Sphere center motion during stance is rolling, not a fixed-center foot pivot. No lateral recovery motion is invented.')
                rows=[]
                for frame in generated['frames']:
                    robot=frame['robot']
                    row={'time (s)':frame['time'],'L phase':frame['phases']['L'],'R phase':frame['phases']['R'],
                         'pelvis x (mm)':robot['frames']['pelvis'][0][0]*1000,'pelvis z (mm)':robot['frames']['pelvis'][0][2]*1000,
                         'CoM x (mm)':robot['com'][0]*1000,'CoM y (mm)':robot['com'][1]*1000,
                         'dynamic balance':NOT_EVALUATED}
                    for side in ('L','R'):
                        row[f'{side} nub center x (mm)']=robot['legs'][side]['foot'][0]*1000
                        row[f'{side} ground clearance (mm)']=robot['legs'][side]['sole'][0,2]*1000
                        for joint,angle in zip(('hip','knee'),robot['legs'][side]['angles']):
                            row[f'{side} {joint} (deg)']=float(np.degrees(angle))
                    rows.append(row)
                shown=display_rows(rows)
                with st.expander('Frame measurements and joint angles'):
                    st.dataframe(shown,hide_index=True,width='stretch')
                buffer=io.StringIO(newline='');writer=csv.DictWriter(buffer,fieldnames=list(shown[0]));writer.writeheader();writer.writerows(shown)
                st.download_button('Export preview samples CSV',buffer.getvalue(),'kinematic_walking_preview.csv','text/csv',key='export_motion')
                st.download_button('Download interactive walking preview',movie,'walking_preview.html','text/html',key='export_motion_html')
                st.caption('Exported joint angles are model coordinates, not calibrated servo commands. This file does not validate a controller or gait.')

with guide:
    st.markdown("""
1. In **Design inputs → Geometry**, enter CAD hip-to-knee and knee-to-foot-reference lengths. Keep symmetry on initially. Check the MG996R envelopes against your real units.
2. In **Pose & IK**, start with hip −20°, knee +40°, and fixed foot angle −20°. Change one joint to see why a rigid foot tilts. Use **Place lowest selected sole on ground** when you deliberately want to translate pelvis height.
3. Enter each printed part's mass and local CoM from CAD. Enter the battery and payload positions. Observe the CoM projection and both front/top views as well as the side view.
4. Set height, reach and mass requirements. Adjust the working torque assumption only with an explicit basis. Inspect **Static loading** and its equilibrium residuals.
5. Run **Dimension comparison**, inspect failed checks, and export the table with its source configuration. Transfer promising candidates back into CAD for real clearance, strength and assembly checks.
""")
    st.write("**Coordinate convention · side view**")
    st.code("             z ↑\n       platform / pelvis\n               H (hip)   x → forward\n               |\\  +hip tips −z toward +x\n               | K (knee; relative angle)\n               |  \\\n               |   E (foot reference)\n              rigid foot: orientation = pelvis + hip + knee + attachment angle\n       ───────────────────── ground z=0\n       y points to robot's left; all joint axes parallel to −y")
    st.latex(r"x_E=L_1\sin q_1+L_2\sin(q_1+q_2),\quad z_E=-L_1\cos q_1-L_2\cos(q_1+q_2)")
    st.latex(r"c_2=\frac{x^2+z^2-L_1^2-L_2^2}{2L_1L_2},\quad q_2=\pm\arccos c_2,\quad q_1=\mathrm{atan2}(x,-z)-\mathrm{atan2}(L_2\sin q_2,L_1+L_2\cos q_2)")
    st.latex(r"p_{CoM}=\frac{\sum_i m_i(o_{body(i)}+R_{body(i)}p_{i,local})}{\sum_i m_i}")
    st.markdown("Sources: [Modern Robotics · two-link IK](https://modernrobotics.northwestern.edu/nu-gm-book-resource/inverse-kinematics-of-open-chains/), [MIT · legged contact](https://underactuated.mit.edu/humanoids.html), [TowerPro · MG996R specifications](https://towerpro.com.tw/product/mg996R/). Full assumptions, coordinate definitions and limitations are in `SOURCES_AND_MODEL.md` beside the code.")

