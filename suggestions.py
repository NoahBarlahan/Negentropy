"""Explainable finite-grid dimension suggestions, using the existing mechanics."""
from copy import deepcopy
import numpy as np
from config import effective, validate
from model import FEASIBLE, evaluate
from sweep import CRITERIA, compare, design_criteria

PRIORITIES = ["Height near middle of required band", "Lowest screening torque", "Smallest length change"]
CHANGED_KEYS = ["L_thigh", "R_thigh", "L_shin", "R_shin", "pelvis_z", "mode"]


def suggest(data, thigh_range=(45.,90.), shin_range=(45.,90.), count=6, priority=PRIORITIES[0]):
    """Never mutate the source; only recommend a candidate passing ALL checks.

    Lexicographic ranking exposes the chosen priority without hidden weights.
    Include current L dimensions when within bounds to avoid needless movement
    just because they fall between grid points. Propose common lengths on both legs.
    """
    c=effective(data)
    if priority not in PRIORITIES:
        raise ValueError("Unknown suggestion priority.")
    if isinstance(count,bool) or not isinstance(count,(int,np.integer)) or not 2<=count<=15:
        raise ValueError("Use 2–15 samples per axis.")
    axes=[]
    for bounds,key in ((thigh_range,"L_thigh"),(shin_range,"L_shin")):
        if len(bounds)!=2 or not np.isfinite(bounds).all() or not 5<=bounds[0]<=bounds[1]<=200:
            raise ValueError("Search bounds must be finite, ordered and within 5–200 mm.")
        values=list(np.linspace(*bounds,count))
        if bounds[0]<=c[key]<=bounds[1]:
            values.append(c[key])
        axes.append(np.unique(values))
    required=design_criteria(c)
    rows=compare(c,*axes,required)
    target=(c["min_height"]+c["max_height"])/2
    allowance=c["stall"]*c["derating"]
    for row in rows:
        row["height target error (mm)"]=abs(row["overall height (mm)"]-target)
        row["length change (mm)"]=sum(abs(row[f"{part} (mm)"]-c[f"{side}_{part}"]) for side in ("L","R") for part in ("thigh","shin"))
        row["peak screening utilization"]=row["objective worst screening (N·m)"]/allowance
    order={PRIORITIES[0]:["height target error (mm)","peak screening utilization","length change (mm)"],
           PRIORITIES[1]:["peak screening utilization","height target error (mm)","length change (mm)"],
           PRIORITIES[2]:["length change (mm)","height target error (mm)","peak screening utilization"]}[priority]
    feasible=sorted([r for r in rows if r["selected checks"]==FEASIBLE and np.isfinite(r["objective worst screening (N·m)"])],
                    key=lambda r:tuple(round(float(r[key]),10) for key in order))
    report={"source":deepcopy(data),"priority":priority,"order":order,"target_mm":target,
            "thigh_range":tuple(thigh_range),"shin_range":tuple(shin_range),"count":int(count),
            "required":required,"qualification":"Geometry/reach and supported-pelvis gravity candidate; dynamic balance not evaluated" if c["foot_model"]=="Rounded nub" else "Static screening candidate; dynamic balance not evaluated",
            "tested":len(rows),"feasible":len(feasible),"failures":{
                criterion:sum(r[criterion]!=FEASIBLE for r in rows) for criterion in CRITERIA},
            "shortlist":feasible[:5],"chosen":None,"applied":None}
    if feasible:
        best=feasible[0]
        proposed=deepcopy(data)
        for side in ("L","R"):
            proposed[f"{side}_thigh"]=best["thigh (mm)"]
            proposed[f"{side}_shin"]=best["shin (mm)"]
        proposed["pelvis_z"]=best["hip height (mm)"]
        proposed["mode"]="Grounded pose"
        validate(proposed)
        # Re-evaluate exactly what will be applied, including symmetry semantics.
        final=evaluate(proposed)
        checked=[v for k,v in final["checks"].items() if k not in ("Static ground equilibrium","Dynamic walking balance","Screening torque allowance")]
        if (any(v!=FEASIBLE for v in checked) or final['contact']['errors'] or
            (c["foot_model"]=="Flat sole" and final["status"]!=FEASIBLE)):
            raise ValueError("Candidate failed the final application check; no inputs changed.")
        report["chosen"]=best
        report["applied"]=proposed
    return report


def undo(current,report):
    """Undo only suggested fields; preserve later edits to requirements/masses."""
    if report.get("applied") is None:
        raise ValueError("No applied suggestion to undo.")
    if any(current[key]!=report["applied"][key] for key in CHANGED_KEYS):
        raise ValueError("Suggested fields have since been edited. Undo is unavailable; change them manually or calculate a new guess.")
    restored=deepcopy(current)
    for key in CHANGED_KEYS:
        restored[key]=report["source"][key]
    return validate(restored)
