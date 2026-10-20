"""Finite candidate comparisons. No global optimality or dynamic gait claim."""
from copy import deepcopy
import numpy as np
from config import effective
from model import evaluate, reach_samples, build, torque_rows, FEASIBLE, INFEASIBLE, NOT_EVALUATED

CRITERIA = ["Height", "Mass", "Payload", "Joint limits", "Reach targets", "Packaging", "Contact geometry", "Stance", "Torque"]


def design_criteria(data):
    return [key for key in CRITERIA if key!="Stance"] if data["foot_model"]=="Rounded nub" else list(CRITERIA)


def compare(data, thighs, shins, selected):
    rows=[]
    for thigh in thighs:
        for shin in shins:
            c=effective(data)
            for side in ("L","R"):
                c[f"{side}_thigh"],c[f"{side}_shin"] = float(thigh),float(shin)
            # Explicit comparison protocol: keep angles, translate pelvis z to
            # put the lowest sole on ground. This is never applied to UI config.
            candidate=build(c)
            shift=min(l["sole"][:,2].min() for l in candidate["legs"].values())*1000
            c["pelvis_z"]-=shift
            if not -100 <= c["pelvis_z"] <= 500:
                continue
            c["mode"]="Grounded pose"
            e=evaluate(c)
            checks=e["checks"]
            samples=reach_samples(c)
            peak=max((r["screening demand (N·m)"] for r in e["rows"]),default=np.nan)
            all_reach=all(s["within limits"] for s in samples)
            # For each limb/endpoint pick the in-limit branch minimizing that
            # limb's maximum suspended torque, then take the worst endpoint.
            swing_peak=0.
            if all_reach:
                for sample in samples:
                    values=[]
                    for hip,knee in sample["solutions"]:
                        test=deepcopy(c);test["symmetry"]=False
                        side=sample["side"]
                        test[f"{side}_hip"],test[f"{side}_knee"]=float(np.degrees(hip)),float(np.degrees(knee))
                        values.append(max(r["screening demand (N·m)"] for r in torque_rows(build(test)) if r["joint"].startswith(side)))
                    swing_peak=max(swing_peak,min(values))
            else:
                swing_peak=np.nan
            allowance=c["stall"]*c["derating"]
            rounded=c["foot_model"]=="Rounded nub"
            supported_peak=max(r["screening demand (N·m)"] for r in torque_rows(e["robot"]))
            worst=(max(supported_peak,swing_peak) if np.isfinite(swing_peak) else np.nan) if rounded else (
                max(peak,swing_peak) if np.isfinite(peak) and np.isfinite(swing_peak) else np.nan)
            flags={"Height": checks["Current overall geometric height"],"Mass":checks["Total mass"],
                   "Payload":checks["Entered payload target"],"Joint limits":checks["Joint limits"],
                   "Reach targets":FEASIBLE if all_reach else INFEASIBLE,"Packaging":checks["Approximate packaging"],
                   "Contact geometry":INFEASIBLE if e["contact"]["errors"] else FEASIBLE,
                   "Stance":checks["Static ground equilibrium"],
                   "Torque":(FEASIBLE if worst<=allowance else INFEASIBLE) if np.isfinite(worst) else NOT_EVALUATED}
            chosen=[flags[k] for k in selected]
            status=INFEASIBLE if INFEASIBLE in chosen else NOT_EVALUATED if NOT_EVALUATED in chosen or not chosen else FEASIBLE
            rows.append({"thigh (mm)":float(thigh),"shin (mm)":float(shin),"overall height (mm)":e["robot"]["height"]*1000,
                         "hip height (mm)":c["pelvis_z"],"mass (g)":e["robot"]["mass"]*1000,
                         "peak stance screening (N·m)":peak,"peak swing screening (N·m)":swing_peak,
                         "objective worst screening (N·m)":worst,"selected checks":status,
                         "protocol":"Current angles + explicit ground translation; 3 swing endpoints per leg; manual masses fixed",
                         "torque basis":"Externally supported pelvis gravity only; grounded/dynamic loads excluded" if rounded else "Stance plus supported-pelvis swing endpoints",
                         "dynamic balance":NOT_EVALUATED,
                         "selected criteria":", ".join(selected), **flags})
    # Ranking only evaluated feasible candidates, by stated objective.
    feasible=sorted([r for r in rows if r["selected checks"]==FEASIBLE and np.isfinite(r["objective worst screening (N·m)"])],key=lambda r:r["objective worst screening (N·m)"])
    for row in rows:
        row["rank"]=next((i+1 for i,r in enumerate(feasible) if r is row),None)
    return rows
