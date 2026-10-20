"""SI engineering core. World x forward, y left, z up; pitch is about −y."""
from dataclasses import dataclass
import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull, QhullError
from config import effective, MASS_ITEMS

G = 9.80665
GROUND_TOL = 0.00005  # 0.05 mm geometric tolerance, not a compliance model
FEASIBLE = "feasible under this model"
INFEASIBLE = "infeasible"
NOT_EVALUATED = "not evaluated"


def rotation(angle):
    """Radians; positive pitch swings a downward link forward."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])


def fk(thigh, shin, hip, knee):
    """SI lengths/radians; returns knee and endpoint relative to hip."""
    if not np.isfinite([thigh, shin, hip, knee]).all() or min(thigh, shin) <= 0:
        raise ValueError("Positive finite link lengths and finite angles required.")
    k = rotation(hip) @ np.array([0., 0., -thigh])
    return k, k + rotation(hip + knee) @ np.array([0., 0., -shin])


def ik(thigh, shin, x, z):
    """Both analytic branches in radians; target in the hip's parent x/z frame."""
    if not np.isfinite([thigh, shin, x, z]).all() or min(thigh, shin) <= 0:
        raise ValueError("Positive finite lengths and finite target required.")
    d = (x*x + z*z - thigh*thigh - shin*shin) / (2*thigh*shin)
    if abs(d) > 1 + 1e-10:
        return []
    if np.hypot(x, z) < 1e-10 and abs(thigh-shin) < 1e-10:
        # Infinite hip solutions. A representative is returned and explicitly flagged.
        return [(0., np.pi), (0., -np.pi)]
    a = np.arccos(np.clip(d, -1, 1))
    solutions = []
    for q2 in (a, -a):
        q1 = np.arctan2(x, -z) - np.arctan2(shin*np.sin(q2), thigh+shin*np.cos(q2))
        q1 = (q1 + np.pi) % (2*np.pi) - np.pi
        solutions.append((float(q1), float(q2)))
    return solutions


def within_limits(c, side, angles):
    return all(c[f"{side}_{joint}_min"] - 1e-8 <= np.degrees(a) <= c[f"{side}_{joint}_max"] + 1e-8
               for joint, a in zip(("hip", "knee"), angles))


def nub_ik(c,side,x,z):
    """Both IK branches for a nub center in pelvis x/z relative to its hip."""
    ox=c[f'{side}_foot_ox']/1000
    down=(c[f'{side}_shin']-c[f'{side}_foot_oz'])/1000
    length=np.hypot(ox,down)
    if length<1e-9:
        return []
    beta=np.arctan2(ox,down)
    return [(a,float((b-beta+np.pi)%(2*np.pi)-np.pi))
            for a,b in ik(c[f'{side}_thigh']/1000,length,x,z)]


def box(center, orientation, dimensions):
    signs = np.array([[-1,-1,-1], [1,-1,-1], [1,1,-1], [-1,1,-1],
                      [-1,-1,1], [1,-1,1], [1,1,1], [-1,1,1]])
    return center + (signs * np.asarray(dimensions)/2) @ orientation.T


@dataclass
class Mass:
    name: str
    body: str
    kg: float
    position: np.ndarray


def build(data):
    c = effective(data)
    rp = rotation(np.radians(c["pitch"]))
    pelvis = np.array([c[f"pelvis_{a}"] for a in "xyz"]) / 1000
    frames = {"pelvis": (pelvis, rp)}
    platform = pelvis + rp @ (np.array([c["platform_x"], 0, c["platform_z"]])/1000)
    frames["platform"] = (platform, rp)
    boxes = {"platform": box(platform, rp, [c[f"platform_{a}"]/1000 for a in ("l","w","t")])}
    legs = {}
    for side, sign in (("L", 1), ("R", -1)):
        hip = pelvis + rp @ np.array([0, sign*c["spacing"]/2000, 0])
        q1, q2 = np.radians([c[f"{side}_hip"], c[f"{side}_knee"]])
        knee_rel, end_rel = fk(c[f"{side}_thigh"]/1000, c[f"{side}_shin"]/1000, q1, q2)
        knee, end = hip + rp @ knee_rel, hip + rp @ end_rel
        rt, rs = rp @ rotation(q1), rp @ rotation(q1 + q2)
        rounded=c["foot_model"]=="Rounded nub"
        rf = rs if rounded else rs @ rotation(np.radians(c[f"{side}_foot_angle"]))
        foot = end + rs @ np.array([c[f"{side}_foot_ox"], 0, c[f"{side}_foot_oz"]])/1000
        frames.update({f"{side}_hip": (hip, rp), f"{side}_thigh": (hip, rt),
                       f"{side}_shin": (knee, rs), f"{side}_foot": (foot, rf)})
        thickness = c[f"{side}_foot_t"]/1000
        radius=c[f"{side}_nub_radius"]/1000
        boxes[f"{side}_foot"] = (box(foot,np.eye(3),[2*radius]*3) if rounded else
            box(foot + rf @ np.array([0,0,-thickness/2]), rf,
                [c[f"{side}_foot_l"]/1000, c[f"{side}_foot_w"]/1000, thickness]))
        for joint, origin, carrier in (("hip", hip, rp), ("knee", knee, rt)):
            center = origin + carrier @ (np.array([c[f"{side}_{joint}_case_{a}"] for a in "xyz"])/1000)
            boxes[f"{side}_{joint}"] = box(center, carrier, [c[f"servo_{a}"]/1000 for a in ("l","w","h")])
        legs[side] = {"hip": hip, "knee": knee, "end": end, "foot": foot, "angles": (q1,q2),
                      "sole": np.array([foot-[0,0,radius]]) if rounded else boxes[f"{side}_foot"][:4],
                      "tilt": None if rounded else c["pitch"] + np.degrees(q1+q2) + c[f"{side}_foot_angle"],
                      "radius": radius if rounded else 0.}
    masses = []
    for key, label, body, *_ in MASS_ITEMS:
        origin, r = frames[body]
        pos = origin + r @ (np.array([c[f"{key}_com_{a}"] for a in "xyz"])/1000)
        masses.append(Mass(label, body, c[f"{key}_mass"]/1000, pos))
    total = sum(m.kg for m in masses)
    if total <= 0:
        raise ValueError("Effective (symmetry-applied) mass must be positive.")
    com = sum((m.kg*m.position for m in masses), start=np.zeros(3))/total
    vertices = np.vstack([*boxes.values(), *[np.array([l["hip"], l["knee"], l["end"]]) for l in legs.values()]])
    return {"config": c, "legs": legs, "boxes": boxes, "masses": masses, "mass": total,
            "com": com, "vertices": vertices, "height": np.ptp(vertices[:,2]), "frames": frames}


def support_polygon(points):
    p = np.unique(np.asarray(points).reshape(-1, 2), axis=0)
    if len(p) < 3:
        return np.empty((0,2))
    try:
        return p[ConvexHull(p).vertices]
    except QhullError:
        return np.empty((0,2))


def tipping_margin(polygon, point):
    """Minimum signed inward distance to a CCW convex polygon, SI units."""
    if len(polygon) < 3:
        return None
    edges = np.roll(polygon, -1, axis=0) - polygon
    offsets = np.asarray(point)[:2] - polygon
    return float(np.min((edges[:,0]*offsets[:,1]-edges[:,1]*offsets[:,0])/np.linalg.norm(edges, axis=1)))


def contacts(robot):
    """Only the lowest sole corners can contact; tilted soles contribute an edge."""
    c = robot["config"]
    selected = {"Both": {"L","R"}, "Left": {"L"}, "Right": {"R"}, "None": set()}[c["contacts"]]
    actual, owners, details, errors = [], [], [], []
    for side, leg in robot["legs"].items():
        sole = leg["sole"]
        minimum = float(sole[:,2].min())
        if minimum < -GROUND_TOL:
            errors.append(f"{side} foot penetrates ground by {-minimum*1000:.3f} mm.")
        if side in selected:
            if abs(minimum) > GROUND_TOL:
                errors.append(f"{side} selected contact is not on ground (lowest sole {minimum*1000:.3f} mm).")
                continue
            low = sole[np.abs(sole[:,2]-minimum) < 1e-9]
            # Set only tolerated numerical contact heights to ground in force calculations.
            low = low.copy()
            low[:,2] = 0
            actual.extend(low)
            owners.extend([side]*len(low))
            details.append(f"{side}: {'rounded nub — one point' if c['foot_model']=='Rounded nub' else 'flat sole' if len(low)==4 else 'edge' if len(low)==2 else 'point'} ({len(low)} vertices)")
        elif minimum <= GROUND_TOL:
            errors.append(f"{side} unselected foot also touches/penetrates ground; contact selection is inconsistent.")
    for name, verts in robot["boxes"].items():
        if not name.endswith("_foot") and verts[:,2].min() < -GROUND_TOL:
            errors.append(f"{name} envelope penetrates ground.")
    points = np.array(actual).reshape(-1,3)
    polygon = support_polygon(points[:,:2])
    distance=None
    if len(points) and not len(polygon):
        a,b=points[0,:2],points[-1,:2]
        ab=b-a
        t=np.clip(np.dot(robot["com"][:2]-a,ab)/np.dot(ab,ab),0,1) if np.dot(ab,ab)>1e-15 else 0
        distance=float(np.linalg.norm(robot["com"][:2]-(a+t*ab)))
    return {"points": points, "owners": owners, "polygon": polygon, "details": details,
            "errors": errors, "margin": tipping_margin(polygon, robot["com"]),"support_distance":distance}


def gravity_moment(masses, joint):
    """Actuator moment about −y required to balance gravity alone."""
    return float(sum(m.kg*G*(m.position[0]-joint[0]) for m in masses))


def distal_masses(robot, side, joint):
    bodies = {f"{side}_shin", f"{side}_foot"}
    if joint == "hip":
        bodies.add(f"{side}_thigh")
    return [m for m in robot["masses"] if m.body in bodies]


def torque_rows(robot, contact=None, forces=None):
    c = robot["config"]
    allowable = c["stall"]*c["derating"]
    rows = []
    for side, leg in robot["legs"].items():
        for joint in ("hip", "knee"):
            origin = leg[joint]
            gravity = gravity_moment(distal_masses(robot, side, joint), origin)
            reaction = 0.
            if forces is not None:
                reaction = sum(f*(p[0]-origin[0]) for f,p,owner in zip(forces, contact["points"], contact["owners"]) if owner == side)
            signed = gravity - reaction
            rows.append({"joint": f"{side} {joint}", "gravity moment (N·m)": gravity,
                         "ground moment (N·m)": reaction, "signed actuator (N·m)": signed,
                         "static demand (N·m)": abs(signed), "screening demand (N·m)": abs(signed)*c["multiplier"],
                         "allowable assumption (N·m)": allowable,
                         "utilization": abs(signed)*c["multiplier"]/allowable})
    return rows


def stance(robot, contact):
    """Vertical-only contact solution with full global force/moment balance.

    Minimize peak vertex force subject to global force/pitch/roll equilibrium,
    nonnegative contact forces. Fixed joint axes transmit structural roll moments.
    Horizontal reactions/internal prestress are set to zero. This is one load
    sharing assumption, not a unique or compliance-predicted force distribution.
    """
    result = {"status": NOT_EVALUATED, "reason": "Pelvis-fixed exploration has external support.", "rows": []}
    c = robot["config"]
    if c["mode"] != "Grounded pose":
        return result
    if contact["errors"]:
        return {**result, "status": INFEASIBLE, "reason": " ".join(contact["errors"])}
    if not len(contact["points"]):
        return {**result, "status": INFEASIBLE, "reason": "No active ground contacts."}
    if contact["margin"] is None and c["foot_model"]!="Rounded nub":
        return {**result, "reason": "Degenerate point/line support region; stance loading is not implemented for this case."}
    if contact["margin"] is not None and contact["margin"] < -1e-8:
        return {**result, "status": INFEASIBLE, "reason": "CoM projection lies outside actual support region."}
    p, n = contact["points"], len(contact["points"])
    weight = robot["mass"]*G
    # Shift moments to CoM to improve conditioning; scale lengths to ~1.
    aeq = [np.ones(n), (p[:,0]-robot["com"][0])*10, (p[:,1]-robot["com"][1])*10]
    beq = [weight, 0, 0]
    aeq = np.column_stack([np.array(aeq), np.zeros(len(aeq))])
    aub = np.column_stack([np.eye(n), -np.ones(n)])
    solution = linprog(np.r_[np.zeros(n), 1.], A_ub=aub, b_ub=np.zeros(n), A_eq=aeq,
                       b_eq=beq, bounds=[(0,None)]*(n+1), method="highs")
    if not solution.success:
        return {**result, "status": INFEASIBLE, "reason": "No nonnegative vertical reaction distribution satisfies global equilibrium under the selected load-sharing model."}
    forces = solution.x[:n]
    force_residual = np.array([0.,0.,forces.sum()-weight])
    moment_residual = sum((np.cross(point-robot["com"], np.array([0.,0.,f])) for point,f in zip(p,forces)), start=np.zeros(3))
    roll_moments = {}
    for side, leg in robot["legs"].items():
        gravity_roll = sum(m.kg*G*(m.position[1]-leg["hip"][1]) for m in distal_masses(robot, side, "hip"))
        contact_roll = sum(f*(point[1]-leg["hip"][1]) for point,f,owner in zip(p,forces,contact["owners"]) if owner==side)
        roll_moments[side] = gravity_roll-contact_roll
    if max(np.linalg.norm(force_residual), np.linalg.norm(moment_residual)) > 1e-6:
        return {**result, "reason": "Reaction solver residual exceeds tolerance."}
    return {"status": FEASIBLE, "reason": "Vertical reactions satisfy global force and moment balance; joint limits and actuator capacity are checked separately." +
            (" Point/line contact has zero area and no robust static tipping margin; dynamic balance is not evaluated." if c["foot_model"]=="Rounded nub" else ""),
            "forces": forces, "force_residual": force_residual, "moment_residual": moment_residual,
            "structural_roll_moments": roll_moments, "rows": torque_rows(robot, contact, forces)}


def packaging(robot):
    c = robot["config"]
    gap = c["pack_gap"]
    issues = []
    for side in ("L","R"):
        if c[f"{side}_thigh"] < c["servo_h"] + gap:
            issues.append(f"{side} thigh shorter than servo z envelope + mounting gap.")
        if c[f"{side}_shin"] < c["servo_h"]/2 + gap:
            issues.append(f"{side} shin shorter than half servo z envelope + mounting gap.")
    width=max(2*c["L_nub_radius"],2*c["R_nub_radius"]) if c["foot_model"]=="Rounded nub" else max(c["L_foot_w"], c["R_foot_w"])
    if c["spacing"] < max(width,c["servo_w"])+gap:
        issues.append("Hip spacing below foot/servo width + gap screen.")
    return issues


def reach_samples(data):
    """Three discrete pelvis-fixed swing endpoints, including both IK branches."""
    robot = build(data)
    c = robot["config"]
    results = []
    for side in ("L", "R"):
        _, base = fk(c[f"{side}_thigh"]/1000, c[f"{side}_shin"]/1000, *robot["legs"][side]["angles"])
        rounded=c['foot_model']=='Rounded nub'
        if rounded:
            base+=rotation(sum(robot['legs'][side]['angles']))@np.array([c[f'{side}_foot_ox'],0,c[f'{side}_foot_oz']])/1000
        for name, dx, dz in [("lift",0,c["lift"]), ("forward",c["reach"],0), ("combined",c["reach"],c["lift"])]:
            sols = nub_ik(c,side,base[0]+dx/1000,base[2]+dz/1000) if rounded else ik(c[f"{side}_thigh"]/1000, c[f"{side}_shin"]/1000, base[0]+dx/1000, base[2]+dz/1000)
            valid = [a for a in sols if within_limits(c,side,a)]
            results.append({"side": side, "sample": name, "reachable": bool(sols), "within limits": bool(valid), "solutions": valid,
                            "x (mm)": base[0]*1000+dx, "z (mm)": base[2]*1000+dz,
                            "target reference":"nub center" if rounded else "two-link endpoint"})
    return results


def height_survey(data, count=61):
    """Sample symmetric flat-foot grounded poses with knee fixed by foot angle.

    Asymmetric configurations need a coupled solve and are explicitly unevaluated.
    This is a sample range, not a proof of continuous feasibility or equilibrium.
    """
    c = effective(data)
    if not c["symmetry"]:
        return None
    heights, hips = [], []
    rounded=c["foot_model"]=="Rounded nub"
    pairs=[(h,k) for h in np.linspace(c['L_hip_min'],c['L_hip_max'],13)
           for k in np.linspace(c['L_knee_min'],c['L_knee_max'],13)] if rounded else [
               (h,-c['pitch']-c['L_foot_angle']-h) for h in np.linspace(c['L_hip_min'],c['L_hip_max'],count)]
    for hip,knee in pairs:
        if not c["L_knee_min"] <= knee <= c["L_knee_max"]:
            continue
        candidate = dict(c, L_hip=float(hip), L_knee=float(knee), pelvis_z=200.)
        r = build(candidate)
        delta = r["legs"]["L"]["sole"][:,2].min()*1000
        candidate["pelvis_z"] -= delta
        if not -100 <= candidate["pelvis_z"] <= 500:
            continue
        r = build(candidate)
        if not contacts(r)["errors"] and r["vertices"][:,2].min() >= -GROUND_TOL:
            heights.append(r["height"]*1000)
            hips.append(candidate["pelvis_z"])
    if not heights:
        return {"count": 0}
    return {"count": len(heights), "min": min(heights), "max": max(heights), "hip_min": min(hips), "hip_max": max(hips),
            "protocol":"13 × 13 hip/knee grid with spherical contact" if rounded else "61 hip samples with flat-sole orientation constraint"}


def evaluate(data):
    robot = build(data)
    c = robot["config"]
    contact = contacts(robot)
    ground = stance(robot, contact)
    limits = all(within_limits(c,s,l["angles"]) for s,l in robot["legs"].items())
    singular = [s for s,l in robot["legs"].items() if abs(np.sin(l["angles"][1])) < 0.05]
    rows = ground["rows"] if c["mode"] == "Grounded pose" else torque_rows(robot)
    checks = {
        "Current overall geometric height": FEASIBLE if c["min_height"] <= robot["height"]*1000 <= c["max_height"] else INFEASIBLE,
        "Total mass": FEASIBLE if robot["mass"]*1000 <= c["max_mass"] else INFEASIBLE,
        "Entered payload target": FEASIBLE if c["payload_mass"] >= c["payload_target"] else INFEASIBLE,
        "Joint limits": FEASIBLE if limits else INFEASIBLE,
        "Approximate packaging": INFEASIBLE if packaging(robot) else FEASIBLE,
        "Static ground equilibrium": ground["status"],
        "Screening torque allowance": (FEASIBLE if max(r["utilization"] for r in rows) <= 1 else INFEASIBLE) if rows else NOT_EVALUATED,
        "Swing endpoint reach and joint limits": FEASIBLE if all(s["within limits"] for s in reach_samples(c)) else INFEASIBLE,
    }
    if c["foot_model"]=="Rounded nub":
        checks["Dynamic walking balance"]=NOT_EVALUATED
    if INFEASIBLE in checks.values():
        status = INFEASIBLE
    elif NOT_EVALUATED in checks.values():
        status = NOT_EVALUATED
    else:
        status = FEASIBLE
    return {"robot": robot, "contact": contact, "stance": ground, "rows": rows,
            "checks": checks, "singular": singular, "status": status}
