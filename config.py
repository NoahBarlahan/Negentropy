"""Editable assumptions and strict, versioned configuration I/O (UI units)."""
from copy import deepcopy
from dataclasses import dataclass
import json
import math


@dataclass(frozen=True)
class Field:
    key: str
    label: str
    group: str
    default: float
    low: float
    high: float
    step: float
    explanation: str
    source: str = "Assumed default; replace with your CAD or measurement."


FIELDS = []


def add(key, label, group, default, low, high, step, explanation, source=None):
    FIELDS.append(Field(key, label, group, float(default), float(low), float(high),
                        float(step), explanation, source or Field.__dataclass_fields__["source"].default))


for key, label, default, low, high in [
    ("spacing", "Hip spacing (mm)", 65, 10, 200),
    ("platform_l", "Platform length (mm)", 85, 5, 250),
    ("platform_w", "Platform width (mm)", 100, 5, 250),
    ("platform_t", "Platform thickness (mm)", 4, 1, 30),
    ("platform_x", "Platform center x from pelvis (mm)", 0, -150, 150),
    ("platform_z", "Platform center z from pelvis (mm)", 27, -100, 150),
    ("servo_l", "Servo envelope x (mm)", 40.7, 5, 100),
    ("servo_w", "Servo envelope y (mm)", 19.7, 5, 100),
    ("servo_h", "Servo envelope z (mm)", 42.9, 5, 100),
    ("pack_gap", "Approximate mounting gap (mm)", 5, 0, 40),
]:
    add(key, label, "Geometry", default, low, high, 0.1,
        "Box dimensions are along local x/y/z; positions are relative to the hip midpoint in the pelvis frame. "
        "Affects rendering, overall height and approximate packaging. Envelope spacing is a screening check, not CAD clearance.",
        "TowerPro MG996R specification: body 40.7 × 19.7 × 42.9 mm; envelope orientation is assumed."
        if key.startswith("servo_") else None)

for key, label, default, low, high in [
    ("pelvis_x", "Pelvis world x (mm)", 0, -300, 300),
    ("pelvis_y", "Pelvis world y (mm)", 0, -200, 200),
    ("pelvis_z", "Hip midpoint world z (mm)", 128.1600407, -100, 500),
    ("pitch", "Pelvis pitch (deg)", 0, -90, 90),
]:
    add(key, label, "Pose", default, low, high, 0.1,
        "World origin is on level ground; x forward, y left, z up. Positive pitch tips a downward link forward. "
        "Moves the entire robot and affects contacts, height and moments. No automatic ground snapping.")

for side in ("L", "R"):
    for key, label, default, low, high in [
        ("thigh", "Thigh: hip axis → knee axis (mm)", 65, 5, 200),
        ("shin", "Shin: knee axis → foot reference (mm)", 65, 5, 200),
        ("nub_radius", "Rounded nub radius (mm)", 6, 1, 30),
        ("foot_l", "Foot length (mm)", 75, 5, 180),
        ("foot_w", "Foot width (mm)", 42, 5, 120),
        ("foot_t", "Foot thickness (mm)", 6, 1, 25),
        ("foot_ox", "Attachment offset x in shin (mm)", 0, -80, 80),
        ("foot_oz", "Attachment offset z in shin (mm)", 0, -80, 80),
        ("foot_angle", "Fixed sole angle from shin frame (deg)", -20, -180, 180),
    ]:
        add(f"{side}_{key}", label, f"{side} geometry", default, low, high, 0.1,
            "For a rounded nub the endpoint plus attachment offset defines the sphere center; radius sets ground clearance. Sole angle and rectangular dimensions are unused. "
            "Links point along local −z at zero angle. For a flat sole, foot reference is the top-center before the shin-frame "
            "attachment offset; sole is one thickness below it in the foot frame. Angle rotates the rigid foot with the shin. "
            "Affects endpoint reach, foot tilt, height, contacts and mass transforms.")
    for joint, angle, lo, hi in [("hip", -20, -75, 75), ("knee", 40, -120, 120)]:
        for suffix, label, default in [("", "angle", angle), ("_min", "lower limit", lo), ("_max", "upper limit", hi)]:
            add(f"{side}_{joint}{suffix}", f"{joint.title()} {label} (deg)", f"{side} pose", default, -180, 180, 0.1,
                "Hip is relative to pelvis; knee is relative to thigh. Positive angle swings a −z link toward +x. "
                "Limits are mechanical-angle assumptions, not servo pulse calibration or verified MG996R travel. "
                "Affects FK/IK, singularities, reach and torque.")
    for axis, default in [("x", 0), ("z", -122.1600407)]:
        add(f"{side}_target_{axis}", f"IK foot reference {axis} in pelvis frame (mm)", f"{side} pose", default, -400, 400, 0.1,
            "Target is the two-link endpoint, before the foot attachment offset; measured from this hip in pelvis x/z. "
            "IK changes only this leg's joint angles when you press Apply. Foot orientation is a consequence.")
    for joint in ("hip", "knee"):
        for axis, default in [("x", 0), ("y", 0), ("z", -10)]:
            add(f"{side}_{joint}_case_{axis}", f"{joint.title()} case center offset {axis} (mm)", f"{side} geometry", default, -80, 80, 0.1,
                "Visual case center relative to joint, in its carrier frame: hip case on pelvis; knee case on thigh. "
                "Mass centers are entered separately. Affects the visible envelope and total geometric height.")

# Each mass is counted once. Local xyz entries stay fixed when geometry changes.
MASS_ITEMS = []
for side in ("L", "R"):
    MASS_ITEMS.extend([
        (f"{side}_hip_servo", f"{side} hip servo", f"{side}_hip", 55, (0, 0, -10)),
        (f"{side}_knee_servo", f"{side} knee servo", f"{side}_thigh", 55, (0, 0, -65)),
        (f"{side}_thigh_print", f"{side} printed thigh", f"{side}_thigh", 18, (0, 0, -32.5)),
        (f"{side}_shin_print", f"{side} printed shin", f"{side}_shin", 16, (0, 0, -32.5)),
        (f"{side}_foot", f"{side} foot / nub assembly", f"{side}_foot", 22, (0, 0, 0)),
    ])
MASS_ITEMS.extend([
    ("platform", "Platform", "platform", 30, (0, 0, 0)),
    ("electronics", "ESP32 / electronics", "pelvis", 15, (0, 0, 35)),
    ("battery", "Battery", "pelvis", 45, (0, 0, 15)),
    ("hardware", "Upper hardware", "pelvis", 15, (0, 0, 0)),
    ("payload", "Payload", "pelvis", 0, (0, 0, 45)),
])
for key, label, body, mass, pos in MASS_ITEMS:
    add(f"{key}_mass", f"{label} mass (g)", "Mass", mass, 0, 2000, 0.1,
        f"One lumped mass carried by {body}. Counted exactly once in CoM and gravity moments. "
        "Changing geometry never scales manually entered mass.",
        "TowerPro MG996R specification: 55 g per servo; actual units/horns may differ."
        if "servo" in key else None)
    for axis, val in zip("xyz", pos):
        add(f"{key}_com_{axis}", f"Local CoM {axis} (mm)", "Mass", val, -300, 300, 0.1,
            f"Mass center relative to the {body} frame origin, along its local {axis}. "
            "Hip carrier is at that hip with pelvis orientation; thigh at hip; shin at knee; foot at offset top-center; "
            "platform at its box center. Affects CoM and torque. This is independent of the visual servo center.")

for key, label, default, low, high, step, detail in [
    ("voltage", "Servo supply (V)", 6, 4.8, 6.6, 0.1, "Operating assumption. Published stall values exist at 4.8 and 6.0 V only. No interpolation is assumed."),
    ("stall", "Reference stall torque (N·m)", 1.0787315, 0.01, 3, 0.001, "11 kgf·cm × 0.0980665 = 1.0787315 N·m at 6 V. At 4.8 V: 9.4 kgf·cm = 0.9218251 N·m. Editable reference; never continuous torque."),
    ("derating", "Unvalidated working / stall fraction", 0.25, 0.01, 1, 0.01, "Allowable torque = reference stall × fraction. Unvalidated engineering assumption until bench tested; no thermal model."),
    ("multiplier", "Screening load multiplier", 1, 1, 5, 0.05, "Multiplies static demand only. Not a validated safety factor and does not calculate dynamics."),
    ("min_height", "Minimum overall height (mm)", 101.6, 10, 500, 0.1, "Requested lower bound, default 4 inches. Geometry determines actual height; this input does not resize the robot."),
    ("max_height", "Maximum overall height (mm)", 203.2, 10, 500, 0.1, "Requested upper bound, default 8 inches. Actual geometric height includes link/joint references, feet, platform and servo boxes; lumped payload has no envelope."),
    ("lift", "Desired foot-reference lift (mm)", 15, 0, 120, 0.1, "Swing target above current endpoint at fixed pelvis. Both knee branches searched within limits; sampled end targets do not prove a collision-free trajectory."),
    ("reach", "Desired forward foot-reference reach (mm)", 20, 0, 160, 0.1, "Swing target forward of current endpoint at fixed pelvis. Used with lift as a combined third target in comparisons."),
    ("max_mass", "Maximum total mass (g)", 800, 10, 5000, 1, "Requirement applied to sum of all masses, including entered payload."),
    ("payload_target", "Required payload mass (g)", 0, 0, 2000, 1, "Requirement: modeled payload must be at least this value. Does not silently add payload mass."),
]:
    add(key, label, "Actuators" if key in ("voltage", "stall", "derating", "multiplier") else "Requirements",
        default, low, high, step, detail,
        "TowerPro reference at 6 V; editable sourced default." if key == "stall" else None)


def defaults():
    return {"version": 2, "foot_model": "Rounded nub", "symmetry": True, "mode": "Grounded pose", "contacts": "Both",
            "working_basis": "Unvalidated 25% stall assumption; replace after bench characterization.",
            "provenance": "Unedited defaults: see field labels and SOURCES_AND_MODEL.md.",
            **{f.key: f.default for f in FIELDS}}


def validate(data):
    # Preserve the geometry of old rectangular-foot files rather than silently
    # reinterpreting their dimensions, mass coordinates or contact assumptions.
    if isinstance(data,dict) and type(data.get("version")) is int and data["version"]==1:
        old_keys=set(defaults())-{"foot_model","L_nub_radius","R_nub_radius"}
        if set(data)!=old_keys:
            raise ValueError("Legacy configuration fields do not match version 1.")
        data={**data,"version":2,"foot_model":"Flat sole","L_nub_radius":6.,"R_nub_radius":6.}
    if not isinstance(data, dict) or set(data) != set(defaults()):
        raise ValueError("Configuration must contain exactly the version-2 fields; export a fresh template.")
    if type(data["version"]) is not int or data["version"] != 2 or type(data["symmetry"]) is not bool:
        raise ValueError("Invalid version or symmetry flag.")
    if data["foot_model"] not in ("Rounded nub","Flat sole"):
        raise ValueError("Invalid foot model.")
    if data["mode"] not in ("Grounded pose", "Pelvis fixed") or data["contacts"] not in ("Both", "Left", "Right", "None"):
        raise ValueError("Invalid mode/contact selection.")
    for key in ("working_basis", "provenance"):
        if not isinstance(data[key], str) or len(data[key]) > 4000:
            raise ValueError(f"Invalid {key} text.")
    for f in FIELDS:
        val = data[f.key]
        if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val) or not f.low <= val <= f.high:
            raise ValueError(f"{f.key} must be a finite number between {f.low} and {f.high}.")
    for side in ("L", "R"):
        for joint in ("hip", "knee"):
            if data[f"{side}_{joint}_min"] > data[f"{side}_{joint}_max"]:
                raise ValueError(f"{side} {joint} lower limit exceeds upper limit.")
    if data["min_height"] > data["max_height"]:
        raise ValueError("Minimum required height exceeds maximum.")
    if sum(data[f"{m[0]}_mass"] for m in MASS_ITEMS) <= 0:
        raise ValueError("At least one mass must be positive.")
    return deepcopy(data)


def effective(data):
    c = validate(data)
    if c["symmetry"]:
        for key in c:
            if key.startswith("R_"):
                c[key] = c["L_" + key[2:]]
    return c


def dumps(data):
    return json.dumps(validate(data), indent=2, allow_nan=False)


def loads(text):
    if len(text) > 200_000:
        raise ValueError("Configuration exceeds 200 kB.")
    def unique(pairs):
        d = {}
        for k, v in pairs:
            if k in d:
                raise ValueError(f"Duplicate JSON key: {k}")
            d[k] = v
        return d
    try:
        return validate(json.loads(text, object_pairs_hook=unique))
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError(f"Invalid JSON: {exc}") from exc
