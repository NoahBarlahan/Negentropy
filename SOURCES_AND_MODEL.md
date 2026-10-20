# Sources, assumptions and engineering model

**Version-2 update:** Rounded nub is now the default foot model. Spheres produce one point contact per foot, with static reactions calculated only when force/moment equilibrium permits. Point/line contacts have no finite-area tipping margin. Rounded-foot recommendations use supported-pelvis gravity screening and do not certify stance or dynamic stability. The new animation is a prescribed kinematic sequence with spherical rolling and IK; it is not a dynamics simulation. See [Rounded feet and walking preview](WALKING_PREVIEW.md) for the current definitions. The rectangular-sole definitions below apply to the retained Flat sole option and legacy files.

Primary sources checked for this implementation on 2026-10-15. The implementation's frame convention and numerical examples below are defined here, rather than copied from a different robot's convention.

## Sourced hardware defaults

| Quantity | Default / reference | Source and treatment |
|---|---|---|
| Servo | TowerPro MG996R | User's four-servo architecture; “mggr996” interpreted as MG996R |
| Each servo mass | 55 g | [TowerPro MG996R specification](https://towerpro.com.tw/product/mg996R/); excludes any unmeasured extra assembly mass |
| Body envelope | 40.7 × 19.7 × 42.9 mm | [TowerPro MG996R specification](https://towerpro.com.tw/product/mg996R/); orientation/mounting offsets are assumptions, not a full horn/lug/cable envelope |
| Stall reference at 4.8 V | 9.4 kgf·cm → 0.9218251 N·m | [TowerPro MG996R specification](https://towerpro.com.tw/product/mg996R/); site's “kg/cm” interpreted as customary servo kgf·cm |
| Stall reference at 6 V | 11 kgf·cm → 1.0787315 N·m | [TowerPro MG996R specification](https://towerpro.com.tw/product/mg996R/); UI initial reference |
| Supply range | 4.8–6.6 V | [TowerPro MG996R specification](https://towerpro.com.tw/product/mg996R/); no stall interpolation/extrapolation implemented |

Conversion uses 1 kgf·cm = 9.80665 × 0.01 N·m. The manufacturer page also has a differently labeled dimensional table, so the headline body dimensions are only an approximate envelope. Measure the actual units, horn travel and mounting datum in CAD. No continuous torque or verified joint-travel rating is asserted.

The working allowance starts at 25% of the entered stall reference, an **unvalidated assumption**. The 1× screening multiplier is separate and is **not a validated safety factor**. Neither predicts heating, duty cycle, gearbox life or walking loads. The supply/reference warning does not replace measurement.

## User requirements versus assumptions

The initial requested overall-height band is 101.6–203.2 mm (4–8 inches). These are requirements, not geometric degrees of freedom. All link dimensions, feet, platform, mounting offsets, pose/limits, printed/electronics/battery/hardware/payload masses and local mass-center positions are assumed defaults. The example JSON states this provenance. Edited values are user entries; the tool cannot verify that they were measured.

Manual masses and local CoMs remain fixed when dimensions change. No automatic length/mass or density scaling is enabled. Right-side values are derived from the left when symmetry is enabled. The saved configuration preserves stored independent right values so turning symmetry off restores them; those stored values do not contribute while symmetry is on.

## Frames and geometry

World coordinates: x forward, y robot-left, z upward, ground z=0. Pelvis origin: midpoint between hip axes. Both leg planes are parallel to x/z. Pitch angles are about −y: a positive angle swings a downward link forward.

```text
                 z ↑
        platform [======]
                   H hip      x → forward
                    \  L1
                     K knee
                      \  L2
                       E two-link endpoint / foot reference
                 [rigid foot]
    ─────────────────────────── ground
    y is robot-left; axes are parallel to −y
```

At zero hip/knee angles and pelvis pitch, links extend along −z. Hip angle is relative to the pelvis. Knee angle is relative to the thigh. Thigh length measures hip axis to knee axis. Shin length measures knee axis to E. Foot top-center is E plus its x/z attachment offset rotated by the shin frame. Foot orientation = pelvis pitch + hip + knee + fixed attachment angle. Sole lies one foot thickness below the top-center in the foot frame. The foot has no independent leveling degree of freedom.

Platform origin is its box center, with position measured in pelvis coordinates and pelvis orientation. Hip servo case orientation follows pelvis; knee case follows thigh. Their visual offsets are independent of the editable lumped CoM offsets.

Mass carrier origins: pelvis at midpoint; hip carrier at its hip with pelvis orientation; thigh at hip with thigh orientation; shin at knee with shin orientation; foot at offset top-center with foot orientation; platform at box center. Each mass appears once. Hip servo masses are upstream of both actuated leg subtrees. Knee servo masses are downstream of hip but upstream of knee, because their cases ride on thighs.

The model uses meters, kilograms, radians and N·m internally. UI/configuration use mm, grams, degrees and N·m.

## FK, IK and height

In the pelvis frame relative to the relevant hip:

```text
xE = L1 sin(q1) + L2 sin(q1+q2)
zE = −L1 cos(q1) − L2 cos(q1+q2)
c2 = (xE² + zE² − L1² − L2²)/(2 L1 L2)
q2 = ± acos(c2)
q1 = atan2(xE, −zE) − atan2(L2 sin(q2), L1 + L2 cos(q2))
```

This is an analytic two-link solution based on the law of cosines, with both branches and an annular reach test. The approach and multiple-solution interpretation are supported by [Modern Robotics: Inverse Kinematics of Open Chains](https://modernrobotics.northwestern.edu/nu-gm-book-resource/inverse-kinematics-of-open-chains/). This implementation adapts its axes to downward links.

An out-of-annulus target returns no solution. IK buttons reject branches outside entered joint limits without changing other inputs. Near `|sin(knee)| < 0.05` the app warns about singularity. At the origin with equal links folded, infinitely many hip solutions exist; a representative hip=0, knee=±180° is returned and subjected to the limits.

Overall geometric height is max(z)−min(z) across rendered boxes and joint/link references. Hip height is world pelvis z. A lumped payload has no size envelope, and link thickness/horns are not included. Pelvis-fixed spans do not claim a grounded standing height.

The optional achievable-height survey samples 61 symmetric hip angles and imposes `knee = −pelvis_pitch − attachment_angle − hip` to keep feet flat. It derives the corresponding pelvis z from ground contact and rejects invalid limits/penetration. It reports only a sampled geometric range, not an exact continuous range or statically feasible range. Coupled asymmetric height exploration is not evaluated. Desired lift/reach are three discrete endpoint displacements in the pelvis x/z frame, before the attachment offset.

## CoM and support

For each mass, `p_world = body_origin + body_rotation @ p_local`; whole-robot CoM is `Σ(m p_world)/Σm`. The support region is the convex hull of actual selected lowest sole vertices on level ground. A flat sole contributes four corners, a tilted sole its lowest edge (two corners), and a general corner would contribute one point. This pitch-only geometry normally gives flat or edge contacts. The 0.05 mm ground tolerance permits small coordinate rounding; it is not compliance. Lowest-corner selection within each sole uses a much tighter 1e−9 m tie tolerance, so ground tolerance does not flatten tilted feet.

Selected feet must contact ground; other feet must be clear. Foot or other envelope penetration invalidates grounded equilibrium. No automatic adjustment occurs. A line/point hull has no finite-area tipping margin and its stance torque is marked **not evaluated**. Other invalid contact geometry is **infeasible**. The geometric tipping margin is the minimum signed distance from the CoM projection to the convex hull edges: positive inside, zero on its boundary, negative outside. Zero is marginal, not a robust balance reserve.

The unilateral force/contact and whole-body balance approach is supported by [MIT Underactuated Robotics: Highly-articulated Legged Robots](https://underactuated.mit.edu/humanoids.html). Here ground is level, bodies are rigid, conditions are quasi-static and friction is adequate. Front/top checks matter: a side view alone does not establish lateral balance. Four pitch joints cannot actively shift weight laterally. Static support does not establish dynamic walking stability.

## Static loads and sharing

Suspended mode assumes a fixture holds pelvis position and orientation. The signed gravity moment about −y at joint j is:

```text
tau_gravity,j = Σdistal mi g (xi − xj)
```

For example, an isolated 0.2 kg mass 0.04 m forward of a joint demands `0.2 × 9.80665 × 0.04 = 0.0784532 N·m`. The app shows a current-input worked example with all left-hip distal contributions.

Grounded mode solves nonnegative vertical reactions at selected contact vertices:

```text
ΣFz = M g
Σxk Fz,k = M g xCoM
Σyk Fz,k = M g yCoM
Fz,k ≥ 0
```

Horizontal reactions and internal horizontal prestress are set to zero. Among distributions satisfying these constraints, a linear program minimizes the largest vertex force. This explicitly defines the double-support sharing assumption; it does not predict pressure/compliance or establish that all other distributions produce the same joint load. It is a feasible gravity-only reaction distribution, not a worst-case torque bound. Force and all three moment residuals are checked and displayed; infeasible/unimplemented cases have no stance torque table.

For each distal subtree:

```text
tau_actuator,j = Σdistal mi g (xi − xj) − Σleg_contact Fz,k (xk − xj)
```

Whole-body weight is included when solving reactions. It is not added a second time to the distal gravity sum. A stance estimate based on leg weight alone would miss body loading. The case on the thigh is excluded from the knee's distal mass sum. Fixed-axis joints may transmit structural moments in their blocked roll/yaw directions; the app reports required hip roll reaction moments. Bracket/bearing strength and actuator side loading are not assessed. These structural reactions do not create a hip-roll degree of freedom or powered lateral control.

Static demand is absolute actuator torque. Screening demand is static demand × user multiplier. Utilization is screening demand / user working allowance. Ground equilibrium may pass while torque, joint limits, or other requirements fail; these are separate checks.

## Boundaries

Not implemented: dynamics, stable gait generation, foot compliance, general frictional prestress, contact transitions, motor heating or continuous ratings, strength/buckling/fatigue, PLA material/process validation, detailed interference or Bambu A1 print checks. Packaging is a crude lower-bound screen: thigh ≥ servo z size + gap, shin ≥ half servo z size + gap, hip spacing ≥ largest foot/servo y size + gap. It ignores rotated envelope intersections and may reject some valid alternative mounting arrangements. Treat its result as approximate, not clearance verification.
