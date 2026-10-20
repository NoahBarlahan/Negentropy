# Rounded feet and stepping preview

The default **Rounded nub** is a rigid sphere at the two-link endpoint plus the editable shin-frame attachment offset. Radius sets the lowest contact point. Sphere orientation does not affect its shape, so rectangular dimensions and fixed sole angle are inactive. No ankle joint/motor is added. Radius starts at an **assumed 6 mm**. Nub/attachment mass retains the editable assumed 22 g; measure it in CAD. The mass frame is at the sphere center, with shin orientation and default local CoM at the center. Manual masses/CoMs never rescale with geometry.

## Contact and calculations

- Ground point = sphere center − radius along world z. No rubber contact patch is assumed.
- One or two active nubs form a point or line, not a finite support area. The overview reports CoM projection distance from that point/segment, not a dynamic stability score.
- Static vertical reactions are available only if nonnegative forces satisfy whole-body force/moment equilibrium. Otherwise stance torque is unavailable. Passing this test still gives zero robust support-area margin.
- Dynamic walking balance is always **not evaluated**. A static-equilibrium failure does not prove that dynamically controlled point-foot walking is impossible.
- Rounded-foot good guesses require geometry, mass, payload, limits, reach, approximate packaging, contact clearance and **externally supported-pelvis gravity torque** checks. Their default filters omit static Stance. The torque objective screens the supported base pose and six supported-pelvis swing endpoints; it excludes actual grounded stance, accelerations and impacts. A passing candidate is not a validated walking design.
- Flat soles retain their static ground-reaction torque model. Version-1 JSON imports retain Flat sole and original mass coordinates. Switch explicitly to Rounded nub when adapting an older file; version-2 exports include the model and radii.

## Build and play

Open **Walking preview**, press **Build walking preview**, then Play. Pause or scrub the Plotly frame slider. The preview uses the same transformed model as the other calculations.

| Input | Meaning |
|---|---|
| Forward reach, under Requirements | Body advance per alternating step; each foot repeats after twice that distance (one stride) |
| Desired foot-reference lift | Maximum nub-bottom swing clearance |
| Full cycle duration | Timing only; does not simulate servo response |
| Stance fraction | Fraction of each leg cycle on ground; legs are half a cycle apart, so values above 0.5 create double-contact intervals |
| Cycles | Repetitions; 40 samples per cycle plus the final endpoint |

Pelvis height, orientation and lateral position are prescribed by the current inputs. Pelvis x advances at constant speed. IK generates new joint angles; your saved pose is not changed. No lateral recovery motion is invented. Ground penetration, IK residuals and joint limits are checked at each displayed sample, not continuously between samples.

### Stance rolling

A sphere rigidly attached to a rotating shin cannot have both a fixed center and no slip. The sagittal rolling constraint is:

```text
center_z = radius
center_x + radius × (shin_pitch − touchdown_shin_pitch) = touchdown_center_x
```

Pitch is positive about world −y. The nonlinear constraints are solved together with leg FK. The contact location can move as the nub rolls; friction capacity and actual dynamic contact forces are not calculated.

### Swing motion

The center moves from its rolling lift-off position to its next touchdown. A quintic Hermite horizontal path matches intended center velocities at both ends. Vertical motion is `radius + lift × sin²(pi × swing_fraction)`, with zero vertical endpoint velocity. IK includes shin-frame attachment offsets and selects a continuous in-limit knee branch. This is prescribed motion, not a dynamics or impact solution.

A failed sample stops generation with a reason and the last valid frame. No clamped angles, stretched links or silently shortened steps are substituted. Try smaller reach/lift or a different explicit pelvis height and regenerate. Changes to inputs/timing hide stale animation. Unit switching converts the same motion.

CSV exports time, model joint angles, body position, nub clearance, phases and CoM coordinates. **These are not calibrated servo commands.** Peak joint speed is a sampled kinematic value, not proof of actuator capability.

**Download interactive walking preview** saves a self-contained HTML animation with its plotting library embedded, so playback and scrubbing work offline. The dashboard uses the same embedded animation document because its native static-chart renderer does not register animation frames.

## Real-world limits

The preview shows a proposed reachable stepping sequence. It does not establish that MG996R servos can execute it under load or that the robot balances. There is no IMU feedback, foot-placement controller, inverse dynamics, torque-speed/thermal model, impact response, friction-limit model, lateral recovery or full self-collision check. A real robot could fall during a clean preview. A dynamics/control study and supported hardware tests are needed to predict actual walking.

The existing [sources/model notes](SOURCES_AND_MODEL.md) cover FK/IK and contact equilibrium. For the distinction between contact geometry and dynamic locomotion, see [MIT Underactuated Robotics](https://underactuated.mit.edu/humanoids.html).
