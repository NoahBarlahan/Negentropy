# Biped Studio — four MG996R servos

**Rounded-foot update:** the default now uses spherical nubs with no ankles. Select **Foot contact model** in the sidebar. In **Walking preview**, press **Build walking preview**, then Play; use the frame slider to inspect motion. This is a **kinematic stepping preview**, not a prediction of real-world balance. [Contact assumptions, motion controls and equations](WALKING_PREVIEW.md).

A local Streamlit dashboard for a two-leg robot with hip pitch and knee pitch on each leg, rigid feet, a flat upper platform, and editable lumped masses. The dashboard app and modules live directly in the Negentropy project root.

Use **Length units** in the sidebar to switch between millimeters and inches. Dimension inputs, derived lengths, plot coordinates and comparison CSVs follow the selection. Switching units preserves the physical design; inch entries convert with exactly 25.4 mm/in. JSON configurations retain their existing millimeter schema, so older saved designs still load in either view. Mass, angles, force and torque units stay unchanged; source specifications and SI equation examples keep their explicitly labeled reference units.

## Calculate a good starting point

After editing requirements, geometry, pose or masses, press **Calculate & apply a good guess**. It searches the Dimension comparison ranges and sample count, adding current left-leg lengths inside the bounds. It tests common lengths on both legs and derives pelvis height from the lowest foot point. Rounded-foot suggestions screen geometry/reach/contact clearance and supported-pelvis gravity loads; static Stance is not required and dynamic balance is not evaluated. Flat soles also require their static stance check. It applies only the four lengths, pelvis z and grounded mode; requirements, angles and masses/CoMs stay fixed.

The default priority is **Height near middle of required band**. For a 4–8 inch height band, the preferred height is 6 inches. The search first minimizes absolute error from that height; ties favor lower worst screening torque utilization, then a smaller total change to the four leg lengths. You can instead prioritize **Lowest screening torque** or **Smallest length change**. These are ordered comparisons, with no hidden score weights. The midpoint is a preference inside your requirements, not a second independent geometric constraint.

Open **How the calculated good guess was derived** above the robot to see search bounds, rejected constraints, before/after dimensions, the chosen candidate's metrics and up to five passing alternatives. A no-solution search leaves inputs unchanged. For example, length changes cannot correct a tilted sole at fixed angles, reduce a manually entered mass, or add a missing payload. Expand bounds or revise the failed assumptions instead of expecting the calculator to relax a requirement.

**Undo last good guess** restores only the fields the suggestion controlled, preserving later edits to masses and requirements. Undo refuses to overwrite subsequent manual changes to those controlled fields. The explanation remains a snapshot of its calculation; editing inputs marks it stale until you calculate again. Unit switching does not invalidate the design.

Every candidate uses the existing FK, CoM, contact and gravity model. Rounded-foot torque screening uses the externally supported base pose and six supported-pelvis swing endpoints; flat soles also include grounded stance. Manual masses/CoMs do not scale with dimensions. Update them from CAD and calculate again. This finite search does not validate a gait.

## Run on Windows from VS Code

Open `C:\Users\noah\Desktop\Personal Engineering\Code Projects\Negentropy` in VS Code. In a PowerShell terminal:

```powershell
py -3.12 -m venv .dashboard-venv
.\.dashboard-venv\Scripts\python.exe -m pip install -r dashboard-requirements.txt
.\.dashboard-venv\Scripts\python.exe -m streamlit run app.py --server.port 8502
```

Open the local URL printed by Streamlit, here `http://localhost:8502`. Stop with Ctrl+C. Activation is unnecessary, so PowerShell execution-policy settings do not need to change. In VS Code, select `.dashboard-venv\Scripts\python.exe` as the Python interpreter if you want editor completion.

If this copy already has its virtual environment and dependencies installed, run the third command; otherwise run all three setup commands once. No hardware connection, credentials, ROS, CAD integration or motor control is required.

Run the engineering and UI tests:

```powershell
.\.dashboard-venv\Scripts\python.exe -m pytest dashboard_tests -q
```

`dashboard-requirements.txt` specifies compatible major versions. `dashboard-requirements-tested.txt` records the exact direct-package versions used for validation. The root project's requirements remain unchanged.

## First walkthrough

1. Start from 65 mm thigh/shin lengths, measured between joint/reference locations. The initial −20° hip, +40° knee and assumed 6 mm spherical nubs touch ground at approximately 128.16 mm hip height; overall envelope height is approximately 157.16 mm. Rounded contacts have no sole angle. Every non-sourced value is an assumption.
2. Change the thigh with its slider, then type its numeric value. Both controls stay synchronized. Feet can leave or penetrate the ground: no automatic leveling or repositioning occurs. Press **Place lowest selected sole on ground** to explicitly translate only pelvis z. This does not guarantee that the other selected foot contacts correctly.
3. Try changing the knee by 10°. Inspect the foot pitch and lowest edge in the side view. A tilted sole does not contribute its whole rectangle to support. Restore defaults before comparing other effects.
4. In **Mass distribution**, enter the CAD masses and local mass centers of your printed parts. The hip cases are on the pelvis; knee cases are on the thighs. Changing length does not automatically move a manually entered CoM or change its mass. The same distinction applies to visual servo-center offsets versus servo mass centers.
5. Add a payload and move its local x or y coordinate. Observe the whole-robot CoM, projection, support margin, and stance reaction forces. Use the front/top views to inspect lateral geometry. No powered hip roll or ankle is present.
6. Switch to **Pelvis fixed** to explore unsupported foot motion or IK. The external fixture supports the pelvis in every direction. The torque table now represents suspended legs; no free-standing equilibrium is claimed.
7. Set your height, reach, mass and payload requirements. In **Actuators**, replace the unvalidated working-torque basis with bench evidence when available. A voltage edit never silently changes the stall-torque reference; the UI flags an unsourced voltage or mismatch.
8. Run the length sweep. Inspect per-criterion statuses before reading ranks. Save the comparison CSV **and source configuration** to keep a reproducible record. Use **Save / load configuration** in the sidebar to carry measurements between CAD sessions.

## What the files do

| File | Responsibility |
|---|---|
| `config.py` | Field definitions, defaults, bounds, symmetry and strict versioned JSON |
| `model.py` | SI FK/IK, transforms, CoM, actual sole contacts, convex support margin, static reactions and joint moments |
| `plots.py` | 3D and orthographic views from the same calculated geometry |
| `sweep.py` | Explicit finite-grid comparison and ranking protocol |
| `app.py` | Streamlit controls, explanations, tables and exports |
| `example_config.json` | Clearly documented initial assumptions; import through the sidebar |
| `SOURCES_AND_MODEL.md` | Coordinate frames, equations, provenance and scope |
| `VALIDATION.md` | Executed checks and practical limits |

## Reading statuses

- **feasible under this model** means the named check passed with the assumptions displayed. It does not certify the robot or predict walking stability.
- **infeasible** means a named geometric, requirement, equilibrium or assumed torque constraint failed.
- **not evaluated** means the needed case/model is unavailable, including grounded equilibrium in externally supported mode and stance loading for a degenerate line/point support region.

The overall badge covers the checks listed below the visualization. Dynamic stability, bracket strength, collision clearance, power delivery and continuous/thermal servo performance are always outside this model. Torque feasibility relies on your working-torque assumption.

## Comparison protocol

Both legs receive each candidate thigh/shin pair. The current angles, fixed attachment geometry, and manual masses/local CoMs are preserved. The comparison explicitly translates the candidate pelvis in z until the lowest sole reaches ground, then requires the selected contacts to agree. This translation is recorded by the candidate hip height and never applied to the live model.

Additional endpoints are evaluated separately per limb with the pelvis externally supported: current endpoint + lift, + forward reach, and + both. These displacements are in the pelvis x/z frame. Each reachable endpoint searches both knee branches within the entered joint limits, choosing the branch minimizing peak limb torque. These are six discrete endpoint tests, not an interpolated path, collision-free trajectory, or gait. Swing foot-ground clearance is not a comparison criterion; stance contact geometry is.

The objective is the worst multiplied torque across the candidate stance and those swing samples. Only candidates passing all selected criteria and having an evaluated objective receive a rank. You may deselect criteria, in which case the status explicitly applies only to the selected checks; all individual results remain visible. A finite grid cannot prove global optimality. The approximate packaging thresholds prevent trivially short links from winning without a warning, but cannot establish CAD fit.

## Simple design goals

Start at the top of the dashboard: enter overall robot height, payload mass,
step length and foot clearance, then press **Optimize my design**. An optional
weight budget is collapsed below the four main controls. Length inputs follow
the mm/in selector. Detailed geometry, mass locations and servo assumptions are
in **Advanced inputs**; legacy placement/contact/search controls are collapsed
in the sidebar's **Advanced settings**.

The new goal search tries 18 combinations: thigh shares of 45%, 50%, 55% and
knee bends of 25, 35, 45, 55, 65, 75 degrees. It solves total link length by
bisection against the full rendered envelope height, grounds the rounded feet,
and chooses the hip angle to place the two-link endpoint under the hip before
attachment offsets. It adds the requested payload to both the actual mass model
and the payload requirement. Knee-servo z follows thigh length and printed-link
CoM z is estimated at half-link length; component masses and remaining offsets
stay fixed. Body layout, spherical radius and hardware ratings remain constraints.

Candidates must pass the existing geometry, packaging, mass, endpoint reach and
suspended-gravity checks plus a full one-cycle kinematic preview at 17 time
samples. At each sampled pose the torque screen uses the absolute distal gravity
moment plus a full-robot-weight vertical reaction moment on each stance leg,
multiplied by the configured screening factor. This deliberately conservative
load case is not a solved ground reaction or dynamic equilibrium. The passing
candidate with the smallest peak screening demand is selected; ties favor equal
links, then less knee bend. This is a bounded search, not a global optimum.

The applied result includes a 1 mm height tolerance, symmetric pose, derived
lengths/hip height, payload, goals and estimated link mass locations. It can be
undone until edited. No passing candidate leaves the entire previous model
unchanged and reports failing constraints. Configuration JSON remains version 2;
goal results are saved as ordinary model parameters. Explanations and search
history are session-only. Dynamic balance, servo thermal capability, friction,
acceleration and structural sizing still need validation; animation is kinematic.

## SolidWorks / Notepad parameter export

In **CAD / SolidWorks**, finalize current dimensions and download the equations
text or CAD ZIP. The simple rounded-foot export contains just 10 variables:
HipSpacing, WaistLength, WaistWidth, WaistHeight, left/right ThighLength,
ShinLength and FootRadius. It omits poses, servo envelopes, offsets, overall
height, mounting clearance and redundant diameter variables.

Optional **Waist & printed link dimensions** adds ThighWidth, ThighThickness,
ShinWidth and ShinThickness when you enter them. Cross-sections apply to both
legs. Blank fields are omitted. Waist dimensions default to the existing platform
length, width and thickness; override them for a CAD enclosure/body. CAD-only
dimensions do not alter the mechanics model, masses or optimized geometry, and
are not structural sizing recommendations. Link lengths remain joint-reference
distances; with zero foot offset, ShinLength ends at the sphere center.

Equations always use explicit mm. Use `="ThighLength_L"` on your thigh sketch,
`="ThighWidth"` on its width when supplied, and `="FootRadius_L"` on a nub radius.
Use `=2*"FootRadius_L"` for a diameter. Existing CAD references to removed export
variables must be updated to this smaller parameter set.

Import through **Tools > Equations > Import**, enable **Link to File** when
available, and connect sketch/feature dimensions to the variables. Link the same
file in each part that uses them. Replace the linked text at the same path and
rebuild to apply revised exports. Downloads remain frozen until you finalize
again. The ZIP includes a parameter map, setup notes, dashboard configuration
and separate CAD dimensions JSON. Dashboard restore JSON does not contain the
CAD-only dimensions; re-enter them from cad_dimensions.json for another session.

Actual offsets are retained in the source config and setup notes rather than
cluttering the equations. Mounts, fasteners and print fits still need hardware
measurements. Actual SolidWorks import/rebuild remains untested here.

Reference: [SOLIDWORKS import/link instructions](https://help.solidworks.com/2023/english/SolidWorks/sldworks/t_EQN_Importing_Equations.htm).



You can also double-click `run_dashboard.bat` from the project root. It uses the dashboard's dedicated interpreter and runs the root `app.py` on port 8502. Use `python -m pip` and `python -m streamlit` through that interpreter; no environment activation is needed.

