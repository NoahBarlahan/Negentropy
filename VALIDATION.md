# Validation record

Playback verification: the embedded self-contained player was observed advancing from Frame 0 to Frame 10 (0.40 s) in the live Negentropy browser. The native Streamlit Plotly renderer drew only a starting pose, so animation now uses a document that registers Plotly frames explicitly. The affected motion/UI tests passed again (15 tests), followed by 10 targeted tests covering HTML frame registration and the actual iframe payload. The offline HTML includes Plotly and all 81 default frames without a CDN dependency.

Rounded-foot/animation update: **55 tests passed in 103.74 seconds**. New checks cover orientation-invariant spherical geometry, actual point contacts, exact point-contact reaction equilibrium, absence of a dynamic-stability claim, qualified rounded-foot recommendations, legacy JSON migration, 81-frame default motion, IK/limits/clearance and rolling residuals, timing versus joint speeds, nonzero attachment offsets, stopping on unreachable motion, inch-mode animation coordinates, Play/Pause frame construction, UI generation and stale-animation hiding. Flat-sole regression cases remain tested with their original assumptions.

Good-guess calculation update: **45 tests passed in 35.51 seconds**. Tests cover all three ordered ranking priorities, exact revalidation of applied geometry, unchanged requirements/masses, no-solution behavior, current-length inclusion, invalid bounds, inch-mode button application, immutable calculation records, stale-input notices and undo behavior. With the default six grid samples per axis plus current lengths, 49 candidates are tested, 41 pass, and the height-priority result uses 63 mm thighs and shins for 153.401 mm (6.039 in) overall height. This numerical example is conditional on the default manual mass/CoM and actuator assumptions.

Length-unit update: **37 tests passed in 61.19 seconds**. Added coverage for mm/in input synchronization, conversion without design changes, inch edits, JSON loading/reset and ground placement in inches, sweep-range conversion/result preservation, table headers, plot coordinates/dimension labels and equal spatial proportions. The live dashboard was switched to inches and its 6.187 in overall-height result verified in the browser.

Validated locally on Windows with Python 3.12. Direct package versions are recorded in `dashboard-requirements-tested.txt`.

**Result: 33 tests passed in 58.07 seconds.**

After the final Streamlit API cleanup, the three UI tests also passed again in 23.00 seconds. A separate scene check verified the reachable-workspace overlay and equal millimeter scaling on all three explicit axis ranges. The final orthographic 3D framing was visually inspected after restarting the server; `preview.png` captures that view.

```powershell
.\.dashboard-venv\Scripts\python.exe -m pytest dashboard_tests -q
```

The automated suite exercises:

- Straight, bent and folded known FK positions; both analytic IK branches across 63 input angle pairs, inner/outer unreachable targets, limits and singular warnings.
- Independent single-mass gravity moment and suspended torque versus a finite-difference potential-energy derivative.
- Symmetric CoM, rigid transforms, four separate servo masses, correct knee-case carrier, fixed manual mass/CoM assumptions and unique configuration field keys.
- Double-support force/moment residuals, body/payload load effects, raised-leg single support with lateral payload, structural roll reactions and opposing tilted edge supports.
- Ground penetration, floating selected contacts, inconsistent unselected contacts, tipping outside the hull and degenerate edge/point support rejection.
- Finite-grid comparisons, packaging failures, objective ranking, symmetry propagation, fixed masses and preservation of source inputs.
- Strict JSON round trips, duplicate/missing fields, nonfinite/out-of-bounds values, invalid limits, all-zero mass and invalid JSON rejection.
- Real Streamlit `AppTest` runs: slider → number and number → slider, IK apply, pasted JSON loading, reset, independent right-leg controls, invalid-load state preservation, recovery from invalid live limits, sweep execution, range/count synchronization and stale-result invalidation.

The app was launched on `http://127.0.0.1:8501` and inspected in the in-app browser. The 3D model is interactive; all views consume the same transformed model used for engineering results. UI tests exercise the actual Streamlit script, rather than copied callback logic.

Default-model numerical reference (assumptions, not measured robot performance):

| Output | Result |
|---|---:|
| Overall geometric height | 157.160 mm |
| Hip midpoint height | 128.160 mm |
| Total modeled mass | 437.000 g |
| Geometric tipping margin | 30.174 mm |
| Largest static stance joint demand, chosen reaction distribution | 0.025399 N·m |
| Assumed working allowance | 0.269683 N·m |
| Net vertical-force residual magnitude | < 1e−12 N |
| Net moment residual magnitude | < 1e−12 N·m |
| Sampled symmetric flat-foot geometric height range | 112.903–161.080 mm |

Not validated with hardware: true servo specifications/travel, horn/bracket/PLA strength, real mass distributions, CAD collisions, friction, supply transients, continuous torque/thermal limits and dynamic walking. File-upload/download behavior is supported by standard Streamlit controls; configuration serialization and paste/load/reset paths are automated, while a native file-chooser transfer was not automated. No motor was commanded. Degenerate supports and coupled asymmetric achievable-height ranges are explicitly **not evaluated**.

## Goal-driven interface update

The main interface now exposes height, payload, step length and foot clearance,
with an optional weight budget. Detailed engineering inputs remain in Advanced.
The complete suite passes: **59 tests**. Added coverage verifies derived envelope
height, actual payload insertion, payload-dependent torque screening, estimated
mass locations, unchanged hardware ratings/masses, no mutation on failed search,
full-resolution preview of an optimized candidate, inches input, and UI undo.
Live browser verification on Negentropy port 8502: 180 mm height and 100 g payload
produced 81.7 mm thighs, 66.8 mm shins and 537 g modeled mass; undo restored the
starting design. `simple-goals-preview.png` shows the new main controls.
These checks validate the software model, not unsupported walking or hardware.

## SolidWorks parameter export

12 export and Streamlit UI tests passed. Coverage includes quoted equation
syntax, explicit units, dependency order, unique variable names, mirrored and
independent right geometry, contact-model-specific dimensions, frozen revisions,
ZIP/configuration consistency, inches UI export and stale-snapshot warnings.
The live Negentropy interface finalized revision 1e0d736419f6 (current model:
65 mm thighs/shins, 65 mm hip spacing). A matching text file and CAD package are
saved in cad_handoff/. solidworks-export-preview.png shows the live downloads.
Official SolidWorks import/link and external-text-file documentation informed the
format. An actual SolidWorks import/rebuild was not executed in this environment.

## Simplified CAD dimensions

The rounded-foot equations file now contains 10 core dimensions: hip spacing,
waist length/width/height, and left/right thigh length, shin length and foot radius.
Four optional CAD-only link cross-sections bring the total to 14 when entered.
Unspecified cross-sections are omitted. Waist defaults use the platform envelope.
Six targeted export/UI tests passed, including optional dimensions, schema and
snapshot behavior, millimeter output from inches inputs, and unit-switch handling.
CAD-only dimensions do not update or validate the mechanical mass/geometry model.



## Project-root relocation

Dashboard code now resides in the Negentropy root. Its dedicated environment is .dashboard-venv, dependencies are dashboard-requirements.txt, tests are dashboard_tests, and the launcher is run_dashboard.bat. Existing project files were retained. Previous root docs and dashboard backup files are in archive/dashboard-layout-backup-fe67a7cc. The relocated interpreter, root launcher and HTTP health endpoint were verified. Six CAD/export and Streamlit UI checks passed from the new location in 17.18 seconds.

