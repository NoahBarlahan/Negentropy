"""Explicit-unit SOLIDWORKS global-variable files from frozen model snapshots."""
from copy import deepcopy
from datetime import datetime, timezone
import csv
import hashlib
import io
import json
import math
import zipfile
from config import dumps, effective, validate

EQUATIONS_FILE='negentropy_equations.txt'
HELP_URL='https://help.solidworks.com/2023/english/SolidWorks/sldworks/t_EQN_Importing_Equations.htm'


EXPORT_SCHEMA=2
CAD_FIELDS={
    'WaistLength':'Waist length', 'WaistWidth':'Waist width', 'WaistHeight':'Waist height / thickness',
    'ThighWidth':'Thigh width', 'ThighThickness':'Thigh thickness',
    'ShinWidth':'Shin width', 'ShinThickness':'Shin thickness',
}


def clean_dimensions(dimensions=None):
    result={}
    for key,value in (dimensions or {}).items():
        if key not in CAD_FIELDS or isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or not .1<=value<=500:
            raise ValueError('CAD dimensions must be known dimensions between 0.1 and 500 mm.')
        result[key]=float(value)
    return result


def parameters(data,dimensions=None,unit='mm'):
    if unit not in ('mm','in'):
        raise ValueError('CAD export length units must be mm or in.')
    c=effective(data);dimensions=clean_dimensions(dimensions)
    rows=[]
    def add(name,value,meaning):
        shown=float(value)/(25.4 if unit=='in' else 1.)
        rows.append(dict(variable=name,value=shown,unit=unit,meaning=meaning,
                         equation=f'{shown:.8f}'.rstrip('0').rstrip('.')+' '+unit))
    add('HipSpacing',c['spacing'],'Distance between hip pitch axes')
    for name,key in (('WaistLength','platform_l'),('WaistWidth','platform_w'),('WaistHeight','platform_t')):
        add(name,dimensions.get(name,c[key]),'CAD waist dimension; defaults to the dashboard platform envelope')
    for name in ('ThighWidth','ThighThickness','ShinWidth','ShinThickness'):
        if name in dimensions:
            add(name,dimensions[name],'User-selected CAD cross-section, shared by both legs; not optimized or strength-validated')
    for side in ('L','R'):
        add('ThighLength_'+side,c[f'{side}_thigh'],'Hip axis to knee axis, center-to-center; not printed outside length')
        add('ShinLength_'+side,c[f'{side}_shin'],'Knee axis to shin endpoint; sphere center only with zero foot offset')
        if c['foot_model']=='Rounded nub':
            add('FootRadius_'+side,c[f'{side}_nub_radius'],'Spherical nub radius')
        else:
            for name,key in (('FootLength','foot_l'),('FootWidth','foot_w'),('FootThickness','foot_t')):
                add(name+'_'+side,c[f'{side}_{key}'],'Flat foot envelope')
    return rows


def equations(data,dimensions=None,unit='mm'):
    return ''.join(f'"{r["variable"]}" = {r["equation"]}\r\n' for r in parameters(data,dimensions,unit))


def finalize(data,dimensions=None,unit='mm'):
    config=deepcopy(validate(data));dimensions=clean_dimensions(dimensions)
    text=equations(config,dimensions,unit)
    signature=hashlib.sha256((dumps(config)+json.dumps(dimensions,sort_keys=True)+str(EXPORT_SCHEMA)+unit).encode('utf-8')).hexdigest()[:12]
    return {'config':config,'dimensions':dimensions,'unit':unit,'export_schema':EXPORT_SCHEMA,'revision':signature,
            'created':datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC'),
            'parameters':parameters(config,dimensions,unit),'text':text}


def with_units(snapshot,unit):
    """Reformat only the frozen design, never the current/live configuration."""
    converted=finalize(snapshot['config'],snapshot['dimensions'],unit)
    converted['created']=snapshot['created']
    return converted


def instructions(snapshot):
    c=effective(snapshot['config'])
    return f'''NEGENTROPY - SIMPLE SOLIDWORKS CAD DIMENSIONS
Revision: {snapshot['revision']}
Finalized: {snapshot['created']}
Contact model: {c['foot_model']}
Equations and parameter-map units: {snapshot.get('unit','mm')}
Symmetry: {c['symmetry']} (right lengths/radii use effective mirrored values)

START YOUR CAD
1. Keep {EQUATIONS_FILE} in a permanent folder beside your CAD project.
2. Open the .txt in Notepad to inspect or edit dimensions. Preserve quotes and unit suffixes.
3. In SolidWorks open Tools > Equations > Import and select this text file.
   Enable Link to File if available to update CAD from later file edits.
4. Set the hip-to-knee sketch distance to ="ThighLength_L" and the knee-to-shin-
   endpoint distance to ="ShinLength_L". Use _R for the other leg.
5. Use ="FootRadius_L" for a rounded nub radius, or =2*"FootRadius_L" for diameter.
   Use WaistLength, WaistWidth and WaistHeight for your waist platform/body.
6. When supplied, ThighWidth/ThighThickness and ShinWidth/ShinThickness drive
   the printed link cross-sections. These four values are shared by both legs.
7. Link/import in each part that needs the variables, then rebuild your CAD.

UPDATING
Finalize again and replace the SAME linked {EQUATIONS_FILE}, then rebuild.
Use a stable path and retain revision ZIPs separately. Without linking, re-import.
Dashboard edits do not change a frozen export until you finalize again.
Switching dashboard length units reformats this same frozen design for download.
Editing the text does not send values back to the dashboard.

DIMENSION REFERENCES
Thigh/shin lengths are joint-reference distances, not outside printed lengths.
The shin endpoint equals the sphere center only with zero foot attachment offset.
Dashboard foot offsets (L x/z, R x/z), in mm:
{c['L_foot_ox']}, {c['L_foot_oz']}; {c['R_foot_ox']}, {c['R_foot_oz']}.
These offsets are notes only, not exported equations.

Waist dimensions default to the dashboard platform envelope. WaistHeight is
platform thickness unless you supply a CAD body-height override. CAD-only width,
thickness and waist overrides do not alter robot geometry, masses or calculated
height in the dashboard; they are not structurally optimized. Update the model
and mass estimates separately as your CAD develops. Unspecified cross-sections
are omitted, rather than assigned guessed values.
No pose angles, case positions, servo envelopes, overall-height targets or
redundant foot diameters are exported. See parameter_map.csv for definitions.
biped_config.json restores the dashboard model. cad_dimensions.json records CAD
extras in mm; re-enter these in the CAD tab when restoring another session.

Measure mounts, shaft/horn geometry, bearings, fasteners and print fits yourself.
These variables do not generate sketches/features/mates or verify robot strength.
Official import/link instructions: {HELP_URL}
'''


def bundle(snapshot):
    buffer=io.BytesIO()
    mapping=io.StringIO(newline='')
    fields=['variable','value','unit','meaning','equation']
    writer=csv.DictWriter(mapping,fieldnames=fields);writer.writeheader();writer.writerows(snapshot['parameters'])
    with zipfile.ZipFile(buffer,'w',zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(EQUATIONS_FILE,snapshot['text'])
        archive.writestr('START_HERE.txt',instructions(snapshot))
        archive.writestr('parameter_map.csv',mapping.getvalue())
        archive.writestr('biped_config.json',dumps(snapshot['config']))
        archive.writestr('cad_dimensions.json',json.dumps(snapshot['dimensions'],indent=2))
    return buffer.getvalue()
