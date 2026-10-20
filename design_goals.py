"""Bounded goal-driven geometry search; no claim of dynamic feasibility."""
from copy import deepcopy
import math
import numpy as np
from config import validate
from model import build, torque_rows, FEASIBLE, G
from sweep import compare, design_criteria
from gait import preview


def optimize(data, height, payload, step, clearance, mass_limit):
    """Return a reviewable result without mutating data, including on failure.

    Lengths solve envelope height, not hip height. Fixed hardware and measured
    masses remain constraints. Link CoMs are explicitly estimated at mid-link.
    A full-weight vertical reaction per stance leg is a conservative load case,
    not a solved ground reaction or a proof of balance.
    """
    values=[height,payload,step,clearance,mass_limit]
    if not np.isfinite(values).all() or not (80<=height<=450 and 0<=payload<=2000 and
            0<=step<=160 and 0<=clearance<=120 and 10<=mass_limit<=5000):
        raise ValueError('Goals are outside the supported search limits.')
    base=deepcopy(validate(data))
    if base['foot_model']!='Rounded nub':
        raise ValueError('Goal optimization currently requires Rounded nub. Select it in Advanced.')
    base.update(symmetry=True,mode='Grounded pose',contacts='Both',pitch=0.,
                payload_mass=float(payload),payload_target=float(payload),
                min_height=max(1.,height-1.),max_height=height+1.,
                reach=float(step),lift=float(clearance),max_mass=float(mass_limit))
    candidates=[]; failures={}; tested=0
    def reject(reason):
        failures[reason]=failures.get(reason,0)+1
    for ratio in (.45,.5,.55):
        for bend in (25.,35.,45.,55.,65.,75.):
            tested+=1
            c=deepcopy(base)
            knee=math.radians(bend)
            hip=math.degrees(math.atan2(-(1-ratio)*math.sin(knee),ratio+(1-ratio)*math.cos(knee)))
            def size(total):
                for side in ('L','R'):
                    a,b=total*ratio,total*(1-ratio)
                    c.update({f'{side}_thigh':a,f'{side}_shin':b,
                              f'{side}_hip':hip,f'{side}_knee':bend,
                              f'{side}_knee_servo_com_z':-a,
                              f'{side}_thigh_print_com_z':-a/2,
                              f'{side}_shin_print_com_z':-b/2})
                c['pelvis_z']=200.
                robot=build(c)
                c['pelvis_z']-=min(l['sole'][:,2].min() for l in robot['legs'].values())*1000
                return robot['height']*1000
            lo=max(5/ratio,5/(1-ratio));hi=min(200/ratio,200/(1-ratio))
            if not size(lo)<=height<=size(hi):
                reject('Height outside geometry range');continue
            for _ in range(25):
                mid=(lo+hi)/2
                if size(mid)<height: lo=mid
                else: hi=mid
            size((lo+hi)/2)
            row=compare(c,[c['L_thigh']],[c['L_shin']],design_criteria(c))[0]
            failed=[key for key in design_criteria(c) if row[key]!=FEASIBLE]
            if failed:
                for key in failed: reject(key)
                continue
            motion=preview(c,cycles=1,samples=16)
            if not motion['complete']:
                reject('Sampled walking reach / clearance');continue
            peak=row['objective worst screening (N·m)']
            for frame in motion['frames']:
                robot=frame['robot']
                for load in torque_rows(robot):
                    side,joint=load['joint'].split()
                    leg=robot['legs'][side]
                    arm=abs(leg['sole'][0,0]-leg[joint][0])
                    reaction=robot['mass']*G*arm if frame['phases'][side].startswith('stance') else 0.
                    peak=max(peak,(abs(load['gravity moment (N·m)'])+reaction)*c['multiplier'])
            if peak>c['stall']*c['derating']:
                reject('Assumed weight-bearing torque allowance');continue
            candidates.append({'config':deepcopy(c),'peak':peak,'bend':bend,'ratio':ratio,
                               'height':row['overall height (mm)'],'mass':row['mass (g)']})
    candidates.sort(key=lambda item:(item['peak'],abs(item['ratio']-.5),item['bend']))
    return {'source':deepcopy(data),'applied':candidates[0]['config'] if candidates else None,
            'chosen':candidates[0] if candidates else None,'tested':tested,'passing':len(candidates),
            'failures':failures,'goals':dict(zip(('height','payload','step','clearance','mass_limit'),values))}
