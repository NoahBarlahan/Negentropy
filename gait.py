"""Prescribed, sagittal, rounded-foot stepping. This is NOT a dynamics solver."""
from copy import deepcopy
import numpy as np
from scipy.optimize import root
import plotly.graph_objects as go
from config import effective
from model import build,contacts,fk,nub_ik,rotation,within_limits,GROUND_TOL
from units import mm_per_unit


def center_ik(c,side,hip,target,previous=None):
    """IK to sphere center, including the rigid shin-frame attachment offset."""
    local=rotation(np.radians(c['pitch'])).T@(np.asarray(target)-hip)
    candidates=[]
    for a,b in nub_ik(c,side,local[0],local[2]):
        if within_limits(c,side,(a,b)):
            candidates.append(np.array([a,b]))
    reference=np.radians([c[f'{side}_hip'],c[f'{side}_knee']]) if previous is None else previous
    return min(candidates,key=lambda q:np.linalg.norm(q-reference)) if candidates else None


def sphere_center(c,side,hip,q):
    end=fk(c[f'{side}_thigh']/1000,c[f'{side}_shin']/1000,*q)[1]
    offset=rotation(sum(q))@np.array([c[f'{side}_foot_ox'],0,c[f'{side}_foot_oz']])/1000
    return hip+rotation(np.radians(c['pitch']))@(end+offset)


def preview(data,step_mm=None,lift_mm=None,period=1.6,duty=.6,cycles=2,samples=40):
    c=effective(data)
    if c['foot_model']!='Rounded nub':
        raise ValueError('Select Rounded nub for this stepping preview.')
    step_mm=c['reach'] if step_mm is None else step_mm
    lift_mm=c['lift'] if lift_mm is None else lift_mm
    if not np.isfinite([step_mm,lift_mm,period,duty]).all() or not (0<=step_mm<=160 and 0<=lift_mm<=120 and .4<=period<=10 and .5<=duty<=.8):
        raise ValueError('Invalid step, lift, cycle time or stance fraction.')
    if type(cycles) is not int or not 1<=cycles<=4 or type(samples) is not int or not 16<=samples<=80:
        raise ValueError('Use 1–4 cycles and 16–80 samples per cycle.')
    stride=2*step_mm/1000
    speed=stride/period
    base=build(c)
    frames=[]
    failure=None
    parameters={}
    for side in ('L','R'):
        hip=base['legs'][side]['hip']
        radius=c[f'{side}_nub_radius']/1000
        touchdown=np.array([hip[0]+stride*duty/2,hip[1],radius])
        qt=center_ik(c,side,hip,touchdown)
        if qt is None:
            return {'frames':[],'failure':f'{side}: touchdown target is unreachable or outside joint limits at the selected pelvis height.','complete':False}
        alpha_t=np.radians(c['pitch'])+sum(qt)
        def stance_q(h,guess,side=side,touchdown=touchdown,radius=radius,alpha_t=alpha_t):
            def residual(q):
                p=sphere_center(c,side,h,q)
                return [p[0]+radius*(np.radians(c['pitch'])+sum(q)-alpha_t)-touchdown[0],p[2]-radius]
            solved=root(residual,guess)
            if np.linalg.norm(residual(solved.x))>1e-7 or not within_limits(c,side,solved.x):
                return None
            return solved.x
        hip_end=hip+np.array([stride*duty,0,0])
        ql=stance_q(hip_end,qt)
        if ql is None:
            return {'frames':[],'failure':f'{side}: stance rolling path exceeds reach or joint limits. Shorten the step or lower the pelvis explicitly.','complete':False}
        center_l=sphere_center(c,side,hip_end,ql)
        def center_speed(q,side=side,hip=hip,radius=radius):
            h=1e-6
            j=np.column_stack([(sphere_center(c,side,hip,q+np.eye(2)[i]*h)-sphere_center(c,side,hip,q-np.eye(2)[i]*h))/(2*h) for i in range(2)])
            a=np.vstack([j[0]+radius,j[2]])
            qdot=np.linalg.lstsq(a,np.array([-speed,0]),rcond=None)[0]
            return -radius*sum(qdot)
        parameters[side]=(qt,ql,touchdown,center_l,alpha_t,center_speed(ql),center_speed(qt))
    previous={s:parameters[s][0] for s in ('L','R')}
    for index,t in enumerate(np.linspace(0,cycles*period,cycles*samples+1)):
        current=deepcopy(c);current['symmetry']=False
        current['pelvis_x']=c['pelvis_x']+speed*t*1000
        if not -300<=current['pelvis_x']<=300:
            failure='Preview exceeds the editable world-x range; reduce step length or cycle count.';break
        current['mode']='Grounded pose'
        hip_robot=build(current)
        phases={};rolling={}
        for side,offset in (('L',0.),('R',.5)):
            u_total=t/period+offset
            cycle=int(np.floor(u_total+1e-12));u=u_total-cycle
            qt,ql,touch,center_l,alpha_t,v0,v1=parameters[side]
            shift=stride*(cycle-offset)
            hip=hip_robot['legs'][side]['hip']
            radius=c[f'{side}_nub_radius']/1000
            if u<=duty+1e-12:
                def residual(q):
                    p=sphere_center(c,side,hip,q)
                    return [p[0]+radius*(np.radians(c['pitch'])+sum(q)-alpha_t)-touch[0]-shift,p[2]-radius]
                solved=root(residual,previous[side])
                q=solved.x
                if np.linalg.norm(residual(q))>1e-7 or not within_limits(c,side,q):
                    failure=f'{side}: stance IK/rolling constraint failed at {t:.2f} s.';break
                phases[side]='stance (rolling)'
                rolling[side]=float(np.linalg.norm(residual(q)))
            else:
                v=(u-duty)/(1-duty)
                # Quintic Hermite x interpolation matches rolling center speed
                # at lift-off and next touchdown; z has zero endpoint velocity.
                duration=period*(1-duty)
                x0=center_l[0]+shift;x1=touch[0]+shift+stride
                delta=x1-x0;w0=v0*duration;w1=v1*duration
                x=x0+w0*v+(10*delta-6*w0-4*w1)*v**3+(-15*delta+8*w0+7*w1)*v**4+(6*delta-3*w0-3*w1)*v**5
                target=np.array([x,hip[1],radius+lift_mm/1000*np.sin(np.pi*v)**2])
                q=center_ik(c,side,hip,target,previous[side])
                if q is None:
                    failure=f'{side}: swing target is unreachable or outside joint limits at {t:.2f} s.';break
                phases[side]='swing'
            current[f'{side}_hip'],current[f'{side}_knee']=map(float,np.degrees(q))
            previous[side]=q
        if failure:
            break
        robot=build(current)
        active=[s for s,l in robot['legs'].items() if abs(l['sole'][0,2])<=GROUND_TOL]
        current['contacts']='Both' if len(active)==2 else 'Left' if active==['L'] else 'Right' if active==['R'] else 'None'
        robot=build(current);contact=contacts(robot)
        if contact['errors']:
            failure=f'Frame {index}: '+ ' '.join(contact['errors']);break
        frames.append({'time':float(t),'robot':robot,'contact':contact,'phases':phases,'rolling_error':rolling,
                       'angles_deg':[current[f'{s}_{j}'] for s in ('L','R') for j in ('hip','knee')]})
    velocity=np.diff([f['angles_deg'] for f in frames],axis=0)/(period/samples) if len(frames)>1 else np.zeros((1,4))
    return {'frames':frames,'failure':failure,'complete':failure is None,'speed_mm_s':speed*1000,
            'max_joint_speed_deg_s':float(np.abs(velocity).max()),'period':period,'step_mm':step_mm,
            'lift_mm':lift_mm,'duty':duty,'cycles':cycles,
            'basis':'Prescribed pelvis motion, no-slip spherical rolling in stance and lifted swing paths; dynamics, friction limits and lateral balance NOT evaluated.'}


def sphere_trace(center,radius,color,scale=1000):
    a,b=np.meshgrid(np.linspace(0,2*np.pi,17),np.linspace(0,np.pi,9))
    xyz=np.array([np.cos(a)*np.sin(b),np.sin(a)*np.sin(b),np.cos(b)]).reshape(3,-1).T*radius+center
    triangles=[]
    for i in range(8):
        for j in range(16):
            n=i*17+j;triangles.extend([(n,n+1,n+17),(n+1,n+18,n+17)])
    tri=np.array(triangles);p=xyz*scale
    return go.Mesh3d(x=p[:,0],y=p[:,1],z=p[:,2],i=tri[:,0],j=tri[:,1],k=tri[:,2],color=color,opacity=.85,showlegend=False,name='Rounded nub')


def animation(result,unit='mm'):
    frames=result['frames']
    if not frames:
        raise ValueError('No valid motion frames to animate.')
    scale=1000/mm_per_unit(unit)
    def traces(frame):
        r=frame['robot'];ct=frame['contact'];data=[]
        for side,color in (('L','#38bdf8'),('R','#fb923c')):
            leg=r['legs'][side];p=np.array([leg['hip'],leg['knee'],leg['end'],leg['foot']])*scale
            data.append(go.Scatter3d(x=p[:,0],y=p[:,1],z=p[:,2],mode='lines+markers',line=dict(color=color,width=10),marker=dict(size=4),name=side+' leg'))
            data.append(sphere_trace(leg['foot'],leg['radius'],color,scale))
        p=r['boxes']['platform']*scale
        data.append(go.Mesh3d(x=p[:,0],y=p[:,1],z=p[:,2],i=[0,0,4,4,0,0,1,1,2,2,3,3],j=[1,2,6,7,4,5,5,6,6,7,7,4],k=[2,3,5,6,5,1,6,2,7,3,4,0],color='#a78bfa',opacity=.65,name='Platform'))
        com=r['com']*scale
        data.append(go.Scatter3d(x=[com[0],com[0]],y=[com[1],com[1]],z=[com[2],0],mode='lines+markers',marker=dict(color='#f43f5e',size=6),line=dict(color='#f43f5e',dash='dash'),name='CoM / projection'))
        p=ct['points']*scale
        data.append(go.Scatter3d(x=p[:,0],y=p[:,1],z=p[:,2],mode='lines+markers',line=dict(color='#34d399',width=5),marker=dict(color='#34d399',size=6),name='Contact point / line (zero area)'))
        return data
    fig=go.Figure(data=traces(frames[0]))
    bounds=np.vstack([f['robot']['vertices'] for f in frames])*scale
    low=bounds.min(axis=0);high=bounds.max(axis=0);low[2]=min(0,low[2]);span=np.maximum(high-low,20/mm_per_unit(unit))*1.15;mid=(low+high)/2
    x0,x1=mid[0]-span[0]/2,mid[0]+span[0]/2;y0,y1=mid[1]-span[1]/2,mid[1]+span[1]/2
    fig.add_trace(go.Mesh3d(x=[x0,x1,x1,x0],y=[y0,y0,y1,y1],z=[0]*4,i=[0,0],j=[1,2],k=[2,3],color='#64748b',opacity=.12,name='Ground'))
    fig.frames=[go.Frame(name=str(i),data=traces(f),traces=list(range(7)),layout=dict(title=dict(text=f"Kinematic preview · {f['time']:.2f} s<br>L: {f['phases']['L']} · R: {f['phases']['R']}",font=dict(size=12)))) for i,f in enumerate(frames)]
    duration=1000*result['period']/(len(frames)-1)*result['cycles'] if len(frames)>1 else 50
    fig.update_layout(height=640,showlegend=False,margin=dict(l=0,r=0,t=70,b=140),template='plotly_dark',paper_bgcolor='#0e1117',
        title=dict(text='Kinematic stepping preview<br>Dynamic balance not evaluated',font=dict(size=12)),
        scene=dict(aspectmode='manual',aspectratio=dict(zip('xyz',span/span.max())),
            **{a+'axis':dict(title=a+f' ({unit})',range=[mid[i]-span[i]/2,mid[i]+span[i]/2],nticks=4) for i,a in enumerate('xyz')},
            camera=dict(eye=dict(x=1.1,y=1.4,z=.7),projection=dict(type='orthographic'))),
        updatemenus=[dict(type='buttons',direction='left',x=0,y=-.21,bgcolor='#1e293b',showactive=False,bordercolor='#64748b',font=dict(color='#f1f5f9'),buttons=[
            dict(label='▶ Play',method='animate',args=[None,dict(frame=dict(duration=duration,redraw=True),transition=dict(duration=0),fromcurrent=True,mode='immediate')]),
            dict(label='❚❚ Pause',method='animate',args=[[None],dict(frame=dict(duration=0,redraw=False),mode='immediate')])])],
        sliders=[dict(x=0,len=1,y=-.03,currentvalue=dict(prefix='Frame '),steps=[dict(label=str(i),method='animate',args=[[str(i)],dict(mode='immediate',frame=dict(duration=0,redraw=True),transition=dict(duration=0))]) for i in range(len(frames))])])
    return fig
