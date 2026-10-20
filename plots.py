"""Plotly views consume the same transformed geometry as the calculations."""
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from model import fk, rotation
from gait import sphere_trace

COLORS = {"L": "#38bdf8", "R": "#fb923c"}
EDGES = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
TRIANGLES = [(0,1,2),(0,2,3),(4,6,5),(4,7,6),(0,4,5),(0,5,1),
             (1,5,6),(1,6,2),(2,6,7),(2,7,3),(3,7,4),(3,4,0)]


def view3d(result, show_masses=True, show_workspace=False, show_dimensions=True):
    robot, contact = result["robot"], result["contact"]
    c = robot["config"]
    fig = go.Figure()
    def line(points, name, color, width=4, dash=None, text=None, legend=True):
        p = np.asarray(points)*1000
        fig.add_trace(go.Scatter3d(x=p[:,0], y=p[:,1], z=p[:,2], mode="lines+text" if text else "lines",
            text=text, textposition="top center", name=name, showlegend=legend,
            line=dict(color=color,width=width,dash=dash), hovertemplate=name+"<br>x %{x:.1f} y %{y:.1f} z %{z:.1f} mm<extra></extra>"))
    for name, verts in robot["boxes"].items():
        if c["foot_model"]=="Rounded nub" and name.endswith("_foot"):
            leg=robot['legs'][name[0]]
            fig.add_trace(sphere_trace(leg['foot'],leg['radius'],COLORS[name[0]]))
            continue
        p = verts*1000
        color = COLORS.get(name[0], "#a78bfa")
        opacity = 0.48 if name.endswith("foot") or name == "platform" else 0.22
        tris = np.array(TRIANGLES)
        fig.add_trace(go.Mesh3d(x=p[:,0],y=p[:,1],z=p[:,2], i=tris[:,0],j=tris[:,1],k=tris[:,2],
            color=color,opacity=opacity,name=name,showlegend=False,hovertemplate=name+"<extra></extra>"))
        wire = []
        for a,b in EDGES:
            wire.extend([verts[a],verts[b],[np.nan]*3])
        line(wire,name,color,2,legend=False)
    for side, leg in robot["legs"].items():
        color = COLORS[side]
        p = np.array([leg["hip"],leg["knee"],leg["end"]])
        line(p, f"{side} leg",color,9)
        fig.add_trace(go.Scatter3d(x=p[:,0]*1000,y=p[:,1]*1000,z=p[:,2]*1000,mode="markers+text",
            marker=dict(size=5,color=color),text=[f"{side} hip",f"{side} knee",f"{side} foot ref"],textposition="top center",showlegend=False))
        for joint in ("hip", "knee"):
            origin = leg[joint]
            line([origin-[0,.015,0],origin+[0,.015,0]],f"{side} {joint} pitch axis", "#94a3b8",2,legend=False)
            base = c["pitch"] + (c[f"{side}_hip"] if joint == "knee" else 0)
            theta = np.radians(np.linspace(base, base+c[f"{side}_{joint}"],25))
            arc = origin + np.column_stack([.015*np.sin(theta),np.zeros(25),-.015*np.cos(theta)])
            line(arc,f"{side} {joint}: {c[f'{side}_{joint}']:.1f}°",color,2,legend=False)
        if show_dimensions:
            mids=(p[:-1]+p[1:])/2 + [0,.012,0]
            fig.add_trace(go.Scatter3d(x=mids[:,0]*1000,y=mids[:,1]*1000,z=mids[:,2]*1000,mode="text",
                text=[f"{c[f'{side}_thigh']:.1f} mm",f"{c[f'{side}_shin']:.1f} mm"],showlegend=False))
            sole = leg["sole"]
            if c["foot_model"]=="Flat sole":
                line([sole[0],sole[1]],"Foot length",color,2,text=[f"{c[f'{side}_foot_l']:.0f} mm", ""],legend=False)
            else:
                line([leg['foot'],leg['sole'][0]],"Nub radius",color,2,text=[f"r = {c[f'{side}_nub_radius']:.1f} mm", ""],legend=False)
        if show_workspace:
            h = np.radians(np.linspace(c[f"{side}_hip_min"],c[f"{side}_hip_max"],19))
            k = np.radians(np.linspace(c[f"{side}_knee_min"],c[f"{side}_knee_max"],19))
            points = np.array([leg["hip"]+rotation(np.radians(c["pitch"])) @ fk(c[f"{side}_thigh"]/1000,c[f"{side}_shin"]/1000,a,b)[1] for a in h for b in k])*1000
            fig.add_trace(go.Scatter3d(x=points[:,0],y=points[:,1],z=points[:,2],mode="markers",name=f"{side} endpoint samples",
                marker=dict(size=2,color=color,opacity=.18)))
    if show_masses:
        masses = robot["masses"]
        p = np.array([m.position for m in masses])*1000
        fig.add_trace(go.Scatter3d(x=p[:,0],y=p[:,1],z=p[:,2],mode="markers",name="Individual mass centers",
            marker=dict(size=4,color="#e2e8f0",symbol="diamond"),text=[f"{m.name}: {m.kg*1000:.1f} g ({m.body})" for m in masses],hovertemplate="%{text}<extra></extra>"))
    com = robot["com"]
    line([com,[com[0],com[1],0]],"CoM projection", "#f43f5e",3,"dash")
    p = np.array([com,[com[0],com[1],0]])*1000
    fig.add_trace(go.Scatter3d(x=p[:,0],y=p[:,1],z=p[:,2],mode="markers",name="Whole-robot CoM",marker=dict(size=7,color="#f43f5e",symbol="cross")))
    if len(contact["points"]):
        p=contact["points"]*1000
        fig.add_trace(go.Scatter3d(x=p[:,0],y=p[:,1],z=p[:,2],mode="markers",name="Active contact vertices (selection)",marker=dict(size=5,color="#34d399")))
    hull=contact["polygon"]
    if not len(hull) and len(contact['points']):
        line(contact['points'],'Contact point / line — zero support area','#34d399',4)
    if len(hull):
        p = np.column_stack([hull,np.zeros(len(hull))])
        line(np.vstack([p,p[0]]),"Support region" if c["mode"]=="Grounded pose" else "Contact geometry only (externally supported)","#34d399",5)
        fig.add_trace(go.Mesh3d(x=p[:,0]*1000,y=p[:,1]*1000,z=np.zeros(len(p)),i=[0]*(len(p)-2),
            j=list(range(1,len(p)-1)),k=list(range(2,len(p))),color="#34d399",opacity=.18,showlegend=False))
    extent=max(.06,np.ptp(robot["vertices"][:,:2],axis=0).max()/2+.02)
    center = robot["frames"]["pelvis"][0]
    xy = np.array([[center[0]-extent,center[1]-extent,0],[center[0]+extent,center[1]-extent,0],
                   [center[0]+extent,center[1]+extent,0],[center[0]-extent,center[1]+extent,0]])
    line(np.vstack([xy,xy[0]]),"Ground z = 0", "#475569",2)
    # Explicit ranges avoid renderer-dependent autorange changes from labels
    # and text traces. Axis span / scene aspect ratio keeps mm equally scaled.
    all_points=np.vstack([robot["vertices"],xy,np.array([m.position for m in robot["masses"]])])
    if show_workspace:
        all_points=np.vstack([all_points, *[np.column_stack([np.asarray(t.x),np.asarray(t.y),np.asarray(t.z)])/1000
            for t in fig.data if t.name and "endpoint samples" in t.name]])
    low,high=all_points.min(axis=0)*1000,all_points.max(axis=0)*1000
    span=np.maximum(high-low,20.)*1.15
    mid=(high+low)/2
    axes={a+"axis":dict(title=title,range=[float(mid[i]-span[i]/2),float(mid[i]+span[i]/2)],nticks=4)
          for i,(a,title) in enumerate(zip("xyz",["Forward x (mm)","Left y (mm)","Up z (mm)"]))}
    fig.update_layout(height=580,margin=dict(l=0,r=0,t=0,b=0),paper_bgcolor="rgba(0,0,0,0)",showlegend=False,
        scene=dict(aspectmode="manual",aspectratio=dict(zip("xyz",span/span.max())),**axes,
                   camera=dict(eye=dict(x=1.05,y=1.15,z=.8),projection=dict(type="orthographic"))),uirevision="robot-camera")
    return fig


def orthographic(result):
    r = result["robot"]
    fig=make_subplots(rows=1,cols=3,subplot_titles=["Side · x/z", "Front · y/z", "Top · x/y"])
    for col,axes in enumerate([(0,2),(1,2),(0,1)],1):
        for side,l in r["legs"].items():
            p=np.array([l["hip"],l["knee"],l["end"]])*1000
            fig.add_trace(go.Scatter(x=p[:,axes[0]],y=p[:,axes[1]],mode="lines+markers",line=dict(color=COLORS[side],width=4),name=side,showlegend=col==1),row=1,col=col)
        for name,verts in r["boxes"].items():
            if r['config']['foot_model']=='Rounded nub' and name.endswith('_foot'):
                leg=r['legs'][name[0]];theta=np.linspace(0,2*np.pi,50)
                fig.add_trace(go.Scatter(x=(leg['foot'][axes[0]]+leg['radius']*np.cos(theta))*1000,
                    y=(leg['foot'][axes[1]]+leg['radius']*np.sin(theta))*1000,mode='lines',line=dict(color=COLORS[name[0]]),showlegend=False),row=1,col=col)
                continue
            p=verts*1000
            xs,ys=[],[]
            for a,b in EDGES:
                xs.extend([p[a,axes[0]],p[b,axes[0]],None]);ys.extend([p[a,axes[1]],p[b,axes[1]],None])
            fig.add_trace(go.Scatter(x=xs,y=ys,mode="lines",line=dict(color=COLORS.get(name[0],"#a78bfa"),width=1),showlegend=False),row=1,col=col)
        com=r["com"]*1000
        fig.add_trace(go.Scatter(x=[com[axes[0]]],y=[com[axes[1]]],mode="markers",marker=dict(color="#f43f5e",symbol="cross",size=10),name="CoM",showlegend=False),row=1,col=col)
        if col < 3:
            fig.add_hline(y=0,line_color="#34d399",row=1,col=col)
        elif len(result["contact"]["polygon"]):
            p=result["contact"]["polygon"]*1000;p=np.vstack([p,p[0]])
            fig.add_trace(go.Scatter(x=p[:,0],y=p[:,1],fill="toself",line=dict(color="#34d399"),showlegend=False),row=1,col=col)
        suffix="" if col==1 else str(col)
        fig.update_yaxes(scaleanchor="x"+suffix,scaleratio=1,row=1,col=col,title_text="xyz"[axes[1]]+" (mm)")
        fig.update_xaxes(title_text="xyz"[axes[0]]+" (mm)",row=1,col=col)
    fig.update_layout(height=400,margin=dict(l=10,r=10,t=40,b=10))
    return fig
