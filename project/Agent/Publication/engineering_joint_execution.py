"""Executable engineering-model development components.

Free observation positions, task successor arcs and continuous segment times.
The restricted pilot uses one parked carrier and one return sortie. It is NOT
the full nineteen-block fleet optimizer. Every restriction is exported.
"""
from pathlib import Path
import math, json, time, os
import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree
from pyscipopt import Model, quicksum
from Agent.Publication.engineering_inspection_primitives import camera_metrics, required_thrust


def xyz(d):return np.array([d['easting_m'],d['northing_m'],d['altitude_m']],float)


def closest_segments(a,b,c,d):
    """Distance of two finite 3D segments; endpoint cases included."""
    u,bv,w=b-a,d-c,a-c
    aa,bb,cc,dd,ee=u@u,u@bv,bv@bv,u@w,bv@w
    candidates=[]
    den=aa*cc-bb*bb
    if den>1e-12:
        s,t=(bb*ee-cc*dd)/den,(aa*ee-bb*dd)/den
        if 0<=s<=1 and 0<=t<=1:candidates.append(np.linalg.norm(w+s*u-t*bv))
    for p,q,r in [(a,c,d),(b,c,d),(c,a,b),(d,a,b)]:
        v=r-q; f=np.clip((p-q)@v/max(v@v,1e-30),0,1)
        candidates.append(np.linalg.norm(p-q-f*v))
    return float(min(candidates))


def cylinder_distance(points,c):
    p=np.asarray(points)
    radial=np.linalg.norm(p[:,:2]-c['center_xy_m'],axis=1)-c['radius_m']
    dz=np.maximum(c['base_elevation_m']-p[:,2],p[:,2]-c['base_elevation_m']-c['height_m'])
    return np.hypot(np.maximum(radial,0),np.maximum(dz,0))+np.minimum(np.maximum(radial,dz),0)


class Geometry:
    def __init__(self,scene):
        self.scene=scene;self.profile=scene['joint_reference_profile']
        self.cyl=scene['obstacle_model']['tower_cylinders']
        self.tree=cKDTree([c['center_xy_m'] for c in self.cyl])
        self.wires=[]
        for w in scene['obstacle_model']['conductor_curves']:
            a,b=np.asarray(w['endpoint_xyz_m']);sag=w['sag_m']
            t=np.linspace(0,1,33)
            p=a[None,:]*(1-t[:,None])+b[None,:]*t[:,None]
            p[:,2]-=4*sag*t*(1-t)
            # The exact parabola stays within sag/32^2 of these chords.
            self.wires.append((p,float(w['radius_m'])+sag/32**2,w['id']))
        self.centers=np.array([p.mean(axis=0) for p,r,k in self.wires])
        self.radii=np.array([np.linalg.norm(p-p.mean(axis=0),axis=1).max() for p,r,k in self.wires])

    def wire_sight_margin(self,a,b):
        mid=(a+b)/2;rad=np.linalg.norm(b-a)/2;answer=1000.
        close=np.flatnonzero(np.linalg.norm(self.centers-mid,axis=1)<=self.radii+rad+.2)
        for idx in close:
            pts,r,k=self.wires[idx]
            lo,hi=np.minimum(a,b)-r-.02,np.maximum(a,b)+r+.02
            selected=np.flatnonzero(np.all(np.maximum(pts[:-1],pts[1:])>=lo,axis=1)&np.all(np.minimum(pts[:-1],pts[1:])<=hi,axis=1))
            for j in selected:answer=min(answer,closest_segments(a,b,pts[j],pts[j+1])-r)
        return answer

    def segment_margin(self,a,b,clearance,ignore_wire_ids=()):
        mid=(a+b)/2;rad=np.linalg.norm(b-a)/2
        ids=self.tree.query_ball_point(mid[:2],rad+clearance+3)
        margin=float('inf')
        # Distance is 1-Lipschitz. Dense line samples plus half spacing give
        # a conservative bound on the entire geometric line (not time grid).
        if ids:
            n=max(3,int(math.ceil(np.linalg.norm(b-a)/.5))+1)
            p=np.linspace(a,b,n);err=np.linalg.norm(b-a)/(2*(n-1))
            for idx in ids:margin=min(margin,float(cylinder_distance(p,self.cyl[idx]).min())-err-clearance)
        close=np.flatnonzero(np.linalg.norm(self.centers-mid,axis=1)<=self.radii+rad+clearance+1)
        for idx in close:
            p,r,k=self.wires[idx]
            if k in ignore_wire_ids:continue
            low=np.minimum(p[:-1],p[1:]);high=np.maximum(p[:-1],p[1:])
            lo,hi=np.minimum(a,b)-clearance-r,np.maximum(a,b)+clearance+r
            keep=np.flatnonzero(np.all(high>=lo,axis=1)&np.all(low<=hi,axis=1))
            for j in keep:margin=min(margin,closest_segments(a,b,p[j],p[j+1])-r-clearance)
        return margin


def smooth(s):
    return 35*s**4-84*s**5+70*s**6-20*s**7


def state(a,b,T,s):
    d=b-a;s=np.asarray(s)
    p=a+smooth(s)[...,None]*d
    v=(140*s**3-420*s**4+420*s**5-140*s**6)[...,None]*d/T
    acc=(420*s**2-1680*s**3+2100*s**4-840*s**5)[...,None]*d/T**2
    jerk=(840*s-5040*s**2+8400*s**3-4200*s**4)[...,None]*d/T**3
    return p,v,acc,jerk


def duration(a,b,profile):
    d=b-a;dist=float(np.linalg.norm(d));air=profile['aircraft']
    # Exact extrema of the seventh-degree rest-to-rest basis. Acceleration
    # is tightened in this feasible initializer; not the model's optimum.
    return max(.05,2.1875*dist/air['ground_speed_max_mps'],
        2.1875*abs(d[2])/air['vertical_speed_max_mps'],
        math.sqrt(7.5131884044*dist/min(1.,air['acceleration_max_mps2'])),
        (52.5*dist/min(2.,air['jerk_max_mps3']))**(1/3))


def safe_transfer(a,b,geometry,ground_z,contact=False):
    p=geometry.profile;clearance=sum(p['aircraft'][k] for k in
        ['body_radius_m','obstacle_clearance_m','tracking_error_m','map_error_m'])
    possibilities=[] if contact else [[a,b]]
    # Both high-clearance and lateral three-segment paths are candidate
    # initializers. They are not a fixed sortie catalogue.
    height=max(a[2],b[2],ground_z+55.)
    possibilities.append([a,np.array([a[0],a[1],height]),np.array([b[0],b[1],height]),b])
    delta=b[:2]-a[:2];norm=np.linalg.norm(delta)
    if norm>1e-8 and not contact:
        perp=np.array([-delta[1],delta[0]])/norm
        for side in [-1,1]:
            shift=np.r_[side*22*perp,0.]
            possibilities.append([a,a+shift,b+shift,b])
    valid=[]
    for pts in possibilities:
        pts=[np.asarray(x,float) for i,x in enumerate(pts) if i==0 or np.linalg.norm(x-pts[i-1])>1e-6]
        margins=[geometry.segment_margin(x,y,clearance) for x,y in zip(pts,pts[1:])]
        if margins and min(margins)>=-1e-7:
            times=[duration(x,y,p) for x,y in zip(pts,pts[1:])]
            valid.append((sum(times),pts,times,min(margins)))
    if not valid:return None
    cost,pts,times,margin=min(valid,key=lambda x:x[0])
    return dict(points=[x.tolist() for x in pts],durations_s=times,duration_s=cost,
                collision_margin_m=margin if math.isfinite(margin) else None,
                representation='septic rest-to-rest initialization; C3 continuous')


def camera_rotation(p,target):
    z=target-p;z=z/np.linalg.norm(z)
    x=np.cross(z,[0,0,1.]);x=x/np.linalg.norm(x)
    y=np.cross(z,x)
    return np.column_stack([x,y,z])


def optimize_viewpoint(task,previous,next_point,profile,geometry=None):
    p0=xyz(task);q=xyz(task['target']);n=np.array(task['target']['normal_world'])
    cam=profile['camera'];radius=task['observation_requirement']['viewpoint_ball_radius_m']
    fx=math.hypot(cam['image_width_px'],cam['image_height_px'])/(2*math.tan(math.radians(cam['diagonal_fov_deg_at_wide']/2)))
    tangent=np.array(task['target']['tangent_basis']).T
    samples=np.array(task['target']['surface_samples_utm_m'])
    def constraints(offset):
        pos=p0+offset;delta=pos-q;dist=np.linalg.norm(delta)
        rot=camera_rotation(pos,q)
        values=[radius**2-offset@offset,dist-cam['focus_min_m'],cam['observation_max_m']-dist,
                delta@n/max(dist,1e-12)-math.cos(math.radians(cam['incidence_angle_max_deg']))]
        for pt in samples:
            m=camera_metrics(pt,pos,rot,tangent,fx,fx,cam['image_width_px'],cam['image_height_px'],cam['frame_border_px'])
            if m['pixel'] is None:return np.full(9,-1e3)
            values.append((cam['gsd_max_m_per_px']-m['worst_gsd_m_per_px'])*1000)
        if geometry is not None:
            margin=geometry.segment_margin(pos,pos,4.6)
            values.append(min(1000.,margin))
            for pt in samples:values.append(10*(geometry.wire_sight_margin(pos,pt)-.001))
        return np.array(values)
    candidates=[];attempts=[]
    for initial in [np.zeros(3)]+[sign*radius*np.eye(3)[axis] for axis in range(3) for sign in [-1,1]]:
        fit=minimize(lambda o:np.linalg.norm(p0+o-previous)+np.linalg.norm(next_point-p0-o),initial,
            method='SLSQP',bounds=[(-radius,radius)]*3,constraints={'type':'ineq','fun':constraints},
            options={'maxiter':80,'ftol':1e-9})
        attempts.append({'success':bool(fit.success),'minimum_margin':float(constraints(fit.x).min())})
        if fit.success and constraints(fit.x).min()>=-1e-7:candidates.append(fit)
        if candidates:break
    fit=min(candidates,key=lambda f:f.fun) if candidates else fit
    offset=fit.x if candidates else np.zeros(3)
    return p0+offset,dict(success=bool(fit.success),offset_m=offset.tolist(),constraint_min=float(constraints(offset).min()),
                        attempts=attempts,optimized_variables=['viewpoint_x','viewpoint_y','viewpoint_z'])


def route_scip(costs,dwell,log_path):
    """Exact successor model on current leg costs, with no ten-second limit."""
    n=len(dwell);start=n;end=n+1
    edges={(i,j):v for (i,j),v in costs.items() if i!=end and j!=start and i!=j}
    m=Model('inspection_sequence_current_geometry');m.hideOutput()
    cwd=os.getcwd()
    try:
        os.chdir(log_path.parent);m.setLogfile(log_path.name)
    finally:os.chdir(cwd)
    m.setParam('limits/gap',.05);m.setParam('limits/memory',2048);m.setParam('parallel/maxnthreads',1)
    h={e:m.addVar(vtype='B',name=f'h_{e[0]}_{e[1]}') for e in edges}
    for i in range(n):
        m.addCons(quicksum(x for (a,b),x in h.items() if a==i)==1)
        m.addCons(quicksum(x for (a,b),x in h.items() if b==i)==1)
    m.addCons(quicksum(x for (a,b),x in h.items() if a==start)==1)
    m.addCons(quicksum(x for (a,b),x in h.items() if b==end)==1)
    order={i:m.addVar(lb=0,ub=n-1,name=f'order_{i}') for i in range(n)}
    for (i,j),hij in h.items():
        if i<n and j<n:m.addCons(order[j]>=order[i]+1-n*(1-hij))
    m.setObjective(quicksum(edges[e]*v for e,v in h.items())+sum(dwell),'minimize')
    nv,nc=m.getNVars(),m.getNConss()
    st=time.perf_counter();m.optimize();elapsed=time.perf_counter()-st
    result={'status':str(m.getStatus()),'seconds':elapsed,'variables':nv,
            'constraints':nc,'time_limit_seconds':None,'target_gap':.05,
            'gap_scope':'task order at CURRENT geometry; not full joint model gap'}
    if m.getNSols()==0:return None,result
    selected={i:j for (i,j),v in h.items() if m.getVal(v)>.5}
    seq=[];cur=start
    for k in range(n+1):
        cur=selected[cur]
        if cur==end:break
        seq.append(cur)
    assert len(seq)==n and len(set(seq))==n
    result.update(objective_s=float(m.getObjVal()),bound_s=float(m.getDualbound()),gap=float(m.getGap()),sequence=seq)
    return seq,result


def reference_power(v,rho,wind,profile):
    e,a=profile['energy'],profile['aircraft']
    va=v-np.asarray(wind)
    return (e['hover_power_W']*np.sqrt(1.225/np.asarray(rho))+.5*a['drag_CdA_m2']*np.asarray(rho)*np.linalg.norm(va,axis=-1)**3
            +a['mass_kg']*9.81*np.maximum(v[...,2],0)/e['propulsion_efficiency'])


def partial_charge_scip(initial_wh,required_wh,profile,log_path):
    """Optimized initial partial charge at the parked carrier, not pack reuse."""
    e=profile['energy'];cap=e['pair_capacity_Wh'];eta=e['charge_efficiency']
    # Curve is stored energy, input includes efficiency exactly once.
    fast=(.7*cap)/1800;tail=(.1*cap)/1200
    m=Model('partial_charge');m.hideOutput()
    cwd=os.getcwd()
    try:
        os.chdir(log_path.parent);m.setLogfile(log_path.name)
    finally:os.chdir(cwd)
    m.setParam('limits/gap',.05)
    q1=m.addVar(lb=0,ub=max(0,.9*cap-initial_wh),name='fast_stored_Wh')
    q2=m.addVar(lb=0,ub=.1*cap,name='tail_stored_Wh')
    tail_active=m.addVar(vtype='B',name='tail_active')
    total=m.addVar(lb=0,ub=cap,name='launch_Wh')
    m.addCons(total==initial_wh+q1+q2)
    m.addCons(total>=required_wh)
    m.addCons(q2<=.1*cap*tail_active)
    m.addCons(q1>=max(0,.9*cap-initial_wh)*tail_active)
    m.setObjective(q1/fast+q2/tail)
    m.optimize()
    if m.getNSols()==0:return {'status':str(m.getStatus()),'feasible':False}
    qf,qt=float(m.getVal(q1)),float(m.getVal(q2))
    phases=[];cursor=0.
    for q,rate in [(qf,fast),(qt,tail)]:
        if q>1e-8:
            dt=q/rate;power=rate*3600/eta
            assert power<=e['supply_max_W_per_vehicle']+1e-8
            phases.append(dict(start_s=cursor,end_s=cursor+dt,input_power_W=power,stored_Wh=q));cursor+=dt
    return dict(status=str(m.getStatus()),feasible=True,initial_Wh=initial_wh,launch_Wh=float(m.getVal(total)),
        duration_s=cursor,phases=phases,charge_energy_Wh=qf+qt,input_energy_Wh=(qf+qt)/eta)
