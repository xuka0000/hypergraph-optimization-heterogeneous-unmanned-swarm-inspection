"""Three-piece C3 transit proposals in the unchanged degree7 flight domain.

The old zero-speed-at-every-piece initializer wastes time on long transfers.
These proposals retain nonzero velocity at internal joins. They are accepted
only by the existing geometry and continuous physics checks, never by this file.
"""
import math
import numpy as np
from Agent.Publication.engineering_regional_curves import control_points
from Agent.Publication.engineering_regional_geometry import restrict
from Agent.Publication.engineering_regional_seed import smooth_controls, duration


def cruise_controls(first,last,pose0,pose1,height,speed=4.5):
    """Acceleration/climb, cruise, deceleration/descent with shared p,v,a,j."""
    first=np.asarray(first,float);last=np.asarray(last,float)
    delta=last[:2]-first[:2];distance=float(np.linalg.norm(delta))
    if distance<1e-6:return None
    direction=delta/distance;dz0=height-first[2];dz1=height-last[2]
    def ramp_time(dz,v):
        return max(.02,3*v/2,math.sqrt(15*v/3),2.1875*abs(dz)/.8,
                   math.sqrt(7.6*abs(dz)/2),(53*abs(dz)/3)**(1/3))
    v=float(speed)
    for _ in range(8):
        t0,t2=ramp_time(dz0,v),ramp_time(dz1,v)
        nxt=min(v,1.9*distance/(t0+t2))
        if abs(nxt-v)<1e-10:break
        v=nxt
    t0,t2=ramp_time(dz0,v),ramp_time(dz1,v)
    remaining=distance-v*(t0+t2)/2
    if remaining<=0 or v<=0:return None
    t1=remaining/v;times=[t0,t1,t2]
    if min(times)<.001:return None
    p1=np.r_[first[:2]+direction*v*t0/2,height]
    p2=np.r_[last[:2]-direction*v*t2/2,height]
    velocity=np.r_[direction*v,0.];zero=np.zeros(3)
    nodes=[[first,zero,zero,zero],[p1,velocity,zero,zero],
           [p2,velocity,zero,zero],[last,zero,zero,zero]]
    # Translate before converting physical derivatives to controls to avoid
    # cancellation of million-metre UTM coordinates.
    controls=[]
    for a,b,T in zip(nodes,nodes[1:],times):
        left=[a[0]-first,*a[1:]];right=[b[0]-first,*b[1:]]
        controls.append(np.asarray(control_points(left,right,T))+first)
    total=sum(times)
    angle_time=duration(first,first,np.asarray(pose0),np.asarray(pose1))
    scale=max(1.,angle_time/total)
    times=[T*scale for T in times];total=sum(times)
    camera=smooth_controls(pose0,pose1);elapsed=0.;result=[]
    for C,T in zip(controls,times):
        A=restrict(camera,elapsed/total,(elapsed+T)/total)
        result.append(dict(C=C,A=A,duration_s=T));elapsed+=T
    return result


def safe_cruise(ctx,first,last,pose0,pose1,geometry_cache=None):
    """Choose a geometric proposal; all dynamics remain to be checked."""
    if math.dist(first[:2],last[:2])<60:return None
    base=max(first[2],last[2])
    for dz in [0.,4.,8.,12.,18.,24.]:
        proposal=cruise_controls(first,last,pose0,pose1,base+dz)
        if proposal is None:continue
        passed=True
        for seg in proposal:
            c=seg['C'];key=(tuple(c.ravel()),None)
            result=geometry_cache.get(key) if geometry_cache is not None else None
            if result is None:
                result=ctx.geometry.segment(c)
                if geometry_cache is not None:geometry_cache[key]=result
            if not result['passed']:passed=False;break
            try:
                ground,_=ctx.enclosure.terrain_enclosure(c)
                if float(c[:,2].max()-ground.lo)>100:passed=False;break
            except (ValueError,ArithmeticError):passed=False;break
        if passed:return proposal
    return None
