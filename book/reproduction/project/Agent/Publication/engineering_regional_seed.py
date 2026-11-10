"""Numerical warm-start construction; never fixes joint optimization choices.

Reference geography and all safety/optical requirements are evaluated by the
same independent continuous oracles used to reject solver candidates.
"""
import copy,math
from types import SimpleNamespace
import numpy as np
from pyproj import Transformer
from scipy.interpolate import LinearNDInterpolator
from Agent.Publication.engineering_regional_structure import load_regional
from Agent.Publication.engineering_regional_weather import retained_regional_forecast
from Agent.Publication.engineering_regional_safety import RegionalSafety
from Agent.Publication.engineering_regional_geometry import RegionalGeometry
from Agent.Publication.engineering_regional_enclosure import RegionalEnclosure


def numeric_context(root):
    scene,record=load_regional(root)
    tr=Transformer.from_crs(4326,scene['coordinate_reference']['projected'],always_xy=True)
    samples=scene['terrain']['samples']
    x,y=tr.transform([q['location']['lng'] for q in samples],[q['location']['lat'] for q in samples])
    terrain=LinearNDInterpolator(np.column_stack([x,y]),[q['elevation'] for q in samples])
    origin=np.asarray(scene['fleet']['vehicles'][0]['initial_xyz_m'])
    curves=SimpleNamespace(m=None,path=None,rel=None,origin=origin,scene=scene)
    safety=RegionalSafety(curves,None,None,None);forecast=retained_regional_forecast(root)
    enclosure=RegionalEnclosure(scene,terrain,forecast)
    targets=[]
    for source,r in zip(scene['inspection_tasks'],record['corrections']):
        t=copy.deepcopy(source)
        for q,key in enumerate(['easting_m','northing_m','altitude_m']):
            t[key]=r['viewpoint_center_xyz_m'][q];t['target'][key]=r['target_xyz_m'][q]
        t['target']['surface_samples_utm_m']=r['surface_samples_utm_m'];targets.append(t)
    return SimpleNamespace(scene=scene,record=record,terrain=terrain,forecast=forecast,
        enclosure=enclosure,safety=safety,geometry=RegionalGeometry(safety,enclosure),targets=targets)


def camera_pose(ctx,xyz,target,clock,yaw=0.,zoom=1.5):
    agl=float(xyz[2]-ctx.terrain(*xyz[:2]));W=ctx.forecast.evaluate(clock,agl)['values']
    va=-np.array([W['wind_east_ms'],W['wind_north_ms'],0.])
    force=np.array([0.,0.,7*9.81])+.5*.25*W['density_kg_m3']*np.linalg.norm(va)*va
    b3=force/np.linalg.norm(force);cs,sn=math.cos(yaw),math.sin(yaw)
    b1=np.array([b3[2]*cs,b3[2]*sn,-b3[0]*cs-b3[1]*sn]);b1/=np.linalg.norm(b1);b2=np.cross(b3,b1)
    look=np.asarray(target)-xyz;look/=np.linalg.norm(look)
    local=np.column_stack([b1,b2,b3]).T@look
    return np.array([yaw,math.asin(np.clip(local[2],-1,1)),math.atan2(local[1],local[0]),zoom])


def smooth_controls(a,b):return np.array([a]*4+[b]*4,float)


def numeric_power(ctx,c,T,start,s):
    from Agent.Publication.engineering_regional_geometry import evaluate
    c=np.asarray(c);local=c-c[0];xyz=evaluate(local,s)+c[0]
    v=evaluate(7*np.diff(local,axis=0)/T,s)
    h=float(xyz[2]-ctx.terrain(*xyz[:2]));W=ctx.forecast.evaluate(start+s*T,h)['values']
    u=v-np.array([W['wind_east_ms'],W['wind_north_ms'],0.]);rho=W['density_kg_m3']
    cfg=ctx.scene['joint_reference_profile'];air=cfg['aircraft'];energy=cfg['energy']
    return energy['hover_power_W']*np.sqrt(1.225/rho)+.5*air['drag_CdA_m2']*rho*np.linalg.norm(u)**3+air['mass_kg']*9.81*max(0.,v[2])/energy['propulsion_efficiency']


def duration(a,b,pa,pb):
    delta=np.asarray(b)-a;d=float(np.linalg.norm(delta));angle=float(np.max(abs(np.asarray(pb)[:3]-np.asarray(pa)[:3])))
    zoom=abs(pb[3]-pa[3])*6.83
    if max(d,angle,zoom)<1e-9:return 0.
    return max(.01,2.1875*d/4.5,2.1875*abs(delta[2])/.9,(7.6*d/2.5)**.5,
               (53*d/4.)**(1/3),2.1875*angle/.45,(7.6*angle/.45)**.5,2.1875*zoom/8.)


def observation_candidates(ctx,index):
    task=ctx.targets[index];t=task['target'];center=np.array([t[k] for k in ['easting_m','northing_m','altitude_m']])
    normal=np.array(t['normal_world']);tangent=np.array(t['tangent_basis'][0])
    # Search initializers inside the existing distance/incidence domain. The
    # optimizer retains continuous positions; this is not a view catalogue.
    for d,z,side in [(20,8,0),(20,10,0),(22,8,0),(18,8,0),(20,6,0),(18,6,0)]+[
        (d,z,side) for d in [20,22,18,16,12] for z in [8,5,2,0,-2] for side in [3,-3,6,-6,0]]:
        xyz=center+d*normal+np.array([0,0,z])+side*tangent
        distances=np.linalg.norm(xyz-np.array(t['surface_samples_utm_m']),axis=1)
        cosines=(xyz-np.array(t['surface_samples_utm_m']))@normal/distances
        if distances.min()<8.01 or distances.max()>24.99 or cosines.min()<math.cos(math.radians(29.9)):continue
        yield xyz


def find_observation(ctx,index,escape_altitude=None):
    t=ctx.targets[index];target=ctx.record['corrections'][index]['target_xyz_m'];attempts=[]
    for xyz in observation_candidates(ctx,index):
        c=np.tile(xyz,(8,1));body=ctx.geometry.segment(c)
        if not body['passed']:
            attempts.append(dict(stage='body',result=body));continue
        if escape_altitude is not None:
            escape=ctx.geometry.segment(smooth_controls(xyz,np.r_[xyz[:2],escape_altitude]))
            if not escape['passed']:
                attempts.append(dict(stage='seed_vertical_escape',result=escape));continue
        rays=[ctx.geometry.sight_line(c,np.array(p)) for p in t['target']['surface_samples_utm_m']]
        if not all(r['passed'] for r in rays):
            attempts.append(dict(stage='visibility',results=[r for r in rays if not r['passed']]));continue
        pose=camera_pose(ctx,xyz,target,120.)
        try:
            ok,checks,physical=ctx.enclosure.check(c,t['effective_service_seconds'],np.tile(pose,(8,1)),120.)
            optical,details=ctx.enclosure.optics(c,t['effective_service_seconds'],np.tile(pose,(8,1)),t,physical)
        except (ArithmeticError,ValueError) as e:
            attempts.append(dict(stage='physical',error=str(e)));continue
        if ok and optical:return dict(passed=True,xyz_m=xyz.tolist(),pose=pose.tolist(),attempts=len(attempts)+1,
            body=body,visibility=rays,physical=checks,optical=details)
        attempts.append(dict(stage='physical',checks=checks,optical=details))
    return dict(passed=False,attempts=len(attempts),failures=attempts,not_infeasibility_proof=True)
