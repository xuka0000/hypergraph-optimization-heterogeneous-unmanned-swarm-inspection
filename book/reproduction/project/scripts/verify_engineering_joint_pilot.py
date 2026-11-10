"""Independent evaluation; never repairs or optimizes a candidate solution."""
from pathlib import Path
import argparse, datetime, hashlib, json, math, sys
import numpy as np
from scipy.interpolate import LinearNDInterpolator
from scipy.spatial.transform import Rotation
from pyproj import Transformer

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from Agent.Publication.engineering_joint_execution import Geometry
from run_engineering_joint_pilot import weather_function
OUT=ROOT/'总结/03_实验数据/工程化联合求解'


def solid_cylinder_intersection(a,b,c):
    # Intersect radial quadratic and vertical slab for t in the open sightline.
    v=b-a;w=a[:2]-np.array(c['center_xy_m']);A=v[:2]@v[:2];B=2*w@v[:2];C=w@w-c['radius_m']**2
    if A<1e-20:
        if C>0:return False
        lo,hi=0.,1.
    else:
        disc=B*B-4*A*C
        if disc<=0:return False
        lo,hi=sorted([(-B-math.sqrt(disc))/(2*A),(-B+math.sqrt(disc))/(2*A)])
    if abs(v[2])<1e-15:
        if not c['base_elevation_m']<a[2]<c['base_elevation_m']+c['height_m']:return False
    else:
        z0,z1=sorted([(c['base_elevation_m']-a[2])/v[2],(c['base_elevation_m']+c['height_m']-a[2])/v[2]])
        lo=max(lo,z0);hi=min(hi,z1)
    return max(lo,1e-8)<min(hi,1-1e-8)


def eval_septic(c,T,x):
    # Evaluate the exported Bernstein controls directly, independent of runner's smoothstep.
    states=[];c=np.asarray(c,float)
    for order in range(4):
        n=len(c)-1
        b=np.array([math.comb(n,k)*x**k*(1-x)**(n-k) for k in range(n+1)])
        states.append(b@c)
        c=n*np.diff(c,axis=0)/T
    return states


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--candidate',type=Path,default=OUT/'pilot_results.json')
    parser.add_argument('--protocol',type=Path,default=OUT/'pilot_protocol.json');parser.add_argument('--output',type=Path,default=OUT/'pilot_independent_validation.json')
    args=parser.parse_args();p=args.candidate;d=json.loads(p.read_text(encoding='utf-8'))
    protocol=json.loads(args.protocol.read_text(encoding='utf-8'))
    s=json.loads((ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes/rommerskirchen.json').read_text(encoding='utf-8'))
    cfg=s['joint_reference_profile'];camera=cfg['camera'];air=cfg['aircraft'];en=cfg['energy'];geo=Geometry(s)
    tr=Transformer.from_crs(4326,s['coordinate_reference']['projected'],always_xy=True)
    dem=s['terrain']['samples'];x,y=tr.transform([r['location']['lng'] for r in dem],[r['location']['lat'] for r in dem])
    terrain=LinearNDInterpolator(np.column_stack([x,y]),[r['elevation'] for r in dem])
    tri=terrain.tri.simplices;coords=terrain.points[tri]
    terrain_bounds=dict(lo=coords.min(axis=1),hi=coords.max(axis=1),max_z=terrain.values[:,0][tri].max(axis=1))
    start=datetime.datetime.fromisoformat(protocol['start_UTC']);weather=weather_function(s,start)
    tasks={t['id']:t for t in d['targets']};camrows=[];fail=[]
    fx=math.hypot(camera['image_width_px'],camera['image_height_px'])/(2*math.tan(math.radians(camera['diagonal_fov_deg_at_wide']/2)))
    for capture in d['captures']:
        t=tasks[capture['task_id']];pos=np.array(capture['position_xyz_m']);target=t['target']
        q=np.array([target[k] for k in ['easting_m','northing_m','altitude_m']]);look=(q-pos)/np.linalg.norm(q-pos)
        az=math.atan2(look[1],look[0]);wind,rho=weather(pos,capture['start_s'],float(terrain(*pos[:2])))
        va=-wind;drag=-.5*rho*air['drag_CdA_m2']*np.linalg.norm(va)*va
        f=np.array([0,0,air['mass_kg']*9.81])-drag;b3=f/np.linalg.norm(f)
        b2=np.cross(b3,[math.cos(az),math.sin(az),0]);b2/=np.linalg.norm(b2);b1=np.cross(b2,b3);rb=np.column_stack([b1,b2,b3])
        optical=rb.T@look;gy=math.atan2(optical[1],optical[0]);gp=math.atan2(optical[2],np.linalg.norm(optical[:2]))
        right=np.array([math.sin(gy),-math.cos(gy),0.]);down=np.cross(optical,right)
        rc=rb@np.column_stack([right,down,optical]);tan=np.array(target['tangent_basis']).T
        gsds=[];fov=[];cylinder_blocked=[];wire_blocked=[]
        for sample in target['surface_samples_utm_m']:
            pt=np.array(sample);xc,yc,zc=rc.T@(pt-pos)
            pix=np.array([fx*xc/zc+camera['image_width_px']/2,fx*yc/zc+camera['image_height_px']/2])
            jac=np.array([[fx/zc,0,-fx*xc/zc**2],[0,fx/zc,-fx*yc/zc**2]])@rc.T@tan
            gsd=1/np.linalg.svd(jac,compute_uv=False)[-1];gsds.append(float(gsd))
            margin=camera['frame_border_px'];fov.append(bool(zc>0 and margin<=pix[0]<=camera['image_width_px']-margin and margin<=pix[1]<=camera['image_height_px']-margin))
            ids=geo.tree.query_ball_point((pos[:2]+pt[:2])/2,np.linalg.norm(pt-pos)/2+3)
            cylinder_blocked.extend(c['id'] for c in (geo.cyl[i] for i in ids) if solid_cylinder_intersection(pos,pt,c))
            # Conservative chord-inflated wire check; cylinder sightline uses
            # exact intersection above rather than collision clearance margins.
            from Agent.Publication.engineering_joint_execution import closest_segments
            center=(pos+pt)/2;radius=np.linalg.norm(pt-pos)/2
            close=np.flatnonzero(np.linalg.norm(geo.centers-center,axis=1)<=geo.radii+radius+.2)
            for i in close:
                points,r,wid=geo.wires[i]
                if any(closest_segments(pos,pt,a,b)<r-1e-8 for a,b in zip(points,points[1:])):wire_blocked.append(wid)
        row=dict(task_id=t['id'],max_GSD_m_per_px=max(gsds),all_samples_in_FOV=all(fov),gimbal_pitch_deg=math.degrees(gp),gimbal_yaw_deg=math.degrees(gy),
            cylinder_occluders=sorted(set(cylinder_blocked)),wire_occluders=sorted(set(wire_blocked)),
            ideal_stationary_exposure_blur_px=0.,image_quality_measured=False)
        distance=float(np.linalg.norm(pos-q));incidence=float(np.dot((pos-q)/distance,target['normal_world']))
        row.update(observation_distance_m=distance,incidence_angle_deg=math.degrees(math.acos(np.clip(incidence,-1,1))))
        row['reference_optical_pass']=(all(fov) and max(gsds)<=camera['gsd_max_m_per_px'] and not cylinder_blocked and not wire_blocked
            and camera['focus_min_m']-1e-7<=distance<=camera['observation_max_m']+1e-7
            and incidence>=math.cos(math.radians(camera['incidence_angle_max_deg']))-1e-7
            and camera['gimbal_pitch_deg'][0]<=math.degrees(gp)<=camera['gimbal_pitch_deg'][1]
            and camera['gimbal_yaw_deg'][0]<=math.degrees(gy)<=camera['gimbal_yaw_deg'][1]
            and capture['end_s']-capture['start_s']>=t['observation_requirement']['min_dwell_s']-1e-7)
        if not row['reference_optical_pass']:fail.append({'family':'optical','task':t['id']})
        camrows.append(row)
    # Independent higher-order energy integration and control endpoint continuity.
    nodes,weights=np.polynomial.legendre.leggauss(24);energy=0.;join_error=0.;previous=None
    maxima=dict(ground_speed_mps=0.,air_speed_mps=0.,vertical_speed_mps=0.,acceleration_mps2=0.,jerk_mps3=0.,tilt_deg=0.,thrust_N=0.)
    limits=dict(ground_speed_mps=air['ground_speed_max_mps'],air_speed_mps=air['air_speed_max_mps'],vertical_speed_mps=air['vertical_speed_max_mps'],
        acceleration_mps2=air['acceleration_max_mps2'],jerk_mps3=air['jerk_max_mps3'],tilt_deg=air['tilt_max_deg'],thrust_N=air['thrust_max_N'])
    collision_margins=[];min_ground=float('inf');anchor=np.array(d['timeline'][0]['from_xyz_m']);sample_count=0
    for seg in d['timeline']:
        T=seg['end_s']-seg['start_s'];c=np.array(seg['control_points_m'])
        left=eval_septic(c,T,0.);right=eval_septic(c,T,1.)
        if previous is not None:join_error=max(join_error,max(np.linalg.norm(a-b) for a,b in zip(previous,left)))
        previous=right
        # The current controls lie on their endpoint line segment. For general
        # free controls this line certificate is invalid and must be replaced.
        a,b=c[0],c[-1]
        clearance=air['body_radius_m']+air['obstacle_clearance_m']+air['tracking_error_m']+air['map_error_m']
        if not np.allclose(c,np.array([a]*4+[b]*4),atol=1e-9,rtol=0):
            from verify_free_trajectory_block import geometry_certificate
            passed,depth=geometry_certificate(c,geo,clearance,terrain,terrain_bounds,seg.get('departure_contact',False) or seg.get('arrival_contact',False))
            if not passed:fail.append({'family':'adaptive_Bernstein_geometry_certificate'})
            margin=float('inf') # The endpoint chord is not the free curve.
        else:margin=geo.segment_margin(a,b,clearance)
        if math.isfinite(margin):collision_margins.append(margin)
        if margin < -1e-7:fail.append({'family':'line_envelope_collision','margin_m':margin})
        for x in np.linspace(0,1,max(65,int(math.ceil(T/.25))+1)):
            pos,vel,acc,jerk=eval_septic(c,T,x);ground=float(terrain(*pos[:2]));sample_count+=1
            wind,rho=weather(pos,seg['start_s']+x*T,ground);va=vel-wind
            drag=-.5*rho*air['drag_CdA_m2']*np.linalg.norm(va)*va;force=air['mass_kg']*(acc+[0,0,9.81])-drag
            values=[np.linalg.norm(vel),np.linalg.norm(va),abs(vel[2]),np.linalg.norm(acc),np.linalg.norm(jerk),
                math.degrees(math.acos(np.clip(force[2]/np.linalg.norm(force),-1,1))),np.linalg.norm(force)]
            for key,value in zip(maxima,values):maxima[key]=max(maxima[key],float(value))
            clearance=pos[2]-ground-air['body_radius_m'];min_ground=min(min_ground,clearance)
            contact=(seg.get('departure_contact') or seg.get('arrival_contact')) and np.linalg.norm(pos[:2]-anchor[:2])<1e-5
            need=.01 if contact else air['obstacle_clearance_m']+air['tracking_error_m']+air['map_error_m']
            if not math.isfinite(clearance) or clearance<need-1e-5:fail.append({'family':'ground_clearance','slack_m':clearance-need})
        # Split at vertical weather layer crossings as well as hour boundaries.
        # The high-order integral is a numerical convergence cross-check.
        val=0.
        for nd,wt in zip(nodes,weights):
            x=(nd+1)/2;pos,vel,acc,jerk=eval_septic(c,T,x)
            wind,rho=weather(pos,seg['start_s']+x*T,float(terrain(*pos[:2])))
            power=en['hover_power_W']*math.sqrt(1.225/rho)+.5*air['drag_CdA_m2']*rho*np.linalg.norm(vel-wind)**3+air['mass_kg']*9.81*max(vel[2],0)/en['propulsion_efficiency']
            val+=wt*power*T/7200
        energy+=val
    difference=abs(energy-d['aircraft_energy_Wh'])
    for k,value in maxima.items():
        if value>limits[k]+1e-5:fail.append({'family':k,'actual':value,'limit':limits[k]})
    if difference>cfg['numerical']['energy_tolerance_Wh']:fail.append({'family':'energy_integration','difference_Wh':difference})
    if join_error>1e-6:fail.append({'family':'C3_continuity','max_error':join_error})
    for c in d['charging_cases']:
        if not c['feasible']:fail.append({'family':'charge_infeasible','SOC':c['initial_SOC']});continue
        stored=sum((r['end_s']-r['start_s'])*r['input_power_W']*en['charge_efficiency']/3600 for r in c['phases'])
        if abs(c['initial_Wh']+stored-c['launch_Wh'])>1e-8:fail.append({'family':'charge_Wh_conservation'})
        if c['launch_Wh']-energy<d['energy_reserve_Wh']-.01:fail.append({'family':'terminal_energy','SOC':c['initial_SOC']})
    result=dict(observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),candidate_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
        camera_rows=camrows,photo_tasks=len(camrows),reference_optical_pass=sum(r['reference_optical_pass'] for r in camrows),
        independent_energy_Wh=energy,energy_quadrature_delta_Wh=difference,max_C3_join_error=join_error,
        sampled_maxima=maxima,limits=limits,sample_count=sample_count,min_ground_body_clearance_m=min_ground,
        min_obstacle_line_envelope_margin_m=min(collision_margins) if collision_margins else None,
        independent_failures=fail,runner_failures=len(d['diagnostic_violations']) if 'diagnostic_violations' in d else None,
        evaluated_restricted_pilot_pass=not fail and not d.get('diagnostic_violations',[]),
        complete_joint_solver=False,full_body_gimbal_rate_trajectory_checked=False,field_ready=False,
        status='partial_implementation_validation; missing constraints are not counted as pass')
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='camera_rows'},ensure_ascii=False))
    if not result['evaluated_restricted_pilot_pass']:raise SystemExit(1)


if __name__=='__main__':main()
