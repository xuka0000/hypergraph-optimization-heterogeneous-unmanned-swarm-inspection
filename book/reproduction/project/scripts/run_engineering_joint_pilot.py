"""Bounded implementation check, NOT a complete fleet model or comparison.

Real regional coordinates, all nine tasks of one deterministic tower. A parked
carrier makes recovery and charging verifiable before ground/fleet coupling.
"""
from pathlib import Path
import copy, datetime, hashlib, json, math, sys, time
import numpy as np
from scipy.interpolate import LinearNDInterpolator
from pyproj import Transformer

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from Agent.Publication.engineering_joint_execution import (
    Geometry, xyz, state, route_scip, safe_transfer, optimize_viewpoint,
    camera_rotation, camera_metrics, reference_power, partial_charge_scip,
    required_thrust)

OUT=ROOT/'总结/03_实验数据/工程化联合求解'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,d):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def weather_function(scene,start):
    records={(r['grid_id'],r['time_utc'],r['height_agl_m']):r for r in scene['weather']['records']}
    cells={r['grid_id']:(r['grid_latitude'],r['grid_longitude']) for r in scene['weather']['records']}
    tr=Transformer.from_crs(scene['coordinate_reference']['projected'],4326,always_xy=True)
    def evaluate(pos,seconds,ground):
        lon,lat=tr.transform(pos[0],pos[1])
        grid=min(cells,key=lambda k:(cells[k][0]-lat)**2+((cells[k][1]-lon)*math.cos(math.radians(lat)))**2)
        stamp=(start+datetime.timedelta(seconds=float(seconds))).replace(minute=0,second=0,microsecond=0).strftime('%Y-%m-%dT%H:00Z')
        height=max(30.,pos[2]-ground);layers=[30,60,100]
        if height>100+1e-8:raise ValueError('above weather model ceiling')
        lo=max(h for h in layers if h<=height);hi=min(h for h in layers if h>=height)
        r0,r1=records[(grid,stamp,lo)],records[(grid,stamp,hi)]
        if not (r0['reference_flight_allowed'] and r1['reference_flight_allowed']):raise ValueError('weather flight window violated')
        w=(height-lo)/(hi-lo) if hi>lo else 0.
        get=lambda key:(1-w)*r0[key]+w*r1[key]
        return np.array([get('wind_east_ms'),get('wind_north_ms'),0.]),get('air_density_proxy_kg_m3')
    return evaluate


def main():
    if (OUT/'pilot_results.json').exists():
        old=json.loads((OUT/'pilot_results.json').read_text(encoding='utf-8'))
        vp=OUT/'pilot_independent_validation.json'
        validation=json.loads(vp.read_text(encoding='utf-8')) if vp.exists() else {}
        if '--repair' not in sys.argv or not (old['diagnostic_violations'] or validation.get('independent_failures')):
            raise RuntimeError('Do not duplicate completed pilot; inspect its identity')
        old['independent_validation']=validation
        hp=OUT/'pilot_attempts.json';history=json.loads(hp.read_text(encoding='utf-8')) if hp.exists() else []
        history.append(old);save(hp,history)
    started=time.perf_counter()
    scene_path=ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes/rommerskirchen.json'
    scene=json.loads(scene_path.read_text(encoding='utf-8'));profile=scene['joint_reference_profile']
    base=scene['fleet']['vehicles'][0]['initial_xyz_m']
    tower=min(scene['towers'],key=lambda t:(t['easting_m']-base[0])**2+(t['northing_m']-base[1])**2)
    tasks=copy.deepcopy([t for t in scene['inspection_tasks'] if t['tower_id']==tower['id']])
    stop=min(scene['stops'],key=lambda t:(t['xy_m'][0]-tower['easting_m'])**2+(t['xy_m'][1]-tower['northing_m'])**2)
    anchor=np.array(stop['xy_m']+[stop['ground_m']+profile['vehicle']['pad_offset_m'][2]])
    start=datetime.datetime(2025,7,1,8,tzinfo=datetime.timezone.utc)
    restrictions=['one tower, all nine required observations','one sortie; task sequence is free',
        'one carrier parked at a source stop, no driving','one aircraft and one battery pair',
        'same launch/recovery stop','septic rest-to-rest transfer initialization, at most3 subsegments',
        'camera look-at rotation derived, focal length fixed to wide reference',
        'reference M300 energy proxy, not transferred M100 calibration',
        'ground contact exemption only inside vertical departure/arrival column',
        'sampled force/body-rate/weather checks; no full continuous safety certificate']
    protocol=dict(task_id='inspection_joint_execution',case='regional_nine_task_parked_carrier_pilot',
        frozen_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),scene=scene['scene_id'],scene_sha256=sha(scene_path),
        tower_id=tower['id'],selection='nearest tower to original V1 depot; no selection by solver results',
        stop_id=stop['id'],start_UTC=start.isoformat(),tasks=[t['id'] for t in tasks],
        initial_soc_conditions=[1.,.2],conditions_role='reference and charging-boundary diagnostic, not a stress benchmark',
        restrictions=restrictions,formal_comparison=False,complete_joint_solver=False,field_calibrated=False,
        scipy_viewpoint_iterations=2,SCIP_gap=.05,SCIP_time_limit_s=None,
        termination='finite two alternating geometry/sequence updates; no global joint gap claim')
    save(OUT/'pilot_protocol.json',protocol)
    # Explicit execution overlay: opaque reference surfaces must be on the
    # exterior of the reference collision primitive. Original source unchanged.
    overlay=[];geometry=Geometry(scene)
    for task in tasks:
        target=task['target'];n=np.array(target['normal_world']);old=xyz(target)
        radial=np.linalg.norm(old[:2]-[tower['easting_m'],tower['northing_m']])
        # A 0.5m square cannot be treated as the centreline of a 0.05m wire.
        # Move its bounding sphere minimally outside the DECLARED primitives.
        # This is an explicit simulation-model repair, never a measured offset.
        patch_radius=math.sqrt(.25**2+.25**2)
        shift=None
        for trial in np.linspace(0,3.,601):
            trial_q=old+n*trial
            if geometry.segment_margin(trial_q,trial_q,patch_radius+.01)>=0:
                shift=float(trial);break
        if shift is None:raise ValueError('No exterior reference target patch in declared repair bound')
        new=old+n*shift
        target.update(easting_m=float(new[0]),northing_m=float(new[1]),altitude_m=float(new[2]))
        target['surface_samples_utm_m']=(np.array(target['surface_samples_utm_m'])+n*shift).tolist()
        overlay.append(dict(task_id=task['id'],shift_m=shift,old_target_xyz_m=old.tolist(),new_target_xyz_m=new.tolist(),
            reason='external reference target surface; does not claim surveyed tower geometry'))
    save(OUT/'target_surface_execution_overlay.json',{'items':overlay,'original_scene_modified':False})
    tr=Transformer.from_crs(4326,scene['coordinate_reference']['projected'],always_xy=True)
    dem=scene['terrain']['samples'];x,y=tr.transform([r['location']['lng'] for r in dem],[r['location']['lat'] for r in dem])
    terrain=LinearNDInterpolator(np.column_stack([x,y]),[r['elevation'] for r in dem])
    n=len(tasks);points=[xyz(t) for t in tasks]+[anchor,anchor]
    dwell=[t['observation_requirement']['min_dwell_s'] for t in tasks]
    history=[];legs={};seq=list(range(n));view_updates=[]
    for idx in range(n):
        candidate,receipt=optimize_viewpoint(tasks[idx],anchor,anchor,profile,geometry)
        points[idx]=candidate;view_updates.append(dict(iteration='initial_feasibility',task=tasks[idx]['id'],**receipt))
    for iteration in range(3):
        if iteration:
            order=[n]+seq+[n+1]
            for k,idx in enumerate(seq,1):
                candidate,receipt=optimize_viewpoint(tasks[idx],points[order[k-1]],points[order[k+1]],profile,geometry)
                # Preserve a checked view if nonlinear local optimization fails.
                if geometry.segment_margin(candidate,candidate,4.6)>=-1e-7:points[idx]=candidate
                view_updates.append(dict(iteration=iteration,task=tasks[idx]['id'],**receipt))
        costs={};legs={}
        for i in range(n+1):
            for j in list(range(n))+[n+1]:
                if i==j or (i==n and j==n+1):continue
                leg=safe_transfer(points[i],points[j],geometry,tower['ground_elevation_m'],contact=(i==n or j==n+1))
                if leg is not None:legs[i,j]=leg;costs[i,j]=leg['duration_s']
        seq,receipt=route_scip(costs,dwell,OUT/f'sequence_{iteration}.log')
        history.append(receipt);print('sequence',iteration,receipt,flush=True)
        if seq is None:
            save(OUT/'pilot_failure.json',{'stage':'route','history':history,'view_updates':view_updates,
                'outdegree':{str(i):sum(a==i for a,b in costs) for i in range(n+1)},
                'points':[p.tolist() for p in points]});return
    order=[n]+seq+[n+1]
    timeline=[];clock=0.;captures=[]
    for i,j in zip(order,order[1:]):
        leg=legs[i,j]
        for a,b,T in zip(leg['points'],leg['points'][1:],leg['durations_s']):
            timeline.append(dict(kind='transfer',start_s=clock,end_s=clock+T,from_xyz_m=a,to_xyz_m=b,
                control_points_m=[a]*4+[b]*4,departure_contact=bool(i==n and clock==0),arrival_contact=bool(j==n+1 and b==anchor.tolist())))
            clock+=T
        if j<n:
            captures.append(dict(task_id=tasks[j]['id'],task_index=j,start_s=clock,end_s=clock+dwell[j],position_xyz_m=points[j].tolist()))
            timeline.append(dict(kind='capture',start_s=clock,end_s=clock+dwell[j],from_xyz_m=points[j].tolist(),to_xyz_m=points[j].tolist(),
                control_points_m=[points[j].tolist()]*8,departure_contact=False,arrival_contact=False))
            clock+=dwell[j]
    weather=weather_function(scene,start)
    air=profile['aircraft'];energy=0.;trace=[];maxima={'ground_speed_mps':0.,'air_speed_mps':0.,'vertical_speed_mps':0.,'acceleration_mps2':0.,'jerk_mps3':0.,'tilt_deg':0.,'thrust_N':0.}
    min_ground=float('inf');failures=[]
    # All points use the actual source weather layers and time. Integration
    # splits at hour boundaries (including any later charge-dependent shift).
    for segment in timeline:
        a,b=np.array(segment['from_xyz_m']),np.array(segment['to_xyz_m']);T=segment['end_s']-segment['start_s']
        xs=np.linspace(0,1,max(65,int(T/.5)+1));pos,vel,acc,jerk=state(a,b,T,xs)
        grounds=np.asarray(terrain(pos[:,:2])).reshape(-1)
        if not np.isfinite(grounds).all():raise ValueError('terrain outside source hull')
        powers=[]
        for ix,(p,v,ac,jk,g) in enumerate(zip(pos,vel,acc,jerk,grounds)):
            t=segment['start_s']+xs[ix]*T
            try:w,rho=weather(p,t,g)
            except (KeyError,ValueError) as exc:
                failures.append({'family':'weather','seconds':float(t),'reason':str(exc)});continue
            va=v-w;drag=-.5*rho*air['drag_CdA_m2']*np.linalg.norm(va)*va
            thrust=required_thrust(air['mass_kg'],ac,drag)
            values=[np.linalg.norm(v),np.linalg.norm(va),abs(v[2]),np.linalg.norm(ac),np.linalg.norm(jk),thrust['tilt_deg'],thrust['thrust_n']]
            for key,value in zip(maxima,values):maxima[key]=max(maxima[key],float(value))
            clearance=p[2]-g-air['body_radius_m'];min_ground=min(min_ground,float(clearance))
            contact=(segment['departure_contact'] or segment['arrival_contact']) and np.linalg.norm(p[:2]-anchor[:2])<1e-5
            required=.01 if contact else air['obstacle_clearance_m']+air['tracking_error_m']+air['map_error_m']
            if clearance<required-1e-5:failures.append({'family':'ground_clearance','seconds':float(t),'slack_m':float(clearance-required)})
            powers.append(float(reference_power(v,rho,w,profile)))
        if len(powers)!=len(xs):continue
        integral=float(np.trapz(powers,xs*T)/3600)
        energy+=integral;segment['energy_Wh']=integral
        trace.append({'end_s':segment['end_s'],'cumulative_Wh':energy})
    limits=dict(ground_speed_mps=air['ground_speed_max_mps'],air_speed_mps=air['air_speed_max_mps'],vertical_speed_mps=air['vertical_speed_max_mps'],
        acceleration_mps2=air['acceleration_max_mps2'],jerk_mps3=air['jerk_max_mps3'],tilt_deg=air['tilt_max_deg'],thrust_N=air['thrust_max_N'])
    for k,v in maxima.items():
        if v>limits[k]+1e-6:failures.append({'family':k,'actual':v,'limit':limits[k]})
    # No clipping hides infeasible energy demands.
    reserve=profile['energy']['minimum_SOC']*profile['energy']['pair_capacity_Wh']
    if energy+reserve>profile['energy']['pair_capacity_Wh']:failures.append({'family':'battery','required_Wh':energy+reserve})
    charging=[]
    for soc in protocol['initial_soc_conditions']:
        c=partial_charge_scip(soc*profile['energy']['pair_capacity_Wh'],energy+reserve,profile,OUT/f'charge_{soc}.log')
        c['initial_SOC']=soc;charging.append(c)
        if c['feasible']:
            c['launch_s']=c['duration_s']+profile['recovery']['preparation_seconds']
            c['complete_recovery_s']=c['launch_s']+clock+profile['recovery']['recovery_seconds']
            # Weather layers are constant within each source hour. A changed
            # hour needs a new coupled power evaluation, not old energy reuse.
            keys=lambda t:(start+datetime.timedelta(seconds=float(t))).strftime('%Y-%m-%dT%H')
            c['same_weather_hour_verified']=all(keys(c['launch_s']+seg[k])==keys(seg[k])
                for seg in timeline for k in ['start_s','end_s'])
            if not c['same_weather_hour_verified']:failures.append({'family':'charge_weather_coupling','initial_SOC':soc,'reason':'crosses weather boundary; must recompute power and reschedule'})
    result=dict(status='restricted_pilot_candidate_requires_independent_replay',protocol_sha256=sha(OUT/'pilot_protocol.json'),
        scene=scene['scene_id'],tower_id=tower['id'],task_count=n,sequence=[tasks[i]['id'] for i in seq],
        flight_seconds=clock,aircraft_energy_Wh=energy,energy_reserve_Wh=reserve,
        solver_history=history,viewpoint_updates=view_updates,timeline=timeline,captures=captures,
        targets=tasks,charging_cases=charging,sampled_maxima=maxima,limits=limits,min_ground_body_clearance_m=min_ground,
        diagnostic_violations=failures,restrictions=restrictions,complete_joint_solver=False,
        geometry_occlusion_certified=False,calibrated_M300_energy=False,
        emergency_return_witness='remaining planned collision-checked path to same stationary carrier; no separate optimum-return claim',
        elapsed_seconds=time.perf_counter()-started)
    save(OUT/'pilot_results.json',result)
    print(json.dumps({k:result[k] for k in ['task_count','flight_seconds','aircraft_energy_Wh','sampled_maxima','elapsed_seconds']},ensure_ascii=False))
    print('violations',len(failures),'complete_joint_solver',False)


if __name__=='__main__':main()
