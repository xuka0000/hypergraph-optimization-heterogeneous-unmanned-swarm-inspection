"""Resume-safe inline engineering solver development on the authorized tower.

No daemon, no worker spawning. Preserve original depots and resource owners.
Every new physical trajectory remains a candidate until independent replay.
"""
from pathlib import Path
import os,sys,json,hashlib,datetime,time,copy,argparse
os.environ.setdefault('OMP_NUM_THREADS','1');os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
import numpy as np
import psutil
from scipy.interpolate import LinearNDInterpolator
from pyproj import Transformer
from Agent.Publication.engineering_joint_execution import Geometry,safe_transfer,state,reference_power
from Agent.Publication.engineering_multicarrier_schedule import solve
from run_engineering_joint_pilot import weather_function
from verify_engineering_joint_pilot import eval_septic
OUT=ROOT/'总结/03_实验数据/工程化联合求解'
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def save(p,d):
    temp=p.with_suffix(p.suffix+'.tmp');temp.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8');temp.replace(p)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stamp():return datetime.datetime.now(datetime.timezone.utc).isoformat()


def context():
    sp=ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes/rommerskirchen.json';scene=read(sp);pilot=read(OUT/'pilot_results.json')
    tr=Transformer.from_crs(4326,scene['coordinate_reference']['projected'],always_xy=True);dem=scene['terrain']['samples']
    x,y=tr.transform([r['location']['lng'] for r in dem],[r['location']['lat'] for r in dem]);terrain=LinearNDInterpolator(np.column_stack([x,y]),[r['elevation'] for r in dem])
    start=datetime.datetime(2025,7,1,8,tzinfo=datetime.timezone.utc);weather=weather_function(scene,start)
    return scene,pilot,terrain,weather


def energy_of(timeline,cfg,terrain,weather,offset=0.):
    energy=0.;nodes,weights=np.polynomial.legendre.leggauss(24)
    for seg in timeline:
        T=seg['end_s']-seg['start_s'];val=0.
        start=offset+seg['start_s'];end=start+T
        cuts=[start]+[float(h) for h in range((int(start//3600)+1)*3600,int(end)+1,3600) if start<h<end]+[end]
        for left,right in zip(cuts,cuts[1:]):
            for fraction,w in zip((nodes+1)/2,weights/2):
                time=left+fraction*(right-left);s=(time-start)/T
                p,v,ac,j=eval_septic(seg['control_points_m'],T,s);wind,rho=weather(p,time,float(terrain(*p[:2])))
                val+=w*(right-left)*float(reference_power(v,rho,wind,cfg))/3600
        seg['energy_Wh']=val;energy+=val
    return energy


def leg_timeline(leg,departure=False,arrival=False):
    if 'timeline' in leg:return copy.deepcopy(leg['timeline'])
    timeline=[];clock=0.
    for k,(a,b,T) in enumerate(zip(leg['points'],leg['points'][1:],leg['durations_s'])):
        timeline.append(dict(kind='transfer',start_s=clock,end_s=clock+T,from_xyz_m=a,to_xyz_m=b,control_points_m=[a]*4+[b]*4,
            departure_contact=departure and k==0,arrival_contact=arrival and k==len(leg['durations_s'])-1));clock+=T
    return timeline


def make_input(scene,pilot,terrain,weather):
    old=read(OUT/'fleet_kernel_input.json');n=len(pilot['targets']);geo=Geometry(scene);cfg=scene['joint_reference_profile'];points=old['points'][:n]
    tower=next(t for t in scene['towers'] if t['id']==pilot['tower_id']);legs=[]
    for v,vehicle in enumerate(scene['fleet']['vehicles']):
        anchor=list(vehicle['initial_xyz_m']);anchor[2]+=cfg['vehicle']['pad_offset_m'][2]
        for r in old['legs']:
            if r['i']<n and r['j']<n:legs.append(dict(vehicle=v,**copy.deepcopy(r)))
        for i,j in [(n,k) for k in range(n)]+[(k,n+1) for k in range(n)]:
            a=anchor if i==n else points[i];b=anchor if j==n+1 else points[j]
            leg=safe_transfer(np.array(a),np.array(b),geo,tower['ground_elevation_m'],contact=True)
            if leg is None:continue
            tl=leg_timeline(leg,i==n,j==n+1);wh=energy_of(tl,cfg,terrain,weather)
            legs.append(dict(vehicle=v,i=i,j=j,energy_Wh=wh,**leg))
    return dict(task_id='inspection_full_joint_execution',created_at=stamp(),scene_sha256=sha(ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes/rommerskirchen.json'),
        tasks=pilot['targets'],tower_id=pilot['tower_id'],points=points,legs=legs,dwell_s=old['dwell_s'],dwell_Wh=old['dwell_Wh'],horizon_s=3600.,
        start_UTC='2025-07-01T08:00:00+00:00',current_vehicle_block='original depots; mobile route block pending',source_geometry_overlay_sha256=sha(OUT/'target_surface_execution_overlay.json'))


def export_sorties(solution,data,scene,terrain,weather,prefix):
    n=len(data['tasks']);lookup={(r['vehicle'],r['i'],r['j']):r for r in data['legs']};exports=[]
    for sortie in solution.get('sorties',[]):
        timeline=[];captures=[];clock=0.;v=sortie['vehicle'];order=[n]+sortie['task_indices']+[n+1]
        for i,j in zip(order,order[1:]):
            leg=lookup[v,i,j];segments=leg_timeline(leg,i==n,j==n+1)
            for seg in segments:
                seg['start_s']+=clock;seg['end_s']+=clock;seg['source_arc']=[v,i,j,sortie['slot']];timeline.append(seg)
            clock+=leg['duration_s']
            if j<n:
                pos=data['points'][j];dwell=data['dwell_s'][j]
                captures.append(dict(task_id=data['tasks'][j]['id'],task_index=j,start_s=clock,end_s=clock+dwell,position_xyz_m=pos))
                timeline.append(dict(kind='capture',start_s=clock,end_s=clock+dwell,from_xyz_m=pos,to_xyz_m=pos,control_points_m=[pos]*8,departure_contact=False,arrival_contact=False,
                    source_arc=[v,j,order[order.index(j)+1],sortie['slot']],capture_offset=True));clock+=dwell
        energy=energy_of(timeline,scene['joint_reference_profile'],terrain,weather,sortie['launch_s'])
        candidate=dict(task_id='inspection_full_joint_execution',status='candidate_requires_independent_replay',task_count=len(captures),
            sequence=[data['tasks'][i]['id'] for i in sortie['task_indices']],flight_seconds=clock,aircraft_energy_Wh=energy,energy_reserve_Wh=109.6,
            timeline=timeline,captures=captures,targets=data['tasks'],charging_cases=[],source_sortie=sortie,complete_joint_solver=False)
        path=OUT/f'{prefix}_sortie{sortie["slot"]}.json';save(path,candidate)
        launch=datetime.datetime.fromisoformat(data['start_UTC'])+datetime.timedelta(seconds=sortie['launch_s'])
        protocol=dict(start_UTC=launch.isoformat(),scene_sha256=data['scene_sha256'],source_schedule_sha256=sha(OUT/f'{prefix}.json'),
            carrier_initial_heading_rad=scene['fleet']['vehicles'][v]['initial_heading_rad'])
        save(path.with_name(path.stem+'_protocol.json'),protocol);exports.append(path.name)
    return exports


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--phase',choices=['build','discrete','continuous','road'],required=True);parser.add_argument('--iteration',type=int,default=0);args=parser.parse_args()
    status=OUT/('full_joint_road_status.json' if args.phase=='road' else 'full_joint_status.json')
    if status.exists():
        prior=read(status)
        if prior.get('state')=='running' and psutil.pid_exists(prior['pid']):
            proc=psutil.Process(prior['pid'])
            if abs(proc.create_time()-prior['process_create_time'])<.01:raise RuntimeError('Existing identical executor is live')
    receipt=dict(task_id='inspection_full_joint_execution',phase=args.phase,iteration=args.iteration,state='running',pid=os.getpid(),process_create_time=psutil.Process().create_time(),observed_at=stamp())
    save(status,receipt)
    try:
        scene,pilot,terrain,weather=context();ip=OUT/f'full_joint_input_{args.iteration}.json'
        if args.phase=='build':
            if ip.exists():raise RuntimeError('Input already exists; reuse it')
            data=make_input(scene,pilot,terrain,weather);save(ip,data);print('built',len(data['legs']),'directed carrier-task legs',flush=True)
        elif args.phase=='discrete':
            data=read(ip);rp=OUT/f'full_joint_discrete_{args.iteration}.json'
            if rp.exists():raise RuntimeError('Result already exists; inspect before a new scientific or repair iteration')
            result=solve(data,scene,OUT/f'full_joint_discrete_{args.iteration}.log');save(rp,result)
            exports=export_sorties(result,data,scene,terrain,weather,rp.stem);print(json.dumps({k:v for k,v in result.items() if k not in ['sorties','terminal_pair_Wh']}),flush=True)
            receipt['exports']=exports
        elif args.phase=='continuous':
            from Agent.Publication.engineering_free_trajectory import optimize_transfer
            data=read(ip);solution=read(OUT/f'full_joint_discrete_{args.iteration}.json');n=len(data['tasks']);geo=Geometry(scene);used=set()
            for sortie in solution['sorties']:
                order=[n]+sortie['task_indices']+[n+1]
                used.update((sortie['vehicle'],i,j) for i,j in zip(order,order[1:]))
            receipts=[]
            for row in data['legs']:
                if (row['vehicle'],row['i'],row['j']) not in used:continue
                cache=OUT/f'free_leg_{row["vehicle"]}_{row["i"]}_{row["j"]}.json'
                if cache.exists():r=read(cache);timeline=r['timeline'];rec=r['receipt']
                else:
                    initial=dict(row,departure_contact=row['i']==n,arrival_contact=row['j']==n+1)
                    timeline,rec=optimize_transfer(initial,scene['joint_reference_profile'],geo,terrain,weather)
                    save(cache,dict(timeline=timeline,receipt=rec,source_input_sha256=sha(ip)))
                receipts.append(dict(vehicle=row['vehicle'],i=row['i'],j=row['j'],**rec))
                if timeline is not None:
                    row['timeline']=timeline;row['duration_s']=timeline[-1]['end_s'];row['energy_Wh']=energy_of(timeline,scene['joint_reference_profile'],terrain,weather)
                print('free leg',row['vehicle'],row['i'],row['j'],rec['status'],row['duration_s'],flush=True)
                receipt.update(completed_legs=len(receipts),observed_at=stamp());save(status,receipt)
            save(OUT/f'full_joint_continuous_{args.iteration}.json',dict(receipts=receipts,independent_replay_pending=True))
            save(OUT/f'full_joint_input_{args.iteration+1}.json',data)
        else:
            from Agent.Publication.engineering_road_motion import RoadMotion
            road=RoadMotion(scene);rows=[]
            for vehicle in scene['fleet']['vehicles']:
                path=OUT/f'full_road_{vehicle["id"]}_P012.json'
                if path.exists():result=read(path)
                else:
                    result=road.itinerary(vehicle,['P012']);save(path,result)
                rows.append(dict(vehicle=vehicle['id'],feasible=result is not None,duration_s=result['duration_s'] if result else None,length_m=result['length_m'] if result else None,visits=result.get('visit_events') if result else None))
                print(json.dumps(rows[-1]),flush=True)
            save(OUT/'full_road_probe.json',dict(observed_at=stamp(),rows=rows,scene_sha256=sha(ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes/rommerskirchen.json')))
        receipt.update(state='completed',completed_at=stamp());save(status,receipt)
    except BaseException as exc:
        receipt.update(state='failed',completed_at=stamp(),error=repr(exc));save(status,receipt);raise


if __name__=='__main__':main()
