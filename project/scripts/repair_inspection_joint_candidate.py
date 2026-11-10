"""Repair a numerical initializer, then apply full continuous layer checks.

The candidate is a witness in the complete237-tower domain. Its four selected
towers are not a reduced optimization domain or a complete-coverage claim.
"""
from pathlib import Path
import sys,json,datetime,time,argparse,hashlib,gzip
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from Agent.Publication.engineering_regional_seed import numeric_context,camera_pose,smooth_controls,duration,find_observation
from Agent.Publication.engineering_regional_curves import control_points

OUT=ROOT/'总结/03_实验数据/工程化联合求解/constraint_layers/repaired_joint_seed.json'
RECORD=ROOT/'research_quality/inspection_joint_repair.json'


def stamp():return datetime.datetime.now(datetime.timezone.utc).isoformat()
def save(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False),'utf-8')
def signature(s):return hashlib.sha256(json.dumps({k:s[k] for k in ['task','index','C','A','start_s','duration_s']},sort_keys=True).encode()).hexdigest()


def contact_repair(ctx,plan):
    """Use allowed one-sided acceleration/jerk for upright windy touchdown.

    Position and velocity stay continuous. The existing mathematical contract
    explicitly permits the flight-side acceleration to differ from parking.
    """
    for f in plan['flights']:
        for s,end in [(f['segments'][0],0),(f['segments'][-1],1)]:
            c=np.array(s['C']);T=s['duration_s'];xyz=c[0 if end==0 else -1]
            W=ctx.forecast.evaluate(s['start_s']+end*T,float(xyz[2]-ctx.terrain(*xyz[:2])))
            vals=W['values'];u=-np.array([vals['wind_east_ms'],vals['wind_north_ms'],0.]);rho=vals['density_kg_m3'];speed=np.linalg.norm(u)
            acc=-.5*.25*rho*speed*u/7
            ud=acc-np.array([W['time_derivative']['wind_east_ms'],W['time_derivative']['wind_north_ms'],0.])
            rd=W['time_derivative']['density_kg_m3']
            jerk=-.5*.25*(rd*speed*u+rho*((u@ud/max(speed,1e-12))*u+speed*ud))/7
            left=[np.zeros(3)]*4;right=[c[-1]-c[0],np.zeros(3),np.zeros(3),np.zeros(3)]
            if end==0:left=[np.zeros(3),np.zeros(3),acc,jerk]
            else:right=[c[-1]-c[0],np.zeros(3),acc,jerk]
            s['C']=(np.array(control_points(left,right,T))+c[0]).tolist()
            s['contact_one_sided']=dict(endpoint=end,acceleration=acc.tolist(),jerk=jerk.tolist())


def make_plan(ctx,observations):
    byid={t['id']:i for i,t in enumerate(ctx.targets)};origin=np.array(ctx.scene['fleet']['vehicles'][0]['initial_xyz_m'])
    towers=sorted((float(np.linalg.norm(np.asarray(ctx.record['corrections'][byid[ids[0]]]['viewpoint_center_xyz_m'])[:2]-origin[:2])),t,[byid[i] for i in ids]) for t,ids in ctx.record['domain']['tower_tasks'].items())[:4]
    flights=[];clock=0.
    for order,(_,tower,ids) in enumerate(towers):
        carrier=order//2;local=order%2;v=ctx.scene['fleet']['vehicles'][carrier]
        xy=np.array(v['initial_xyz_m'][:2]);ground=float(ctx.terrain(*xy));dock=np.r_[xy,ground+2.4];exitpoint=np.r_[xy,ground+6.4]
        yaw=v['initial_heading_rad'];groundpose=np.array([yaw,0.,0.,1.]);clock+=120.;start=clock
        cruise=max(ground+10,max(observations[i]['xyz_m'][2] for i in ids)+8)
        for i in ids:
            xyz=np.array(observations[i]['xyz_m'])
            if not ctx.geometry.segment(smooth_controls(xyz,np.r_[xyz[:2],cruise+22]))['passed']:
                replacement=find_observation(ctx,i,cruise+22)
                if not replacement['passed']:raise RuntimeError('No safe vertical initializer for task '+str(i))
                observations[i]=replacement
        previous=dock;previouspose=groundpose;segments=[]
        def safe_transfer(first,last,root=False,recovery=False):
            for extra in [0,4,8,12,16,22,30]:
                h=cruise+extra
                if recovery:points=[first,np.r_[first[:2],h],exitpoint,last]
                else:points=[first,exitpoint if root else np.r_[first[:2],h],np.r_[last[:2],h],last]
                contact_segment=2 if recovery else (0 if root else None)
                outcomes=[ctx.geometry.segment(smooth_controls(points[k],points[k+1]),dock if k==contact_segment else None) for k in range(3)]
                if all(x['passed'] for x in outcomes):return points
            raise RuntimeError('Initializer needs a geometric detour: '+str(outcomes))
        def add(i,k,a,b,pa,pb):
            nonlocal clock
            T=duration(a,b,pa,pb);seg=dict(task=i,index=k,start_s=clock,duration_s=T,
                C=smooth_controls(a,b).tolist(),A=smooth_controls(pa,pb).tolist())
            segments.append(seg);clock+=T
        for ix,i in enumerate(ids):
            target=np.array(observations[i]['xyz_m']);pose=camera_pose(ctx,target,ctx.record['corrections'][i]['target_xyz_m'],clock+150.,yaw)
            points=safe_transfer(previous,target,root=ix==0)
            poses=[previouspose,previouspose,pose,pose]
            for k in range(3):add(i,k,points[k],points[k+1],poses[k],poses[k+1])
            T=ctx.targets[i]['effective_service_seconds'];segments.append(dict(task=i,index=3,start_s=clock,duration_s=T,C=np.tile(target,(8,1)).tolist(),A=np.tile(pose,(8,1)).tolist()));clock+=T
            if ix==8:
                points=safe_transfer(target,dock,recovery=True);poses=[pose,pose,groundpose,groundpose]
                for k in range(3):add(i,k+4,points[k],points[k+1],poses[k],poses[k+1])
            previous=target;previouspose=pose
        flights.append(dict(slot=carrier*78+local,carrier=carrier,tower=tower,tasks=ids,start_s=start,end_s=clock,dock=dock.tolist(),segments=segments));clock+=240.
    return dict(observed_at=stamp(),revision=ctx.record['revision'],source_sha256=ctx.record['source']['sha256'],
        model_sha256=ctx.record['model']['sha256'],flights=flights,makespan_s=clock,domain_towers=237,selected_towers=4,
        purpose='free initializer, not variable fixings; feasibility stages recorded separately')


def main():
    p=argparse.ArgumentParser();p.add_argument('--through',choices=['geometry','physics'],default='physics');p.add_argument('--resume',action='store_true');p.add_argument('--contact-repair',action='store_true');a=p.parse_args()
    ctx=numeric_context(ROOT);start=time.perf_counter()
    if a.resume:plan=json.loads(OUT.read_text('utf-8-sig'))
    else:
        if OUT.exists():
            old=OUT.read_bytes();identity=hashlib.sha256(old).hexdigest()[:16]
            with gzip.open(OUT.with_name('seed_attempt_'+identity+'.json.gz'),'wb') as f:f.write(old)
        obs=json.loads((ROOT/'research_quality/inspection_observation_domain_repair.json').read_text('utf-8-sig'))
        assert obs['all_tasks_checked'] and obs['passed']==2133
        plan=make_plan(ctx,{r['index']:r for r in obs['rows']});save(OUT,plan)
    if a.contact_repair:
        old=OUT.read_bytes();identity=hashlib.sha256(old).hexdigest()[:16]
        with gzip.open(OUT.with_name('seed_attempt_'+identity+'.json.gz'),'wb') as f:f.write(old)
        for f in plan['flights']:
            for s in f['segments']:
                if s.get('continuous_physics',{}).get('passed'):s['physics_signature']=signature(s)
        contact_repair(ctx,plan);save(OUT,plan)
    record=dict(observed_at=stamp(),model_sha256=ctx.record['model']['sha256'],candidate=str(OUT.relative_to(ROOT)),layers=[],complete=False)
    if RECORD.exists():record['prior_attempts']=json.loads(RECORD.read_text('utf-8-sig')).get('prior_attempts',[])+[json.loads(RECORD.read_text('utf-8-sig'))|{'prior_attempts':[]}]
    checks=[]
    for f in plan['flights']:
        for s in f['segments']:
            if s['duration_s']<=0:continue
            contact=np.array(f['dock']) if (s['index']==0 and s['task']==f['tasks'][0]) or (s['index']==6 and s['task']==f['tasks'][-1]) else None
            out=ctx.geometry.segment(np.array(s['C']),contact);checks.append(dict(family='body',task=s['task'],index=s['index'],result=out))
            if not out['passed']:break
            if s['index']==3:
                for i,point in enumerate(ctx.targets[s['task']]['target']['surface_samples_utm_m']):
                    out=ctx.geometry.sight_line(np.array(s['C']),np.array(point));checks.append(dict(family='LOS',task=s['task'],index=s['index'],sample=i,result=out))
                    if not out['passed']:break
            if not out['passed']:break
        if not out['passed']:break
    record['layers'].append(dict(layer='continuous_static_body_visibility',passed=all(x['result']['passed'] for x in checks),checks=checks,wall_s=time.perf_counter()-start));save(RECORD,record)
    print(json.dumps(dict(layer=record['layers'][-1]['layer'],passed=record['layers'][-1]['passed'],checks=len(checks),first_failure=next((x for x in checks if not x['result']['passed']),None))),flush=True)
    if not record['layers'][-1]['passed'] or a.through=='geometry':return
    checks=[]
    for f in plan['flights']:
        for s in f['segments']:
            if s['duration_s']<=0:continue
            t=ctx.targets[s['task']] if s['index']==3 else None
            print(json.dumps(dict(checking='force_optics_energy',task=s['task'],index=s['index'],duration=s['duration_s'])),flush=True)
            before=time.perf_counter()
            cached=s.get('physics_signature')==signature(s) and s.get('continuous_physics',{}).get('passed')
            out=s['continuous_physics'] if cached else ctx.enclosure.integrate_and_check(np.array(s['C']),s['duration_s'],np.array(s['A']),s['start_s'],t,1e-4,22)
            checks.append(dict(task=s['task'],index=s['index'],result=out,wall_s=time.perf_counter()-before))
            s['continuous_physics']=out;s['physics_signature']=signature(s);save(OUT,plan)
            record['layers']=record['layers'][:1]+[dict(layer='continuous_force_optics_energy',passed=False,checked=len(checks),checks=checks)]
            save(RECORD,record);print(json.dumps(checks[-1]),flush=True)
            if not out['passed']:return
    record['layers'][-1]['passed']=True;save(RECORD,record)


if __name__=='__main__':main()
