"""Cumulative whole-scene feasibility diagnosis, explicitly authorized by user.

This is NOT a replacement regional optimizer, a method comparison, or a claim
that an early layer meets omitted physical constraints. All original choices
remain free. Constructive witnesses are warm starts, never variable fixings.
The next layer is admitted only after SCIP and the indexed-row evaluator agree
on a nonempty feasible vector for the preceding layer.
"""
from pathlib import Path
import argparse, ctypes, datetime, gzip, hashlib, json, math, os, sys, time
from types import SimpleNamespace
import numpy as np
import psutil
from scipy.interpolate import LinearNDInterpolator
from pyproj import Transformer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from Agent.Publication.engineering_regional_structure import RegionalPathBlock, load_regional, pad_slot_bound
from Agent.Publication.engineering_regional_resources import RegionalResources
from Agent.Publication.engineering_regional_roads import RegionalVehicles
from Agent.Publication.engineering_regional_charging import RegionalCharging
from Agent.Publication.engineering_regional_curves import RegionalCurves
from Agent.Publication.engineering_indexed_terrain import IndexedTerrain
from Agent.Publication.engineering_regional_rendezvous import RegionalRendezvous
from Agent.Publication.engineering_regional_weather import retained_regional_forecast
from Agent.Publication.engineering_regional_physics import RegionalPhysics

OUT = ROOT/'总结/03_实验数据/工程化联合求解/constraint_layers'
RECORD = ROOT/'research_quality/inspection_constraint_layers.json'

def stamp(): return datetime.datetime.now(datetime.timezone.utc).isoformat()
def save(p, x): p.write_text(json.dumps(x, ensure_ascii=False, indent=2, allow_nan=False), 'utf-8')

def commit_memory():
    class Memory(ctypes.Structure):
        _fields_=[('length',ctypes.c_ulong),('load',ctypes.c_ulong)]+[(n,ctypes.c_ulonglong) for n in
            ['total_phys','avail_phys','total_page','avail_page','total_virtual','avail_virtual','extended']]
    x=Memory();x.length=ctypes.sizeof(x)
    if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(x)):raise OSError('Cannot read Windows commit headroom')
    return dict(commit_limit_GiB=x.total_page/2**30,commit_available_GiB=x.avail_page/2**30,physical_available_GiB=x.avail_phys/2**30)

class Diagnosis:
    def __init__(self, record_run=True):
        self.record_run=record_run
        regional=OUT.parent/'regional_joint/run_manifest.json'
        if regional.exists():
            live=json.loads(regional.read_text('utf-8-sig'))
            if live.get('pid')!=os.getpid():
                try:
                    process=psutil.Process(live['pid'])
                    if abs(process.create_time()-live['process_create_time'])<.01:
                        raise RuntimeError('The regional optimizer is live; do not duplicate its task')
                except psutil.NoSuchProcess:pass
        self.scene, self.record = load_regional(ROOT)
        self.forecast = retained_regional_forecast(ROOT)
        self.run = f'{int(time.time())}_{os.getpid()}'
        self.started = time.perf_counter()
        self.results = []
        self.values = {}
        self.explicit_before = set()
        self.bounds_before = np.empty((0,2),dtype=float)
        self.previous_passed = True
        self.seed_paths = {}
        self.sol = None
        OUT.mkdir(parents=True, exist_ok=True)
        old = OUT/'status.json'
        self.resume_record = None
        if old.exists():
            x=json.loads(old.read_text('utf-8-sig'))
            if x.get('running') and psutil.pid_exists(x['pid']):
                if abs(psutil.Process(x['pid']).create_time()-x['created'])<.01:
                    raise RuntimeError('A matching layer diagnosis is already live')
            self.resume_record=x
            if record_run:save(OUT/(x['run_id']+'_receipt.json'),x)
        self.base = dict(experiment_id='inspection_constraint_layers', run_id=self.run,
            pid=os.getpid(), created=psutil.Process().create_time(), owner_thread=os.environ.get('CODEX_THREAD_ID'),
            source_sha256=self.record['source']['sha256'], model_sha256=self.record['model']['sha256'],
            code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            purpose='cumulative full237/2133 model feasibility diagnosis; no algorithm comparison',
            scope=dict(towers=237, tasks=2133, optional_flight_slots=156, road_phases=[256,256], charging_events=[156,156]),
            approval='用户：一层层叠加约束，叠加一层跑一次！通过有解再下一层',
            gate='candidate constraint feasibility and all237 tower coverage are separate; layer advancement requires both',
            witness_role='constructive warm start only; never fixed decisions; early physical omissions explicit',
            stop_rule='first verified feasible solution per layer; no wall time/node limit; stop adding layers on failure',
            original_run=dict(pid=47724,created=1794204739.9118638,status='exited_native_allocation_failure',
                evidence='research_quality/inspection_regional_memory_exit.json'), running=True)
        if record_run:
            save(OUT/'run_manifest.json', self.base)
            with gzip.open(OUT/('source_'+self.run+'.py.gz'),'wb') as f:f.write(Path(__file__).read_bytes())
            os.chdir(OUT) # Native SCIP logger needs a relative ASCII filename here.
            self.persist('starting')

    def persist(self, phase, **kw):
        if not self.record_run:return
        d=dict(self.base, observed_at=stamp(), phase=phase, elapsed_s=time.perf_counter()-self.started,
            layers=self.results, **kw)
        save(OUT/'status.json',d); save(RECORD,d)
        with (OUT/'progress.jsonl').open('a',encoding='utf-8') as f:
            f.write(json.dumps(d,ensure_ascii=False,allow_nan=False)+'\n')

    def protect(self):
        free=psutil.virtual_memory().available/2**30
        disk={d:psutil.disk_usage(d+':/').free/2**30 for d in ['C','D']}
        cm=commit_memory()
        # Construction is single-process, with no native transformed copy yet.
        # Keep2GiB commit here; optimize admission separately reserves its peak.
        if free<2 or cm['commit_available_GiB']<2 or disk['C']<10.5 or disk['D']<30.5:
            raise MemoryError(f'Layer resource floor: RAM {free}, disks {disk}, commit {cm}')

    def bound_snapshot(self):
        variables=self.m.getVars(transformed=False)
        ans=np.full((max(v.getIndex() for v in variables)+1,2),np.nan)
        for v in variables:ans[v.getIndex()]=[v.getLbOriginal(),v.getUbOriginal()]
        return ans

    def put(self, v, x): self.values[v.name]=float(x)
    def sel(self, s, k):
        self.put(s.value,k)
        for j,b in enumerate(s.bits): self.put(b,(k>>j)&1)
    def val(self,v): return self.values[v.name]

    def defaults(self):
        for v in self.m.getVars():
            if v.name not in self.values:
                self.put(v,min(v.getUbOriginal(),max(v.getLbOriginal(),0.)))

    def set_path(self):
        p=self.path
        # Two independent, complete task paths, one per carrier; this seed is
        # only for pre-physics relaxations. Later constraints may change it.
        ids=list(range(1,len(p.tasks)+1)); split=9*119
        self.seed_paths={0:ids[:split],78:ids[split:]}
        C=0.
        for k,ids in self.seed_paths.items():
            f=p.flights[k];self.put(f.active,1);self.sel(f.first,ids[0]);self.sel(f.last,ids[-1])
            clock=p.pre;self.put(f.start,clock)
            for a,i in enumerate(ids):
                t=p.tasks[i-1]
                for v,x in [(t.active,1),(t.root,a==0),(t.last,a==len(ids)-1),
                            (t.has_pred,a>0),(t.has_succ,a<len(ids)-1)]:self.put(v,x)
                self.sel(t.flight,k+1);self.sel(t.pred,ids[a-1] if a else 0)
                self.sel(t.succ,ids[a+1] if a<len(ids)-1 else 0)
                self.put(t.incoming,4);clock+=4;self.put(t.time,clock)
                dwell=float(p.task_rows[i-1]['observation_requirement']['min_dwell_s'])
                self.put(t.dwell,dwell);clock+=dwell
                if a==len(ids)-1:self.put(t.recovery,4);clock+=4
            self.put(f.end,clock);C=max(C,clock+p.post)
        self.put(p.C,C)
        for z in p.towers.values():self.put(z,1)

    def seed_resources(self):
        r=self.resources
        for i,(f,z) in enumerate(zip(self.path.flights,r.flights)):
            active=i in self.seed_paths
            for kind in ['aircraft','battery']:
                chain=getattr(z,kind); self.sel(chain.resource,1 if active else 0)
                for k,a in enumerate(chain.assignment):self.put(a,active and k==0)
                self.put(chain.first,active);self.put(chain.last,active)
                for k in range(len(chain.assignment)):
                    self.values[f'{kind}_{i+1}_root_identity{k+1}']=float(active and k==0)
            for v in [z.before_charge,z.launch_energy,z.landing_energy]: self.put(v,r.capacity)

    def seed_roads(self):
        v=self.vehicles;C=self.val(self.path.C)
        for carrier,phases in enumerate(v.phases):
            vehicle=self.scene['fleet']['vehicles'][carrier];mode=v.domain.stop[vehicle['depot_stop']]
            z=v.domain.modes[mode];xy=z['center']; state=[*xy,vehicle['initial_heading_rad'],0,0]
            for k,phase in enumerate(phases):
                self.sel(phase.mode,mode);self.put(phase.start,0 if k==0 else C)
                self.put(phase.end,C);self.put(phase.duration,C if k==0 else 0)
                self.put(phase.parked,1)
                for row in phase.controls:
                    for x,y in zip(row,state):self.put(x,y)
                for key,value in zip(phase.geometry,[*xy,0,0,0,0,0]):self.put(phase.geometry[key],value)
                if k:
                    prefix=f'vehicle{carrier}_phase{k}_transition'
                    self.sel(self.selectors[prefix+'_option'],0)
                    self.sel(self.selectors[prefix+'_pair'],mode*v.domain.max_options)
        for (carrier,k,s),(state,derivative) in v.point_aux.items():
            values=[self.val(z) for z in v.phases[carrier][k].controls[0]]
            for x,y in zip(state,values):self.put(x,y)
            for x in derivative:self.put(x,0)

    def seed_forecasts(self):
        # All charging events unused; their declared, ungated interpolation
        # definitions must nevertheless receive valid original coefficients.
        lo,hi,*coeff=self.forecast.coefficients(0,0)
        for i,e in enumerate(self.charging.events):
            for side in ['start','end']:
                key=f'charging{i}_{side}'
                self.sel(self.selectors[key+'_weather_cell'],0)
                self.values[key+'_time_origin']=0.;self.values[key+'_height_origin']=lo
                for a,row in enumerate(coeff):
                    for b,x in enumerate(row):self.values[key+f'_coefficient{a}_{b}']=float(x)

    def set_node(self,node,xyz,yaw):
        for r,row in enumerate(node.position):
            for q,v in enumerate(row):self.put(v,xyz[q] if r==0 else 0)
        for r,row in enumerate(node.pose):
            for q,v in enumerate(row):self.put(v,([yaw,0,0,1][q] if r==0 else 0))

    def seed_curves(self, anchors=False):
        c=self.curves
        for i,t in enumerate(self.path.tasks):
            k=int(self.val(t.flight.value))-1;f=self.path.flights[k]
            vehicle=self.scene['fleet']['vehicles'][f.carrier]
            xy=np.asarray(vehicle['initial_xyz_m'][:2]);ground=float(self.terrain(*xy))
            xyz=np.r_[xy,ground+6.4]-c.origin;yaw=vehicle['initial_heading_rad']
            d=c.by_task[i]
            for node in d['nodes']:self.set_node(node,xyz,yaw)
            if anchors and self.val(t.root):
                dock=xyz.copy();dock[2]-=4;self.set_node(d['nodes'][0],dock,yaw)
            if anchors and self.val(t.last):
                dock=xyz.copy();dock[2]-=4;self.set_node(d['nodes'][-1],dock,yaw)
            for s in d['segments']:
                T=(self.val(t.incoming) if s.index==0 else self.val(t.dwell) if s.role=='capture'
                    else self.val(t.recovery) if s.index==6 else 0)
                self.put(s.duration,T);self.put(s.active,T>0)
                for row in s.middle_pose_controls:
                    for q,x in enumerate(row):self.put(x,[yaw,0,0,1][q])

    def seed_terrain(self,prefix,xy):
        it=self.terrain_index
        k=int(it.terrain.tri.find_simplex(xy+it.origin));assert k>=0
        cell=it.cells[k];ids=cell['vertices']
        weights=np.linalg.solve(np.vstack([it.xy[ids].T,np.ones(3)]),np.r_[xy,1.])
        self.sel(self.selectors[prefix+'_cell'],k)
        for j,x in enumerate(weights):self.values[prefix+f'_weight{j}']=float(x)
        self.values[prefix+'_z']=float(weights@it.z[ids])
        for q,x in enumerate(cell['gradient']):self.values[prefix+f'_slope{q}']=float(x)

    def seed_anchors(self):
        for i,(f,a) in enumerate(zip(self.path.flights,self.rendezvous.anchors)):
            vehicle=self.scene['fleet']['vehicles'][f.carrier];xy=np.asarray(vehicle['initial_xyz_m'][:2])
            xyz=np.r_[xy,float(self.terrain(*xy))+2.4]-self.curves.origin
            for name,node in [('launch',a.launch),('land',a.land)]:
                self.set_node(node,xyz,vehicle['initial_heading_rad'])
                self.seed_terrain(f'flight{i+1}_{name}_ground',xyz[:2])
            if i in self.seed_paths:
                first=self.seed_paths[i][0]-1;last=self.seed_paths[i][-1]-1
                self.put(a.pad_exit,self.val(f.start)+self.val(self.curves.by_task[first]['segments'][0].duration))
                self.put(a.pad_entry,self.val(f.end)-self.val(self.curves.by_task[last]['segments'][-1].duration))
        self.seed_curves(anchors=True)

    def seed_camera(self):
        for z in self.rendezvous.ground_camera:
            i=int(z['name'].split('_')[0][6:])-1;f=self.path.flights[i]
            tail=z['name'].endswith('_tail')
            resource=self.resources.flights[i].aircraft
            previous=int(self.val(resource.pred.value))
            ts=self.val(self.path.flights[self.resources.by_carrier[f.carrier][previous-1]].end) if previous else 0.
            self.values[f'flight{i+1}_ground_camera_start_s']=ts
            T=(self.val(self.path.C)-self.val(f.end) if self.val(resource.last)>.5 else 0) if tail else self.val(f.start)-ts
            self.put(z['duration'],T);self.put(z['safe_duration'],T+1-self.val(z['active']))
            for endpoint in ['start','end']:
                for r,row in enumerate(z[endpoint]):
                    for q,x in enumerate(row):self.put(x,1 if r==0 and q==2 else 0)
            for row in z['middle']+z['shared_controls']:
                for q,x in enumerate(row):self.put(x,1 if q==2 else 0)

    def resource_witness(self):
        #24 towers selected from the full237;8 physical packs, no charging.
        #This is a warm start, not a restriction on the optimization domain.
        self.values={};self.defaults();self.seed_paths={};C=0.
        for carrier in range(2):
            clock=0.
            for local in range(4):
                k=carrier*78+local;f=self.path.flights[k]
                first=(carrier*4+local)*27+1;ids=list(range(first,first+27));self.seed_paths[k]=ids
                self.put(f.active,1);self.sel(f.first,ids[0]);self.sel(f.last,ids[-1])
                clock+=self.path.pre;self.put(f.start,clock)
                for a,i in enumerate(ids):
                    t=self.path.tasks[i-1]
                    for v,x in [(t.active,1),(t.root,a==0),(t.last,a==26),(t.has_pred,a>0),(t.has_succ,a<26)]:self.put(v,x)
                    self.sel(t.flight,k+1);self.sel(t.pred,ids[a-1] if a else 0);self.sel(t.succ,ids[a+1] if a<26 else 0)
                    self.put(t.incoming,10);clock+=10;self.put(t.time,clock)
                    dwell=float(self.path.task_rows[i-1]['observation_requirement']['min_dwell_s']);self.put(t.dwell,dwell);clock+=dwell
                    if a==26:self.put(t.recovery,10);clock+=10
                self.put(f.end,clock);clock+=self.path.post;C=max(C,clock)
        self.put(self.path.C,C)
        used={self.path.tasks[i-1].id for ids in self.seed_paths.values() for i in ids}
        for tower,ids in self.path.tower_tasks.items():self.put(self.path.towers[tower],set(ids)<=used)
        for i,(f,z) in enumerate(zip(self.path.flights,self.resources.flights)):
            for v in [z.before_charge,z.launch_energy,z.landing_energy]:self.put(v,self.resources.capacity)
            if i not in self.seed_paths:continue
            local=i%78
            for kind,identity,prev,nxt in [('aircraft',local%2+1,local-2 if local>=2 else None,local+2 if local<2 else None),('battery',local+1,None,None)]:
                chain=getattr(z,kind);self.sel(chain.resource,identity)
                for k,a in enumerate(chain.assignment):self.put(a,k+1==identity)
                self.sel(chain.pred,prev+1 if prev is not None else 0);self.sel(chain.succ,nxt+1 if nxt is not None else 0)
                for v,x in [(chain.first,prev is None),(chain.last,nxt is None),(chain.has_pred,prev is not None),(chain.has_succ,nxt is not None)]:self.put(v,x)
                for k in range(len(chain.assignment)):self.values[f'{kind}_{i+1}_root_identity{k+1}']=float(prev is None and k+1==identity)
            E=self.power_floor_W*(self.val(f.end)-self.val(f.start))/3600
            self.put(z.energy,E);self.put(z.landing_energy,self.resources.capacity-E)
        self.seed_roads();self.seed_forecasts();self.seed_curves(anchors=True);self.seed_anchors();self.seed_camera()

    def resume(self):
        old=self.resume_record
        if not old or not old.get('layers'):return 1
        good=[]
        for r in old['layers']:
            if r.get('status')!='FEASIBLE_VERIFIED':break
            good.append(r)
        if len(good)!=len(old['layers']):
            self.base['retained_failed_attempts']=old.get('retained_failed_attempts',[])+old['layers'][len(good):]
            self.base['repair']='Retry same failed layer; pad-clock and absolute dock-height seed repairs; no physical constraint or bound changed.'
        if good[-1]['layer']>9:
            if not getattr(self,'repair',False):raise RuntimeError('Use --repair for verified-prefix restoration and complete repair')
            self.base['prior_higher_layers']=good[9:];good=good[:9]
        self.persist('restoring_verified_layers')
        for k in range(1,len(good)+1):self.build_layer(k)
        last=good[-1];w=ROOT/last['witness']
        assert hashlib.sha256(w.read_bytes()).hexdigest()==last['witness_sha256']
        with gzip.open(w,'rt',encoding='utf-8') as f:self.values=json.load(f)
        assert len(self.values)==self.m.getNVars()==last['variables']
        assert self.m.getNConss()==last['explicit_rows']
        sol=self.m.createOrigSol()
        for v in self.m.getVars():self.m.setSolVal(sol,v,self.values[v.name])
        audit=self.evaluate(sol);self.m.freeSol(sol)
        assert audit['passed'],audit
        self.results=good;self.explicit_before={c.name for c in self.m.getConss()}
        self.bounds_before=self.bound_snapshot()
        self.base['resume_from']=dict(run_id=old['run_id'],last_layer=last['layer'],witness_audit=audit,earlier_layers_not_reoptimized=True)
        return len(good)+1

    def geometry_witness(self):
        """Four actual towers, four free warm-start flights, whole237 domain.

        Smooth degree7 zero-end-derivative transfers use three pieces via a
        tower-height cruise level. True weather determines capture camera pose.
        Independent checks, not this constructor, decide feasibility.
        """
        self.values={};self.defaults();self.seed_paths={};c=self.curves
        by_id={t.id:i+1 for i,t in enumerate(self.path.tasks)}
        origin=np.asarray(self.scene['fleet']['vehicles'][0]['initial_xyz_m'])
        towers=[]
        for tower,ids in self.path.tower_tasks.items():
            ii=[by_id[t] for t in ids];xyz=np.asarray(c.corrected_targets[ii[0]-1]['easting_m'])
            q=self.record['corrections'][ii[0]-1]['viewpoint_center_xyz_m']
            towers.append((float(np.linalg.norm(np.asarray(q)[:2]-origin[:2])),tower,ii))
        # Headroom includes Windows commit and native transformed-model peak.
        # Four seed flights reduce materialization memory; no task, flight slot,
        # route choice or physical constraint is removed from the model domain.
        count=2;towers.sort();chosen=towers[:2*count];C=0.
        def duration(a,b,pa,pb):
            delta=np.asarray(b)-a;d=float(np.linalg.norm(delta));angle=float(np.max(abs(np.asarray(pb)[:3]-np.asarray(pa)[:3])))
            if d<1e-9 and angle<1e-9:return 0.
            return max(.01,2.1875*d/4.5,2.1875*abs(delta[2])/.9,(7.6*d/2.5)**.5,(53*d/4.)**(1/3),2.1875*angle/.45,(7.6*angle/.45)**.5)
        def camera(i,tt,yaw):
            correction=self.record['corrections'][i-1];xyz=np.asarray(correction['viewpoint_center_xyz_m'])
            agl=float(xyz[2]-self.terrain(*xyz[:2]));W=self.forecast.evaluate(min(tt,self.path.H),agl)['values']
            va=-np.array([W['wind_east_ms'],W['wind_north_ms'],0.])
            force=np.array([0.,0.,7*9.81])+.5*.25*W['density_kg_m3']*np.linalg.norm(va)*va
            b3=force/np.linalg.norm(force);b2=np.cross(b3,[math.cos(yaw),math.sin(yaw),0]);b2/=np.linalg.norm(b2);b1=np.cross(b2,b3)
            look=np.array(correction['target_xyz_m'])-xyz;look/=np.linalg.norm(look)
            local=np.column_stack([b1,b2,b3]).T@look
            return [yaw,math.asin(local[2]),math.atan2(local[1],local[0]),1.]
        def assign_node(node,xyz,pose):
            self.set_node(node,np.asarray(xyz)-c.origin,pose[0])
            for v,x in zip(node.pose[0],pose):self.put(v,x)
        def assign_segment(s,left,right,pose0,pose1,T):
            assign_node(s.left,left,pose0);assign_node(s.right,right,pose1)
            self.put(s.duration,T);self.put(s.active,T>0)
            for row,pose in zip(s.middle_pose_controls,[pose0,pose1]):
                for v,x in zip(row,pose):self.put(v,x)
        # Inactive optional curves have no physical role; use in-bound states.
        for node in c.nodes:
            xyz=np.array([0.,0.,6.4]);self.set_node(node,xyz,0.)
        for carrier in range(2):
            clock=0.;vehicle=self.scene['fleet']['vehicles'][carrier]
            xy=np.asarray(vehicle['initial_xyz_m'][:2]);g=float(self.terrain(*xy));dock=np.r_[xy,g+2.4];exitpoint=np.r_[xy,g+6.4]
            yaw=vehicle['initial_heading_rad'];ground_pose=[yaw,0,0,1.]
            for local in range(count):
                k=carrier*78+local;_,tower,ids=chosen[carrier*count+local];self.seed_paths[k]=ids
                f=self.path.flights[k];self.put(f.active,1);self.sel(f.first,ids[0]);self.sel(f.last,ids[-1])
                clock+=self.path.pre;self.put(f.start,clock)
                cruise=max(g+10,max(self.record['corrections'][i-1]['viewpoint_center_xyz_m'][2] for i in ids)+8)
                previous=dock;previous_pose=ground_pose
                for a,i in enumerate(ids):
                    t=self.path.tasks[i-1];data=c.by_task[i-1];target=np.array(self.record['corrections'][i-1]['viewpoint_center_xyz_m'])
                    pose=camera(i,clock+150,yaw)
                    waypoint1=exitpoint if a==0 else np.r_[previous[:2],cruise]
                    waypoint2=np.r_[target[:2],cruise]
                    points=[previous,waypoint1,waypoint2,target];poses=[previous_pose,previous_pose,pose,pose]
                    incoming=0.
                    for j in range(3):
                        dt=duration(points[j],points[j+1],poses[j],poses[j+1]);incoming+=dt
                        assign_segment(data['segments'][j],points[j],points[j+1],poses[j],poses[j+1],dt)
                    for v,x in [(t.active,1),(t.root,a==0),(t.last,a==8),(t.has_pred,a>0),(t.has_succ,a<8)]:self.put(v,x)
                    self.sel(t.flight,k+1);self.sel(t.pred,ids[a-1] if a else 0);self.sel(t.succ,ids[a+1] if a<8 else 0)
                    self.put(t.incoming,incoming);clock+=incoming;self.put(t.time,clock)
                    dwell=float(self.path.task_rows[i-1]['observation_requirement']['min_dwell_s']);self.put(t.dwell,dwell)
                    assign_segment(data['segments'][3],target,target,pose,pose,dwell);clock+=dwell
                    if a==8:
                        points=[target,np.r_[target[:2],cruise],exitpoint,dock];poses=[pose,pose,ground_pose,ground_pose];recovery=0.
                        for j in range(3):
                            dt=duration(points[j],points[j+1],poses[j],poses[j+1]);recovery+=dt
                            assign_segment(data['segments'][4+j],points[j],points[j+1],poses[j],poses[j+1],dt)
                        self.put(t.recovery,recovery);clock+=recovery
                    previous=target;previous_pose=pose
                self.put(f.end,clock);clock+=self.path.post;C=max(C,clock)
        self.put(self.path.C,C)
        used={self.path.tasks[i-1].id for ids in self.seed_paths.values() for i in ids}
        for tower,ids in self.path.tower_tasks.items():self.put(self.path.towers[tower],set(ids)<=used)
        # Exact identity chains; packs are not reset or re-used in this seed.
        for i,(f,z) in enumerate(zip(self.path.flights,self.resources.flights)):
            for v in [z.before_charge,z.launch_energy,z.landing_energy]:self.put(v,self.resources.capacity)
            if i not in self.seed_paths:continue
            local=i%78
            for kind,identity,prev,nxt in [('aircraft',local%2+1,local-2 if local>=2 else None,local+2 if local+2<count else None),('battery',local+1,None,None)]:
                chain=getattr(z,kind);self.sel(chain.resource,identity)
                for j,a in enumerate(chain.assignment):self.put(a,j+1==identity)
                self.sel(chain.pred,prev+1 if prev is not None else 0);self.sel(chain.succ,nxt+1 if nxt is not None else 0)
                for v,x in [(chain.first,prev is None),(chain.last,nxt is None),(chain.has_pred,prev is not None),(chain.has_succ,nxt is not None)]:self.put(v,x)
                for j in range(len(chain.assignment)):self.values[f'{kind}_{i+1}_root_identity{j+1}']=float(prev is None and j+1==identity)
            E=self.power_floor_W*(self.val(f.end)-self.val(f.start))/3600;self.put(z.energy,E);self.put(z.landing_energy,self.resources.capacity-E)
        self.seed_roads();self.seed_forecasts()
        # Anchor-only update: do not overwrite the new flight geometry.
        for i,(f,a) in enumerate(zip(self.path.flights,self.rendezvous.anchors)):
            vehicle=self.scene['fleet']['vehicles'][f.carrier];xy=np.asarray(vehicle['initial_xyz_m'][:2])
            xyz=np.r_[xy,float(self.terrain(*xy))+2.4]-c.origin
            for name,node in [('launch',a.launch),('land',a.land)]:
                self.set_node(node,xyz,vehicle['initial_heading_rad']);self.seed_terrain(f'flight{i+1}_{name}_ground',xyz[:2])
            if i in self.seed_paths:
                self.put(a.pad_exit,self.val(f.start)+self.val(c.by_task[self.seed_paths[i][0]-1]['segments'][0].duration))
                self.put(a.pad_entry,self.val(f.end)-self.val(c.by_task[self.seed_paths[i][-1]-1]['segments'][-1].duration))
        self.seed_camera()
        self.base['geographic_witness']=dict(towers=[x[1] for x in chosen],completed=len(chosen),all237_choices_retained=True,
            purpose='candidate only; full physical checks determine acceptance')

    def load_repaired_seed(self):
        p=OUT/'repaired_joint_seed.json';plan=json.loads(p.read_text('utf-8-sig'))
        assert plan['model_sha256']==self.record['model']['sha256']
        def derivatives(cp,T,orders):
            cp=np.asarray(cp,float);answer=[]
            for r in range(orders+1):
                answer.append((cp[0].copy(),cp[-1].copy()))
                cp=(len(cp)-1)*np.diff(cp,axis=0)/T
            return answer
        self.quadrature_orders={}
        from Agent.Publication.engineering_regional_seed import numeric_context,numeric_power
        ctx=numeric_context(ROOT)
        for f in plan['flights']:
            slot=f['slot'];flight=self.path.flights[slot]
            assert self.seed_paths[slot]==[i+1 for i in f['tasks']]
            self.put(flight.start,f['start_s']);self.put(flight.end,f['end_s'])
            for s in f['segments']:
                seg=self.curves.by_task[s['task']]['segments'][s['index']];T=s['duration_s']
                self.put(seg.duration,T);self.put(seg.active,T>0)
                C=np.asarray(s['C'])-self.curves.origin;A=np.asarray(s['A'])
                dp=derivatives(C,T,3);da=derivatives(A,T,2)
                if 'contact_one_sided' in s:
                    q=s['contact_one_sided'];side=q['endpoint']
                    for rr,key in [(2,'acceleration'),(3,'jerk')]:
                        vals=list(dp[rr]);vals[side]=np.array(q[key]);dp[rr]=tuple(vals)
                for end,node in enumerate([seg.left,seg.right]):
                    for r,row in enumerate(node.position):
                        for q,v in enumerate(row):self.put(v,dp[r][end][q])
                    for r,row in enumerate(node.pose):
                        for q,v in enumerate(row):self.put(v,da[r][end][q])
                for row,values in zip(seg.middle_pose_controls,A[[3,4]]):
                    for v,x in zip(row,values):self.put(v,x)
                proof=s.get('continuous_physics',{});assert proof.get('passed'),(s['task'],s['index'])
                for order in [5,9,13,17,25,33,49,65]:
                    ns,ws=np.polynomial.legendre.leggauss(order)
                    E=sum(w*numeric_power(ctx,np.array(s['C']),T,s['start_s'],(x+1)/2)*T/7200 for x,w in zip(ns,ws))
                    if proof['lower_Wh']-1e-6<=E<=proof['upper_Wh']+1e-6:break
                else:raise ValueError('Energy quadrature unresolved for '+str((s['task'],s['index'])))
                self.quadrature_orders[s['task'],s['index']]=order
            for i in f['tasks']:
                segs=[s for s in f['segments'] if s['task']==i];task=self.path.tasks[i]
                self.put(task.time,next(s['start_s'] for s in segs if s['index']==3))
                self.put(task.incoming,sum(s['duration_s'] for s in segs if s['index']<3))
                self.put(task.recovery,sum(s['duration_s'] for s in segs if s['index']>3))
            a=self.rendezvous.anchors[slot]
            self.put(a.pad_exit,f['start_s']+f['segments'][0]['duration_s'])
            self.put(a.pad_entry,f['end_s']-f['segments'][-1]['duration_s'])
        self.put(self.path.C,plan['makespan_s']);self.seed_roads();self.seed_camera()
        self.base['repaired_initializer']=dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
            all_choices_free=True,selected_towers=4,serial_warm_schedule=True)

    def set_energy_prefix(self):
        for k,ids in self.seed_paths.items():
            total=0.
            for i in ids:
                d=self.curves.by_task[i-1];before,after=self.resources.prefix[i-1]
                self.put(before,total)
                total+=sum(self.val(self.physics.segment_energy[s.task,s.index]) for s in d['segments'] if s.role!='recovery')
                self.put(after,total)
                recovery=sum(self.val(self.physics.segment_energy[s.task,s.index]) for s in d['segments'] if s.role=='recovery')
                self.values[f'task{i}_recovery_Wh']=recovery;total+=recovery
            z=self.resources.flights[k];self.put(z.energy,total);self.put(z.landing_energy,self.resources.capacity-total)

    def build_layer(self, k):
        if k==1:
            self.path=RegionalPathBlock(self.scene['inspection_tasks'],self.record['domain']['tower_tasks'],
                pad_slot_bound(self.scene,self.record)['per_carrier']).build();self.m=self.path.m
            self.m.hideOutput();self.m.setIntParam('limits/solutions',1)
            self.m.setRealParam('memory/arraygrowfac',1.05)
            self.m.setRealParam('memory/savefac',.5)
            self.m.setBoolParam('misc/usevartable',False);self.m.setBoolParam('misc/useconstable',False)
            self.m.setRealParam('limits/memory',5000.)
            self.defaults();self.set_path()
        elif k==2:
            self.resources=RegionalResources(self.path,self.scene).build()
            # Restore final joint-model original bounds now, so the diagnostic
            # sequence is genuinely cumulative. No charging-flow claim until4.
            for r in self.resources.flights:self.m.chgVarUb(r.charged,self.resources.capacity-self.resources.reserve)
            self.defaults();self.seed_resources()
        elif k==3:
            self.vehicles=RegionalVehicles(self.path,self.scene,[256,256]).build()
            self.selectors={s.name:s for s in self.path.rel.selectors};self.defaults();self.seed_roads()
        elif k==4:
            self.charging=RegionalCharging(self.resources,self.vehicles.parked,[156,156]).build().attach_temperature(self.forecast)
            self.selectors={s.name:s for s in self.path.rel.selectors};self.defaults();self.seed_forecasts()
        elif k==5:
            tr=Transformer.from_crs(4326,self.scene['coordinate_reference']['projected'],always_xy=True)
            samples=self.scene['terrain']['samples'];x,y=tr.transform([s['location']['lng'] for s in samples],[s['location']['lat'] for s in samples])
            self.terrain=LinearNDInterpolator(np.column_stack([x,y]),[s['elevation'] for s in samples])
            self.curves=RegionalCurves(self.path,self.scene,self.record,self.terrain).build()
            self.defaults();self.seed_curves()
        elif k==6:
            self.terrain_index=IndexedTerrain(self.terrain,self.curves.origin[:2])
            self.rendezvous=RegionalRendezvous(self.curves,self.vehicles,self.resources,self.terrain_index).build()
            self.selectors={s.name:s for s in self.path.rel.selectors};self.defaults();self.seed_anchors()
        elif k==7:
            self.rendezvous.attach_global_camera();self.defaults();self.seed_camera()
        elif k==8:
            #Necessary projection of existing power law, not replacement physics.
            rhomax=float(self.forecast.values[:,:,2].max())
            self.power_floor_W=float(self.scene['joint_reference_profile']['energy']['hover_power_W'])*math.sqrt(1.225/rhomax)
            for i,(f,r) in enumerate(zip(self.path.flights,self.resources.flights)):
                self.m.addCons(r.energy>=self.power_floor_W*(f.end-f.start)/3600,name=f'diagnostic_energy_lower_bound_{i}')
            self.resource_witness()
            self.base['energy_projection']=dict(rho_max=rhomax,minimum_hover_power_W=self.power_floor_W,
                derivation='P_hover*sqrt(1.225/rho_max); bilinear rho extrema at corners; drag and positive climb >=0',is_necessary_not_sufficient=True)
        elif k==9:
            allowed=[]
            for interval in self.forecast.intervals:
                if not interval['flight_allowed']:continue
                if allowed and allowed[-1][1]==interval['start_s']:allowed[-1][1]=interval['end_s']
                else:allowed.append([interval['start_s'],interval['end_s']])
            from Agent.Publication.engineering_indexed_constraints import AffineRow
            for i,f in enumerate(self.path.flights):
                window=self.path.rel.selector(f'diagnostic_flight{i}_weather_window',len(allowed))
                def rows(k,f=f):
                    lo,hi=allowed[k]
                    return [AffineRow('launch_after_weather_start',[(1,f.start)],lo=lo),AffineRow('land_before_weather_end',[(1,f.end)],hi=hi)]
                self.path.rel.add(window,rows,[],f.active,f'diagnostic_flight{i}_weather_interval')
            self.defaults();self.base['weather_projection']=dict(allowed_windows_s=allowed,is_necessary_not_sufficient=True)
        elif k==10:
            self.geometry_witness()
            if getattr(self,'repair',False):self.load_repaired_seed()
            self.physics=RegionalPhysics(self.curves,self.terrain_index,self.forecast,self.rendezvous).register_energy(self.resources)
            if getattr(self,'oracle_first',False):
                self.defaults();plan=json.loads((OUT/'repaired_joint_seed.json').read_text('utf-8-sig'))
                for f in plan['flights']:
                    for s in f['segments']:
                        proof=s['continuous_physics'];assert proof['passed']
                        self.put(self.physics.segment_energy[s['task'],s['index']],(proof['lower_Wh']+proof['upper_Wh'])/2)
                self.set_energy_prefix()
                self.base['physics_layer']=dict(oracle_first=True,auxiliary_sampling_not_required_for_acceptance=True,
                    continuous_force_optics_energy_check_required=True,all_optional_task_choices_retained=True)
                return
            self.defaults();seed=self.m.createOrigSol()
            for v in self.m.getVars():self.m.setSolVal(seed,v,self.values[v.name])
            self.physics.seed_solution=seed
            active=[s for s in self.curves.segments if self.val(s.active)>.5]
            for j,s in enumerate(active):
                order=getattr(self,'quadrature_orders',{}).get((s.task,s.index),5)
                self.protect();self.physics.materialize_segment(s,order)
                if j%20==0:self.persist('materializing_active_physics',segments_done=j+1,segments_total=len(active))
                E=float(self.m.getSolVal(seed,self.physics.energy[s.task,s.index,order]))
                self.m.setSolVal(seed,self.physics.segment_energy[s.task,s.index],E)
            self.values={v.name:float(self.m.getSolVal(seed,v)) for v in self.m.getVars()};self.m.freeSol(seed);self.physics.seed_solution=None
            self.set_energy_prefix()
            self.base['physics_layer']=dict(active_segments=len(active),points=sum(self.physics.current_orders.values()),
                all_active_segments_materialized=True,all_optional_task_choices_retained=True,continuous_enclosure_and_safety_not_yet_checked=True,
                seed_construction_residuals=self.physics.seed_residuals[:80],seed_residual_count=len(self.physics.seed_residuals))
            save(OUT/f'{self.run}_physics_seed_residuals.json',self.physics.seed_residuals)
        elif k==11:
            from Agent.Publication.engineering_regional_safety import RegionalSafety
            from Agent.Publication.engineering_regional_enclosure import RegionalEnclosure
            from Agent.Publication.engineering_regional_acceptance import RegionalAcceptance
            self.safety=RegionalSafety(self.curves,self.vehicles,self.rendezvous,self.physics)
            self.enclosure=RegionalEnclosure(self.scene,self.terrain,self.forecast)
            self.acceptance=RegionalAcceptance(self);self.last_acceptance=None
            if hasattr(self,'physical_cache'):
                self.acceptance.physics_cache={(*key,1e-4,22):value for key,value in self.physical_cache.items()}
            seed=self.m.createOrigSol()
            for v in self.m.getVars():self.m.setSolVal(seed,v,self.values[v.name])
            self.physics.seed_solution=seed
            for slot,ids in self.seed_paths.items():
                self.safety.add_pad_point(ids[0]-1,0,0.)
                self.safety.add_pad_point(ids[-1]-1,6,1.)
            self.values={v.name:float(self.m.getSolVal(seed,v)) for v in self.m.getVars()}
            self.m.freeSol(seed);self.physics.seed_solution=None
        else:raise ValueError(k)

    def evaluate(self,sol):
        get=lambda v:self.m.getSolVal(sol,v)
        bad=[];maxerr=0.;rowcount=0
        for sel,gate,factory,name in self.path.rel.relations:
            if gate is not None and get(gate)<1-1e-6:continue
            raw=get(sel.value);cat=int(round(raw))
            if abs(raw-cat)>1e-6 or not 0<=cat<sel.size:
                bad.append(dict(name=name,reason='invalid index'));continue
            if any(abs(get(b)-((cat>>k)&1))>1e-6 for k,b in enumerate(sel.bits)):
                bad.append(dict(name=name,reason='invalid binary encoding'));continue
            for row in factory(cat):
                rowcount+=1;err=row.violation(get);maxerr=max(maxerr,err)
                if err>1e-6:bad.append(dict(relation=name,category=cat,row=row.name,error=err))
        boundbad=[]
        for v in self.m.getVars(transformed=False):
            x=get(v);err=max(v.getLbOriginal()-x,x-v.getUbOriginal(),0)
            if v.vtype()!='CONTINUOUS':err=max(err,abs(x-round(x)))
            if err>1e-6:boundbad.append(dict(variable=v.name,error=err,value=x))
        native=bool(self.m.checkSol(sol,printreason=True,completely=True))
        completed=sum(get(z)>.5 for z in self.path.towers.values())
        missing=[]
        if hasattr(self,'physics'):
            missing=[(s.task,s.index) for s in self.curves.segments if get(s.active)>.5 and (s.task,s.index) not in self.physics.current_orders and not getattr(self,'oracle_first',False)]
        physical_oracle=None
        if getattr(self,'oracle_first',False) and hasattr(self,'physics') and not hasattr(self,'acceptance') and native and not bad and not boundbad:
            from Agent.Publication.engineering_regional_enclosure import RegionalEnclosure
            if not hasattr(self,'enclosure'):self.enclosure=RegionalEnclosure(self.scene,self.terrain,self.forecast)
            if not hasattr(self,'physical_cache'):self.physical_cache={}
            tests=[]
            for s in self.curves.segments:
                if get(s.active)<.5:continue
                cp=s.controls();C=np.array([[get(v) for v in row] for row in cp['C']])+self.curves.origin
                A=np.array([[get(v) for v in row] for row in cp['A']]);T=get(s.duration);start=get(s.clock)
                key=(s.task,s.index,C.tobytes(),A.tobytes(),T,start)
                if key not in self.physical_cache:
                    self.physical_cache[key]=self.enclosure.integrate_and_check(C,T,A,start,self.curves.corrected_targets[s.task] if s.role=='capture' else None,1e-4,22)
                proof=self.physical_cache[key];E=get(self.physics.segment_energy[s.task,s.index])
                valid=bool(proof['passed'] and proof['lower_Wh']-1e-6<=E<=proof['upper_Wh']+1e-6)
                tests.append(dict(task=s.task,index=s.index,passed=valid,result=proof,energy_Wh=E))
                if not valid:break
            physical_oracle=dict(passed=all(r['passed'] for r in tests),segments_checked=len(tests),checks=tests)
            if self.record_run:save(OUT/f'{self.run}_physical_oracle.json',physical_oracle)
        continuous=None
        if hasattr(self,'acceptance') and native and not bad and not boundbad and not missing:
            continuous=self.acceptance.check(sol)
            if self.record_run:save(OUT/f'{self.run}_complete_acceptance.json',continuous)
        return dict(native_feasible=native,indexed_rows_checked=rowcount,indexed_max_error=maxerr,
            indexed_bad_count=len(bad),indexed_bad=bad[:40],bounds_bad_count=len(boundbad),bounds_bad=boundbad[:20],
            completed_towers=completed,active_photos=sum(get(t.active)>.5 for t in self.path.tasks),
            coverage_complete=completed==len(self.path.towers),coverage_required=len(self.path.towers),
            active_flights=sum(get(f.active)>.5 for f in self.path.flights),
            uncompleted_towers=len(self.path.towers)-completed,makespan_s=get(self.path.C),
            missing_active_physical_integrals=len(missing),
            continuous_acceptance=continuous,
            physical_oracle=({k:v for k,v in physical_oracle.items() if k!='checks'} if physical_oracle else None),
            passed=bool(native and not bad and not boundbad and not missing and completed>0 and (continuous is None or continuous['passed']) and (physical_oracle is None or physical_oracle['passed'])))

    def step(self,k,name):
        if not self.previous_passed:raise RuntimeError('Prior layer not feasible; expansion blocked')
        self.protect();start=time.perf_counter();self.persist('building',current_layer=k)
        if k>1 and self.m.getStage()>1:self.m.freeTransform()
        self.build_layer(k);self.path.rel.lock(self.m.getVars())
        bounds=self.bound_snapshot()
        cons={c.name for c in self.m.getConss()}
        assert self.explicit_before<=cons,'Earlier constraints removed'
        old=self.bounds_before;present=~np.isnan(old[:,0])
        assert len(bounds)>=len(old) and np.array_equal(bounds[:len(old)][present],old[present]),'Earlier domain changed'
        self.explicit_before=cons;self.bounds_before=bounds
        self.m.setLogfile(f'{self.run}_layer{k}.log')
        build_s=time.perf_counter()-start
        sol=self.m.createOrigSol()
        for v in self.m.getVars():self.m.setSolVal(sol,v,self.values[v.name])
        a=time.perf_counter();audit=self.evaluate(sol);audit_s=time.perf_counter()-a
        row=dict(layer=k,name=name,observed_at=stamp(),variables=self.m.getNVars(),explicit_rows=self.m.getNConss(),
            indexed_relations=len(self.path.rel.relations),cumulative_constraints_verified=True,
            build_and_seed_s=build_s,warm_audit_s=audit_s,warm_audit=audit)
        self.results.append(row);self.previous_passed=False
        if hasattr(self,'acceptance') and not audit['passed']:
            self.m.freeSol(sol)
            row.update(status='COMPLETE_CANDIDATE_REJECTED',SCIP_status='not_started_invalid_full_warm_candidate',
                optimize_wall_s=None,total_layer_s=time.perf_counter()-start)
            self.persist('repair_required',current_layer=k);return False
        if hasattr(self,'acceptance') and not hasattr(self,'guard'):
            from Agent.Publication.engineering_regional_solver import CandidateGuard
            self.guard=CandidateGuard(self)
            self.m.includeConshdlr(self.guard,'regional_continuous_acceptance',
                'all original physical families checked before accepting candidates',
                enfopriority=-1000000,chckpriority=-1000000,needscons=False)
        if audit['passed']:
            added=self.m.addSol(sol,free=True);row['warm_added']=bool(added)
        else:self.m.freeSol(sol)
        # Preserve diagnostic candidates, even when invalid, before native work.
        with gzip.open(OUT/f'{self.run}_layer{k}_candidate.json.gz','wt',encoding='utf-8') as f:json.dump(self.values,f,ensure_ascii=False,allow_nan=False)
        cm=commit_memory();row['pre_optimize_memory']=cm
        if not audit['passed'] and cm['commit_available_GiB']<8:
            row.update(status='RESOURCE_BLOCKED_NO_VERIFIED_INCUMBENT',SCIP_status='not_started_commit_reservation',
                optimize_wall_s=None,total_layer_s=time.perf_counter()-start)
            self.persist('blocked',current_layer=k);return False
        self.m.setRealParam('limits/memory',max(256.,self.m.getMemTotal()/2**20+max(0.,cm['commit_available_GiB']-4.)*1024*.4))
        self.persist('solving',current_layer=k)
        print(json.dumps(dict(layer=k,name=name,warm=audit,variables=row['variables'],rows=row['explicit_rows']),ensure_ascii=False),flush=True)
        a=time.perf_counter();self.m.optimize();row['optimize_wall_s']=time.perf_counter()-a
        row['SCIP_status']=str(self.m.getStatus());row['SCIP_solving_s']=self.m.getSolvingTime()
        best=self.m.getBestSol();row['solution_count']=self.m.getNSols()
        if best is not None:
            row['final_audit']=self.evaluate(best)
            self.previous_passed=row['final_audit']['passed'] and row['final_audit']['coverage_complete']
            if row['final_audit']['passed']:
                self.values={v.name:float(self.m.getSolVal(best,v)) for v in self.m.getVars(transformed=False)}
                witness=OUT/f'{self.run}_layer{k}_witness.json.gz'
                with gzip.open(witness,'wt',encoding='utf-8') as f:json.dump(self.values,f,ensure_ascii=False,allow_nan=False)
                row['witness']=str(witness.relative_to(ROOT));row['witness_sha256']=hashlib.sha256(witness.read_bytes()).hexdigest()
        row['status']='FEASIBLE_VERIFIED' if self.previous_passed else ('PARTIAL_CANDIDATE_VERIFIED' if row.get('final_audit',{}).get('passed') else ('INFEASIBLE_CERTIFIED' if row['SCIP_status']=='infeasible' else 'UNKNOWN_NO_VERIFIED_INCUMBENT'))
        row['total_layer_s']=time.perf_counter()-start
        self.persist('layer_passed' if self.previous_passed else 'blocked',current_layer=k)
        print(json.dumps(row,ensure_ascii=False),flush=True)
        return self.previous_passed

def main():
    p=argparse.ArgumentParser();p.add_argument('--through',type=int,default=10,choices=range(1,12));p.add_argument('--resume',action='store_true');p.add_argument('--repair',action='store_true');p.add_argument('--oracle-first',action='store_true');a=p.parse_args()
    d=Diagnosis()
    d.repair=a.repair
    d.oracle_first=a.oracle_first
    names=['任务选择、塔完成、航次连接与时序','无人机/实体电池身份与守恒（耗能和充电来源尚未绑定）',
           '全路网车辆模式与连续车辆动力学采样','单通道充电与充电温度','七次连续航迹/位姿及前后继连接',
           '车辆起降会合、DEM接触与起降台占用','跨航次及地面相机连续性',
           '航次耗能必要下界与电池容量','整段航次降雨/阵风/白昼窗口','真实地理航迹、动力学、光学与逐段能量积分',
           '完整连续动力学、机体与视线避碰、车辆协同、实体电池和全班次相机验收']
    try:
        start=d.resume() if a.resume else 1
        for k,name in enumerate(names[:a.through],1):
            if k<start:continue
            if not d.step(k,name):break
        full=bool(d.results and d.results[-1]['layer']==11 and d.results[-1]['status']=='FEASIBLE_VERIFIED')
        d.base['running']=False;d.persist('diagnostic_boundary',next_unverified=None if full else 'full237 joint acceptance',full_model_feasible=full,regional_task_complete=full)
    except BaseException as exc:
        d.base['running']=False;d.persist('engineering_error',error=dict(type=type(exc).__name__,message=str(exc)));raise

if __name__=='__main__':main()
