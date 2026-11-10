"""Equivalent shared returns and valid, inexpensive regional relaxations.

This is an implementation repair, not a public baseline or new scientific
model. Historical witnesses are seeds only; no decision is fixed by mapping.
"""
import gzip,json,hashlib,re,math
from pathlib import Path
import numpy as np
from pyscipopt import quicksum
from Agent.Publication.engineering_indexed_constraints import AffineRow


def add_necessary_bounds(j):
    m=j.path.m;p=j.path;r=j.resources
    from Agent.Publication.engineering_regional_resource_envelope import add_resource_envelope
    resource_envelope=add_resource_envelope(j)
    rho=float(j.forecast.values[:,:,2].max())
    floor=float(j.scene['joint_reference_profile']['energy']['hover_power_W'])*math.sqrt(1.225/rho)
    # Every enabled flight spends a single physical pack between launch and
    # recovery. No in-flight recharge exists. These are implied bounds, not
    # a new time limit on the optimizer or a chosen flight duration.
    maximum_airborne=(r.capacity-r.reserve)*3600/floor
    for i,(f,res) in enumerate(zip(p.flights,r.flights)):
        m.addCons(res.energy>=floor*(f.end-f.start)/3600,name=f'diagnostic_energy_lower_bound_{i}')
        m.addCons(f.end-f.start<=maximum_airborne*f.active,name=f'necessary_pack_duration_{i}')
    # Slots are interchangeable within a carrier. Relabeling all associated
    # task/resource/trajectory variables by launch time preserves every plan.
    for carrier,ids in r.by_carrier.items():
        for left,right in zip(ids,ids[1:]):
            a,b=p.flights[left],p.flights[right]
            m.addCons(a.active>=b.active,name=f'canonical_flight_prefix_{carrier}_{right}')
            m.addCons(a.start<=b.start+p.H*(1-b.active),name=f'canonical_launch_order_{carrier}_{right}')
    m.addCons(quicksum(t.root for t in p.tasks)==quicksum(f.active for f in p.flights),name='necessary_one_root_per_flight')
    m.addCons(quicksum(t.last for t in p.tasks)==quicksum(f.active for f in p.flights),name='necessary_one_last_per_flight')
    for i,t in enumerate(p.tasks):
        for v in [t.incoming,t.dwell,t.recovery]:m.chgVarUb(v,maximum_airborne)
        m.addCons(t.incoming+t.dwell+t.recovery<=maximum_airborne*t.active,name=f'necessary_task_duration_{i}')
    for s in j.curves.segments:
        m.chgVarUb(s.duration,maximum_airborne)
        if s.safe_duration is not None:m.chgVarUb(s.safe_duration,max(1.,maximum_airborne))
        m.addCons(j.physics.segment_energy[s.task,s.index]>=floor*s.duration/3600,
            name=f'necessary_segment_energy_{s.task}_{s.index}')
    # Markov's inequality applied to the degree6 physical rate polynomial:
    # |angle''| <= 2*6^2*rate_max/T. Every active ground interval spans at
    # least preparation or recovery. For an inactive slot the unused second
    # derivative has a zero canonical extension (its interval has T=0).
    # This avoids the 1/Tmin^2 zoom-acceleration bound entering T^2 products
    # at ground endpoints, without imposing a new camera hardware limit.
    minimum_ground=min(p.pre,p.post)
    camera_acc_bounds=[2*6**2*x/minimum_ground for x in j.curves.pose_rate[1:]]
    tightened=set()
    for z in j.rendezvous.ground_camera:
        for node in [z['start'],z['end']]:
            for q,v in enumerate(node[2]):
                bound=camera_acc_bounds[q]
                m.chgVarLb(v,max(v.getLbOriginal(),-bound));m.chgVarUb(v,min(v.getUbOriginal(),bound));tightened.add(v.name)
        m.addCons(z['duration']>=minimum_ground*z['active'],name=z['name']+'_necessary_duration')
    dwell=quicksum(t.dwell for t in p.tasks)
    airborne=quicksum(f.end-f.start for f in p.flights)
    m.addCons(airborne>=dwell,name='necessary_global_capture_time')
    m.addCons(r.total_energy>=floor*dwell/3600,name='necessary_global_capture_energy')
    available=sum(float(b['initial_Wh'])-r.reserve for b in j.scene['fleet']['battery_pairs'])
    m.addCons(r.total_energy<=available+quicksum(x.charged for x in r.flights),name='necessary_global_pack_ledger')
    windows=[]
    for interval in j.forecast.intervals:
        if not interval['flight_allowed']:continue
        if windows and windows[-1][1]==interval['start_s']:windows[-1][1]=interval['end_s']
        else:windows.append([interval['start_s'],interval['end_s']])
    if not windows:raise ValueError('No allowed weather window in retained input')
    for i,f in enumerate(p.flights):
        sel=p.rel.selector(f'diagnostic_flight{i}_weather_window',len(windows))
        def rows(k,f=f):
            lo,hi=windows[k]
            return [AffineRow('launch_after_weather_start',[(1,f.start)],lo=lo),AffineRow('land_before_weather_end',[(1,f.end)],hi=hi)]
        p.rel.add(sel,rows,[],f.active,f'diagnostic_flight{i}_weather_interval')
    return dict(power_floor_W=floor,rho_max=rho,windows=windows,maximum_airborne_s=maximum_airborne,
        ground_camera_acc_bounds=camera_acc_bounds,ground_camera_variables_tightened=len(tightened),
        global_rows=3,resource_envelope=resource_envelope,
        proof='nonnegative drag/climb, nonoverlapping task intervals within each flight, telescoping physical-pack ledger')


def mapped_seed(j):
    root=j.root;out=root/'总结/03_实验数据/工程化联合求解/constraint_layers'
    receipt=json.loads((out/'status.json').read_text('utf-8-sig'));last=receipt['layers'][-1]
    if receipt['source_sha256']!=j.record['source']['sha256'] or receipt['model_sha256']!=j.record['model']['sha256']:
        raise ValueError('Source/model identity differs from old witness')
    witness=root/last['witness']
    if hashlib.sha256(witness.read_bytes()).hexdigest()!=last['witness_sha256']:raise ValueError('Old witness changed')
    with gzip.open(witness,'rt',encoding='utf-8') as f:old=json.load(f)
    m=j.path.m;sol=m.createOrigSol();N=len(j.path.tasks);mapping={};defaults=[]
    def last_task(k):return int(round(old[f'flight_{k}_last']))
    for v in m.getVars(transformed=False):
        name=v.name;source=name
        extra_road=re.fullmatch(r'vehicle(\d+)_phase(\d+)(.*)',name)
        if extra_road and int(extra_road[2])>=256:
            # Extend the old stationary terminal state by zero-duration
            # copies. No new source road or flight choice is fixed in SCIP.
            source=f'vehicle{extra_road[1]}_phase255{extra_road[3]}'
        a=re.fullmatch(r'flight(\d+)_recovery_node(\d+)(.*)',name)
        b=re.fullmatch(r'flight(\d+)_recovery_segment(\d+)(.*)',name)
        c=re.fullmatch(r'segment_task(\d+)_index(\d+)_Wh',name)
        e=re.fullmatch(r'flight(\d+)_recovery_Wh',name)
        fixed=re.fullmatch(r'flight(\d+)_recovery_identity',name)
        parked_time=re.fullmatch(r'necessary_carrier(\d+)_phase(\d+)_parked_duration',name)
        if a:
            k,node,suffix=int(a[1]),int(a[2]),a[3];t=last_task(k)
            source=f'task{t}_curve_node{node+4}{suffix}' if t else None
        elif b:
            k,index,suffix=int(b[1]),int(b[2]),b[3];t=last_task(k)
            source=f'task{t}_segment{index}{suffix}' if t else None
        elif c and int(c[1])>N:
            k=int(c[1])-N;t=last_task(k)
            source=f'segment_task{t}_index{c[2]}_Wh' if t else None
        elif e:
            t=last_task(int(e[1]));source=f'task{t}_recovery_Wh' if t else None
        if parked_time:
            prefix=f'vehicle{parked_time[1]}_phase{min(int(parked_time[2]),255)}'
            value=old[prefix+'_duration']*old[prefix+'_parked']
        elif name.endswith('_safe_duration') and ('segment' in name):
            source_prefix=source[:-len('_safe_duration')] if source else None
            value=old[source_prefix+'_duration']+1-old[source_prefix+'_enabled'] if source_prefix else 1.
        elif fixed:value=float(fixed[1])
        elif source is None:
            value=max(v.getLbOriginal(),min(v.getUbOriginal(),0.));defaults.append(name)
        elif source not in old:
            bit=re.fullmatch(r'(.+)_bit(\d+)',source)
            if bit and bit[1] in old:value=float((int(round(old[bit[1]]))>>int(bit[2]))&1)
            else:raise ValueError('Unmapped active/original variable '+name+' <- '+source)
        else:value=old[source];mapping[name]=source
        m.setSolVal(sol,v,value)
    # Active polynomials and their physical clocks must be identical under
    # the forward/inverse relabeling, before any new optimization is run.
    differences=[];active=0
    for k,d in enumerate(j.curves.recoveries,1):
        t=last_task(k)
        if not t:continue
        for v in [v for node in d['nodes'] for rows in (node.position,node.pose) for row in rows for v in row]:
            differences.append(abs(m.getSolVal(sol,v)-old[mapping[v.name]]))
        for seg in d['segments']:
            if m.getSolVal(sol,seg.active)<.5:continue
            active+=1
            start=old[f'task_{t}_time']+old[f'task_{t}_dwell']+sum(old[f'task{t}_segment{h}_duration'] for h in range(4,seg.index))
            differences.append(abs(m.getSolVal(sol,seg.clock)-start))
    receipt=dict(source_run=receipt['run_id'],witness=last['witness'],witness_sha256=last['witness_sha256'],
        copied_variables=len(mapping),inactive_return_defaults=len(defaults),active_return_segments=active,
        maximum_forward_inverse_state_clock_error=max(differences,default=0.),
        fixed_original_decisions=0,coverage_preserved=True,proof='selected last-task return is relabeled to its flight; inactive task return states have no active physical constraints')
    if receipt['maximum_forward_inverse_state_clock_error']>1e-6:raise ValueError('Shared-return clock/state mismatch')
    old.clear();return sol,receipt


def build_shared_repair(j,add_seed=True):
    road_proof=json.loads((j.root/'research_quality/inspection_deferred_vehicle_regression.json').read_text('utf-8-sig'))
    if not road_proof.get('passed'):raise ValueError('Deferred vehicle rows require native/oracle regression')
    j.oracle_first=True;j.build(shared_recovery=True,attach_guard=False,deferred_vehicle=True)
    j.necessary_bounds=add_necessary_bounds(j)
    sol,mapping=mapped_seed(j);m=j.path.m
    if getattr(j,'restore_dem_grid',False):
        mapping['terrain_reseed']=j.terrain_index.reseed(m,sol)
    j.physics.seed_solution=sol
    for k,f in enumerate(j.path.flights):
        if m.getSolVal(sol,f.active)<.5:continue
        first=int(round(m.getSolVal(sol,f.first.value)))-1
        j.safety.add_pad_point(first,0,0.)
        j.safety.add_pad_point(len(j.path.tasks)+k,6,1.)
    j.physics.seed_solution=None;j.path.rel.lock(m.getVars())
    from scripts.diagnose_inspection_constraint_layers import Diagnosis
    j.m=m;j.record_run=False
    audit=Diagnosis.evaluate(j,sol)
    j.output.mkdir(parents=True,exist_ok=True)
    j.warm_start_audit=dict(mapping=mapping,audit=audit,original_decisions_fixed=0,
        regional_task_complete=False,coverage_complete=audit['coverage_complete'],
        deferred_vehicle_rows=True,required_road_points=8*sum(j.road_phases),
        necessary_bounds=j.necessary_bounds)
    (j.output/'shared_recovery_acceptance.json').write_text(json.dumps(j.warm_start_audit,ensure_ascii=False,indent=2),'utf-8')
    if not audit['passed']:raise ValueError('Mapped full candidate rejected: '+json.dumps(audit,ensure_ascii=False))
    j.last_acceptance=audit['continuous_acceptance'];j.install_handlers()
    if add_seed:j.warm_start_audit['added']=bool(m.addSol(sol,free=True))
    else:
        m.freeSol(sol);j.warm_start_audit.update(added=False,role='regression_witness_only_not_valid_for_all237_requirement')
    regression=j.root/'research_quality/inspection_indexed_presolve_regression.json'
    proof=json.loads(regression.read_text('utf-8-sig'))
    current=hashlib.sha256((j.root/'Agent/Publication/engineering_indexed_constraints.py').read_bytes()).hexdigest()
    if not proof.get('passed') or proof['code_sha256']!=current:raise ValueError('Primal presolve requires current independent registry regression')
    j.path.rel.enable_primal_presolve()
    j.warm_start_audit['primal_presolve_enabled']=True
    j.warm_start_audit['dual_reductions_enabled']=False
    return j
