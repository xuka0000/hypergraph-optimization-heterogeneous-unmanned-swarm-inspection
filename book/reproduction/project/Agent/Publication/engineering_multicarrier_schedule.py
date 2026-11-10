"""Joint discrete fleet block with original ownership and concurrent airspace.

The continuous planner supplies current per-carrier endpoint trajectories.
No serialized-flight constraint. Collision cuts refer to checked trajectory
pairs; the resulting bound belongs to this conditional discrete subproblem.
"""
import time,os,math
from pyscipopt import Model,quicksum

def subset_certificate(data,scene,initial_soc=1.):
    """Exact path-subset DP plus handling lower bounds, for this fixed block.

    The certificate is issued only if two sorties on either carrier cannot
    improve the constructed complete one-sortie-per-carrier incumbent.
    No nonlinear-model, road-block, or collision-free global claim is made.
    """
    n=len(data['tasks']);allmask=(1<<n)-1;cfg=scene['joint_reference_profile'];pre=cfg['recovery']['preparation_seconds'];post=cfg['recovery']['recovery_seconds'];tables=[];parents=[];rows=[]
    lookup={(r['vehicle'],r['i'],r['j']):r for r in data['legs']};start=time.perf_counter()
    if len(scene['fleet']['vehicles'])!=2 or initial_soc not in (1.,.2):return None
    fast=.7*cfg['energy']['pair_capacity_Wh']/1800;factor=1/fast if initial_soc==.2 else 0.
    for v in range(2):
        dp={};parent={}
        for i in range(n):
            if (v,n,i) in lookup:dp[1<<i,i]=lookup[v,n,i]['duration_s']+data['dwell_s'][i]+factor*(lookup[v,n,i]['energy_Wh']+data['dwell_Wh'][i])
        for mask in range(1,allmask+1):
            for j in range(n):
                if (mask,j) not in dp:continue
                for k in range(n):
                    if mask>>k&1 or (v,j,k) not in lookup:continue
                    key=(mask|1<<k,k);val=dp[mask,j]+lookup[v,j,k]['duration_s']+data['dwell_s'][k]+factor*(lookup[v,j,k]['energy_Wh']+data['dwell_Wh'][k])
                    if val<dp.get(key,math.inf):dp[key]=val;parent[key]=j
        table={0:(0.,None)}
        for mask in range(1,allmask+1):
            candidates=[(dp[mask,j]+lookup[v,j,n+1]['duration_s']+factor*lookup[v,j,n+1]['energy_Wh'],j) for j in range(n) if (mask,j) in dp and (v,j,n+1) in lookup]
            if candidates:table[mask]=min(candidates)
        tables.append(table);parents.append(parent)
    candidates=[]
    for mask in range(allmask+1):
        other=allmask^mask
        if mask in tables[0] and other in tables[1]:
            durations=[tables[0][mask][0]+pre+post if mask else 0,tables[1][other][0]+pre+post if other else 0]
            candidates.append((max(durations),mask))
    if not candidates:return None
    best,mask=min(candidates);masks=[mask,allmask^mask];energy_ok=True
    for v,mask in enumerate(masks):
        if not mask:continue
        j=tables[v][mask][1];route=[j];current=mask
        while (current,j) in parents[v]:j0=parents[v][current,j];current^=1<<j;j=j0;route.append(j)
        route.reverse();seq=[n]+route+[n+1];energy=sum(lookup[v,i,j]['energy_Wh'] for i,j in zip(seq,seq[1:]))+sum(data['dwell_Wh'][i] for i in route)
        energy_ok &= energy<=cfg['energy']['pair_capacity_Wh']*(1-cfg['energy']['minimum_SOC'])
        if initial_soc==.2:energy_ok &= energy<=.7*cfg['energy']['pair_capacity_Wh']
        flight=sum(lookup[v,i,j]['duration_s'] for i,j in zip(seq,seq[1:]))+sum(data['dwell_s'][i] for i in route)
        rows.append(dict(vehicle=v,task_indices=route,flight_s=flight,energy_Wh=energy,charge_duration_s=factor*energy))
    import networkx as nx
    minimum=[];mine=[]
    for v in range(2):
        tg=nx.DiGraph();eg=nx.DiGraph()
        for (vv,i,j),r in lookup.items():
            if vv==v:tg.add_edge(i,j,weight=r['duration_s']+(data['dwell_s'][j] if j<n else 0));eg.add_edge(i,j,weight=r['energy_Wh']+(data['dwell_Wh'][j] if j<n else 0))
        minimum.append(nx.shortest_path_length(tg,n,n+1,weight='weight'));mine.append(nx.shortest_path_length(eg,n,n+1,weight='weight'))
    two_lower=[max(pre+d+2*post,2*pre+d+post,pre+d+post+2*factor*e) for d,e in zip(minimum,mine)]
    return dict(method='exact subset Hamiltonian DP + carrier handling lower bound',seconds=time.perf_counter()-start,
        masks_per_vehicle=len(tables[0]),candidate_makespan_s=best,minimum_nonempty_flight_s=minimum,
        two_sortie_carrier_lower_bound_s=two_lower,certified_conditional_optimum=bool(energy_ok and min(two_lower)>=best-1e-8),
        bound_scope='fixed leg costs, fixed endpoints, no pairwise spatial collisions; not the full joint model',
        energy_feasible=bool(energy_ok),initial_SOC=initial_soc,minimum_nonempty_flight_energy_Wh=mine,routes=rows)


def solve(data,scene,log_path,initial_soc=1.,conflicts=(),objective='makespan',makespan_cap=None,warm=None):
    n=len(data['tasks']);L=range(n);N=range(n);V=range(len(scene['fleet']['vehicles']))
    vehicles=scene['fleet']['vehicles'];uavs=scene['fleet']['aircraft'];pairs=scene['fleet']['battery_pairs']
    U=range(len(uavs));P=range(len(pairs));uv={u:next(v for v in V if vehicles[v]['id']==uavs[u]['carrier_id']) for u in U};pv={p:next(v for v in V if vehicles[v]['id']==pairs[p]['carrier_id']) for p in P}
    cfg=scene['joint_reference_profile'];H=float(data['horizon_s']);M=4*H;capacity=cfg['energy']['pair_capacity_Wh'];reserve=capacity*cfg['energy']['minimum_SOC']
    prep=cfg['recovery']['preparation_seconds'];post=cfg['recovery']['recovery_seconds'];fast=.7*capacity/1800;tail=.1*capacity/1200;fulltime=.9*capacity/fast+.1*capacity/tail
    costs={(r['vehicle'],r['i'],r['j']):r for r in data['legs']};dwell=data['dwell_s'];dw=data['dwell_Wh'];S=n;R=n+1
    model=Model('original_fleet_joint');model.hideOutput();cwd=os.getcwd()
    try:os.chdir(log_path.parent);model.setLogfile(log_path.name)
    finally:os.chdir(cwd)
    model.setParam('limits/gap',.05);model.setParam('limits/memory',3072);model.setParam('parallel/maxnthreads',1)
    def var(name,ub=H,b=False,lb=0):return model.addVar(name,lb=lb,ub=1 if b else ub,vtype='B' if b else 'C')
    active={l:var(f'on_{l}',b=True) for l in L};x={(i,l):var(f'task_{i}_{l}',b=True) for i in N for l in L}
    owner={(v,l):var(f'vehicle_{v}_{l}',b=True) for v in V for l in L}
    h={(v,i,j,l):var(f'arc_{v}_{i}_{j}_{l}',b=True) for v,i,j in costs for l in L}
    y={(u,l):var(f'uav_{u}_{l}',b=True) for u in U for l in L};bp={(p,l):var(f'battery_{p}_{l}',b=True) for p in P for l in L}
    launch={l:var(f'launch_{l}') for l in L};land={l:var(f'land_{l}') for l in L};t={(i,l):var(f'time_{i}_{l}') for i in N for l in L}
    energy={l:var(f'flight_Wh_{l}',capacity) for l in L};e0={l:var(f'precharge_Wh_{l}',capacity) for l in L};el={l:var(f'launch_Wh_{l}',capacity) for l in L}
    estate={(p,l):var(f'pair_Wh_{p}_{l}',capacity,lb=reserve) for p in P for l in range(n+1)}
    q={(p,l):var(f'charge_Wh_{p}_{l}',capacity) for p in P for l in L};w={(p,l):var(f'draw_Wh_{p}_{l}',capacity) for p in P for l in L}
    cs={l:var(f'charge_start_{l}') for l in L};ce={l:var(f'charge_end_{l}') for l in L};charging={l:var(f'charging_{l}',b=True) for l in L}
    finish=var('return_makespan');total=var('aircraft_Wh',n*capacity)
    for i in N:model.addCons(quicksum(x[i,l] for l in L)==1)
    for p in P:model.addCons(estate[p,0]==capacity*initial_soc)
    def curve(E,l,tag):
        z=var(f'tail_{tag}_{l}',b=True);f=var(f'chargecurve_{tag}_{l}',fulltime)
        model.addCons(E<=.9*capacity+.1*capacity*z);model.addCons(E>=.9*capacity*z)
        lo=E/fast;hi=.9*capacity/fast+(E-.9*capacity)/tail
        model.addCons(f-lo<=fulltime*z);model.addCons(f-lo>=-fulltime*z)
        model.addCons(f-hi<=3*fulltime*(1-z));model.addCons(f-hi>=-3*fulltime*(1-z));return f
    for l in L:
        model.addCons(quicksum(x[i,l] for i in N)>=active[l]);model.addCons(quicksum(x[i,l] for i in N)<=n*active[l])
        model.addCons(quicksum(owner[v,l] for v in V)==active[l]);model.addCons(quicksum(y[u,l] for u in U)==active[l]);model.addCons(quicksum(bp[p,l] for p in P)==active[l])
        for v in V:
            model.addCons(quicksum(y[u,l] for u in U if uv[u]==v)==owner[v,l]);model.addCons(quicksum(bp[p,l] for p in P if pv[p]==v)==owner[v,l])
            model.addCons(quicksum(h[a,i,j,l] for a,i,j in costs if a==v and i==S)==owner[v,l]);model.addCons(quicksum(h[a,i,j,l] for a,i,j in costs if a==v and j==R)==owner[v,l])
            for i in N:model.addCons(quicksum(h[a,b,j,l] for a,b,j in costs if a==v and b==i)==quicksum(h[a,b,j,l] for a,b,j in costs if a==v and j==i))
            for a,i,j in costs:
                if a==v:model.addCons(h[a,i,j,l]<=owner[v,l])
        for i in N:
            model.addCons(quicksum(h[v,a,j,l] for v,a,j in costs if a==i)==x[i,l]);model.addCons(t[i,l]<=H*x[i,l])
        model.addCons(launch[l]<=H*active[l]);model.addCons(land[l]<=H*active[l])
        for (v,i,j),r in costs.items():
            begin=launch[l] if i==S else t[i,l]+dwell[i];end=land[l] if j==R else t[j,l]
            model.addCons(end>=begin+r['duration_s']-M*(1-h[v,i,j,l]));model.addCons(end<=begin+r['duration_s']+M*(1-h[v,i,j,l]))
        model.addCons(land[l]-launch[l]==quicksum(r['duration_s']*h[v,i,j,l] for (v,i,j),r in costs.items())+quicksum(dwell[i]*x[i,l] for i in N))
        model.addCons(energy[l]==quicksum(r['energy_Wh']*h[v,i,j,l] for (v,i,j),r in costs.items())+quicksum(dw[i]*x[i,l] for i in N))
        model.addCons(el[l]>=energy[l]+reserve*active[l]);model.addCons(el[l]<=capacity*active[l]);model.addCons(e0[l]<=capacity*active[l])
        for p in P:
            model.addCons(e0[l]>=estate[p,l]-capacity*(1-bp[p,l]));model.addCons(e0[l]<=estate[p,l]+capacity*(1-bp[p,l]))
            model.addCons(q[p,l]<=capacity*bp[p,l]);model.addCons(w[p,l]<=capacity*bp[p,l]);model.addCons(w[p,l]<=energy[l]);model.addCons(w[p,l]>=energy[l]-capacity*(1-bp[p,l]))
            model.addCons(estate[p,l+1]==estate[p,l]+q[p,l]-w[p,l])
        model.addCons(el[l]==e0[l]+quicksum(q[p,l] for p in P))
        model.addCons(ce[l]-cs[l]==curve(el[l],l,'end')-curve(e0[l],l,'start'))
        model.addCons(ce[l]>=cs[l]);model.addCons(ce[l]-cs[l]<=H*charging[l]);model.addCons(charging[l]<=active[l])
        model.addCons(cs[l]<=H*active[l]);model.addCons(ce[l]<=H*active[l]);model.addCons(launch[l]>=ce[l]+prep*active[l]);model.addCons(finish>=land[l]+post*active[l])
        if l<n-1:model.addCons(active[l]>=active[l+1]);model.addCons(launch[l]<=launch[l+1]+H*(1-active[l+1]))
    for l in L:
        for k in range(l+1,n):
            for u in U:model.addCons(land[l]+post<=launch[k]-prep+M*(2-y[u,l]-y[u,k]))
            for p in P:model.addCons(land[l]+post+cfg['energy']['cooling_seconds']<=cs[k]+M*(2-bp[p,l]-bp[p,k]))
            for v in V:
                order=var(f'charger_order_{v}_{l}_{k}',b=True);off=M*(4-owner[v,l]-owner[v,k]-charging[l]-charging[k])
                model.addCons(ce[l]<=cs[k]+M*(1-order)+off);model.addCons(ce[k]<=cs[l]+M*order+off)
                for aa,(a0,a1) in enumerate([(launch[l]-prep,launch[l]),(land[l],land[l]+post)]):
                    for bb,(b0,b1) in enumerate([(launch[k]-prep,launch[k]),(land[k],land[k]+post)]):
                        o=var(f'pad_order_{v}_{l}_{k}_{aa}_{bb}',b=True);off=M*(2-owner[v,l]-owner[v,k])
                        model.addCons(a1<=b0+M*(1-o)+off);model.addCons(b1<=a0+M*o+off)
    certificate=subset_certificate(data,scene,initial_soc)
    if certificate:
        for v in V:
            count=quicksum(owner[v,l] for l in L);minimum=certificate['minimum_nonempty_flight_s'][v]
            used=var(f'carrier_used_{v}',b=True)
            model.addCons(count<=n*used);model.addCons(count>=used)
            model.addCons(finish>=prep+minimum+post*count-M*(1-used))
            model.addCons(finish>=prep*count+minimum+post-M*(1-used))
            model.addCons(finish>=prep+minimum+post+quicksum(q[p,l]/fast for p in P if pv[p]==v for l in L)-M*(1-used))
        if certificate['certified_conditional_optimum']:
            model.addCons(finish>=certificate['candidate_makespan_s'])
    # Safe pairwise cuts on physical occupancy intervals for selected arcs.
    # Both alternative temporal orders remain available; no blanket serial rule.
    for num,c in enumerate(conflicts):
        av,ai,aj,al=c['arc_a'];bv,bi,bj,bl=c['arc_b'];oa,ob=c['occupancy_a_s'],c['occupancy_b_s']
        ba=launch[al] if ai==S else t[ai,al]+dwell[ai];bb=launch[bl] if bi==S else t[bi,bl]+dwell[bi]
        o=var(f'airspace_order_{num}',b=True);off=M*(2-h[av,ai,aj,al]-h[bv,bi,bj,bl])
        model.addCons(ba+oa[1]<=bb+ob[0]+M*(1-o)+off);model.addCons(bb+ob[1]<=ba+oa[0]+M*o+off)
    model.addCons(total==quicksum(energy.values()))
    # Identical resources within a carrier: remove label symmetry, not choices.
    for v in V:
        us=[u for u in U if uv[u]==v];ps=[p for p in P if pv[p]==v]
        for k,u in enumerate(us[1:],1):
            for l in L:model.addCons(y[u,l]<=quicksum(y[us[k-1],j] for j in range(l)))
        for k,p in enumerate(ps[1:],1):
            for l in L:model.addCons(bp[p,l]<=quicksum(bp[ps[k-1],j] for j in range(l)))
    if makespan_cap is not None:model.addCons(finish<=makespan_cap)
    if objective=='energy' and makespan_cap is None:raise ValueError('Prior makespan cap required')
    model.setObjective(total if objective=='energy' else finish,'minimize')
    if certificate and certificate['energy_feasible']:
        warm_sol=model.createPartialSol();routes=sorted(certificate['routes'],key=lambda r:r['charge_duration_s']);chosen={l:r for l,r in enumerate(routes)}
        def put(z,value):model.setSolVal(warm_sol,z,float(value))
        for l in L:
            r=chosen.get(l);v=r['vehicle'] if r else None;seq=[S]+r['task_indices']+[R] if r else [];edges=set(zip(seq,seq[1:]))
            put(active[l],bool(r))
            for i in N:put(x[i,l],r is not None and i in r['task_indices'])
            for vv in V:put(owner[vv,l],vv==v)
            for vv,i,j in costs:put(h[vv,i,j,l],vv==v and (i,j) in edges)
            for u in U:put(y[u,l],r is not None and u==min(uu for uu in U if uv[uu]==v))
            for p in P:put(bp[p,l],r is not None and p==min(pp for pp in P if pv[pp]==v))
            if not conflicts:
                put(launch[l],prep+r['charge_duration_s'] if r else 0);put(land[l],prep+r['charge_duration_s']+r['flight_s'] if r else 0)
            put(cs[l],0);put(ce[l],r['charge_duration_s'] if r else 0);put(charging[l],bool(r and r['charge_duration_s']>0))
        model.addSol(warm_sol)
    started=time.perf_counter();nv,nc=model.getNVars(),model.getNConss();model.optimize()
    out=dict(status=str(model.getStatus()),seconds=time.perf_counter()-started,variables=nv,constraints=nc,initial_SOC=initial_soc,
        objective=objective,time_limit_s=None,target_gap=.05,global_joint_gap=None,conditional_bound_only=True,
        simultaneous_flights_allowed=True,collision_cuts=len(conflicts),vehicle_count=len(vehicles),uav_count=len(uavs),battery_pair_count=len(pairs))
    out['subset_certificate']=certificate
    if not model.getNSols():return out
    out.update(makespan_s=float(model.getVal(finish)),aircraft_energy_Wh=float(model.getVal(total)),conditional_gap=float(model.getGap()),conditional_bound=float(model.getDualbound()))
    sorties=[]
    for l in L:
        if model.getVal(active[l])<.5:continue
        v=next(v for v in V if model.getVal(owner[v,l])>.5);successors={i:j for vv,i,j in costs if vv==v and model.getVal(h[v,i,j,l])>.5};route=[];current=S
        for step in range(n+1):
            current=successors[current]
            if current==R:break
            route.append(current)
        assert route and len(set(route))==len(route)
        u=next(u for u in U if model.getVal(y[u,l])>.5);p=next(p for p in P if model.getVal(bp[p,l])>.5)
        sorties.append(dict(slot=l,vehicle=v,vehicle_id=vehicles[v]['id'],aircraft=u,aircraft_id=uavs[u]['id'],battery_pair=p,battery_pair_id=pairs[p]['id'],task_indices=route,
            launch_s=float(model.getVal(launch[l])),land_s=float(model.getVal(land[l])),energy_used_Wh=float(model.getVal(energy[l])),launch_energy_Wh=float(model.getVal(el[l])),
            energy_before_charge_Wh=float(model.getVal(e0[l])),charge_start_s=float(model.getVal(cs[l])),charge_end_s=float(model.getVal(ce[l])),
            capture_times_s={str(i):float(model.getVal(t[i,l])) for i in route}))
    out['sorties']=sorties;out['terminal_pair_Wh']={pairs[p]['id']:float(model.getVal(estate[p,n])) for p in P}
    return out
