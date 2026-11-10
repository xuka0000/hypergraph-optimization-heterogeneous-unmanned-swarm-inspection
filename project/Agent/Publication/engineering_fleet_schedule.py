"""Joint task/sortie/aircraft/battery/partial-charge MILP at a parked carrier.

Leg geometry and costs are provided by the continuous planner. This kernel
does not claim a full nonlinear-model bound or moving-carrier optimization.
"""
import math, os, time
from pyscipopt import Model, quicksum


def solve(costs,energies,dwell,dwell_wh,profile,initial_soc,log_path,serial_airspace=False,airspace_lower_bound_s=None,objective='makespan',makespan_cap_s=None):
    n=len(dwell);L=range(n);U=range(2);P=range(4);S=n;R=n+1
    H=3600.;M=4*H;cap=profile['energy']['pair_capacity_Wh'];reserve=profile['energy']['minimum_SOC']*cap
    pre=profile['recovery']['preparation_seconds'];post=profile['recovery']['recovery_seconds']
    fast=.7*cap/1800;tail=.1*cap/1200;fulltime=.9*cap/fast+.1*cap/tail
    model=Model('task_sortie_battery_joint');model.hideOutput()
    cwd=os.getcwd()
    try:os.chdir(log_path.parent);model.setLogfile(log_path.name)
    finally:os.chdir(cwd)
    model.setParam('limits/gap',.05);model.setParam('limits/memory',3072);model.setParam('parallel/maxnthreads',1)
    def var(name,lb=0,ub=H,binary=False):return model.addVar(name=name,lb=lb,ub=1 if binary else ub,vtype='B' if binary else 'C')
    b={l:var(f'active_{l}',binary=True) for l in L}
    x={(i,l):var(f'task_{i}_{l}',binary=True) for i in range(n) for l in L}
    h={(i,j,l):var(f'arc_{i}_{j}_{l}',binary=True) for l in L for i,j in costs}
    y={(u,l):var(f'aircraft_{u}_{l}',binary=True) for u in U for l in L}
    beta={(p,l):var(f'pack_{p}_{l}',binary=True) for p in P for l in L}
    launch={l:var(f'launch_{l}') for l in L};land={l:var(f'land_{l}') for l in L}
    t={(i,l):var(f'capture_{i}_{l}') for i in range(n) for l in L}
    use={l:var(f'energy_used_{l}',ub=cap) for l in L}
    e0={l:var(f'energy_before_charge_{l}',ub=cap) for l in L}
    el={l:var(f'launch_energy_{l}',ub=cap) for l in L}
    estate={(p,l):var(f'pack_energy_{p}_{l}',lb=reserve,ub=cap) for p in P for l in range(n+1)}
    q={(p,l):var(f'charge_Wh_{p}_{l}',ub=cap) for p in P for l in L}
    w={(p,l):var(f'discharge_Wh_{p}_{l}',ub=cap) for p in P for l in L}
    cs={l:var(f'charge_start_{l}') for l in L};ce={l:var(f'charge_end_{l}') for l in L}
    charging={l:var(f'charging_active_{l}',binary=True) for l in L}
    makespan=var('makespan');total_energy=var('total_energy',ub=n*cap)
    for i in range(n):model.addCons(quicksum(x[i,l] for l in L)==1)
    for p in P:model.addCons(estate[p,0]==initial_soc*cap)
    def curve(E,l,tag):
        z=var(f'tail_region_{tag}_{l}',binary=True);f=var(f'charge_curve_{tag}_{l}',ub=fulltime)
        model.addCons(E<=.9*cap+.1*cap*z);model.addCons(E>=.9*cap*z)
        low=E/fast;high=.9*cap/fast+(E-.9*cap)/tail
        model.addCons(f-low<=fulltime*z);model.addCons(f-low>=-fulltime*z)
        model.addCons(f-high<=3*fulltime*(1-z));model.addCons(f-high>=-3*fulltime*(1-z))
        return f
    for l in L:
        model.addCons(quicksum(x[i,l] for i in range(n))>=b[l]);model.addCons(quicksum(x[i,l] for i in range(n))<=n*b[l])
        model.addCons(quicksum(y[u,l] for u in U)==b[l]);model.addCons(quicksum(beta[p,l] for p in P)==b[l])
        model.addCons(quicksum(h[i,j,l] for i,j in costs if i==S)==b[l]);model.addCons(quicksum(h[i,j,l] for i,j in costs if j==R)==b[l])
        for i in range(n):
            model.addCons(quicksum(h[a,j,l] for a,j in costs if a==i)==x[i,l])
            model.addCons(quicksum(h[a,j,l] for a,j in costs if j==i)==x[i,l])
            model.addCons(t[i,l]<=H*x[i,l])
        model.addCons(launch[l]<=H*b[l]);model.addCons(land[l]<=H*b[l])
        for (i,j),duration in costs.items():
            begin=launch[l] if i==S else t[i,l]+dwell[i]
            end=land[l] if j==R else t[j,l]
            model.addCons(end>=begin+duration-M*(1-h[i,j,l]))
            model.addCons(end<=begin+duration+M*(1-h[i,j,l]))
        model.addCons(use[l]==quicksum(energies[i,j]*h[i,j,l] for i,j in costs)+quicksum(dwell_wh[i]*x[i,l] for i in range(n)))
        # Redundant exact identity strengthens the LP without changing any
        # feasible integer route, time, energy or scheduling decision.
        model.addCons(land[l]-launch[l]==quicksum(costs[i,j]*h[i,j,l] for i,j in costs)+quicksum(dwell[i]*x[i,l] for i in range(n)))
        model.addCons(el[l]>=use[l]+reserve*b[l]);model.addCons(el[l]<=cap*b[l]);model.addCons(e0[l]<=cap*b[l])
        for p in P:
            model.addCons(e0[l]>=estate[p,l]-cap*(1-beta[p,l]));model.addCons(e0[l]<=estate[p,l]+cap*(1-beta[p,l]))
            model.addCons(q[p,l]<=cap*beta[p,l]);model.addCons(w[p,l]<=cap*beta[p,l])
            model.addCons(w[p,l]<=use[l]);model.addCons(w[p,l]>=use[l]-cap*(1-beta[p,l]))
            model.addCons(estate[p,l+1]==estate[p,l]+q[p,l]-w[p,l])
        model.addCons(el[l]==e0[l]+quicksum(q[p,l] for p in P))
        model.addCons(ce[l]-cs[l]==curve(el[l],l,'end')-curve(e0[l],l,'begin'))
        model.addCons(ce[l]-cs[l]>=0);model.addCons(ce[l]-cs[l]<=H*charging[l]);model.addCons(charging[l]<=b[l])
        model.addCons(cs[l]<=H*b[l]);model.addCons(ce[l]<=H*b[l])
        model.addCons(launch[l]>=ce[l]+pre*b[l]);model.addCons(makespan>=land[l]+post*b[l])
        if l<n-1:
            model.addCons(b[l]>=b[l+1]);model.addCons(launch[l]<=launch[l+1]+H*(1-b[l+1]))
    # Label symmetry only: identical aircraft and identical initial battery pairs.
    model.addCons(y[0,0]==b[0]);model.addCons(beta[0,0]==b[0])
    for l in L:
        for k in range(l+1,n):
            for u in U:model.addCons(land[l]+post<=launch[k]-pre+M*(2-y[u,l]-y[u,k]))
            for p in P:model.addCons(land[l]+post+profile['energy']['cooling_seconds']<=cs[k]+M*(2-beta[p,l]-beta[p,k]))
            order=var(f'charger_order_{l}_{k}',binary=True)
            model.addCons(ce[l]<=cs[k]+M*(1-order)+M*(2-charging[l]-charging[k]))
            model.addCons(ce[k]<=cs[l]+M*order+M*(2-charging[l]-charging[k]))
            if serial_airspace:model.addCons(land[l]<=launch[k]+M*(2-b[l]-b[k]))
            # One handling position: both preparation and recovery intervals.
            for aa,(a0,a1) in enumerate([(launch[l]-pre,launch[l]),(land[l],land[l]+post)]):
                for bb,(b0,b1) in enumerate([(launch[k]-pre,launch[k]),(land[k],land[k]+post)]):
                    o=var(f'pad_order_{l}_{k}_{aa}_{bb}',binary=True)
                    model.addCons(a1<=b0+M*(1-o)+M*(2-b[l]-b[k]));model.addCons(b1<=a0+M*o+M*(2-b[l]-b[k]))
    model.addCons(total_energy==quicksum(use.values()))
    if serial_airspace:
        model.addCons(makespan>=quicksum(land[l]-launch[l] for l in L)+pre+post)
        if airspace_lower_bound_s is not None:model.addCons(makespan>=float(airspace_lower_bound_s))
        # Earliest source-to-sink path is a lower bound for ANY nonempty sortie.
        import networkx as nx
        tg=nx.DiGraph();eg=nx.DiGraph()
        for i,j in costs:
            tg.add_edge(i,j,weight=costs[i,j]+(dwell[i] if i<n else 0))
            eg.add_edge(i,j,weight=energies[i,j]+(dwell_wh[i] if i<n else 0))
        min_flight=nx.shortest_path_length(tg,S,R,weight='weight')
        min_energy=nx.shortest_path_length(eg,S,R,weight='weight')
        # All charges use one channel. The latest charge ends before its
        # preparation, nonempty flight and recovery; no charging work omitted.
        model.addCons(makespan>=quicksum(ce[l]-cs[l] for l in L)+pre+post+min_flight)
        first_charge_lb=max(0,min_energy-max(0,initial_soc*cap-reserve))/fast
        model.addCons(makespan>=quicksum(land[l]-launch[l] for l in L)+pre+post+first_charge_lb)
    if makespan_cap_s is not None:model.addCons(makespan<=float(makespan_cap_s))
    if objective not in ('makespan','energy'):raise ValueError(objective)
    if objective=='energy' and makespan_cap_s is None:raise ValueError('Energy stage requires a bound on the prior makespan decision')
    model.setObjective(makespan if objective=='makespan' else total_energy,'minimize');nv,nc=model.getNVars(),model.getNConss()
    started=time.perf_counter();model.optimize()
    receipt=dict(status=str(model.getStatus()),seconds=time.perf_counter()-started,variables=nv,constraints=nc,
        target_gap=.05,time_limit_s=None,model_scope='free sortie formation +2aircraft+4pairs+one charger/handling position; fixed parked carrier/leg geometry',
        slot_count=n,initial_SOC=initial_soc,serial_airspace=serial_airspace,full_joint_model=False,
        redundant_formulation_strengthening='route duration identity; serial airspace, single charger and first-charge lower bounds; same integer feasible domain',
        current_objective=objective, makespan_cap_s=makespan_cap_s,
        objective_scope='all nine tasks required; energy stage, when enabled, preserves the previous makespan incumbent within its declared tolerance')
    if not model.getNSols():return receipt
    receipt.update(makespan_s=float(model.getVal(makespan)),aircraft_Wh=float(model.getVal(total_energy)),gap=float(model.getGap()))
    receipt['bound_s' if objective=='makespan' else 'energy_bound_Wh']=float(model.getDualbound())
    sorties=[]
    for l in L:
        if model.getVal(b[l])<.5:continue
        successor={i:j for i,j in costs if model.getVal(h[i,j,l])>.5};seq=[];current=S
        for _ in range(n+1):
            current=successor[current]
            if current==R:break
            seq.append(current)
        assert seq and len(seq)==len(set(seq))
        sorties.append(dict(slot=l,task_indices=seq,aircraft=next(u for u in U if model.getVal(y[u,l])>.5),
            battery_pair=next(p for p in P if model.getVal(beta[p,l])>.5),launch_s=float(model.getVal(launch[l])),land_s=float(model.getVal(land[l])),
            charge_start_s=float(model.getVal(cs[l])),charge_end_s=float(model.getVal(ce[l])),
            charge_active=bool(model.getVal(charging[l])>.5),energy_before_charge_Wh=float(model.getVal(e0[l])),
            launch_energy_Wh=float(model.getVal(el[l])),energy_used_Wh=float(model.getVal(use[l])),
            capture_times_s={str(i):float(model.getVal(t[i,l])) for i in seq}))
    receipt['sorties']=sorties;receipt['terminal_pack_Wh']={str(p):float(model.getVal(estate[p,n])) for p in P}
    return receipt
