"""Preemptible charging on common variable-time events in the joint model.

For the declared single-channel carrier, constant capacity/efficiency, zero
idle draw, and two constant power bands, each next-flight charging job has
one release, one deadline, and at most two nonzero SOC phases. A feasible
preemptive schedule can switch at releases, vehicle availability boundaries,
or phase completions (EDF exchange within a common availability interval).
Their count supplies the finite event bound; no fixed time grid is used.
"""


def add_event_charging(j):
    cfg=j.cfg;energy=cfg['energy'];capacity=energy['pair_capacity_Wh'];eta=energy['charge_efficiency']
    vehicles=j.scene['fleet']['vehicles'];visits=sorted({(v,k) for v,k in j.arr})
    if energy['idle_power_W']!=0 or cfg['vehicle']['charging_on_road']:
        raise ValueError('Event-count proof requires the declared zero-idle, stopped-carrier charging profile')
    if any(v['charger_channels']!=1 for v in vehicles):
        raise ValueError('Multi-channel event-count proof is not installed')
    events=2+2*j.n+2*len(visits)+2*j.n
    K=events-1;j.charge_event_count=events
    seeds=getattr(j,'initial_charge_intervals',[])
    seed_times=sorted({0.,j.H}|{float(r[key]) for r in seeds for key in ['start_s','end_s']})
    if len(seed_times)>events:raise ValueError('Initializer exceeds derived event bound')
    seed_times+= [j.H]*(events-len(seed_times))
    tau=[j.var(f'charge_event_time_{k}',0,j.H,seed_times[k],group='event_times') for k in range(events)]
    j.row(tau[0],0,0,'charge_grid_start','battery_availability');j.row(tau[-1],j.H,j.H,'charge_grid_end','battery_availability')
    for k in range(K):j.row(tau[k+1]-tau[k],lo=0,name=f'charge_grid_order{k}',family='battery_availability')
    j.charge_events=tau;j.charge_power={};j.charge_interval_active={};j.charge_progress={}
    availability={}
    for v in range(len(vehicles)):
        visit_ids=[k for vv,k in visits if vv==v]
        for k in range(K):
            at=[]
            for visit in visit_ids:
                used=any(r['carrier']==v and r['start_s']<=seed_times[k]+1e-8 and r['end_s']>=seed_times[k+1]-1e-8 and seed_times[k+1]>seed_times[k] for r in seeds)
                candidates=[q for q in visit_ids if j.val(j.arr[v,q])<=seed_times[k]+1e-8 and j.val(j.dep[v,q])>=seed_times[k+1]-1e-8]
                bit=j.binary(f'charge_stopped_{v}_{k}_{visit}',used and bool(candidates) and visit==candidates[0])
                j.gate(tau[k]-j.arr[v,visit],bit,lo=0,name=f'charge_stop_begin_{v}_{k}_{visit}',family='battery_availability')
                j.gate(j.dep[v,visit]-tau[k+1],bit,lo=0,name=f'charge_stop_end_{v}_{k}_{visit}',family='battery_availability')
                at.append(bit)
            availability[v,k]=sum(at)
            j.row(sum(at),hi=1,name=f'charge_one_stop_{v}_{k}',family='battery_availability')
    fast=.7*capacity/1800/eta*3600;tail=.1*capacity/1200/eta*3600;knee=.9*capacity
    owned={}
    for l in range(j.n):
        def initial_energy(time):
            return j.val(j.eb[l])+eta*sum(r['input_power_W']*max(0.,min(time,r['end_s'])-r['start_s']) for r in seeds if r['next_flight']==l)/3600
        E=[j.var(f'charge_job_energy_{l}_{k}',energy['minimum_SOC']*capacity,capacity,initial_energy(seed_times[k]),group='battery_energy') for k in range(events)]
        j.charge_progress[l]=E
        j.row(E[0]-j.eb[l],0,0,f'charge_job_initial_{l}','battery_conservation')
        j.row(E[-1]-j.ea[l],0,0,f'charge_job_final_{l}','battery_conservation')
        j.gate(j.ce[l]-j.s[l],j.b[l],-cfg['recovery']['preparation_seconds'],-cfg['recovery']['preparation_seconds'],name=f'charge_job_deadline{l}',family='battery_availability')
        for k in range(K):
            initial_power=sum(r['input_power_W'] for r in seeds if r['next_flight']==l and r['start_s']<=seed_times[k]+1e-8 and r['end_s']>=seed_times[k+1]-1e-8 and seed_times[k+1]>seed_times[k])
            on=j.binary(f'charge_job_active_{l}_{k}',initial_power>0,'charging_channel');P=j.var(f'charge_job_power_{l}_{k}',0,fast,initial_power,group='charging_input_power')
            j.charge_interval_active[l,k]=on;j.charge_power[l,k]=P
            j.row(on-j.b[l],hi=0,name=f'charge_job_enabled_{l}_{k}',family='battery_availability')
            j.row(P-fast*on,hi=0,name=f'charge_job_power_on_{l}_{k}',family='battery_conservation')
            j.row(E[k+1]-E[k]-eta*P*(tau[k+1]-tau[k])/3600,0,0,f'charge_job_balance_{l}_{k}','battery_conservation')
            j.gate(tau[k]-j.cs[l],on,lo=0,name=f'charge_job_release_{l}_{k}',family='battery_availability')
            j.gate(j.ce[l]-tau[k+1],on,lo=0,name=f'charge_job_due_{l}_{k}',family='battery_availability')
            is_tail=j.binary(f'charge_job_tail_{l}_{k}',initial_energy(seed_times[k])>=knee)
            early=j.and2(on,1-is_tail,f'charge_job_fast_on_{l}_{k}')
            late=j.and2(on,is_tail,f'charge_job_tail_on_{l}_{k}')
            j.gate(E[k+1],early,hi=knee,name=f'charge_job_fast_knee_{l}_{k}',family='battery_conservation')
            j.gate(E[k],late,lo=knee,name=f'charge_job_tail_knee_{l}_{k}',family='battery_conservation')
            j.row(P-tail-(fast-tail)*(1-is_tail),hi=0,name=f'charge_job_rate_band_{l}_{k}',family='battery_conservation')
            for v in range(len(vehicles)):
                owner=sum(j.y[u,l] for u in range(2*v,2*v+2))
                selected=j.and2(on,owner,f'charge_job_carrier_{l}_{k}_{v}');owned[l,k,v]=selected
                j.row(selected-availability[v,k],hi=0,name=f'charge_job_stopped_{l}_{k}_{v}',family='battery_availability')
    for v in range(len(vehicles)):
        for k in range(K):
            j.row(sum(owned[l,k,v] for l in range(j.n)),hi=vehicles[v]['charger_channels'],name=f'charge_event_channels_{v}_{k}',family='battery_availability')
            j.row(sum(owned[l,k,v]*j.charge_power[l,k] for l in range(j.n)),hi=vehicles[v]['supply_max_W'],name=f'charge_event_supply_{v}_{k}',family='battery_conservation')
    j.charge_domain=dict(mode='preemptive_variable_events',events=events,intervals=K,
        counting='2 endpoints +2 per flight release/deadline +2 per vehicle visit +2 SOC-phase completions per flight',
        assumptions=['single charger per carrier','zero idle power','constant capacity and efficiency','charging while stopped','two constant power bands'],
        energy_state='per-next-flight charge-progress ledger; physical pack state reconstructed along assigned flights')


def extract_event_charging(j,value):
    if not hasattr(j,'charge_events'):return None
    intervals=[]
    for (l,k),on in j.charge_interval_active.items():
        a,b=value(j.charge_events[k]),value(j.charge_events[k+1]);p=value(j.charge_power[l,k])
        if value(on)>.5 and b-a>1e-8 and p>1e-8:
            u=next(u for u in range(4) if value(j.y[u,l])>.5)
            pack=next(p for p in range(8) if value(j.beta[p,l])>.5)
            intervals.append(dict(next_flight=l,event_index=k,carrier=u//2,pack=pack,start_s=a,end_s=b,input_power_W=p,
                initial_Wh=value(j.charge_progress[l][k]),final_Wh=value(j.charge_progress[l][k+1])))
    return dict(**j.charge_domain,intervals_exported=intervals)
