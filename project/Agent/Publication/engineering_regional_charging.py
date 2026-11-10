"""Preemptive one-channel charging on endogenous parked intervals.

The caller supplies intervals belonging to the same vehicle trajectory model,
not a solved/frozen itinerary. A finite event budget is explicitly part of the
transcription; this module never calls an arbitrary budget lossless. The model
can be rebuilt with more events without changing source data or resource IDs.
"""
from dataclasses import dataclass
from pyscipopt import quicksum
from Agent.Publication.engineering_indexed_constraints import AffineRow


@dataclass
class ParkedInterval:
    carrier: int
    start: object
    end: object
    active: object
    # Required evidence that the supplied interval is bound to the live road
    # trajectory. This is an object reference, not a claimed validation flag.
    road_phase: object


class RegionalCharging:
    def __init__(self, resources, parked, events_per_carrier):
        self.r=resources;self.p=resources.path;self.m=resources.m;self.rel=resources.rel
        self.parked=parked;self.budgets=events_per_carrier;self.events=[]
        if resources.charging_attached:raise ValueError('Charging ledger already attached')
        if len(events_per_carrier)!=len(self.p.slots_per_carrier):raise ValueError('One event budget per carrier')
        if any(j.road_phase is None for j in parked):raise ValueError('Unbound/frozen road intervals are not allowed')
        self.jobs={};self.by_carrier={}
        self.temperature_attached=False

    def build(self):
        m,rel,r,p=self.m,self.rel,self.r,self.p;Q=r.capacity;reserve=r.reserve
        efficiency=float(r.scene['joint_reference_profile']['energy']['charge_efficiency'])
        fast=.7*Q*2/efficiency;tail=.1*Q*3/efficiency
        knee=.9*Q
        for carrier,budget in enumerate(self.budgets):
            if budget<0:raise ValueError('Negative event budget')
            ids=r.by_carrier[carrier];parks=[j for j in self.parked if j.carrier==carrier]
            if budget and not parks:raise ValueError('Charging requires endogenous parking intervals')
            events=[]
            for k in range(budget):
                key=f'charge_v{carrier}_event{k+1}'
                a=r.var(key+'_active',0,1,True);job=rel.selector(key+'_next_flight',len(ids)+1)
                m.addCons(job.value>=a);m.addCons(job.value<=len(ids)*a)
                start=r.var(key+'_start',0,p.H);end=r.var(key+'_end',0,p.H)
                m.addCons(end>=start);m.addCons(start<=p.H*a);m.addCons(end<=p.H*a)
                m.addCons(end<=p.C)
                if events:
                    m.addCons(a<=events[-1]['active'])
                    r.gated([(1,start),(-1,events[-1]['end'])],a,key+'_one_channel',lo=0)
                slow=r.var(key+'_tail',0,1,True);m.addCons(slow<=a)
                before=r.var(key+'_before_Wh',reserve,Q);after=r.var(key+'_after_Wh',reserve,Q)
                delta=r.var(key+'_gain_Wh',0,Q-reserve)
                # Exact projection of constant event power: for dt>0,
                # P=3600*delta/(eta*dt). For dt=0 the energy is0 and P=0 is
                # an equivalent representative. Every partial power remains
                # available; no max-power assumption / bilinear relaxation.
                supply=float(r.scene['fleet']['vehicles'][carrier]['supply_max_W'])
                fast_rate=efficiency*min(fast,supply)/3600
                tail_rate=efficiency*min(tail,supply)/3600
                m.addCons(delta<=fast_rate*(end-start),name=key+'_fast_energy_duration')
                r.gated([(1,delta),(-tail_rate,end),(tail_rate,start)],slow,key+'_tail_energy_duration',hi=0)
                m.addCons(after==before+delta);m.addCons(delta<=(Q-reserve)*a)
                r.gated([(1,after)],a-slow,key+'_fast_knee',hi=knee)
                r.gated([(1,before)],slow,key+'_tail_knee',lo=knee)
                pred=rel.selector(key+'_previous_charge',budget+1)
                succ=rel.selector(key+'_next_charge',budget+1)
                first,last,hp,hs=[r.var(key+'_'+q,0,1,True) for q in ['first','last','has_pred','has_succ']]
                m.addCons(first+hp==a);m.addCons(last+hs==a)
                m.addCons(pred.value>=hp);m.addCons(pred.value<=k*hp)
                m.addCons(succ.value>=(k+2)*hs);m.addCons(succ.value<=budget*hs)
                site=rel.selector(key+'_parked_interval',len(parks))
                event=dict(carrier=carrier,index=k,active=a,job=job,start=start,end=end,
                    before=before,after=after,delta=delta,pred=pred,succ=succ,slow=slow,
                    first=first,last=last,hp=hp,hs=hs,site=site)
                events.append(event);self.events.append(event)
                def site_rows(j,event=event,parks=parks):
                    park=parks[j]
                    return [r.eq('parked_mode_active',[(1,park.active)],1),
                            AffineRow('after_park_start',[(1,event['start']),(-1,park.start)],lo=0),
                            AffineRow('before_park_end',[(1,park.end),(-1,event['end'])],lo=0)]
                rel.add(site,site_rows,[a,start,end,*[v for q in parks for v in [q.active,q.start,q.end]]],a,key+'_parking')
            self.by_carrier[carrier]=events
            for local,i in enumerate(ids,1):
                f=p.flights[i];res=r.flights[i]
                on=r.var(f'flight{i+1}_has_charging',0,1,True);m.addCons(on<=f.active)
                first=rel.selector(f'flight{i+1}_first_charge',budget+1)
                last=rel.selector(f'flight{i+1}_last_charge',budget+1)
                for sel in [first,last]:
                    m.addCons(sel.value>=on);m.addCons(sel.value<=budget*on)
                m.chgVarUb(res.charged,Q-reserve)
                m.addCons(res.charged<=(Q-reserve)*on)
                self.jobs[i]=dict(active=on,first=first,last=last)
                for role,sel in [('first',first),('last',last)]:
                    def endpoint(j,role=role,i=i,local=local,events=events):
                        if not j:return [AffineRow('nonzero_charge_event',[],lo=1)]
                        ev=events[j-1]
                        return [r.eq('event_active',[(1,ev['active'])],1),
                                r.eq('correct_flight',[(1,ev['job'].value)],local),
                                r.eq('endpoint_flag',[(1,ev[role])],1)]
                    rel.add(sel,endpoint,[],on,f'flight{i+1}_charge_{role}')
            for k,e in enumerate(events):
                def job_rows(local,e=e,ids=ids):
                    if not local:return [AffineRow('nonzero_charge_job',[],lo=1)]
                    i=ids[local-1];f=p.flights[i];res=r.flights[i];job=self.jobs[i]
                    return [r.eq('flight_active',[(1,f.active)],1),r.eq('job_active',[(1,job['active'])],1),
                            AffineRow('pack_available',[(1,e['start']),(-1,res.available_after)],lo=0),
                            AffineRow('before_preparation',[(1,f.start),(-1,e['end'])],lo=p.pre)]
                rel.add(e['job'],job_rows,[],e['active'],f'charge_{carrier}_{k}_window')
                def first_rows(local,e=e,k=k,ids=ids):
                    if not local:return [AffineRow('nonzero_first_job',[],lo=1)]
                    i=ids[local-1];res=r.flights[i]
                    return [r.eq('first_event_identity',[(1,self.jobs[i]['first'].value)],k+1),
                            r.eq('carry_pack_energy',[(1,e['before']),(-1,res.before_charge)])]
                rel.add(e['job'],first_rows,[],e['first'],f'charge_{carrier}_{k}_first')
                def last_rows(local,e=e,k=k,ids=ids):
                    if not local:return [AffineRow('nonzero_last_job',[],lo=1)]
                    i=ids[local-1];res=r.flights[i]
                    return [r.eq('last_event_identity',[(1,self.jobs[i]['last'].value)],k+1),
                            r.eq('launch_energy_after_charging',[(1,e['after']),(-1,res.launch_energy)])]
                rel.add(e['job'],last_rows,[],e['last'],f'charge_{carrier}_{k}_last')
                for role,flag,selector in [('pred','hp',e['pred']),('succ','hs',e['succ'])]:
                    def chain(j,role=role,e=e,k=k,events=events):
                        if not j or (role=='pred' and j>=k+1) or (role=='succ' and j<=k+1):
                            return [AffineRow('invalid_charge_chain_order',[],lo=1)]
                        q=events[j-1]
                        ans=[r.eq('other_event_active',[(1,q['active'])],1),
                             r.eq('same_next_flight',[(1,e['job'].value),(-1,q['job'].value)]),
                             r.eq('reciprocal',[(1,q['succ' if role=='pred' else 'pred'].value)],k+1)]
                        if role=='pred':ans.append(r.eq('energy_carry',[(1,e['before']),(-1,q['after'])]))
                        return ans
                    rel.add(selector,chain,[],e[flag],f'charge_{carrier}_{k}_{role}')
        rel.lock(m.getVars())
        r.charging_attached=True
        self.input_energy=quicksum(e['delta']/efficiency for e in self.events)
        return self

    def power_value(self,event,value):
        dt=value(event['end'])-value(event['start']);gain=value(event['delta'])
        if dt<=1e-12:
            if abs(gain)>1e-8:raise ArithmeticError('Positive charging energy in zero time')
            return 0.
        eta=float(self.r.scene['joint_reference_profile']['energy']['charge_efficiency'])
        return 3600*gain/(eta*dt)

    def status(self):
        return dict(events_per_carrier=self.budgets,total_events=len(self.events),
                    finite_transcription=True,unrestricted_event_count_equivalence_proved=False,
                    preemption_supported=True,charging_temperature_attached=self.temperature_attached,
                    independent_wh_ledger_required=True,
                    partial_power_preserved=True,power_representation='exact energy-duration projection with physical-power reconstruction')

    def attach_temperature(self,forecast):
        if self.temperature_attached:raise ValueError('Charging temperature already attached')
        # The reference has one spatial cell and clamps all ground heights to
        # the30m layer. Extremes of each time-linear interval occur at its
        # endpoints. Check event endpoints and every enclosed forecast knot.
        m=self.m;r=self.r;zero=m.addVar('charging_reference_height',lb=0,ub=0)
        for i,e in enumerate(self.events):
            for side in ['start','end']:
                data=forecast.add_point(m,self.rel,e[side],zero,e['active'],f'charging{i}_{side}',operation='charge')
                temp=data['temperature_C']
                m.addCons(e['active']*temp>=0);m.addCons(e['active']*(40-temp)>=0)
            for j,t in enumerate(forecast.times):
                temp=float(forecast.values[j,0,3])
                if not 0<=temp<=40:
                    before=r.var(f'charging{i}_outside_bad_knot{j}',0,1,True)
                    # The strict open violation neighborhoods are enforced by
                    # endpoint checks plus knot exclusion; no time cap added.
                    m.addCons(e['end']<=float(t)+r.H*(1-before+1-e['active']))
                    m.addCons(e['start']>=float(t)-r.H*(before+1-e['active']))
        self.rel.lock(m.getVars());self.temperature_attached=True
        return self
