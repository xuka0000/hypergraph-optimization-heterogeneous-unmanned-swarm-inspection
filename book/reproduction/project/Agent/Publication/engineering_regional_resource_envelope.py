"""Necessary aggregate cuts implied by the physical resource chains.

No battery, weather, route, coverage target or task duration is changed. Indexed
identities already impose these at integer feasible solutions; exposing their
telescoping sums to the native relaxation prevents phantom regional resources.
"""
from pyscipopt import quicksum


def add_resource_envelope(j):
    p,r,m=j.path,j.resources,j.path.m;scene=j.scene;cfg=scene['joint_reference_profile']
    H=p.H;Q=r.capacity;eta=float(cfg['energy']['charge_efficiency'])
    # Clip and union retained allowed intervals: a loose capacity upper bound,
    # even when continuous wind/temperature further excludes parts of them.
    intervals=sorted((max(0.,float(x['start_s'])),min(H,float(x['end_s'])))
                     for x in j.forecast.intervals if x['flight_allowed'])
    windows=[]
    for a,b in intervals:
        if b<=a:continue
        if windows and a<=windows[-1][1]:windows[-1][1]=max(b,windows[-1][1])
        else:windows.append([a,b])
    allowed=sum(b-a for a,b in windows);receipts=[]
    for carrier,ids in r.by_carrier.items():
        vehicle=scene['fleet']['vehicles'][carrier]
        packs={b['id']:b for b in scene['fleet']['battery_pairs']}
        initial=sum(float(packs[k]['initial_Wh'])-r.reserve for k in vehicle['battery_pair_ids'])
        count=len(vehicle['uav_ids']);events=j.charging.by_carrier[carrier]
        credits=quicksum(r.flights[k].charged for k in ids)
        gains=quicksum(e['delta'] for e in events)
        charge_time=quicksum(e['end']-e['start'] for e in events)
        energy=quicksum(r.flights[k].energy for k in ids)
        airborne=quicksum(p.flights[k].end-p.flights[k].start for k in ids)
        used=quicksum(p.flights[k].active for k in ids)
        key=f'necessary_carrier{carrier}_'
        # Unique first/last charge events + reciprocal acyclic charge chains
        # imply exactly one telescoping credit per charged physical pack use.
        m.addCons(credits==gains,name=key+'charge_credit_conservation')
        m.addCons(energy<=initial+gains,name=key+'physical_pack_telescoping')
        m.addCons(airborne<=count*allowed,name=key+'aircraft_weather_capacity')
        m.addCons(airborne+(p.pre+p.post)*used<=count*p.C,name=key+'aircraft_shift_capacity')
        m.addCons(charge_time<=p.C,name=key+'single_charger_shift_capacity')
        if vehicle['charger_channels']!=1:
            raise ValueError('The current charging executor models one channel per carrier')
        # Existing fast/tail event constraints use input power, then eta once.
        stored_rate=min(eta*float(vehicle['supply_max_W']),.7*Q*2)/3600
        m.addCons(energy<=initial+stored_rate*p.C,name=key+'optimistic_energy_capacity')
        parks=[]
        if not cfg['vehicle']['charging_on_road']:
            for phase in j.vehicles.phases[carrier]:
                t=m.addVar(key+f'phase{phase.index}_parked_duration',lb=0,ub=H)
                m.addCons(t<=phase.duration)
                m.addCons(t<=H*phase.parked)
                m.addCons(t>=phase.duration-H*(1-phase.parked))
                parks.append(t)
            parked=quicksum(parks)
            m.addCons(charge_time<=parked,name=key+'charging_requires_parking_time')
            m.addCons(energy<=initial+stored_rate*parked,name=key+'road_charge_energy_capacity')
        receipts.append(dict(carrier=carrier,aircraft=count,initial_usable_Wh=initial,
            weather_airtime_upper_s=count*allowed,stored_fast_power_W=stored_rate*3600,
            all_day_optimistic_energy_upper_Wh=initial+stored_rate*H,
            parked_duration_variables=len(parks)))
    return dict(carriers=receipts,weather_windows=windows,
        original_feasible_decisions_removed=False,
        proof='telescoping unique pack/charge chains; nonoverlap of UAV occupations and charger intervals; charging interval containment in disjoint source-road parking phases',
        implication_scope='current one-channel-per-carrier reference model',
        full_joint_feasible=None)
