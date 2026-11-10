"""Replay exported decisions without repairing, rescheduling or rebinding them."""
from run_engineering_full_joint import *
from verify_joint_optical_trajectory import evaluate
from Agent.Publication.engineering_road_certificate import certify as certify_road
from Agent.Publication.engineering_road_certificate import certify_stationary
from Agent.Publication.engineering_energy_enclosure import EnergyEnclosure
import math


def resources(result,values,flights,scene,options,terrain,weather):
    rows=[];cfg=scene['joint_reference_profile'];en=cfg['energy'];cap=en['pair_capacity_Wh']
    pre=cfg['recovery']['preparation_seconds'];post=cfg['recovery']['recovery_seconds'];reserve=cap*en['minimum_SOC'];tol=1e-5
    def check(name,ok,**detail):rows.append(dict(check=name,passed=bool(ok),**detail))
    def close(a,b):return abs(a-b)<=tol
    def overlap(a,b):return min(a[1],b[1])-max(a[0],b[0])>tol
    assignments=[];charging=[];handling=[];captured=[];vehicles=[];energy=0.;maxend=0.
    energy_oracle=EnergyEnclosure(scene,terrain);energy_lower=energy_upper=0.
    for v,routeid in enumerate(result['route_selection']):
        route=options[v][routeid];arr=[values[f'vehicle_{v}_arrival{k}'] for k in range(3)];dep=[values[f'vehicle_{v}_departure{k}'] for k in range(3)]
        check(f'vehicle {v} starts at t0',close(arr[0],0))
        check(f'vehicle {v} nonnegative dwell',all(b>=a-tol for a,b in zip(arr,dep)))
        check(f'vehicle {v} source depot',route['sites'][0]['stop_id']==scene['fleet']['vehicles'][v]['depot_stop'])
        check(f'vehicle {v} returns to source depot',route['sites'][2]['stop_id']==scene['fleet']['vehicles'][v]['depot_stop'])
        for k in [1,2]:
            scale=values[f'vehicle_speed_scale_{v}_{k}']
            check(f'vehicle {v} leg{k} clock',scale>=1-tol and close(arr[k]-dep[k-1],scale*route['leg_seconds'][k-1]))
        certificate=certify_road(scene,route['route'],[values[f'vehicle_speed_scale_{v}_{k}'] for k in [1,2]])
        if route['route'] is None:certificate=certify_stationary(scene,v,route['sites'][0])
        check(f'vehicle {v} continuous road certificate',certificate['passed'],certificate=certificate)
        maxend=max(maxend,arr[2]);vehicles.append(dict(route=route,arrival=arr,departure=dep))
    event_plan=result.get('charging_plan');event_stored={}
    event_variables=any(key.startswith('charge_job_power_') for key in values)
    check('charging representation exported',event_variables==(event_plan is not None))
    if event_plan is not None:
        expected=[]
        for key,value in values.items():
            if not key.startswith('charge_job_power_'):continue
            l,k=map(int,key.removeprefix('charge_job_power_').split('_'))
            if value>1e-8 and values[f'charge_event_time_{k+1}']-values[f'charge_event_time_{k}']>1e-8:expected.append((l,k))
        check('all positive charging intervals exported',sorted(expected)==sorted((r['next_flight'],r['event_index']) for r in event_plan['intervals_exported']))
        for k,row in enumerate(event_plan['intervals_exported']):
            l,v,p=row['next_flight'],row['carrier'],row['pack'];a,b,P=row['start_s'],row['end_s'],row['input_power_W']
            event=row['event_index']
            check(f'event charge{k} decision binding',close(a,values[f'charge_event_time_{event}']) and close(b,values[f'charge_event_time_{event+1}']) and close(P,values[f'charge_job_power_{l}_{event}']) and values[f'charge_job_active_{l}_{event}']>.5)
            stored=en['charge_efficiency']*P*(b-a)/3600
            event_stored[l]=event_stored.get(l,0.)+stored
            check(f'event charge{k} owner',p//4==v and close(values[f'beta_{p}_{l}'],1) and any(values[f'y_{u}_{l}']>.5 for u in range(2*v,2*v+2)))
            check(f'event charge{k} interval and power',a>=-tol and b>a and P>0)
            check(f'event charge{k} conservation',close(row['final_Wh']-row['initial_Wh'],stored))
            fast=.7*cap/1800/en['charge_efficiency']*3600;tail=.1*cap/1200/en['charge_efficiency']*3600
            rate_ok=(row['final_Wh']<=.9*cap+tol and P<=fast+tol) or (row['initial_Wh']>=.9*cap-tol and P<=tail+tol)
            check(f'event charge{k} source SOC band',rate_ok and reserve-tol<=row['initial_Wh']<=row['final_Wh']<=cap+tol)
            check(f'event charge{k} job release/deadline',a>=values[f'charge_start_{l}']-tol and b<=values[f'launch_{l}']-pre+tol)
            stopped=any(a>=aa-tol and b<=dd+tol for aa,dd in zip(vehicles[v]['arrival'],vehicles[v]['departure']))
            check(f'event charge{k} stationary carrier',stopped)
            charging.append(dict(vehicle=v,flight=l,pack=p,interval=[a,b],power_W=P))
    for d in flights:
        l=d['root'];s=d['source_sortie'];v=s['vehicle'];u=s['aircraft'];p=s['battery_pair'];start=d['optimized_launch_s'];land=start+d['flight_seconds']
        check(f'flight {l} roots/ownership',close(values[f'b_{l}'],1) and u//2==v and p//4==v)
        check(f'flight {l} aircraft and pack decision',close(values[f'y_{u}_{l}'],1) and close(values[f'beta_{p}_{l}'],1))
        check(f'flight {l} common event clock',close(start,values[f'launch_{l}']) and close(land,values[f'land_{l}']))
        check(f'flight {l} within source planning horizon',start>=-tol and land<=cfg['planning']['horizon_seconds']+tol)
        for kind,time0,time1,seg,at in [('launch',start-pre,start,d['timeline'][0],0.),('return',land,land+post,d['timeline'][-1],1.)]:
            selected=[(vv,k) for vv in range(2) for k in range(3) if values[f'{kind}_visit_{l}_{vv}_{k}']>.5]
            check(f'flight {l} {kind} exactly one owned visit',len(selected)==1 and selected[0][0]==v)
            if len(selected)!=1:continue
            vv,k=selected[0];visit=vehicles[vv];site=visit['route']['sites'][k]
            check(f'flight {l} {kind} stopped throughout handling',time0>=visit['arrival'][k]-tol and time1<=visit['departure'][k]+tol)
            state=eval_septic(seg['control_points_m'],seg['end_s']-seg['start_s'],at);pose=eval_septic(seg['pose_controls_yaw_gp_gy_focal_ratio'],seg['end_s']-seg['start_s'],at)
            anchor=np.asarray(site['xyz_m'])+np.asarray(cfg['vehicle']['pad_offset_m'])
            check(f'flight {l} {kind} pad match',np.linalg.norm(state[0]-anchor)<=tol)
            check(f'flight {l} {kind} at rest',max(np.linalg.norm(x) for x in state[1:])<=tol)
            angle_error=math.atan2(math.sin(pose[0][0]-site['yaw_rad']),math.cos(pose[0][0]-site['yaw_rad']))
            check(f'flight {l} {kind} pose match',abs(angle_error)<=tol and np.linalg.norm(pose[0][1:3])<=tol)
            check(f'flight {l} {kind} pose at rest',max(np.linalg.norm(x) for x in pose[1:3])<=tol)
            handling.append(dict(vehicle=v,flight=l,interval=[time0,time1]))
        exact=energy_of(copy.deepcopy(d['timeline']),cfg,terrain,weather,start);energy+=exact
        enclosed=energy_oracle.integrate(d['timeline'],start,cfg['numerical']['energy_tolerance_Wh']/max(1,len(flights)))
        energy_lower+=enclosed['lower_Wh'];energy_upper+=enclosed['upper_Wh']
        check(f'flight {l} energy integral enclosed',enclosed['converged'],enclosure=enclosed)
        before=values[f'pack_before_{l}'];after_charge=values[f'pack_after_charge_{l}'];after=values[f'pack_after_flight_{l}']
        cs=values[f'charge_start_{l}'];ce=values[f'charge_end_{l}'];power=values[f'charge_power_{l}']
        knee=values.get(f'charge_knee_time_{l}',ce);tailpower=values.get(f'charge_tail_power_{l}',0.)
        stored=en['charge_efficiency']*(power*(knee-cs)+tailpower*(ce-knee))/3600
        if event_plan is not None:
            stored=event_stored.get(l,0.)
            timeline=sorted([r for r in event_plan['intervals_exported'] if r['next_flight']==l],key=lambda r:r['start_s'])
            current=before
            for k,row in enumerate(timeline):
                check(f'flight {l} event charge energy chain{k}',close(row['initial_Wh'],current))
                current=row['final_Wh']
            check(f'flight {l} event charge terminal energy',close(current,after_charge))
        check(f'flight {l} numerical energy',max(abs(enclosed['lower_Wh']-d['aircraft_energy_Wh']),abs(enclosed['upper_Wh']-d['aircraft_energy_Wh']))<=.01,energy_24_Wh=exact,energy_model_Wh=d['aircraft_energy_Wh'])
        check(f'flight {l} charging conservation',close(after_charge-before,stored) and ce>=cs-tol and power>=-tol and tailpower>=-tol)
        check(f'flight {l} model pack conservation',close(after_charge-d['aircraft_energy_Wh'],after))
        # A quadrature tolerance must not conceal a physically insufficient reserve.
        check(f'flight {l} enclosed reserve',after_charge-enclosed['upper_Wh']>=reserve-1e-6 and after_charge<=cap+tol,terminal_energy_lower_Wh=after_charge-enclosed['upper_Wh'])
        check(f'flight {l} charge then preparation',ce<=start-pre+tol)
        if event_plan is None and max(power,tailpower)>tol and ce-cs>tol:
            fast=.7*cap/1800/en['charge_efficiency']*3600;tail=.1*cap/1200/en['charge_efficiency']*3600
            if f'charge_knee_time_{l}' in values:
                middle=before+en['charge_efficiency']*power*(knee-cs)/3600
                check(f'flight {l} two charge phases ordered',cs-tol<=knee<=ce+tol)
                check(f'flight {l} charge knee conservation',close(middle,values[f'pack_at_charge_knee_{l}']))
                check(f'flight {l} fast phase source bound',power<=fast+tol and (power<=tol or middle<=.9*cap+tol))
                check(f'flight {l} tail phase source bound',tailpower<=tail+tol and (tailpower<=tol or middle>=.9*cap-tol))
            else:check(f'flight {l} source charge rate',power<=(tail if after_charge>.9*cap+tol else fast)+tol)
            selected=[k for k in range(3) if values[f'launch_visit_{l}_{v}_{k}']>.5]
            check(f'flight {l} charging while stopped',len(selected)==1 and cs>=vehicles[v]['arrival'][selected[0]]-tol and ce<=vehicles[v]['departure'][selected[0]]+tol)
            charging.append(dict(vehicle=v,flight=l,pack=p,interval=[cs,ce],power_W=max(power,tailpower)))
        assignments.append(dict(flight=l,vehicle=v,aircraft=u,pack=p,launch=start,land=land,charge_start=cs,before=before,stored_Wh=stored,flight_energy_24_Wh=exact,flight_energy_lower_Wh=enclosed['lower_Wh'],flight_energy_upper_Wh=enclosed['upper_Wh'],after_model=after,after_actual=after_charge-exact,initial_SOC=d['initial_SOC']))
        sequence=[]
        for c in d['captures']:
            i=c['task_index'];sequence.append(i);captured.append(i);task=d['targets'][i]
            check(f'capture {i} assignment',close(values[f'x_{i}_{l}'],1) and close(values[f'a_{i}'],1))
            check(f'capture {i} clock and dwell',close(values[f't_capture_{i}'],start+c['start_s']) and c['end_s']-c['start_s']>=task['observation_requirement']['min_dwell_s']-tol)
        check(f'flight {l} chosen sequence',sequence==d['task_indices'] and sequence[0]==l and close(values[f'last_{sequence[-1]}'],1))
        for a,b in zip(sequence,sequence[1:]):check(f'flight {l} successor {a} {b}',close(values[f'h_{a}_{b}'],1))
        maxend=max(maxend,land+post)
    planned=[i for i in range(9) if values[f'a_{i}']>.5]
    check('task coverage matches decisions',sorted(captured)==planned and len(captured)==len(set(captured)))
    check('whole component matches all nine requirements',bool(result['complete_component']>.5)==(len(captured)==9))
    check('active flights match exports',sorted(d['root'] for d in flights)==[i for i in range(9) if values[f'b_{i}']>.5])
    check('makespan covers fleet return and all recovery',result['makespan_s']>=maxend-tol)
    for i,a in enumerate(assignments):
        for b in assignments[i+1:]:
            if a['aircraft']==b['aircraft']:check(f'aircraft occupancy {a["flight"]} {b["flight"]}',not overlap([a['launch']-pre,a['land']+post],[b['launch']-pre,b['land']+post]))
    for name,events in [('handling',handling),('charger',charging)]:
        for i,a in enumerate(events):
            for b in events[i+1:]:
                if a['vehicle']==b['vehicle']:check(f'{name} occupancy {a["flight"]} {b["flight"]}',not overlap(a['interval'],b['interval']))
    for c in charging:check(f'charger supply {c["flight"]}',c['power_W']<=scene['fleet']['vehicles'][c['vehicle']]['supply_max_W']+tol)
    for p in range(8):
        chain=sorted([a for a in assignments if a['pack']==p],key=lambda a:a['launch'])
        if not chain:continue
        expected=chain[0]['initial_SOC']*cap;expected_lower=expected_upper=expected;available=0
        for a in chain:
            check(f'pack {p} available for flight {a["flight"]}',a['charge_start']>=available-tol)
            check(f'pack {p} carried energy flight {a["flight"]}',abs(a['before']-expected)<=.01)
            if event_plan is not None:
                for charge in sorted([r for r in event_plan['intervals_exported'] if r['next_flight']==a['flight']],key=lambda r:r['start_s']):
                    added=en['charge_efficiency']*charge['input_power_W']*(charge['end_s']-charge['start_s'])/3600
                    fast=.7*cap/1800/en['charge_efficiency']*3600;tail=.1*cap/1200/en['charge_efficiency']*3600
                    check(f'pack {p} enclosed physical charge band {a["flight"]} {charge["event_index"]}',
                          ((expected_upper+added<=.9*cap+tol and charge['input_power_W']<=fast+tol) or
                           (expected_lower>=.9*cap-tol and charge['input_power_W']<=tail+tol)) and expected_upper+added<=cap+tol)
                    expected_lower+=added;expected_upper+=added
            else:
                expected_lower+=a['stored_Wh'];expected_upper+=a['stored_Wh']
            expected+=a['stored_Wh']-a['flight_energy_24_Wh'];available=a['land']+post
            expected_lower-=a['flight_energy_upper_Wh'];expected_upper-=a['flight_energy_lower_Wh']
            check(f'pack {p} cumulative physical reserve flight {a["flight"]}',expected_lower>=reserve-1e-6 and expected_upper<=cap+tol,energy_lower_Wh=expected_lower,energy_upper_Wh=expected_upper)
    return dict(all_pass=all(r['passed'] for r in rows),checks=rows,check_count=len(rows),aircraft_energy_24_Wh=energy,aircraft_energy_enclosure_Wh=[energy_lower,energy_upper],minimum_actual_makespan_s=maxend,assignments=assignments,charging=charging,scope='exported resource decisions with enclosed integrals, reference charging model, source-stationary road certificate; no decisions repaired')


def verify(tag='unified_SOC1',skip_physical=False,raise_on_failure=True):
    r=read(OUT/(tag+'_result.json'));s=read(OUT/(tag+'_status.json'));assert r['attempt']==s['attempt'] and s['state']=='candidate_ready','stale or live experiment'
    values=read(OUT/(tag+'_solution_values.json'));flights=[read(OUT/f) for f in r['sortie_files']];scene,_,terrain,weather=context();options=read(OUT/'unified_road_options.json')['options']
    started=time.perf_counter();receipt=resources(r,values,flights,scene,options,terrain,weather)
    receipt.update(observed_at=stamp(),seconds=time.perf_counter()-started,attempt=r['attempt'],result_sha256=sha(OUT/(tag+'_result.json')),values_sha256=sha(OUT/(tag+'_solution_values.json')),evaluator_sha256=sha(Path(__file__)),source_hashes={f:sha(OUT/f) for f in r['sortie_files']})
    save(OUT/(tag+'_resource_validation.json'),receipt)
    physical=None if skip_physical else evaluate(tag,r['sortie_files'])
    from Agent.Publication.engineering_joint_safety import witnesses_from_replay
    feedback=witnesses_from_replay(flights,physical) if physical is not None else {'witnesses':[],'unmapped_failures':[{'reason':'physical_check_skipped'}]}
    # Failure cannot be promoted by a successful check of another subsystem.
    combined=dict(observed_at=stamp(),attempt=r['attempt'],resource_pass=receipt['all_pass'],physical_pass=None if physical is None else physical['all_pass'],candidate_accepted=receipt['all_pass'] and physical is not None and physical['all_pass'],complete_joint_contract=False,global_gap=None,resource_receipt_sha256=sha(OUT/(tag+'_resource_validation.json')))
    combined.update(feedback)
    save(OUT/(tag+'_acceptance.json'),combined);print(json.dumps(combined))
    if raise_on_failure and not combined['candidate_accepted']:raise SystemExit(1)
    return combined


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--tag',default='unified_SOC1');ap.add_argument('--skip-physical',action='store_true');args=ap.parse_args()
    verify(args.tag,args.skip_physical)


if __name__=='__main__':main()
