"""Causal mission-state contract for the unified rolling inspection model.

This module prepares the actual joint problem's inputs. It is not a solver,
trajectory repair, state simulator, or proof of large-scale feasibility.
All timestamps below are seconds from the same fixed mission epoch.
"""
from __future__ import annotations
import copy
import datetime
import hashlib
import json
import math
from Agent.Publication.engineering_service_boundary import service_boundary


def fingerprint(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,ensure_ascii=False,
                                    allow_nan=False,separators=(',',':')).encode()).hexdigest()


def finite(value,name):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError('finite numeric value required: '+name)
    return float(value)


def vector(value,size,name):
    if not isinstance(value,(list,tuple)) or len(value)!=size:raise ValueError('vector shape: '+name)
    return [finite(v,name) for v in value]


def validate_snapshot(scene,state):
    now=finite(state['time_s'],'time_s')
    if now<0:raise ValueError('negative mission clock')
    if type(state.get('revision')) is not int or state['revision']<0:raise ValueError('invalid state revision')
    epoch=datetime.datetime.fromisoformat(state.get('epoch_utc','').replace('Z','+00:00'))
    if epoch.utcoffset()!=datetime.timedelta(0):raise ValueError('mission epoch must be explicit UTC')
    if state.get('scene_id')!=scene['scene_id']:raise ValueError('different scene')
    for group in ['vehicles','aircraft','battery_pairs']:
        expected={row['id'] for row in scene['fleet'][group]}
        if set(state[group])!=expected:raise ValueError('incomplete fleet state: '+group)
    tasks={t['id'] for t in scene['inspection_tasks']}
    if set(state['tasks'])!=tasks:raise ValueError('missing or extra task states')
    for name,row in state['tasks'].items():
        if row['status'] not in ['pending','in_progress','awaiting_acceptance','accepted','failed']:raise ValueError('unknown task status')
        if row['status']=='accepted':
            if row.get('image_quality_accepted') is not True or not row.get('evidence_id'):
                raise ValueError('planned capture is not executed image acceptance: '+name)
            if not 0<=finite(row['accepted_at_s'],'accepted_at_s')<=now:raise ValueError('image evidence outside executed history')
    carriers={x['id']:x['carrier_id'] for x in scene['fleet']['aircraft']}
    packs={x['id']:x for x in scene['fleet']['battery_pairs']};used=set()
    for name,row in state['vehicles'].items():
        vector(row['position_m'],3,'vehicle.position_m')
        for field in ['yaw_rad','speed_mps','steering_rad']:finite(row[field],'vehicle.'+field)
        if row.get('observed_at_s')!=now:raise ValueError('vehicle state is not synchronized')
    for name,row in state['aircraft'].items():
        if row['mode'] not in ['stowed','handling','airborne','recovering']:raise ValueError('aircraft mode')
        vector(row['position_m'],3,'aircraft.position_m');vector(row['velocity_mps'],3,'aircraft.velocity_mps')
        vector(row['acceleration_mps2'],3,'aircraft.acceleration_mps2')
        vector(row['body_quaternion_wxyz'],4,'aircraft.body_quaternion_wxyz')
        if abs(sum(q*q for q in row['body_quaternion_wxyz'])-1)>1e-5:raise ValueError('nonunit attitude quaternion')
        vector(row['gimbal_rad'],2,'aircraft.gimbal_rad');finite(row['yaw_rad'],'aircraft.yaw_rad')
        if row.get('observed_at_s')!=now:raise ValueError('aircraft state is not synchronized')
        if state.get('physical_state_schema')=='rigid_ecm_reference':
            vector(row['body_rate_radps'],3,'aircraft.body_rate_radps')
            thrust=vector(row['rotor_thrust_N'],4,'aircraft.rotor_thrust_N')
            if min(thrust)<0:raise ValueError('Negative rotor thrust')
        if row['mode'] in ['airborne','recovering']:
            p=row.get('battery_pair_id')
            if p not in packs or p in used or not row.get('sortie_id'):raise ValueError('invalid in-flight battery/sortie')
            if packs[p]['carrier_id']!=carriers[name]:raise ValueError('carrier transfer not authorized')
            battery=state['battery_pairs'][p]
            if battery['mode']!='in_flight' or battery.get('aircraft_id')!=name:raise ValueError('battery/aircraft ownership mismatch')
            used.add(p)
    for name,row in state['battery_pairs'].items():
        energy=finite(row['energy_Wh'],'battery.energy_Wh')
        # Low-energy measurements remain valid state inputs; an emergency
        # response may be necessary. Do not reject truth to obtain feasibility.
        if not 0<=energy<=packs[name]['capacity_Wh']:raise ValueError('energy outside physical capacity')
        if row['mode'] not in ['available','charging','cooling','in_flight','handling']:raise ValueError('battery mode')
        if row['mode']=='in_flight' and name not in used:raise ValueError('unbound in-flight battery')
        finite(row['available_after_s'],'battery.available_after_s')
        if row.get('observed_at_s')!=now:raise ValueError('battery state is not synchronized')
        if state.get('physical_state_schema')=='rigid_ecm_reference':
            # energy_Wh is the legacy nameplate-scale diagnostic only; it
            # must NOT be used to reconstruct SOC, polarization or heat.
            soc=finite(row['SOC'],'battery.SOC')
            if not 0<=soc<=1:raise ValueError('SOC outside physical domain')
            if type(row.get('charge_hot_blocked')) is not bool:raise ValueError('Missing charging hysteresis memory')
            for key in ['polarization_V','temperature_C','grid_input_Wh','load_output_Wh','resistive_heat_Wh']:
                val=finite(row[key],'battery.'+key)
                if key.endswith('_Wh') and val<0:raise ValueError('Negative cumulative energy')
    action_ids=set()
    for action in state.get('committed_actions',[]):
        start=finite(action['start_s'],'action.start_s');end=finite(action['end_s'],'action.end_s')
        if start<0 or end<start or not action.get('action_id'):raise ValueError('invalid committed action')
        if action['action_id'] in action_ids:raise ValueError('duplicate committed action')
        action_ids.add(action['action_id'])
    if state.get('service_attempts') is not None:
        boundary=service_boundary(tasks,state['service_attempts'],now)
        for task,row in state['tasks'].items():
            if (row['status']=='in_progress') != (task in boundary['ongoing']):raise ValueError('Ongoing service record/status mismatch')
            if row['status']=='awaiting_acceptance' and task not in {a['task_id'] for a in boundary['pending_acceptance']}:
                raise ValueError('Pending acceptance record missing')
    elif any(row['status'] in ['in_progress','awaiting_acceptance'] for row in state['tasks'].values()):
        raise ValueError('Service attempt history required at mid-service/acceptance boundary')
    return now


def validate_candidate_takeover(candidate,snapshot,now_s,position_tolerance_m=.05,
                                velocity_tolerance_mps=.05,physical_tolerance=1e-6):
    """Reject a stale candidate instead of resetting to its old initial state.

    The caller must still run full geometric/physical feasibility validation.
    This check covers actual takeover state and commitments, not future safety.
    """
    now=finite(now_s,'takeover time')
    if candidate.get('state_revision')!=snapshot.get('revision'):
        raise ValueError('Candidate uses a stale state revision')
    if candidate.get('information_time_s')>now or candidate.get('ready_at_s',float('inf'))>now:
        raise ValueError('Candidate not yet available at takeover')
    initial=candidate['takeover_state']
    if initial['time_s']!=now or snapshot['time_s']!=now:raise ValueError('Unsynchronized takeover')
    for group in ['vehicles','aircraft','battery_pairs']:
        if set(initial[group])!=set(snapshot[group]):raise ValueError('Takeover identity mismatch')
    for group in ['vehicles','aircraft']:
        for key,true in snapshot[group].items():
            pred=initial[group][key]
            if math.dist(pred['position_m'],true['position_m'])>position_tolerance_m:raise ValueError('Stale takeover position')
            if group=='vehicles':
                for field in ['yaw_rad','speed_mps','steering_rad']:
                    if field in true and abs(pred[field]-true[field])>physical_tolerance:raise ValueError('Stale vehicle state: '+field)
            if group=='aircraft':
                if math.dist(pred['velocity_mps'],true['velocity_mps'])>velocity_tolerance_mps:raise ValueError('Stale takeover velocity')
                q=pred['body_quaternion_wxyz'];r=true['body_quaternion_wxyz']
                if min(math.dist(q,r),math.dist(q,[-v for v in r]))>physical_tolerance:raise ValueError('Stale takeover attitude')
                for field in ['body_rate_radps','rotor_thrust_N']:
                    if field in true and math.dist(pred[field],true[field])>physical_tolerance:raise ValueError('Stale takeover actuator state')
                for field in ['battery_pair_id','sortie_id','mode']:
                    if pred.get(field)!=true.get(field):raise ValueError('Changed ongoing aircraft identity')
    for key,true in snapshot['battery_pairs'].items():
        pred=initial['battery_pairs'][key]
        for field in ['SOC','polarization_V','temperature_C','grid_input_Wh','load_output_Wh','resistive_heat_Wh']:
            if field in true and abs(pred[field]-true[field])>physical_tolerance:raise ValueError('Reset battery state: '+field)
        for field in ['mode','aircraft_id']:
            if pred.get(field)!=true.get(field):raise ValueError('Changed battery ownership')
        if pred.get('charge_hot_blocked')!=true.get('charge_hot_blocked'):raise ValueError('Reset charging hysteresis')
    actual=[a for a in snapshot.get('committed_actions',[]) if a['end_s']>now]
    if fingerprint(candidate.get('committed_actions',[]))!=fingerprint(actual):raise ValueError('Changed issued commitment')
    if fingerprint(initial.get('service_attempts',[]))!=fingerprint(snapshot.get('service_attempts',[])):
        raise ValueError('Changed service/acceptance history at takeover')
    return dict(state_compatible=True,full_future_feasibility_verified=False)


def available_forecast(packets,now,end):
    qualified=[]
    for packet in packets:
        # A historical weather realization must never be relabelled as a
        # forecast even when its valid time is in the future of this replay.
        if packet.get('data_role')!='forecast':continue
        issue=finite(packet['issued_at_s'],'forecast.issued_at_s')
        received=finite(packet['available_at_s'],'forecast.available_at_s')
        if received<issue:raise ValueError('forecast received before issued')
        start=finite(packet['valid_from_s'],'forecast.valid_from_s')
        stop=finite(packet['valid_until_s'],'forecast.valid_until_s')
        if stop<=start:raise ValueError('empty forecast validity')
        if issue<=now and received<=now and start<=now and stop>=end:
            if not packet.get('source_id') or not packet.get('scenarios'):raise ValueError('forecast provenance/scenarios absent')
            ids=[s['id'] for s in packet['scenarios']]
            if len(set(ids))!=len(ids):raise ValueError('duplicate weather scenarios')
            qualified.append(packet)
    if not qualified:raise ValueError('no causally available forecast covering the horizon; reanalysis is not online information')
    return copy.deepcopy(max(qualified,key=lambda p:(p['issued_at_s'],p['available_at_s'])))


def prepare_joint_replanning(scene,state,forecast_packets,mission_end_s):
    now=validate_snapshot(scene,state);end=finite(mission_end_s,'mission_end_s')
    if end<=now:raise ValueError('mission horizon is finished')
    forecast=available_forecast(forecast_packets,now,end)
    completed=sorted(k for k,v in state['tasks'].items() if v['status']=='accepted')
    remaining=sorted(set(state['tasks'])-set(completed))
    ongoing={u:dict(sortie_id=s['sortie_id'],battery_pair_id=s['battery_pair_id'])
             for u,s in state['aircraft'].items() if s['mode'] in ['airborne','recovering']}
    commitments=[copy.deepcopy(a) for a in state.get('committed_actions',[]) if a['end_s']>now]
    service=service_boundary(set(state['tasks']),state.get('service_attempts',[]),now)
    # Earlier accepted tasks may have their evidence in the task record rather
    # than a complete attempt archive. They still cannot become new services.
    service['new_service_eligible_task_ids']=sorted(set(service['new_service_eligible_task_ids'])-set(completed))
    return dict(schema_version=1,scene_id=scene['scene_id'],epoch_utc=state['epoch_utc'],
        information_time_s=now,mission_end_s=end,state_revision=state['revision'],
        state_sha256=fingerprint(state),scene_sha256=fingerprint(scene),
        fingerprint_encoding='canonical sorted JSON, SHA256; not raw-file byte hash',
        remaining_task_ids=remaining,completed_task_ids=completed,
        initial_state=copy.deepcopy(state),ongoing_sorties=ongoing,
        service_boundary=service,
        committed_actions=commitments,forecast=forecast,
        full_road_domain={'locator':'/roads','sha256':fingerprint(scene['roads'])},
        full_stop_domain={'locator':'/stops','count':len(scene['stops']),'sha256':fingerprint(scene['stops'])},
        uncommitted_choices='joint task assignment, all-source road/stop choices, flight/pose/time and charge decisions',
        solver_implemented=False,complete_joint_optimization_verified=False,
        scope='causal problem input contract only; no future observation, state reset or route catalogue substitution')


def replan_reasons(state,last_information_time_s,period_s,events):
    now=finite(state['time_s'],'time_s');period=finite(period_s,'period_s')
    if period<=0:raise ValueError('positive review period required')
    if last_information_time_s is not None:
        last_information_time_s=finite(last_information_time_s,'last_information_time_s')
        if not 0<=last_information_time_s<=now:raise ValueError('invalid previous information time')
    reasons=[]
    if last_information_time_s is None or now-last_information_time_s>=period:reasons.append('periodic')
    for event in events:
        received=finite(event['available_at_s'],'event.available_at_s')
        if received>now or (last_information_time_s is not None and received<=last_information_time_s):continue
        if event['type'] in {'forecast_update','energy_deviation','road_closure','task_failure','resource_failure','safety_alarm'}:
            reasons.append(event['type'])
    return sorted(set(reasons))


def contract_extension():
    return dict(mode='receding_horizon_joint_mixed_discrete_continuous_optimization',
        information='only observed/received data available at each information time',
        state='measured_or_estimated_vehicles_aircraft_batteries_tasks_and_commitments',
        dynamics='same full joint physical model; continuous trajectories and durations remain variable',
        replanning='periodic plus causal events; periods and thresholds are declared experimental parameters, not measured facts',
        horizon='all remaining work through fixed mission end; shortened horizon requires an explicit terminal continuation model',
        uncertainty='weather and power-residual scenarios with nonanticipativity; actual future weather and energy only in execution evaluator',
        risk_aggregation='componentwise worst-case lexicographic objectives over the declared finite scenario set; not a probability guarantee',
        commitments='executed actions and issued prefix fixed; ongoing aircraft/pack identities retained until safe recovery',
        takeover='candidate must be replayed against actual takeover-time state; stale or unsafe candidate is not issued',
        objectives=['weighted_incomplete_components','fleet_return_makespan_seconds','aircraft_energy_Wh'],
        large_scale='all configured tasks and full source road/stop domains; no9-task restriction',
        mathematical_definition=True,input_contract_implemented=True,
        full_joint_solver_implemented=False,closed_loop_large_scale_verified=False,
        forecast_issue_archive_available=False,measured_M300_calibration=False,
        source='https://www.do-mpc.com/en/latest/theory_mpc.html',
        source_role='receding horizon and nonanticipativity definitions; do-mpc is not claimed as the inspection solver')
