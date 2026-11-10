"""Event-time carryover for ongoing photography and delayed acceptance.

This forms boundary data for a joint model. It does not optimize a mission.
Pending image outcomes confer no guaranteed completion credit until received;
a new repeat, if scheduled, has a distinct attempt identity.
"""
from __future__ import annotations
import copy, math


def _time(x):
    if isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x):
        raise ValueError('Finite event time required')
    return float(x)


def service_boundary(task_ids, attempts, now_s):
    now=_time(now_s)
    if now<0:raise ValueError('Negative mission time')
    task_ids=set(task_ids); seen=set(); active={}; accepted={}; pending=[]; failures=[]; history=[]
    by_aircraft={}
    for raw in attempts:
        a=copy.deepcopy(raw); key=a['attempt_id'];task=a['task_id']
        if not key or key in seen or task not in task_ids:raise ValueError('Attempt identity')
        seen.add(key)
        begin=_time(a['start_s']);end=_time(a['end_s'])
        if not 0<=begin<end or begin>now:raise ValueError('History contains an unstarted attempt')
        if not all(a.get(k) for k in ['aircraft_id','battery_pair_id','sortie_id']):raise ValueError('Missing service identity')
        exposure=a['exposure_intervals_s']
        if not exposure:raise ValueError('Missing committed exposure')
        for x,y in exposure:
            if not begin<=_time(x)<_time(y)<=end:raise ValueError('Exposure outside service')
        if not a.get('prefix_evidence_id'):raise ValueError('Missing executed-prefix evidence')
        by_aircraft.setdefault(a['aircraft_id'],[]).append((begin,end,key))
        received=a.get('acceptance_received_at_s')
        if received is not None:
            received=_time(received);event=_time(a['acceptance_event_at_s'])
            if event<end or received<event:raise ValueError('Acceptance before completion/event')
            if type(a.get('quality_passed')) is not bool or not a.get('image_evidence_id'):
                raise ValueError('Acceptance needs image evidence and explicit result')
        # The boundary output never exposes a result that has not arrived.
        public={k:v for k,v in a.items() if k not in ['quality_passed','image_evidence_id','acceptance_event_at_s','acceptance_received_at_s']}
        if begin<=now<end:
            if task in active:raise ValueError('Duplicate ongoing service for one task')
            past=[];future=[]
            for x,y in exposure:
                if x<now:past.append([x,min(y,now)])
                if y>now:future.append([max(x,now),y])
            active[task]=dict(public,remaining_s=end-now,original_start_s=begin,
                boundary_start_s=now,executed_exposure_intervals_s=past,
                remaining_exposure_intervals_s=future,requires_prefix_evidence=True)
        elif received is None or received>now:
            pending.append(public)
        elif a['quality_passed']:
            accepted.setdefault(task,[]).append(dict(attempt_id=key,received_at_s=received,image_evidence_id=a['image_evidence_id']))
        else:
            failures.append(dict(attempt_id=key,task_id=task,received_at_s=received,image_evidence_id=a['image_evidence_id']))
        history.append(public)
    for rows in by_aircraft.values():
        rows.sort()
        if any(a[1]>b[0] for a,b in zip(rows,rows[1:])):raise ValueError('Overlapping service on one aircraft')
    return dict(time_s=now,accepted_task_ids=sorted(accepted),accepted_evidence=accepted,
        ongoing=active,pending_acceptance=pending,failed_attempts=failures,
        new_service_eligible_task_ids=sorted(task_ids-set(accepted)-set(active)),
        history=history,pending_completion_credit=False,
        prefix_rule='preserve fixed start/identities/exposure prefix; constrain remainder only; evaluate full exposure using retained prefix',
        full_joint_solver_verified=False)


def check_service_plan(boundary, new_services, continuations):
    """Validate source-node carryover and task flow bookkeeping only."""
    now=boundary['time_s']; expected=boundary['ongoing']
    if set(continuations)!=set(expected):raise ValueError('Ongoing service lost or duplicated')
    for task,old in expected.items():
        new=continuations[task]
        for field in ['attempt_id','aircraft_id','battery_pair_id','sortie_id','original_start_s','end_s',
                      'remaining_exposure_intervals_s','executed_exposure_intervals_s','prefix_evidence_id']:
            if new.get(field)!=old.get(field):raise ValueError('Changed service prefix or commitment: '+field)
        if new.get('remaining_s')!=old['remaining_s'] or new.get('boundary_start_s')!=now:
            raise ValueError('Restarted service duration')
    old_ids={a['attempt_id'] for a in boundary['history']};new_ids=set();new_tasks=set()
    allowed=set(boundary['new_service_eligible_task_ids'])
    for row in new_services:
        if row['attempt_id'] in old_ids|new_ids or row['task_id'] in new_tasks:raise ValueError('Repeated attempt or new task identity')
        if row['task_id'] not in allowed or _time(row['start_s'])<now:raise ValueError('Invalid new service')
        if _time(row['end_s'])<=row['start_s']:raise ValueError('Nonpositive new service duration')
        new_ids.add(row['attempt_id']);new_tasks.add(row['task_id'])
    return dict(bookkeeping_valid=True,physical_exposure_and_route_feasibility_verified=False)


def remaining_event_clock(start_s,end_s,now_s):
    """Event clock used by the same visit-flow equation after cutting history."""
    start,end,now=map(_time,[start_s,end_s,now_s])
    if end<=start or end<=now:raise ValueError('Service has no remaining interval')
    effective_start=max(start,now)
    return dict(start_s=effective_start,duration_s=end-effective_start,end_s=end)
