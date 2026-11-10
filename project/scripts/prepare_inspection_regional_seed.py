"""Whole-region initializer diagnostics, never a replacement joint solve.

Construct source-road/flight starts for every tower.  A finite candidate search
failure is not infeasibility.  Proposals are NOT admitted to SCIP before the
complete regional acceptance (including schedule, resources and road dynamics).
"""
from pathlib import Path
import argparse, datetime, hashlib, json, math, os, sys, time
import psutil

# Scope these linear-algebra thread settings to this numerical initializer.
# The observed import private memory drops from about0.56GiB to0.062GiB;
# this is an execution setting, not a different physical model.
for _thread_env in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_thread_env] = '1'

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
OUT = ROOT/'总结/03_实验数据/工程化联合求解/regional_joint/full_coverage_initializer.json'


def stamp():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def write(data):
    tmp = OUT.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), 'utf-8')
    os.replace(tmp, OUT)


def dependency_identity():
    names = ['scripts/prepare_inspection_regional_seed.py',
             'research_quality/inspection_observation_domain_repair.json',
             'research_quality/inspection_regional_model_data.json']
    names += [p.relative_to(ROOT).as_posix() for p in
              (ROOT/'Agent/Publication').glob('engineering_*.py')]
    return {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sorted(names)}


def road_scaffold(radius=None):
    """Source-edge routes covering the nearest-stop assignment of ALL towers.

    This cheap deterministic initializer has no continuous driving certificate.
    Its centerline travel time is a nominal estimate, not a physical lower bound.
    """
    import heapq
    record=json.loads((ROOT/'research_quality/inspection_regional_model_data.json').read_text('utf-8-sig'))
    source=ROOT/record['source']['path']
    if hashlib.sha256(source.read_bytes()).hexdigest()!=record['source']['sha256']:
        raise ValueError('Source geography changed')
    scene=json.loads(source.read_text('utf-8-sig')); stops=scene['stops']
    nodes={n['id']:(n['easting_m'],n['northing_m']) for n in scene['roads']['nodes']}
    graph={n:[] for n in nodes}; edge_by_id={e['id']:e for e in scene['roads']['edges']}
    for e in scene['roads']['edges']:
        length=math.dist(nodes[e['tail']],nodes[e['head']])
        speed=min(float(e['speed_limit_mps']),float(scene['joint_reference_profile']['vehicle']['speed_max_mps']))
        graph[e['tail']].append((e['head'],length/speed,e['id']))
    def shortest(start,end=None):
        dist={start:0.}; pred={}; heap=[(0.,start)]
        while heap:
            cost,a=heapq.heappop(heap)
            if cost!=dist[a]:continue
            if a==end:break
            for b,w,eid in graph[a]:
                c=cost+w
                if c<dist.get(b,math.inf):
                    dist[b]=c;pred[b]=(a,eid);heapq.heappush(heap,(c,b))
        return dist,pred
    bystop={s['id']:s for s in stops}; distances={}
    for s in stops:
        dist,_=shortest(s['osm_node_id'])
        distances[s['id']]={t['id']:dist[t['osm_node_id']] for t in stops}
    task_by_id={q['task_id']:q for q in record['corrections']}
    centers={tower:[sum(task_by_id[t]['target_xyz_m'][q] for t in tasks)/len(tasks) for q in range(2)]
             for tower,tasks in record['domain']['tower_tasks'].items()}
    selected=stops
    if radius is not None:
        if radius<=0:raise ValueError('Positive seed radius required')
        uncovered=set(centers);selected=[]
        while uncovered:
            choices=[]
            for stop in stops:
                covered={t for t in uncovered if math.dist(centers[t],stop['xy_m'])<=radius}
                if covered:
                    depot_cost=min(distances[v['depot_stop']][stop['id']] for v in scene['fleet']['vehicles'])
                    choices.append((-len(covered),depot_cost,stop['id'],covered))
            if not choices:
                # The radius is a heuristic only, never a new feasibility cap.
                t=min(uncovered);stop=min(stops,key=lambda s:math.dist(centers[t],s['xy_m']))
                covered={t}
            else:
                _,_,sid,covered=min(choices,key=lambda z:z[:3]);stop=bystop[sid]
            selected.append(stop);uncovered-=covered
    assigned={s['id']:[] for s in stops}; mapping=[]
    for tower,tasks in record['domain']['tower_tasks'].items():
        center=[sum(task_by_id[t]['target_xyz_m'][q] for t in tasks)/len(tasks) for q in range(2)]
        s=min(selected,key=lambda s:math.dist(center,s['xy_m']))
        assigned[s['id']].append(tower)
        mapping.append(dict(tower=tower,stop=s['id'],tasks=tasks,
                            nominal_target_to_stop_m=math.dist(center,s['xy_m'])))
    service={sid:sum(task_by_id[t]['min_dwell_s'] for tower in towers
                      for t in record['domain']['tower_tasks'][tower])/2
             for sid,towers in assigned.items()}
    routes=[[v['depot_stop'],v['terminal_depot_stop']] for v in scene['fleet']['vehicles']]
    workload=[0.,0.]; pending={s for s,t in assigned.items() if t}
    # Cheapest insertion balanced by capture work; the workload estimate omits
    # flight travel/handling/charging and is NEVER used to claim feasibility.
    while pending:
        best=None
        for s in sorted(pending):
            for car,route in enumerate(routes):
                for k,(a,b) in enumerate(zip(route,route[1:])):
                    change=distances[a][s]+distances[s][b]-distances[a][b]+service[s]
                    loads=workload.copy();loads[car]+=change
                    key=(max(loads),sum(loads),s,car,k)
                    if best is None or key<best[0]:best=(key,change,s,car,k)
        _,change,s,car,k=best;routes[car].insert(k+1,s);workload[car]+=change;pending.remove(s)
    vehicles=[]
    for car,route in enumerate(routes):
        legs=[];modes=[['stop',route[0]]]; seconds=0.;metres=0.
        for a,b in zip(route,route[1:]):
            start=bystop[a]['osm_node_id'];end=bystop[b]['osm_node_id']; dist,pred=shortest(start,end)
            edges=[];node=end
            while node!=start:
                node,eid=pred[node];edges.append(eid)
            edges.reverse();current=start
            mode_leg=[['junction',start]]
            length=0.
            for eid in edges:
                e=edge_by_id[eid]
                if e['tail']!=current:raise ValueError('Disconnected source-edge witness')
                current=e['head']; length+=math.dist(nodes[e['tail']],nodes[e['head']])
                mode_leg += [['edge',eid],['junction',current]]
            if current!=end:raise ValueError('Wrong road endpoint')
            mode_leg.append(['stop',b]);modes+=mode_leg
            legs.append(dict(from_stop=a,to_stop=b,source_edge_ids=edges,nominal_time_s=dist[end],length_m=length))
            seconds+=dist[end];metres+=length
        towers=[t for sid in route[1:-1] for t in assigned[sid]]
        vehicles.append(dict(carrier=car,stop_sequence=route,towers=towers,legs=legs,
                             source_mode_sequence=modes,needed_phases_for_this_scaffold=len(modes),
                             nominal_centerline_travel_s=seconds,centerline_length_m=metres,
                             capture_work_per_two_aircraft_s=sum(service[sid] for sid in route[1:-1]),
                             full_route_continuous_feasible=None))
    served=[t for v in vehicles for t in v['towers']]
    if len(served)!=len(set(served)) or set(served)!=set(record['domain']['tower_tasks']):
        raise ValueError('Regional initializer lost/duplicated towers')
    return dict(observed_at=stamp(),source_sha256=record['source']['sha256'],model_sha256=record['model']['sha256'],
                towers=len(served),tasks=sum(len(x['tasks']) for x in mapping),
                source_nodes=len(nodes),source_edges=len(edge_by_id),configured_stops=len(stops),
                selected_stops=sum(bool(x) for x in assigned.values()),tower_stop_assignment=mapping,vehicles=vehicles,
                seed_cover_radius_m=radius,seed_radius_is_not_a_model_constraint=True,
                all237_source_graph_paths_verified=True,full_joint_feasible=None,SCIP_solution=False,
                routes_are_free_initializers_only=True,original_grid_not_changed=True,
                no_claim_that_this_stop_set_or_grid_size_is_necessary=True)


def make_flight(ctx, obs, tasks, stop, start=120., yaw=0., geometry_cache=None,
                slew_during_capture=False, landing_stop=None, cruise_transfers=False,
                straight_cruise_transfers=False):
    """Free degree7 seed; try direct transfers before obstacle detours.

    Each incoming/recovery transfer retains exactly three optional pieces.
    Inactive pieces copy both endpoint states.  These controls and choices
    are proposed values only; no SCIP variable is fixed.
    """
    import numpy as np
    from Agent.Publication.engineering_regional_seed import camera_pose, smooth_controls, duration, numeric_power
    from scripts.repair_inspection_joint_candidate import contact_repair
    xy = np.array(stop['xy_m']); ground = float(ctx.terrain(*xy))
    if not math.isfinite(ground):
        return dict(passed=False, reason='stop_outside_DEM')
    dock = np.r_[xy, ground+2.4]; exitpoint = np.r_[xy, ground+6.4]
    landing_stop = stop if landing_stop is None else landing_stop
    landing_xy = np.array(landing_stop['xy_m'])
    landing_ground = float(ctx.terrain(*landing_xy))
    if not math.isfinite(landing_ground):
        return dict(passed=False, reason='landing_stop_outside_DEM')
    landing_dock = np.r_[landing_xy, landing_ground+2.4]
    landing_exit = np.r_[landing_xy, landing_ground+6.4]
    parked_pose = np.array([yaw, 0., 0., 1.])
    previous, oldpose, clock = dock, parked_pose, start
    segments = []; geometry_checks = 0

    def geometry(c, contact=None):
        key = (tuple(np.asarray(c).ravel()), None if contact is None else tuple(contact))
        if geometry_cache is not None and key in geometry_cache:
            return geometry_cache[key]
        result = ctx.geometry.segment(c, contact)
        if geometry_cache is not None:
            geometry_cache[key] = result
        return result

    def transfer(first, last, root=False, recovery=False):
        nonlocal geometry_checks
        # Zero-duration copies allow one/two-piece direct transfers in the
        # unchanged three-piece finite domain.
        alternatives = []
        if root:
            alternatives.append([first, exitpoint, last, last])
        elif recovery:
            alternatives.append([first, landing_exit, landing_exit, last])
        else:
            alternatives.append([first, last, last, last])
        base = max(first[2], last[2], (landing_ground if recovery else ground)+10)
        for dz in [4., 8., 12., 18., 24., 32.]:
            h = base+dz
            if recovery:
                alternatives.append([first, np.r_[first[:2], h], landing_exit, last])
            else:
                alternatives.append([first, exitpoint if root else np.r_[first[:2], h], np.r_[last[:2], h], last])
        # The old initializer only climbed diagonally toward the tower. Source
        # conductors can intersect that diagonal. Keep the same three pieces,
        # but also try an elevated intermediate point along/aside the approach.
        if root or recovery:
            a=exitpoint if root else first; b=last if root else landing_exit
            delta=b[:2]-a[:2];length=float(np.linalg.norm(delta))
            normal=np.array([-delta[1],delta[0]])/max(length,1e-12)
            for dz in [12.,24.,40.,56.]:
                for fraction in [.15,.4,.7,.9]:
                    for side in [0.,20.,-20.]:
                        middle=np.r_[(1-fraction)*a[:2]+fraction*b[:2]+side*normal,base+dz]
                        alternatives.append([first,exitpoint,middle,last] if root else [first,middle,landing_exit,last])
        failures = []
        for points in alternatives:
            ok = True
            for k in range(3):
                contact = dock if root and k == 0 else (landing_dock if recovery and k == 2 else None)
                if np.linalg.norm(points[k+1]-points[k]) < 1e-9:
                    continue
                result = geometry(smooth_controls(points[k], points[k+1]), contact)
                geometry_checks += 1
                if not result['passed']:
                    failures.append(result); ok = False; break
            if ok:
                return points
        raise ValueError(json.dumps(dict(stage='transfer_geometry',last_failure=failures[-1]),ensure_ascii=False))

    def add(i, k, a, b, pa, pb):
        nonlocal clock
        T = duration(a,b,pa,pb)
        segments.append(dict(task=i,index=k,start_s=clock,duration_s=T,
                             C=smooth_controls(a,b).tolist(),A=smooth_controls(pa,pb).tolist()))
        clock += T

    def direct_cruise(first,last,pa,pb,contact_at=None):
        if not straight_cruise_transfers:return None
        from Agent.Publication.engineering_regional_seed_motion import straight_cruise
        parts=straight_cruise(first,last,pa,pb,contact=contact_at is not None)
        if parts is None:return None
        for k,part in enumerate(parts):
            contact_point=first if contact_at==0 and k==0 else (last if contact_at==2 and k==2 else None)
            if not geometry(part['C'],contact_point)['passed']:return None
            ground,_=ctx.enclosure.terrain_enclosure(part['C'])
            if float(part['C'][:,2].max()-ground.lo)>100:return None
        return parts

    try:
        for ix,i in enumerate(tasks):
            target = np.array(obs[i]['xyz_m'])
            pose = camera_pose(ctx,target,ctx.record['corrections'][i]['target_xyz_m'],clock,yaw)
            cruise = direct_cruise(previous,target,oldpose,pose,contact_at=0 if ix==0 else None)
            if cruise is None and cruise_transfers and ix:
                from Agent.Publication.engineering_regional_seed_motion import safe_cruise
                cruise = safe_cruise(ctx,previous,target,oldpose,pose,geometry_cache)
            if cruise is not None:
                for k,part in enumerate(cruise):
                    T=part['duration_s']
                    segments.append(dict(task=i,index=k,start_s=clock,duration_s=T,
                                         C=part['C'].tolist(),A=part['A'].tolist(),seed_motion='C3_cruise'))
                    clock+=T
            else:
                points = transfer(previous,target,root=ix == 0)
                # Change pose on the actual moving piece; copied pieces preserve it.
                moving = [k for k in range(3) if np.linalg.norm(points[k+1]-points[k]) > 1e-9]
                pivot = moving[-1] if moving else 0
                poses = [oldpose if k <= pivot else pose for k in range(4)]
                for k in range(3):
                    add(i,k,points[k],points[k+1],poses[k],poses[k+1])
            dwell = float(ctx.targets[i]['effective_service_seconds'])
            final_pose = pose
            if slew_during_capture and ix+1 < len(tasks):
                nxt = tasks[ix+1]
                if np.linalg.norm(target-np.asarray(obs[nxt]['xyz_m'])) < 1e-9:
                    # Exposure remains at [1,1.001]s. The degree7 endpoint-flat
                    # slew is a proposal; the optical oracle must still check it.
                    final_pose = camera_pose(ctx,target,ctx.record['corrections'][nxt]['target_xyz_m'],clock+dwell,yaw)
            segments.append(dict(task=i,index=3,start_s=clock,duration_s=dwell,
                                 C=np.tile(target,(8,1)).tolist(),A=smooth_controls(pose,final_pose).tolist()))
            clock += dwell; previous,oldpose = target,final_pose
        recovery_cruise=direct_cruise(previous,landing_dock,oldpose,parked_pose,contact_at=2)
        if recovery_cruise is not None:
            for k,part in enumerate(recovery_cruise):
                T=part['duration_s']
                segments.append(dict(task=tasks[-1],index=k+4,start_s=clock,duration_s=T,
                                     C=part['C'].tolist(),A=part['A'].tolist(),seed_motion='C3_cruise'))
                clock+=T
        else:
            points = transfer(previous,landing_dock,recovery=True)
            poses = [oldpose,parked_pose,parked_pose,parked_pose]
            for k in range(3):
                add(tasks[-1],k+4,points[k],points[k+1],poses[k],poses[k+1])
        flight = dict(tasks=tasks,stop=stop['id'],landing_stop=landing_stop['id'],
                      start_s=start,end_s=clock,dock=dock.tolist(),landing_dock=landing_dock.tolist(),segments=segments)
        contact_repair(ctx,dict(flights=[flight]))
        # Recheck the actual repaired contacts, not only their predecessors.
        for seg, contact in [(segments[0],dock),(segments[-1],landing_dock)]:
            result = geometry(np.array(seg['C']),contact); geometry_checks += 1
            if not result['passed']:
                return dict(passed=False,reason='repaired_contact_geometry',detail=result)
        nodes,weights = np.polynomial.legendre.leggauss(9)
        energy = 0.
        for seg in segments:
            T = seg['duration_s']
            if T:
                energy += T/7200*sum(w*numeric_power(ctx,seg['C'],T,seg['start_s'],(s+1)/2) for s,w in zip(nodes,weights))
        flight.update(geometry_checked=True,geometry_checks=geometry_checks,
                      reference_quadrature_Wh=float(energy),continuous_physics_checked=False,
                      time_is_reference_not_scheduled=True)
        return dict(passed=True,flight=flight)
    except (ValueError, ArithmeticError) as exc:
        return dict(passed=False,reason='candidate_construction',task=i,detail=str(exc))


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--resume',action='store_true')
    parser.add_argument('--road-scaffold-only',action='store_true')
    parser.add_argument('--seed-stop-radius',type=float,default=None,help='Heuristic stop sharing only; never a joint-model constraint')
    parser.add_argument('--retry-failed',action='store_true',help='Preserve prior failed proposals and retry after a constructor repair')
    parser.add_argument('--repair-view-escapes',action='store_true',help='Search another allowed observation position when a vertical escape is blocked')
    parser.add_argument('--memory-wait-s',type=float,default=0.,help='Foreground wait at safe boundaries, not a solver time limit')
    args=parser.parse_args()
    # No independent supervisor/worker.  Resume only matching immutable inputs.
    identity=dependency_identity(); old=json.loads(OUT.read_text('utf-8')) if OUT.exists() else None
    if old:
        try:
            p=psutil.Process(old['pid'])
            if abs(p.create_time()-old['process_create_time'])<.01:
                raise RuntimeError('Initializer is already live')
        except psutil.NoSuchProcess:
            pass
        if args.resume and old.get('rows') and old['dependency_sha256'] != identity:
            changes={k for k in set(identity)|set(old['dependency_sha256']) if identity.get(k)!=old['dependency_sha256'].get(k)}
            if changes != {'scripts/prepare_inspection_regional_seed.py'}:
                raise ValueError('Physical validators or inputs changed: retained results need revalidation')
            old.setdefault('construction_revisions',[]).append(dict(observed_at=stamp(),dependency_sha256=old['dependency_sha256'],
                checked=old.get('checked'),geometry_passed=old.get('geometry_passed')))
            old['dependency_sha256']=identity
    if args.road_scaffold_only:
        scaffold=road_scaffold(args.seed_stop_radius);data=old or dict(rows=[],full_joint_candidate_accepted=False,full_joint_feasible=None)
        if 'road_scaffold' in data:data.setdefault('road_scaffold_attempts',[]).append(data['road_scaffold'])
        data.update(pid=os.getpid(),process_create_time=psutil.Process().create_time(),running=False,
                    dependency_sha256=identity,road_scaffold=scaffold,observed_at=stamp())
        write(data)
        print(json.dumps(dict(towers=scaffold['towers'],tasks=scaffold['tasks'],selected_stops=scaffold['selected_stops'],
                              vehicles=[{k:v[k] for k in ['carrier','needed_phases_for_this_scaffold','nominal_centerline_travel_s','centerline_length_m','capture_work_per_two_aircraft_s']} for v in scaffold['vehicles']])),flush=True)
        return 0
    from Agent.Publication.engineering_regional_memory import memory_state
    for drive,floor in [('C:/',10.),('D:/',30.15)]:
        if psutil.disk_usage(drive).free/2**30<floor:raise RuntimeError('Storage floor/peak reservation: '+drive)
    waiting=old or dict(rows=[],full_joint_candidate_accepted=False,full_joint_feasible=None)
    waiting.update(pid=os.getpid(),process_create_time=psutil.Process().create_time(),running=True,
                   status='waiting_resource_admission',dependency_sha256=identity,observed_at=stamp())
    write(waiting)
    def memory_ready(required):
        before=time.monotonic();mem=memory_state();announced=False
        while min(mem['physical_available_GiB'],mem['commit_available_GiB'])<required and time.monotonic()-before<args.memory_wait_s:
            if not announced:print(json.dumps(dict(status='foreground_memory_wait',required_GiB=required,observed=mem)),flush=True);announced=True
            time.sleep(min(2.,max(0.,args.memory_wait_s-(time.monotonic()-before))))
            mem=memory_state()
        return mem
    mem=memory_ready(2.35)
    if min(mem['physical_available_GiB'],mem['commit_available_GiB']) < 2.35:
        admission=dict(status='resource_admission_pending',required_GiB=2.35,observed=mem,
                       observed_at=stamp(),host_reserve_GiB=2,estimated_additional_GiB=.35,
                       numerical_threads=1)
        data=old or dict(rows=[],full_joint_candidate_accepted=False,full_joint_feasible=None)
        data.update(pid=os.getpid(),process_create_time=psutil.Process().create_time(),
                    dependency_sha256=identity,running=False,admission=admission)
        write(data)
        print(json.dumps(admission),flush=True)
        return 1
    from Agent.Publication.engineering_regional_seed import numeric_context
    ctx=numeric_context(ROOT)
    observations=json.loads((ROOT/'research_quality/inspection_observation_domain_repair.json').read_text('utf-8-sig'))
    if observations['source_sha256'] != ctx.record['source']['sha256'] or observations['model_sha256'] != ctx.record['model']['sha256']:
        raise ValueError('Observation/source/model mismatch')
    # Only positions are reused. Old pose/physical PASS is not inherited.
    obs={q['index']:q for q in observations['rows']}; byid={t['id']:i for i,t in enumerate(ctx.targets)}
    towers=list(ctx.record['domain']['tower_tasks'].items()); starts=time.perf_counter()
    data=old if old and args.resume and old.get('rows') else dict(
        experiment_id='inspection_regional_day_ahead',stage='full237_initializer_repair',
        source_sha256=ctx.record['source']['sha256'],model_sha256=ctx.record['model']['sha256'],
        dependency_sha256=identity,towers_total=len(towers),tasks_total=len(ctx.targets),
        rows=[],full_joint_candidate_accepted=False,full_joint_feasible=None,
        candidate_search_failure_is_infeasibility=False,
        fixed_joint_decisions=0,old_observation_physics_pass_reused=False)
    data.update(pid=os.getpid(),process_create_time=psutil.Process().create_time(),running=True,
                status='constructing_individual_flight_paths',observed_at=stamp())
    if old and 'road_scaffold' in old:
        data['road_scaffold']=old['road_scaffold'];data['road_scaffold_attempts']=old.get('road_scaffold_attempts',[])
    write(data); done={row['tower'] for row in data['rows'] if row['passed'] or not args.retry_failed}
    for tower,ids in towers:
        if tower in done: continue
        mem=memory_ready(2.)
        if min(mem['physical_available_GiB'],mem['commit_available_GiB'])<2:
            data.update(status='resource_protection',memory=mem); break
        tasks=[byid[t] for t in ids]
        center=[sum(obs[i]['xyz_m'][q] for i in tasks)/len(tasks) for q in range(2)]
        preferred=next((r['stop'] for r in data.get('road_scaffold',{}).get('tower_stop_assignment',[]) if r['tower']==tower),None)
        stops=sorted(ctx.scene['stops'],key=lambda s:(s['id']!=preferred,sum((s['xy_m'][q]-center[q])**2 for q in range(2))))
        failures=[]; candidates=[]; observation_changes=[]
        # Finite initializer search only; all104 stops remain in the joint model.
        for stop in stops[:4]:
            trial=make_flight(ctx,obs,tasks,stop)
            if trial['passed']:
                candidates.append(trial['flight']); break
            failures.append(dict(stop=stop['id'],**trial))
        if not candidates and args.repair_view_escapes:
            from Agent.Publication.engineering_regional_seed import find_observation
            revised=obs.copy();height=max(obs[i]['xyz_m'][2] for i in tasks)+56.
            for i in tasks:
                replacement=find_observation(ctx,i,escape_altitude=height)
                if replacement['passed']:
                    revised[i]=replacement
                    if math.dist(replacement['xyz_m'],obs[i]['xyz_m'])>1e-8:
                        observation_changes.append(dict(task=i,old_xyz_m=obs[i]['xyz_m'],new_xyz_m=replacement['xyz_m'],
                                                        body_visibility_local_physics_passed=True))
            if observation_changes:
                for stop in stops[:4]:
                    trial=make_flight(ctx,revised,tasks,stop)
                    if trial['passed']:
                        candidates.append(trial['flight']);break
                    failures.append(dict(stop=stop['id'],after_view_repair=True,**trial))
        if candidates:
            flight=candidates[0]
            row=dict(tower=tower,passed=True,flight=flight,
                     battery_reference_feasible=flight['reference_quadrature_Wh']<=438.4,
                     failed_stops=failures,observation_changes=observation_changes)
        else:
            row=dict(tower=tower,passed=False,failed_stops=failures,observation_changes=observation_changes)
        existing=next((k for k,r in enumerate(data['rows']) if r['tower']==tower),None)
        row['construction_code_sha256']=identity['scripts/prepare_inspection_regional_seed.py']
        if existing is None:data['rows'].append(row)
        else:
            previous=data['rows'][existing];row['previous_trials']=previous.pop('previous_trials',[])+[previous]
            data['rows'][existing]=row
        data.update(observed_at=stamp(),checked=len(data['rows']),geometry_passed=sum(r['passed'] for r in data['rows']),
                    cumulative_wall_s=data.get('prior_wall_s',0)+time.perf_counter()-starts)
        write(data)
        print(json.dumps(dict(tower=tower,checked=data['checked'],geometry_passed=data['geometry_passed'],
                              energy_Wh=row.get('flight',{}).get('reference_quadrature_Wh'),
                              duration_s=(flight['end_s']-flight['start_s']) if candidates else None,
                              elapsed_s=time.perf_counter()-starts)),flush=True)
    else:
        data['status']='all_tower_initial_paths_checked'
    data.update(running=False,observed_at=stamp(),prior_wall_s=data.get('cumulative_wall_s',0),
                next_required=['combine towers into energy-feasible flights','road trajectory and calendar',
                               'physical pack/charging chain','continuous whole237 acceptance'])
    write(data)
    return 0 if data['status']=='all_tower_initial_paths_checked' else 1


if __name__=='__main__':
    raise SystemExit(main())
