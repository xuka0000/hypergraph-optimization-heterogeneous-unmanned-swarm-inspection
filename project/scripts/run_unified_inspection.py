"""Single-session, identity-guarded construction/solve of the joint model.

This is within-stage model implementation, not a public-algorithm comparison.
No workers, observer daemon, cloud call or background restart is created.
"""
from run_engineering_full_joint import *
from Agent.Publication.engineering_unified_inspection import JointModel
from Agent.Publication.engineering_road_motion import RoadMotion
from Agent.Publication.engineering_joint_optimality import require_complete_model,solve_lexicographic
from Agent.Publication.engineering_solve_trace import SolveTrace


def project_warm_continuity(initial):
    """Remove sub-micrometre source rounding; never repair a solver output.

    Fixed-duration linear projection is initializer preparation only. All
    durations/controls remain free in the ensuing single MINLP.
    """
    import math
    from scipy.sparse import csr_matrix
    from scipy.sparse.linalg import lsmr
    rows=[]
    for flight in initial:
        tl=flight['timeline'];n=len(tl);origin=np.asarray(tl[0]['control_points_m'][0]);cp=np.concatenate([np.asarray(s['control_points_m'])-origin for s in tl]);matrix=[];rhs=[]
        def boundary(k,s,r):
            row=np.zeros(8*n);T=tl[k]['end_s']-tl[k]['start_s'];factor=math.factorial(7)/math.factorial(7-r)/T**r
            offset=0 if s==0 else 7-r
            for h in range(r+1):row[8*k+offset+h]=factor*(-1)**(r-h)*math.comb(r,h)
            return row
        for k in range(n-1):
            for r in range(4):matrix.append(boundary(k,1,r)-boundary(k+1,0,r));rhs.append([0,0,0])
        for k,s in [(0,0),(n-1,1)]:
            for r in range(4):matrix.append(boundary(k,s,r));rhs.append((cp[0 if s==0 else -1] if r==0 else np.zeros(3)).tolist())
        A=np.asarray(matrix);rhs=np.asarray(rhs);norm=np.linalg.norm(A,axis=1);scaled=A/norm[:,None];residual=(rhs-A@cp)/norm[:,None]
        delta=np.column_stack([lsmr(csr_matrix(scaled),residual[:,j],atol=1e-14,btol=1e-14,maxiter=5000)[0] for j in range(3)])
        error=float(np.abs(delta).max());assert error<1e-5,(error,'not a rounding-only repair')
        fixed=cp+delta;remaining=float(np.abs(A@fixed-rhs).max())
        for k,seg in enumerate(tl):seg['control_points_m']=(fixed[k*8:(k+1)*8]+origin).tolist()
        rows.append(dict(max_control_change_m=error,max_absolute_boundary_residual=remaining,source_flight=flight['sequence'],role='warm initializer rounding projection only; independent replay still required'))
    return rows


def road_options(scene,terrain,initial):
    cache=OUT/'unified_road_options.json'
    source=sha(ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes/rommerskirchen.json')
    code=sha(ROOT/'Agent/Publication/engineering_road_motion.py')
    if cache.exists():
        old=read(cache)
        if old['scene_sha256']==source and old['road_code_sha256']==code:return old['options']
        raise RuntimeError('Road cache identity changed; retain and diagnose')
    pts=np.concatenate([np.asarray(s['control_points_m']) for d in initial for s in d['timeline']]);lo=pts[:,:2].min(axis=0)-300;hi=pts[:,:2].max(axis=0)+300
    stops=[s['id'] for s in scene['stops'] if np.all(np.asarray(s['xy_m'])>=lo) and np.all(np.asarray(s['xy_m'])<=hi)]
    road=RoadMotion(scene);options=[]
    for v,vehicle in enumerate(scene['fleet']['vehicles']):
        xyz=vehicle['initial_xyz_m'];yaw=vehicle['initial_heading_rad']
        rows=[dict(id='stationary',visits=[],leg_seconds=[0.,0.],sites=[dict(stop_id=vehicle['depot_stop'],xyz_m=xyz,yaw_rad=yaw)]*3,route=None)]
        for stop in stops:
            if stop==vehicle['depot_stop']:continue
            route=road.itinerary(vehicle,[stop])
            if route is None:continue
            event=route['visit_events'][0];xy=event['position_xy_m'];ground=float(terrain(*xy))
            rows.append(dict(id=stop,visits=[stop],leg_seconds=[event['route_time_s'],route['duration_s']-event['route_time_s']],sites=[dict(stop_id=vehicle['depot_stop'],xyz_m=xyz,yaw_rad=yaw),dict(stop_id=stop,xyz_m=[*xy,ground],yaw_rad=event['yaw_rad']),dict(stop_id=vehicle['depot_stop'],xyz_m=xyz,yaw_rad=route['terminal_yaw_rad'])],route=route))
        options.append(rows);print('candidate vehicle tours',v,len(rows),flush=True)
    save(cache,dict(scene_sha256=source,road_code_sha256=code,selection='all configured stops in source initial flight bounding box plus300m; no objective/result-based selection',crop_xy_m=[lo.tolist(),hi.tolist()],candidate_stops=stops,scope='stationary or one intermediate stop; source-directed feasible Ackermann primitives with variable time scale; not all regional vehicle paths',options=options))
    return options


def main():
    p=argparse.ArgumentParser();p.add_argument('--soc',type=float,default=1.);p.add_argument('--construct-only',action='store_true');p.add_argument('--repair',action='store_true');p.add_argument('--collocation-order',type=int,default=5);p.add_argument('--validate-existing',action='store_true');args=p.parse_args()
    if args.collocation_order not in [5,9,17]:raise ValueError('Declared collocation refinement orders are 5,9,17')
    tag=f'unified_SOC{args.soc:g}';status=OUT/(tag+'_status.json');history=OUT/(tag+'_attempts.json')
    if args.validate_existing:
        from verify_unified_inspection import verify
        verify(tag)
        # Validation is explicitly separate from a new scientific solve.
        check_record=read(status);check_record.update(independent_candidate_accepted=True,acceptance=tag+'_acceptance.json',validation_entry_sha256=sha(Path(__file__)),validation_observed_at=stamp());save(status,check_record)
        return
    if not args.construct_only:
        # Latest user: complete the model first, then measure time to optimum.
        # This check happens before changing any prior experiment records.
        require_complete_model(ROOT,'research_quality/inspection_unified_completion.json')
    else:
        tag=f'unified_build_SOC{args.soc:g}';status=OUT/(tag+'_status.json');history=OUT/(tag+'_attempts.json')
    if status.exists():
        prior=read(status)
        if prior.get('state')=='running' and psutil.pid_exists(prior['pid']) and abs(psutil.Process(prior['pid']).create_time()-prior['create_time'])<.01:raise RuntimeError('Identical experiment live; do not duplicate')
        if not args.repair:raise RuntimeError('Existing attempt: inspect before compatible repair')
        rows=read(history) if history.exists() else []
        retained={}
        for suffix in ['_construction.json','_result.json','_solution_values.json','_validation.json','_resource_validation.json','_acceptance.json','_initializer_projection.json']:
            path=OUT/(tag+suffix)
            if path.exists():retained[path.name]=read(path)
        for path in OUT.glob(tag+'_sortie[0-9].json'):retained[path.name]=read(path)
        prior['retained_artifacts']=retained
        rows.append(prior);save(history,rows)
    if min(__import__('shutil').disk_usage('C:/').free-10*2**30,__import__('shutil').disk_usage('D:/').free-30*2**30)<300*2**20:
        raise RuntimeError('Storage floor plus300MiB peak reservation not available')
    guard=dict(process_commit_limit_bytes=None,solver_memory_limit='SCIP default unlimited',
               storage_floors_GiB=dict(C=10,D=30),resource_interrupt_is_completion=False)
    record=dict(task_id='inspection_unified_execution',experiment_id=tag,state='running',pid=os.getpid(),create_time=psutil.Process().create_time(),observed_at=stamp(),attempt=1+(len(read(history)) if history.exists() else 0),code_sha256=sha(ROOT/'Agent/Publication/engineering_unified_inspection.py'),entry_sha256=sha(Path(__file__)),memory_guard=guard,
        admission=dict(scope='authorized same-stage model implementation; no comparison or stage transition',public_baseline_preflight='not applicable to model/evaluator development',cloud_cost=0,workers=0,time_limit_s=None,relative_gap=0.,absolute_gap=0.,termination='all lexicographic priorities numerically optimal and independently accepted',resource_interrupt_is_completion=False))
    save(status,record);begin=time.perf_counter()
    code_archive=OUT/'unified_implementation_code.json';versions=read(code_archive) if code_archive.exists() else {}
    for path in [ROOT/'Agent/Publication/engineering_unified_inspection.py',Path(__file__)]:
        versions.setdefault(sha(path),dict(path=str(path.relative_to(ROOT)),text=path.read_text(encoding='utf-8'),first_observed_at=record['observed_at']))
    save(code_archive,versions)
    trace=None
    try:
        scene,pilot,terrain,weather=context();initial=[read(OUT/f'joint_optical_bound_SOC1_sortie{k}.json') for k in range(2)]
        options=road_options(scene,terrain,initial)
        projection=project_warm_continuity(initial);save(OUT/(tag+'_initializer_projection.json'),dict(rows=projection,source_files=[f'joint_optical_bound_SOC1_sortie{k}.json' for k in range(2)],source_modified=False))
        witness_path=ROOT/'research_quality/inspection_unified_witnesses.json'
        witnesses=read(witness_path).get('rows',[]) if witness_path.exists() else []
        j=JointModel(scene,initial[0]['targets'],initial,terrain,weather,options,args.soc,args.collocation_order).compile(witnesses)
        graph=j.graph();save(OUT/(tag+'_incidence.json'),graph)
        residuals=sorted(j.residuals,key=lambda x:x['violation'],reverse=True)
        warm_valid=j.m.checkSol(j.sol,printreason=True,completely=True)
        construction=dict(observed_at=stamp(),code_sha256=record['code_sha256'],seconds=time.perf_counter()-begin,variables=len(j.vars),constraints=len(j.rows),graph={k:v for k,v in graph.items() if k!='edges'},warm_start_scip_feasible=warm_valid,warm_start_violations=residuals[:100],all_violation_count=len(residuals),physics_acceptance='exact sampled body rates/blur and source terrain/weather; external continuous collision/visibility/physical checks still required',complete_contract_implemented=False,
            limits=['one intermediate vehicle stop per candidate tour','eight source hours sampled by optimized clock; continuous boundary traces need closure','finite quadrature/collocation; continuous separation and energy error closure pending'])
        save(OUT/(tag+'_construction.json'),construction);print('construction',json.dumps({k:construction[k] for k in ['seconds','variables','constraints','warm_start_scip_feasible','all_violation_count']}),flush=True)
        construction.update(attempt=record['attempt'],entry_sha256=record['entry_sha256'],collocation_order=args.collocation_order,transcription_change='5-node initial relaxation with independent continuous acceptance; different finite grid from9, not an algebraic equivalence claim')
        save(OUT/(tag+'_construction.json'),construction)
        if not warm_valid:
            bounds=[dict(variable=name,value=j.val(v),lb=v.getLbOriginal(),ub=v.getUbOriginal()) for name,v in j.vars.items() if j.val(v)<v.getLbOriginal()-1e-7 or j.val(v)>v.getUbOriginal()+1e-7]
            construction['warm_bound_violations']=bounds;save(OUT/(tag+'_construction.json'),construction)
            save(OUT/(tag+'_warm_values.json'),j.values(j.sol));raise RuntimeError('Existing independent feasible plan did not map to joint transcription; repair before solve')
        if args.construct_only:
            candidate=j.extract(j.sol);flights=candidate.pop('flights')
            from verify_unified_inspection import resources
            replay=resources(candidate,j.values(j.sol),flights,scene,options,terrain,weather)
            replay.update(observed_at=stamp(),source='constructed initializer, not optimizer result',code_sha256=record['code_sha256'],entry_sha256=record['entry_sha256'])
            save(OUT/(tag+'_initializer_resource_check.json'),replay)
            if not replay['all_pass']:raise RuntimeError('Expanded initializer resource replay failed')
            record.update(state='constructed',observed_at=stamp());save(status,record);return
        j.m.addSol(j.sol,free=False)
        # SCIP's native fopen fails on this Unicode absolute path on Windows;
        # a relative ASCII leaf under the already authorized output directory
        # keeps the same output location and avoids changing the environment.
        logfile=tag+f'_attempt{record["attempt"]}_scip.log'
        os.chdir(OUT);j.m.setLogfile(logfile);j.m.setIntParam('display/verblevel',4)
        # The exported incidence is authoritative. Python expression trees
        # and their duplicate graph strings are unnecessary during search.
        del graph,residuals;j.rows.clear();j.residuals.clear();__import__('gc').collect()
        acceptance_round=0
        def phase_acceptance(sol):
            nonlocal acceptance_round
            candidate=j.extract(sol);flights=candidate.pop('flights')
            candidate.update(attempt=record['attempt'],sortie_files=[f'{tag}_sortie{k}.json' for k in range(len(flights))])
            for k,d in enumerate(flights):save(OUT/f'{tag}_sortie{k}.json',d)
            save(OUT/(tag+'_solution_values.json'),j.values(sol));save(OUT/(tag+'_result.json'),candidate)
            save(status,dict(record,state='candidate_ready',phase_validation=True))
            from verify_unified_inspection import verify
            report=verify(tag,raise_on_failure=False)
            report['accepted']=report['candidate_accepted']
            # Preserve the actual checked candidate and both receipts for
            # each pass. Scalar-variable snapshots are already in trace.
            retained=dict(candidate=candidate,flights=flights,acceptance=report,
                resources=read(OUT/(tag+'_resource_validation.json')),
                physical=read(OUT/(tag+'_validation.json')))
            save(trace_dir/f'acceptance_{acceptance_round:06d}.json',retained)
            acceptance_round+=1
            save(status,dict(record,state='running',phase_validation=False));return report
        def refine(report):
            from Agent.Publication.engineering_joint_safety import apply_witnesses
            before=len(j.rows)
            added=apply_witnesses(j,report['witnesses'])
            if added:
                witnesses.extend(added)
                save(witness_path,dict(rows=witnesses,observed_at=stamp(),
                    scope='necessary constraints only; continuous completeness not implied',
                    source_attempt=record['attempt'],source_trace=str(trace_dir.relative_to(ROOT))))
                save(trace_dir/f'refinement_{acceptance_round:06d}.json',dict(witnesses=added,
                    new_rows=len(j.rows)-before,unmapped_failures=report.get('unmapped_failures',[])))
            return len(j.rows)-before
        trace_dir=OUT/(tag+f'_attempt{record["attempt"]}_trace')
        trace=SolveTrace(trace_dir,record,j.objectives,j.vars,begin=begin)
        j.m.includeEventhdlr(trace,'quality_time_trace','Every incumbent, bounds, nodes and objective vector')
        trace.write('initializer',j.sol,snapshot=True,extra={'role':'warm initializer, not a new optimizer result'})
        record['trace_directory']=str(trace_dir.relative_to(ROOT));save(status,record)
        lex=solve_lexicographic(j.m,j.objectives,lambda x:save(OUT/(tag+'_optimality.json'),x),phase_acceptance,trace,refine)
        trace.close('complete' if lex['complete'] else 'unfinished')
        result=dict(observed_at=stamp(),solver_status=str(j.m.getStatus()),solver_seconds=j.m.getSolvingTime(),solution_count=j.m.getNSols(),dual_bound=j.m.getDualbound(),primal_bound=j.m.getPrimalbound(),gap=j.m.getGap(),nodes=j.m.getNNodes(),construction_seconds=construction['seconds'],warm_start_only_unknown=True,full_continuous_acceptance=False,complete_contract_implemented=False)
        sol=j.m.getBestSol()
        result.update(attempt=record['attempt'],code_sha256=record['code_sha256'],entry_sha256=record['entry_sha256'],collocation_order=args.collocation_order,logfile=logfile)
        result['lexicographic_optimality']=lex
        if sol is not None:
            result.update(j.extract(sol));flights=result.pop('flights')
            for k,d in enumerate(flights):save(OUT/f'{tag}_sortie{k}.json',d)
            result['sortie_files']=[f'{tag}_sortie{k}.json' for k in range(len(flights))]
            save(OUT/(tag+'_solution_values.json'),j.values(sol))
            initial_vector=[j.val(e) for _,e,_ in j.objectives]
            final_vector=[float(j.m.getSolVal(sol,e)) for _,e,_ in j.objectives]
            result['objective_vector']=final_vector
            result['objective_vector_improvement_over_initializer']=[a-b for a,b in zip(initial_vector,final_vector)]
            result['objective_improvement_over_initializer']=initial_vector[1]-final_vector[1]
            result['improved_over_initializer']=any(a-b>1e-5 for a,b in zip(initial_vector,final_vector))
            result['warm_start_only_unknown']=not result['improved_over_initializer']
        save(OUT/(tag+'_result.json'),result)
        record.update(state='candidate_ready' if sol is not None else 'finished_without_candidate',observed_at=stamp(),result=tag+'_result.json',total_seconds=time.perf_counter()-begin);save(status,record)
        print(json.dumps(result,ensure_ascii=False),flush=True)
        if sol is not None:
            # The checker consumes exactly the exported result. It must not
            # reschedule, reassign, repair trajectories or optimize anything.
            del sol,j;__import__('gc').collect()
            from verify_unified_inspection import verify
            verify(tag)
            record.update(independent_candidate_accepted=True,acceptance=tag+'_acceptance.json',validation_entry_sha256=sha(Path(__file__)),validation_observed_at=stamp(),total_seconds=time.perf_counter()-begin);save(status,record)
    except BaseException as exc:
        record.update(state='failed',observed_at=stamp(),error=repr(exc),total_seconds=time.perf_counter()-begin);save(status,record);raise
    finally:
        if trace is not None:trace.close('interrupted_or_failed')


if __name__=='__main__':main()
