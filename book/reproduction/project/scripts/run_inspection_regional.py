"""Whole237-tower unified integration entry; no automatic paid cloud actions."""
from pathlib import Path
import argparse,datetime,hashlib,json,os,sys,time,traceback
import psutil
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def stamp():return datetime.datetime.now(datetime.timezone.utc).isoformat()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--solve',action='store_true')
    parser.add_argument('--numeric-threads',type=int,default=1,help='BLAS/OpenMP threads for this process; does not set SCIP search limits')
    parser.add_argument('--staged-memory-admission',action='store_true',help='Admit construction at its measured peak; admit search separately with the live native RAM/commit guard')
    parser.add_argument('--road-phases',type=int,default=None,help='Explicit diagnostic grid; default covers every source stop topologically')
    parser.add_argument('--charge-events',type=int,default=156)
    parser.add_argument('--disable-nlp-relaxation',action='store_true')
    parser.add_argument('--compact-memory',action='store_true')
    parser.add_argument('--disable-primal-heuristics',action='store_true')
    parser.add_argument('--verified-repair',action='store_true',help='Recheck and adopt the complete layer11 warm witness; no fixed decisions')
    parser.add_argument('--shared-recovery',action='store_true',help='Equivalent per-flight return curves, mapped witness and necessary regional bounds')
    parser.add_argument('--require-all-towers',action='store_true',help='Explicit full-coverage diagnostic; separate from the original maximum-coverage objective')
    parser.add_argument('--aggressive-lp-scaling',action='store_true',help='SCIP documented lp/scaling=2; feasibility and stability checks unchanged')
    parser.add_argument('--capture-first-lp',action='store_true',help='Save the first actual relaxation and coefficients through verified ASCII staging')
    parser.add_argument('--stable-lp',action='store_true',help='SCIP NUMERICS LP factorization settings only; all mathematical and feasibility tolerances unchanged')
    args=parser.parse_args()
    if args.numeric_threads<1:parser.error('--numeric-threads must be positive')
    for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
        os.environ[key]=str(args.numeric_threads)
    # Set these BEFORE any NumPy/SciPy/SCIP imports. Import-only private memory
    # was0.56GiB with inherited threads and0.062GiB with one numerical thread.
    from Agent.Publication.engineering_regional_solver import RegionalJointModel
    output=ROOT/'总结/03_实验数据/工程化联合求解/regional_joint'
    output.mkdir(parents=True,exist_ok=True);manifest_path=output/'run_manifest.json'
    prior=json.loads(manifest_path.read_text(encoding='utf-8-sig')) if manifest_path.exists() else None
    if prior:
        try:
            p=psutil.Process(prior['pid'])
            if abs(p.create_time()-prior['process_create_time'])<.01:
                raise RuntimeError('Existing owned regional process is live; do not duplicate '+str(prior['pid']))
        except psutil.NoSuchProcess:pass
        with (output/'attempt_history.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(prior,ensure_ascii=False)+'\n')
    from Agent.Publication.engineering_regional_structure import load_regional
    from Agent.Publication.engineering_regional_roads import RegionalRoadDomain
    source,_=load_regional(ROOT);domain=RegionalRoadDomain(source,source['fleet']['vehicles'][0]['initial_xyz_m'][:2])
    minimum=[domain.minimum_all_stop_phases(v)[0] for v in source['fleet']['vehicles']]
    phase_counts=[max(256,n) for n in minimum] if args.road_phases is None else [args.road_phases]*2
    del source,domain
    # Exact code/input identity is captured BEFORE importing a built model or
    # entering SCIP. No late hash is used as a claim about a running version.
    files=sorted((ROOT/'Agent/Publication').glob('engineering_*.py'))+[Path(__file__),ROOT/'scripts/diagnose_inspection_constraint_layers.py',ROOT/'Agent/Publication/engineering_inspection_model.tex',ROOT/'research_quality/inspection_regional_model_data.json']
    identities=[dict(path=p.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files]
    fingerprint=hashlib.sha256(json.dumps(identities,sort_keys=True).encode()).hexdigest()
    manifest=dict(task_id='inspection_regional_day_ahead',run_id=fingerprint[:16]+'_'+str(os.getpid()),
        observed_at=stamp(),pid=os.getpid(),process_create_time=psutil.Process().create_time(),
        user_authorization='完成就开始求解！ / 开始完整接入 / 继续任务',
        scope='all237 towers2133tasks; all source roads and DEM; unified finite transcription with continuous candidate acceptance',
        status='building',solve_requested=args.solve,wall_time_limit_s=None,node_limit=None,gap_target=0,
        solver_configuration={'nlp/disable':args.disable_nlp_relaxation,
            'numerical_library_threads':args.numeric_threads,
            'staged_memory_admission':args.staged_memory_admission,
            'verified_repair_warm_start':args.verified_repair,
            'shared_recovery':args.shared_recovery,'require_all_towers_diagnostic':args.require_all_towers,
            'lp/scaling':2 if args.aggressive_lp_scaling else 1,
            'capture_first_lp':args.capture_first_lp,'stable_lp':args.stable_lp,
            'compact_memory':args.compact_memory,
            'primal_heuristics_disabled':args.disable_primal_heuristics,
            'compact_parameters':{'memory/arraygrowfac':1.05,'memory/savefac':.5,'misc/usevartable':False,'misc/useconstable':False} if args.compact_memory else {},
            'source':'https://scipopt.org/doc/html/PARAMETERS.php',
            'reason':'Explicit memory diagnostic; changes internal relaxation algorithm, not mathematical feasible decisions'},
        numerical_grid=dict(road_phases_per_carrier=phase_counts,minimum_all_stop_topology_phases=minimum,charging_events_per_carrier=args.charge_events,
            trajectory_degree=7,transfer_pieces=3,minimum_positive_piece_s=.001),
        grids_are_not_lossless_physical_limits=True,full_continuous_domain_optimality_claim=False,
        peak_output_reservation_GiB=.25,storage_floors_GiB=dict(C=10,D=30),files=identities,
        fee_authorization_used=False,live_background_supervisor=False)
    manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    from Agent.Publication.engineering_regional_memory import memory_state
    resources=memory_state()
    # Last observed root-LP process peak was4.213GiB. Reserve2GiB for
    # the host. This is a revisable admission estimate, not a solve limit.
    # Avoid rebuilding a known-unadmitted model and repeatedly consuming RAM.
    peak_rss,peak_private=(4.25,4.5) if args.solve and not args.staged_memory_admission else (2.25,2.5)
    required_ram=2+max(0.,peak_rss-psutil.Process().memory_info().rss/2**30)
    required_commit=2+max(0.,peak_private-resources['process_private_GiB'])
    manifest['admission']=dict(mode='construction_then_native_search_guard' if args.staged_memory_admission else 'historical_full_peak',
        observed=resources,required_free_RAM_GiB=required_ram,required_free_commit_GiB=required_commit,
        estimated_admitted_peak_RSS_GiB=peak_rss,estimated_admitted_peak_private_GiB=peak_private,
        previous_full_search_peak_estimate_RSS_GiB=4.25,previous_full_search_peak_estimate_private_GiB=4.5,
        host_reserve_GiB=2,search_admission='fresh RAM and commit; native memory cap; event checks',
        time_node_gap_limits_unchanged=True)
    if resources['physical_available_GiB']<required_ram or resources['commit_available_GiB']<required_commit:
        manifest.update(status='resource_admission_pending',observed_at=stamp(),build_started=False,
            result=dict(status='resource_admission_pending',complete=False,incumbent=None),
            admission=dict(observed=resources,required_free_RAM_GiB=required_ram,required_free_commit_GiB=required_commit,
                estimated_process_peak_RSS_GiB=peak_rss,estimated_process_peak_private_GiB=peak_private,host_reserve_GiB=2,
                estimate_is_revisable=True,not_a_time_or_gap_limit=True,
                next_action='resume this bound executor after fresh RAM/commit admission or a verified lower-memory representation'))
        manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(dict(status=manifest['status'],admission=manifest['admission'],road_phase_counts=phase_counts),ensure_ascii=False),flush=True)
        return 1
    archive=ROOT/'总结/03_实验数据/工程化联合求解/unified_implementation_code.json';stored=json.loads(archive.read_text(encoding='utf-8-sig'))
    for p,row in zip(files,identities):stored.setdefault(row['sha256'],dict(path=row['path'],text=p.read_text(encoding='utf-8-sig')))
    archive.write_text(json.dumps(stored,ensure_ascii=False,indent=2),encoding='utf-8')
    del stored
    started=time.perf_counter();model=None
    try:
        model=RegionalJointModel(ROOT,output,phase_counts,[args.charge_events]*2,args.disable_nlp_relaxation,args.compact_memory,args.disable_primal_heuristics)
        model.run_id=manifest['run_id']
        model.capture_first_lp=args.capture_first_lp
        if args.shared_recovery:
            from Agent.Publication.engineering_regional_compaction import build_shared_repair
            build_shared_repair(model,add_seed=not args.require_all_towers)
        elif args.verified_repair:model.build_verified_repair()
        else:model.build()
        model.path.m.setIntParam('lp/scaling',2 if args.aggressive_lp_scaling else 1)
        if args.stable_lp:
            # Exact subset of installed SCIP NUMERICS emphasis. Avoid its
            # nonlinear bound/side relaxation changes: no tolerance loosened.
            stable={'lp/minmarkowitz':.999,'lp/fastmip':0,'lp/presolving':False,'lp/refactorinterval':40}
            for key,value in stable.items():model.path.m.setParam(key,value)
            manifest['solver_configuration']['stable_lp_parameters']=stable
        if args.require_all_towers:
            for tower,z in model.path.towers.items():model.path.m.addCons(z==1,name='full_coverage_required_'+str(tower))
        manifest['SCIP_log']='scip_'+manifest['run_id']+'.log'
        # SCIP's native Windows fopen cannot reliably open a Unicode parent
        # path. Open the ASCII basename from the existing output directory.
        previous_directory=Path.cwd()
        try:
            os.chdir(output);model.path.m.setLogfile(manifest['SCIP_log'])
        finally:os.chdir(previous_directory)
        if not (output/manifest['SCIP_log']).exists():raise OSError('SCIP log was not created')
        if args.verified_repair or args.shared_recovery:
            manifest['warm_start']=model.warm_start_audit
            manifest['row_hooks']=dict(mode='all_families_checked_by_full_continuous_guard',
                auxiliary_points_added_on_violation=True,prior_materialization_tests='row_hook_progress.json',
                no_physical_family_omitted=True)
        else:
            manifest['row_hooks']=model.exercise_required_row_hooks()
            if not manifest['row_hooks']['all_hooks_materialized']:raise RuntimeError('A required deferred family did not materialize')
        manifest['build']=dict(wall_s=time.perf_counter()-started,tasks=len(model.path.tasks),towers=len(model.path.towers),
            optional_flights=len(model.path.flights),variables=model.path.m.getNVars(),explicit_rows=model.path.m.getNConss(),
            indexed_relations=len(model.path.rel.relations),physical_integrals_materialized=len(model.physics.current_orders),
            physical_families_guarded_by_candidate_handler=True,full_scalar_incidence_available=False,
            rss_GiB=psutil.Process().memory_info().rss/2**30,available_RAM_GiB=psutil.virtual_memory().available/2**30)
        manifest['status']='solving' if args.solve else 'build_complete_not_solved';manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(manifest['build'],ensure_ascii=False),flush=True)
        if args.solve:
            result=model.solve();manifest['result']=result;manifest['status']=result['status']
            print(json.dumps(result,ensure_ascii=False),flush=True)
    except Exception as exc:
        manifest['status']='resource_protection' if model is not None and model.resource_stop else 'engineering_error'
        manifest['error']=dict(type=type(exc).__name__,message=str(exc),traceback=traceback.format_exc())
        print(traceback.format_exc(),flush=True)
    finally:
        manifest['observed_at']=stamp();manifest['elapsed_s']=time.perf_counter()-started
        manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
        if model is not None and hasattr(model,'path'):model.path.m.freeProb()
    return 0 if manifest['status'] in {'build_complete_not_solved','finite_grid_lexicographic_optimal'} else 1


if __name__=='__main__':raise SystemExit(main())
