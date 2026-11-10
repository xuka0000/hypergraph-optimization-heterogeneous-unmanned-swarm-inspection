"""Finite local stress batch; no LLM workers, cloud calls or solver time cutoff."""
import argparse, concurrent.futures, gc, json, math, os, shutil, subprocess, sys, threading, time, traceback
from pathlib import Path
import psutil
from hypergraph_stress_common import *
from run_hypergraph_scip_demo import native_library, native_solve, enumerate_reference
from run_author_bmatching_ils import solve as ils_solve, verify_source

LOCK = threading.Lock()


def validated_result(data, selected):
    v=evaluate(data,selected)
    assert v['feasible']
    usage=v.pop('resource_usage')
    v.update(selected_count=len(selected),active_resources=sum(n==1 for n in usage.values()),
             active_resource_fraction=sum(n==1 for n in usage.values())/max(1,len(usage)),
             original_input_recomputed=True,output_repair=False)
    return v


def scip(data, progress=None, trace=None):
    from pyscipopt import Model, quicksum, SCIP_EVENTTYPE
    start=time.perf_counter(); cpu0=time.process_time()
    model=Model(data['scenario_id']);model.hideOutput()
    model.setIntParam('parallel/maxnthreads',1)
    model.setIntParam('randomization/randomseedshift',0)
    model.setRealParam('limits/gap',.05)
    model.setRealParam('limits/memory',1536.)
    assert model.getParam('limits/time')==1e20
    xs=[model.addVar(vtype='B',name=j['id']) for j in data['jobs']]
    incident={r['id']:[] for r in data['resources']}
    for i,j in enumerate(data['jobs']):
        for r in j['resources']:incident[r].append(i)
    for r in data['resources']:
        model.addCons(quicksum(xs[i] for i in incident[r['id']])<=r['capacity'])
    model.setObjective(quicksum(x*j['value'] for x,j in zip(xs,data['jobs'])),'maximize')
    # Trivial all-zero start only, identical information for every cold-start SCIP run.
    empty=model.createSol()
    for x in xs:model.setSolVal(empty,x,0.)
    model.addSol(empty)
    trajectory=[{'total_seconds':0.,'objective':0.,'upper_bound':sum(j['value'] for j in data['jobs']),
                 'kind':'trivial feasible empty selection, no search yet'}]
    last=[-1e9]; last_obj=[-1.]
    def observe(m,event):
        elapsed=time.perf_counter()-start
        obj=m.getPrimalbound()
        improved=math.isfinite(obj) and obj>last_obj[0]+1e-8 and abs(obj)<1e19
        if not improved and elapsed-last[0]<15:return
        last[0]=elapsed
        if improved:last_obj[0]=obj
        dual=m.getDualbound();gap=m.getGap()
        point={'observed_at':now(),'total_seconds':elapsed,'solver_seconds':elapsed-build,
               'objective':obj if math.isfinite(obj) and abs(obj)<1e19 else None,
               'upper_bound':dual if math.isfinite(dual) and abs(dual)<1e19 else None,
               'gap':gap if math.isfinite(gap) and abs(gap)<1e19 else None,
               'nodes':m.getNTotalNodes()}
        trajectory.append(point)
        if progress:atomic(progress,dict(point,scenario_id=data['scenario_id'],method='SCIP',pid=os.getpid(),status='running',independently_checked=False))
        if trace:
            with trace.open('a',encoding='utf-8') as f:f.write(json.dumps(point,ensure_ascii=False,allow_nan=False)+'\n')
    model.attachEventHandlerCallback(observe,[SCIP_EVENTTYPE.BESTSOLFOUND,SCIP_EVENTTYPE.NODEFOCUSED,SCIP_EVENTTYPE.LPSOLVED])
    build=time.perf_counter()-start
    model.optimize();solving_end=time.perf_counter();sol=model.getBestSol()
    selected=[j['id'] for j,x in zip(data['jobs'],xs) if sol and model.getSolVal(sol,x)>.5]
    end=time.perf_counter();gap=model.getGap();dual=model.getDualbound()
    result={'selected':selected,'objective_reported':model.getSolObjVal(sol) if sol else 0,
            'status':str(model.getStatus()),'gap':gap if abs(gap)<1e19 else None,
            'upper_bound':dual if abs(dual)<1e19 else None,'gap_target_met':gap<=.05+1e-9,
            'time_limit_seconds':None,'scip_memory_MiB':1536,'threads':1,'seed':0,
            'warm_start':'trivial empty only; no hypergraph or prior SCIP decision',
            'build_seconds':build,'solver_seconds':solving_end-start-build,'output_seconds':end-solving_end,
            'total_seconds':end-start,'cpu_seconds':time.process_time()-cpu0,
            'trajectory':trajectory,'nodes':model.getNTotalNodes()}
    model.freeProb();return result


def one(sid,method):
    manifest=load(OUT/'input_manifest.json')
    row=next(r for r in manifest['instances'] if r['scenario_id']==sid)
    protocol=load(OUT/'protocol.json')
    for evidence in protocol['runtime_code']:
        assert sha(ROOT/evidence['path'])==evidence['sha256']
    p=ROOT/row['input']['path'];assert sha(p)==row['input']['sha256']
    data=load(p);verify(data)
    target=OUT/'raw'/f'{sid}__{method}.json'
    assert not target.exists(), 'Never duplicate a completed run'
    cpu0=time.process_time()
    if method=='SCIP':
        result=scip(data,OUT/'progress'/f'{sid}__SCIP.json',OUT/'progress'/f'{sid}__SCIP.jsonl')
    elif method=='Streaming':
        result=native_solve(data,native_library(),.1)
    else:
        verify_source()
        result=ils_solve(data,-1 if method=='Greedy' else 1000,0)
    result['solve_wrapper_cpu_seconds']=time.process_time()-cpu0
    result['validation']=validated_result(data,result['selected'])
    if method=='SCIP':assert abs(result['validation']['objective']-result['objective_reported'])<1e-5
    result.update(method=method,scenario_id=sid,input=row['input'],observed_at=now(),pid=os.getpid(),
                  runner_sha256=sha(Path(__file__)),native_source_unchanged=True)
    atomic(target,result)
    print(json.dumps({'case':sid,'method':method,'objective':result['validation']['objective'],
                      'seconds':result['total_seconds'],'status':result['status']},ensure_ascii=False),flush=True)


def regression():
    results=[]
    fixtures=[('independent',[('AB',4),('CD',5)],9),('conflict',[('AB',4),('BC',8)],8),
              ('swap',[('AB',7),('AC',6),('BD',6)],12)]
    for name,edges,expected in fixtures:
        data={'scenario_id':'regression_'+name,
              'resources':[{'id':r,'capacity':1} for r in sorted(set(''.join(v for v,w in edges)))],
              'jobs':[{'id':'J'+str(i),'resources':list(v),'value':w} for i,(v,w) in enumerate(edges)]}
        exact=enumerate_reference(data);assert exact['objective']==expected
        run=scip(data);v=validated_result(data,run['selected']);assert v['objective']==expected and run['gap_target_met']
        methods={}
        for m in ['Streaming','Greedy','ILS1000']:
            z=native_solve(data,native_library(),.1) if m=='Streaming' else ils_solve(data,-1 if m=='Greedy' else 1000)
            vv=validated_result(data,z['selected']);assert 0<vv['objective']<=expected;methods[m]=vv
        scaled=copy.deepcopy(data)
        for j in scaled['jobs']:j['value']*=10
        exact_scaled=enumerate_reference(scaled)
        assert exact_scaled['objective']==10*expected and exact_scaled['optimal_selections']==exact['optimal_selections']
        results.append({'fixture':name,'expected':expected,'scip':v,'native':methods,'exact_positive_scale_invariance':True})
    # Changes in each controlled factor preserve the other three requested controls.
    a=regular(32,4,4,100,0);b=regular(32,8,4,100,0)
    assert all(j['resources']==k['resources'][:4] and j['value']==k['value'] for j,k in zip(a['jobs'],b['jobs']))
    assert verify(a)['degree_max']==verify(b)['degree_max']==4
    save(OUT/'runner_regression.json',{'observed_at':now(),'passed':True,'fixtures':results,
         'controlled_nested_rank_verified':True,'no_time_limit':True,'runner':ref(Path(__file__))})
    print('Regression passed: exact known cases, all four methods, objective-scale invariance, generator nesting.')


def guard():
    sys.path.insert(0,str(BRIDGE))
    from check_research_stage import require_transition
    receipt=require_transition(ROOT,f'research_quality/{TASK}_review_nodes.json',TASK)
    save(OUT/'transition_receipt.json',receipt)
    p=subprocess.run([sys.executable,'-X','utf8','-B',str(BRIDGE/'check_experiment_preflight.py'),
         '--root',str(ROOT),'--record',str(ROOT/f'research_quality/{TASK}_preflight.json'),'--next-step','public_baselines'],
         capture_output=True,text=True,encoding='utf-8',timeout=30)
    (OUT/'preflight_receipt.json').write_text(p.stdout,encoding='utf-8')
    assert p.returncode==0 and json.loads(p.stdout)['preflight_allowed'],p.stdout+p.stderr


def batch(resume=False):
    guard()
    identity_path=OUT/'experiment_identity.json'
    ident=load(identity_path)
    assert ident['status']=='prepared_authorized' or resume
    if ident.get('pid'):
        assert not psutil.pid_exists(ident['pid']), 'Existing batch PID is live; inspect identity before resume'
    if resume:
        assert not any(psutil.pid_exists(v['pid']) for v in ident.get('active',{}).values()), 'Solver child is live'
    claim=OUT/'batch_claim.json'
    if not resume:
        with claim.open('x',encoding='utf-8') as f:json.dump({'pid':os.getpid(),'time':now()},f)
    ident.update(status='running',pid=os.getpid(),process_created_at=psutil.Process().create_time(),
                 started_at=now(),active={},last_heartbeat=now())
    atomic(identity_path,ident)
    rows=load(OUT/'input_manifest.json')['instances']
    queue=[(r['scenario_id'],m) for r in rows for m in METHODS]
    complete=[]; failures=[]
    def run(sid,method):
        key=f'{sid}__{method}';target=OUT/'raw'/f'{key}.json'
        if target.exists():
            # Compatible result reuse only after verifying identity; no best-of-run selection.
            result=load(target);row=next(x for x in rows if x['scenario_id']==sid)
            assert result['input']==row['input'] and result['method']==method
            return key,'reused'
        if (OUT/'failures'/f'{key}.json').exists():return key,'failure_retained'
        command=[sys.executable,'-X','utf8','-u','-B',str(Path(__file__)),'one','--scenario',sid,'--method',method]
        started=now();clock=time.monotonic();peak=0;reason=None
        with (OUT/'logs'/f'{key}.log').open('a',encoding='utf-8') as log:
            process=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                        creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            proc=psutil.Process(process.pid)
            with LOCK:
                ident['active'][key]={'pid':process.pid,'created_at':proc.create_time(),'started_at':started,'method':method}
                atomic(identity_path,ident)
            while process.poll() is None:
                try:
                    rss=proc.memory_info().rss;peak=max(peak,rss)
                    floors=shutil.disk_usage('C:/').free>=10*2**30 and shutil.disk_usage('D:/').free>=30*2**30
                    if rss>2560*2**20:reason='process_memory_guard_2560MiB'
                    if not floors:reason='storage_hard_floor'
                    if reason:process.terminate();break
                except psutil.NoSuchProcess:break
                time.sleep(1)
            code=process.wait()
        accounting={'observed_at':now(),'scenario_id':sid,'method':method,'pid':process.pid,
                    'started_at':started,'process_wall_seconds':time.monotonic()-clock,
                    'peak_sampled_rss_bytes':peak or None,'exit_code':code,'engineering_stop':reason,
                    'sampling_interval_seconds':1,'peak_note':'sampled process RSS, not exact allocation peak'}
        save(OUT/'logs'/f'{key}_accounting.json',accounting)
        with LOCK:
            ident['active'].pop(key,None);ident['last_heartbeat']=now();atomic(identity_path,ident)
        if code!=0 or not target.exists():
            save(OUT/'failures'/f'{key}.json',dict(accounting,status='incomplete_failure',log=ref(OUT/'logs'/f'{key}.log')))
            return key,'failure'
        return key,'complete'
    # Two bounded SCIP slots, one native slot. Existing unrelated SCIP job remains untouched.
    # Fast kernels cover all cases before ILS so a long local search cannot hide the other cases.
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as mip, concurrent.futures.ThreadPoolExecutor(max_workers=1) as native:
        futures=[]
        for sid,m in queue:
            if m=='SCIP':futures.append(mip.submit(run,sid,m))
        for method in ['Streaming','Greedy','ILS1000']:
            for row in rows:futures.append(native.submit(run,row['scenario_id'],method))
        last_report=-1e9
        while futures:
            done,pending=concurrent.futures.wait(futures,timeout=2,return_when=concurrent.futures.FIRST_COMPLETED)
            for future in done:
                try:
                    key,state=future.result()
                    (failures if 'failure' in state else complete).append(key)
                except Exception as e:
                    failures.append(repr(e));save(OUT/'failures'/f'batch_exception_{len(failures)}.json',{'observed_at':now(),'error':repr(e),'traceback':traceback.format_exc()})
            futures=list(pending)
            with LOCK:
                ident.update(completed_runs=len(complete),failed_runs=len(failures),last_heartbeat=now())
                atomic(identity_path,ident)
            if done or time.monotonic()-last_report>30:
                from summarize_hypergraph_stress import summarize
                summarize();last_report=time.monotonic()
    with LOCK:
        ident.update(status='finished_with_failures' if failures else 'complete',pid=None,completed_at=now())
        atomic(identity_path,ident)
    from summarize_hypergraph_stress import summarize
    summarize()


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['regression','batch','one'])
    ap.add_argument('--scenario');ap.add_argument('--method',choices=METHODS);ap.add_argument('--resume',action='store_true')
    args=ap.parse_args()
    if args.mode=='regression':regression()
    elif args.mode=='one':one(args.scenario,args.method)
    else:batch(args.resume)
