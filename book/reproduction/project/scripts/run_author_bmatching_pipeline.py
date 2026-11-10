"""Paper-configured original-instance comparison. No author core is modified."""
import argparse, ctypes, datetime, itertools, json, os, random, subprocess, sys, time
from pathlib import Path
import psutil
from run_hypergraph_scip_demo import ROOT, OUT as BASE, BRIDGE, load, save, sha, evaluate
from run_author_bmatching_ils import verify_source

OUT = BASE/'论文完整流程原始场景对比'
TASK = 'hypergraph_paper_pipeline_original'

def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()

def atomic(path, value):
    t=path.with_suffix('.tmp'); save(t,value); t.replace(path)

def solve(data, seed=0, tries=15):
    start=time.perf_counter()
    lib=ctypes.CDLL(str(OUT/'author_pipeline.dll'))
    ip=ctypes.POINTER(ctypes.c_int)
    lib.solve_pipeline.argtypes=[ctypes.c_int,ctypes.c_int,ip,ip,ip,ip,ctypes.c_int,ctypes.c_int,ip,ctypes.POINTER(ctypes.c_double),ip]
    lib.solve_pipeline.restype=ctypes.c_int
    ids={r['id']:i for i,r in enumerate(data['resources'])}; ptr=[0];pins=[];weights=[]
    for j in data['jobs']:
        pins.extend(ids[r] for r in j['resources']);ptr.append(len(pins));weights.append(j['value'])
    assert all(isinstance(w,int) and w>0 for w in weights) and sum(weights)<2**31
    caps=[r['capacity'] for r in data['resources']]
    p=(ctypes.c_int*len(ptr))(*ptr);v=(ctypes.c_int*len(pins))(*pins)
    w=(ctypes.c_int*len(weights))(*weights);c=(ctypes.c_int*len(caps))(*caps)
    chosen=(ctypes.c_int*len(weights))();times=(ctypes.c_double*5)();stats=(ctypes.c_int*6)()
    t1=time.perf_counter();n=lib.solve_pipeline(len(caps),len(weights),p,v,w,c,seed,tries,chosen,times,stats);t2=time.perf_counter()
    assert 0<=n<=len(weights)
    selected=[data['jobs'][chosen[i]]['id'] for i in range(n)]
    assert len(set(selected))==n
    t3=time.perf_counter();valid=evaluate(data,selected)
    assert valid['feasible'] and valid['objective']==stats[4]
    return dict(scenario_id=data['scenario_id'],method='HeiHGM_reductions_pin_ILS15_unfold',seed=seed,max_tries=tries,
        selected=selected,validation=valid,reported_objective=stats[4],status='author_stagnation_rule_complete',
        reduction_fixed_edges=stats[0],reduction_folds=stats[1],reduced_greedy_weight=stats[2],reduced_ils_weight=stats[3],residual_nonfree_candidates=stats[5],
        author_graph_build_seconds=times[0],author_reduction_seconds=times[1],author_pin_seconds=times[2],
        author_ils_seconds=times[3],author_unfold_output_seconds=times[4],native_call_seconds=t2-t1,
        python_conversion_seconds=t1-start,output_seconds=t3-t2,total_seconds=t3-start,
        time_limit_seconds=None,optimality_proved=False,observed_at=now(),
        dll_sha256=sha(OUT/'author_pipeline.dll'),adapter_sha256=sha(ROOT/'scripts/author_bmatching_pipeline_adapter.cpp'))

def regression():
    commit=verify_source(); results=[]
    cases=[('fold', [('ab',7),('ac',6),('bd',6)],1),('disjoint',[('ab',4),('cd',5)],1),
           ('cap2',[('ab',7),('ac',6),('ad',5)],2)]
    rng=random.Random(47025)
    for i in range(20):
        cases.append((f'random{i}',[(''.join(rng.sample('abcdef',rng.randint(1,4))),rng.randint(1,30)) for j in range(10)],1+i%2))
    for label,edges,cap in cases:
        d=dict(scenario_id='fixture_'+label,resources=[dict(id=r,capacity=cap) for r in sorted(set(''.join(e for e,w in edges)))],
            jobs=[dict(id=str(i),resources=list(e),value=w) for i,(e,w) in enumerate(edges)])
        optimum=max(sum(w for b,(e,w) in zip(bits,edges) if b) for bits in itertools.product([0,1],repeat=len(edges))
            if all(sum(b for b,(e,w) in zip(bits,edges) if v['id'] in e)<=cap for v in d['resources']))
        print('CHECK',label,flush=True)
        r=solve(d);assert r['validation']['objective']<=optimum
        if label in ['fold','disjoint','cap2']:assert r['validation']['objective']==optimum
        r['enumerated_optimum']=optimum;results.append(r)
    assert sum(x['reduction_folds'] for x in results)>0, 'Need an exercised unfold path'
    from run_scip_gap05 import solve as scip_solve
    toy=load(BASE/'instance.json');sr=scip_solve(toy,[]);sv=evaluate(toy,sr['selected']);assert sv['feasible'] and sv['objective']==42 and sr['scip_gap']==0
    save(OUT/'runner_regression.json',dict(passed=True,observed_at=now(),source_commit=commit,source_files_verified=148,
        author_fixtures=results,scip_fixture_objective=42,original_evaluator_sha256=sha(ROOT/'scripts/run_hypergraph_scip_demo.py'),
        scope='I/O, exact reductions and unfold feasibility exercised against exhaustive tiny optima; no performance claim.'))
    print(json.dumps(dict(passed=True,cases=len(results),folds=sum(x['reduction_folds'] for x in results))),flush=True)

def one(sid,seed):
    verify_source();p=load(OUT/'protocol.json');assert seed in p['seeds']
    row=next(x for x in load(OUT/'input_manifest.json')['instances'] if x['scenario_id']==sid)
    assert sha(ROOT/row['path'])==row['sha256']
    name=f'{sid}_author_s{seed}' if seed>=0 else f'{sid}_SCIP_cold_gap05'
    dest=OUT/'raw'/f'{name}.json';assert not dest.exists()
    with (OUT/'claims'/f'{name}.json').open('x',encoding='utf-8') as f:
        json.dump(dict(pid=os.getpid(),create_time=psutil.Process().create_time(),observed_at=now()),f)
    d=load(ROOT/row['path'])
    if seed>=0:r=solve(d,seed)
    else:
        old=load(BASE/'SCIP_gap5持续求解/experiment_identity.json')
        assert old.get('current_instance')!=sid or old.get('status')!='running', 'Avoid duplicate live instance solve'
        from run_scip_gap05 import solve as scip_solve
        r=scip_solve(d,[],OUT/'progress'/f'{sid}_SCIP.json')
        r['warm_start']='empty feasible solution only; no previous SCIP or hypergraph incumbent'
        r['method']='SCIP_MILP_cold_gap05';r['validation']=evaluate(d,r['selected'])
        assert r['validation']['feasible'] and abs(r['validation']['objective']-r['reported_objective'])<1e-5
    r.update(input_sha256=row['sha256'],input_path=row['path'],observed_at=now())
    atomic(dest,r);print(json.dumps({k:r.get(k) for k in ['scenario_id','method','seed','reported_objective','scip_gap','total_seconds','status']},ensure_ascii=False),flush=True)

def batch():
    sys.path.insert(0,str(BRIDGE));from check_research_stage import require_transition
    save(OUT/'transition_receipt.json',require_transition(ROOT,f'research_quality/{TASK}_review_nodes.json',TASK))
    c=subprocess.run([sys.executable,'-X','utf8','-B',str(BRIDGE/'check_experiment_preflight.py'),'--root',str(ROOT),'--record',str(ROOT/f'research_quality/{TASK}_preflight.json'),'--next-step','public_baselines'],capture_output=True,text=True,encoding='utf-8',timeout=30)
    (OUT/'preflight_receipt.json').write_text(c.stdout,encoding='utf-8');assert c.returncode==0 and json.loads(c.stdout)['preflight_allowed'],c.stdout+c.stderr
    ident=load(OUT/'experiment_identity.json');assert ident['status']=='prepared_authorized' and ident['pid'] is None
    with (OUT/'run_claim.json').open('x',encoding='utf-8') as f:json.dump(dict(pid=os.getpid(),observed_at=now()),f)
    ident.update(status='running',pid=os.getpid(),create_time=psutil.Process().create_time(),started_at=now(),completed=0,failed=0,active=[])
    atomic(OUT/'experiment_identity.json',ident)
    rows=load(OUT/'input_manifest.json')['instances'];seeds=load(OUT/'protocol.json')['seeds']
    # One native slot and at most one additional cold-start SCIP slot.
    todo=[(r['scenario_id'],s) for r in rows for s in seeds if s>=0]
    scip_todo=[(r['scenario_id'],-1) for r in rows if not r['reuse_optimal']]
    active=[]
    while todo or scip_todo or active:
        for lane,q in [('author',todo),('scip',scip_todo)]:
            if q and not any(x['lane']==lane for x in active) and psutil.virtual_memory().available>2.5*2**30:
                sid,s=q.pop(0);name=f'{sid}_s{s}';log=(OUT/'logs'/f'{name}.log').open('w',encoding='utf-8')
                child=subprocess.Popen([sys.executable,'-X','utf8','-u','-B',str(Path(__file__)),'one','--scenario',sid,'--seed',str(s)],stdout=log,stderr=subprocess.STDOUT)
                active.append(dict(lane=lane,p=child,log=log,sid=sid,seed=s,create_time=psutil.Process(child.pid).create_time()))
                print('START',lane,sid,s,child.pid,flush=True)
        for x in active[:]:
            code=x['p'].poll()
            if code is None:
                proc=psutil.Process(x['p'].pid)
                # Process safety only; resource stop is a failure, never successful scientific termination.
                if proc.memory_info().rss>8*2**30 or psutil.virtual_memory().available<768*2**20:
                    x['p'].terminate();x['guard_stop']='RAM protection';code=x['p'].wait()
            if code is not None:
                x['log'].close();active.remove(x);ident['completed' if code==0 else 'failed']+=1
                if code!=0:atomic(OUT/'failures'/f'{x["sid"]}_s{x["seed"]}.json',dict(scenario_id=x['sid'],seed=x['seed'],exit_code=code,guard=x.get('guard_stop'),observed_at=now()))
                print('END',x['sid'],x['seed'],code,flush=True)
        ident.update(active=[dict(lane=x['lane'],scenario_id=x['sid'],seed=x['seed'],pid=x['p'].pid,create_time=x['create_time']) for x in active],last_observed_at=now(),pending=len(todo)+len(scip_todo))
        atomic(OUT/'experiment_identity.json',ident)
        if active or todo or scip_todo:time.sleep(2)
    ident.update(status='complete' if ident['failed']==0 else 'incomplete_failures',pid=None,finished_at=now());atomic(OUT/'experiment_identity.json',ident)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['regression','one','batch']);ap.add_argument('--scenario');ap.add_argument('--seed',type=int,default=0);a=ap.parse_args()
    if a.mode=='regression':regression()
    elif a.mode=='one':one(a.scenario,a.seed)
    else:batch()
