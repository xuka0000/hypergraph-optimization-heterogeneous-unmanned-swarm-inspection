"""Continue the unchanged standard MILP to the user-approved 5% relative gap.

No solver time cutoff and no first-solution interruption. Previous SCIP incumbents
are standard MIP starts; previous native outputs are never supplied to SCIP.
"""
import argparse, datetime, gc, json, os, subprocess, sys, time, traceback
from pathlib import Path
from pyscipopt import Model, SCIP_EVENTTYPE, quicksum
from run_hypergraph_scip_demo import ROOT, OUT as BASE, BRIDGE, load, save, sha, evaluate

OUT = BASE/'SCIP_gap5持续求解'
PREFIX = 'hypergraph_scip_gap05'


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def atomic(path, data):
    tmp = path.with_suffix(path.suffix+'.tmp')
    save(tmp, data)
    tmp.replace(path)


def solve(data, selected, progress=None):
    start = time.perf_counter()
    model = Model(data['scenario_id'])
    model.hideOutput()
    model.setIntParam('parallel/maxnthreads', 1)
    model.setIntParam('randomization/randomseedshift', 0)
    model.setRealParam('limits/gap', .05)
    model.setRealParam('limits/memory', 6144.)
    assert model.getParam('limits/time') == 1e20
    jobs = data['jobs']
    xs = [model.addVar(name=j['id'], vtype='B') for j in jobs]
    incident = {r['id']: [] for r in data['resources']}
    for i, j in enumerate(jobs):
        for r in j['resources']:
            incident[r].append(i)
    for r in data['resources']:
        if incident[r['id']]:
            model.addCons(quicksum(xs[i] for i in incident[r['id']])<=r['capacity'],name=r['id'])
    model.setObjective(quicksum(j['value']*x for j,x in zip(jobs,xs)),'maximize')
    v = evaluate(data, selected)
    assert v['feasible']
    init = model.createSol()
    chosen_set = set(selected)
    for j,x in zip(jobs,xs):
        model.setSolVal(init,x,float(j['id'] in chosen_set))
    assert model.addSol(init)
    last = [0.]
    trajectory = []
    def observe(m,event):
        wall = time.perf_counter()-t1
        is_best = event.getType() == SCIP_EVENTTYPE.BESTSOLFOUND
        if not is_best and wall-last[0]<15:
            return
        last[0]=wall
        primal,dual,gap = m.getPrimalbound(),m.getDualbound(),m.getGap()
        item={'observed_at':now(),'solver_seconds':wall,'objective':primal,
              'upper_bound':None if m.isInfinity(abs(dual)) else dual,
              'gap':None if m.isInfinity(abs(gap)) else gap,'nodes':m.getNTotalNodes()}
        trajectory.append(item)
        if progress:
            atomic(progress,{'scenario_id':data['scenario_id'],'pid':os.getpid(),'status':'running',**item})
    model.attachEventHandlerCallback(observe,[SCIP_EVENTTYPE.BESTSOLFOUND,SCIP_EVENTTYPE.NODEFOCUSED,SCIP_EVENTTYPE.LPSOLVED])
    t1=time.perf_counter()
    model.optimize()
    t2=time.perf_counter()
    sol=model.getBestSol()
    selected_out=[j['id'] for j,x in zip(jobs,xs) if model.getSolVal(sol,x)>.5]
    gap=model.getGap();dual=model.getDualbound()
    r={'method':'SCIP_MILP_gap05','scenario_id':data['scenario_id'],'selected':selected_out,
       'reported_objective':model.getSolObjVal(sol),'primal_bound':model.getPrimalbound(),
       'dual_bound':None if model.isInfinity(abs(dual)) else dual,
       'scip_gap':None if model.isInfinity(abs(gap)) else gap,'status':str(model.getStatus()),
       'optimal':str(model.getStatus())=='optimal','target_met':gap<=.05+1e-9,
       'nodes_total':model.getNTotalNodes(),'solution_count':model.getNSols(),
       'time_limit_seconds':None,'gap_target':.05,'memory_guard_MiB':6144,
       'initial_scip_objective':v['objective'],'warm_start':'previous SCIP incumbent only',
       'trajectory':trajectory,'build_seconds':t1-start,'solver_seconds':t2-t1,
       'solver_internal_seconds':model.getSolvingTime()}
    t3=time.perf_counter();r.update(total_seconds=t3-start,output_seconds=t3-t2)
    model.freeProb()
    return r


def regression():
    OUT.mkdir(exist_ok=True)
    d=load(BASE/'instance.json')
    s=next(r for r in load(BASE/'results.json')['summary'] if r['method']=='SCIP_MILP')
    r=solve(d,s['selected']);v=evaluate(d,r['selected'])
    assert r['target_met'] and r['optimal'] and v['objective']==42 and v['feasible']
    save(OUT/'runner_regression.json',{'observed_at':now(),'passed':True,'scip_actual':v['objective'],
         'target_met':r['target_met'],'time_limit_seconds':None,'gap':r['scip_gap'],
         'runner_sha256':sha(Path(__file__)),'first_solution_stop':False})
    print('Regression passed:42,gap0,no time limit',flush=True)


def run_one(sid):
    manifest=load(OUT/'input_manifest.json')
    row=next(x for x in manifest['instances'] if x['scenario_id']==sid)
    assert not row['reuse_optimal']
    data=load(ROOT/row['path']);assert sha(ROOT/row['path'])==row['sha256']
    previous=load(ROOT/row['initial_result']['path'])
    assert sha(ROOT/row['initial_result']['path'])==row['initial_result']['sha256']
    r=solve(data,previous['selected'],OUT/'progress'/f'{sid}.json')
    v=evaluate(data,r['selected']);assert v['feasible'] and abs(v['objective']-r['reported_objective'])<1e-5
    assert v['objective']>=r['initial_scip_objective']
    r.update(validation=v,input_sha256=row['sha256'],observed_at=now(),initial_result=row['initial_result'])
    atomic(OUT/'raw'/f'{sid}_SCIP.json',r)
    atomic(OUT/'progress'/f'{sid}.json',{'scenario_id':sid,'status':'complete' if r['target_met'] else 'resource_limited','pid':None,
       'objective':v['objective'],'upper_bound':r['dual_bound'],'gap':r['scip_gap'],'total_seconds':r['total_seconds'],'observed_at':now()})
    print(json.dumps({k:r[k] for k in ['scenario_id','reported_objective','scip_gap','status','total_seconds','target_met']},ensure_ascii=False),flush=True)


def run_batch():
    sys.path.insert(0,str(BRIDGE))
    from check_research_stage import require_transition
    receipt=require_transition(ROOT,f'research_quality/{PREFIX}_review_nodes.json',PREFIX)
    save(OUT/'transition_receipt.json',receipt)
    c=subprocess.run([sys.executable,'-X','utf8','-B',str(BRIDGE/'check_experiment_preflight.py'),'--root',str(ROOT),'--record',str(ROOT/f'research_quality/{PREFIX}_preflight.json'),'--next-step','public_baselines'],capture_output=True,text=True,encoding='utf-8',timeout=30)
    (OUT/'preflight_receipt.json').write_text(c.stdout,encoding='utf-8')
    assert c.returncode==0 and json.loads(c.stdout)['preflight_allowed'],c.stdout+c.stderr
    ident=load(OUT/'experiment_identity.json')
    assert ident['status']=='prepared_authorized' and ident['pid'] is None
    with (OUT/'run_claim.json').open('x',encoding='utf-8') as f:json.dump({'pid':os.getpid(),'time':now()},f)
    ident.update(status='running',pid=os.getpid(),started_at=now());atomic(OUT/'experiment_identity.json',ident)
    rows=load(OUT/'input_manifest.json')['instances']
    try:
        for row in rows:
            if row['reuse_optimal']:continue
            sid=row['scenario_id'];ident.update(current_instance=sid,last_progress_at=now());atomic(OUT/'experiment_identity.json',ident)
            print('START '+sid,flush=True)
            p=subprocess.Popen([sys.executable,'-X','utf8','-u','-B',str(Path(__file__)),'one','--scenario',sid],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8')
            ident.update(solver_pid=p.pid);atomic(OUT/'experiment_identity.json',ident)
            with (OUT/'logs'/f'{sid}.log').open('w',encoding='utf-8') as log:
                for line in p.stdout:log.write(line);log.flush();print(line,end='',flush=True)
            assert p.wait()==0, f'Solver process failed: {sid}; see log'
            r=load(OUT/'raw'/f'{sid}_SCIP.json')
            ident['completed_new']+=1
            ident['target_met_new']+=int(r['target_met'])
            ident.update(solver_pid=None,last_progress_at=now());atomic(OUT/'experiment_identity.json',ident)
        ident.update(status='complete' if ident['target_met_new']==26 else 'incomplete_resource_limits',pid=None,completed_at=now())
    except BaseException as e:
        ident.update(status='failed',pid=None,error=repr(e),traceback=traceback.format_exc())
        raise
    finally:atomic(OUT/'experiment_identity.json',ident)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['regression','batch','one']);ap.add_argument('--scenario');a=ap.parse_args()
    if a.mode=='regression':regression()
    elif a.mode=='one':run_one(a.scenario)
    else:run_batch()
