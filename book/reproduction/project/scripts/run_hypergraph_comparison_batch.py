"""Sequential finite-budget comparison using the existing public native core."""
from pathlib import Path
import argparse, datetime, hashlib, json, os, sys, time, traceback
from run_hypergraph_scip_demo import ROOT, BRIDGE, evaluate, native_library, native_solve, load, save, sha

def scip_solve(data,seconds=10):
    from pyscipopt import Model,quicksum
    t0=time.perf_counter_ns();model=Model(data['scenario_id']);model.hideOutput()
    model.setRealParam('limits/time',float(seconds));model.setIntParam('parallel/maxnthreads',1)
    model.setIntParam('randomization/randomseedshift',0)
    jobs=data['jobs'];xs=[model.addVar(name=j['id'],vtype='B') for j in jobs]
    incident={r['id']:[] for r in data['resources']}
    for i,j in enumerate(jobs):
        for r in j['resources']:incident[r].append(i)
    for r in data['resources']:
        if incident[r['id']]:model.addCons(quicksum(xs[i] for i in incident[r['id']])<=r['capacity'],name=r['id'])
    model.setObjective(quicksum(j['value']*x for j,x in zip(jobs,xs)),'maximize')
    t1=time.perf_counter_ns();model.optimize();t2=time.perf_counter_ns()
    sol=model.getBestSol();assert sol is not None,'No feasible incumbent'
    chosen=[j['id'] for j,x in zip(jobs,xs) if model.getSolVal(sol,x)>0.5]
    r={'method':'SCIP_MILP','selected':chosen,'status':str(model.getStatus()),'solver_seconds':(t2-t1)/1e9,
       'primal_bound':model.getPrimalbound(),'dual_bound':model.getDualbound(),'scip_gap':model.getGap(),
       'nodes_total':model.getNTotalNodes(),'optimal':str(model.getStatus())=='optimal','solution_count':model.getNSols(),
       'reported_objective':model.getSolObjVal(sol),'original_variables':len(xs),'original_nonempty_constraints':sum(bool(v) for v in incident.values())}
    t3=time.perf_counter_ns();r.update(build_seconds=(t1-t0)/1e9,total_seconds=(t3-t0)/1e9,output_seconds=(t3-t2)/1e9)
    model.freeProb();return r

def regression(out):
    original=load(ROOT/'总结/03_实验数据/超图与SCIP组合任务对比/instance.json')
    r=scip_solve(original);v=evaluate(original,r['selected'])
    assert r['optimal'] and v['feasible'] and v['objective']==42 and r['dual_bound']==42
    lib=native_library();n=native_solve(original,lib,0.1);nv=evaluate(original,n['selected'])
    assert nv['feasible'] and nv['objective']==39
    save(out/'runner_regression.json',{'observed_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'purpose':'Sparse model interface regression on known original example; not a new research instance or timing replicate',
        'scip_expected':42,'scip_actual':v['objective'],'native_expected':39,'native_actual':nv['objective'],'passed':True,'code_sha256':sha(Path(__file__))})

def run(out,review_record,target,preflight):
    import subprocess
    sys.path.insert(0,str(BRIDGE));from check_research_stage import require_transition
    ident=load(out/'experiment_identity.json');assert ident['pid'] is None
    assert ident['status'] in {'prepared_pending_scope_review','prepared_authorized'}
    assert not (out/'results.json').exists() and not list((out/'raw').glob('*.json')) if (out/'raw').exists() else not (out/'results.json').exists()
    manifest=load(out/'input_manifest.json');protocol=load(out/'protocol.json');rows=manifest['instances']
    assert len(rows)==protocol['instances']
    expected_binary=protocol['hypergraph']['binary_sha256']
    assert sha(ROOT/'总结/03_实验数据/超图与SCIP组合任务对比/author_streaming.dll')==expected_binary
    for row in rows:assert sha(ROOT/row['path'])==row['sha256']
    receipt=require_transition(ROOT,review_record,target);save(out/'transition_receipt.json',receipt)
    c=subprocess.run([sys.executable,'-X','utf8','-B',str(BRIDGE/'check_experiment_preflight.py'),'--root',str(ROOT),'--record',str(ROOT/preflight),'--next-step','public_baselines'],capture_output=True,text=True,encoding='utf-8',timeout=30)
    (out/'preflight_receipt.json').write_text(c.stdout,encoding='utf-8')
    assert c.returncode==0 and json.loads(c.stdout)['preflight_allowed'],c.stdout+c.stderr
    (out/'raw').mkdir(exist_ok=True);ident.update(status='running',pid=os.getpid(),started_at=datetime.datetime.now(datetime.timezone.utc).isoformat())
    save(out/'experiment_identity.json',ident);lib=native_library();started=time.monotonic();results=[]
    try:
        for pos,row in enumerate(rows):
            if time.monotonic()-started>protocol['budget']['batch_wall_guard_seconds']-12:raise TimeoutError('Batch guard nearing limit')
            data=load(ROOT/row['path']);assert all(r['capacity']==1 for r in data['resources']), 'Native matching requires b=1'
            pair=[]
            for method in (['SCIP_MILP','HeiHGM_Streaming'] if pos%2==0 else ['HeiHGM_Streaming','SCIP_MILP']):
                t=datetime.datetime.now(datetime.timezone.utc).isoformat()
                r=scip_solve(data,protocol['scip']['seconds_per_instance']) if method=='SCIP_MILP' else native_solve(data,lib,protocol['hypergraph']['epsilon'])
                v=evaluate(data,r['selected']);r.update(scenario_id=row['scenario_id'],input_sha256=row['sha256'],observed_at=t,validation=v)
                save(out/'raw'/(row['scenario_id']+'_'+method+'.json'),r)
                assert v['feasible']
                if method=='SCIP_MILP':assert abs(v['objective']-r['reported_objective'])<1e-5
                pair.append(r)
            a=next(r for r in pair if r['method']=='SCIP_MILP');b=next(r for r in pair if r['method']=='HeiHGM_Streaming')
            sv=a['validation']['objective'];hv=b['validation']['objective'];ub=a['dual_bound'];lb=max(sv,hv)
            assert ub+1e-5>=lb
            summary={'scenario_id':row['scenario_id'],'n':len(data['jobs']),'resource_count':len(data['resources']),
                     'r_max':max(len(j['resources']) for j in data['jobs']),'seed':row.get('seed'),
                     'scip_value':sv,'hypergraph_value':hv,'scip_status':a['status'],'scip_proved_optimal':a['optimal'],
                     'scip_upper_bound':ub,'hypergraph_relative_difference_to_scip':(hv-sv)/sv if sv else None,
                     'hypergraph_true_loss_lower_bound':(lb-hv)/lb if lb else 0,
                     'hypergraph_true_loss_upper_bound':(ub-hv)/ub if ub else 0,
                     'scip_seconds':a['total_seconds'],'hypergraph_seconds':b['total_seconds'],
                     'scip_solver_seconds':a['solver_seconds'],'hypergraph_solver_seconds':b['solver_seconds'],
                     'all_feasible':True,'input_sha256':row['sha256']}
            results.append(summary);save(out/'partial_results.json',results)
            ident.update(completed_cells=len(results)*2,last_completed=row['scenario_id']);save(out/'experiment_identity.json',ident)
            print(json.dumps(summary,ensure_ascii=False),flush=True)
        save(out/'results.json',{'experiment_id':protocol['experiment_id'],'observed_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
             'instances':len(results),'methods':2,'rows':results,'protocol_sha256':sha(out/'protocol.json'),'runner_sha256':sha(Path(__file__)),
             'elapsed_batch_seconds':time.monotonic()-started,'note':'Time-limit outcomes are valid incumbents, not proven optima; comparisons describe this fixed public one-pass method.'})
        ident.update(status='complete',pid=None,completed_at=datetime.datetime.now(datetime.timezone.utc).isoformat());save(out/'experiment_identity.json',ident)
    except BaseException as e:
        save(out/'failure.json',{'error':repr(e),'traceback':traceback.format_exc(),'completed_instances':len(results),'observed_at':datetime.datetime.now(datetime.timezone.utc).isoformat()})
        ident.update(status='failed',pid=None);save(out/'experiment_identity.json',ident);raise

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['regression','run']);ap.add_argument('--directory',required=True);ap.add_argument('--review-record');ap.add_argument('--transition');ap.add_argument('--preflight');a=ap.parse_args();out=(ROOT/a.directory).resolve();assert out.is_relative_to(ROOT)
    if a.mode=='regression':regression(out)
    else:run(out,a.review_record,a.transition,a.preflight)
