"""Bounded SCIP versus byte-preserved public hypergraph solver demonstration."""
from pathlib import Path
import argparse, ctypes, datetime, hashlib, itertools, json, os, platform
import statistics, subprocess, sys, time, tempfile, shutil

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'总结/03_实验数据/超图与SCIP组合任务对比'
BRIDGE=Path('D:/博士研究生/具身智能大论文架构/cross_work_bridge/scripts')

def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,obj):p.write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf-8')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def evaluate(instance, selected):
    """Independent verifier: sums the original JSON; never repairs a solution."""
    byid={j['id']:j for j in instance['jobs']}
    assert len(set(selected))==len(selected), 'duplicate selection'
    assert set(selected)<=set(byid), 'unknown job'
    used={r['id']:0 for r in instance['resources']}
    total=0
    for k in selected:
        j=byid[k];total+=j['value']
        for resource in j['resources']:used[resource]+=1
    return {'feasible':all(used[r['id']]<=r['capacity'] for r in instance['resources']),
            'objective':total,'resource_usage':used}

def enumerate_reference(instance):
    n=len(instance['jobs']); best=-1; solutions=[];feasible=0
    for bits in itertools.product((0,1),repeat=n):
        chosen=[j['id'] for j,b in zip(instance['jobs'],bits) if b]
        v=evaluate(instance,chosen)
        if not v['feasible']:continue
        feasible+=1
        if v['objective']>best:best=v['objective'];solutions=[chosen]
        elif v['objective']==best:solutions.append(chosen)
    return {'objective':best,'optimal_selections':solutions,'subsets_checked':2**n,
            'feasible_subsets':feasible,'method':'all subsets, original-JSON capacity sums, no SCIP/native solver calls'}

def native_library():
    lib=ctypes.CDLL(str(OUT/'author_streaming.dll'))
    intp=ctypes.POINTER(ctypes.c_int)
    lib.solve.argtypes=[ctypes.c_int,ctypes.c_int,intp,intp,intp,ctypes.c_float,intp,ctypes.POINTER(ctypes.c_double)]
    lib.solve.restype=ctypes.c_int
    return lib

def native_solve(instance,lib,eps):
    t0=time.perf_counter_ns()
    index={r['id']:i for i,r in enumerate(instance['resources'])}
    ptr=[0];pins=[];weights=[]
    for job in instance['jobs']:
        pins.extend(index[r] for r in job['resources']);ptr.append(len(pins));weights.append(job['value'])
    p=(ctypes.c_int*len(ptr))(*ptr);v=(ctypes.c_int*len(pins))(*pins);w=(ctypes.c_int*len(weights))(*weights)
    selected=(ctypes.c_int*len(weights))();elapsed=ctypes.c_double()
    t1=time.perf_counter_ns()
    count=lib.solve(len(index),len(weights),p,v,w,eps,selected,ctypes.byref(elapsed))
    t2=time.perf_counter_ns()
    assert 0<=count<=len(weights)
    result=[instance['jobs'][selected[i]]['id'] for i in range(count)]
    t3=time.perf_counter_ns()
    return {'method':'HeiHGM_Streaming','selected':sorted(result),'status':'approximate_one_pass_complete',
            'solver_seconds':elapsed.value,'native_call_seconds':(t2-t1)/1e9,
            'total_seconds':(t3-t0)/1e9,'build_seconds':(t1-t0)/1e9,'output_seconds':(t3-t2)/1e9,
            'solver_optimality_certificate':False,'solver_primal_bound':None,'solver_dual_bound':None}

def scip_solve(instance,diagnostic=False):
    from pyscipopt import Model,quicksum
    t0=time.perf_counter_ns()
    model=Model('bundle_allocation_12');model.hideOutput()
    model.setRealParam('limits/time',10.0)
    model.setIntParam('parallel/maxnthreads',1)
    model.setIntParam('randomization/randomseedshift',0)
    if diagnostic:model.setLogfile('scip.log')
    x={j['id']:model.addVar(name=j['id'],vtype='B') for j in instance['jobs']}
    for r in instance['resources']:
        model.addCons(quicksum(x[j['id']] for j in instance['jobs'] if r['id'] in j['resources'])<=r['capacity'],name='capacity_'+r['id'])
    model.setObjective(quicksum(j['value']*x[j['id']] for j in instance['jobs']),'maximize')
    t1=time.perf_counter_ns();model.optimize();t2=time.perf_counter_ns()
    sol=model.getBestSol()
    selected=[j['id'] for j in instance['jobs'] if model.getSolVal(sol,x[j['id']])>0.5] if sol else []
    result={'method':'SCIP_MILP','selected':selected,'status':str(model.getStatus()),
            'solver_optimality_certificate':str(model.getStatus())=='optimal',
            'solver_primal_bound':model.getPrimalbound(),'solver_dual_bound':model.getDualbound(),
            'solver_gap':model.getGap(),'solver_nodes':model.getNNodes(),
            'solver_internal_seconds':model.getSolvingTime()}
    t3=time.perf_counter_ns()
    result.update(solver_seconds=(t2-t1)/1e9,total_seconds=(t3-t0)/1e9,
                  build_seconds=(t1-t0)/1e9,output_seconds=(t3-t2)/1e9)
    if diagnostic:
        # PySCIPOpt absolutizes even relative paths before invoking C fopen.
        # Only exports stage in a verified ASCII temporary directory on D:.
        staging_root=Path('D:/temp').resolve()
        with tempfile.TemporaryDirectory(prefix='paper11-scip-',dir=staging_root) as staging:
            staging_path=Path(staging).resolve()
            assert staging_path.is_relative_to(staging_root)
            model.writeProblem(str(staging_path/'model.lp'))
            model.writeBestSol(str(staging_path/'scip_solution.sol'))
            model.writeParams(str(staging_path/'scip_parameters.set'),comments=True,onlychanged=True)
            for name in ['model.lp','scip_solution.sol','scip_parameters.set']:
                target=(OUT/name).resolve();assert target.is_relative_to(OUT.resolve())
                shutil.copyfile(staging_path/name,target)
    model.freeProb()
    return result

def validate():
    data=load(OUT/'instance.json');p=load(OUT/'protocol.json')
    assert sha(OUT/'instance.json')==p['instance_sha256']
    assert len(data['resources'])==12 and len(data['jobs'])==12
    assert all(r['capacity']==1 for r in data['resources'])
    assert all(isinstance(j['value'],int) and j['value']>0 and len(set(j['resources']))==len(j['resources']) for j in data['jobs'])
    ref=enumerate_reference(data)
    assert ref['objective']==42 and ref['optimal_selections']==[['T02','T03','T04','T12']]
    # Hand-checked fixtures verify public-core input/output adaptation, not tuning.
    fixtures=[('disjoint',[('AB',4),('CD',5)],['F0','F1']),
              ('shared_resource',[('AB',4),('BC',8)],['F1']),
              ('all_or_nothing',[('ABC',7),('AD',3)],['F0'])]
    lib=native_library();checks=[]
    for label,edges,expected in fixtures:
        letters=sorted(set(''.join(e[0] for e in edges)))
        d={'resources':[{'id':x,'capacity':1} for x in letters],
           'jobs':[{'id':'F'+str(i),'resources':list(v),'value':w} for i,(v,w) in enumerate(edges)]}
        r=native_solve(d,lib,p['hypergraph']['eps']);ev=evaluate(d,r['selected'])
        assert r['selected']==expected and ev['feasible']
        checks.append({'id':label,'expected':expected,'actual':r['selected'],'pass':True})
    audit=load(OUT/'adapter_audit.json');assert audit['stream_body_verbatim'] and audit['transform_body_verbatim']
    save(OUT/'benchmark_validation.json',{'observed_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
         'input_sha256':sha(OUT/'instance.json'),'units_parameters_pass':True,
         'hand_expected_optimum':42,'enumerated_reference':ref,'known_solution_pass':True,'adapter_fixture_checks':checks,
         'scenario_count':1,'reference_not_shared_with_solvers':True})
    print(json.dumps(ref,ensure_ascii=False))

def run():
    if (OUT/'results.json').exists():raise SystemExit('Results already exist; no duplicate run.')
    identity=load(OUT/'experiment_identity.json')
    assert identity['status']=='preparing'
    p=load(OUT/'protocol.json');data=load(OUT/'instance.json')
    assert sha(OUT/'instance.json')==p['instance_sha256']
    # This bound checker executes at the actual entry, not as a post-hoc report.
    preflight=ROOT/'research_quality/hypergraph_scip_demo_preflight.json'
    command=[sys.executable,'-X','utf8','-B',str(BRIDGE/'check_experiment_preflight.py'),'--root',str(ROOT),'--record',str(preflight),'--next-step','public_baselines']
    checked=subprocess.run(command,capture_output=True,text=True,encoding='utf-8',timeout=30)
    (OUT/'preflight_stdout.json').write_text(checked.stdout,encoding='utf-8')
    if checked.returncode or not json.loads(checked.stdout).get('preflight_allowed'):
        raise RuntimeError('Preflight rejected: '+checked.stdout+checked.stderr)
    identity.update(status='running',pid=os.getpid(),observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat())
    save(OUT/'experiment_identity.json',identity)
    import pyscipopt
    os.chdir(OUT)
    lib=native_library();rows=[];start=time.monotonic()
    try:
        for phase,count in [('quality',1),('timing',p['run_plan']['timing_repeats'])]:
            for repeat in range(count):
                order=['SCIP_MILP','HeiHGM_Streaming'] if repeat%2==0 else ['HeiHGM_Streaming','SCIP_MILP']
                for method in order:
                    assert time.monotonic()-start<300,'total solver budget'
                    r=scip_solve(data,phase=='quality') if method=='SCIP_MILP' else native_solve(data,lib,p['hypergraph']['eps'])
                    r.update(phase=phase,repeat=repeat,input_sha256=p['instance_sha256'],validation=evaluate(data,r['selected']))
                    assert r['validation']['feasible'],'infeasible output retained as failure'
                    rows.append(r);save(OUT/'raw_runs.json',rows)
        ref=load(OUT/'benchmark_validation.json')['enumerated_reference']
        summary=[]
        for method in ['SCIP_MILP','HeiHGM_Streaming']:
            q=next(r for r in rows if r['phase']=='quality' and r['method']==method)
            timed=[r for r in rows if r['phase']=='timing' and r['method']==method]
            summary.append({'method':method,'selected':q['selected'],'objective':q['validation']['objective'],
                'feasible':q['validation']['feasible'],'relative_gap_to_proved_optimum':(ref['objective']-q['validation']['objective'])/ref['objective'],
                'solver_certificate':q['solver_optimality_certificate'],'status':q['status'],
                'timing_repeats':len(timed),'objectives_all_repeats':sorted(set(r['validation']['objective'] for r in timed)),
                'solver_median_ms':1000*statistics.median(r['solver_seconds'] for r in timed),
                'total_median_ms':1000*statistics.median(r['total_seconds'] for r in timed),
                'total_min_ms':1000*min(r['total_seconds'] for r in timed),
                'total_max_ms':1000*max(r['total_seconds'] for r in timed)})
        result={'observed_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'experiment_id':p['experiment_id'],
            'instance_sha256':p['instance_sha256'],'scenario_count':1,'jobs':12,'resources':12,'r_max':6,'r_mean':3.5,
            'binary_variables':12,'capacity_constraints':12,'incidences':42,'reference':ref,'summary':summary,
            'environment':{'platform':platform.platform(),'processor':platform.processor(),'python':sys.version,'pyscipopt':pyscipopt.__version__,'scip':'10.0.2','soplex':'8.0.2','logical_cpus':os.cpu_count()},
            'algorithm_kernel_hash':sha(OUT/'author_streaming.dll'),'runner_hash':sha(Path(__file__)),
            'timing_interpretation':'Warm process and loaded libraries; new instance state/model every time. Same physical machine and single-thread solve. Native C++ vs Python model builder is part of interface cost. Repetitions are timing observations of one instance, not scientific replicates.',
            'limits':'One constructed example; no general superiority, significance, memory or scaling claim. Full author CLI was not executed; exact author kernels were adapted for Windows I/O.'}
        save(OUT/'results.json',result)
        identity.update(status='complete',pid=None,observed_at=result['observed_at'],results='results.json')
        save(OUT/'experiment_identity.json',identity)
        print(json.dumps(summary,ensure_ascii=False,indent=2))
    except BaseException as e:
        save(OUT/('failure_'+identity.get('attempt_id','initial')+'.json'),{'error':repr(e),'rows_completed':len(rows),'observed_at':datetime.datetime.now(datetime.timezone.utc).isoformat()})
        identity.update(status='failed',pid=None);save(OUT/'experiment_identity.json',identity)
        raise

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['validate','run']);args=parser.parse_args()
    validate() if args.mode=='validate' else run()
