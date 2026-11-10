"""Faithful public greedy+ILS kernel through a Windows array I/O adapter."""
import argparse, ctypes, datetime, json, os, subprocess, sys, time
from pathlib import Path
from run_hypergraph_scip_demo import ROOT, OUT as BASE, BRIDGE, load, save, sha, evaluate

OUT = BASE/'原作者ILS迭代'
SOURCE = ROOT/'总结/07_证据与复现索引/超图建模与原生求解/Bmatching_author'


def verify_source():
    manifest=load(SOURCE/'source_manifest.json')
    assert all(sha(SOURCE/x['path'])==x['sha256'] for x in manifest['files'])
    return manifest['commit']


def solve(data, tries, seed=0):
    start=time.perf_counter()
    lib=ctypes.CDLL(str(OUT/'author_ils.dll'))
    ip=ctypes.POINTER(ctypes.c_int)
    lib.solve_ils.argtypes=[ctypes.c_int,ctypes.c_int,ip,ip,ip,ip,ctypes.c_int,ctypes.c_int,ip,ctypes.POINTER(ctypes.c_double),ip]
    lib.solve_ils.restype=ctypes.c_int
    ids={r['id']:i for i,r in enumerate(data['resources'])}
    ptr=[0];pins=[];weights=[]
    for j in data['jobs']:
        pins.extend(ids[r] for r in j['resources']);ptr.append(len(pins));weights.append(j['value'])
    assert all(isinstance(w,int) and 0<w<2147483647 for w in weights)
    caps=[r['capacity'] for r in data['resources']]
    p=(ctypes.c_int*len(ptr))(*ptr);v=(ctypes.c_int*len(pins))(*pins)
    w=(ctypes.c_int*len(weights))(*weights);c=(ctypes.c_int*len(caps))(*caps)
    chosen=(ctypes.c_int*len(weights))();times=(ctypes.c_double*3)();g=ctypes.c_int()
    t1=time.perf_counter()
    n=lib.solve_ils(len(caps),len(weights),p,v,w,c,seed,tries,chosen,times,ctypes.byref(g))
    t2=time.perf_counter();assert 0<=n<=len(weights)
    selected=[data['jobs'][chosen[i]]['id'] for i in range(n)]
    t3=time.perf_counter()
    validation=evaluate(data,selected)
    assert validation['feasible'] and validation['objective']>=g.value
    return {'method':'HeiHGM_Bmatching_greedy_ILS' if tries>=0 else 'HeiHGM_Bmatching_greedy',
            'scenario_id':data.get('scenario_id'),'seed':seed,'max_tries':tries,
            'max_tries_semantics':'consecutive non-improving iterations; reset on improvement; author stops when tries > max_tries',
            'time_limit_seconds':None,'selected':selected,'validation':validation,
            'greedy_initial_objective':g.value,'author_graph_build_seconds':times[0],
            'author_greedy_seconds':times[1],'author_ils_seconds':times[2],
            'native_call_seconds':t2-t1,'python_conversion_seconds':t1-start,
            'output_seconds':t3-t2,'total_seconds':t3-start,
            'observed_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'optimality_proved':False,'status':'author_stagnation_rule_complete' if tries>=0 else 'greedy_complete',
            'dll_sha256':sha(OUT/'author_ils.dll'),'adapter_sha256':sha(ROOT/'scripts/author_bmatching_ils_adapter.cpp')}


def regression():
    commit=verify_source()
    cases=[('shared', [('AB',4),('BC',8)]),('disjoint',[('AB',4),('CD',5)]),
           ('weighted_swap',[('AB',7),('AC',6),('BD',6)]),('capacity2',[('AB',7),('AC',6),('AD',5)])]
    results=[]
    for label,edges in cases:
        d={'scenario_id':'fixture_'+label,'resources':[{'id':r,'capacity':2 if label=='capacity2' and r=='A' else 1} for r in sorted(set(''.join(e[0] for e in edges)))],
           'jobs':[{'id':str(i),'resources':list(e),'value':w} for i,(e,w) in enumerate(edges)]}
        for tries in [-1,1000]:
            x=solve(d,tries);results.append(x)
    toy=load(BASE/'instance.json');results.append(solve(toy,1000))
    assert results[-1]['validation']['objective']<=42
    save(OUT/'adapter_regression.json',{'passed':True,'commit':commit,'source_files_byte_verified':148,'fixtures':results,
        'scope':'Input/output feasibility and public-kernel execution; not a replication of the full author CLI or proof of performance.'})
    print(json.dumps([{'id':x['scenario_id'],'tries':x['max_tries'],'objective':x['validation']['objective'],'seconds':x['total_seconds']} for x in results]))


def one(sid,tries):
    verify_source()
    protocol=load(OUT/'protocol.json')
    assert sid in protocol['scenario_ids'] and tries in protocol['max_tries_values']
    checked=subprocess.run([sys.executable,'-X','utf8','-B',str(BRIDGE/'check_experiment_preflight.py'),'--root',str(ROOT),'--record',str(ROOT/'research_quality/hypergraph_ils_capability_preflight.json'),'--next-step','public_baselines'],capture_output=True,text=True,encoding='utf-8',timeout=30)
    save(OUT/'preflight_receipt.json',json.loads(checked.stdout))
    assert checked.returncode==0 and json.loads(checked.stdout)['preflight_allowed'],checked.stdout+checked.stderr
    manifest=load(BASE/'SCIP_gap5持续求解/input_manifest.json')
    row=next(x for x in manifest['instances'] if x['scenario_id']==sid)
    p=ROOT/row['path'];assert sha(p)==row['sha256']
    target=OUT/'raw'/f'{sid}_tries{tries}.json'
    assert not target.exists(),'Do not duplicate completed experiment'
    claims=OUT/'claims';claims.mkdir(exist_ok=True)
    with (claims/f'{sid}_tries{tries}.json').open('x',encoding='utf-8') as f:
        json.dump({'pid':os.getpid(),'started_at':datetime.datetime.now(datetime.timezone.utc).isoformat()},f)
    r=solve(load(p),tries);r['input']=dict(path=row['path'],sha256=row['sha256'])
    target.parent.mkdir(exist_ok=True);save(target,r)
    print(json.dumps({'scenario_id':sid,'tries':tries,'objective':r['validation']['objective'],'greedy':r['greedy_initial_objective'],'seconds':r['total_seconds']}),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['regression','one']);ap.add_argument('--scenario');ap.add_argument('--tries',type=int,default=1000);a=ap.parse_args()
    if a.mode=='regression':regression()
    else:one(a.scenario,a.tries)
