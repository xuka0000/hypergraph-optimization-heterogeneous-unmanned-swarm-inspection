"""Frozen four-factor stress experiment: data, verification and common paths."""
import copy, datetime, hashlib, json, math
from collections import Counter, deque
from pathlib import Path
import numpy as np
from run_hypergraph_scip_demo import ROOT, OUT as BASE, BRIDGE, load, sha, evaluate

OUT = BASE / '四因素压力测试'
TASK = 'hypergraph_four_factor_stress'
METHODS = ['Streaming', 'Greedy', 'ILS1000', 'SCIP']


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def atomic(path, data):
    tmp = path.with_suffix(path.suffix+'.tmp')
    save(tmp, data)
    tmp.replace(path)


def ref(path):
    return {'path': path.relative_to(ROOT).as_posix(), 'sha256': sha(path)}


def metrics(data):
    jobs = data['jobs']
    rank = np.array([len(j['resources']) for j in jobs])
    degree = Counter(r for j in jobs for r in j['resources'])
    deg = np.array(list(degree.values()))
    w = np.array([j['value'] for j in jobs], dtype=float)
    return {'n': len(jobs), 'm': len(data['resources']), 'nnz': int(rank.sum()),
            'rank_mean': float(rank.mean()), 'rank_p95': float(np.quantile(rank,.95)), 'rank_max': int(rank.max()),
            'degree_mean': float(deg.mean()), 'degree_p95': float(np.quantile(deg,.95)), 'degree_max': int(deg.max()),
            'weight_min': int(w.min()), 'weight_max': int(w.max()), 'weight_mean': float(w.mean()),
            'weight_cv': float(w.std()/w.mean()), 'weight_ratio': float(w.max()/w.min()),
            'pair_conflict_incidence_upper_bound': sum(v*(v-1)//2 for v in degree.values()),
            'note': 'Pair-conflict sum counts a pair again when it shares multiple resources; not a unique edge count.'}


def verify(data):
    ids = [j['id'] for j in data['jobs']]
    resources = {r['id'] for r in data['resources']}
    assert len(ids) == len(set(ids)) and len(resources) == len(data['resources'])
    assert all(r['capacity'] == 1 for r in data['resources'])
    assert sum(j['value'] for j in data['jobs']) < 2147483647, 'author int32 objective guard'
    for j in data['jobs']:
        assert type(j['value']) is int and j['value'] > 0
        assert j['resources'] and len(j['resources']) == len(set(j['resources']))
        assert set(j['resources']) <= resources
    return metrics(data)


def regular(n, rank, degree, spread, seed):
    """r independent random partitions of jobs into resource groups of size Delta.

    Exactly n jobs, rank r, resource degree Delta; m=n*r/Delta is dependent.
    Increasing r retains prior partitions. Increasing Delta coarsens same permutations.
    """
    assert n % degree == 0
    jobs = [{'id':f'J{i}', 'resources':[], 'value':0} for i in range(n)]
    resources = []
    for layer in range(rank):
        perm = np.random.default_rng(np.random.SeedSequence([seed, layer, 7741])).permutation(n)
        for start in range(0,n,degree):
            rid = f'L{layer}G{start//degree}'
            resources.append({'id':rid,'capacity':1})
            for i in perm[start:start+degree]:
                jobs[int(i)]['resources'].append(rid)
    u = np.random.default_rng(np.random.SeedSequence([seed,9127])).permutation(n)/(n-1)
    for j, quantile in zip(jobs,u):
        j['value'] = max(1, int(round(spread ** float(quantile))))
    return {'resources':resources,'jobs':jobs}


def compact_jobs(jobs):
    used = sorted({r for j in jobs for r in j['resources']})
    return {'resources':[{'id':r,'capacity':1} for r in used],'jobs':jobs}


def prepare():
    assert not (OUT/'input_manifest.json').exists(), 'Prepared inputs already exist; inspect, do not regenerate.'
    OUT.mkdir(exist_ok=True)
    for name in ['instances','raw','progress','logs','claims','failures']:
        (OUT/name).mkdir(exist_ok=True)
    rows = []
    def put(data, sid, family, axis, value, seed, controls, provenance):
        data.update(scenario_id=sid, provenance=provenance)
        stats = verify(data)
        p = OUT/'instances'/f'{sid}.json'
        # Compact input encoding; no date-based copy of source archives.
        p.write_text(json.dumps(data,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
        rows.append(dict(scenario_id=sid,family=family,axis=axis,level=value,seed=seed,
                         controls=controls,metrics=stats,input=ref(p),provenance=provenance))
    baseline = dict(n=256,rank=4,degree=4,spread=100)
    levels = {'n':[1024,4096,16384], 'rank':[8,16,32], 'degree':[8,16,32], 'spread':[1,10,1000]}
    for seed in [0,1,2]:
        settings = [('base',None,baseline)]
        for axis, values in levels.items():
            settings += [(axis,v,dict(baseline,**{axis:v})) for v in values]
        for axis,value,parameters in settings:
            d = regular(**parameters,seed=seed)
            sid = f'reg_s{seed}_{axis}{value if value is not None else ""}'
            put(d,sid,'controlled_regular',axis,value,seed,parameters,
                {'kind':'synthetic controlled intervention','generator':'r random partitions; equal resource degree',
                 'independent_unit':'root structural seed; paired across factors',
                 'dependent_quantity':'m=n*r/Delta; it cannot be held fixed when n,r,Delta are independently varied.'})
    manifest = load(BASE/'SCIP_gap5持续求解/input_manifest.json')
    source = next(r for r in manifest['instances'] if r['scenario_id']=='paper2026_dac2012_superblue19')
    p = ROOT/source['path']
    assert sha(p)==source['sha256']
    full = load(p)
    adjacency = {}
    for i,j in enumerate(full['jobs']):
        for resource in j['resources']:
            adjacency.setdefault(resource,[]).append(i)
    anchor = min(adjacency, key=lambda r:(-len(adjacency[r]),r))
    queue = deque(adjacency[anchor]); seen = set(queue); visited_resources = set(); order=[]
    while queue and len(order)<65536:
        i=queue.popleft();order.append(i)
        for resource in full['jobs'][i]['resources']:
            if resource in visited_resources:continue
            visited_resources.add(resource)
            for k in adjacency[resource]:
                if k not in seen:seen.add(k);queue.append(k)
    assert len(order)==65536
    rmax=max(len(j['resources']) for j in full['jobs'])
    provenance={'kind':'public-topology-derived exploratory intervention','source':ref(p),
                'original_hmetis':{'path':source['source_path'],'sha256':source['source_sha256']},
                'selection':'BFS from highest-degree resource, lexical tie break, source edge order for solver input',
                'anchor_resource':anchor,'anchor_degree':len(adjacency[anchor]),
                'independent_unit':'one source topology; two weights and nested subsets are not independent datasets',
                'original_full_cases':['paper2026_dac2012_superblue19','paper2025_dac2012_superblue19']}
    for regime in ['random100','LHG']:
        def subset(n):
            js = copy.deepcopy([full['jobs'][i] for i in sorted(order[:n])])
            if regime=='LHG':
                for j in js:j['value']=rmax+1-len(j['resources'])
            return js
        cases=[('n',v) for v in [4096,16384,65536]]
        cases += [('rank_cap',v) for v in [8,32]]
        cases += [('degree_cap',v) for v in [32,128]]
        cases += [('weight_spread',v) for v in [1,1000]]
        cases += [('scale_control',10)]
        for axis,value in cases:
            jobs=subset(value if axis=='n' else 16384)
            if axis=='rank_cap':
                for j in jobs:
                    # Nested, deterministic pin priorities independent of observed solver performance.
                    priority=sorted(j['resources'],key=lambda r:hashlib.sha256((j['id']+'|'+r).encode()).digest())
                    j['resources']=sorted(priority[:value])
            elif axis=='degree_cap':
                used=Counter()
                for j in jobs:
                    new=[]
                    for r in j['resources']:
                        new.append(r+'__split'+str(used[r]//value));used[r]+=1
                    j['resources']=new
            elif axis=='weight_spread':
                unique=sorted({j['value'] for j in jobs});quantile={w:i/max(1,len(unique)-1) for i,w in enumerate(unique)}
                for j in jobs:j['value']=max(1,int(round(value**quantile[j['value']])))
            elif axis=='scale_control':
                for j in jobs:j['value']*=value
            sid=f'sb_{regime}_{axis}{value}'
            notes=dict(provenance,weight_regime=regime,intervention=axis,
                       warning='Derived intervention, not unchanged public benchmark. Rank clipping also changes degree; degree splitting relaxes conflict constraints. Actual shifts recorded.')
            put(compact_jobs(jobs),sid,'public_'+regime,axis,value,0,
                {'n_requested':value if axis=='n' else 16384,'single_intervention':axis,'level':value},notes)
    assert len(rows)==59
    # Start with lower scales, preserve breadth across axes and seeds in the fixed queue.
    rows.sort(key=lambda r:(r['metrics']['n'],r['seed'],r['family'],r['axis'],str(r['level'])))
    save(OUT/'input_manifest.json',{'observed_at':now(),'instances':rows,'cases':59,
         'controlled_cases':39,'public_derived_cases':18,'objective_scale_controls':2,
         'fixed_methods':METHODS,'planned_solver_runs':59*4})
    save(OUT/'input_validation.json',{'observed_at':now(),'passed':True,'cases_checked':59,
         'controlled_rank_degree_exact':all(r['metrics']['rank_max']==r['controls']['rank'] and r['metrics']['degree_max']==r['controls']['degree'] and r['metrics']['nnz']==r['controls']['n']*r['controls']['rank'] for r in rows if r['family']=='controlled_regular'),
         'schema_integer_capacity_and_native_range_checked':True,'public_source_hash':source['sha256'],
         'manifest':ref(OUT/'input_manifest.json'),'input_bytes':sum((ROOT/r['input']['path']).stat().st_size for r in rows),
         'n_r_degree_not_synonyms':True,'neural_feature_d_not_used':True})
    print(json.dumps({'cases':len(rows),'bytes':sum((ROOT/r['input']['path']).stat().st_size for r in rows),'public_anchor':anchor},ensure_ascii=False))


if __name__=='__main__':
    prepare()
