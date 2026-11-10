"""Read-only reanalysis of existing instances/results; no optimization calls."""
from collections import Counter
from pathlib import Path
import datetime
import gc
import hashlib
import json
import math
import numpy as np
import psutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'总结/03_实验数据/超图与SCIP组合任务对比'
OUT=ROOT/'research_quality/hypergraph_cross_scenario_comparison.json'


def read(p):
    return json.loads(Path(p).read_text(encoding='utf-8-sig'))


def binding(p):
    p=Path(p)
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return dict(path=p.relative_to(ROOT).as_posix(),sha256=h.hexdigest())


def distribution(a):
    a=np.asarray(a)
    return dict(min=int(a.min()),mean=float(a.mean()),median=float(np.median(a)),
                p95=float(np.quantile(a,.95)),max=int(a.max()),
                histogram=dict(sorted(Counter(map(int,a)).items())))


def metrics(data):
    jobs=data['jobs']; resources=data['resources']; n=len(jobs); m=len(resources)
    index={r['id']:i for i,r in enumerate(resources)}
    assert len(index)==m
    degree=np.zeros(m,dtype=np.int64); first=np.full(m,-1,dtype=np.int64)
    parent=np.arange(n,dtype=np.int64); size=np.ones(n,dtype=np.int64)
    rank=np.empty(n,dtype=np.int64); weights=np.empty(n,dtype=np.float64)
    signatures=set()
    def find(a):
        while parent[a]!=a:
            parent[a]=parent[parent[a]];a=parent[a]
        return a
    for i,j in enumerate(jobs):
        rr=j['resources']; assert len(set(rr))==len(rr)
        rank[i]=len(rr);weights[i]=j['value'];signatures.add(tuple(sorted(rr)))
        for rid in rr:
            k=index[rid];degree[k]+=1
            if first[k]<0:first[k]=i
            else:
                a,b=find(i),find(first[k])
                if a!=b:
                    if size[a]<size[b]:a,b=b,a
                    parent[b]=a;size[a]+=size[b]
    components=Counter(find(i) for i in range(n))
    capacities=np.asarray([r['capacity'] for r in resources])
    nnz=int(rank.sum());assert nnz==int(degree.sum())
    return dict(n=n,m=m,m_nonempty=int(np.count_nonzero(degree)),nnz=nnz,
        r=distribution(rank),Delta=distribution(degree),
        incidence_density=nnz/(n*m),
        capacity_min=float(capacities.min()),capacity_max=float(capacities.max()),
        positive_unit_capacity=bool(np.all(capacities==1)),
        weight_min=float(weights.min()),weight_max=float(weights.max()),
        weight_ratio=float(weights.max()/weights.min()),weight_cv=float(weights.std()/weights.mean()),
        duplicate_membership_columns=n-len(signatures),
        decision_components=len(components),largest_component_variables=max(components.values()),
        largest_component_fraction=max(components.values())/n,
        pair_conflict_incidence_upper_bound=sum(int(v)*(int(v)-1)//2 for v in degree),
        pair_conflict_note='Repeated pairs counted again; not number of unique conflict edges.',
        binary_variables=n,continuous_variables=0,
        treewidth=None,irreducible_high_order_proved=False,
        all_capacity_scopes_pairwise_representable=bool(np.all(capacities==1)))


def measure(path,expected):
    ref=binding(path);assert ref['sha256']==expected,(path,'input hash mismatch')
    data=read(path);stat=metrics(data);del data;gc.collect()
    return stat,ref


def difference(a,b):
    return dict(absolute=a-b,percent=100*(a-b)/abs(b) if b else None)


def compact_method(x):
    return {k:x.get(k) for k in ['objective','total_seconds','solver_seconds','status','gap','upper_bound',
        'complete','independently_checked','evidence','validation','process_accounting','build_seconds','output_seconds']}


def main():
    now=datetime.datetime.now(datetime.timezone.utc).isoformat()
    base=read(BASE/'全部案例求解结果与时间.json')
    pipeline=read(BASE/'论文完整流程原始场景对比/results.json')
    stress=read(BASE/'四因素压力测试/results.json')
    iterations=read(BASE/'原作者ILS迭代/results.json')
    source_paths=[BASE/'全部案例求解结果与时间.json',BASE/'论文完整流程原始场景对比/results.json',
        BASE/'四因素压力测试/results.json',BASE/'原作者ILS迭代/results.json',
        BASE/'算法论文出版信息.json',ROOT/'research_quality/inspection_engineering_complexity.json',
        ROOT/'Agent/Publication/engineering_inspection_contract.json',Path(__file__).resolve()]
    manifest={}
    for sub in ['复杂度扩展','论文原始实例_HypOp2025','论文原始实例_匹配2025_2026']:
        mp=BASE/sub/'input_manifest.json';source_paths.append(mp)
        manifest.update({r['scenario_id']:r for r in read(mp)['instances']})
    manifest['bundle_allocation_12']=dict(path=(BASE/'instance.json').relative_to(ROOT).as_posix(),
        sha256=base['rows'][0]['input_sha256'])
    pm={r['scenario_id']:r for r in pipeline['rows']}
    im={r['scenario_id']:r for r in iterations['rows']}
    rows=[]
    for i,row in enumerate(base['rows'],1):
        sid=row['scenario_id'];meta=manifest[sid]
        stat,ref=measure(ROOT/meta['path'],meta['sha256'])
        assert (stat['n'],stat['m'],stat['r']['max'])==(row['variables'],row['resources'],row['max_rank'])
        # Preserve every old result; prefer separately verified completed later runs.
        previous=row['scip']
        scip=dict(objective=previous['objective'],total_seconds=previous['total_seconds'],
            status=previous['status'],time_basis='historical total build+solve+extract',
            gap=0.0 if previous.get('optimality_proved') else
                ((previous['upper_bound']-previous['objective'])/abs(previous['objective'])
                 if previous.get('upper_bound') is not None and previous['objective'] else None),
            upper_bound=previous.get('upper_bound'),
            evidence=previous['evidence'],independently_checked=previous.get('feasible'),
            stop_label=previous.get('stop_label'),warm_start='see original protocol',live=False)
        raw_path=BASE/'SCIP_gap5持续求解/raw'/f'{sid}_SCIP.json'
        if raw_path.exists():
            raw=read(raw_path);assert raw['input_sha256']==meta['sha256']
            assert raw['validation']['feasible']
            scip=dict(objective=raw['reported_objective'],total_seconds=raw['total_seconds'],
                status=raw['status'],gap=raw['scip_gap'],upper_bound=raw['dual_bound'],
                evidence=binding(raw_path),independently_checked=True,live=False,
                time_basis='later warm-started run total; previous run cost excluded',
                warm_start=raw['warm_start'],stop_label='gap<=5%; no time cutoff')
        if sid in pm:
            p=pm[sid]
            scip.update(objective=p['scip_objective'],total_seconds=p['scip_total_seconds'],
                status=p['scip_status'],gap=p['scip_gap'],live=False,independently_checked=True,
                evidence=binding(BASE/'论文完整流程原始场景对比/results.json'),
                time_basis='complete-pipeline comparison table total; first six reused historical optimum',
                warm_start='empty only on newly run superblue19; first six historical',
                stop_label='optimal or gap<=5%; no time cutoff on new large-instance run')
        live_path=BASE/'SCIP_gap5持续求解/progress'/f'{sid}.json'
        live=None
        if live_path.exists():
            progress=read(live_path)
            if progress.get('status')=='running':
                pid=progress.get('pid');alive=bool(pid and psutil.pid_exists(pid))
                identity={}
                if alive:
                    proc=psutil.Process(pid)
                    identity=dict(pid=pid,name=proc.name(),create_time=proc.create_time(),
                        matches_known_experiment=pid==40284 and abs(proc.create_time()-1794046886.26545)<.1,
                        process_status=proc.status(),cpu_seconds=sum(proc.cpu_times()[:2]))
                live=dict(progress=progress,observed_at=now,process=identity,process_exists=alive,
                    selected_solution_independently_checked=False,
                    evidence=binding(live_path),not_a_completed_result=True)
        stream=row['hypergraph']
        item=dict(id=f'C{i:02}',scenario_id=sid,label=row['label'],family=row['group'],paper_id=row['paper_id'],
            metrics=stat,input=ref,scip_latest_completed=scip,scip_historical=previous,
            streaming=stream,streaming_minus_completed_scip=difference(stream['objective'],scip['objective']),
            live_snapshot=live,author_full_pipeline=pm.get(sid),author_iteration_levels=im.get(sid))
        rows.append(item)
        print(f'Base {i}/33 {sid}',flush=True)
    stress_rows=[]
    for i,row in enumerate(stress['rows'],1):
        stat,ref=measure(ROOT/row['input']['path'],row['input']['sha256'])
        old=row['metrics']
        assert (stat['n'],stat['m'],stat['nnz'],stat['r']['max'],stat['Delta']['max'])==(
            old['n'],old['m'],old['nnz'],old['rank_max'],old['degree_max'])
        assert abs(stat['r']['mean']-old['rank_mean'])<1e-10
        methods={k:compact_method(v) for k,v in row['methods'].items()}
        # Existing independent validation reused only with exact bound raw files.
        for name,m in methods.items():
            ev=m['evidence'];assert binding(ROOT/ev['path'])['sha256']==ev['sha256']
            assert m['validation']['feasible'] and m['independently_checked']
        scip_raw=read(ROOT/methods['SCIP']['evidence']['path'])
        latency={}
        for horizon in [3,5]:
            if methods['SCIP']['total_seconds']<=horizon:
                incumbent=methods['SCIP']['objective'];basis='completed independently verified output'
            else:
                events=[e for e in scip_raw['trajectory'] if e.get('total_seconds',float('inf'))<=horizon
                        and e.get('objective') is not None]
                incumbent=max((e['objective'] for e in events),default=None)
                basis='best causal solver-reported incumbent recorded by deadline; intermediate selected set not independently checked'
            latency[str(horizon)]=dict(scip_recorded_objective=incumbent,scip_basis=basis,
                native_completed_objectives={k:v['objective'] if v['total_seconds']<=horizon else None
                    for k,v in methods.items() if k!='SCIP'},
                missing_native='Not finished by deadline; intermediate quality unrecorded, not zero')
        stress_rows.append(dict(id=f'S{i:02}',scenario_id=row['scenario_id'],family=row['family'],
            axis=row['axis'],level=row['level'],seed=row['seed'],metrics=stat,input=ref,methods=methods,
            differences={k:difference(v['objective'],methods['SCIP']['objective']) for k,v in methods.items() if k!='SCIP'},
            provenance=row['provenance'],latency_observations=latency))
    # A true structural view uses ranges over the actual conditions, never a fictitious joint maximum.
    groups={}
    for family,items in [('toy',[r for r in rows if r['family']=='toy']),
        ('initial_synthetic',[r for r in rows if r['paper_id'] is None and r['family']!='toy']),
        ('HypOp2025_MIS',[r for r in rows if r['paper_id']=='P1']),
        ('JGAA2026_original',[r for r in rows if r['paper_id']=='P2']),
        ('ESA2025_original',[r for r in rows if r['paper_id']=='P3']),
        ('controlled_stress',[r for r in stress_rows if r['family']=='controlled_regular']),
        ('public_derived_stress',[r for r in stress_rows if r['family']!='controlled_regular'])]:
        assert items,family
        summary={'conditions':len(items),'scenario_ids':[r['scenario_id'] for r in items]}
        for key in ['n','m','nnz','weight_ratio','weight_cv','decision_components','largest_component_fraction']:
            vals=[r['metrics'][key] for r in items];summary[key]={'min':min(vals),'max':max(vals)}
        for key in ['r','Delta']:
            summary[key]={stat:{'min':min(r['metrics'][key][stat] for r in items),
                                'max':max(r['metrics'][key][stat] for r in items)}
                          for stat in ['mean','p95','max']}
        groups[family]=summary
    original=read(ROOT/'research_quality/inspection_engineering_complexity.json')
    contract=read(ROOT/'Agent/Publication/engineering_inspection_contract.json')
    paper=read(BASE/'算法论文出版信息.json')
    # Publication dates reused, updated use description prevents stale pre-pipeline claims.
    papers=paper['papers']
    papers['P2']['use']='7 original topologies/random100 conditions; full reductions+S,pin greedy+ILS15+unfold now tested with10 seeds;5 failures retained'
    methods=[dict(name='SCIP',paper='SCIP Optimization Suite',version='10.0.2 / PySCIPOpt6.2.1',
        scope='MILP on all matching cases; full engineering model not implemented'),
        dict(name='Streaming delta_g',published=papers['P3']['published'],doi=papers['P3']['doi'],
             scope='33 initial +59 stress conditions; one pass'),
        dict(name='Greedy+ILS1000/5000',published=papers['P2']['published'],doi=papers['P2']['doi'],
             scope='3 initial diagnostic cases, plus ILS1000 on59 stress conditions; excludes full reduction/unfold pipeline'),
        dict(name='Author full pipeline',published=papers['P2']['published'],doi=papers['P2']['doi'],
             scope='7 existing original instances x10 fixed seeds; 65 valid author runs,5 failures; reductions10 rounds, S,pin, ILS15,unfold'),
        dict(name='HypOp neural solver',published=papers['P1']['published'],doi=papers['P1']['doi'],
             scope='Not executed; only six original MIS inputs reused')]
    runtime_summary={}
    for method in ['SCIP','Streaming','Greedy','ILS1000']:
        seconds=[r['methods'][method]['total_seconds'] for r in stress_rows]
        memory=[r['methods'][method]['process_accounting']['peak_sampled_rss_bytes']/2**20 for r in stress_rows
                if r['methods'][method].get('process_accounting') and r['methods'][method]['process_accounting'].get('peak_sampled_rss_bytes') is not None]
        runtime_summary[method]=dict(total_conditions=59,
            completed_within3=sum(t<=3 for t in seconds),completed_within5=sum(t<=5 for t in seconds),
            time_seconds=dict(min=min(seconds),median=float(np.median(seconds)),max=max(seconds)),
            sampled_peak_RSS_MiB=dict(count=len(memory),min=min(memory) if memory else None,
                median=float(np.median(memory)) if memory else None,max=max(memory) if memory else None))
        if method!='SCIP':
            vals=[]
            for r in stress_rows:
                l=r['latency_observations']['5'];v=l['native_completed_objectives'][method]
                if v is not None and l['scip_recorded_objective'] is not None:vals.append(v-l['scip_recorded_objective'])
            runtime_summary[method]['at5_seconds_exploratory']=dict(eligible=len(vals),
                above=sum(v>1e-6 for v in vals),equal=sum(abs(v)<=1e-6 for v in vals),below=sum(v < -1e-6 for v in vals),
                excluded_unfinished=59-len(vals),basis='Native complete by5s vs SCIP causally recorded feasible value by5s; not independent seeds or acceptance threshold')
    result=dict(schema_version=1,task_id='hypergraph_cross_scenario_comparison',observed_at=now,
        user_request='完整对比各个维度',
        definitions=dict(n='scalar decision variables',m='declared resource/constraint rows',
            r='maximum constraint/resource incidences per variable (column support)',
            Delta='maximum variables per resource/constraint (row support); equals previous d',
            variable_vertex_view='rank=Delta, max vertex degree=r',resource_vertex_view='rank=r, max vertex degree=Delta',
            weight_ratio='max positive weight / min positive weight, separate from solved objective',
            median_p95='numpy linear sample quantiles on all declared nodes/edges, including zero-degree resources',
            scope_warning='New inspection constraints include logical relations; cannot call all of them physical resources'),
        coverage=dict(initial_conditions=33,stress_conditions=59,stress_runs=236,
            full_author_conditions=7,full_author_valid_runs=65,full_author_failed_runs=5,
            author_conditions_overlap_initial=True,iteration_conditions=3,iteration_runs=9,
            experimental_condition_count=92,independent_dataset_count=None,
            inspection_counted_representations=6+len(original.get('configured_regional_representations',[])),unconfigured_geographic_scenes=len(original['unavailable_scenes']),
            no_new_solver_runs=True,no_cloud_mutations=True),
        family_summaries=groups,initial_cases=rows,stress_cases=stress_rows,
        inspection_representations=original['rows'],unconfigured_geographic_scenes=original['unavailable_scenes'],
        configured_regional_representations=original.get('configured_regional_representations',[]),
        engineering_contract=dict(joint_solver_implemented=contract['joint_solver_implemented'],
            variable_blocks=len(contract['variables']),constraint_families=len(contract['constraint_families']),
            missing_required_groups=len(contract['missing_required_parameters']),
            local_structure=original['engineering_local_structure'],full_n=None,full_m=None,full_r=None,full_Delta=None),
        objective_comparison=dict(experiment='Single additive maximization sum w*x',
            engineering='Lexicographic incomplete components, fleet return time, operational energy',
            cross_instance_objectives_directly_comparable=False,new_engineering_solved_objective=None),
        papers=papers,methods=methods,
        paired_controlled_groups=stress['controlled_groups'],stress_runtime_summary=runtime_summary,
        exploratory_thresholds='Earlier95%-quality/2x-speed rule was agent-defined, not a user acceptance criterion; not used to declare success here.',
        source_bindings=[binding(p) for p in source_paths],
        verification=dict(all92_inputs_hash_verified=True,all92_structures_recounted=True,
            stress236_bound_raw_results_verified=True,existing_solution_validation_reused=True,
            no_new_solution_repair=True,no_runtime_restart=True),
        limitations=['Live SCIP values are solver-reported snapshots, not independently checked completed decisions.',
            'Earlier10s/first-positive results remain labelled historical; no new10s experiment launched.',
            'Historical/full-pipeline/stress stopping, warm starts and CPU concurrency differ; no equal-budget speed ranking.',
            'All matching cases use capacity1; pairwise conflict constraints represent the same binary feasible set. High rank alone does not prove irreducible high-order dependence.',
            'Full engineering model is mixed discrete/continuous and nonlinear; no full solver or graph instantiated.',
            'Maximum metrics across a family need not occur in the same instance; raw per-instance rows retained.'])
    OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({'groups':groups,'coverage':result['coverage']},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
