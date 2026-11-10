"""Prepare a fixed scale/rank extension for review; never invoke a solver."""
from pathlib import Path
import collections, datetime, hashlib, json, os, random, shutil

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'总结/03_实验数据/超图与SCIP组合任务对比/复杂度扩展'

def save(p,obj):p.write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf-8')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    if (OUT/'protocol.json').exists():raise SystemExit('Review batch already exists; inspect rather than overwrite.')
    OUT.mkdir(parents=True,exist_ok=True);(OUT/'instances').mkdir(exist_ok=True)
    specs=[(300,6),(1000,6),(3000,6),(1000,3),(1000,12),(1000,24)]
    seeds=[0,1,2];rows=[]
    for seed in seeds:
        # Shared prefix construction pairs weights and resource sets across axes.
        # Independent streams avoid rank/size-dependent changes in random draws.
        rr=random.Random(814000+seed);wr=random.Random(927000+seed)
        bundles=[rr.sample(range(200),24) for _ in range(3000)]
        weights=[wr.randint(1,100) for _ in range(3000)]
        for n,r in specs:
            sid=f'n{n}_r{r}_seed{seed}'
            jobs=[{'id':f'T{i:04d}','resources':[f'R{v:03d}' for v in sorted(bundles[i][:r])],'value':weights[i]} for i in range(n)]
            data={'scenario_id':sid,'provenance':'Synthetic pedagogical/development generator; not a published benchmark or field data.','resource_count':200,'rank':r,'instance_seed':seed,
                  'resources':[{'id':f'R{i:03d}','capacity':1,'unit':'indivisible resource unit'} for i in range(200)],'jobs':jobs,
                  'value_unit':'dimensionless synthetic benefit','stream_order':'listed order, shared across both solvers; never sorted by outcome'}
            p=OUT/'instances'/(sid+'.json');save(p,data)
            incidence={res['id']:[] for res in data['resources']}
            for j in jobs:
                assert len(j['resources'])==len(set(j['resources']))==r
                assert 1<=j['value']<=100
                for v in j['resources']:incidence[v].append(j['id'])
            degrees=[len(x) for x in incidence.values()]
            rows.append({'scenario_id':sid,'n':n,'r':r,'resources':200,'seed':seed,'incidences':sum(degrees),'mean_variables_per_capacity_constraint':sum(degrees)/200,
                         'min_variables_per_capacity_constraint':min(degrees),'max_variables_per_capacity_constraint':max(degrees),
                         'nonempty_resource_constraints':sum(x>0 for x in degrees),'distinct_nonempty_constraint_rows':len({tuple(x) for x in incidence.values() if x}),
                         'duplicate_bundles':n-len({tuple(j['resources']) for j in jobs}),
                         'path':p.relative_to(ROOT).as_posix(),'sha256':sha(p)})
    byid={row['scenario_id']:json.loads((ROOT/row['path']).read_text(encoding='utf-8')) for row in rows}
    for seed in seeds:
        large=byid[f'n3000_r6_seed{seed}']['jobs']
        assert byid[f'n300_r6_seed{seed}']['jobs']==large[:300]
        assert byid[f'n1000_r6_seed{seed}']['jobs']==large[:1000]
        for a,b in zip([3,6,12],[6,12,24]):
            ja=byid[f'n1000_r{a}_seed{seed}']['jobs'];jb=byid[f'n1000_r{b}_seed{seed}']['jobs']
            assert all(x['id']==y['id'] and x['value']==y['value'] and set(x['resources'])<=set(y['resources']) for x,y in zip(ja,jb))
    now=datetime.datetime.now(datetime.timezone.utc).isoformat()
    protocol={'experiment_id':'hypergraph-scip-complexity-extension','created_at':now,'status':'prepared_pending_scope_review','scope':'Extend the previously shown pedagogical bundle-selection comparison; not formal paper evaluation or inspection model revision.',
       'question':'How do decision count and bundle rank affect observed quality, time and ability to prove optimality?',
       'specifications':[{'tasks':n,'rank':r,'resources':200} for n,r in specs],'seeds':seeds,'instances':18,'methods':['SCIP10.0.2_MILP','HeiHGM_Streaming_delta_g'],
       'scip':{'seconds_per_instance':10,'threads':1,'random_seed_shift':0,'other_settings':'defaults including presolve/heuristics; sparse equivalent model construction'},
       'hypergraph':{'commit':'f2f2af98d13d34b47cde6ce3d096c4240dcc5a5d','epsilon':0.1,'termination':'one pass then original reverse-stack output','binary_sha256':sha(OUT.parent/'author_streaming.dll'),'same_existing_adapter':True},
       'budget':{'solver_time_ceiling_seconds':180,'batch_wall_guard_seconds':300,'paid_cost':0,'cloud':False,'peak_extra_write_reservation_MiB':50},
       'repetitions':'3 fixed paired synthetic seeds per configuration; no result-driven seeds/tuning/replacement; not sufficient for population significance claim.',
       'pairing':'Resource universe200 fixed. Task counts are nested prefixes at rank6. Ranks3,6,12,24 at n1000 are nested resource requirements with identical weights and order.',
       'metric_rules':{'quality':'objective, feasibility, selected tasks, SCIP primal/dual bound and termination status','unknown_optimum':'When time-limited, label relative difference to SCIP incumbent, never an optimality loss; also give dual-bound certificate range when finite.','timing':'Same machine; build/convert+solve+extract excluding disk/import/evaluator, with solver-only time separate. No padding of native runtime to10s.'},
       'interpretation_limits':['Increasing rank also changes overlap and feasible-set size; not a universal monotonic hardness experiment.','Increasing task count at fixed resources increases competition as well as variables.','Capacity1 set packing remains exactly expressible as a conflict-graph independent set.','Only this particular public one-pass hypergraph solver is assessed; not all hypergraph algorithms.'],
       'stop_conditions':['Every prelisted instance complete or recorded failed','Finite batch guard reached','Explicit user pause/stop','Validation or source binding fails'],
       'review_reason':'Current user asks what happens if complexity grows; this concrete batch is prepared for confirmation under local reviewed scale-transition rule. No new solver comparisons started.'}
    save(OUT/'protocol.json',protocol)
    save(OUT/'input_manifest.json',{'observed_at':now,'generator_sha256':sha(Path(__file__)),'instances':rows,'checks':{'all_units_and_memberships_valid':True,'nested_task_pairing':True,'nested_rank_pairing':True,'no_solver_called':True}})
    identity={'experiment_id':protocol['experiment_id'],'status':'prepared_pending_scope_review','owner_thread':os.getenv('CODEX_THREAD_ID'),'created_at':now,'pid':None,'completed_cells':0,
      'prior_demo_status':'complete; not rerun','live_duplicate_check':'No matching solver process or existing scale task found before preparation.','previous_inspection_pauses_unchanged':True,'free_GiB':{d:shutil.disk_usage(d).free/2**30 for d in ['C:/','D:/']}}
    save(OUT/'experiment_identity.json',identity)
    table=['复杂度扩展：待确认方案','', '状态：输入已生成并校验；尚未调用任何求解器。','', '档位\t资源数\t任务变量数\t每任务资源数r\t每条资源约束平均联系的任务数\t实例数']
    for i,(n,r) in enumerate(specs,1):table.append(f'{i}\t200\t{n}\t{r}\t{n*r/200:g}\t3')
    table += ['', '求解方法：保持上一轮SCIP10.0.2与ESA2025原始—对偶栈作者核心；ε=0.1不变。',
      '预算：SCIP每例最多10秒、单线程，18例最多180秒求解时间；整批墙钟保护300秒；只用本地CPU，无云费用。',
      '配对：种子0、1、2；任务数增加沿用相同前缀；r增加保留同一任务及收益并逐步增加资源需求。',
      '结果：保留全部18例×2方法的可行性、收益、建模/求解时间、SCIP上下界及是否证明最优。',
      'SCIP超时时只报告当前最好可行解与上界，不能称其结果为全局最优。',
      '停止：固定矩阵完成、300秒整批保护、校验失败或用户停止；不为找到超图获胜案例继续增加样本。',
      '用途：研究示例的扩展诊断，不作为巡检论文正式比较；原巡检耦合量化继续保留待办。',
      '规则依据：.codex/paper_rules/reviewed_benchmark_workflow.md 第3节；具体扩展档位需先获确认。']
    (OUT/'扩展方案.txt').write_text('\n'.join(table)+'\n',encoding='utf-8')
    print(json.dumps({'status':identity['status'],'instances':len(rows),'solves_started':0,'max_capacity_constraint_sizes':{str((n,r)):max(x['max_variables_per_capacity_constraint'] for x in rows if x['n']==n and x['r']==r) for n,r in specs},'input_bytes':sum((ROOT/x['path']).stat().st_size for x in rows)},ensure_ascii=False))

if __name__=='__main__':main()
