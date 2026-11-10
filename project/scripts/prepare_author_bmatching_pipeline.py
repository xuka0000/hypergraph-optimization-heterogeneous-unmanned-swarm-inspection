"""Freeze the explicitly requested original-paper comparison and real evidence."""
import datetime, hashlib, itertools, json, os, sys
from pathlib import Path
from run_author_bmatching_pipeline import ROOT, BASE, BRIDGE, OUT, TASK, now, verify_source
from run_hypergraph_scip_demo import load, save, sha, evaluate

def ref(p, **kw):
    p=Path(p);p=p if p.is_absolute() else ROOT/p
    return dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p),**kw)

def main():
    for sub in ['raw','logs','claims','progress','failures']: (OUT/sub).mkdir(exist_ok=True)
    assert not (OUT/'run_claim.json').exists()
    verify_source()
    rows=[r for r in load(BASE/'SCIP_gap5持续求解/input_manifest.json')['instances'] if r['scenario_id'].startswith('paper2026_')]
    assert len(rows)==7
    validations=[]
    for r in rows:
        assert sha(ROOT/r['path'])==r['sha256'] and sha(ROOT/r['source_path'])==r['source_sha256']
        d=load(ROOT/r['path']);assert len(d['jobs'])==r['n'] and len(d['resources'])==r['resources']
        assert all(x['capacity']==1 and x['id']==f'R{i+1}' for i,x in enumerate(d['resources']))
        with (ROOT/r['source_path']).open(encoding='utf-8') as f:
            lines=(s.strip() for s in f if s.strip() and not s.startswith('%'));h=next(lines).split()
            assert list(map(int,h[:2]))==[r['n'],r['resources']]
            count=0
            for i,(line,j) in enumerate(zip(itertools.islice(lines,r['n']),d['jobs'],strict=True)):
                v=list(map(int,line.split()));assert j['id']==f'E{i+1}' and j['value']==v[0]
                assert list(map(lambda k:int(k[1:]),j['resources']))==v[1:]
                assert len(set(v[1:]))==len(v)-1;count+=1
            footer=list(lines)
            if int(h[2])==11:assert len(footer)==r['resources'] and all(int(v)>0 for v in footer)
            else:assert not footer
        assert sum(x['value'] for x in d['jobs'])<2**31
        if r['reuse_optimal']:
            s=load(ROOT/r['initial_result']['path']);assert sha(ROOT/r['initial_result']['path'])==r['initial_result']['sha256']
            ev=evaluate(d,s['selected']);assert ev['feasible'] and ev['objective']==s['reported_objective'] and s['optimal'] and s['scip_gap']==0
        validations.append(dict(scenario_id=r['scenario_id'],source_hgr_rows_compared=count,weights_pins_verified=True,capacity_verified='Paper uniform b=1 condition; source random-capacity footer deliberately overridden by both methods as in author --capacity 1',scip_optimum_revalidated=r['reuse_optimal']))
    save(OUT/'input_manifest.json',dict(instances=rows))
    save(OUT/'input_validation.json',dict(passed=True,observed_at=now(),instances=validations))
    target='Compare feasible objective and measured full-pipeline cost on seven complete original JGAA instances; no preset winning threshold.'
    version='2f456c32c1d28de28b02822d48e383d769e667ea_reductions10_Spin_ILS15_unfold'
    protocol=dict(task_id=TASK,question=target,scenario_ids=[r['scenario_id'] for r in rows],capacity=1,
        seeds=list(range(10))+[-1],author_seeds=list(range(10)),scip_seed=0,methods=['SCIP','PaperPipeline'],
        source_commit=verify_source(),author_configuration=dict(reductions='all_removals_exhaustive',max_runs=10,NR=10,WIER_edge=8,WIER_clique=80,WD_size=6,WD_candidates=6,WT=4,greedy='S,pin',max_tries=15,unfold=True),
        source=ref(ROOT/'总结/07_证据与复现索引/超图建模与原生求解/Bmatching_author/source_manifest.json'),
        no_time_cutoff=True,author_time_default_deviation='Public optional 1800s disabled per retained user no-time-cutoff request; stagnation rule unchanged.',
        empty_residual_adapter_guard='Skip ILS only if no residual nonfree candidates, avoiding public empty random draw; no optimization added.',
        scip=dict(version='SCIP10.0.2/PySCIPOpt6.2.1',gap=.05,threads=1,memory_MiB=6144,time_limit_seconds=None,initial_solution='empty only'),
        reuse='Six same-input SCIP certified optima revalidated. Their timings are historical single runs, not ten-run means.',
        pending_old_scip='Old 33-case queue retained unchanged; superblue19 warm-start continuation is pending, not live. New cold-start condition has distinct identity and cannot launch while same scenario is live in old queue. Later warm-start result remains separately identified.',
        scope='Exploratory comparison of seven already selected full instances, four data collections, one capacity; not reproduction of all 488 topologies or all capacities.',
        statistical_unit='Ten fixed author seeds per unchanged topology. Report mean/min/max, all seeds; not ten independent datasets.',
        timing='Wall-clock construction, optimization and extraction; excludes shared input disk load and independent validation; author complete pipeline vs SCIP original model; unequal stopping requirements explicit.',
        budget=dict(paid_cost=0,cloud=False,author_slots=1,new_scip_slots=1,peak_extra_write_bytes=2**30,RAM_guard_MiB=8192),
        source_code=[ref(ROOT/'scripts/run_author_bmatching_pipeline.py'),ref(ROOT/'scripts/author_bmatching_pipeline_adapter.cpp'),ref(OUT/'author_pipeline.dll'),ref(ROOT/'scripts/run_scip_gap05.py')])
    save(OUT/'protocol.json',protocol)
    src=dict(kind='codex_user_message',path=r'C:\Users\Out\.codex\sessions\2026\11\07\rollout-2026-11-07T12-57-32-01a1b45e-78cb-7362-a8c7-13f897a636c7.jsonl',line=3590,
        sha256='666467ee1f3bcf6510bd3b987b3fc77b2350942a81996b8d23b5b295298acb7c',quote='对！超边关联资源最大值 和 资源耦合变量远远高于我们！ 直接在他的场景中测试一下SCIP和他的算法看看效果！',observed_timestamp='2026-11-07T13:30:23.931Z')
    q=ROOT/'research_quality'
    def put(s,v):p=q/f'{TASK}_{s}.json';save(p,v);return ref(p)
    auth=put('user_approval',dict(source=src,scope='Direct execution request for original-paper scenarios against SCIP. Reuse full existing seven inputs, author recipe, local cost zero.',interpretation='Task-specific execution authorization, not user acceptance of subsequent results or a new scientific winning threshold.'))
    mat=dict(code=ref(ROOT/'scripts/run_author_bmatching_pipeline.py'),config=ref(OUT/'protocol.json'),data=ref(OUT/'input_manifest.json'),evaluator=ref(ROOT/'scripts/run_hypergraph_scip_demo.py'))
    pr=put('protocol',dict(claim_id='original_hypergraph_structure_comparison',comparison_id=TASK,core_target=target,scenario_ids=protocol['scenario_ids'],baseline_ids=['PaperPipeline'],ablation_ids=[],own_method_id='SCIP',role_note='Schema role only: SCIP is not our invented algorithm.',method_versions=dict(SCIP='10.0.2_original_capacity_MILP_gap05_or_reused_optimal',PaperPipeline=version),benchmark_materials=mat))
    controls=put('controls',dict(paused=False,cancelled=False,deadline=(datetime.datetime.now(datetime.timezone.utc)+datetime.timedelta(hours=24)).isoformat(),deadline_kind='admission_snapshot_valid_until',user_task_deadline=None,authorization=auth,budget=protocol['budget'],original_jobs_and_pauses_unchanged=True))
    expected=put('expected',dict(instances=7,passed=True))
    content=put('content',dict(schema_version=1,task_id=TASK,stage='research',checks=[dict(id='input_count',kind='derived_consistency',scope='Seven full original instances',data=ref(OUT/'input_manifest.json',pointer='/instances'),operation='count',reported=dict(**expected,pointer='/instances')),dict(id='regression',kind='identity',scope='23 tiny known-case evaluations with actual folding and unfolding',expected=expected,actual=ref(OUT/'runner_regression.json'),fields=[dict(expected='passed',actual='passed')])],coverage={}))
    reasons=dict(entrypoint='Actual batch invokes transition and preflight; one claim per execution, separate child identities, no same live SCIP scenario.',units_parameters='Every original HGR row, weight and pin equals JSON; capacity1; positive integer weights and signed32 total bound.',initial_events='Static original hypergraphs; no altered topology, weights or temporal information.',constraints_termination='Same binary weighted capacity model; SCIP gap5%, no time cutoff; author original stagnation k15 and documented empty residual guard.',failure_semantics='All seven instances and ten predetermined seeds remain in denominator; failures retained separately without invented objective.',known_solution='23 exhaustive tiny cases include actual folds; six original SCIP optima independently reevaluated.',random_pairing='Identical full input bytes; fixed seeds0..9, no selection of favorable repeats.',data_roles='Exploratory original scenarios already exposed, not untouched formal test.',evaluation_purity='Pure evaluator sums original selected weights and original capacities; no repair or extra optimizer.')
    evidence=[ref(OUT/'input_validation.json'),ref(OUT/'runner_regression.json'),ref(OUT/'protocol.json'),ref(ROOT/'scripts/run_author_bmatching_pipeline.py')]
    checks=[dict(id=k,status='PASS',reason=v,evidence=evidence,**({'machine_check_ids':['input_count' if k=='units_parameters' else 'regression']} if k in ['units_parameters','known_solution'] else {})) for k,v in reasons.items()]
    benchmark=put('benchmark',dict(materials=mat,content_manifest=content,checks=checks))
    source=put('source',dict(method_id='PaperPipeline',method_version=version,paper_url='https://doi.org/10.7155/jgaa.v30i1.3166',code_url='https://github.com/HeiHGM/Bmatching',commit=protocol['source_commit'],citation_key='GrossmannJRS2026',license='MIT, original LICENSE retained',adaptation='Array interface calling unchanged reductions, S,pin, ILS and unfold headers; skips undefined empty residual random draw; diagnostic timers disabled; no search mechanism added; full CLI not used.',fairness_reason='Same full original problem, CPU single thread, full pipeline timing. Six certified SCIP optima reused; largest SCIP cold-start with gap5, no author solution supplied. Different termination and historical timing reported.',faithfulness='PASS',execution_mode='nonlearning',first_public_fulltext='2024-08-13',evidence=evidence+protocol['source_code']+[protocol['source'],ref(BASE/'原作者ILS迭代/论文实验核对.json')]))
    exception=put('selection_exception',dict(reason='User specifically requests this paper algorithm versus SCIP; not a new-method SOTA comparison. Do not add unrelated algorithms.',reviewer='Original owner interpreting explicit current request',baseline_ids=['PaperPipeline'],covers=['classic_count','recent_count'],search_evidence=[source,auth]))
    baselines=put('baselines',dict(search_as_of='2026-11-07',methods=[dict(id='PaperPipeline',category='other',source_review=source)],critical_competitor_ids=['PaperPipeline'],selection_exception=exception))
    preflight=put('preflight',dict(schema_version=1,rule_id='RESEARCH-EXPERIMENT-SEQUENCE-01',task_id=TASK,protocol=pr,controls=controls,benchmark=benchmark,baselines=baselines))
    sys.path.insert(0,str(BRIDGE));from review_gate import material_binding,scope_binding
    design=dict(id=TASK+'_requested',kind='design',scope_version='1',protocol_version='1',depends_on=[],scope=dict(baseline_set=['PaperPipeline'],data_role='development',core_target=target),artifacts=[dict(id='protocol',**mat['config']),dict(id='inputs',**mat['data']),dict(id='regression',**ref(OUT/'runner_regression.json')),dict(id='authorization',**auth),dict(id='preflight',**preflight)],blockers=[])
    nxt=dict(id=TASK,kind='benchmark',scope_version='1',protocol_version='1',depends_on=[design['id']],scope=dict(baseline_set=['PaperPipeline'],data_role='development',core_target=target,protocol_sha256=mat['config']['sha256'],manifest_sha256=mat['data']['sha256']),review=dict(status='draft'))
    design['review']=dict(status='approved',decision='approved',reviewed_at=src['observed_timestamp'],source=src,material_binding=material_binding(design),predecessor_bindings={},annotations=[],approved_next={TASK:scope_binding(nxt)},interpretation='User explicitly directs this local experiment; no claim they reviewed files generated after their instruction or accepted results.')
    put('review_nodes',dict(schema_version=3,stage='S1',review_nodes=[design,nxt]))
    save(OUT/'experiment_identity.json',dict(task_id=TASK,status='prepared_authorized',pid=None,created_at=now(),protocol_sha256=sha(OUT/'protocol.json'),planned_new_runs=71,reused_scip_optima=6,owner_thread_id=os.environ.get('CODEX_THREAD_ID')))
    print('Frozen seven original instances; 70 author runs plus one cold SCIP run; six SCIP optima reused.')

if __name__=='__main__':main()
