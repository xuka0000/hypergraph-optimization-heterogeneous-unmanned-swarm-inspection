"""Numeric snapshot of every approved condition; pending cases stay explicit."""
import copy, datetime, json
from pathlib import Path
from run_hypergraph_scip_demo import ROOT, OUT as BASE, load, save, sha, evaluate

OUT=BASE/'SCIP_gap5持续求解'


def snapshot():
    previous=load(BASE/'全部案例求解结果与时间.json')
    inputs={x['scenario_id']:x for x in load(OUT/'input_manifest.json')['instances']}
    rows=[];counts={'reused_optimal':0,'new_gap05_met':0,'running':0,'queued':0,'resource_limited':0}
    for old in previous['rows']:
        sid=old['scenario_id'];entry=inputs[sid]
        r={'scenario_id':sid,'label':old['label'],'paper_id':old['paper_id'],
           'variables':old['variables'],'resources':old['resources'],'max_rank':old['max_rank'],
           'scip_previous_objective':old['scip']['objective'],
           'scip_previous_total_seconds':old['scip']['total_seconds'],
           'hypergraph_streaming':old['hypergraph'],
           'new_solver_seconds':None,'new_total_seconds':None,'gap_target_met':False}
        initial=ROOT/entry['initial_result']['path'];assert sha(initial)==entry['initial_result']['sha256']
        raw_path=OUT/'raw'/f'{sid}_SCIP.json';progress_path=OUT/'progress'/f'{sid}.json'
        if entry['reuse_optimal']:
            r.update(state='reused_optimal',scip_objective=old['scip']['objective'],
                     upper_bound=old['scip']['objective'],gap=0.,gap_target_met=True,
                     numeric_evidence=entry['initial_result'],independently_checked=True)
        elif raw_path.exists():
            raw=load(raw_path);data_path=ROOT/entry['path'];assert sha(data_path)==entry['sha256']
            v=evaluate(load(data_path),raw['selected']);assert v['feasible'] and v['objective']==raw['validation']['objective']
            assert abs(v['objective']-raw['reported_objective'])<1e-5
            r.update(state='new_gap05_met' if raw['target_met'] else 'resource_limited',
                     scip_objective=v['objective'],upper_bound=raw['dual_bound'],gap=raw['scip_gap'],
                     new_solver_seconds=raw['solver_seconds'],new_total_seconds=raw['total_seconds'],
                     gap_target_met=raw['target_met'],independently_checked=True,
                     numeric_evidence={'path':raw_path.relative_to(ROOT).as_posix(),'sha256':sha(raw_path)})
        elif progress_path.exists():
            progress=load(progress_path)
            r.update(state='running',scip_objective=progress['objective'],upper_bound=progress['upper_bound'],
                     gap=progress['gap'],new_solver_seconds=progress['solver_seconds'],
                     observation_time=progress['observed_at'],independently_checked=False,
                     numeric_evidence={'path':progress_path.relative_to(ROOT).as_posix(),
                                       'dynamic_source':True,'captured_snapshot':progress})
        else:
            raw=load(initial)
            r.update(state='queued',scip_objective=old['scip']['objective'],
                     upper_bound=raw.get('dual_bound'),gap=raw.get('scip_gap'),
                     independently_checked=True,numeric_evidence=entry['initial_result'])
        counts[r['state']]+=1
        if r['upper_bound'] is not None and abs(r['upper_bound'])>=1e19:r['upper_bound']=None
        if r['gap'] is not None and r['gap']>=1e19:r['gap']=None
        if r['gap_target_met']:assert r['gap']<=.05+1e-9
        delta=r['hypergraph_streaming']['objective']-r['scip_objective']
        r['streaming_minus_scip']=delta
        r['streaming_relative_difference_percent']=None if r['scip_objective']==0 else 100*delta/abs(r['scip_objective'])
        rows.append(r)
    out={'observed_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'cases':len(rows),'counts':counts,
         'complete':sum(r['gap_target_met'] for r in rows)==len(rows),'scip_time_limit_seconds':None,
         'scip_first_positive_stop':False,'scip_gap_target':.05,'rows':rows,'papers':previous['papers'],
         'difference_definition':'Delta = Streaming objective minus SCIP displayed objective. Relative difference (%) = 100*delta/abs(SCIP objective); null when SCIP=0. Positive favors Streaming for maximization. This is not SCIP optimality gap. Pending rows compare against the explicitly labelled old SCIP result.',
         'timing_note':'New times are warm-started model-rebuild run costs. Prior SCIP times are separate. Running rows expose solver-reported incumbent and solver wall time; selected solution is independently checked only after output. Source values on queued rows are previous results, not new gap05 results.',
         'fairness_note':'Different stopping rules; exploratory comparison, not an equal-budget/equal-precision speed ranking. Compile and ILS processes coexist with the long SCIP run; no CPU-isolated wall-time claim.'}
    save(OUT/'数值进度表.json',out)
    state={'reused_optimal':'已最优，沿用','new_gap05_met':'新结果≤5%','running':'持续求解','queued':'待续跑，列旧值','resource_limited':'资源中止，未达标'}
    def num(v,places=3):return '—' if v is None else f'{v:,.{places}f}'
    lines=['全部案例数值进度（最大化目标）',f"观测时间：{out['observed_at']}",
           'SCIP：无10秒限时，无首个非零解停止；目标gap≤5%。待续跑行保留旧数值，不计为新结果。',
           '本轮秒数=SCIP求解器已运行时间；本轮总耗时另存JSON。旧SCIP及Streaming秒数包含构建/转换与输出。',
           '目标差=Streaming−SCIP；相对差%=目标差/|SCIP|×100。正值表示Streaming目标更高；该差值不是SCIP最优性gap。',
           '| 案例 | SCIP目标值 | 上界 | gap% | 本轮求解秒 | 旧SCIP总秒 | Streaming目标值 | 目标差（超图−SCIP） | 相对差% | Streaming总秒 | 状态 |',
           '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|']
    for r in rows:
        h=r['hypergraph_streaming'];relative='—' if r['streaming_relative_difference_percent'] is None else f"{r['streaming_relative_difference_percent']:+.3f}"
        lines.append(f"| {r['label']} | {num(r['scip_objective'],0)} | {num(r['upper_bound'])} | {num(None if r['gap'] is None else r['gap']*100)} | {num(r['new_solver_seconds'],6)} | {num(r['scip_previous_total_seconds'],6)} | {num(h['objective'],0)} | {r['streaming_minus_scip']:+,.0f} | {relative} | {num(h['total_seconds'],6)} | {state[r['state']]} |")
    lines+=['','论文来源：']+[p['title']+'；'+p['doi'] for p in previous['papers']]
    (OUT/'数值进度表.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps({'observed_at':out['observed_at'],'counts':counts,'all_gap05_complete':out['complete']},ensure_ascii=False))


if __name__=='__main__':snapshot()
