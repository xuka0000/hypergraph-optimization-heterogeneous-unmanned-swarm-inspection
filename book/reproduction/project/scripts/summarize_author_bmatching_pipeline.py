"""Pure validation and numeric table from raw original-instance decisions."""
import collections, datetime, json, statistics
from pathlib import Path
from run_author_bmatching_pipeline import ROOT, BASE, OUT, TASK, now, verify_source
from run_hypergraph_scip_demo import load, save, sha

def ref(p):return dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p))

def independent_check(data,result):
    byid={x['id']:x for x in data['jobs']};cap={x['id']:x['capacity'] for x in data['resources']}
    chosen=result['selected'];assert len(chosen)==len(set(chosen)) and all(x in byid for x in chosen)
    use=collections.Counter(v for x in chosen for v in byid[x]['resources'])
    assert all(n<=cap[v] for v,n in use.items())
    value=sum(byid[x]['value'] for x in chosen);assert abs(value-result['reported_objective'])<1e-6
    if 'validation' in result:assert value==result['validation']['objective']
    assert result['total_seconds']>=result['output_seconds']>=0
    return value

def main():
    protocol=load(OUT/'protocol.json');verify_source()
    for f in protocol['source_code']:assert sha(ROOT/f['path'])==f['sha256']
    rows=[];allraw=[];validated=0;failure_refs=[]
    for r in load(OUT/'input_manifest.json')['instances']:
        d=load(ROOT/r['path']);assert sha(ROOT/r['path'])==r['sha256']
        author=[]
        for seed in protocol['author_seeds']:
            f=OUT/'raw'/f'{r["scenario_id"]}_author_s{seed}.json'
            if not f.exists():continue
            x=load(f);assert x['seed']==seed and x['input_sha256']==r['sha256'] and x['max_tries']==15
            assert x['dll_sha256']==sha(OUT/'author_pipeline.dll') and x['adapter_sha256']==sha(ROOT/'scripts/author_bmatching_pipeline_adapter.cpp')
            independent_check(d,x);author.append(x);allraw.append(ref(f));validated+=1
        sfile=ROOT/r['initial_result']['path'] if r['reuse_optimal'] else OUT/'raw'/f'{r["scenario_id"]}_SCIP_cold_gap05.json'
        scip=load(sfile) if sfile.exists() else None
        if scip:
            assert scip['input_sha256']==r['sha256'];independent_check(d,scip);allraw.append(ref(sfile));validated+=1
        failures=[]
        for seed in protocol['author_seeds']:
            f=OUT/'failures'/f'{r["scenario_id"]}_s{seed}.json'
            if f.exists():
                failures.append(load(f));failure_refs.append(ref(f))
        done=len(author)==10
        v=[x['reported_objective'] for x in author];t=[x['total_seconds'] for x in author]
        avg=statistics.mean(v) if done else None;tm=statistics.mean(t) if done else None
        sv=scip['reported_objective'] if scip else None
        deg=collections.Counter(v for x in d['jobs'] for v in x['resources'])
        assert max(map(lambda j:len(j['resources']),d['jobs']))==r['r'] and max(deg.values())==r['max_variables_per_capacity_constraint']
        row=dict(scenario_id=r['scenario_id'],family=r['family'],n=r['n'],resources=r['resources'],rank_max=r['r'],degree_max=r['max_variables_per_capacity_constraint'],mean_rank=r['mean_rank'],mean_degree=r['mean_variables_per_capacity_constraint'],
            author_completed=len(author),author_failed=len(failures),author_failures=failures,author_planned=10,author_mean_objective=avg,author_valid_subset_mean_objective=statistics.mean(v) if v else None,author_min_objective=min(v) if v else None,author_max_objective=max(v) if v else None,author_std_objective=statistics.stdev(v) if done else None,
            author_mean_seconds=tm,author_valid_subset_mean_seconds=statistics.mean(t) if t else None,author_min_seconds=min(t) if t else None,author_max_seconds=max(t) if t else None,
            scip_objective=sv,scip_total_seconds=scip['total_seconds'] if scip else None,scip_solver_seconds=scip['solver_seconds'] if scip else None,
            scip_gap=scip['scip_gap'] if scip else None,scip_status=scip['status'] if scip else 'running_or_pending',scip_reused=r['reuse_optimal'],
            objective_difference=avg-sv if avg is not None and sv is not None else None,
            objective_difference_percent=100*(avg-sv)/sv if avg is not None and sv else None,
            scip_seconds_div_author_mean=scip['total_seconds']/tm if tm and scip else None,
            author_seeds=[dict(seed=x['seed'],objective=x['reported_objective'],total_seconds=x['total_seconds'],reduction_seconds=x['author_reduction_seconds'],ILS_seconds=x['author_ils_seconds'],folds=x['reduction_folds'],fixed_edges=x['reduction_fixed_edges']) for x in author])
        if scip and done:
            ub=scip['dual_bound'];assert ub+1e-4>=max(v)
            row['author_mean_gap_to_valid_upper_bound']=max(0,(ub-avg)/avg)
            trace=[x for x in scip.get('trajectory',[]) if x['objective']>=avg]
            row['scip_first_observed_at_least_author_mean_seconds_including_build']=scip['build_seconds']+trace[0]['solver_seconds'] if trace else None
        rows.append(row)
    complete=validated+len(failure_refs)==77 and all(r['scip_objective'] is not None for r in rows)
    all_valid=validated==77 and all(r['scip_gap'] is not None and r['scip_gap']<=.05+1e-9 for r in rows)
    save(OUT/'results.json',dict(task_id=TASK,observed_at=now(),status=('complete_with_failures' if not all_valid else 'complete') if complete else 'partial',rows=rows,raw_results=allraw,failed_author_attempts=failure_refs,protocol=ref(OUT/'protocol.json')))
    save(OUT/'delivery_validation.json',dict(passed=complete,observed_at=now(),scope='All fixed outcomes and reported numbers accounted for, not all methods successful',expected_decisions=77,independently_verified_decisions=validated,failed_author_runs=len(failure_refs),all_runs_feasible_and_scip_gap05=all_valid,author_expected=70,scip_expected=7,raw_result_hashes=allraw,failed_author_attempts=failure_refs,frozen_code_verified=True,author_source_files_verified=148,complete=complete))
    def fmt(x, digits=2):return '—' if x is None else f'{x:,.{digits}f}'
    lines=['原论文完整场景：SCIP与作者完整流程对比',f'观测时间：{now()}；有效决策：{validated}/77；作者失败：{len(failure_refs)}；状态：'+('固定批次结束，含失败' if complete else '进行中'),'',
        '| 场景 | 变量数 | 资源数 | 最大超边阶数 | 最大资源耦合度 | 作者种子完成 |', '|---|---:|---:|---:|---:|---:|']
    for r in rows:lines.append(f'| {r["scenario_id"].removeprefix("paper2026_")} | {r["n"]:,} | {r["resources"]:,} | {r["rank_max"]:,} | {r["degree_max"]:,} | {r["author_completed"]}/10（失败{r["author_failed"]}） |')
    lines+=['','| 场景 | SCIP目标 | 作者目标均值 | 作者最小—最大 | 差值：作者−SCIP | 相对差值% | SCIP gap% |','|---|---:|---:|---:|---:|---:|---:|']
    for r in rows:lines.append(f'| {r["scenario_id"].removeprefix("paper2026_")} | {fmt(r["scip_objective"],0)} | {fmt(r["author_mean_objective"],1)} | {fmt(r["author_min_objective"],0)}—{fmt(r["author_max_objective"],0)} | {fmt(r["objective_difference"],1)} | {fmt(r["objective_difference_percent"])} | {fmt(100*r["scip_gap"] if r["scip_gap"] is not None else None,3)} |')
    lines+=['','| 场景 | SCIP总耗时/s | 作者平均总耗时/s | 作者最小—最大耗时/s | SCIP/作者耗时倍数 | SCIP状态 |','|---|---:|---:|---:|---:|---|']
    for r in rows:lines.append(f'| {r["scenario_id"].removeprefix("paper2026_")} | {fmt(r["scip_total_seconds"],4)} | {fmt(r["author_mean_seconds"],4)} | {fmt(r["author_min_seconds"],4)}—{fmt(r["author_max_seconds"],4)} | {fmt(r["scip_seconds_div_author_mean"])} | {r["scip_status"]} |')
    lines+=['','| 不完整场景（有效子集仅作诊断） | 有效/固定次数 | 有效子集目标均值 | 有效子集平均耗时/s |','|---|---:|---:|---:|']
    for r in rows:
        if r['author_failed']:lines.append(f'| {r["scenario_id"].removeprefix("paper2026_")} | {r["author_completed"]}/10 | {fmt(r["author_valid_subset_mean_objective"],1)} | {fmt(r["author_valid_subset_mean_seconds"],4)} |')
    lines+=['','| 失败记录 | 原因 |','|---|---|']
    for r in rows:
        for f in r['author_failures']:
            reason=f.get('guard') or ('原始约束校验失败：R426占用2，容量1' if 'ibm02' in r['scenario_id'] else '原作者内核Windows异常0xc00000ff，原因未完全定位')
            lines.append(f'| {r["scenario_id"].removeprefix("paper2026_")} / seed {f["seed"]} | {reason} |')
    lines+=['','| 项目 | 设置 |','|---|---|','| 数学模型 | 最大化 Σ wₑxₑ；每个资源 Σ xₑ≤1；xₑ∈{0,1} |','| 数据 | 七个完整原始拓扑及原始random100边权；统一容量b=1，为论文实验条件之一；未采用文件中的随机容量列 |','| 作者算法 | 精确归约（最多10轮）→S,pin贪心→ILS（k=15）→unfold还原；固定种子0—9 |','| SCIP | 单线程；gap≤5%或最优；无时间截断；大实例冷启动，仅空解，无作者解热启动 |','| 结果复用 | 前六个SCIP为同输入历史最优结果；大实例为本批新结果 |','| 时间口径 | 数据已在内存后的构建、求解、提取；不含公共磁盘读取与独立校验；作者含全部归约与还原成本 |','| 时间条件 | 本机共享CPU并发；前六个SCIP是历史单次，作者为本批10次均值；不是独占CPU等预算性能排名 |','| 适配 | 原作者核心源码未改；Windows数组接口；空残余候选时跳过无合法操作的ILS；禁用原可选1800秒截断 |','| 统计范围 | 七个固定实例，每例10个作者种子；不代表488个全数据集或独立数据样本；未设优势达标线 |','',
        '[1] Großmann E., Joos F., Reinstädtler H., Schulz C. Engineering Hypergraph b-Matching Algorithms. JGAA, 30(1):1–24, 2026-01-17. https://doi.org/10.7155/jgaa.v30i1.3166',
        '[2] 原作者源码：https://github.com/HeiHGM/Bmatching；commit 2f456c32c1d28de28b02822d48e383d769e667ea；MIT。',
        '[3] 原作者数据：https://doi.org/10.5281/zenodo.18225669；逐行权重、关联关系与本地JSON核验见input_validation.json。',
        '[4] SCIP Optimization Suite 10.0：https://www.scipopt.org/；本机SCIP10.0.2 / PySCIPOpt6.2.1。']
    (OUT/'原始场景对比报告.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(dict(complete=complete,verified=validated,rows=[{k:v for k,v in r.items() if k not in ['author_seeds']} for r in rows]),ensure_ascii=False))

if __name__=='__main__':main()
