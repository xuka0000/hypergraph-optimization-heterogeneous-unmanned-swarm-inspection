"""Refresh a numerical partial/final report from actual stress outputs."""
import statistics
import os
import tempfile
import time
from hypergraph_stress_common import *


def publish_text(path, text):
    """Publish one report, tolerating a briefly locked Windows destination.

    Keep this repair outside the frozen solver/common code. Unique staging
    files avoid clashes between an owner refresh and a batch refresh.
    """
    fd, name = tempfile.mkstemp(prefix=path.name+'.', suffix='.tmp', dir=path.parent)
    staged = Path(name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8', newline='\n') as stream:
            stream.write(text)
        for attempt in range(21):
            try:
                staged.replace(path)
                return
            except PermissionError:
                if attempt == 20:
                    raise
                time.sleep(.25)
    finally:
        if staged.exists():
            staged.unlink()


def n(v,places=0):
    return '—' if v is None else f'{v:,.{places}f}'


def summarize():
    manifest=load(OUT/'input_manifest.json');rows=[];finished=0;failed=0;certified=0
    for row in manifest['instances']:
        result=dict(row,methods={},comparisons={})
        for method in METHODS:
            p=OUT/'raw'/f"{row['scenario_id']}__{method}.json"
            progress=OUT/'progress'/f"{row['scenario_id']}__{method}.json"
            fail=OUT/'failures'/f"{row['scenario_id']}__{method}.json"
            if p.exists():
                raw=load(p);assert raw['input']==row['input']
                z={k:raw.get(k) for k in ['status','total_seconds','solver_seconds','gap','upper_bound','gap_target_met','build_seconds','output_seconds','observed_at']}
                z.update(objective=raw['validation']['objective'],validation=raw['validation'],
                         complete=True,independently_checked=True,evidence=ref(p))
                if method=='SCIP':z['trajectory']=raw['trajectory'];certified+=int(bool(raw['gap_target_met']))
                finished+=1
            elif fail.exists():
                z={'status':'failure','objective':None,'total_seconds':None,'complete':False,'evidence':ref(fail)};failed+=1
            elif progress.exists():
                z=dict(load(progress),complete=False,independently_checked=False)
            else:
                z={'status':'queued','objective':None,'total_seconds':None,'complete':False}
            acct=OUT/'logs'/f"{row['scenario_id']}__{method}_accounting.json"
            if acct.exists():z['process_accounting']=load(acct)
            result['methods'][method]=z
        s=result['methods']['SCIP']
        for method in ['Streaming','Greedy','ILS1000']:
            h=result['methods'][method];f=h.get('objective');baseline=s.get('objective')
            if f is None or baseline is None:continue
            diff=f-baseline
            t=s.get('total_seconds');ht=h.get('total_seconds')
            ratio=t/ht if t and ht else None
            upper=s.get('upper_bound')
            certificate=None if upper is None or not f else max(0,(upper-f)/abs(f))
            # Only completed, independently checked SCIP decisions support the final comparison.
            reliable=bool(s.get('complete') and s.get('gap_target_met') and h.get('complete'))
            suitable=bool(reliable and f>=.95*baseline and ratio is not None and ratio>=2)
            dominance=bool(reliable and f>=baseline and ratio is not None and ratio>=1)
            at_same_time=None
            if s.get('trajectory') and ht is not None:
                known=[p['objective'] for p in s['trajectory'] if p['total_seconds']<=ht and p['objective'] is not None]
                at_same_time=max(known) if known else 0
            result['comparisons'][method]={'objective_difference':diff,
                'relative_difference_percent':100*diff/abs(baseline) if baseline else None,
                'scip_seconds_divided_by_native_seconds':ratio,'native_certified_gap_from_scip_bound':certificate,
                'reference_complete_gap05':reliable,'quality95_and_speed2x':suitable,
                'weak_pareto_dominance':dominance,'scip_incumbent_available_at_native_finish':at_same_time,
                'interpretation':'Provisional until SCIP independently checked and gap<=5%;95% uses SCIP reference objective, not unknown optimum.'}
        rows.append(result)
    groups=[]
    baselines={'n':256,'rank':4,'degree':4,'spread':100}
    levels={'n':[256,1024,4096,16384],'rank':[4,8,16,32],'degree':[4,8,16,32],'spread':[1,10,100,1000]}
    for factor, values in levels.items():
        for value in values:
            subset=[r for r in rows if r['family']=='controlled_regular' and
                    ((r['axis']=='base' and value==baselines[factor]) or (r['axis']==factor and r['level']==value))]
            assert len(subset)==3
            for method in ['Streaming','Greedy','ILS1000']:
                cs=[r['comparisons'].get(method,{}) for r in subset]
                complete=all(c.get('reference_complete_gap05') for c in cs)
                groups.append({'factor':factor,'level':value,'method':method,'planned_paired_seeds':3,
                    'valid_completed_pairs':sum(bool(c.get('reference_complete_gap05')) for c in cs),
                    'complete':complete,'quality95_speed2x_count':sum(c.get('quality95_and_speed2x',False) for c in cs) if complete else None,
                    'median_relative_difference_percent':statistics.median(c['relative_difference_percent'] for c in cs) if complete else None,
                    'min_relative_difference_percent':min(c['relative_difference_percent'] for c in cs) if complete else None,
                    'max_relative_difference_percent':max(c['relative_difference_percent'] for c in cs) if complete else None,
                    'median_speed_ratio':statistics.median(c['scip_seconds_divided_by_native_seconds'] for c in cs) if complete else None,
                    'scenario_ids':[r['scenario_id'] for r in subset]})
    report={'observed_at':now(),'task_id':TASK,'planned_cases':59,'planned_runs':236,
            'completed_runs':finished,'failed_runs':failed,'scip_gap05_complete':certified,
            'all_methods_returned':finished+failed==236,'all_comparisons_certified':certified==59 and finished==236,
            'rows':rows,'controlled_groups':groups,
            'definitions':{'n':'candidate binary variables','r':'resources jointly consumed per candidate hyperedge',
                'Delta':'variables coupled in a capacity constraint','W':'relative positive objective-weight range; not optimized objective value',
                'neural_d':'not applicable; replacing feature width explicitly with structural Delta',
                'controlled_identity':'n*r=m*Delta; m is derived, not a fifth independently fixed factor'},
            'prospective_suitability_rule':'Against completed gap<=5% SCIP reference, native objective>=95% of SCIP and total time<=50% of SCIP. Also report absolute objective differences, certified bound gaps, memory and counterexamples. No universal advantage assumption.',
            'limitations':['3 fixed seeds are exploratory, no significance or generalization claim.',
                'One public source topology;two weight schemes/nested subgraphs are paired conditions,not independent real-world samples.',
                'Public rank clipping changes degree too;resource splitting changes the feasible set. Only synthetic regular block isolates the other requested controls.',
                'No10s cutoff;SCIPgap5 or resource failure. Pending cases remain in fixed denominator.',
                'Concurrent old SCIP and new solver slots; wall time is local resource-concurrent timing,not CPU-isolated benchmark.',
                'NativeILS1000 is separate from Streaming;publicGreedy/ILS search kernels only,not full author reduction/ILP pipeline.',
                'Capacity1 can be represented exactly as conflict-graph independent set. Results concern these solvers,not exclusive expressive power of hypergraphs.'],
            'source_manifest':ref(OUT/'input_manifest.json'),'protocol':ref(OUT/'protocol.json')}
    report['method_summary'] = {
        method: {
            'completed_comparisons': sum(r['comparisons'].get(method, {}).get('reference_complete_gap05', False) for r in rows),
            'quality95_speed2x_count': sum(r['comparisons'].get(method, {}).get('quality95_and_speed2x', False) for r in rows),
            'objective_at_least_scip_count': sum(r['comparisons'].get(method, {}).get('reference_complete_gap05', False)
                and r['comparisons'][method]['objective_difference'] >= 0 for r in rows),
        } for method in ['Streaming', 'Greedy', 'ILS1000']}
    publish_text(OUT/'results.json', json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    lines=['四因素压力测试报告', '观测时间：'+report['observed_at'],
           f"实际完成运行：{finished}/236；失败：{failed}；SCIP达到gap≤5%：{certified}/59。",
           '模型：max Σ w_e x_e；Σ_{e含v}x_e≤1；x_e∈{0,1}。四方法共享完全相同的实例与评价器。',
           'SCIP没有求解时间上限；原生单遍／贪心／ILS分别按原算法停止。目标值均为实际输出，越大越好。','',
           '| 方法 | 完整比较/59 | 质量≥SCIP的95%且至少快2倍 | 目标值≥SCIP |',
           '|---|---:|---:|---:|']
    for method, counts in report['method_summary'].items():
        lines.append(f"| {method} | {counts['completed_comparisons']}/59 | {counts['quality95_speed2x_count']}/59 | {counts['objective_at_least_scip_count']}/59 |")
    lines += ['', '上述为固定条件的描述性计数，含同源派生与尺度校验条件，不是独立样本成功率。', '',
           '| 因素 | 控制实验水平 | 固定基准 |', '|---|---|---|',
           '| n：决策变量数 | 256 / 1024 / 4096 / 16384 | r=4，Δ=4，W=100 |',
           '| r：超边基数 | 4 / 8 / 16 / 32 | n=256，Δ=4，W=100 |',
           '| Δ：约束耦合度 | 4 / 8 / 16 / 32 | n=256，r=4，W=100 |',
           '| W：权重范围 | 1 / 10 / 100 / 1000 | n=256，r=4，Δ=4 |',
           '控制实验3个固定种子；共39条件。公共superblue19派生18条件＋目标统一乘10的2个校验条件。',
           '神经特征d不适用于本轮非神经求解器；明确用结构耦合度Δ替代。m=n×r/Δ随控制量变化。','',
           '全部条件：每个算法单元格为目标值（秒）；—为尚无结果。SCIP运行中数值为实时暂存解。',
           '| 条件 | n | r最大/均值 | Δ最大/均值 | W实测比值 | SCIP | Streaming | Greedy | ILS1000 | SCIP状态 |',
           '|---|---:|---:|---:|---:|---:|---:|---:|---:|---|']
    for r in rows:
        st=r['metrics'];cells=[]
        for m in METHODS:
            z=r['methods'][m];cells.append(n(z.get('objective'))+'（'+n(z.get('total_seconds'),6)+'）')
        # Reorder for human comparison.
        lines.append(f"| {r['scenario_id']} | {st['n']} | {st['rank_max']}/{st['rank_mean']:.2f} | {st['degree_max']}/{st['degree_mean']:.2f} | {st['weight_ratio']:.2f} | {cells[3]} | {cells[0]} | {cells[1]} | {cells[2]} | {r['methods']['SCIP']['status']} |")
    lines+=['','已完成配对的单因素汇总；3个配对种子均完成后才计算组中位数。',
            '适用计数：目标至少为SCIP参考解的95%，且总时间至少快2倍；此95%不表示距最优仅5%。',
            '| 因素 | 水平 | 方法 | 有效配对/3 | 目标相对差中位数% | 差值范围% | 速度比中位数 | 适用/3 |',
            '|---|---:|---|---:|---:|---:|---:|---:|']
    for g in groups:
        lines.append(f"| {g['factor']} | {g['level']} | {g['method']} | {g['valid_completed_pairs']}/3 | {n(g['median_relative_difference_percent'],3)} | {n(g['min_relative_difference_percent'],3)} 至 {n(g['max_relative_difference_percent'],3)} | {n(g['median_speed_ratio'],2)} | {n(g['quality95_speed2x_count'])}/3 |")
    lines+=['','逐条件差距：差值=超图算法−SCIP；相对差分母为|SCIP|。',
            '| 条件 | 方法 | 目标差 | 相对差% | SCIP/本方法耗时 | 基于SCIP上界的本方法gap% | 比较状态 |',
            '|---|---|---:|---:|---:|---:|---|']
    for r in rows:
        for method,c in r['comparisons'].items():
            label='完整参考' if c['reference_complete_gap05'] else '暂存／未达标'
            gap=c['native_certified_gap_from_scip_bound']
            lines.append(f"| {r['scenario_id']} | {method} | {n(c['objective_difference'])} | {n(c['relative_difference_percent'],3)} | {n(c['scip_seconds_divided_by_native_seconds'],2)} | {n(None if gap is None else 100*gap,3)} | {label} |")
    lines+=['','公共数据干预与解释边界：',
            'n：同一最高度资源出发的嵌套BFS子图4096/16384/65536变量，保留选中超边全部关联；保持原作者输入顺序。',
            'r：16384变量子图按固定哈希次序截取至8/32个关联；这是派生任务，约束度也会变化。',
            'Δ：将高度资源拆成每份最多32/128个变量的资源；该干预放松冲突，其他结构变化如表列出。',
            'W：保留原权重次序／并列关系，改成1或1000范围；另统一乘10，校验纯数值尺度不改变数学最优选择。',
            '32、33原始511685变量全实例沿用既有SCIP队列及原始Streaming输出，本批不重复启动。',
            '本报告从实际原始输出重建；观测时间见报告首行。没有后台Codex或离线人工监督。',
            '证据仅用于界定局部适用区间，所有反例、失败和未完成条件保留；不保证超图算法胜出。','',
            '论文与源码：',
            'Streaming：Reinstädtler et al., Semi-Streaming Algorithms for Hypergraph Matching, ESA2025,正式发表2025-10-01. https://doi.org/10.4230/LIPIcs.ESA.2025.79',
            'Greedy/ILS：Großmann et al., Engineering Hypergraph b-Matching Algorithms, JGAA30(1),正式发表2026-01-17. https://doi.org/10.7155/jgaa.v30i1.3166',
            '来源与固定commit继承算法论文出版信息.json；没有改变原作者搜索机制。']
    publish_text(OUT/'压力测试报告.txt', '\n'.join(lines)+'\n')
    return report


if __name__=='__main__':
    r=summarize();print({k:r[k] for k in ['observed_at','completed_runs','failed_runs','scip_gap05_complete']})
