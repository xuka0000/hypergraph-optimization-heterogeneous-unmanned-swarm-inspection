"""Read-only source/model/result audit; write derived user report, never optimize."""
from pathlib import Path
import hashlib,json,re,statistics,datetime,shutil
from prepare_hypergraph_scip_demo import body
from run_hypergraph_scip_demo import ROOT,OUT,evaluate,enumerate_reference,load,save,sha

def main():
    data=load(OUT/'instance.json');protocol=load(OUT/'protocol.json');results=load(OUT/'results.json')
    rows=load(OUT/'raw_runs.json');reference=enumerate_reference(data)
    assert sha(OUT/'instance.json')==protocol['instance_sha256']==results['instance_sha256']
    assert reference==results['reference']
    assert len(rows)==44
    for row in rows:
        ev=evaluate(data,row['selected']);assert ev==row['validation'] and ev['feasible']
        assert row['input_sha256']==results['instance_sha256']
        if row['method']=='SCIP_MILP':
            assert row['solver_primal_bound']==row['solver_dual_bound']==reference['objective']
            assert row['status']=='optimal' and row['solver_gap']==0
    for s in results['summary']:
        timed=[r for r in rows if r['phase']=='timing' and r['method']==s['method']]
        assert s['total_median_ms']==1000*statistics.median(r['total_seconds'] for r in timed)
        assert s['solver_median_ms']==1000*statistics.median(r['solver_seconds'] for r in timed)
    author=ROOT/'总结/07_证据与复现索引/超图建模与原生求解/Streaming_author'
    original=(author/'app/algorithms/streaming/matching/greedy.cc').read_text(encoding='utf-8')
    adapter=(OUT/'author_streaming_adapter.cpp').read_text(encoding='utf-8')
    assert body(original,'void Stream(',original.index('class GreedyMatchingStreaming :'))==body(adapter,'void Stream(')
    assert body(original,'auto Transform(HS<StreamableHypergraph, DT>& storage)')==body(adapter,'auto Transform(Storage<StreamableHypergraph,DT>& storage)')
    expected_header=(author/'ds/streaming/matching.h').read_text(encoding='utf-8').replace('  bool valid() {','  const auto& observed_edges() const { return matched_edges; }\n  bool valid() {',1)
    assert expected_header==(OUT/'author_matching_with_output_accessor.h').read_text(encoding='utf-8')
    lp=(OUT/'model.lp').read_text(encoding='utf-8')
    objective=lp.split(' Obj: ')[1].split('\nSubject to')[0]
    parsed_objective={k:int(v) for v,k in re.findall(r'\+(\d+) (T\d+)',objective)}
    assert parsed_objective=={j['id']:j['value'] for j in data['jobs']}
    capacity_rows={}
    for resource in data['resources']:
        line=next(line for line in lp.splitlines() if line.startswith(' capacity_'+resource['id']+':'))
        lhs,rhs=line.split('<=');expected={j['id'] for j in data['jobs'] if resource['id'] in j['resources']}
        assert set(re.findall(r'T\d+',lhs))==expected and int(rhs)==resource['capacity']
        assert all(int(v)==1 for v in re.findall(r'\+(\d+) T\d+',lhs))
        capacity_rows[resource['id']]=sorted(expected)
    duplicates=[]
    for a,va in capacity_rows.items():
        for b,vb in capacity_rows.items():
            if a<b and va==vb:duplicates.append([a,b])
    arities=[len(j['resources']) for j in data['jobs']]
    assert sum(arities)==results['incidences'] and max(arities)==results['r_max']
    validation={'observed_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'input_and_source_hash_verified':True,'public_stream_and_transform_bodies_identical':True,
        'matching_class_only_readonly_accessor_added':True,'LP_objective_and_each_constraint_match_JSON':True,
        'raw_rows_checked':len(rows),'all_rows_feasible':True,'exact_reference':reference,
        'timing_summaries_recomputed':True,'duplicate_capacity_rows':duplicates,
        'unique_capacity_row_count':len({tuple(x) for x in capacity_rows.values()}),
        'maximum_variables_per_capacity_constraint':max(map(len,capacity_rows.values())),
        'limitations':['Single synthetic instance; no scaling or population statistics.','Input hyperedge rank concerns resource bundle size; each capacity constraint here contains only 3 or 4 job variables.','J and K capacity rows identical; retained original frozen input, not inflated as independent constraints.','Capacity-one set packing also admits exact conflict-graph independent-set formulation.'],
        'results_sha256':sha(OUT/'results.json'),'raw_sha256':sha(OUT/'raw_runs.json')}
    save(OUT/'delivery_validation.json',validation)
    scip,hg=results['summary']
    lines=['SCIP与超图专用算法：组合任务包分配',
       '', '数据与模型',
       '数据来源：当前会话人工构造的教学样例；非实测巡检数据，非论文公开benchmark。',
       '问题来源：Reinstädtler et al., ESA 2025, §2–3；最大权超图匹配，资源容量b(v)=1。',
       '语义：一个作业窗口，A–L各有1份不可分资源；每项任务必须一次获得整个资源包，才产生收益。',
       '收益单位：无量纲教学分值。各任务收益是输入，不是论文数据或实测经济收益。',
       '', '参数\t数值',
       '资源节点数 |V|\t12','候选任务/超边数 |E|\t12','0–1变量数\t12',
       '容量约束行数\t12（其中J、K两行相同；不同约束行11）',
       '单条容量约束联系的任务变量数\t3–4','超边资源数\t3或6','最大/平均超边阶数\t6 / 3.5',
       '关联项总数\t42','神经隐藏维数\t不适用；两种方法均不使用神经网络',
       '', '任务\t必须同时取得的资源\t收益']
    lines.extend(f"{j['id']}\t{','.join(j['resources'])}\t{j['value']}" for j in data['jobs'])
    lines += ['', '等价数学模型',
       'x_j∈{0,1}：任务j是否被选择；S_j：任务j需要的完整资源集合；w_j：任务收益。',
       'max Σ_j w_j x_j',
       's.t. Σ_{j:r∈S_j} x_j ≤ 1，∀r∈{A,…,L}。',
       '例如资源A：x_01+x_02+x_06≤1；资源G：x_02+x_05+x_08+x_11≤1。',
       '超图表示：资源为节点，每个任务的完整资源包为一条带权超边；选互不相交超边。',
       '该模型也可以转为冲突图的最大权独立集；本例不证明超图具有独占表达能力。',
       '', '方法\tSCIP整数规划\t超图原始—对偶栈',
       '实现\tSCIP10.0.2 + PySCIPOpt6.2.1\tHeiHGM/Streaming作者C++核心，经输入输出适配',
       '求解规则\t默认预求解及搜索；单线程，最多10s\tδ_g更新，ε=0.1；固定输入顺序，单遍+逆序出栈',
       '神经网络/邻域搜索\t无\t无',
       '选择任务\t'+','.join(scip['selected'])+'\t'+','.join(hg['selected']),
       f"总收益\t{scip['objective']}\t{hg['objective']}",
       '资源约束\t全部满足\t全部满足',
       f"相对最优值损失\t0%\t{hg['relative_gap_to_proved_optimum']*100:.4f}%",
       '求解器最优性证明\t有；上下界均42\t无',
       f"求解核心中位耗时/ms\t{scip['solver_median_ms']:.6f}\t{hg['solver_median_ms']:.6f}",
       f"建模/转换+求解+取解中位耗时/ms\t{scip['total_median_ms']:.6f}\t{hg['total_median_ms']:.6f}",
       f"上述总耗时最小–最大/ms\t{scip['total_min_ms']:.6f}–{scip['total_max_ms']:.6f}\t{hg['total_min_ms']:.6f}–{hg['total_max_ms']:.6f}",
       '计时重复次数\t21\t21',
       '', '核验与结果范围',
       '全部4096种任务子集独立穷举；62种可行；唯一最优组合T02+T03+T04+T12，收益12+11+10+9=42。',
       '超图算法选择T01+T05，收益19+20=39，损失3/42=7.1429%。',
       '1个实例；一次质量运行和21次计时重复/方法。计时重复不算独立案例，不报告显著性。',
       '计时排除库加载、文件读写、结果核验；包含每次重新建模或输入转换。Python建模接口与C++核心存在开销差异。',
       '本例体现精确最优与快速近似的取舍；两边都在毫秒级以内或附近，不能外推大规模速度倍率。',
       '6阶说明一次资源整包关系；容量约束实际仅联系3–4个任务变量。它是高阶建模的极小演示，不是复杂度压测。',
       '未修改冻结的任务、收益、顺序或epsilon来寻找有利结果；原巡检的电池/充电/天气/会合量化继续保留待办。',
       '', '来源',
       '[1] Reinstädtler H, Ferdous S M, Pothen A, Uçar B, Schulz C. Semi-Streaming Algorithms for Hypergraph Matching. ESA 2025, LIPIcs351,79:1–79:19. DOI:10.4230/LIPIcs.ESA.2025.79.',
       'https://doi.org/10.4230/LIPIcs.ESA.2025.79',
       '[2] https://github.com/HeiHGM/Streaming/tree/f2f2af98d13d34b47cde6ce3d096c4240dcc5a5d；MIT。',
       '[3] https://www.scipopt.org/；https://pyscipopt.readthedocs.io/。SCIP为开源通用整数规划/约束求解器。',
       '', '复现文件',
       'instance.json：所有输入；protocol.json：求解前固定方案；model.lp：SCIP导出的完整数学模型。',
       'results.json / raw_runs.json：汇总及44条原始运行；scip_solution.sol / scip.log：解及求解记录。',
       'author_streaming_adapter.cpp / author_streaming.dll：作者核心及本地适配编译件。',
       'delivery_validation.json：逐行可行性、LP系数、作者核心逐字对照及计时重算。',
       'scripts/run_hypergraph_scip_demo.py：实际入口含绑定的前置检查及防重复运行；verify_hypergraph_scip_demo.py：只读核验。',
    ]
    (OUT/'对比报告.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps({'verified_rows':len(rows),'reference':reference,'duplicate_capacity_rows':duplicates,'report':str(OUT/'对比报告.txt')},ensure_ascii=False))

if __name__=='__main__':main()
