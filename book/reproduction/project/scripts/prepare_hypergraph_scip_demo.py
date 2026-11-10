"""Freeze a single illustrative instance and adapt unchanged author kernels.

The adapter changes I/O only. It is not a reimplementation of a baseline.
"""
from pathlib import Path
import datetime, hashlib, json

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '总结/03_实验数据/超图与SCIP组合任务对比'
AUTHOR = ROOT / '总结/07_证据与复现索引/超图建模与原生求解/Streaming_author'

def save(p, obj):
    p.write_text(json.dumps(obj, ensure_ascii=False, indent=2), encoding='utf-8')

def body(src, marker, start=0):
    a = src.index(marker, start)
    a = src.index('{', a)
    depth = 1
    b = a + 1
    while depth:
        depth += (src[b] == '{') - (src[b] == '}')
        b += 1
    return src[a:b]

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    if (OUT / 'instance.json').exists():
        raise SystemExit('Frozen instance already exists; do not overwrite after observation.')
    tasks = [
        ('T01','ABCDEF',19), ('T02','ABG',12), ('T03','CDH',11),
        ('T04','EFI',10), ('T05','GHIJKL',20), ('T06','AJK',9),
        ('T07','BHL',9), ('T08','CGI',8), ('T09','DJK',9),
        ('T10','EHL',8), ('T11','FGI',8), ('T12','JKL',9),
    ]
    instance = {
        'scenario_id':'bundle_allocation_12',
        'provenance':'Manually constructed pedagogical data; not measured inspection data or a published benchmark.',
        'meaning':'One scheduling window; each resource has one indivisible unit. A job is valuable only if its complete resource bundle is allocated. Maximize sum of accepted job values.',
        'resources':[{'id':v,'capacity':1,'unit':'indivisible resource unit'} for v in 'ABCDEFGHIJKL'],
        'jobs':[{'id':i,'resources':list(v),'value':w,'value_unit':'dimensionless illustrative benefit'} for i,v,w in tasks],
        'stream_order':[i for i,_,_ in tasks],
    }
    save(OUT/'instance.json', instance)
    protocol = {
        'frozen_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'experiment_id':'hypergraph-scip-bundle-demo',
        'instance_sha256':hashlib.sha256((OUT/'instance.json').read_bytes()).hexdigest(),
        'claim':'Observe exact-MIP versus direct hypergraph streaming tradeoff on one illustrative case; no superiority, scalability or significance claim.',
        'data_rule':'Exactly this one instance; no outcome-driven instance/weight/order/epsilon search.',
        'scip':{'time_limit_s':10,'max_threads':1,'random_seed_shift':0,'other_settings':'defaults; no deliberate presolve/heuristic disabling'},
        'hypergraph':{'repository':'https://github.com/HeiHGM/Streaming','commit':'f2f2af98d13d34b47cde6ce3d096c4240dcc5a5d','method':'GreedyMatchingStreaming, delta_g primal-dual stack, single pass','eps':0.1,'order':'fixed input order; author no_shuffle setting','precision':'original float32 potentials and original comparison expression','postprocessing':'original reverse-stack feasible matching; no second pass or repair'},
        'run_plan':{'quality_runs':1,'timing_repeats':21,'timing_repeats_are_independent_instances':False,'timing_order':'alternate first method; new SCIP model and new native state each repetition','total_local_solver_budget_s':300,'external_timeout_s':10},
        'timing':'Report build/solve/output total separately from kernel time; exclude imports, file I/O, log export and exhaustive evaluator. No claim of implementation-neutral speedup.',
        'reference':'Independent exhaustive enumeration of all 4096 job subsets, not supplied to either solver.',
        'failure_rule':'Retain errors and all repetitions; invalid or timed-out output does not become a feasible result.',
        'scope':'New local teaching demonstration explicitly requested by user; existing paper pauses and review nodes unchanged; no own research algorithm.',
    }
    save(OUT/'protocol.json', protocol)
    src=(AUTHOR/'app/algorithms/streaming/matching/greedy.cc').read_text(encoding='utf-8')
    start=src.index('class GreedyMatchingStreaming :')
    stream=body(src, 'void Stream(', start)
    transform=body(src, 'auto Transform(HS<StreamableHypergraph, DT>& storage)')
    # Preserve original algorithm bodies byte-for-byte; only template/I/O scaffolding differs.
    header=(AUTHOR/'ds/streaming/matching.h').read_text(encoding='utf-8')
    adapted_header=header.replace('  bool valid() {',
        '  const auto& observed_edges() const { return matched_edges; }\n  bool valid() {',1)
    (OUT/'author_matching_with_output_accessor.h').write_text(adapted_header,encoding='utf-8')
    cpp=r'''#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
#include "author_matching_with_output_accessor.h"
struct Edge { std::vector<size_t> pins; int w; int id; };
struct Graph { using NodeType=size_t; using EdgeType=size_t; using WeightType=int; using StreamMember=Edge; };
template<class G, typename DT=float> struct Storage {
  std::vector<DT> phi; std::vector<typename G::StreamMember> stack;
  size_t stack_size() const { return stack.size(); }
};
using ResultType=heihgm::ds::streaming::MatchingResult<Graph>;
using FloatType=float;
void Stream(const Edge& member, Storage<Graph>& storage, FloatType eps) STREAM_BODY
template<class StreamableHypergraph, typename DT>
auto Transform(Storage<StreamableHypergraph,DT>& storage) TRANSFORM_BODY
extern "C" __declspec(dllexport)
int solve(int n, int m, const int* ptr, const int* pins, const int* weights,
          float eps, int* selected, double* elapsed) {
  const auto t0=std::chrono::steady_clock::now();
  Storage<Graph> storage; storage.phi.assign(n, 0.0f);
  for(int e=0;e<m;++e) {
    Edge member; member.id=e; member.w=weights[e];
    for(int k=ptr[e];k<ptr[e+1];++k) member.pins.push_back(pins[k]);
    Stream(member,storage,eps);
  }
  auto result=Transform(storage);
  const auto t1=std::chrono::steady_clock::now();
  *elapsed=std::chrono::duration<double>(t1-t0).count();
  int count=0;
  for(const auto& edge:result->observed_edges()) selected[count++]=edge.id;
  return count;
}
'''
    # Author uses an alias in the original namespace. Keep that template name.
    cpp=cpp.replace('using ResultType=', 'template<class G> using MatchingResult=heihgm::ds::streaming::MatchingResult<G>;\nusing ResultType=')
    cpp=cpp.replace('STREAM_BODY',stream).replace('TRANSFORM_BODY',transform)
    (OUT/'author_streaming_adapter.cpp').write_text(cpp,encoding='utf-8')
    save(OUT/'adapter_audit.json',{
        'author_source_sha256':hashlib.sha256((AUTHOR/'app/algorithms/streaming/matching/greedy.cc').read_bytes()).hexdigest(),
        'stream_body_sha256':hashlib.sha256(stream.encode()).hexdigest(),
        'transform_body_sha256':hashlib.sha256(transform.encode()).hexdigest(),
        'stream_body_verbatim':stream in cpp,'transform_body_verbatim':transform in cpp,
        'matching_header_only_change':'Read-only observed_edges accessor to export selected edge IDs; original feasibility and selection bodies unchanged.',
        'adapter':'C ABI, CSR input, persistent original edge ID, monotonic timer; original float arithmetic; no extra solver/search/repair.',
        'not_official_full_cli':True,
        'environment_obstacle':'Author wheels support Linux/macOS; existing WSL returned CreateInstance/0xd0000022. Use original C++ kernels with portable compiler, not reimplement search.',
        'license':'MIT; retained next to original source',
    })
    print('Frozen one instance and generated byte-preserved author kernel adapter.')

if __name__=='__main__':main()
