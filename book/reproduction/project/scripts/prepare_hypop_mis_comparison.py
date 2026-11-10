"""Exact incidence reduction of untouched author MIS graphs; no solver changes."""
from pathlib import Path
import datetime,json,shutil
from run_hypergraph_scip_demo import ROOT,load,save,sha

def main():
    base=ROOT/'总结/07_证据与复现索引/超图建模与原生求解/公开数据/HypOp_Reuse'
    old=ROOT/'总结/03_实验数据/超图与SCIP组合任务对比'
    out=old/'论文原始实例_HypOp2025';assert not (out/'protocol.json').exists()
    (out/'instances').mkdir(parents=True,exist_ok=True);rows=[]
    sources=load(base/'source_manifest.json')['files']
    for degree in [3,5]:
        log=(base/f'maxind_syn_d{degree}_ep2w_it20.log').read_text(encoding='utf-8')
        for n in [500,1000,2000]:
            file=base/f'G_reg_{n}_{degree}.txt';lines=file.read_text().splitlines();nv,ne=map(int,lines[0].split());assert n==nv
            edges=[tuple(map(int,l.split())) for l in lines[1:] if l.strip()];assert len(edges)==ne and len(set(tuple(sorted(e)) for e in edges))==ne
            assert all(1<=a<=n and 1<=b<=n and a!=b for a,b in edges)
            incident=[[] for _ in range(n)]
            for i,(a,b) in enumerate(edges):incident[a-1].append('R'+str(i));incident[b-1].append('R'+str(i))
            assert all(len(x)==degree for x in incident)
            assert file.name in log,'Author experiment log must name the original instance'
            sid=f'hypop2025_mis_n{n}_d{degree}'
            source=next(s for s in sources if s['original_path'].endswith(file.name))
            data={'scenario_id':sid,'problem':'maximum independent set, exactly represented as hypergraph matching',
                  'source':source,'paper':'10.1038/s42256-025-01141-4','data_role':'original public author instance; exploratory new solver comparison, not HypOp rerun',
                  'resources':[{'id':'R'+str(i),'capacity':1} for i in range(ne)],
                  'jobs':[{'id':'V'+str(i+1),'resources':rs,'value':1} for i,rs in enumerate(incident)],
                  'mapping':'Original vertex v -> job Vv; original graph edge (u,v) -> capacity1 resource used exactly by jobs Vu,Vv. Independent iff matching, same cardinality. Vertex-ID order ascending.'}
            p=out/'instances'/(sid+'.json');save(p,data)
            rows.append({'scenario_id':sid,'n':n,'r':degree,'resources':ne,'seed':None,'incidences':n*degree,'path':p.relative_to(ROOT).as_posix(),'sha256':sha(p),'source_sha256':sha(file),'max_variables_per_capacity_constraint':2,'min_variables_per_capacity_constraint':2,'mean_variables_per_capacity_constraint':2})
    p=load(old/'复杂度扩展/protocol.json')
    p.update(experiment_id='hypop2025-original-MIS-comparison',status='prepared_authorized',created_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
             scope='Same two methods on six complete author-published MIS instances under exact incidence reduction; no HypOp training, no change to original objective or constraints',
             question='Quality and runtime of SCIP and fixed original stack algorithm on published MIS instances',instances=len(rows),specifications=[{'nodes':n,'degree':d} for d in [3,5] for n in [500,1000,2000]],seeds=[],
             repetitions='One paired solve per original public graph; author data not regenerated; no inferential statistics',pairing='Exact same unit objective and pairwise incompatibilities; vertex order ascending',
             interpretation_limits=['These original scenarios are graphs; incidence rank3/5 is not irreducible higher-order coupling.','This compares fixed SCIP and Streaming, not the HypOp neural method.'],review_reason='Explicit current user request includes original 2025-26 paper scenarios; bounded representative selection before any results.')
    p['budget'].update(solver_time_ceiling_seconds=60,batch_wall_guard_seconds=180)
    save(out/'protocol.json',p);save(out/'input_manifest.json',{'observed_at':p['created_at'],'instances':rows,'adapter_sha256':sha(Path(__file__)),'source_manifest':str(base/'source_manifest.json')})
    save(out/'experiment_identity.json',{'experiment_id':p['experiment_id'],'status':'prepared_authorized','pid':None,'completed_cells':0,'created_at':p['created_at']})
    shutil.copyfile(old/'复杂度扩展/runner_regression.json',out/'runner_regression.json')
    save(out/'representation_audit.json',{'bijection_proof':data['mapping'],'all_six_graph_headers_edge_counts_degrees_and_author_logs_checked':True,'capacity_constraints':2,'full_graphs_no_subsampling':True,'objective':'original independent-set cardinality','input_source_commit':'953916404c59aca208ab33d2132d1a97d3b02300','source_to_transformed_output_checks_required':True})
    print(json.dumps({'instances':len(rows),'output':str(out)},ensure_ascii=False))

if __name__=='__main__':main()
