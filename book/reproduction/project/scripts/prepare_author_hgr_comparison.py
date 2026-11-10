"""Prepare complete published weighted hypergraphs, preserving original member order."""
from pathlib import Path
import datetime,json,shutil,collections
from run_hypergraph_scip_demo import ROOT,load,save,sha

def parse_hgr(path):
    with path.open(encoding='utf-8') as f:
        m,n,fmt=map(int,f.readline().split());assert fmt==11
        edges=[];weights=[];degrees=[0]*n
        for _ in range(m):
            row=list(map(int,f.readline().split()));assert row and 1<=row[0]<=100
            pins=row[1:];assert pins and len(set(pins))==len(pins) and all(1<=v<=n for v in pins)
            weights.append(row[0]);edges.append(pins)
            for v in pins:degrees[v-1]+=1
        vertex_weights=[int(f.readline().strip()) for _ in range(n)]
        assert not f.read().strip(),'Unexpected trailing data'
    return n,edges,weights,degrees,vertex_weights

def main():
    base=ROOT/'总结/07_证据与复现索引/超图建模与原生求解/公开数据'
    selected=load(base/'public_selection_frozen.json')['rows'];receipt=load(base/'JGAA2026/extraction_receipt.json')
    old=ROOT/'总结/03_实验数据/超图与SCIP组合任务对比';out=old/'论文原始实例_匹配2025_2026'
    assert not (out/'protocol.json').exists();(out/'instances').mkdir(parents=True,exist_ok=True)
    rows=[];audits=[]
    for spec in selected:
        filename=spec['graph']+'.hgr.random100';member=next(r for r in receipt['full_members'] if Path(r['name']).name==filename)
        p=ROOT/member['path'];assert sha(p)==member['sha256'];n,edges,weights,degree,original_vw=parse_hgr(p)
        stats={'vertices':n,'edges':len(edges),'pins':sum(map(len,edges)),'rmax':max(map(len,edges))}
        assert stats==dict(zip(['vertices','edges','pins','rmax'],[int(spec[k]) for k in ['HNs','HEs','pins','maxHEsize']]))
        for year in ([2026,2025] if spec['family']=='DAC2012' else [2026]):
            sid=f"paper{year}_"+spec['graph'].replace('.','_')
            vals=weights if year==2026 else [stats['rmax']+1-len(e) for e in edges]
            data={'scenario_id':sid,'problem':'maximum weighted hypergraph matching','source':member,
                  'paper':'10.7155/jgaa.v30i1.3166' if year==2026 else '10.4230/LIPIcs.ESA.2025.79',
                  'topology_source':'Original complete M_HG member, with graph name, vertex count, edge count, pins and maximum rank matched to original M_HG statistics; DAC2012 also appears in L_HG.',
                  'weight_source':'Original archived random100 edge weights unchanged' if year==2026 else 'Reconstructed deterministically from the pinned ESA2025 author make_uniform_negative.cc: rmax+1-size(e). Paper prose omits+1; original generated weighted file not available.',
                  'capacity_source':'Uniform1, explicit experimental condition in JGAA2026 section4, or ordinary matching in ESA2025. Archived random vertex weights retained in original file and not used for this condition.',
                  'resources':[{'id':'R'+str(i+1),'capacity':1} for i in range(n)],
                  'jobs':[{'id':'E'+str(i+1),'resources':['R'+str(v) for v in e],'value':w} for i,(e,w) in enumerate(zip(edges,vals))]}
            f=out/'instances'/(sid+'.json');f.write_text(json.dumps(data,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
            rows.append({'scenario_id':sid,'n':len(edges),'resources':n,'r':stats['rmax'],'seed':None,'incidences':stats['pins'],
                        'mean_rank':stats['pins']/len(edges),'mean_variables_per_capacity_constraint':stats['pins']/n,
                        'max_variables_per_capacity_constraint':max(degree),'min_variables_per_capacity_constraint':min(degree),
                        'family':spec['family'],'paper_year':year,'path':f.relative_to(ROOT).as_posix(),'sha256':sha(f),
                        'source_path':member['path'],'source_sha256':member['sha256'],'weight_source':data['weight_source']})
            audits.append({'scenario_id':sid,'original_header_and_all_edge_weights_incidence_order_preserved':year==2026,
                           'complete_topology_preserved':True,'published_statistics_all_match':stats,'weight_min':min(vals),'weight_max':max(vals),
                           'capacity_condition':1,'original_random_capacity_range':[min(original_vw),max(original_vw)]})
            print(sid,stats,flush=True)
    p=load(old/'复杂度扩展/protocol.json');now=datetime.datetime.now(datetime.timezone.utc).isoformat()
    p.update(experiment_id='published-matching2025-2026-comparison',status='prepared_authorized',created_at=now,instances=len(rows),
             scope='Seven complete JGAA2026 benchmark members with unchanged published weights, capacity1; one shared full L_HG DAC2012 topology with ESA2025 public-code weight preprocessing. Same two fixed methods.',
             question='Observe quality, proof status and runtime on complete original paper scenarios including high rank and a half-million-variable problem.',
             specifications=[{'id':r['scenario_id'],'n':r['n'],'resources':r['resources'],'rank':r['r']} for r in rows],seeds=[],
             repetitions='One paired run per frozen instance and weight condition; no result-driven selection or tuning.',
             pairing='Original hyperedge order, complete original topology, same paired weights; no reduction, subsampling or warm start.',
             interpretation_limits=['Representative size-based subset, not full-paper reproduction.','ESA2025 weights regenerated exactly by author public source, with documented paper-code +1 discrepancy.','Only b=1 conditions; b=3,5,random are incompatible with this fixed native solver.','Rank maxima can be rare outliers; average rank and constraint degrees are reported.'],
             review_reason='Current explicit user instruction requests original 2025-26 paper scenarios; pre-result size-based selection and data-source resolution preserved.')
    p['budget'].update(solver_time_ceiling_seconds=80,batch_wall_guard_seconds=300,peak_extra_write_reservation_MiB=600)
    save(out/'protocol.json',p);save(out/'input_manifest.json',{'observed_at':now,'instances':rows,'adapter_sha256':sha(Path(__file__))})
    save(out/'representation_audit.json',{'observed_at':now,'rows':audits,'source_archive_md5':receipt['archive_md5_verified'],'capacity_source_pdf':'bmatching_jgaa2026.pdf physicalpage13 section4','esa_weight_source_resolution':str(base/'esa2025_source_resolution.json')})
    save(out/'experiment_identity.json',{'experiment_id':p['experiment_id'],'status':'prepared_authorized','pid':None,'completed_cells':0,'created_at':now})
    shutil.copyfile(old/'复杂度扩展/runner_regression.json',out/'runner_regression.json')

if __name__=='__main__':main()
