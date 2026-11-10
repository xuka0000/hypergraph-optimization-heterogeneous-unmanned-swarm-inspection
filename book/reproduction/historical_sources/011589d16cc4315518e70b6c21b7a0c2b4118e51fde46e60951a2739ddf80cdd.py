"""Same full regional domain; a bounded, explicitly experimental message run."""
from pathlib import Path
import argparse,datetime,hashlib,json,os,sys,time,traceback,gzip
os.environ.setdefault('OPENBLAS_NUM_THREADS','1');os.environ.setdefault('OMP_NUM_THREADS','1')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
import psutil
from Agent.Publication.engineering_conditional_hypergraph import RecordingModel,FactorConsensus,expand_selected_relations
from Agent.Publication.engineering_regional_solver import RegionalJointModel,stamp
import Agent.Publication.engineering_regional_structure as structure


def write(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def resources():
    p=psutil.Process();r=dict(available_RAM_GiB=psutil.virtual_memory().available/2**30,rss_GiB=p.memory_info().rss/2**30,
                            private_GiB=p.memory_info().private/2**30,free_disks_GiB={d:psutil.disk_usage(d+':/').free/2**30 for d in 'CD'})
    r['allowed']=r['available_RAM_GiB']>=2 and r['private_GiB']<=6 and r['free_disks_GiB']['C']>=10.5 and r['free_disks_GiB']['D']>=30.5
    return r


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--sweeps',type=int,default=200);args=parser.parse_args()
    if args.sweeps<=0:raise ValueError('Positive finite engineering sweep budget required')
    validation=ROOT/'research_quality/inspection_conditional_hypergraph_validation.json'
    vr=json.loads(validation.read_text('utf-8-sig'));code=ROOT/'Agent/Publication/engineering_conditional_hypergraph.py'
    if not vr['passed'] or vr['solver_file_sha256']!=sha(code):raise RuntimeError('Actual message/update regression for this code is required')
    # This is same-stage diagnostic execution, not a formal comparison or a
    # transition to own_method/formal_evaluation preflight. No comparison claim.
    output=ROOT/'总结/03_实验数据/工程化联合求解/conditional_hypergraph';output.mkdir(exist_ok=True,parents=True)
    mp=output/'run_manifest.json'
    if mp.exists():
        prior=json.loads(mp.read_text('utf-8-sig'))
        try:
            p=psutil.Process(prior['pid'])
            if abs(p.create_time()-prior['process_create_time'])<.01:raise RuntimeError('Existing hypergraph attempt live')
        except psutil.NoSuchProcess:pass
        with (output/'attempt_history.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(prior,ensure_ascii=False)+'\n')
    before=resources()
    if not before['allowed'] or before['available_RAM_GiB']<6:raise MemoryError('Insufficient coexistence headroom '+str(before))
    files=sorted((ROOT/'Agent/Publication').glob('engineering_*.py'))+[Path(__file__),ROOT/'research_quality/inspection_regional_model_data.json',ROOT/'research_quality/inspection_conditional_hypergraph_protocol.json']
    identities=[dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p)) for p in files]
    run_id=sha(code)[:12]+'_'+str(os.getpid());start=time.perf_counter()
    manifest=dict(task_id='inspection_conditional_hypergraph_solver',run_id=run_id,pid=os.getpid(),process_create_time=psutil.Process().create_time(),
                  started_at=stamp(),status='building',user_authorization='开始尝试！议采用“带生效条件的变量—约束超图”，并用超图算法求解！',
                  method='experimental inexact factor-consensus message passing',public_baseline_reproduction=False,formal_comparison=False,
                  sweeps_budget=args.sweeps,wall_limit_s=None,SCIP_optimize_called=False,files=identities,resources_at_start=before,
                  original_SCIP_run_preserved=True,storage_reservation_GiB=.5,host_RAM_floor_GiB=2,process_private_cap_GiB=6,
                  accepted_objective_vector=None,accepted_checks=0,global_optimality_claim=False,
                  initialization='continuous clip(0.001,original bounds), integer0 clipped; not a feasible seed',
                  final_original_domains_synchronized=True)
    write(mp,manifest)
    with gzip.open(output/('source_'+run_id+'.json.gz'),'wt',encoding='utf-8') as f:
        json.dump({x['path']:p.read_text('utf-8-sig') for x,p in zip(identities,files)},f,ensure_ascii=False)
    old_model=structure.Model;j=None;engine=None;best=None;seen=set();history=[]
    try:
        structure.Model=RecordingModel
        j=RegionalJointModel(ROOT,output,compact_memory=True);j.run_id=run_id;j.build();hooks=j.exercise_required_row_hooks()
        structure.Model=old_model
        m=j.path.m;t=m.tape
        if m.getNVars()!=869442 or m.getNConss()!=394966:raise ValueError('Unexpected bound model identity/count change')
        if len(t.names)!=m.getNConss():raise ValueError('Missing explicit algebra factors')
        manifest.update(build_wall_s=time.perf_counter()-start,variables=m.getNVars(),explicit_rows=m.getNConss(),indexed_relations=len(j.path.rel.relations),
                        towers=len(j.path.towers),photo_tasks=len(j.path.tasks),initial_hypergraph=t.counts(),hooks=hooks,
                        model_sha256=j.record['model']['sha256'],source_sha256=j.record['source']['sha256'])
        print(json.dumps(dict(event='built',**manifest['initial_hypergraph'],build_wall_s=manifest['build_wall_s'])),flush=True)
        write(mp,manifest)
        if not resources()['allowed']:raise MemoryError('Postbuild coexistence resource guard')
        engine=FactorConsensus(m);engine.set_objective(j.path.objective)
        if any(engine.lb[v.getIndex()]!=v.getLbOriginal() or engine.ub[v.getIndex()]!=v.getUbOriginal() for v in engine.variables):
            raise ValueError('Original domain mismatch')
        manifest['original_bound_audit']=dict(variables_checked=len(engine.variables),all_equal=True,
            fixed_variables=int(sum(engine.lb==engine.ub)),integer_variables=int(sum(engine.integer)))
        # Loose auxiliary bounds can reach1e12; their midpoint is not a
        # physically informed seed. A small positive, bound-respecting start
        # avoids that manufactured residual without changing any constraint.
        physical=engine.physical();continuous=~engine.integer
        physical[continuous]=np.clip(.001,engine.lb[continuous],engine.ub[continuous])
        engine.z=physical/engine.scale
        with gzip.open(output/('variables_'+run_id+'.json.gz'),'wt',encoding='utf-8') as f:json.dump(t.varnames,f)
        np.savez_compressed(output/('incidence_'+run_id+'.npz'),row_ptr=np.asarray(t.pinptr),variable_ids=np.asarray(t.ids),enabled=np.asarray(t.enabled))
        manifest['status']='message_iterations';write(mp,manifest)
        def trace(event,extra=None):
            met=engine.metrics();met.pop('accepted_objective_vector',None)
            row=dict(observed_at=stamp(),run_id=run_id,elapsed_s=time.perf_counter()-start,sweep=engine.iteration,event=event,
                **met,**(extra or {}),resources=resources(),materialized_rows=len(t.names),physical_integrals=len(j.physics.current_orders),
                selected_relation_categories=len(seen),accepted_checks=manifest['accepted_checks'],accepted_objective_vector=manifest['accepted_objective_vector'])
            with (output/'progress.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
            write(output/'status.json',row);history.append(row)
            print(json.dumps({k:row[k] for k in ['event','sweep','elapsed_s','explicit_violated_rows','undefined_expression_rows','candidate_objective_unvalidated','accepted_checks']}),flush=True)
            return row
        current=trace('initial_iterate')
        for it in range(args.sweeps):
            if not resources()['allowed']:
                manifest['status']='resource_guard';break
            msg=engine.sweep()
            # Activate valid conditional factors in repeated batches. Neither
            # a selected road nor a predecessor is fixed between sweeps.
            if (it+1)%10==0:
                ex=expand_selected_relations(j,engine,seen)
                current=trace('message_update',dict(**msg,**ex))
                merit=(current['undefined_expression_rows'],current['explicit_violated_rows'],current['linearized_scaled_violation_norm'])
                if best is None or merit<best:
                    best=merit;np.savez_compressed(output/('best_unvalidated_'+run_id+'.npz'),values=engine.physical(),sweep=engine.iteration)
                if current['explicit_violated_rows']==0 and current['undefined_expression_rows']==0:
                    sol=engine.solution()
                    # Native algebra and original indexed/continuous checking;
                    # evaluation never repairs the candidate.
                    try:
                        j.guard.in_algebra_check=True
                        try:algebra=bool(m.checkSol(sol,printreason=False,completely=True,checkbounds=True,checkintegrality=True))
                        finally:j.guard.in_algebra_check=False
                        accepted=j.acceptance.check(sol) if algebra else dict(passed=False,algebra_failed=True)
                        write(output/('acceptance_'+run_id+'_'+str(it+1)+'.json'),accepted)
                        if accepted['passed']:
                            vec=[float(m.getSolVal(sol,j.path.objective)),float(m.getSolVal(sol,j.path.C)),float(m.getSolVal(sol,j.resources.total_energy))]
                            manifest['accepted_checks']+=1
                            if manifest['accepted_objective_vector'] is None or vec<manifest['accepted_objective_vector']:
                                manifest['accepted_objective_vector']=vec
                                np.savez_compressed(output/('accepted_'+run_id+'.npz'),values=engine.physical(),objective=vec)
                        elif accepted.get('requests'):
                            added=j.materialize(accepted['requests'])
                            if added:engine.refresh();engine.set_objective(j.path.objective)
                    finally:m.freeSol(sol)
            if (it+1)%50==0:
                np.savez_compressed(output/('checkpoint_'+run_id+'.npz'),z=engine.z,u=engine.u,iteration=engine.iteration)
                manifest.update(completed_sweeps=engine.iteration,observed_at=stamp());write(mp,manifest)
        else:manifest['status']='engineering_batch_completed'
        final=trace('batch_returned');manifest.update(completed_sweeps=engine.iteration,final_metrics=final,final_materialized_hypergraph=t.counts(),
                    result='accepted_candidate' if manifest['accepted_checks'] else 'no_accepted_full_region_plan',
                    full_region_feasibility_established=bool(manifest['accepted_checks']))
    except Exception as exc:
        manifest.update(status='engineering_error',error=dict(type=type(exc).__name__,message=str(exc),traceback=traceback.format_exc()))
        print(traceback.format_exc(),flush=True)
    finally:
        structure.Model=old_model;manifest.update(observed_at=stamp(),elapsed_s=time.perf_counter()-start,resources_at_end=resources());write(mp,manifest)
        if j is not None and hasattr(j,'path'):j.path.m.freeProb()
    return 0 if manifest['status']=='engineering_batch_completed' else 1
if __name__=='__main__':raise SystemExit(main())
