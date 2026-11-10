"""Known-problem checks of the production refinement path, not field results."""
from run_engineering_full_joint import ROOT,save,read,stamp,sha
from Agent.Publication.engineering_joint_optimality import solve_lexicographic
from Agent.Publication.engineering_solve_trace import SolveTrace
from Agent.Publication.engineering_joint_safety import witnesses_from_replay,apply_witnesses
from check_unified_model_completion import bare
from pyscipopt import Model
import numpy as np


def main():
    destination=ROOT/'research_quality/inspection_unified_refinement_regression.json'
    if destination.exists():
        history=destination.with_name('inspection_unified_refinement_attempts.json')
        previous=read(history) if history.exists() else []
        previous.append(read(destination));save(history,previous)
    else:previous=[]
    attempt=len(previous)+1;rows=[]
    def check(name,passed,**details):
        rows.append(dict(name=name,passed=bool(passed),**details));print(name,bool(passed),flush=True)
    # The original semi-infinite constraint is
    # x-(1.5-0.5*b)+4*(t-0.5)^2 >= 0 for every t in [0,1].
    # Endpoints admit (b,x,y)=(0,.75,1.25); its true optimum is (1,1,1).
    m=Model();m.hideOutput()
    b=m.addVar('b',vtype='B');x=m.addVar('x',lb=0,ub=2);y=m.addVar('y',lb=0,ub=2)
    m.addCons(x>=.75+.25*b);m.addCons(x+y==2)
    for t in [0.,1.]:m.addCons(x-(1.5-.5*b)+4*(t-.5)**2>=0)
    objectives=[('fleet_makespan_seconds',x,0.),('aircraft_energy_Wh',y,0.)]
    trace_dir=ROOT/f'research_quality/unified_refinement_regression/attempt{attempt}'
    trace=SolveTrace(trace_dir,{'kind':'known_problem'},objectives,{'b':b,'x':x,'y':y},enforce_storage=True)
    m.includeEventhdlr(trace,'known_refinement_trace','known continuous-constraint test')
    candidates=[];used=set();receipts=[]
    def acceptance(sol):
        values=[float(m.getSolVal(sol,v)) for v in [b,x,y]];candidates.append(values)
        ok=values[1]>=1.5-.5*values[0]-1e-7
        return dict(accepted=bool(ok),witnesses=[] if ok else [dict(t=.5)])
    def refine(report):
        added=0
        for row in report['witnesses']:
            t=row['t']
            if t in used:continue
            m.addCons(x-(1.5-.5*b)+4*(t-.5)**2>=0);used.add(t);added+=1
        return added
    result=solve_lexicographic(m,objectives,receipts.append,acceptance,trace,refine)
    trace.close('complete' if result['complete'] else 'unfinished')
    check('same joint variables change after continuous witness',result['complete'] and np.allclose(candidates[0],[0,.75,1.25]) and np.allclose(candidates[-1],[1,1,1]),candidates=candidates)
    check('rejected pass retained before accepted priority',len(result['refinement_history'])==3 and not result['refinement_history'][0]['accepted'] and result['refinement_history'][1]['accepted'],receipt=result)
    import json
    logs=[json.loads(line) for line in (trace_dir/'progress.jsonl').read_text(encoding='utf-8').splitlines()]
    times=[row['solve_elapsed_s'] for row in logs if row.get('solve_elapsed_s') is not None]
    starts=[(row['priority'],row['refinement']) for row in logs if row['event']=='phase_start']
    check('trace preserves refinement and monotone solver time',starts==[(1,0),(1,1),(2,0)] and all(a<=b+1e-9 for a,b in zip(times,times[1:])),starts=starts)
    # Nonempty dicts containing accepted=False must never be treated as True.
    m2=Model();m2.hideOutput();q=m2.addVar('q',lb=0,ub=1)
    unresolved=solve_lexicographic(m2,[('q',q,0.)],lambda _:None,
        lambda _:dict(accepted=False,witnesses=[{'same':True}]),refine=lambda _:0)
    check('no new constraint is unfinished not accepted',not unresolved['complete'] and unresolved['reason']=='unresolved_without_new_constraint',receipt=unresolved)
    flights=[dict(timeline=[dict(model_segment_key='cap_0_0',start_s=10.,end_s=20.)])]
    physical=dict(rows=[dict(slot=0,failures=[dict(segment=0,start_s=12.,end_s=14.,reason='interval_failed_or_unresolved'),dict(reason='source_weather_attitude_jump')])],
        pairwise_separation=[dict(passed=False,model_segment_keys=['cap_0_0','cap_1_0'],overlap_s=[30.,50.],violation=dict(time_s=35.,distance_m=3.))])
    report=witnesses_from_replay(flights,physical)
    points=[w['curve_parameter'] for w in report['witnesses'] if w['kind']=='physics']
    pair=[w for w in report['witnesses'] if w['kind']=='pair'][0]
    check('replay maps normalized points and common overlap time',np.allclose(points,[.2,.3,.4]) and pair['overlap_fraction']==.25 and len(report['unmapped_failures'])==1,report=report)
    # Execute the actual production airspace witness and duplicate handling.
    j=bare();j.n=2;j.cfg={'aircraft':{'body_radius_m':1.,'inter_uav_clearance_m':3.}}
    j.origin=np.zeros(3);j.t={i:j.var(f't{i}',0,20,0) for i in range(2)}
    j.m.addCons(j.t[0]==0);j.x={(i,l):int(i==l) for i in range(2) for l in range(2)};j.segments={}
    for i,position in enumerate([0.,3.]):
        j.segments[f'cap_{i}_0']=dict(C=[[position,0.,20.]]*8,T=10.,active=1,task=i,segment=0,role='cap')
    witness=dict(kind='pair',block_a='cap_0_0',block_b='cap_1_0',overlap_fraction=.5)
    def accept_pair(sol):
        start=float(j.m.getSolVal(sol,j.t[1]))
        return dict(accepted=start>=10.-1e-6,witnesses=[] if start>=10.-1e-6 else [witness])
    def refine_pair(report):
        before=len(j.rows);apply_witnesses(j,report['witnesses']);return len(j.rows)-before
    pair_result=solve_lexicographic(j.m,[('start',j.t[1],0.)],lambda _:None,accept_pair,refine=refine_pair)
    check('production pair witness reoptimizes event time',pair_result['complete'] and abs(pair_result['priorities'][0]['objective_value']-10.)<1e-6,receipt=pair_result)
    j.m.freeTransform();before=len(j.rows);again=apply_witnesses(j,[witness])
    check('duplicate witness adds no rows',not again and len(j.rows)==before)
    sources=['Agent/Publication/engineering_joint_optimality.py','Agent/Publication/engineering_joint_safety.py','Agent/Publication/engineering_solve_trace.py','Agent/Publication/engineering_unified_inspection.py','scripts/run_unified_inspection.py','scripts/verify_unified_inspection.py','scripts/verify_joint_optical_trajectory.py','scripts/check_unified_refinement.py']
    save(destination,dict(observed_at=stamp(),attempt=attempt,all_pass=all(r['passed'] for r in rows),checks=rows,
        scope='known-case validation of production feedback path; full inspection model remains incomplete',
        inspection_optimization_runs=0,code_sha256={f:sha(ROOT/f) for f in sources},trace_directory=str(trace_dir.relative_to(ROOT))))
    if not all(r['passed'] for r in rows):raise SystemExit(1)


if __name__=='__main__':main()
