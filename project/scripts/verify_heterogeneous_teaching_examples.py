"""Recompute illustrative examples; this is not a public-method comparison."""
from pathlib import Path
import itertools,json,hashlib,datetime
ROOT=Path(__file__).resolve().parents[1]/'总结/超图优化与无人集群巡检建模专著'
E=[[1,0,0],[1,1,0],[0,0,1]]
valid=[p for p in itertools.permutations(range(3)) if all(E[p[j]][j] for j in range(3))]
assert valid==[(0,1,2)]
plans=[]
# Two aircraft, one launch pad; only U2 can perform thermal job J2.
# Flights end at task completion (not return); all times are minutes.
duration={(0,0):10,(1,0):12,(1,1):8}
for assignment in itertools.product(range(2),repeat=2):
 if any((assignment[j],j) not in duration for j in range(2)):continue
 for order in itertools.permutations(range(2)):
  pad=0;avail=[0,0];out=[]
  for j in order:
   u=assignment[j];start=max(pad,avail[u]);launch_end=start+3
   finish=launch_end+duration[u,j];pad=launch_end;avail[u]=finish
   out.append(dict(job=j+1,robot=u+1,pad_start_min=start,pad_end_min=launch_end,finish_min=finish))
  plans.append(dict(assignment=[x+1 for x in assignment],launch_order=[x+1 for x in order],schedule=out,makespan_min=max(x['finish_min'] for x in out)))
assert sorted(p['makespan_min'] for p in plans)==[14,16,26,26]
scope={'R1':['a','b','s'],'R2':['a','c'],'R3':['b','c'],'R4':['s','t','e','q'],'R5':['s','t','z'],'R6':['e','q','z'],'R7':['t','z','q']}
def count(sc):
 vs=set(itertools.chain.from_iterable(sc.values()))
 return dict(incident_variables=len(vs),constraints=len(sc),incidences=sum(map(len,sc.values())),rank=max(map(len,sc.values())),max_degree=max(sum(v in e for e in sc.values()) for v in vs))
struct=count(scope);conditioned=count({k:scope[k] for k in ['R6','R7']})
assert list(struct.values())==[8,7,20,4,3]
assert list(conditioned.values())==[4,2,6,3,2]
c_observations=[dict(time_min=t,actual=['U2'] if t<4 else [],known=['U2'] if t<6 else []) for t in [0,3.999,4,5.999,6,7]]
assert c_observations[2]['actual']==[] and c_observations[2]['known']==['U2']
assert c_observations[3]['known']==['U2'] and c_observations[4]['known']==[]
records=[dict(success=True,completed=2,total=2,cost_min=14),dict(success=True,completed=2,total=2,cost_min=16),dict(success=False,completed=1,total=2,cost_min=None)]
successes=[x for x in records if x['success']]
out=dict(identity='illustrative_synthetic_teaching_examples',source='author-defined parameters, exhaustive enumeration and arithmetic; no physical measurements, no public algorithm reproduction',observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 A=dict(robots=['U1 visible camera','U2 visible+thermal camera','V1 ground support'],tasks=['visible inspection','thermal inspection','ground support'],eligibility=E,permutations_checked=6,feasible_assignments=[[x+1 for x in p] for p in valid],meaning='one task per robot for this illustrative assignment only; not a model of the full shift'),
 B=dict(unit='min',launch_pad_capacity=1,launch_service=3,flight_duration={'U1_J1':10,'U2_J1':12,'U2_J2':8},release=0,objective='minimum last inspection completion time',assumptions='return, battery and route omitted in this analytic example; same-U2 two jobs start next launch only after previous completion',plans=plans,optimum=14),
 C=dict(failure_physical_time_min=4,failure_received_time_min=6,initial_thermal_capable=['U2'],after_received_thermal_capable=[],unreceived_interval='[4,6)',uncompleted_jobs_after_failure=['J2'],checked_information_boundaries=c_observations,rule='prior decisions cannot condition on later information; after received, U1 cannot replace thermal sensing, so J2 remains unmet unless new certified capability is supplied'),
 D=dict(variables=['a','b','c','s','t','e','q','z'],domains='a,b,c,s binary; t,e,q,z in [0,1], all dimensionless abstract demonstration variables; not geographic engineering units',constraints={'R1':'s=a+b','R2':'a+c<=1','R3':'b+c<=1','R4':'s*(e-q+t)=0','R5':'s*(t-z)<=0','R6':'e+q-z=0','R7':'t+z+q<=1'},scopes=scope,structural=struct,condition='fix s=0 and propagate R1, so a=b=0; R2/R3 reduce to c<=1 already in domain; R4/R5 vanish',remaining_free_variables=['c','t','e','q','z'],isolated_free_variables=['c'],remaining_nontrivial_constraints=['R6','R7'],conditioned=conditioned),
 E=dict(identity='illustrative bookkeeping, not three experimental runs',records=records,success_rate=len(successes)/len(records),task_coverage=sum(x['completed'] for x in records)/sum(x['total'] for x in records),success_only_mean_min=sum(x['cost_min'] for x in successes)/len(successes),total_trials=len(records),successful_trials=len(successes),unreported_failure_cost=None))
(ROOT/'evidence/heterogeneous_teaching_examples.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(A_feasible=len(valid),B_optimum=14,D=struct,D_conditioned=conditioned,E=out['E']),ensure_ascii=False))
