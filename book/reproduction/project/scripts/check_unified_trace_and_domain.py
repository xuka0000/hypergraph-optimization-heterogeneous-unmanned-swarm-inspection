"""Known-case tests of the actual trace callback and full source DEM adapter."""
from run_engineering_full_joint import *
from Agent.Publication.engineering_unified_inspection import JointModel,basis
from Agent.Publication.engineering_joint_optimality import solve_lexicographic
from Agent.Publication.engineering_solve_trace import SolveTrace,unrestricted_stopping
from pyscipopt import Model


def main():
    rows=[]
    def check(name,passed,**data):
        rows.append(dict(name=name,passed=bool(passed),**data));print(name,bool(passed),flush=True)
    receipt=ROOT/'research_quality/inspection_unified_trace_domain_regression.json'
    attempts=ROOT/'research_quality/inspection_unified_trace_domain_attempts.json'
    history=read(attempts) if attempts.exists() else []
    if receipt.exists():history.append(read(receipt));save(attempts,history)
    previous=ROOT/'research_quality/unified_trace_regression'
    attempt=1+max([int(p.name[7:]) for p in previous.glob('attempt*') if p.name[7:].isdigit()]+[0])
    m=Model('trace_known_optimum');m.hideOutput()
    x=m.addVar('x','B');y=m.addVar('y',lb=0,ub=5);z=m.addVar('z',lb=0,ub=5)
    m.addCons(y>=2*x);m.addCons(z>=1)
    objectives=[('incomplete_components',1-x,0),('fleet_makespan_seconds',y,0),('aircraft_energy_Wh',z,0)]
    # Seed actual early-stop settings; all must be erased by the real entry.
    m.setRealParam('limits/time',.01);m.setLongintParam('limits/nodes',1)
    m.setIntParam('limits/solutions',1);m.setIntParam('limits/bestsol',1)
    m.setRealParam('limits/memory',2800);m.setRealParam('limits/gap',.05)
    directory=ROOT/f'research_quality/unified_trace_regression/attempt{attempt}'
    trace=SolveTrace(directory,{'role':'known-case regression, not inspection experiment','attempt':attempt},objectives,{'x':x,'y':y,'z':z})
    m.includeEventhdlr(trace,'regression_trace','exercise production event lifecycle')
    result=solve_lexicographic(m,objectives,lambda x:None,lambda s:True,trace)
    trace.close('known_case_complete')
    data=[json.loads(line) for line in (directory/'progress.jsonl').read_text(encoding='utf-8').splitlines()]
    phase_ends=[r for r in data if r['event']=='phase_end']
    check('lex known optimum and callback phases',result['complete'] and [r['objective_value'] for r in result['priorities']]==[0,2,1] and len(phase_ends)==3,result=result)
    check('every phase carries its own units and bounds',all(r['priority']==i+1 and r['objective_name']==objectives[i][0] and r['acceptance']=='accepted' for i,r in enumerate(phase_ends)),phase_ends=phase_ends)
    check('monotone elapsed clocks',all(a['elapsed_wall_s']<=b['elapsed_wall_s'] for a,b in zip(data,data[1:])))
    incumbents=[r for r in data if r['event']=='incumbent']
    snapshots=[]
    for row in data:
        if row.get('values_sha256'):
            import gzip,hashlib
            payload=gzip.decompress((directory/row['solution_id']).read_bytes())
            values=json.loads(payload)
            snapshots.append(hashlib.sha256(payload).hexdigest()==row['values_sha256'] and set(values)=={'x','y','z'})
    check('actual incumbent events and recoverable variable snapshots',bool(incumbents) and bool(snapshots) and all(snapshots),events=len(incumbents),snapshots=len(snapshots))
    limits=unrestricted_stopping(m)
    check('time nodes memory gap and solution caps reset',limits['limits/time']>=m.infinity() and all(limits[k]==-1 for k in ['limits/nodes','limits/solutions','limits/bestsol','limits/stallnodes']) and limits['limits/gap']==0 and limits['limits/memory']>1e12,effective_limits=limits)
    import csv
    csvrows=list(csv.DictReader((directory/'progress.csv').open(encoding='utf-8')))
    check('CSV and JSONL evidence aligned',len(csvrows)==len(data) and [int(r['sequence']) for r in csvrows]==list(range(len(data))))
    scene,_,terrain,weather=context();initial=[read(OUT/f'joint_optical_bound_SOC1_sortie{k}.json') for k in range(2)]
    j=JointModel(scene,initial[0]['targets'],initial,terrain,weather,[])
    j.prepare_terrain()
    check('every source DEM triangle retained',len(j.triangles)==len(terrain.tri.simplices)==3886,triangles=len(j.triangles),state_lower=j.state_lower.tolist(),state_upper=j.state_upper.tolist())
    cp=np.array([4*k*(7-k)/(7*6) for k in range(8)])
    values=np.array([basis(7,s)@cp for s in np.linspace(0,1,1001)])
    normalized=(j.position_control_upper[:,0]-j.position_control_lower[:,0])/(j.state_upper[0]-j.state_lower[0])
    check('valid curve with controls outside state range retained',cp.max()>1 and values.min()>=0 and values.max()<=1+1e-12 and normalized.min()>=1,example_control_max=float(cp.max()),example_curve_max=float(values.max()),inverse_row_norm=j.control_inverse_norm.tolist())
    # A source point >2500m from the old origin, excluded by the old box.
    point=np.r_[terrain.tri.points[1000],float(terrain.values[1000,0])+40]
    # Avoid ambiguous slopes at a vertex by taking the centroid of a far cell.
    cell=max(range(len(terrain.tri.simplices)),key=lambda k:np.linalg.norm(terrain.tri.points[terrain.tri.simplices[k]].mean(0)-j.origin[:2]))
    xy=terrain.tri.points[terrain.tri.simplices[cell]].mean(0);point=np.r_[xy,float(terrain(*xy))+40]
    p=[j.var('far_'+str(q),point[q],point[q],point[q]) for q in range(3)]
    g,_,_=j.ground(p,'far_source_point')
    feasible=j.m.checkSol(j.sol,printreason=True,completely=True)
    check('far source point exact DEM warm parity',feasible and abs(j.val(g)-float(terrain(*xy)))<1e-9 and np.max(abs(xy-j.origin[:2]))>2500,point=point.tolist(),ground_m=j.val(g),source_ground_m=float(terrain(*xy)),variables=len(j.vars))
    save(receipt,dict(observed_at=stamp(),attempt=attempt,all_pass=all(r['passed'] for r in rows),checks=rows,inspection_optimization_runs=0,trace_directory=str(directory.relative_to(ROOT)),code_sha256={p:sha(ROOT/p) for p in ['Agent/Publication/engineering_solve_trace.py','Agent/Publication/engineering_joint_optimality.py','Agent/Publication/engineering_unified_inspection.py','scripts/check_unified_trace_and_domain.py']}))
    if not all(r['passed'] for r in rows):raise SystemExit(1)


if __name__=='__main__':main()
