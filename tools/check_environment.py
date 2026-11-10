"""Small portable examples, core imports and one finite SCIP decision."""
from pathlib import Path
import importlib, itertools, json, math, sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'project'))
modules=['engineering_battery_electrothermal','engineering_reference_flight',
         'engineering_reference_weather','engineering_image_acceptance','engineering_replanning',
         'engineering_service_boundary','engineering_joint_transcription',
         'engineering_platform_contact','engineering_joint_charging','engineering_road_motion']
for name in modules:importlib.import_module('Agent.Publication.'+name)
from Agent.Publication.engineering_battery_electrothermal import BatteryParameters,initial_state,simulate_segment
from dataclasses import replace
p=replace(BatteryParameters(),resistance_temperature_coefficient_per_K=0.)
state=initial_state(.8,25.,.1)
sol=simulate_segment(state,30.,dict(kind='current',value=10.),25.,p)
exact=.8-10*30/(3600*p.capacity_Ah)
error=abs(float(sol['y'][0,-1])-exact)
assert sol['completed'] and error<1e-9
from pyscipopt import Model,quicksum
m=Model('portable_finite_check');m.hideOutput()
x=[m.addVar(vtype='B',name=f'x{i}') for i in range(3)]
m.addCons(x[0]+x[1]<=1);m.addCons(x[1]+x[2]<=1)
m.setObjective(3*x[0]+4*x[1]+2*x[2],'maximize');m.optimize()
assert str(m.getStatus())=='optimal' and abs(m.getObjVal()-5)<1e-9
E=[[1,0,0],[1,1,0],[0,0,1]]
feasible=[perm for perm in itertools.permutations(range(3)) if all(E[perm[j]][j] for j in range(3))]
assert feasible==[(0,1,2)]
print(json.dumps(dict(core_modules_imported=len(modules),battery_soc_error=error,scip_status=str(m.getStatus()),scip_objective=m.getObjVal(),heterogeneous_assignment_feasible=len(feasible),full_regional_experiment=False),indent=2))
