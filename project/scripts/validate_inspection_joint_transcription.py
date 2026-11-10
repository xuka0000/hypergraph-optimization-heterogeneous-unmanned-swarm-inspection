"""RHS parity and coupled SCIP discretization, not a regional mission solve."""
from pathlib import Path
import sys, json, hashlib, datetime, time, math, os
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import root

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from Agent.Publication import engineering_joint_transcription as jt
from Agent.Publication import engineering_reference_flight as flight
from Agent.Publication import engineering_battery_electrothermal as battery
SOURCE_PATHS=[Path(jt.__file__),Path(flight.__file__),Path(battery.__file__),Path(__file__)]
START_HASHES={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCE_PATHS}

OUT = ROOT/'总结/03_实验数据/工程化联合求解/module_validation/joint_transcription'


def write(p, v):
    p.write_text(json.dumps(v, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf8')


def coupled_hover(dt, horizon=4.):
    print('building coupled interval mesh',dt,flush=True)
    p, bp = flight.FlightParameters(), battery.BatteryParameters()
    tr = jt.Transcription('coupled_rotor_ECM_midpoint')
    hover = flight.hover_state(p=p)
    y0 = [tr.var('flight_start_'+str(i), float(x), float(x)) for i, x in enumerate(hover)]
    commands = [tr.var('command_'+str(i), p.mass_kg*p.gravity_mps2/4, p.mass_kg*p.gravity_mps2/4) for i in range(4)]
    initial = battery.initial_state(.8, 25.)
    b0 = [tr.var('battery_start_'+str(i), float(x), float(x)) for i, x in enumerate(initial)]
    n = int(round(horizon/dt))
    assert abs(n*dt-horizon)<1e-12
    power = flight.actuator_power(hover, np.zeros(4), p)
    oracle = solve_ivp(lambda t,y:battery.derivative(y,battery.current_for_power(y,power,bp),25.,bp),
                       (0,horizon), initial, rtol=1e-12, atol=1e-13, max_step=.02, dense_output=True)
    nodes = []
    intervals = []
    warm={v.name:float(x) for v,x in zip(y0,hover)}
    warm.update({v.name:float(x) for v,x in zip(b0,initial)})
    warm.update({v.name:p.mass_kg*p.gravity_mps2/4 for v in commands})
    previous=initial.copy();initialization_s=0.
    for k in range(n):
        print('adding interval',k,flush=True)
        y1 = [tr.var(f'flight_{k+1}_{i}', float(x), float(x)) for i,x in enumerate(hover)]
        bounds = [(bp.soc_min,bp.soc_max),(-5,5),(bp.flight_temperature_min_C,bp.flight_temperature_max_C),(0,2),(0,2),(0,2)]
        b1 = [tr.var(f'battery_{k+1}_{i}', *bound) for i,bound in enumerate(bounds)]
        intervals.append(jt.add_flight_battery_interval(tr, f'interval{k}', y0,y1,b0,b1,commands,
                                                     dt,[0.,0.,0.],1.225,25.,p,bp))
        t_init=time.perf_counter()
        def residual(next_state):
            mid=(previous+next_state)/2
            return next_state-previous-dt*battery.derivative(mid,battery.current_for_power(mid,power,bp),25.,bp)
        guess=oracle.sol((k+1)*dt)
        candidate=root(residual,guess,tol=1e-11)
        if not candidate.success and np.max(abs(residual(candidate.x)))>1e-10:
            raise ArithmeticError('Midpoint initialization failed')
        end=candidate.x;end[3]=0. # Flight has exactly zero charging input.
        midpoint=(previous+end)/2
        warm.update({v.name:float(x) for v,x in zip(y1,hover)})
        warm.update({v.name:float(x) for v,x in zip(b1,end)})
        warm[intervals[-1]['current'].name]=battery.current_for_power(midpoint,power,bp)
        warm[intervals[-1]['power'].name]=power
        previous=end.copy();initialization_s+=time.perf_counter()-t_init
        nodes.append(b1);y0,b0=y1,b1
    tr.model.setObjective(0.)
    print('extracting incidence',dt,flush=True)
    graph=tr.hypergraph()
    tr.model.setRealParam('numerics/feastol',1e-9)
    # The provider's native logger cannot open a Unicode absolute path.
    before_cwd=Path.cwd();os.chdir(OUT)
    tr.model.setLogfile('solver_'+str(dt)+'.log')
    sol=tr.model.createSol()
    assert set(warm)==set(tr.variables)
    for name,value in warm.items():tr.model.setSolVal(sol,tr.variables[name],value)
    tr.model.addSol(sol)
    print('solving',dt,flush=True)
    started=time.perf_counter();tr.model.optimize();wall=time.perf_counter()-started
    os.chdir(before_cwd)
    if tr.model.getNSols()==0:
        return dict(dt=dt,status=str(tr.model.getStatus()),wall_s=wall,solution=None,hypergraph=graph)
    final=np.array([tr.model.getVal(v) for v in nodes[-1]])
    values=[[tr.model.getVal(v) for v in node] for node in nodes]
    return dict(dt=dt,status=str(tr.model.getStatus()),wall_s=wall,initialization_s=initialization_s,
                initialization='independent nonlinear midpoint root; submitted as candidate, not fixed variables',solution=final.tolist(),
                node_states=values,reference_solution=oracle.y[:,-1].tolist(),
                max_state_error=float(np.max(np.abs(final-oracle.y[:,-1]))),
                temperature_error_C=float(abs(final[2]-oracle.y[2,-1])),
                SOC_error=float(abs(final[0]-oracle.y[0,-1])),
                energy_Wh=float(final[4]),nominal_power_W=power,
                currents_A=[float(tr.model.getVal(i['current'])) for i in intervals],hypergraph=graph)


def main():
    import faulthandler
    faulthandler.dump_traceback_later(30,repeat=True)
    OUT.mkdir(parents=True,exist_ok=True);checks=[]
    def check(name,ok,**kw):checks.append(dict(name=name,passed=bool(ok),**kw))
    p,bp=flight.FlightParameters(),battery.BatteryParameters()
    errors=[]
    # Fixed analytical grid; no outcome-selected random states or trials.
    for k in range(40):
        phase=k/39
        q=np.array([1.,.1*math.sin(k),.07*math.cos(k),.15*math.sin(k/2)])
        q/=np.linalg.norm(q)
        y=np.r_[[1,2,30],[2*math.sin(k),math.cos(k),.5*math.cos(k/3)],q,
                 [.1*math.sin(k),.1*math.cos(k),.1],np.array([16,17,18,19])+phase]
        command=y[13:]+np.array([1,-1,2,-2])*phase
        wind=np.array([2*math.cos(k),math.sin(k),.2]);rho=1.+.3*phase
        expected=flight.plant_derivative(y,command,wind,rho,p)
        actual=np.array(jt.flight_rhs(y,command,wind,rho,p,jt.NumericOps))
        ep=flight.actuator_power(y,expected[13:],p);ap=jt.rotor_power(y,actual[13:],p,jt.NumericOps)
        b=battery.initial_state(.3+.6*phase,5+35*phase,.2*math.sin(k));I=-15+60*phase
        er=battery.derivative(b,I,10+20*phase,bp)
        ar,v,_,_=jt.battery_rhs(b,I,10+20*phase,bp,jt.NumericOps)
        errors.append([np.max(abs(actual-expected)),abs(ep-ap),np.max(abs(np.array(ar)-er)),abs(v-battery.terminal_voltage(b,I,bp))])
    maxima=np.max(errors,axis=0)
    for name,error in zip(['rigid_rotor_RHS_parity','actuator_power_parity','battery_RHS_parity','terminal_voltage_parity'],maxima):
        check(name,error<1e-10,maximum_absolute_error=float(error),states=40)
    print('RHS parity completed',maxima,flush=True)
    # Check gated incidence and exact binary deactivation independently of RHS.
    tr=jt.Transcription('conditional_incidence');x=tr.var('x',0,2);a=tr.var('active',0,1,'B')
    tr.row('conditional_x1',x-1,'test',activation=a);g=tr.hypergraph()
    check('conditional_row_contains_gate_and_scalar',g['edges'][0]['variables']==['active','x'] and g['r']==2 and g['N_I']==2)
    tr.model.setObjective(x);tr.model.optimize()
    check('inactive_constraint_does_not_force_equality',tr.model.getNSols()>0 and abs(tr.model.getVal(x))<1e-8 and abs(tr.model.getVal(a))<1e-8)
    tr=jt.Transcription('active_incidence');x=tr.var('x',0,2);a=tr.var('active',1,1,'B')
    tr.row('conditional_x1',x-1,'test',activation=a);tr.model.setObjective(x);tr.model.optimize()
    check('active_constraint_is_enforced',tr.model.getNSols()>0 and abs(tr.model.getVal(x)-1)<1e-8)
    runs=[]
    for dt in [2.,1.,.5]:
        r=coupled_hover(dt);runs.append(r)
        check('SCIP_coupled_midpoint_'+str(dt),r['solution'] is not None and r['status']=='optimal',status=r['status'])
        write(OUT/('midpoint_'+str(dt)+'.json'),r)
    if all(r['solution'] is not None for r in runs):
        errs=[r['temperature_error_C'] for r in runs]
        check('mesh_refinement_second_order',errs[1]<errs[0]/3.5 and errs[2]<errs[1]/3.5,temperature_errors_C=errs)
        check('coupled_energy_matches_rotor_demand',all(abs(r['energy_Wh']-r['nominal_power_W']*4/3600)<1e-8 for r in runs))
        check('incidence_is_actual_scalar_not_block_count',all(r['hypergraph']['n']>50 and r['hypergraph']['m']>50 and r['hypergraph']['r']>1 for r in runs))
    check('loaded_sources_unchanged_during_run',START_HASHES=={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCE_PATHS})
    result=dict(observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        experiment_id='inspection_joint_physics_transcription_component',checks=checks,
        all_checks_passed=all(r['passed'] for r in checks),
        loaded_source_sha256=START_HASHES,
        source_sha256=hashlib.sha256((ROOT/'Agent/Publication/engineering_joint_transcription.py').read_bytes()).hexdigest(),
        validator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        runs=[{k:v for k,v in r.items() if k not in ['hypergraph','node_states']} for r in runs],
        scope='fixed hover with free battery states; exact RHS parity and implicit-midpoint refinement only',
        full_joint_solver_implemented=False,regional_optimization_runs=0,algorithm_comparison=False)
    (OUT/'attempts').mkdir(exist_ok=True);stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S')
    write(OUT/'attempts'/f'{stamp}.json',result);write(OUT/'results.json',result)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    faulthandler.cancel_dump_traceback_later()
    if not result['all_checks_passed']:raise SystemExit(1)


if __name__=='__main__':main()
