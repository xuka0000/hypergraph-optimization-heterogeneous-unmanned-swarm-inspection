"""Reproducible component evidence, including resource decisions, not a joint mission."""
from pathlib import Path
import sys, json, hashlib, datetime, time, itertools
from dataclasses import asdict, replace
import numpy as np
from scipy.integrate import quad
from scipy.optimize import brentq

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from Agent.Publication.engineering_battery_electrothermal import (
    BatteryParameters, initial_state, simulate_segment, simulate_charge,
    current_for_power, charging_current, terminal_voltage,
    energy_balance_residual_Wh, electrical_parameters)
from pyscipopt import Model

OUT=ROOT/'总结/03_实验数据/工程化联合求解/module_validation/battery'
SOURCE=ROOT/'Agent/Publication/engineering_battery_electrothermal.py'
PROFILE=ROOT/'Agent/Publication/inspection_reference_battery_profile.json'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def source_profile():
    p=BatteryParameters()
    official={'capacity_Ah','terminal_voltage_max_V','charge_current_max_A',
              'charge_temperature_min_C','charge_temperature_max_C',
              'flight_temperature_min_C','flight_temperature_max_C'}
    return dict(profile_id='inspection_reference_matched_pair_ecm',schema_version=1,
        platform_role='declared generic reference battery pair at TB60 published scale; not calibrated hardware',
        topology='two identical packs represented by an ideal parallel equivalent; research assumption, not manufacturer topology claim',
        parameters=asdict(p),nominal_nameplate_pair_energy_Wh=548,
        energy_interpretation='nameplate is metadata; SOC follows Ah, chemical energy integrates declared OCV; never use 548*SOC as stored energy',
        sources=[dict(id='DJI_M300_manual',path='总结/07_证据与复现索引/综合报告检索/场景设置文献/dji_m300_manual.pdf',
                      sha256=sha(ROOT/'总结/07_证据与复现索引/综合报告检索/场景设置文献/dji_m300_manual.pdf'),
                      url='https://dl.djicdn.com/downloads/matrice-300/20230526UM/M300_RTK_User_Manual_EN_v3.4.pdf',
                      pages=[30,32,35,97],role='per-pack Ah, voltage ceiling, two charging outputs and temperature boundaries; no RC/thermal fit'),
                 dict(id='Huang2021',url='https://doi.org/10.3390/electronics10222753',
                      role='power-input RC and electrothermal model structure; source uses two RC/two thermal states; no numerical parameter transfer'),
                 dict(id='PyBaMM_Thevenin',url='https://docs.pybamm.org/en/latest/source/api/models/equivalent_circuit/thevenin.html',
                      role='ECM/CCCV model family; this implementation is independently written and not PyBaMM performance')],
        provenance={k:('manufacturer_boundary_or_derived_pair_scale' if k in official else 'declared_reference_assumption') for k in asdict(p)},
        assumptions=['linear temperature-independent OCV','zero reversible entropic heat',
                     'unity Coulombic efficiency','lumped average temperature, not spatially resolved core temperature',
                     'R0/R1 temperature dependence declared, no aging','continuous controllable charger interface; not verified BS60 API',
                     'thermal-ready hysteresis 39/40 C; passive cooling only'],
        sensitivity=dict(R0_multiplier=[0.5,1.,2.],thermal_conductance_multiplier=[0.5,1.,2.],ambient_C=[5.,25.,35.]),
        measured_M300_calibration=False,public_M100_transfer=False)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    write(PROFILE,source_profile())
    protocol=dict(experiment_id='inspection_battery_electrothermal_component',
        purpose='validate signed electrical/thermal states and test their effect on battery availability and charger order',
        scientific_comparison=False,complete_joint_mission=False,
        termination='finite physical trajectories and SCIP optimality; no ten-second optimizer cutoff',
        independent_oracles=['constant-current closed form','quadrature thermal response','energy conservation','enumerated two-job scheduling'],
        tolerances=dict(state=2e-7,energy_balance_Wh=2e-6,scheduling_s=1e-5),
        source_sha256=sha(SOURCE),validator_sha256=sha(Path(__file__)),profile_sha256=sha(PROFILE))
    write(OUT/'protocol.json',protocol)
    old=OUT/'results.json'
    if old.exists():
        prior=json.loads(old.read_text(encoding='utf-8'))
        if prior.get('protocol')==protocol and prior.get('all_checks_passed'):
            print('Reusing compatible battery evidence');return
    attempt=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S')
    checks=[];start=time.perf_counter();p=BatteryParameters();p0=replace(p,resistance_temperature_coefficient_per_K=0.)
    def check(name,ok,**e):checks.append(dict(name=name,passed=bool(ok),**e))
    y=initial_state(.8,25,.1);current=10.;dt=300.
    sol=simulate_segment(y,dt,dict(kind='current',value=current),25.,p0)
    tau=p0.R1_ohm*p0.C1_F
    vp=lambda t:p0.R1_ohm*current+(y[1]-p0.R1_ohm*current)*np.exp(-t/tau)
    exact_soc=y[0]-current*dt/(3600*p0.capacity_Ah)
    heat=lambda t:p0.R0_ohm*current**2+vp(t)**2/p0.R1_ohm
    exact_T=25.+quad(lambda t:np.exp(-(dt-t)*p0.thermal_conductance_W_per_K/p0.thermal_capacity_J_per_K)*heat(t)/p0.thermal_capacity_J_per_K,0,dt,epsabs=1e-11)[0]
    err=max(abs(sol['y'][0,-1]-exact_soc),abs(sol['y'][1,-1]-vp(dt)),abs(sol['y'][2,-1]-exact_T))
    check('constant_current_closed_form',sol['completed'] and err<2e-7,max_state_error=err,
          final_soc=float(sol['y'][0,-1]),final_voltage_V=terminal_voltage(sol['y'][:,-1],current,p0),final_temperature_C=float(sol['y'][2,-1]))
    relax=simulate_segment(sol['y'][:,-1],200,dict(kind='rest'),25,p0)
    check('polarization_relaxation_and_persistent_soc',abs(relax['y'][1,-1]-vp(dt)*np.exp(-2))<2e-7 and abs(relax['y'][0,-1]-exact_soc)<2e-7)
    power_errors=[]
    for s,T,P in itertools.product([.2,.6,.95],[5.,25.,45.],[-700.,0.,300.,1200.]):
        z=initial_state(s,T,.15);i=current_for_power(z,P,p)
        power_errors.append(abs(terminal_voltage(z,i,p)*i-P))
    check('power_port_high_voltage_branch',max(power_errors)<1e-8,max_power_residual_W=max(power_errors))
    try: current_for_power(y,1e6,p);rejected=False
    except ValueError:rejected=True
    check('excessive_power_rejected',rejected)
    rest=simulate_segment(initial_state(.5,48),500,dict(kind='rest'),25,p0)
    cooling=brentq(lambda t:25+23*np.exp(-t*6/2700)-39,0,500)
    charged=simulate_charge(initial_state(.65,48),.98,25,p0)
    actual_cooling=charged['phases'][0]['end_s']
    check('hot_pack_cools_before_charging',charged['completed'] and abs(actual_cooling-cooling)<1e-5,
          cooling_duration_s=actual_cooling,analytic_s=cooling)
    charge_modes=set();maxV=0.;maxInput=0.;max_chargeT=-1e9
    for t,z in zip(charged['t'],charged['y'].T):
        if t>=actual_cooling-1e-7:
            i,mode=charging_current(z,p0);v=terminal_voltage(z,i,p0)
            charge_modes.add(mode);maxV=max(maxV,v);maxInput=max(maxInput,-v*i/p0.charge_efficiency);max_chargeT=max(max_chargeT,z[2])
    check('cc_cv_supply_and_temperature_limits',{'CC','CV'}.issubset(charge_modes) and maxV<=52.8+1e-7 and maxInput<=1000+1e-7 and max_chargeT<=40+1e-7,
          modes=sorted(charge_modes),max_voltage_V=maxV,max_grid_W=maxInput,max_temperature_C=max_chargeT,
          charge_end_s=float(charged['t'][-1]))
    residues=[abs(energy_balance_residual_Wh(y,sol['y'][:,-1],p0)),
              abs(energy_balance_residual_Wh(initial_state(.65,48),charged['y'][:,-1],p0))]
    check('chemical_rc_loss_grid_energy_balance',max(residues)<2e-6,max_residual_Wh=max(residues))
    small=simulate_segment(initial_state(.205,25),100,dict(kind='power',value=1200),25,p)
    check('soc_threshold_stops_without_clipping',not small['completed'] and small['stop_reason']=='SOC_min' and small['t'][-1]<100,
          stop_s=float(small['t'][-1]),final_soc=float(small['y'][0,-1]))
    unchargeable=simulate_charge(initial_state(.5,48),.9,45,p)
    check('unreachable_temperature_is_not_fake_ready',not unchargeable['completed'] and unchargeable['stop_reason']=='ambient_cannot_reach_charge_window')
    whole=simulate_segment(initial_state(.9,25),400,dict(kind='power',value=900),25,p)
    prefix=simulate_segment(initial_state(.9,25),137,dict(kind='power',value=900),25,p)
    suffix=simulate_segment(prefix['y'][:,-1],263,dict(kind='power',value=900),25,p)
    stateerr=float(np.max(np.abs(whole['y'][:,-1]-suffix['y'][:,-1])))
    check('replanning_carries_all_six_states',stateerr<2e-7,max_state_error=stateerr)
    partial=simulate_charge(initial_state(.65,25),.9,25,p,fraction=.5)
    full=simulate_charge(initial_state(.65,25),.9,25,p)
    check('partial_charge_is_actual_control',full['completed'] and partial['completed'] and partial['t'][-1]>full['t'][-1],full_s=float(full['t'][-1]),half_s=float(partial['t'][-1]))
    bad=0
    for pp in [replace(p,capacity_Ah=0),replace(p,charge_efficiency=1.1),replace(p,R1_ohm=-1),replace(p,charge_restart_temperature_C=40)]:
        try:pp.validate()
        except ValueError:bad+=1
    check('invalid_physical_parameters_rejected',bad==4,rejected=bad)
    # OFAT mechanism sensitivity, frozen before observing outputs above.
    sensitivity=[]
    cases=[('R0_multiplier',v,replace(p,R0_ohm=p.R0_ohm*v),25.) for v in [.5,1.,2.]]
    cases += [('thermal_conductance_multiplier',v,replace(p,thermal_conductance_W_per_K=p.thermal_conductance_W_per_K*v),25.) for v in [.5,1.,2.]]
    cases += [('ambient_C',v,p,v) for v in [5.,25.,35.]]
    for axis,value,pp,Ta in cases:
        z=initial_state(.8,Ta)
        x=simulate_segment(z,600,dict(kind='power',value=1200),Ta,pp)
        sensitivity.append(dict(axis=axis,value=value,completed=x['completed'],stop_reason=x['stop_reason'],
             end_s=float(x['t'][-1]),final_SOC=float(x['y'][0,-1]),peak_temperature_C=float(x['y'][2].max()),
             load_Wh=float(x['y'][4,-1]),resistive_heat_Wh=float(x['y'][5,-1]),
             energy_residual_Wh=float(energy_balance_residual_Wh(z,x['y'][:,-1],pp))))
    check('sensitivity_energy_balance',max(abs(x['energy_residual_Wh']) for x in sensitivity)<2e-6)
    # Genuine component optimization: two pack recharge jobs, one charger.
    # Here alpha_R=0 so electrical charge duration is independent of queue
    # temperature; actual thermal trajectory is replayed for both orders.
    jobs=[dict(id='A',soc=.65,T=48.,weight=5.),dict(id='B',soc=.8,T=25.,weight=1.)]
    for j in jobs:
        isolated=simulate_charge(initial_state(j['soc'],j['T']),.9,25,p0)
        j['release_s']=sum(q['end_s']-q['start_s'] for q in isolated['phases'] if q['mode']=='cooling')
        j['duration_s']=float(isolated['t'][-1])-j['release_s']
    schedules=[]
    for thermal in [False,True]:
        m=Model('reference_pair_charge_order');m.hideOutput()
        starts=[m.addVar('start_'+j['id'],lb=j['release_s'] if thermal else 0.,ub=9000.) for j in jobs]
        order=m.addVar('A_before_B',vtype='B');M=10000.
        m.addCons(starts[1]>=starts[0]+jobs[0]['duration_s']-M*(1-order))
        m.addCons(starts[0]>=starts[1]+jobs[1]['duration_s']-M*order)
        m.setObjective(sum(j['weight']*(starts[i]+j['duration_s']) for i,j in enumerate(jobs)),'minimize')
        begin=time.perf_counter();m.optimize();wall=time.perf_counter()-begin
        candidates=[]
        for perm in [(0,1),(1,0)]:
            s=[0.,0.];cursor=0.
            for i in perm:
                s[i]=max(cursor,jobs[i]['release_s'] if thermal else 0.)
                cursor=s[i]+jobs[i]['duration_s']
            obj=sum(j['weight']*(s[i]+j['duration_s']) for i,j in enumerate(jobs))
            candidates.append(dict(order=[jobs[i]['id'] for i in perm],start_s=s,objective_weighted_s=obj))
        oracle=min(candidates,key=lambda x:x['objective_weighted_s'])
        chosen=[float(m.getVal(v)) for v in starts]
        actual=[];feasible=True
        for i,j in enumerate(jobs):
            wait=simulate_segment(initial_state(j['soc'],j['T']),chosen[i],dict(kind='rest'),25,p0)
            z=wait['y'][:,-1];ready=z[2]<=40+1e-7
            replay=simulate_charge(z,.9,25,p0)
            actual.append(dict(pack=j['id'],scheduled_start_s=chosen[i],temperature_at_start_C=float(z[2]),
                               thermal_ready=bool(ready),actual_charge_end_s=chosen[i]+float(replay['t'][-1])))
            feasible=feasible and ready and abs(float(replay['t'][-1])-j['duration_s'])<1e-5
        result=dict(thermal_constraints=thermal,status=str(m.getStatus()),objective_weighted_s=m.getObjVal(),
            wall_s=wall,solver_s=m.getSolvingTime(),nodes=m.getNNodes(),
            order=[jobs[i]['id'] for i in sorted(range(2),key=lambda i:chosen[i])],
            starts_s=chosen,independent_enumeration=oracle,replay=actual,physically_feasible=bool(feasible))
        schedules.append(result)
        check('scheduler_matches_enumeration_'+str(thermal),str(m.getStatus())=='optimal' and abs(m.getObjVal()-oracle['objective_weighted_s'])<1e-5)
    check('thermal_constraint_changes_decision_and_rejects_old_plan',schedules[0]['order']!=schedules[1]['order'] and not schedules[0]['physically_feasible'] and schedules[1]['physically_feasible'])
    hot_initial=initial_state(.65,48.)
    prefix=simulate_charge(hot_initial,.9,25,p0,horizon_s=200.)
    suffix=simulate_charge(prefix['y'][:,-1],.9,25,p0,horizon_s=300.,thermal_memory=prefix['thermal_memory'])
    whole=simulate_charge(hot_initial,.9,25,p0,horizon_s=500.)
    check('hot_hysteresis_memory_survives_planning_boundary',prefix['thermal_memory']['hot_blocked'] and
      39<prefix['y'][2,-1]<40 and suffix['phases'][0]['mode']=='cooling')
    check('split_charge_same_physical_state_as_continuous',max(abs(whole['y'][:,-1]-suffix['y'][:,-1]))<1e-7)
    result=dict(protocol=protocol,attempt_id=attempt,observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                checks=checks,all_checks_passed=all(x['passed'] for x in checks),sensitivity=sensitivity,
                charger_jobs=jobs,schedules=schedules,wall_s=time.perf_counter()-start,
                full_joint_model_verified=False,parameter_identification_from_TB60_measurements=False)
    (OUT/'attempts').mkdir(exist_ok=True)
    write(OUT/'attempts'/f'{attempt}.json',result);write(OUT/'results.json',result)
    print(json.dumps(dict(passed=sum(x['passed'] for x in checks),total=len(checks),failed=[x for x in checks if not x['passed']],
                         schedules=schedules,wall_s=result['wall_s']),ensure_ascii=False,indent=2))
    if not result['all_checks_passed']:raise SystemExit(1)

if __name__=='__main__':main()
