"""Physical-equation checks and coupled reference flight/battery experiments."""
from pathlib import Path
import sys,json,hashlib,datetime,time
from dataclasses import replace,asdict
import numpy as np
from scipy.integrate import solve_ivp,trapezoid
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from Agent.Publication.engineering_reference_flight import *
from Agent.Publication.engineering_battery_electrothermal import BatteryParameters,initial_state,simulate_power_trace,energy_balance_residual_Wh
from Agent.Publication.engineering_platform_contact import ContactParameters
OUT=ROOT/'总结/03_实验数据/工程化联合求解/module_validation/flight'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def main():
    OUT.mkdir(parents=True,exist_ok=True);p=FlightParameters()
    profile=dict(profile_id='inspection_reference_rigid_quadrotor',parameters=asdict(p),
      physical_identity='generic 7kg rigid quadrotor at existing scene scale, not a fitted M300',
      declared_assumptions=list(asdict(p)),
      manufacturer_scale_reference=dict(arm_radius_m='895mm diagonal wheelbase / 2; reference rotor-center geometry',
          source='https://dl.djicdn.com/downloads/matrice-300/20230526UM/M300_RTK_User_Manual_EN_v3.4.pdf',page=95),
      dynamics='17 states: position3,velocity3,quaternion4,bodyrate3,rotor_thrust4',
      contact='four-foot smooth unilateral pad contact; static certificate is one component; no impact approximation',
      contact_parameters=asdict(ContactParameters()),
      contact_parameter_provenance='all declared reference gear, deck and ground-drive assumptions; no M300 identification',
      controller='geometric PD construction with declared saturation and lag; no desired-rate feedforward',
      citation='Lee, Leok, McClamroch, Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3), arXiv1003.2005v4,2011',
      public_algorithm_reproduction=False,source_parameter_transfer=False,
      plant_omissions=['rotor gyroscopic moments','ground effect','aeroelasticity'],
      power='sum(max(kq*Omega^3+Jrot*fdot/(2*kf),0))/eta+Paux',measured_calibration=False)
    pp=ROOT/'Agent/Publication/inspection_reference_flight_profile.json';write(pp,profile)
    protocol=dict(experiment_id='inspection_rigid_flight_component',scientific_comparison=False,
      purpose='verify revision8 rigid-body/thrust/contact equations and reference tracking with battery replay',
      profile_sha256=sha(pp),source_sha256=sha(ROOT/'Agent/Publication/engineering_reference_flight.py'),
      battery_source_sha256=sha(ROOT/'Agent/Publication/engineering_battery_electrothermal.py'),validator_sha256=sha(Path(__file__)),
      time_steps_s=[.04,.02],scenario_set=['calm','crosswind','time_varying_3d_wind'],
      scope='finite reference trajectories; not actual aircraft calibration, robust guarantee or joint optimization')
    write(OUT/'protocol.json',protocol)
    if (OUT/'results.json').exists():
        old=json.loads((OUT/'results.json').read_text(encoding='utf-8'))
        if old.get('protocol')==protocol and old.get('all_checks_passed'):print('Reusing compatible flight evidence');return
    checks=[];begin=time.perf_counter()
    def check(name,ok,**kw):checks.append(dict(name=name,passed=bool(ok),**kw))
    y=hover_state(p=p);cmd=y[13:].copy();A=allocation(p)
    d=plant_derivative(y,cmd,[0,0,0],1.225,p)
    check('force_and_torque_hover_equilibrium',np.max(np.abs(d))<1e-10,max_derivative=float(np.max(np.abs(d))))
    check('rotor_wrench_mapping_invertible',np.linalg.matrix_rank(A)==4 and np.max(np.abs(A@cmd-np.array([p.mass_kg*p.gravity_mps2,0,0,0])))<1e-10)
    yaw=y.copy();yaw[10:13]=[0,0,.1]
    sol=solve_ivp(lambda t,z:plant_derivative(z,cmd,[0,0,0],1.225,p),(0,2),yaw,rtol=1e-11,atol=1e-13)
    qexpected=np.array([np.cos(.1),0,0,np.sin(.1)])
    check('quaternion_yaw_matches_exact_solution',np.linalg.norm(sol.y[6:10,-1]-qexpected)<1e-9,
          quaternion_error=float(np.linalg.norm(sol.y[6:10,-1]-qexpected)))
    plus=cmd+1
    lag=solve_ivp(lambda t,z:plant_derivative(z,plus,[0,0,0],1.225,p),(0,.3),y,rtol=1e-11,atol=1e-13)
    lagerr=np.max(np.abs(lag.y[13:,-1]-(cmd+1-np.exp(-.3/p.thrust_lag_s))))
    check('motor_lag_matches_exact_solution',lagerr<1e-8,max_thrust_error_N=float(lagerr))
    pw=actuator_power(y,np.zeros(4),p);expected=4*p.torque_coefficient*400**3/p.motor_efficiency+p.auxiliary_power_W
    check('hover_rotor_power_identity',abs(pw-expected)<1e-8,hover_power_W=pw)
    gain=actuator_power(y,np.full(4,1.),p)-pw
    check('spinup_power_contains_rotor_inertia',abs(gain-4*p.rotor_inertia_kg_m2/(2*p.thrust_coefficient*p.motor_efficiency))<1e-8,extra_W=gain)
    ground=hover_state([0,0,0],p);ground[13:]=0
    contact=pad_reaction(ground,cmd,[0,0,0],1.225,p)
    check('zero_thrust_ground_support',contact['valid'] and abs(contact['normal_N']-68.67)<1e-9,normal_N=contact['normal_N'])
    ground[3]=.5
    try:pad_reaction(ground,cmd,[0,0,0],1.225,p);reject=False
    except ValueError:reject=True
    check('impact_not_silently_stopped',reject)
    contactwind=pad_reaction(y,cmd,[5,0,0],1.225,p)
    check('frictionless_liftoff_contact_rejected_in_crosswind',not contactwind['valid'],normal_N=contactwind['normal_N'],needed_friction_N=contactwind['friction_N'])
    # Smooth seventh-degree displacement: actual 3D plant, not waypoint jumps.
    def reference(t):
        T=30.;s=min(max(t/T,0.),1.)
        H=35*s**4-84*s**5+70*s**6-20*s**7
        dH=(140*s**3-420*s**4+420*s**5-140*s**6)/T if t<T else 0.
        ddH=(420*s**2-1680*s**3+2100*s**4-840*s**5)/T**2 if t<T else 0.
        delta=np.array([12.,4.,2.])
        return np.array([0.,0.,10.])+delta*H,delta*dH,delta*ddH,0.
    weathers={'calm':lambda t,x:(np.zeros(3),1.225),
              'crosswind':lambda t,x:(np.array([0.,3.,0.]),1.225),
              'time_varying_3d_wind':lambda t,x:(np.array([2*np.sin(t/8),3*np.sin(t/11),.4*np.sin(t/7)]),1.20)}
    experiments=[];traces={}
    for name,weather in weathers.items():
        x=simulate_tracking(y,40,reference,weather,p,max_step_s=.02)
        z0=initial_state(.9,25);b=simulate_power_trace(z0,x['t'],x['power_W'],25,BatteryParameters())
        row=dict(scenario=name,peak_position_error_m=float(x['position_error_m'].max()),
          rms_position_error_m=float(np.sqrt(np.mean(x['position_error_m']**2))),
          final_position_error_m=float(x['position_error_m'][-1]),quaternion_norm_error=float(np.max(np.abs(np.linalg.norm(x['y'][6:10],axis=0)-1))),
          peak_bodyrate_radps=float(np.max(np.abs(x['y'][10:13]))),
          peak_thrust_N=float(x['y'][13:].max()),saturation_samples=x['saturation_samples'],
          flight_energy_Wh=float(trapezoid(x['power_W'],x['t'])/3600),
          battery_load_Wh=float(b['y'][4,-1]),ending_SOC=float(b['y'][0,-1]),
          battery_peak_temperature_C=float(b['y'][2].max()),battery_domain_pass=b['completed'],
          energy_residual_Wh=float(energy_balance_residual_Wh(z0,b['y'][:,-1],BatteryParameters())))
        experiments.append(row);traces[name]=x
        check('tracking_and_power_battery_'+name,row['peak_position_error_m']<p.allowed_position_error_m and row['quaternion_norm_error']<1e-7
              and b['completed'] and abs(row['energy_residual_Wh'])<2e-6 and abs(row['flight_energy_Wh']-row['battery_load_Wh'])<2e-6,
              max_error_m=row['peak_position_error_m'])
    coarse=simulate_tracking(y,40,reference,weathers['time_varying_3d_wind'],p,max_step_s=.04)
    fine=traces['time_varying_3d_wind']
    difference=float(np.max(np.abs(fine['y'][:,::2]-coarse['y'])))
    check('step_refinement',difference<2e-5,max_state_difference=difference)
    # Store sampled evidence, not only aggregate checks.
    np.savez_compressed(OUT/'tracking_traces.npz',**{name+'_'+key:x[key] for name,x in traces.items() for key in ['t','y','power_W','position_error_m']})
    result=dict(protocol=protocol,observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                checks=checks,all_checks_passed=all(c['passed'] for c in checks),experiments=experiments,
                wall_s=time.perf_counter()-begin,full_joint_verified=False,field_calibration=False)
    (OUT/'attempts').mkdir(exist_ok=True);attempt=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S')
    write(OUT/'attempts'/f'{attempt}.json',result);write(OUT/'results.json',result)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if not result['all_checks_passed']:raise SystemExit(1)

if __name__=='__main__':main()
