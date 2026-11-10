"""One calm-air reference launch/return with live rotor and ECM states."""
from pathlib import Path
import sys,json,datetime,hashlib,time,math,shutil
import numpy as np
from scipy.integrate import solve_ivp
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from Agent.Publication import engineering_reference_flight as flight
from Agent.Publication import engineering_battery_electrothermal as battery
from Agent.Publication import engineering_platform_contact as contact
from Agent.Publication import engineering_joint_transcription as jt
OUT=ROOT/'总结/03_实验数据/工程化联合求解/module_validation/takeoff_landing'


def H(s):return 35*s**4-84*s**5+70*s**6-20*s**7
def H1(s):return 140*s**3-420*s**4+420*s**5-140*s**6
def H2(s):return 420*s**2-1680*s**3+2100*s**4-840*s**5
def H3(s):return 840*s-5040*s**2+8400*s**3-4200*s**4


def run(step):
    fp=flight.FlightParameters();bp=battery.BatteryParameters();cp=contact.ContactParameters()
    initial_flight=flight.hover_state((0.,0.,2.4));initial_flight[13:]=0.
    initial_battery=battery.initial_state(.9,25.)
    state=np.r_[initial_flight,initial_battery];clock=0.;samples=[];boundaries=[]
    phases=[('spinup',1.),('ascent',12.),('hover',2.),('descent',12.),('spindown',1.)]
    started=time.perf_counter()
    for name,duration in phases:
        grounded=name in ['spinup','spindown'];local=state.copy()
        if grounded:local[13:17]=np.sqrt(np.maximum(state[13:17],0)/fp.thrust_coefficient)
        before=state.copy()
        def evaluate(t,z):
            s=min(1.,max(0.,t/duration));y=z[:17].copy()
            if grounded:
                omega=z[13:17];alpha=np.full(4,(1 if name=='spinup' else -1)*400*H1(s)/duration)
                y[13:17]=fp.thrust_coefficient*omega**2
                fdot=2*fp.thrust_coefficient*omega*alpha
                forces=np.zeros((4,3));forces[:,2]=max(fp.mass_kg*fp.gravity_mps2-y[13:17].sum(),0)/4
                F,M=contact.contact_wrench(y,forces,cp,jt.NumericOps)
                dy=flight.plant_derivative(y,y[13:17],[0.,0.,0.],1.225,fp,force_disturbance=F,torque_disturbance=M)
                dy[13:17]=alpha
                motor_torque=fp.torque_coefficient*omega**2+fp.rotor_inertia_kg_m2*alpha
                command_peak=float(np.max(abs(alpha)));target_z=2.4
            else:
                sign=1 if name=='ascent' else -1
                if name=='hover':vz=az=jerk=0.;target_z=6.4
                else:
                    vz=sign*4*H1(s)/duration
                    az=sign*4*H2(s)/duration**2;jerk=sign*4*H3(s)/duration**3
                    target_z=(2.4 if name=='ascent' else 6.4)+sign*4*H(s)
                # The prescribed vertical motion must also overcome the same
                # body-axis drag present in the independent execution plant.
                desired_f=(fp.mass_kg*(fp.gravity_mps2+az)+.5*1.225*fp.drag_area_z_m2*abs(vz)*vz)/4
                desired_fdot=(fp.mass_kg*jerk+1.225*fp.drag_area_z_m2*abs(vz)*az)/4
                command=np.full(4,desired_f+fp.thrust_lag_s*desired_fdot)
                dy=flight.plant_derivative(y,command,[0.,0.,0.],1.225,fp)
                fdot=dy[13:17];forces=np.zeros((4,3));motor_torque=np.zeros(4);command_peak=float(np.max(command))
            power=flight.actuator_power(y,fdot,fp)
            current=battery.current_for_power(z[17:],power,bp)
            db=battery.derivative(z[17:],current,25.,bp)
            row=dict(time_s=clock+t,phase=name,state=np.r_[y,z[17:]].tolist(),power_W=power,current_A=current,
                terminal_voltage_V=battery.terminal_voltage(z[17:],current,bp),target_z_m=target_z,
                contact_residual=max(v for r in contact.residuals(y,forces,cp,pad_xyz=[0.,0.,1.8],active=grounded) for k,v in r.items() if k!='foot'),
                maximum_thrust_rate_Nps=float(np.max(abs(fdot))),motor_torque_peak_Nm=float(np.max(abs(motor_torque))),
                ground_alpha_or_flight_command_peak=command_peak,grounded=grounded)
            return np.r_[dy,db],row
        sol=solve_ivp(lambda t,z:evaluate(t,z)[0],(0.,duration),local,
             t_eval=np.linspace(0,duration,int(round(duration/step))+1),max_step=step,rtol=1e-10,atol=1e-12)
        if not sol.success:raise ArithmeticError(sol.message)
        rows=[evaluate(t,z)[1] for t,z in zip(sol.t,sol.y.T)]
        state=np.array(rows[-1]['state'])
        boundaries.append(dict(phase=name,start_s=clock,end_s=clock+duration,
            start_state=before.tolist(),end_state=state.tolist(),coordinate_transform_only=True))
        samples.extend(rows if not samples else rows[1:]);clock+=duration
    return dict(step_s=step,wall_s=time.perf_counter()-started,phases=boundaries,samples=samples,
                initial_state=np.r_[initial_flight,initial_battery].tolist(),end_state=state.tolist())


def main():
    OUT.mkdir(parents=True,exist_ok=True);checks=[]
    def check(n,ok,**kw):checks.append(dict(name=n,passed=bool(ok),**kw))
    results=[run(.02),run(.01)];r=results[-1];arr=np.array([x['state'] for x in r['samples']]);fp=flight.FlightParameters();bp=battery.BatteryParameters()
    check('phase_handoffs_preserve_physical_states',all(np.max(abs(np.array(a['end_state'])-b['start_state']))<1e-12 for a,b in zip(r['phases'],r['phases'][1:])))
    e=max(abs(x['state'][2]-x['target_z_m']) for x in r['samples'])
    check('vertical_reference_execution',e<1e-6,maximum_position_error_m=e)
    check('returns_to_same_pad_with_zero_velocity',np.linalg.norm(arr[-1,:3]-[0,0,2.4])<1e-6 and np.linalg.norm(arr[-1,3:6])<1e-6)
    check('finite_start_and_stop_without_thrust_reset',max(abs(arr[-1,13:17]))<1e-12 and np.max(arr[:,13:17])>17)
    check('quaternion_norm_preserved',max(abs(np.linalg.norm(arr[:,6:10],axis=1)-1))<1e-9)
    check('ground_contact_physical_residual',max(x['contact_residual'] for x in r['samples'])<1e-5,
          maximum_residual=max(x['contact_residual'] for x in r['samples']))
    check('rotor_thrust_and_rate_bounds',np.min(arr[:,13:17])>=-1e-9 and np.max(arr[:,13:17])<fp.thrust_max_per_rotor_N
          and max(x['maximum_thrust_rate_Nps'] for x in r['samples'])<fp.thrust_rate_max_Nps)
    check('ground_motor_torque_and_acceleration',max(x['motor_torque_peak_Nm'] for x in r['samples'])<.5
          and max(x['ground_alpha_or_flight_command_peak'] for x in r['samples'] if x['grounded'])<2000)
    check('flight_command_domain',all(0<=x['ground_alpha_or_flight_command_peak']<=fp.thrust_max_per_rotor_N for x in r['samples'] if not x['grounded']))
    check('battery_domain_all_samples',np.min(arr[:,17])>=.2 and np.max(arr[:,17])<=1
          and all(36<=x['terminal_voltage_V']<=52.8 and 0<=x['current_A']<=50 for x in r['samples']))
    check('no_spurious_grid_energy_during_operation',np.max(abs(arr[:,20]))<1e-10)
    balance=battery.energy_balance_residual_Wh(np.array(r['initial_state'])[17:],arr[-1,17:],bp)
    check('battery_energy_balance',abs(balance)<1e-7,residual_Wh=balance)
    diff=float(np.max(abs(np.array(results[0]['end_state'])-arr[-1])))
    check('integration_refinement',diff<1e-6,maximum_endpoint_difference=diff)
    np.savez_compressed(OUT/'execution_trace.npz',time_s=np.array([x['time_s'] for x in r['samples']]),
        state=arr,power_W=np.array([x['power_W'] for x in r['samples']]),
        current_A=np.array([x['current_A'] for x in r['samples']]))
    summary=dict(observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),experiment_id='inspection_calm_takeoff_landing_reference',
        checks=checks,all_checks_passed=all(x['passed'] for x in checks),phases=r['phases'],
        total_duration_s=28.,energy_Wh=float(arr[-1,21]),end_SOC=float(arr[-1,17]),end_temperature_C=float(arr[-1,19]),
        max_vertical_velocity_mps=float(np.max(abs(arr[:,5]))),max_motor_torque_Nm=max(x['motor_torque_peak_Nm'] for x in r['samples']),
        integration_wall_s=[x['wall_s'] for x in results],
        validator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='28s ideal calm-air reference plant; programmed vertical cycle, not trajectory optimization or M300 flight test',
        full_joint_solver_implemented=False,windy_takeoff_landing_verified=False)
    (OUT/'attempts').mkdir(exist_ok=True);stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S')
    shutil.copyfile(OUT/'execution_trace.npz',OUT/'attempts'/f'{stamp}_trace.npz')
    summary['execution_trace']=f'attempts/{stamp}_trace.npz'
    summary['trace_sha256']=hashlib.sha256((OUT/'execution_trace.npz').read_bytes()).hexdigest()
    for p in [OUT/'attempts'/f'{stamp}.json',OUT/'results.json']:p.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in summary.items() if k!='phases'},ensure_ascii=False,indent=2))
    if not summary['all_checks_passed']:raise SystemExit(1)


if __name__=='__main__':main()
