"""Contact constitutive cases and a SCIP wind-loaded static equilibrium."""
from pathlib import Path
import sys,json,datetime,hashlib,time,math
import numpy as np
from scipy.optimize import root
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from Agent.Publication import engineering_platform_contact as contact
from Agent.Publication import engineering_joint_transcription as jt
from Agent.Publication import engineering_reference_flight as flight
from Agent.Publication import engineering_battery_electrothermal as battery
SOURCE_PATHS=[Path(jt.__file__),Path(contact.__file__),Path(flight.__file__),Path(battery.__file__),Path(__file__)]
START_HASHES={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCE_PATHS}
OUT=ROOT/'总结/03_实验数据/工程化联合求解/module_validation/platform_contact'


def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
def maxerr(rows):return max(v for r in rows for k,v in r.items() if k!='foot')


def coupled_ground(start_speed,end_speed):
    """One-second angular-speed step; both states and energy remain coupled."""
    fp=flight.FlightParameters();bp=battery.BatteryParameters();cp=contact.ContactParameters()
    tr=jt.Transcription('ground_drive_contact_ECM')
    a=flight.hover_state((0.,0.,cp.foot_vertical_offset_m));z=a.copy()
    a[13:]=fp.thrust_coefficient*start_speed**2;z[13:]=fp.thrust_coefficient*end_speed**2
    y0=[tr.var('before'+str(i),float(v),float(v)) for i,v in enumerate(a)]
    y1=[tr.var('after'+str(i),float(v),float(v)) for i,v in enumerate(z)]
    alpha=end_speed-start_speed;commands=[tr.var('alpha'+str(i),alpha,alpha) for i in range(4)]
    initial=battery.initial_state(.9,25)
    b0=[tr.var('battery_before'+str(i),float(v),float(v)) for i,v in enumerate(initial)]
    b1=[tr.var('battery_after'+str(i),*bound) for i,bound in enumerate([(.2,1),(-5,5),(-20,50),(0,1),(0,1),(0,1)])]
    active=tr.var('pad_mode',1,1,'B')
    block=jt.add_flight_battery_interval(tr,'step',y0,y1,b0,b1,commands,1.,[0.,0.,0.],1.225,25.,
            contact=dict(active=active,pad_xyz=[0.,0.,0.],parameters=cp),actuator_mode='ground_angular_acceleration')
    midpoint=(a+z)/2;om=(start_speed+end_speed)/2;midpoint[13:]=fp.thrust_coefficient*om**2
    fdot=np.full(4,2*fp.thrust_coefficient*om*alpha)
    power=flight.actuator_power(midpoint,fdot,fp)
    def res(end):
        mid=(initial+end)/2
        return end-initial-battery.derivative(mid,battery.current_for_power(mid,power,bp),25.,bp)
    root_result=root(res,initial,tol=1e-11);end=root_result.x;end[3]=0.
    if np.max(abs(res(end)))>1e-10:raise ArithmeticError('Battery seed failed')
    sol=tr.model.createSol();warm={}
    for vv,xx in [(y0,a),(y1,z),(b0,initial),(b1,end),(commands,[alpha]*4)]:warm.update({v.name:float(x) for v,x in zip(vv,xx)})
    warm[active.name]=1;warm[block['current'].name]=battery.current_for_power((initial+end)/2,power,bp);warm[block['power'].name]=power
    normals=(fp.mass_kg*fp.gravity_mps2-midpoint[13:].sum())/4
    for foot in block['contact']['forces']:
        warm.update({foot[0].name:0.,foot[1].name:0.,foot[2].name:float(normals)})
    for rotor in block['ground_angular_states']:
        warm[rotor['start'].name]=start_speed;warm[rotor['end'].name]=end_speed
    assert set(warm)==set(tr.variables)
    for name,value in warm.items():tr.model.setSolVal(sol,tr.variables[name],value)
    tr.model.addSol(sol);tr.model.setObjective(0.);graph=tr.hypergraph()
    start=time.perf_counter();tr.model.optimize();wall=time.perf_counter()-start
    feasible=tr.model.getNSols()>0
    return dict(status=str(tr.model.getStatus()),wall_s=wall,feasible=feasible,
       start_speed_radps=start_speed,end_speed_radps=end_speed,ground_acceleration_radps2=alpha,
       end_thrust_N=[float(tr.model.getVal(v)) for v in y1[13:]] if feasible else None,
       battery_end=[float(tr.model.getVal(v)) for v in b1] if feasible else None,
       midpoint_power_W=power,normal_per_foot_N=normals,
       midpoint_energy_Wh=power/3600,hypergraph=graph,
       scope='single midpoint step, not a complete launch/landing trajectory')


def main():
    OUT.mkdir(exist_ok=True,parents=True);p=contact.ContactParameters();fp=flight.FlightParameters()
    checks=[];cases=[]
    def check(n,ok,**kw):checks.append(dict(name=n,passed=bool(ok),**kw))
    y=flight.hover_state((0,0,p.foot_vertical_offset_m));y[13:]=0
    forces=np.zeros((4,3));forces[:,2]=fp.mass_kg*fp.gravity_mps2/4
    check('resting_weight_supported',maxerr(contact.residuals(y,forces,p))<1e-10)
    wrench=contact.contact_wrench(y,forces,p,jt.NumericOps)
    dy=flight.plant_derivative(y,np.zeros(4),[0,0,0],1.225,fp,
         force_disturbance=wrench[0],torque_disturbance=wrench[1])
    check('resting_rigid_dynamics_balanced',np.max(abs(dy))<1e-10)
    flying=y.copy();flying[2]=5.;flying[3]=1.
    check('unloaded_foot_can_move',maxerr(contact.residuals(flying,np.zeros((4,3)),p,active=False))<1e-10)
    bad=y.copy();bad[2]-=.01
    check('penetration_rejected',maxerr(contact.residuals(bad,forces,p))>.009)
    check('force_without_contact_rejected',maxerr(contact.residuals(flying,forces,p,active=False))>1)
    lifted=y.copy();lifted[2]+=.01
    check('positive_gap_cannot_carry_normal',maxerr(contact.residuals(lifted,forces,p))>.1)
    impact=y.copy();impact[5]=-1
    check('nonzero_normal_impact_not_silently_accepted',maxerr(contact.residuals(impact,forces,p))>1)
    sliding=y.copy();sliding[3]=1.;friction=forces.copy();friction[:,0]=-p.friction_coefficient*friction[:,2]
    check('sliding_Coulomb_opposes_velocity',maxerr(contact.residuals(sliding,friction,p))<1e-10)
    wrong=friction.copy();wrong[:,0]*=-1
    check('friction_cannot_propulse_sliding_body',maxerr(contact.residuals(sliding,wrong,p))>1)
    missing=forces.copy()
    check('moving_loaded_foot_cannot_select_zero_friction',maxerr(contact.residuals(sliding,missing,p))>1)
    outside=y.copy();outside[0]=3
    check('normal_force_requires_actual_deck_support',maxerr(contact.residuals(outside,forces,p))>1)
    # Static equilibrium with rotor thrust at90% of weight and3m/s crosswind.
    y[13:]=.9*fp.mass_kg*fp.gravity_mps2/4
    wind=np.array([3.,0,0]);rho=1.225
    drag=flight.drag_world(y,wind,rho,fp)
    totalN=fp.mass_kg*fp.gravity_mps2-y[13:].sum()
    fx=-drag[0];a=p.foot_radius_m/math.sqrt(2)
    moment_requirement=-p.foot_vertical_offset_m*fx
    pos_total=(totalN+moment_requirement/a)/2;neg_total=totalN-pos_total
    ref=np.array([[fx/4,0,pos_total/2],[fx/4,0,neg_total/2],
                  [fx/4,0,neg_total/2],[fx/4,0,pos_total/2]])
    check('wind_loaded_reference_contact_valid',maxerr(contact.residuals(y,ref,p))<1e-10)
    F,M=contact.contact_wrench(y,ref,p,jt.NumericOps)
    deriv=flight.plant_derivative(y,y[13:],wind,rho,fp,force_disturbance=F,torque_disturbance=M)
    check('wind_loaded_wrench_balances_all_six_axes',np.max(abs(deriv))<1e-9)
    tr=jt.Transcription('wind_loaded_four_foot_contact')
    yy=[tr.var('state'+str(i),float(v),float(v)) for i,v in enumerate(y)]
    active=tr.var('pad_mode',1,1,'B')
    block=contact.add_contact(tr,'pad',yy,active,[0.,0.,0.],parameters=p)
    d=jt.flight_rhs(yy,list(y[13:]),list(wind),rho,fp,jt.ScipOps)
    inertia=[fp.inertia_x,fp.inertia_y,fp.inertia_z]
    for i in range(3):
        tr.row('force_balance'+str(i),fp.mass_kg*d[3+i]+block['force_world'][i],'contact_rigid_coupling')
        tr.row('moment_balance'+str(i),inertia[i]*d[10+i]+block['moment_body'][i],'contact_rigid_coupling')
    graph=tr.hypergraph();sol=tr.model.createSol()
    for v,x in zip(yy,y):tr.model.setSolVal(sol,v,float(x))
    tr.model.setSolVal(sol,active,1)
    for row,values in zip(block['forces'],ref):
        for v,x in zip(row,values):tr.model.setSolVal(sol,v,float(x))
    tr.model.addSol(sol);tr.model.setObjective(0.);start=time.perf_counter();tr.model.optimize();wall=time.perf_counter()-start
    solved=tr.model.getNSols()>0
    output=np.array([[tr.model.getVal(v) for v in row] for row in block['forces']]) if solved else None
    check('SCIP_contact_equilibrium_verified',solved and str(tr.model.getStatus())=='optimal'
          and maxerr(contact.residuals(y,output,p))<1e-6)
    ground_runs=[coupled_ground(0.,400.),coupled_ground(400.,0.)]
    for r in ground_runs:
        check('coupled_ground_drive_'+str(r['end_speed_radps']),r['feasible'] and r['status']=='optimal'
              and abs(r['battery_end'][4]-r['midpoint_energy_Wh'])<1e-8)
    check('finite_stop_without_state_or_energy_reset',ground_runs[1]['end_thrust_N']==[0.]*4
          and ground_runs[1]['battery_end'][0]<.9 and ground_runs[1]['battery_end'][4]>0)
    check('loaded_sources_unchanged_during_run',START_HASHES=={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCE_PATHS})
    result=dict(observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
       experiment_id='inspection_four_foot_contact_component',parameters=contact.asdict(p),checks=checks,
       all_checks_passed=all(c['passed'] for c in checks),
       loaded_source_sha256=START_HASHES,
       SCIP=dict(status=str(tr.model.getStatus()),wall_s=wall,normal_total_N=float(totalN),
         force_world_N=output.tolist() if solved else None,initialization='analytic equilibrium candidate; force variables remain free'),
       ground_drive_runs=ground_runs,
       source_sha256=hashlib.sha256((ROOT/'Agent/Publication/engineering_platform_contact.py').read_bytes()).hexdigest(),
       kernel_sha256=hashlib.sha256((ROOT/'Agent/Publication/engineering_joint_transcription.py').read_bytes()).hexdigest(),
       validator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hypergraph=graph,
       scope='pointwise smooth contact law and static equilibrium, not an executed takeoff/landing trajectory',
       full_joint_solver_implemented=False,full_takeoff_landing_validation=False)
    (OUT/'attempts').mkdir(exist_ok=True);stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S')
    save(OUT/'attempts'/f'{stamp}.json',result);save(OUT/'results.json',result)
    print(json.dumps(dict(all_checks_passed=result['all_checks_passed'],checks=checks,
        SCIP=result['SCIP'],ground_drive_runs=[{k:v for k,v in r.items() if k!='hypergraph'} for r in ground_runs]),ensure_ascii=False,indent=2))
    if not result['all_checks_passed']:raise SystemExit(1)


if __name__=='__main__':main()
