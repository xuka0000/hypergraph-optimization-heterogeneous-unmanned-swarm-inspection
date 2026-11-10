"""SCIP/independent-root checks of shared charging and ECM coupling."""
from pathlib import Path
import sys,json,datetime,hashlib,time
import numpy as np
from scipy.optimize import root,brentq
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from Agent.Publication import engineering_joint_transcription as jt
from Agent.Publication import engineering_joint_charging as charging
from Agent.Publication import engineering_battery_electrothermal as battery
OUT=ROOT/'总结/03_实验数据/工程化联合求解/module_validation/joint_charging'
SOURCES=[Path(jt.__file__),Path(charging.__file__),Path(battery.__file__),Path(__file__)]
START_HASHES={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCES}


def save(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')


def endpoint(initial,current,dt=10.):
    p=battery.BatteryParameters()
    def residual(end):return end-initial-dt*battery.derivative((initial+end)/2,-current,25.,p)
    out=root(residual,initial,tol=1e-11)
    if max(abs(residual(out.x)))>1e-9:raise ArithmeticError('Independent midpoint root failed')
    out.x[4]=initial[4]
    mid=(initial+out.x)/2
    voltage=battery.terminal_voltage(mid,-current,p)
    return out.x,voltage,voltage*current/p.charge_efficiency


def admissible_current(initial,input_cap=1000.):
    p=battery.BatteryParameters();upper=p.charge_current_max_A
    for fun in [lambda c:endpoint(initial,c)[1]-p.terminal_voltage_max_V,
                lambda c:endpoint(initial,c)[2]-input_cap,
                lambda c:endpoint(initial,c)[0][0]-p.soc_max]:
        if fun(upper)>0:upper=brentq(fun,0.,upper,xtol=1e-12)
    return upper


def block(tr,name,initial,connected_bounds=(1,1),hot=0):
    b0=[tr.var(name+'_start'+str(j),float(x),float(x)) for j,x in enumerate(initial)]
    b1=[tr.var(name+'_end'+str(j),*v) for j,v in enumerate([(.2,1),(-5,5),(-20,50),(0,10),(0,10),(0,10)])]
    c=tr.var(name+'_connected',*connected_bounds,'B');h=tr.var(name+'_hot',hot,hot,'B')
    value=charging.add_charge_interval(tr,name,b0,b1,10.,25.,c,h)
    return b0,b1,value


def seed(tr,entries):
    values={}
    for b0,b1,b,initial,current,connected,hot in entries:
        end,v,P=endpoint(initial,current)
        values.update({a.name:float(x) for a,x in zip(b0,initial)})
        values.update({a.name:float(x) for a,x in zip(b1,end)})
        values.update({b['charge_current'].name:current,b['grid_power'].name:P,
                       b['connected'].name:connected,b['hot_blocked'].name:hot})
    if set(values)!=set(tr.variables):raise ValueError('Incomplete numerical candidate')
    sol=tr.model.createSol()
    for n,v in values.items():tr.model.setSolVal(sol,tr.variables[n],v)
    tr.model.addSol(sol)


def solve_one(soc,cap=1000.,hot=0):
    tr=jt.Transcription('charge_ECM');initial=battery.initial_state(soc,39.5 if hot else 25.)
    b0,b1,b=block(tr,'battery',initial,hot=hot)
    charging.add_charger_capacity(tr,'vehicle',[b],1,cap)
    expected=admissible_current(initial,cap)
    if not hot:seed(tr,[(b0,b1,b,initial,expected,1,0)])
    tr.model.setObjective(b['charge_current'],'maximize')
    tr.model.setRealParam('numerics/feastol',1e-9)
    graph=tr.hypergraph();start=time.perf_counter();tr.model.optimize();wall=time.perf_counter()-start
    feasible=tr.model.getNSols()>0
    return dict(initial_SOC=soc,input_cap_W=cap,hot_blocked=hot,status=str(tr.model.getStatus()),wall_s=wall,
                current_A=float(tr.model.getVal(b['charge_current'])) if feasible else None,
                oracle_current_A=expected if not hot else None,
                grid_W=float(tr.model.getVal(b['grid_power'])) if feasible else None,
                battery_end=[float(tr.model.getVal(v)) for v in b1] if feasible else None,
                hypergraph=graph)


def shared_capacity():
    tr=jt.Transcription('two_batteries_one_charger');initial=battery.initial_state(.5,25.)
    aa=block(tr,'A',initial,(0,1));bb=block(tr,'B',initial,(0,1))
    charging.add_charger_capacity(tr,'vehicle',[aa[2],bb[2]],1,1000.)
    seed(tr,[(*aa,initial,17.8,1,0),(*bb,initial,0.,0,0)])
    tr.model.setObjective(2*aa[2]['charge_current']+bb[2]['charge_current'],'maximize')
    graph=tr.hypergraph();start=time.perf_counter();tr.model.optimize();wall=time.perf_counter()-start
    ok=tr.model.getNSols()>0
    return dict(status=str(tr.model.getStatus()),wall_s=wall,
                objective=float(tr.model.getObjVal()) if ok else None,
                currents_A=[float(tr.model.getVal(x[2]['charge_current'])) for x in [aa,bb]] if ok else None,
                connected=[int(round(tr.model.getVal(x[2]['connected']))) for x in [aa,bb]] if ok else None,
                end_states=[[float(tr.model.getVal(v)) for v in x[1]] for x in [aa,bb]] if ok else None,
                hypergraph=graph)


def main():
    OUT.mkdir(parents=True,exist_ok=True);checks=[]
    def check(n,ok,**kw):checks.append(dict(name=n,passed=bool(ok),**kw))
    runs=[]
    for soc,cap in [(.5,1000.),(.99,1000.),(.5,250.)]:
        r=solve_one(soc,cap);runs.append(r)
        check(f'current_ceiling_{soc}_{cap}',r['current_A'] is not None and
              abs(r['current_A']-r['oracle_current_A'])<1e-5,status=r['status'])
        if r['battery_end']:
            check(f'energy_account_{soc}_{cap}',abs(battery.energy_balance_residual_Wh(
                battery.initial_state(soc,25.),np.array(r['battery_end']),battery.BatteryParameters()))<1e-8)
    hot=solve_one(.5,hot=1);runs.append(hot)
    check('hot_memory_cannot_be_overridden_by_connection',hot['status']=='infeasible')
    queue=shared_capacity()
    check('one_charger_allocated_to_higher_weight',queue['connected']==[1,0] and abs(queue['objective']-35.6)<1e-6)
    check('unconnected_battery_does_not_receive_energy',queue['end_states'] is not None and abs(queue['end_states'][1][3])<1e-10)
    check('active_battery_keeps_electrothermal_state',queue['end_states'] is not None and queue['end_states'][0][0]>.5
          and queue['end_states'][0][1]<0 and queue['end_states'][0][2]>25.)
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCES}
    check('loaded_sources_unchanged_during_run',hashes==START_HASHES)
    result=dict(observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
      experiment_id='inspection_joint_charge_ECM_component',checks=checks,all_checks_passed=all(c['passed'] for c in checks),
      source_sha256=START_HASHES,runs=runs,shared_capacity=queue,
      scope='10-second event-aligned component optimization; maximum instantaneous current objective',
      full_mission_charging_schedule=False,continuous_thermal_guard_verified=False,regional_optimization_runs=0)
    (OUT/'attempts').mkdir(exist_ok=True);stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S')
    save(OUT/'attempts'/f'{stamp}.json',result);save(OUT/'results.json',result)
    print(json.dumps(dict(checks=checks,runs=[{k:v for k,v in r.items() if k!='hypergraph'} for r in runs],
             shared_capacity={k:v for k,v in queue.items() if k!='hypergraph'}),ensure_ascii=False,indent=2))
    if not result['all_checks_passed']:raise SystemExit(1)


if __name__=='__main__':main()
