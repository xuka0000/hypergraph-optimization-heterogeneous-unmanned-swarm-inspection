"""Focused independent checks of changed model semantics; no inspection solve."""
from run_engineering_full_joint import *
from Agent.Publication.engineering_unified_inspection import JointModel
from Agent.Publication.engineering_joint_optimality import solve_lexicographic,require_complete_model
from Agent.Publication.engineering_joint_weather import hourly_table
from Agent.Publication.engineering_joint_safety import bezier,cylinder_signed_distance,wire_point,pair_witness,point_witness
from Agent.Publication.engineering_event_charging import add_event_charging
from pyscipopt import Model


def bare():
    j=JointModel.__new__(JointModel);j.m=Model();j.m.hideOutput();j.sol=j.m.createSol()
    j.vars={};j.groups={};j.rows=[];j.residuals=[];return j


def main():
    scene,_,terrain,weather=context();rows=[]
    def check(name,passed,**detail):
        rows.append(dict(name=name,passed=bool(passed),**detail));print(name,bool(passed),flush=True)
    initial=[read(OUT/f'joint_optical_bound_SOC1_sortie{k}.json') for k in range(2)]
    # Exercise the actual schedule implementation. Analytical expected time
    # integrates each source charge-rate interval, independently of SCIP.
    for a,b in [(.2,.8),(.89,.95),(.95,1.),(1.,1.)]:
        j=JointModel(scene,initial[0]['targets'],initial,terrain,weather,[],1.,charge_mode='two_phase')
        j.schedule();i=0
        j.m.addCons(j.eb[i]==a*548);j.m.addCons(j.ea[i]==b*548)
        j.m.setObjective(j.ce[i]-j.cs[i]);j.m.optimize()
        observed=float(j.m.getObjVal())
        expected=max(0,min(b,.9)-a)/.7*1800+max(0,b-max(a,.9))/.1*1200
        tolerance=scene['joint_reference_profile']['numerical']['time_tolerance_s']
        check(f'charge {a}->{b}',str(j.m.getStatus())=='optimal' and abs(observed-expected)<tolerance,expected_seconds=expected,observed_seconds=observed,time_tolerance_seconds=tolerance,status=str(j.m.getStatus()))
        del j
    table=hourly_table(scene,'2025-07-01T08:00:00+00:00',28800)
    differences=[]
    for hour,row in enumerate(table):
        for h,r in zip([30,60,100],row['rows']):
            xy=scene['fleet']['vehicles'][0]['initial_xyz_m'][:2];ground=float(terrain(*xy))
            w,rho=weather(np.r_[xy,ground+h],hour*3600+1,ground)
            differences.append(max(abs(rho-r['air_density_proxy_kg_m3']),abs(w[0]-r['wind_east_ms']),abs(w[1]-r['wind_north_ms'])))
    check('eight source hours and24 layers',len(table)==8 and max(differences)<1e-10,max_error=max(differences))
    # A200-second job around a mandatory [100,200]100-second job requires
    # preemption. The production event model must admit the split witness.
    j=bare();j.n=2;j.H=300.;j.scene=scene;j.cfg=scene['joint_reference_profile'];j.arr={};j.dep={}
    j.eb={};j.ea={};j.cs={};j.ce={};j.s={};j.b={};j.y={(u,l):int(u==0) for u in range(4) for l in range(2)}
    rate=.7*548/1800/.9*3600
    for v in range(2):
        j.arr[v,0]=j.var(f'arr{v}',0,0,0);j.dep[v,0]=j.var(f'dep{v}',300,300,300)
    for l,release,deadline,processing in [(0,0,300,200),(1,100,200,100)]:
        for field,value in [('eb',109.6),('ea',109.6+.9*rate*processing/3600),('cs',release),('ce',deadline),('s',deadline+120),('b',1)]:
            getattr(j,field)[l]=j.var(f'{field}{l}',value,value,value)
    j.initial_charge_intervals=[dict(next_flight=l,carrier=0,start_s=a,end_s=b,input_power_W=rate) for l,a,b in [(0,0,100),(1,100,200),(0,200,300)]]
    add_event_charging(j)
    accepted=j.m.checkSol(j.sol,completely=True,printreason=True)
    check('preemptible charging witness',accepted,intervals=j.initial_charge_intervals,event_bound=j.charge_event_count,uninterrupted_job0_longest_gap_s=100,job0_required_charge_s=200)
    # Same-time separation must survive retiming, not freeze a time from the
    # old incumbent. Two stationary aircraft3m apart collide during overlap.
    for shift,expected in [(0.,False),(20.,True)]:
        j=bare();j.n=2;j.cfg=scene['joint_reference_profile'];j.origin=np.zeros(3)
        j.t={i:j.var(f't{i}',-100,100,[0.,shift][i]) for i in range(2)}
        for i in range(2):j.m.addCons(j.t[i]==[0.,shift][i])
        j.x={(i,l):int(i==l) for i in range(2) for l in range(2)};j.segments={}
        for i,x in enumerate([0.,3.]):
            T=j.var(f'T{i}',1,50,10);j.m.addCons(T==10)
            C=[[j.var(f'p{i}_{k}_{q}',-100,100,[x,0.,20.][q]) for q in range(3)] for k in range(8)]
            for row in C:
                for q,v in enumerate(row):j.m.addCons(v==[x,0.,20.][q])
            j.segments[str(i)]=dict(C=C,T=T,active=1,task=i,segment=0,role='cap')
        pair_witness(j,'0','1',.5);j.m.optimize()
        feasible=str(j.m.getStatus())=='optimal'
        check(f'same-time pair shift{shift}',feasible==expected,status=str(j.m.getStatus()),expected_feasible=expected)
    # Signed cylinder distance independently checked against elementary cases.
    cylinder=dict(center_xy_m=[0,0],radius_m=2,base_elevation_m=0,height_m=10)
    for point,expected in [([0,0,5],-2),([5,0,5],3),([5,0,14],5),([0,0,12],2)]:
        j=bare();p=[j.var(f'p{q}',-20,20,x) for q,x in enumerate(point)]
        observed=j.val(cylinder_signed_distance(p,cylinder))
        check('cylinder '+str(point),abs(observed-expected)<1e-8,expected=expected,observed=observed)
    m=Model();m.hideOutput();u=m.addVar('u',vtype='I',lb=0,ub=2);x=m.addVar('x',lb=0,ub=3);y=m.addVar('y',lb=0,ub=3)
    m.addCons(x>=2-u);m.addCons(x+y>=3)
    receipts=[]
    lex=solve_lexicographic(m,[('u',u,0.),('x',x,0.),('y',y,0.)],receipts.append,
        lambda sol: m.getSolVal(sol,x)>=2-m.getSolVal(sol,u)-1e-7 and m.getSolVal(sol,x)+m.getSolVal(sol,y)>=3-1e-7)
    vector=[row.get('objective_value') for row in lex['priorities']]
    check('three priorities same variable domain',lex['complete'] and np.max(np.abs(np.asarray(vector)-[0,2,1]))<1e-7,vector=vector,receipt=lex)
    try:require_complete_model(ROOT,'research_quality/inspection_unified_completion.json')
    except RuntimeError as exc:check('incomplete full-model optimization prevented',True,reason=str(exc))
    else:check('incomplete full-model optimization prevented',False)
    result=dict(observed_at=stamp(),scope='model-component regression; no complete inspection optimization',checks=rows,all_pass=all(r['passed'] for r in rows),
                code_sha256={p:sha(ROOT/p) for p in ['Agent/Publication/engineering_unified_inspection.py','Agent/Publication/engineering_joint_optimality.py','Agent/Publication/engineering_joint_weather.py','Agent/Publication/engineering_joint_safety.py','Agent/Publication/engineering_event_charging.py','scripts/check_unified_model_completion.py']})
    save(ROOT/'research_quality/inspection_unified_completion_regression.json',result)
    if not result['all_pass']:raise SystemExit(1)


if __name__=='__main__':main()
