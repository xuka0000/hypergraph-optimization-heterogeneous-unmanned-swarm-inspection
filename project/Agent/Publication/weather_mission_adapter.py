"""Bounded weather/mission interface around the original CMU dynamics.

Author dynamics and controller retained, except the controller's 0/0 at an exact
hover target is assigned its continuous zero-vector limit. No new controller,
learned weights, terrain CFD, or calibrated battery thermal model is introduced.
"""
import ast, math, pathlib
import numpy as np
import pandas as pd

ROOT=pathlib.Path(__file__).resolve().parents[2]
SOURCE=ROOT/'总结/03_实验数据/电池能耗公开机型验证/sources/cmu2021/QuadSim.py'

def author_classes():
    tree=ast.parse(SOURCE.read_text('utf8'))
    nodes=[n for n in tree.body if isinstance(n,ast.ClassDef)]
    assert {n.name for n in nodes}=={'QuadSim','QuadState'}
    count=[0]
    class HoverLimit(ast.NodeTransformer):
        def visit_Assign(self,n):
            if any(isinstance(t,ast.Name) and t.id=='error_vel_ratio' for t in n.targets):
                count[0]+=1
                n.value=ast.IfExp(test=ast.Compare(left=ast.Name(id='error_vel_mag',ctx=ast.Load()),ops=[ast.Eq()],comparators=[ast.Constant(0)]),body=ast.Constant(0.),orelse=n.value)
            return n
    tree=HoverLimit().visit(ast.Module(body=nodes,type_ignores=[]));ast.fix_missing_locations(tree)
    assert count[0]==2
    ns={'np':np,'pd':pd};exec(compile(tree,str(SOURCE),'exec'),ns)
    return ns['QuadSim'],ns['QuadState']

QuadSim,QuadState=author_classes()

class ZeroDryden:
    def update(self):return np.zeros(3)

class PrescribedWind:
    """Explicit steady horizontal background, NWU coordinates as author expects."""
    def __init__(self,wind_nwu):self.vector=np.asarray(wind_nwu,dtype=float)
    def wind_at_point(self,x,y):return math.degrees(math.atan2(self.vector[1],self.vector[0])),float(np.linalg.norm(self.vector[:2]))

def create_sim(start_nwu,density,wind_nwu,dt=.1):
    sim=QuadSim.__new__(QuadSim)
    sim.sim_dt=dt;sim.traj_res=dt;sim.mass=3.71;sim.payload=0.;sim.max_tilt_angle=35
    sim.command_airspeed=False;sim.ground_speed=5.;sim.air_density=density
    sim.windField=PrescribedWind(wind_nwu);sim.dryden=ZeroDryden();sim.states_hist=[]
    state=QuadState(pos=np.asarray(start_nwu,dtype=float),vel=np.zeros(3),acc=np.zeros(3),rpy=np.zeros(3),airVel=-np.asarray(wind_nwu,dtype=float),thrust=sim.mass*9.81)
    return sim,state

def regression():
    sim,s=create_sim([0,0,60],1.2,[0,0,0]);u=sim.sim_controller(s,s)
    assert np.isfinite(u).all() and np.allclose(u,[0,0,3.71*9.81])
    n=sim.dynamics_model(s,u,dist_std=0,rho=1.2)
    assert np.allclose(n.pos,s.pos) and np.allclose(n.vel,0) and np.allclose(n.acc,0)
    dest=QuadState(pos=np.array([2.,3.,60.]));u=sim.sim_controller(s,dest)
    ev=dest.pos-s.pos;ev[:2]*=min(np.linalg.norm(ev[:2]),5)/np.linalg.norm(ev[:2]);ea=3*ev;ea[2]+=9.81
    roll=-np.sin(ea[1]/np.linalg.norm(ea));pitch=np.sin(ea[0]/(np.linalg.norm(ea)*np.cos(roll)))
    tilt=np.degrees(np.arccos(np.cos(roll)*np.cos(pitch)))
    if tilt>35:roll*=35/tilt;pitch*=35/tilt
    assert np.allclose(u,[roll,pitch,ea[2]*3.71])
    sim,s=create_sim([0,0,60],1.2,[0,4,0]);u=np.array([0.,0.,3.71*9.81]);n=sim.dynamics_model(s,u,dist_std=0,rho=1.2)
    expected=-.5*.65*1.2*.25*np.square(s.airVel)*np.sign(s.airVel)/3.71
    assert np.allclose(n.acc,expected) and np.allclose(n.airVel,[0,-4,0])
    # NWU north/west/up to ENU east/north/up is orthogonal and preserves speed.
    m=np.array([[0,-1,0],[1,0,0],[0,0,1]])
    assert np.allclose(m@m.T,np.eye(3)) and np.allclose(m@np.array([2,3,4]),[-3,2,4])
    return {'passed':True,'checks':['exact_hover_zero_limit','no_wind_hover_force_balance','nonzero_controller_matches_original_algebra','wind_drag_and_air_triangle','NWU_ENU_rotation'],
      'author_dynamics_source':str(SOURCE.relative_to(ROOT)).replace('\\','/'),'controller_patch':'Only two division-by-zero expressions get continuous zero limit; no gain/force/tilt change',
      'controlled_diagnostic':'dist_std=0; Dryden=explicit zero; prescribed horizontal wind; not claimed to reproduce unsteady outdoor turbulence'}

def simulate(trajectory,case):
    dt=.1;warmup=20.;last=float(trajectory['duration_s']);target_times=np.array([p['time_s'] for p in trajectory['points']])
    targets=np.array([p['nwu_m'] for p in trajectory['points']])
    times=np.arange(-int(warmup/dt),int(round(last/dt))+1)*dt
    desired=np.column_stack([np.interp(times,target_times,targets[:,j]) for j in range(3)])
    sim,state=create_sim(targets[0],case['density_kgm3'],case['wind_nwu_ms'],dt)
    extra=[]
    for t,target in zip(times,desired):
        sim.append_states_hist(float(t),state)
        extra.append([*state.pos,*state.vel,*state.rpy,*state.airVel,*target,float(np.linalg.norm(state.pos-target))])
        u=sim.sim_controller(state,QuadState(pos=target))
        state=sim.dynamics_model(state,u,dist_std=0,rho=case['density_kgm3'])
        assert np.isfinite(state.pos).all() and np.isfinite(state.rpy).all()
    cols=['time','airspeed','airspeed_angle','east_position','north_position','up_position','east_velocity','north_velocity','vertspd','aoa','heading','payload','density']
    states=pd.DataFrame(sim.states_hist,columns=cols)
    extras=pd.DataFrame(extra,columns=['pos_n','pos_w','pos_u','vel_n','vel_w','vel_u','roll','pitch','yaw','air_n','air_w','air_u','target_n','target_w','target_u','tracking_error_m'])
    for c in extras:states[c]=extras[c]
    return states
