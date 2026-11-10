"""One mixed discrete/continuous inspection model; no freeze-and-pass stages.

The scalar constraint graph is exported from the actual original SCIP model.
The finite transcription and the continuous acceptance checker are distinct.
An incumbent is never labelled physically accepted by SCIP feasibility alone.
"""
from __future__ import annotations
import math, time, json, collections
import numpy as np
from pyscipopt import Model, Expr, quicksum, sin, cos, sqrt
from Agent.Publication.engineering_joint_weather import hourly_table, sampled_weather


def dot(a,b): return sum(x*y for x,y in zip(a,b))
def cross(a,b): return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def plus(a,b): return [x+y for x,y in zip(a,b)]
def times(a,s): return [x*s for x in a]
def maximum(a,b): return (a+b+abs(a-b))/2
def minimum(a,b): return (a+b-abs(a-b))/2
def basis(n,s): return np.array([math.comb(n,k)*s**k*(1-s)**(n-k) for k in range(n+1)])


class JointModel:
    def __init__(self,scene,targets,initial,terrain,weather,road_options,soc=1.,collocation_order=5,charge_mode="event",weather_start_utc='2025-07-01T08:00:00+00:00'):
        self.scene=scene;self.targets=targets;self.initial=initial;self.terrain=terrain;self.weather=weather
        self.cfg=scene['joint_reference_profile'];self.n=len(targets);self.H=float(self.cfg['planning']['horizon_seconds']);self.soc=soc
        self.weather_table=hourly_table(scene,weather_start_utc,self.H)
        self.collocation_order=collocation_order;self.charge_mode=charge_mode
        angle_envelope=2*math.pi+math.radians(30)*self.H
        inverse=np.linalg.inv(np.array([basis(7,s) for s in np.linspace(0,1,8)]))
        self.yaw_state_bound=angle_envelope;self.yaw_control_bound=angle_envelope*np.max(np.sum(np.abs(inverse),axis=1))
        self.origin=np.array(scene['fleet']['vehicles'][0]['initial_xyz_m'])
        # Source domain, not the warm trajectory's bounding box. A polynomial
        # may stay inside its state domain while its Bernstein controls lie
        # outside it: invert eight evaluation equations for valid bounds.
        self.state_lower=np.r_[terrain.tri.points.min(axis=0),float(terrain.values.min())]
        self.state_upper=np.r_[terrain.tri.points.max(axis=0),float(terrain.values.max())+100.]
        self.control_inverse_norm=np.sum(np.abs(inverse),axis=1)
        def coefficient_bounds(lo,hi):
            center=(np.asarray(lo)+np.asarray(hi))/2
            radius=(np.asarray(hi)-np.asarray(lo))/2
            return center-self.control_inverse_norm[:,None]*radius,center+self.control_inverse_norm[:,None]*radius
        self.position_control_lower,self.position_control_upper=coefficient_bounds(self.state_lower-self.origin,self.state_upper-self.origin)
        self.pose_state_lower=np.array([-self.yaw_state_bound,math.radians(-120),math.radians(-320),1.])
        self.pose_state_upper=np.array([self.yaw_state_bound,math.radians(30),math.radians(320),119.94/6.83])
        self.pose_control_lower,self.pose_control_upper=coefficient_bounds(self.pose_state_lower,self.pose_state_upper)
        self.m=Model('inspection_one_joint_problem');self.m.hideOutput()
        self.m.setIntParam('parallel/maxnthreads',1);self.m.setIntParam('lp/threads',1)
        self.m.setRealParam('limits/gap',0.);self.m.setRealParam('limits/absgap',0.)
        # Preserve shared physical definitions; substitution previously
        # expanded the identical nonlinear expressions until memory limit.
        self.m.setBoolParam('presolving/donotaggr',True)
        self.m.setBoolParam('presolving/donotmultaggr',True)
        self.m.setIntParam('presolving/maxrounds',0)
        # Automorphism detection on the large auxiliary expression graph is
        # optional solver work, not part of the mathematical feasible domain.
        self.m.setIntParam('misc/usesymmetry',0)
        self.sol=self.m.createSol();self.vars={};self.groups={};self.rows=[];self.residuals=[]
        self.road_options=road_options;self.segments={};self.energy={};self.pose={}
        self.initial_roots={};self.initial_orders={};self.initial_task={}
        self.index={t['id']:i for i,t in enumerate(targets)}
        self.warm_energy={};self.warm_end={};self.warm_charge={};self.warm_launch={}
        for v,d in enumerate(initial):
            order=[self.index[x] for x in d['sequence']];root=order[0]
            self.initial_roots[root]=v;self.initial_orders[root]=order
            for i in order:self.initial_task[i]=root
            self.warm_energy[root]=d['aircraft_energy_Wh']
            charge=max(0,d['aircraft_energy_Wh']+.2*548-soc*548)
            delay=charge*1800/(.7*548)
            self.warm_charge[root]=charge;self.warm_launch[root]=120+delay
            self.warm_end[root]=120+delay+d['flight_seconds']
        self.initial_makespan=max(self.warm_end.values())+240
        self.initial_charge_intervals=[dict(next_flight=l,carrier=self.initial_roots[l],start_s=0.,end_s=charge*1800/(.7*548),input_power_W=.7*548*2/.9) for l,charge in self.warm_charge.items() if charge>0]
        self.timings={};self.physical_rows=0;self.boundaries={}

    def val(self,e):
        if isinstance(e,(int,float,np.number)):return float(e)
        return float(self.m.getSolVal(self.sol,e))

    def var(self,name,lb=0,ub=None,initial=0,kind='C',group='auxiliary'):
        if name in self.vars:raise ValueError('duplicate variable '+name)
        v=self.m.addVar(name=name,lb=lb,ub=ub,vtype=kind)
        self.vars[name]=v;self.groups[name]=group
        self.m.setSolVal(self.sol,v,float(initial));return v

    def row(self,e,lo=None,hi=None,name='',family='link'):
        if isinstance(e,(int,float,np.number)):
            assert (lo is None or e>=lo-1e-7) and (hi is None or e<=hi+1e-7),(name,e,lo,hi)
            return
        value=self.val(e);violation=max(0,(lo-value) if lo is not None else 0,(value-hi) if hi is not None else 0)
        if violation>1e-6:self.residuals.append(dict(name=name,value=value,lo=lo,hi=hi,violation=violation))
        if lo is not None and hi is not None and lo==hi:
            c=self.m.addCons(e==lo,name=name);self.rows.append((c,e,lo,hi,family))
        else:
            if lo is not None:
                c=self.m.addCons(e>=lo,name=name+'_lo');self.rows.append((c,e,lo,None,family))
            if hi is not None:
                c=self.m.addCons(e<=hi,name=name+'_hi');self.rows.append((c,e,None,hi,family))

    def aux(self,e,name,lb=-1e9,ub=1e9,group='physical_auxiliary'):
        v=self.var(name,lb,ub,self.val(e),group=group);self.row(v-e,0,0,name+'_def','definition');return v

    def avec(self,x,name,lb=-1e9,ub=1e9):return [self.aux(z,name+'_'+str(k),lb,ub) for k,z in enumerate(x)]

    def gate(self,e,active,lo=None,hi=None,M=1e5,name='',family='link'):
        # For affine expressions derive valid M values from actual finite
        # variable bounds. Nonlinear rows use exact binary multiplication.
        if isinstance(e,Expr) and all(len(t)<=1 for t in e.terms):
            lower=upper=0.
            for term,c in e.terms.items():
                if not term:lower+=c;upper+=c;continue
                v=list(term)[0];a=c*v.getLbOriginal();b=c*v.getUbOriginal();lower+=min(a,b);upper+=max(a,b)
            if lo is not None:self.row(e+max(0.,lo-lower)*(1-active),lo=lo,name=name+'_lo',family=family)
            if hi is not None:self.row(e-max(0.,upper-hi)*(1-active),hi=hi,name=name+'_hi',family=family)
            return
        if lo is not None and hi is not None and lo==hi:
            self.row(active*(e-lo),0,0,name,family);return
        if lo is not None:self.row(active*(e-lo),lo=0,name=name+'_lo',family=family)
        if hi is not None:self.row(active*(e-hi),hi=0,name=name+'_hi',family=family)

    def binary(self,name,initial=0,group='auxiliary'):
        return self.var(name,0,1,initial,'B',group)

    def and2(self,a,b,name):
        v=self.binary(name,round(self.val(a))*round(self.val(b)))
        self.row(v-a,hi=0,name=name+'_a');self.row(v-b,hi=0,name=name+'_b')
        self.row(v-a-b,lo=-1,name=name+'_ab');return v

    def schedule(self):
        n=self.n;self.b={};self.x={};self.y={};self.beta={};self.h={};self.last={};self.a={}
        self.s={};self.e={};self.t={};self.q0={};self.q1={};self.cs={};self.ce={};self.pin={};self.eb={};self.ea={};self.charge_on={}
        self.cm={};self.em={};self.pin_tail={}
        self.C=self.var('fleet_makespan',0,28800,self.initial_makespan,group='event_times')
        self.z=self.binary('complete_component',1,'complete_component')
        for i in range(n):
            root=self.initial_task[i];v=self.initial_roots[root]
            d=self.initial[v];cap=next(c for c in d['captures'] if self.index[c['task_id']]==i)
            self.a[i]=self.binary(f'a_{i}',1,'task_success_planned')
            self.b[i]=self.binary(f'b_{i}',i in self.initial_roots,'sortie_active')
            self.last[i]=self.binary(f'last_{i}',i==self.initial_orders[root][-1],'task_successor')
            self.t[i]=self.var(f't_capture_{i}',0,self.H,self.warm_launch[root]+cap['start_s'],group='event_times')
            self.s[i]=self.var(f'launch_{i}',0,self.H,self.warm_launch.get(i,120),group='event_times')
            self.e[i]=self.var(f'land_{i}',0,self.H,self.warm_end.get(i,120),group='event_times')
            for l in range(n):self.x[i,l]=self.binary(f'x_{i}_{l}',l==root,'task_assignment')
            self.row(sum(self.x[i,l] for l in range(n))-self.a[i],0,0,f'assign_{i}','assign')
            self.row(self.z-self.a[i],hi=0,name=f'complete_{i}',family='completion')
            for u in range(4):self.y[u,i]=self.binary(f'y_{u}_{i}',i in self.initial_roots and u==2*self.initial_roots[i],'aircraft_assignment')
            for p in range(8):self.beta[p,i]=self.binary(f'beta_{p}_{i}',i in self.initial_roots and p==4*self.initial_roots[i],'battery_pair_assignment')
            self.row(sum(self.y[u,i] for u in range(4))-self.b[i],0,0,f'aircraft_{i}','assign')
            self.row(sum(self.beta[p,i] for p in range(8))-self.b[i],0,0,f'pack_{i}','assign')
            for v in range(2):self.row(sum(self.y[u,i] for u in range(2*v,2*v+2))-sum(self.beta[p,i] for p in range(4*v,4*v+4)),0,0,f'owner_{v}_{i}','assign')
            charge=self.warm_charge.get(i,0);dt=charge*1800/(.7*548)
            self.cs[i]=self.var(f'charge_start_{i}',0,self.H,0,group='event_times')
            self.ce[i]=self.var(f'charge_end_{i}',0,self.H,dt,group='event_times')
            self.cm[i]=self.var(f'charge_knee_time_{i}',0,self.H,dt,group='event_times')
            self.pin[i]=self.var(f'charge_power_{i}',0,1000,(.7*548*2/.9) if charge>0 else 0,group='charging_input_power')
            self.charge_on[i]=self.binary(f'charge_channel_{i}',charge>0,'charging_channel')
            self.eb[i]=self.var(f'pack_before_{i}',109.6,548,soc_energy:=self.soc*548,group='battery_energy')
            self.ea[i]=self.var(f'pack_after_charge_{i}',109.6,548,soc_energy+charge,group='battery_energy')
            self.em[i]=self.var(f'pack_at_charge_knee_{i}',109.6,548,soc_energy+charge,group='battery_energy')
            self.pin_tail[i]=self.var(f'charge_tail_power_{i}',0,182.6666666666667,0,group='charging_input_power')
            if self.charge_mode=="two_phase":
                self.row(self.ce[i]-self.cs[i],lo=0,name=f'charge_interval_{i}',family='battery_availability')
                self.row(self.cm[i]-self.cs[i],lo=0,name=f'charge_fast_interval_{i}',family='battery_availability')
                self.row(self.ce[i]-self.cm[i],lo=0,name=f'charge_tail_interval_{i}',family='battery_availability')
                self.row(self.em[i]-self.eb[i]-.9*self.pin[i]*(self.cm[i]-self.cs[i])/3600,0,0,f'charge_fast_conservation_{i}','battery_conservation')
                self.row(self.ea[i]-self.em[i]-.9*self.pin_tail[i]*(self.ce[i]-self.cm[i])/3600,0,0,f'charge_tail_conservation_{i}','battery_conservation')
                self.row(self.pin[i]-852.4444444444445*self.charge_on[i],hi=0,name=f'charge_rate_{i}',family='battery_conservation')
                self.row(self.pin_tail[i]-182.6666666666667*self.charge_on[i],hi=0,name=f'charge_tail_rate_{i}',family='battery_conservation')
                fast=self.binary(f'charge_fast_active_{i}',charge>0)
                tail=self.binary(f'charge_tail_active_{i}',0)
                self.row(self.pin[i]-852.4444444444445*fast,hi=0,name=f'charge_fast_switch_{i}',family='battery_conservation')
                self.row(self.pin_tail[i]-182.6666666666667*tail,hi=0,name=f'charge_tail_switch_{i}',family='battery_conservation')
                # The fast phase may end at/below90%; the tail may start at/above90%.
                # A zero-charge phase can vanish, including a full initial pack.
                self.gate(self.em[i],fast,hi=493.2,name=f'charge_fast_knee_{i}',family='battery_conservation')
                self.gate(self.em[i],tail,lo=493.2,name=f'charge_tail_knee_{i}',family='battery_conservation')
            else:
                for unused in [self.pin[i],self.pin_tail[i],self.charge_on[i]]:
                    self.m.chgVarUb(unused,0);self.m.setSolVal(self.sol,unused,0)
                self.m.setSolVal(self.sol,self.em[i],soc_energy);self.m.setSolVal(self.sol,self.cm[i],0)
                self.row(self.em[i]-self.eb[i],0,0,f"unused_charge_middle_{i}","definition")
                self.row(self.cm[i]-self.cs[i],0,0,f"unused_charge_time_{i}","definition")
            self.row(self.charge_on[i]-self.b[i],hi=0,name=f'charge_active_{i}')
            self.gate(self.s[i]-self.ce[i],self.b[i],lo=120,M=self.H,name=f'prepare_{i}',family='handling_capacity')
            self.gate(self.C-self.e[i],self.b[i],lo=240,M=28800,name=f'recovery_{i}',family='time_window')
            for j in range(n):
                if i!=j:self.h[i,j]=self.binary(f'h_{i}_{j}',any((i,j) in list(zip(o,o[1:])) for o in self.initial_orders.values()),'task_successor')
        self.row(self.z-sum(self.a.values()),lo=1-n,name='complete_all',family='completion')
        from Agent.Publication.engineering_flight_labels import add_labels,couple
        flight_labels=add_labels(self)
        for i in range(n):
            self.row(self.b[i]+sum(self.h[j,i] for j in range(n) if j!=i)-self.a[i],0,0,f'flow_in_{i}','assign')
            self.row(self.last[i]+sum(self.h[i,j] for j in range(n) if j!=i)-self.a[i],0,0,f'flow_out_{i}','assign')
            self.row(self.x[i,i]-self.b[i],0,0,f'root_label_{i}','assign')
            for l in range(n):
                self.row(self.x[i,l]-self.b[l],hi=0,name=f'active_assignment_{i}_{l}',family='assign')
            for j in range(n):
                if i==j:continue
                couple(self,flight_labels,i,j)
        print('joint schedule variables',len(self.vars),flush=True)

    def polynomial(self,cp,T,s,count=3):
        out=[];n=len(cp)-1
        for r in range(count+1):
            coef=np.zeros(n+1);bb=basis(n-r,s);factor=math.factorial(n)/math.factorial(n-r)
            for k,b in enumerate(bb):
                if not b:continue
                for h in range(r+1):coef[k+h]+=factor*b*(-1)**(r-h)*math.comb(r,h)
            out.append([sum(float(c)*cp[k][j] for k,c in enumerate(coef) if c!=0)/(T**r if r else 1) for j in range(len(cp[0]))])
        return out

    def boundary(self,block,field,s,count):
        key=(block['key'],field,s)
        if key not in self.boundaries:
            states=self.polynomial(block[field],block['T'],s,count)
            self.boundaries[key]=[self.state_aux(z,block['key']+f'_{field}_end{s}_d{r}',field,r) for r,z in enumerate(states)]
        return self.boundaries[key]

    def state_aux(self,z,name,field,derivative):
        if field=='C':
            if derivative==0:
                return [self.aux(x,name+'_'+str(q),float(self.state_lower[q]-self.origin[q]),float(self.state_upper[q]-self.origin[q])) for q,x in enumerate(z)]
            bound=[None,5,3,5][derivative]
            return self.avec(z,name,-bound,bound)
        if derivative==0:
            lo=[-self.yaw_state_bound,math.radians(-120),math.radians(-320),1];hi=[self.yaw_state_bound,math.radians(30),math.radians(320),119.94/6.83]
        else:
            hi=[math.radians(30)]*3+[10/6.83 if derivative==1 else 1e5];lo=[-x for x in hi]
        return [self.aux(x,name+'_'+str(k),lo[k],hi[k]) for k,x in enumerate(z)]

    def trajectories(self):
        seeds={};incoming={};captures={};recovery={}
        for v,d in enumerate(self.initial):
            pending=[]
            for seg in d['timeline']:
                if seg['kind']=='capture':
                    i=self.index[seg['task_id']];incoming[i]=pending;captures[i]=seg;pending=[]
                else:pending.append(seg)
            recovery[self.index[d['sequence'][-1]]]=pending
        self.duration_in={};self.duration_cap={};self.duration_rec={};self.task_energy={};self.rec_energy={}
        reference_rec=next(iter(recovery.values()))
        for i in range(self.n):
            for role,seq,active in [('in',incoming[i],self.a[i]),('cap',[captures[i]],self.a[i]),('rec',recovery.get(i,reference_rec),self.last[i])]:
                assert len(seq)==(1 if role=='cap' else 3)
                blocks=[]
                for k,seg in enumerate(seq):
                    key=f'{role}_{i}_{k}';cp0=np.asarray(seg['control_points_m'])-self.origin;ap0=np.asarray(seg['pose_controls_yaw_gp_gy_focal_ratio']);dt=seg['end_s']-seg['start_s']
                    C=[[self.var(f'{key}_p_{r}_{j}',float(self.position_control_lower[r,j]),float(self.position_control_upper[r,j]),cp0[r,j],group='aircraft_position_control_points') for j in range(3)] for r in range(8)]
                    A=[[self.var(f'{key}_a_{r}_{j}',float(self.pose_control_lower[r,j]),float(self.pose_control_upper[r,j]),ap0[r,j],group=['aircraft_yaw','gimbal_angles','gimbal_angles','camera_focal_length'][j]) for j in range(4)] for r in range(8)]
                    T=self.var(f'{key}_duration',.1,self.H,dt,group='segment_duration')
                    block=dict(key=key,C=C,A=A,T=T,active=active,source=seg,task=i,role=role,segment=k)
                    blocks.append(block);self.segments[key]=block
                for before,after in zip(blocks,blocks[1:]):self.join(before,after,active,f'{role}_{i}_join')
                if role=='in':self.duration_in[i]=sum(b['T'] for b in blocks)
                elif role=='cap':self.duration_cap[i]=blocks[0]['T']
                else:self.duration_rec[i]=sum(b['T'] for b in blocks)
        # Register every curve before connecting arbitrary task predecessors.
        for i in range(self.n):
            self.join(self.segments[f'in_{i}_2'],self.segments[f'cap_{i}_0'],self.a[i],f'in_cap_{i}')
            self.join(self.segments[f'cap_{i}_0'],self.segments[f'rec_{i}_0'],self.last[i],f'cap_rec_{i}')
            dwell=self.targets[i]['observation_requirement']['min_dwell_s']
            self.gate(self.duration_cap[i],self.a[i],lo=dwell,M=dwell,name=f'dwell_{i}',family='time_window')
            self.gate(self.t[i]-self.duration_in[i]-self.s[i],self.b[i],0,0,3*self.H,f'root_clock_{i}','time_window')
            for j in range(self.n):
                if i==j:continue
                h=self.h[i,j]
                self.gate(self.t[j]-self.duration_in[j]-self.t[i]-self.duration_cap[i],h,0,0,5*self.H,f'arc_clock_{i}_{j}','time_window')
                self.join(self.segments[f'cap_{i}_0'],self.segments[f'in_{j}_0'],h,f'arc_state_{i}_{j}')
            for l in range(self.n):
                last=self.and2(self.x[i,l],self.last[i],f'last_member_{i}_{l}')
                self.gate(self.t[i]+self.duration_cap[i]+self.duration_rec[i]-self.e[l],last,0,0,5*self.H,f'end_clock_{i}_{l}','time_window')
        print('joint trajectories',len(self.segments),'variables',len(self.vars),flush=True)

    def join(self,before,after,active,name):
        for field,count in [('C',3),('A',2)]:
            left=self.boundary(before,field,1,count);right=self.boundary(after,field,0,count)
            for r,(a,b) in enumerate(zip(left,right)):
                for j,(x,y) in enumerate(zip(a,b)):self.gate(x-y,active,0,0,1e7,f'{name}_{field}_{r}_{j}','kinematics')

    def prepare_terrain(self):
        self.lower=self.state_lower;self.upper=self.state_upper
        tri=self.terrain.tri;self.triangles=[]
        for k,ids in enumerate(tri.simplices):
            coords=tri.points[ids]
            transform=tri.transform[k];z=self.terrain.values[ids,0]
            gradient=(z[:2]-z[2])@transform[:2]
            plane=np.r_[gradient,z[2]-gradient@coords[2]]
            self.triangles.append((k,transform,plane))
        print('joint terrain triangles',len(self.triangles),flush=True)

    def ground(self,p,key):
        point=np.array([self.val(x) for x in p]);warm=float(self.terrain(*point[:2]));simplex=int(self.terrain.tri.find_simplex(point[:2]))
        g=self.var(key+'_ground',float(self.terrain.values.min()),float(self.terrain.values.max()),warm,group='terrain_auxiliary');choice=[];gx_terms=[];gy_terms=[];incident={};seed_weights={}
        for idx,transform,plane in self.triangles:
            b=self.var(f'{key}_tri_{idx}',0,1,idx==simplex,group='terrain_auxiliary');choice.append(b)
            vertices=self.terrain.tri.simplices[idx]
            for vertex in vertices:incident.setdefault(int(vertex),[]).append(b)
            if idx==simplex:
                a=transform[:2]@(point[:2]-transform[2]);weights=[a[0],a[1],1-a.sum()]
                for vertex,w in zip(vertices,weights):seed_weights[int(vertex)]=float(w)
            gx_terms.append(float(plane[0])*b);gy_terms.append(float(plane[1])*b)
        # SOS1 and sum(phi)=1 select exactly one triangle. Vertex weights are
        # nonzero only for its three vertices. This is the SAME piecewise
        # affine DEM as the earlier one-hot big-M representation.
        sos=self.m.addConsSOS1(choice,name=key+'_terrain_SOS1');self.rows.append((sos,None,None,None,'terrain_SOS1'))
        self.row(quicksum(choice),1,1,key+'_terrain_cell','terrain')
        lam={vertex:self.var(f'{key}_lambda_{vertex}',0,1,seed_weights.get(vertex,0.),group='terrain_auxiliary') for vertex in incident}
        for vertex,var in lam.items():self.row(var-sum(incident[vertex]),hi=0,name=key+f'_vertex_support{vertex}',family='terrain')
        self.row(quicksum(lam.values()),1,1,key+'_barycentric','terrain')
        for q in range(2):self.row(p[q]-self.origin[q]-quicksum(lam[vertex]*float(self.terrain.tri.points[vertex,q]-self.origin[q]) for vertex in lam),0,0,key+f'_terrain_xy{q}','terrain')
        self.row(g-quicksum(lam[vertex]*float(self.terrain.values[vertex,0]) for vertex in lam),0,0,key+'_terrain_z','terrain')
        # Share the two DEM gradients. Inlining thousands of selected-cell
        # terms into every wind/force derivative causes polynomial expansion.
        # These are exact defining equalities, not a different terrain model.
        gx=self.aux(quicksum(gx_terms),key+'_ground_dx',min(x[2][0] for x in self.triangles),max(x[2][0] for x in self.triangles),'terrain_auxiliary')
        gy=self.aux(quicksum(gy_terms),key+'_ground_dy',min(x[2][1] for x in self.triangles),max(x[2][1] for x in self.triangles),'terrain_auxiliary')
        return g,gx,gy

    def physics(self,block,s,key):
        cfg=self.cfg;air=cfg['aircraft'];en=cfg['energy'];active=block['active']
        p,v,a,j=[self.state_aux(x,key+f'_state{r}','C',r) for r,x in enumerate(self.polynomial(block['C'],block['T'],s))]
        angles,rates,accels=[self.state_aux(x,key+f'_pose{r}','A',r) for r,x in enumerate(self.polynomial(block['A'],block['T'],s,2))]
        world=[p[q]+self.origin[q] for q in range(3)]
        g,gx,gy=self.ground(world,key)
        agl=self.aux(world[2]-g,key+'_agl',-300,300)
        self.gate(agl,active,hi=100,M=500,name=key+'_ceiling',family='weather_power')
        clearance=sum(air[k] for k in ['body_radius_m','obstacle_clearance_m','tracking_error_m','map_error_m'])
        minimum_agl=clearance
        if block['role']=='in' and block['segment']==0:minimum_agl=clearance-(clearance-.6)*self.b[block['task']]
        if block['role']=='rec' and block['segment']==2:minimum_agl=.6
        self.gate(agl-minimum_agl,active,lo=0,name=key+'_terrain_clearance',family='continuous_collision_sample')
        low=self.aux(minimum(1,maximum(0,(agl-30)/30)),key+'_low',0,1)
        high=self.aux(minimum(1,maximum(0,(agl-60)/40)),key+'_high',0,1)
        wz,rho,absolute_clock=sampled_weather(self,block,s,key,agl,low,high)
        density=self.aux(rho[0]+low*(rho[1]-rho[0])+high*(rho[2]-rho[1]),key+'_rho',.5,2)
        wind=[wz[0][q]+low*(wz[1][q]-wz[0][q])+high*(wz[2][q]-wz[1][q]) for q in range(3)]
        va=self.avec([v[q]-wind[q] for q in range(3)],key+'_vair',-30,30)
        speed=self.aux(sqrt(dot(va,va)+1e-12),key+'_airspeed',1e-9,60)
        F=self.avec([air['mass_kg']*(a[q]+(9.81 if q==2 else 0))+.5*air['drag_CdA_m2']*density*speed*va[q] for q in range(3)],key+'_force',-300,300)
        Fn=self.aux(sqrt(dot(F,F)+1e-12),key+'_force_norm',1e-9,600)
        b3=self.avec([x/Fn for x in F],key+'_b3',-1,1)
        heading=[cos(angles[0]),sin(angles[0]),0]
        v2=cross(b3,heading);n2=self.aux(sqrt(dot(v2,v2)+1e-12),key+'_n2',1e-9,2)
        b2=self.avec([x/n2 for x in v2],key+'_b2',-1,1);b1=self.avec(cross(b2,b3),key+'_b1',-1,1)
        rb=[b1,b2,b3]
        # Exact derivative on each source weather-height interval. At a layer
        # boundary the independent interval checker evaluates both sides.
        region=[];warm_height=self.val(agl);warm_region=0 if warm_height<30 else (1 if warm_height<60 else 2)
        for r,(lo,hi) in enumerate([(-300,30),(30,60),(60,300)]):
            indicator=self.binary(key+f'_height_region{r}',r==warm_region,'weather_auxiliary');region.append(indicator)
            self.gate(agl,indicator,lo,hi,name=key+f'_height_bounds{r}',family='weather_power')
        self.row(sum(region),1,1,key+'_height_region','weather_power')
        dh=v[2]-gx*v[0]-gy*v[1]
        wg=[region[1]*(wz[1][q]-wz[0][q])/30+region[2]*(wz[2][q]-wz[1][q])/40 for q in range(3)]
        rg=region[1]*(rho[1]-rho[0])/30+region[2]*(rho[2]-rho[1])/40
        vad=self.avec([a[q]-wg[q]*dh for q in range(3)],key+'_vair_dot')
        speed_dot=dot(va,vad)/speed
        fd=self.avec([air['mass_kg']*j[q]+.5*air['drag_CdA_m2']*(rg*dh*speed*va[q]+density*speed_dot*va[q]+density*speed*vad[q]) for q in range(3)],key+'_force_dot')
        b3d=self.avec([(fd[q]-b3[q]*dot(b3,fd))/Fn for q in range(3)],key+'_b3_dot')
        hd=[-sin(angles[0])*rates[0],cos(angles[0])*rates[0],0]
        v2d=plus(cross(b3d,heading),cross(b3,hd));b2d=self.avec([(v2d[q]-b2[q]*dot(b2,v2d))/n2 for q in range(3)],key+'_b2_dot')
        b1d=self.avec(plus(cross(b2d,b3),cross(b2,b3d)),key+'_b1_dot');rbd=[b1d,b2d,b3d]
        omega=[dot(b3,b2d),dot(b1,b3d),dot(b2,b1d)]
        for q,x in enumerate(omega):self.gate(x,active,-math.radians(30),math.radians(30),name=key+f'_body_rate{q}',family='attitude')
        self.gate(dot(v,v),active,hi=air['ground_speed_max_mps']**2,M=1e12,name=key+'_vg',family='kinematics')
        self.gate(v[2],active,-air['vertical_speed_max_mps'],air['vertical_speed_max_mps'],1e8,key+'_vz','kinematics')
        self.gate(dot(va,va),active,hi=air['air_speed_max_mps']**2,M=1e12,name=key+'_va',family='kinematics')
        for state,limit,name in [(a,air['acceleration_max_mps2'],'acc'),(j,air['jerk_max_mps3'],'jerk')]:self.gate(dot(state,state),active,hi=limit**2,M=1e16,name=key+'_'+name,family='kinematics')
        self.gate(Fn,active,hi=air['thrust_max_N'],M=1e9,name=key+'_thrust',family='attitude')
        self.gate(F[2]-math.cos(math.radians(air['tilt_max_deg']))*Fn,active,lo=0,M=1e9,name=key+'_tilt',family='attitude')
        for q in range(3):
            self.gate(rates[q],active,-math.radians(30),math.radians(30),1e7,key+f'_rate_{q}','attitude')
            self.gate(accels[q],active,-math.radians(30),math.radians(30),1e9,key+f'_angular_acc_{q}','attitude')
        self.gate(rates[3]*6.83,active,-10,10,1e7,key+'_zoom_rate','camera_fov')
        power=self.aux(en['hover_power_W']*sqrt(1.225/density)+.5*air['drag_CdA_m2']*density*speed**3+air['mass_kg']*9.81*maximum(v[2],0)/en['propulsion_efficiency'],key+'_power',0,100000)
        return dict(position=world,velocity=v,acceleration=a,jerk=j,angles=angles,rates=rates,ground=g,rotation=rb,rotation_dot=rbd,power=power,agl=agl)

    def optics(self,block,s,data,key):
        cfg=self.cfg['camera'];task=self.targets[block['task']];active=block['active'];a=data['angles'];rb=data['rotation'];p=data['position']
        gp,gy=a[1:3];opt=[cos(gp)*cos(gy),cos(gp)*sin(gy),sin(gp)];right=[sin(gy),-cos(gy),0];down=cross(opt,right)
        def rotate(x):return [sum(rb[k][j]*x[k] for k in range(3)) for j in range(3)]
        rc=[self.avec(rotate(x),key+'_cam_'+str(k),-1,1) for k,x in enumerate([right,down,opt])]
        gpd,gyd=data['rates'][1:3];optd=[-sin(gp)*gpd*cos(gy)-cos(gp)*sin(gy)*gyd,-sin(gp)*gpd*sin(gy)+cos(gp)*cos(gy)*gyd,cos(gp)*gpd]
        rightd=[cos(gy)*gyd,sin(gy)*gyd,0];downd=plus(cross(optd,right),cross(opt,rightd));rbd=data['rotation_dot']
        rcd=[self.avec([sum(rbd[k][j]*x[k]+rb[k][j]*xd[k] for k in range(3)) for j in range(3)],key+'_cam_dot_'+str(c)) for c,(x,xd) in enumerate(zip([right,down,opt],[rightd,downd,optd]))]
        target=task['target'];center=np.array([target[z] for z in ['easting_m','northing_m','altitude_m']]);nominal=np.array([task[z] for z in ['easting_m','northing_m','altitude_m']])
        delta=[p[j]-center[j] for j in range(3)];dist=self.aux(sqrt(dot(delta,delta)+1e-12),key+'_target_distance',1e-9,10000)
        self.gate(dist,active,8,25,1e5,key+'_focus','camera_fov')
        self.gate(dot(target['normal_world'],delta)-math.cos(math.radians(30))*dist,active,lo=0,M=1e5,name=key+'_incidence',family='camera_fov')
        self.gate(sum((p[j]-nominal[j])**2 for j in range(3)),active,hi=task['observation_requirement']['viewpoint_ball_radius_m']**2,M=1e8,name=key+'_view_ball',family='camera_fov')
        fx0=math.hypot(5184,3888)/(2*math.tan(math.radians(66.6/2)));fx=fx0*a[3]
        pixels=[]
        for k,point in enumerate(target['surface_samples_utm_m']):
            qc=self.avec([dot(col,[point[j]-p[j] for j in range(3)]) for col in rc],key+f'_q{k}',-10000,10000)
            self.gate(qc[2],active,lo=.1,M=20000,name=key+f'_depth{k}',family='camera_fov')
            # Positive-depth denominator coincides exactly with projection on
            # active captures; prevents undefined inactive expressions.
            depth=self.aux(maximum(qc[2],.05),key+f'_depth_safe{k}',.05,20000)
            px=self.aux(fx*qc[0]/depth,key+f'_px{k}',-1e5,1e5);py=self.aux(fx*qc[1]/depth,key+f'_py{k}',-1e5,1e5);pixels.append((px,py))
            self.gate(px,active,-2592+32,2592-32,1e10,key+f'_fovx{k}','camera_fov');self.gate(py,active,-1944+32,1944-32,1e10,key+f'_fovy{k}','camera_fov')
            qcd=[dot(cold,[point[j]-p[j] for j in range(3)])-dot(col,data['velocity']) for col,cold in zip(rc,rcd)]
            for q in range(2):
                pixel_rate=fx0*data['rates'][3]*qc[q]/depth+fx*(qcd[q]*depth-qc[q]*qcd[2])/depth**2
                self.gate(.001*pixel_rate,active,-1,1,name=key+f'_blur_exact_{k}_{q}',family='motion_blur')
            J=[]
            for tangent in target['tangent_basis']:
                rv=[dot(col,tangent) for col in rc]
                J.append([self.aux(fx*(rv[q]*depth-qc[q]*rv[2])/depth**2,key+f'_J{k}_{len(J)}_{q}',-1e12,1e12) for q in range(2)])
            aa=self.aux(dot(J[0],J[0])*cfg['gsd_max_m_per_px']**2-1,key+f'_gsd_aa{k}',-1,1e12)
            bb=self.aux(dot(J[1],J[1])*cfg['gsd_max_m_per_px']**2-1,key+f'_gsd_bb{k}',-1,1e12)
            ab=self.aux(dot(J[0],J[1])*cfg['gsd_max_m_per_px']**2,key+f'_gsd_ab{k}',-1e12,1e12)
            for expr,label in [(aa,'a'),(bb,'b'),(aa*bb-ab*ab,'det')]:self.gate(expr,active,lo=0,M=1e30,name=key+f'_gsd_{label}{k}',family='surface_resolution')
        return pixels

    def physical_transcription(self):
        self.prepare_terrain();nodes,weights=np.polynomial.legendre.leggauss(self.collocation_order);nodes=(nodes+1)/2;weights=weights/2
        for index,block in enumerate(self.segments.values()):
            energy=0;capture_nodes=[]
            for k,(s,w) in enumerate(zip(nodes,weights)):
                key=block['key']+f'_node{k}';data=self.physics(block,float(s),key);energy+=float(w)*block['T']*data['power']/3600
                if block['role']=='cap':capture_nodes.append((float(s),self.optics(block,float(s),data,key)))
            self.energy[block['key']]=self.aux(energy,block['key']+'_Wh',0,1e12,'energy_quadrature')
            if index%7==6:print('joint physics',index+1,'/',len(self.segments),'vars',len(self.vars),'rows',len(self.rows),flush=True)
        for i in range(self.n):
            self.task_energy[i]=sum(self.energy[f'in_{i}_{k}'] for k in range(3))+self.energy[f'cap_{i}_0']
            self.rec_energy[i]=sum(self.energy[f'rec_{i}_{k}'] for k in range(3))

    def roads_and_rendezvous(self):
        self.route={};self.site_position={};self.site_yaw={};self.arr={};self.dep={};self.launch_visit={};self.return_visit={}
        for v,options in enumerate(self.road_options):
            for r in range(len(options)):self.route[v,r]=self.binary(f'road_route_{v}_{r}',r==0,'vehicle_road_arcs')
            self.row(sum(self.route[v,r] for r in range(len(options))),1,1,f'one_road_route_{v}','road_vehicle_motion')
            for k in range(3):
                self.site_position[v,k]=[self.aux(sum(self.route[v,r]*(options[r]['sites'][k]['xyz_m'][j]-self.origin[j]) for r in range(len(options))),f'vehicle_{v}_visit{k}_p{j}',-10000,10000,'vehicle_motion') for j in range(3)]
                self.site_yaw[v,k]=self.aux(sum(self.route[v,r]*options[r]['sites'][k]['yaw_rad'] for r in range(len(options))),f'vehicle_{v}_visit{k}_yaw',-2*math.pi,2*math.pi,'vehicle_motion')
                self.arr[v,k]=self.var(f'vehicle_{v}_arrival{k}',0,28800,0 if k==0 else self.initial_makespan,group='event_times')
                self.dep[v,k]=self.var(f'vehicle_{v}_departure{k}',0,28800,self.initial_makespan,group='event_times')
                self.row(self.dep[v,k]-self.arr[v,k],lo=0,name=f'vehicle_dwell_{v}_{k}',family='road_vehicle_motion')
                if k==0:self.row(self.arr[v,k],0,0,f'vehicle_initial_{v}','road_vehicle_motion')
                else:
                    scale=self.var(f'vehicle_speed_scale_{v}_{k}',1,20,1,group='vehicle_motion')
                    travel=sum(self.route[v,r]*options[r]['leg_seconds'][k-1] for r in range(len(options)))
                    self.row(self.arr[v,k]-self.dep[v,k-1]-scale*travel,0,0,f'vehicle_travel_{v}_{k}','road_vehicle_motion')
            self.row(self.C-self.arr[v,2],lo=0,name=f'vehicle_return_{v}',family='road_vehicle_motion')
        launchpads={};returnpads={};launchyaw={};returnyaw={}
        for l in range(self.n):
            v=self.initial_roots.get(l,0);xyz=np.asarray(self.scene['fleet']['vehicles'][v]['initial_xyz_m'])-self.origin;yaw=self.scene['fleet']['vehicles'][v]['initial_heading_rad']
            launchpads[l]=[self.var(f'launch_pad_{l}_{j}',-10000,10000,xyz[j],group='vehicle_motion') for j in range(3)]
            returnpads[l]=[self.var(f'return_pad_{l}_{j}',-10000,10000,xyz[j],group='vehicle_motion') for j in range(3)]
            launchyaw[l]=self.var(f'launch_pad_yaw_{l}',-2*math.pi,2*math.pi,yaw,group='vehicle_motion');returnyaw[l]=self.var(f'return_pad_yaw_{l}',-2*math.pi,2*math.pi,yaw,group='vehicle_motion')
        for l in range(self.n):
            for v in range(2):
                owner=sum(self.y[u,l] for u in range(2*v,2*v+2))
                for k in range(3):
                    warm=l in self.initial_roots and self.initial_roots[l]==v and k==0
                    L=self.binary(f'launch_visit_{l}_{v}_{k}',warm,'vehicle_road_arcs');R=self.binary(f'return_visit_{l}_{v}_{k}',warm,'vehicle_road_arcs')
                    self.launch_visit[l,v,k]=L;self.return_visit[l,v,k]=R
                    self.gate(self.s[l]-self.arr[v,k],L,lo=120,name=f'launch_arrival_{l}_{v}_{k}',family='rendezvous')
                    self.gate(self.dep[v,k]-self.s[l],L,lo=0,name=f'launch_departure_{l}_{v}_{k}',family='rendezvous')
                    self.gate(self.e[l]-self.arr[v,k],R,lo=0,name=f'return_arrival_{l}_{v}_{k}',family='rendezvous')
                    self.gate(self.dep[v,k]-self.e[l],R,lo=240,name=f'return_departure_{l}_{v}_{k}',family='rendezvous')
                    # One preflight charging interval at the chosen launch
                    # visit: declared finite operation parameterization.
                    if self.charge_mode=="two_phase":
                        charging_here=self.and2(L,self.charge_on[l],f'charge_here_{l}_{v}_{k}')
                        self.gate(self.cs[l]-self.arr[v,k],charging_here,lo=0,name=f'charge_arrival_{l}_{v}_{k}',family='battery_availability')
                        self.gate(self.dep[v,k]-self.ce[l],charging_here,lo=0,name=f'charge_departure_{l}_{v}_{k}',family='battery_availability')
                    for j in range(3):
                        self.gate(launchpads[l][j]-self.site_position[v,k][j],L,0,0,name=f'launch_pad_link_{l}_{v}_{k}_{j}',family='rendezvous')
                        self.gate(returnpads[l][j]-self.site_position[v,k][j],R,0,0,name=f'return_pad_link_{l}_{v}_{k}_{j}',family='rendezvous')
                    self.gate(launchyaw[l]-self.site_yaw[v,k],L,0,0,name=f'launch_yaw_link_{l}_{v}_{k}',family='rendezvous')
                    self.gate(returnyaw[l]-self.site_yaw[v,k],R,0,0,name=f'return_yaw_link_{l}_{v}_{k}',family='rendezvous')
                self.row(sum(self.launch_visit[l,v,k] for k in range(3))-owner,0,0,f'launch_owner_{l}_{v}','rendezvous')
                self.row(sum(self.return_visit[l,v,k] for k in range(3))-owner,0,0,f'return_owner_{l}_{v}','rendezvous')
            self.endpoint(self.segments[f'in_{l}_0'],0,self.b[l],launchpads[l],launchyaw[l],f'launch_endpoint_{l}')
        for i in range(self.n):
            xyz=[self.aux(sum(self.x[i,l]*returnpads[l][j] for l in range(self.n)),f'end_pad_{i}_{j}',-10000,10000,'vehicle_motion') for j in range(3)]
            yaw=self.aux(sum(self.x[i,l]*returnyaw[l] for l in range(self.n)),f'end_pad_yaw_{i}',-2*math.pi,2*math.pi,'vehicle_motion')
            self.endpoint(self.segments[f'rec_{i}_2'],1,self.last[i],xyz,yaw,f'return_endpoint_{i}')
        print('joint vehicle/rendezvous linked',flush=True)

    def endpoint(self,block,s,active,vehicle_position,yaw,name):
        pstate=self.boundary(block,'C',s,3);astate=self.boundary(block,'A',s,2)
        pad=[vehicle_position[j]+(1.8 if j==2 else 0) for j in range(3)]
        for j in range(3):self.gate(pstate[0][j]-pad[j],active,0,0,name=name+f'_p{j}',family='rendezvous')
        for r in range(1,4):
            for j in range(3):self.gate(pstate[r][j],active,0,0,name=name+f'_d{r}_{j}',family='rendezvous')
        for j,value in enumerate([yaw,0,0,1]):self.gate(astate[0][j]-value,active,0,0,name=name+f'_pose{j}',family='rendezvous')
        for r in range(1,3):
            for j in range(4):self.gate(astate[r][j],active,0,0,name=name+f'_pose_d{r}_{j}',family='rendezvous')
        for r in range(8):
            for j in range(2):self.gate(block['C'][r][j]-pad[j],active,0,0,name=name+f'_vertical{r}_{j}',family='rendezvous')

    def resources(self):
        self.flight_energy={};self.after={}
        for l in range(self.n):
            e=sum(self.x[i,l]*(self.task_energy[i]+self.last[i]*self.rec_energy[i]) for i in range(self.n))
            self.flight_energy[l]=self.aux(e,f'flight_energy_{l}',0,1e10,'battery_energy')
            used=self.val(e);on=self.val(self.b[l])>.5
            after0=self.soc*548+self.warm_charge.get(l,0)-used
            self.after[l]=self.var(f'pack_after_flight_{l}',109.6,548,max(109.6,after0),group='battery_energy')
            self.row(self.after[l]-self.ea[l]+e,0,0,f'flight_conservation_{l}','battery_conservation')
            self.gate(self.ea[l]-e,self.b[l],lo=109.6,name=f'recovery_reserve_{l}',family='emergency_recovery')
        # Each pack's active flights form a path from its actual initial state.
        for p in range(8):
            first={};last={};arc={}
            for l in range(self.n):
                chosen=self.val(self.beta[p,l])>.5
                first[l]=self.binary(f'pack_first_{p}_{l}',chosen);last[l]=self.binary(f'pack_last_{p}_{l}',chosen)
                self.gate(self.eb[l],first[l],self.soc*548,self.soc*548,name=f'pack_initial_{p}_{l}',family='battery_conservation')
                for k in range(self.n):
                    if k==l:continue
                    arc[l,k]=self.binary(f'pack_next_{p}_{l}_{k}')
                    self.gate(self.eb[k]-self.after[l],arc[l,k],0,0,name=f'pack_carry_{p}_{l}_{k}',family='battery_conservation')
                    self.gate(self.cs[k]-self.e[l],arc[l,k],lo=240,name=f'pack_reuse_{p}_{l}_{k}',family='battery_availability')
            for l in range(self.n):
                self.row(first[l]+sum(arc[k,l] for k in range(self.n) if k!=l)-self.beta[p,l],0,0,f'pack_flow_in_{p}_{l}','battery_availability')
                self.row(last[l]+sum(arc[l,k] for k in range(self.n) if k!=l)-self.beta[p,l],0,0,f'pack_flow_out_{p}_{l}','battery_availability')
            self.row(sum(first.values()),hi=1,name=f'pack_one_chain_{p}',family='battery_availability')
        for l in range(self.n):
            for k in range(l):
                for u in range(4):
                    both=self.and2(self.y[u,l],self.y[u,k],f'same_aircraft_{u}_{l}_{k}')
                    self.no_overlap(self.s[l]-120,self.e[l]+240,self.s[k]-120,self.e[k]+240,both,f'aircraft_{u}_{l}_{k}','handling_capacity')
                for v in range(2):
                    owner_l=sum(self.y[u,l] for u in range(2*v,2*v+2));owner_k=sum(self.y[u,k] for u in range(2*v,2*v+2))
                    both=self.and2(owner_l,owner_k,f'same_vehicle_{v}_{l}_{k}')
                    for a0,a1,labela in [(self.s[l]-120,self.s[l],'pre'),(self.e[l],self.e[l]+240,'post')]:
                        for b0,b1,labelb in [(self.s[k]-120,self.s[k],'pre'),(self.e[k],self.e[k]+240,'post')]:self.no_overlap(a0,a1,b0,b1,both,f'handling_{v}_{l}_{k}_{labela}_{labelb}','handling_capacity')
                    if self.charge_mode=="two_phase":
                        charge_both=self.and2(self.and2(self.charge_on[l],self.charge_on[k],f'charge_both_{v}_{l}_{k}'),both,f'charge_vehicle_{v}_{l}_{k}')
                        self.no_overlap(self.cs[l],self.ce[l],self.cs[k],self.ce[k],charge_both,f'charger_{v}_{l}_{k}','battery_availability')
        if self.charge_mode=="event":
            from Agent.Publication.engineering_event_charging import add_event_charging
            add_event_charging(self)
        self.total_energy=self.aux(sum(self.flight_energy.values()),'total_aircraft_energy',0,1e12,'objective')
        print('all resource decisions linked in the same model',flush=True)

    def no_overlap(self,a0,a1,b0,b1,both,name,family):
        before=self.binary(name+'_before',self.val(a1)<=self.val(b0))
        left=self.and2(both,before,name+'_left');right=self.and2(both,1-before,name+'_right')
        self.gate(b0-a1,left,lo=0,name=name+'_a_then_b',family=family);self.gate(a0-b1,right,lo=0,name=name+'_b_then_a',family=family)

    def compile(self,witnesses=()):
        begin=time.perf_counter()
        self.schedule();self.trajectories();self.physical_transcription();self.roads_and_rendezvous();self.resources()
        from Agent.Publication.engineering_joint_safety import apply_witnesses
        apply_witnesses(self,witnesses)
        # Priorities share this exact variable registry. Only a certified
        # objective bound may be carried between passes; no decision fixing.
        self.objectives=[('incomplete_components',1-self.z,0.),
                         ('fleet_makespan_seconds',self.C,0.),
                         ('aircraft_energy_Wh',self.total_energy,0.)]
        self.m.setObjective(self.objectives[0][1])
        self.timings['construction_s']=time.perf_counter()-begin
        return self

    def values(self,sol):return {name:float(self.m.getSolVal(sol,v)) for name,v in self.vars.items()}

    def extract(self,sol):
        value=lambda x:float(self.m.getSolVal(sol,x));flights=[]
        for l in range(self.n):
            if value(self.b[l])<.5:continue
            order=[l]
            while value(self.last[order[-1]])<.5:
                choices=[j for j in range(self.n) if j!=order[-1] and value(self.h[order[-1],j])>.5]
                if len(choices)!=1 or choices[0] in order:raise ValueError('invalid selected path')
                order.append(choices[0])
            timeline=[];captures=[];clock=0.
            for i in order:
                for key in [f'in_{i}_{k}' for k in range(3)]+[f'cap_{i}_0']:
                    b=self.segments[key];T=value(b['T']);C=np.array([[value(x) for x in row] for row in b['C']])+self.origin;A=[[value(x) for x in row] for row in b['A']]
                    s=dict(model_segment_key=key,kind='capture' if b['role']=='cap' else 'transfer',start_s=clock,end_s=clock+T,control_points_m=C.tolist(),pose_controls_yaw_gp_gy_focal_ratio=A,from_xyz_m=C[0].tolist(),to_xyz_m=C[-1].tolist(),departure_contact=i==l and b['role']=='in' and b['segment']==0,arrival_contact=False,energy_Wh=value(self.energy[key]))
                    if b['role']=='cap':
                        s['task_id']=self.targets[i]['id'];captures.append(dict(task_id=s['task_id'],task_index=i,start_s=clock,end_s=clock+T,position_xyz_m=(basis(7,.5)@C).tolist(),moving_capture=True))
                    timeline.append(s);clock+=T
            for k in range(3):
                key=f'rec_{order[-1]}_{k}';b=self.segments[key];T=value(b['T']);C=np.array([[value(x) for x in row] for row in b['C']])+self.origin;A=[[value(x) for x in row] for row in b['A']]
                timeline.append(dict(model_segment_key=key,kind='transfer',start_s=clock,end_s=clock+T,control_points_m=C.tolist(),pose_controls_yaw_gp_gy_focal_ratio=A,from_xyz_m=C[0].tolist(),to_xyz_m=C[-1].tolist(),departure_contact=False,arrival_contact=k==2,energy_Wh=value(self.energy[key])));clock+=T
            aircraft=next(u for u in range(4) if value(self.y[u,l])>.5);pack=next(p for p in range(8) if value(self.beta[p,l])>.5)
            flights.append(dict(root=l,task_id='inspection_unified_execution',status='unified_transcription_candidate_requires_independent_replay',task_count=len(order),sequence=[self.targets[i]['id'] for i in order],task_indices=order,flight_seconds=clock,aircraft_energy_Wh=value(self.flight_energy[l]),energy_reserve_Wh=109.6,timeline=timeline,captures=captures,targets=self.targets,source_sortie=dict(slot=l,vehicle=aircraft//2,aircraft=aircraft,battery_pair=pack,launch_s=value(self.s[l]),land_s=value(self.e[l])),optimized_launch_s=value(self.s[l]),optimized_recovery_end_s=value(self.e[l])+240,initial_SOC=self.soc,complete_joint_solver=False))
        from Agent.Publication.engineering_event_charging import extract_event_charging
        return dict(charging_plan=extract_event_charging(self,value),makespan_s=value(self.C),aircraft_energy_Wh=value(self.total_energy),complete_component=value(self.z),flights=flights,route_selection=[next(r for r in range(len(options)) if value(self.route[v,r])>.5) for v,options in enumerate(self.road_options)],scope='single finite joint MINLP; not independently accepted or full continuous-domain optimal')


    def graph(self):
        incidence=[];degree=collections.Counter();family=collections.Counter();unknown=[]
        for c,e,lo,hi,f in self.rows:
            try:names=sorted(set(v.name for v in self.m.getConsVars(c)))
            except Exception as exc:unknown.append(dict(name=c.name,error=str(exc)));continue
            incidence.append(dict(name=c.name,variables=names,lower=lo,upper=hi,family=f));degree.update(names);family[f]+=1
        return dict(representation='original scalar SCIP model, including explicitly labelled auxiliaries; before presolve',n=len(self.vars),m=len(incidence),N_I=sum(len(x['variables']) for x in incidence),r_max=max([len(x['variables']) for x in incidence] or [0]),degree_max=max(degree.values(),default=0),groups=dict(collections.Counter(self.groups.values())),families=dict(family),unknown_rows=unknown,edges=incidence)
