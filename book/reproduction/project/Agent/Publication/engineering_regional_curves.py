"""Free regional Bernstein trajectories linked to the compact task graph.

Position degree7 is represented exactly by endpoint (p,v,a,j); angle degree7
by endpoint (angle,rate,acceleration) and its two remaining free controls.
Shared endpoint objects impose C3/C2 joins without duplicate control variables.
This is an exact reparameterization of each finite curve, not a flight library.
Physics/optics/safety must still be attached before an optimization entry opens.
"""
from dataclasses import dataclass
from types import SimpleNamespace
import math
import numpy as np
from pyscipopt import quicksum
from Agent.Publication.engineering_indexed_constraints import AffineRow,Selector


def control_points(left,right,T,middle=None):
    """Endpoint derivatives are in physical seconds, not normalized time."""
    dim=len(left[0]);result=[]
    for q in range(dim):
        c=[None]*8
        c[0]=left[0][q];c[1]=c[0]+T*left[1][q]/7
        c[2]=2*c[1]-c[0]+T*T*left[2][q]/42
        c[7]=right[0][q];c[6]=c[7]-T*right[1][q]/7
        c[5]=2*c[6]-c[7]+T*T*right[2][q]/42
        if middle is None:
            c[3]=3*c[2]-3*c[1]+c[0]+T**3*left[3][q]/210
            c[4]=3*c[5]-3*c[6]+c[7]-T**3*right[3][q]/210
        else:c[3]=middle[0][q];c[4]=middle[1][q]
        result.append(c)
    return [[result[q][k] for q in range(dim)] for k in range(8)]


@dataclass
class CurveNode:
    position: list
    pose: list


@dataclass
class CurveSegment:
    task: int
    role: str
    index: int
    active: object
    duration: object
    clock: object
    left: CurveNode
    right: CurveNode
    middle_pose_controls: list
    safe_duration: object = None

    def controls(self):
        return dict(C=control_points(self.left.position,self.right.position,self.duration),
                    A=control_points(self.left.pose,self.right.pose,self.duration,self.middle_pose_controls))

    def evaluate(self,s,field='C',derivatives=3):
        # Hermite endpoint derivatives are the original decision states.
        # Expanding and then cancelling T^r/T^r at contacts creates spurious
        # reciprocal singularities in interval propagation. At a zero-length
        # copied piece these are its continuation states, with zero time/energy.
        if isinstance(s,(int,float,np.number)) and s in (0.,1.):
            node=self.left if s==0 else self.right
            endpoint=node.position if field=='C' else node.pose
            if derivatives<len(endpoint):return endpoint[:derivatives+1]
        cp=self.controls()[field]
        # A disabled zero-duration segment is a repeated state. It contributes
        # no time/energy. The safe denominator is exactly T when active.
        T=self.safe_duration if self.safe_duration is not None else self.duration+1-self.active
        out=[]
        for r in range(derivatives+1):
            n=len(cp)-1
            out.append([quicksum(math.comb(n,k)*s**k*(1-s)**(n-k)*row[q] for k,row in enumerate(cp))/(T**r if r else 1) for q in range(len(cp[0]))])
            cp=[[n*(cp[k+1][q]-cp[k][q]) for q in range(len(cp[0]))] for k in range(n)]
        return out


class RegionalCurves:
    def __init__(self,path,scene,record,terrain,transfers=3,minimum_positive_duration=.001,shared_recovery=False):
        self.path=path;self.scene=scene;self.record=record;self.m=path.m;self.rel=path.rel
        self.transfers=transfers;self.min_duration=minimum_positive_duration
        self.shared_recovery=shared_recovery;self.recoveries=[];self.return_owners=[]
        if transfers<1 or minimum_positive_duration<=0:raise ValueError('Explicit positive numerical resolution required')
        self.origin=np.asarray(scene['fleet']['vehicles'][0]['initial_xyz_m'],float)
        self.lower=np.r_[np.asarray(terrain.tri.points).min(axis=0),np.asarray(terrain.values).min()]-self.origin
        self.upper=np.r_[np.asarray(terrain.tri.points).max(axis=0),np.asarray(terrain.values).max()+100]-self.origin
        self.nodes=[];self.segments=[];self.by_task=[];self.physics_attached=False;self.anchors_attached=False
        self.corrected_targets=[]
        import copy
        for source,correct in zip(scene['inspection_tasks'],record['corrections']):
            if source['id']!=correct['task_id']:raise ValueError('Correction/source order mismatch')
            task=copy.deepcopy(source)
            for q,key in enumerate(['easting_m','northing_m','altitude_m']):
                task[key]=correct['viewpoint_center_xyz_m'][q];task['target'][key]=correct['target_xyz_m'][q]
            task['target']['surface_samples_utm_m']=correct['surface_samples_utm_m']
            task['longitude']=correct['viewpoint_longitude'];task['latitude']=correct['viewpoint_latitude']
            task['target']['longitude']=correct['target_longitude'];task['target']['latitude']=correct['target_latitude']
            self.corrected_targets.append(task)
        yaw=2*math.pi+math.radians(30)*path.H
        self.pose_lo=[-yaw,math.radians(-120),math.radians(-320),1]
        self.pose_hi=[yaw,math.radians(30),math.radians(320),119.94/6.83]
        self.pose_rate=[math.radians(30)]*3+[10/6.83]
        # A valid finite bound for the unconstrained zoom second derivative of
        # a degree7 polynomial over Tmin; not a new hardware acceleration cap.
        # Markov bound for the degree6 rate polynomial: |f''| <=
        # 2*6^2*max|f'|/T. This is implied by the existing continuous
        # zoom-rate limit, not a newly imposed hardware acceleration limit.
        self.pose_acc=[math.radians(30)]*3+[72*self.pose_rate[3]/self.min_duration]
        B=np.array([[math.comb(7,k)*s**k*(1-s)**(7-k) for k in range(8)] for s in np.linspace(0,1,8)])
        self.inverse_norm=np.sum(abs(np.linalg.inv(B)),axis=1)

    def node(self,name):
        pos=[];pose=[]
        for r in range(4):
            lo,hi=(self.lower,self.upper) if r==0 else ([-[0,5,3,5][r]]*3,[[0,5,3,5][r]]*3)
            pos.append([self.m.addVar(name+f'_p_d{r}_{q}',lb=float(lo[q]),ub=float(hi[q])) for q in range(3)])
        for r in range(3):
            lo,hi=(self.pose_lo,self.pose_hi) if r==0 else (
                [-v for v in (self.pose_rate if r==1 else self.pose_acc)],self.pose_rate if r==1 else self.pose_acc)
            pose.append([self.m.addVar(name+f'_a_d{r}_{q}',lb=float(lo[q]),ub=float(hi[q])) for q in range(4)])
        node=CurveNode(pos,pose);self.nodes.append(node);return node

    @staticmethod
    def continuity_rows(a,b):
        return [AffineRow(f'{name}_d{r}_{q}',[(1,x),(-1,y)],0,0)
                for name,left,right in [('position',a.position,b.position),('pose',a.pose,b.pose)]
                for r,(l,rgt) in enumerate(zip(left,right)) for q,(x,y) in enumerate(zip(l,rgt))]

    def build(self):
        m=self.m;n=self.transfers;count=n+1 if self.shared_recovery else 2*n+1
        for i,task in enumerate(self.path.tasks):
            nodes=[self.node(f'task{i+1}_curve_node{k}') for k in range(count+1)]
            segments=[]
            for k in range(count):
                role='in' if k<n else ('capture' if k==n else 'recovery')
                flag=task.active if k<=n else task.last
                enabled=m.addVar(f'task{i+1}_segment{k}_enabled',vtype='B')
                m.addCons(enabled<=flag)
                if role=='capture':m.addCons(enabled==task.active)
                T=m.addVar(f'task{i+1}_segment{k}_duration',lb=0,ub=self.path.H)
                m.addCons(T>=self.min_duration*enabled);m.addCons(T<=self.path.H*enabled)
                middle=[]
                for j in [3,4]:
                    mid=(np.asarray(self.pose_lo)+self.pose_hi)/2;rad=(np.asarray(self.pose_hi)-self.pose_lo)/2
                    middle.append([m.addVar(f'task{i+1}_segment{k}_pose_control{j}_{q}',
                        lb=float(mid[q]-self.inverse_norm[j]*rad[q]),ub=float(mid[q]+self.inverse_norm[j]*rad[q])) for q in range(4)])
                # A zero-duration piece on an existing flight is an exact
                # copied state. An entirely unselected task has no physical
                # curve. Guarded indexed equalities avoid hundreds of
                # thousands of redundant inactive big-M rows without losing
                # any selected trajectory or allowing a selected-state jump.
                alias=self.rel.binary_selector(f'task{i+1}_segment{k}_copy_index',enabled)
                def copy_rows(category,left=nodes[k],right=nodes[k+1]):
                    return [] if category else self.continuity_rows(left,right)
                self.rel.add(alias,copy_rows,[],flag,f'task{i+1}_segment{k}_disabled_copy')
                if role=='in':clock=task.time-task.incoming+quicksum(s.duration for s in segments)
                elif role=='capture':clock=task.time
                else:clock=task.time+task.dwell+quicksum(s.duration for s in segments if s.role=='recovery')
                segment=CurveSegment(i,role,k,enabled,T,clock,nodes[k],nodes[k+1],middle)
                segment.safe_duration=m.addVar(f'task{i+1}_segment{k}_safe_duration',lb=min(1.,self.min_duration),ub=self.path.H)
                m.addCons(segment.safe_duration==T+1-enabled)
                segments.append(segment);self.segments.append(segment)
            m.addCons(task.incoming==quicksum(s.duration for s in segments if s.role=='in'))
            m.addCons(task.dwell==segments[n].duration)
            if not self.shared_recovery:
                m.addCons(task.recovery==quicksum(s.duration for s in segments if s.role=='recovery'))
            self.by_task.append(dict(nodes=nodes,segments=segments,in_start=nodes[0],capture_start=nodes[n],
                                     capture_end=nodes[n+1],recovery_end=nodes[-1]))
        self.rel.lock(m.getVars())
        for i,task in enumerate(self.path.tasks):
            def rows(k,i=i):
                if not k:return [AffineRow('nonzero_predecessor',[],lo=1)]
                return self.continuity_rows(self.by_task[i]['in_start'],self.by_task[k-1]['capture_end'])
            self.rel.add(task.pred,rows,[],task.has_pred,f'task{i+1}_free_trajectory_predecessor')
        if self.shared_recovery:self.build_shared_recovery()
        return self

    def build_shared_recovery(self):
        """Same finite return curve, indexed by flight rather than last task.

        Forward map: copy the selected last task's recovery states. Inverse:
        copy the flight return into that last task; inactive task returns can
        be arbitrary bounded states. No active polynomial is removed.
        """
        m=self.m;n=self.transfers;N=len(self.path.tasks)
        for k,f in enumerate(self.path.flights):
            owner=N+k
            nodes=[self.node(f'flight{k+1}_recovery_node{j}') for j in range(n+1)]
            durations=[m.addVar(f'flight{k+1}_recovery_segment{n+1+j}_duration',lb=0,ub=self.path.H) for j in range(n)]
            segments=[]
            for j,T in enumerate(durations):
                index=n+1+j;prefix=f'flight{k+1}_recovery_segment{index}'
                on=m.addVar(prefix+'_enabled',vtype='B')
                m.addCons(on<=f.active);m.addCons(T>=self.min_duration*on);m.addCons(T<=self.path.H*on)
                middle=[]
                for h in [3,4]:
                    mid=(np.asarray(self.pose_lo)+self.pose_hi)/2;rad=(np.asarray(self.pose_hi)-self.pose_lo)/2
                    middle.append([m.addVar(prefix+f'_pose_control{h}_{q}',lb=float(mid[q]-self.inverse_norm[h]*rad[q]),ub=float(mid[q]+self.inverse_norm[h]*rad[q])) for q in range(4)])
                alias=self.rel.binary_selector(prefix+'_copy_index',on)
                def copy_rows(category,left=nodes[j],right=nodes[j+1]):
                    return [] if category else self.continuity_rows(left,right)
                self.rel.add(alias,copy_rows,[],f.active,prefix+'_disabled_copy')
                clock=f.end-quicksum(durations[j:])
                segment=CurveSegment(owner,'recovery',index,on,T,clock,nodes[j],nodes[j+1],middle)
                segment.safe_duration=m.addVar(prefix+'_safe_duration',lb=min(1.,self.min_duration),ub=self.path.H)
                m.addCons(segment.safe_duration==T+1-on)
                segments.append(segment);self.segments.append(segment)
            # A fixed identity enables the same indexed contact/physics code;
            # it is not a fixed route, task assignment or vehicle stop.
            identity=m.addVar(f'flight{k+1}_recovery_identity',vtype='I',lb=k+1,ub=k+1)
            selector=Selector(f'flight{k+1}_recovery_identity',len(self.path.flights)+1,[],identity)
            self.rel.selectors.append(selector);self.rel.lock([identity])
            self.return_owners.append(SimpleNamespace(flight=selector,root=0,last=f.active))
            self.recoveries.append(dict(nodes=nodes,segments=segments))
            def start_rows(i,k=k,nodes=nodes):
                if not i:return [AffineRow('nonzero_last_task',[],lo=1)]
                return self.continuity_rows(nodes[0],self.by_task[i-1]['capture_end'])
            self.rel.add(f.last,start_rows,[],f.active,f'flight{k+1}_return_start')
        for i,t in enumerate(self.path.tasks):
            def duration_rows(k,t=t):
                if not k:return [AffineRow('nonzero_return_flight',[],lo=1)]
                return [AffineRow('return_duration',[(1,t.recovery)]+[(-1,s.duration) for s in self.recoveries[k-1]['segments']],0,0)]
            self.rel.add(t.flight,duration_rows,[],t.last,f'task{i+1}_return_duration')
        self.rel.lock(m.getVars())

    def segment(self,task,index):
        if task<len(self.by_task):return self.by_task[task]['segments'][index]
        return self.recoveries[task-len(self.by_task)]['segments'][index-self.transfers-1]

    def owner(self,segment):
        if segment.task<len(self.path.tasks):return self.path.tasks[segment.task]
        return self.return_owners[segment.task-len(self.path.tasks)]

    def status(self):
        return dict(tasks=len(self.by_task),segments=len(self.segments),state_nodes=len(self.nodes),
            position_degree=7,pose_degree=7,transfer_subsegments=self.transfers,
            minimum_positive_duration_s=self.min_duration,zero_duration_piece_supported=True,
            task_order_fixed=False,flight_geometry_fixed=False,position_continuity_order=3,
            pose_continuity_order=2,predecessor_linked=True,physical_constraints_attached=self.physics_attached,
            vehicle_anchors_attached=self.anchors_attached,full_joint_model=False)
