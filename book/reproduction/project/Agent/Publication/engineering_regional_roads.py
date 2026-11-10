"""Source-complete directed road modes and a continuous vehicle transcription.

Every source edge (including parallel IDs), junction and configured stop is an
available decision. Phase count is a declared numerical approximation, never a
physical visit limit. Polynomial dynamics are collocated and require separate
continuous/footprint acceptance before this block is a certified route.
"""
from dataclasses import dataclass
from collections import deque
import math
import numpy as np
from pyscipopt import sin, cos, quicksum
from Agent.Publication.engineering_indexed_constraints import AffineRow
from Agent.Publication.engineering_regional_charging import ParkedInterval


class RegionalRoadDomain:
    def __init__(self,scene,origin_xy):
        self.scene=scene;self.origin=np.asarray(origin_xy,float)
        self.nodes=scene['roads']['nodes'];self.edges=scene['roads']['edges'];self.stops=scene['stops']
        if scene['roads'].get('turn_restrictions') or scene['roads'].get('unsupported_turn_relations'):
            raise ValueError('Nonempty turn relations require explicit incoming-edge state; do not silently ignore')
        self.xy={n['id']:np.array([n['easting_m'],n['northing_m']])-self.origin for n in self.nodes}
        self.modes=[];self.junction={};self.stop={};self.arc={};self.outgoing={n['id']:[] for n in self.nodes}
        adjacent_speeds={n['id']:[] for n in self.nodes}
        for edge in self.edges:
            a=self.xy[edge['tail']];b=self.xy[edge['head']];length=float(np.linalg.norm(b-a))
            if length<=0:raise ValueError('Zero-length directed road edge')
            ell=min(3.,length/4)
            idx=len(self.modes);self.arc[edge['id']]=idx;self.outgoing[edge['tail']].append(idx)
            self.modes.append(dict(kind='edge',id=edge['id'],tail=edge['tail'],head=edge['head'],
                a=a,b=b,direction=(b-a)/length,length=length,ell=ell,
                speed=min(float(edge['speed_limit_mps']),float(scene['joint_reference_profile']['vehicle']['speed_max_mps'])),source_length=float(edge['length_m'])))
            adjacent_speeds[edge['tail']].append(self.modes[-1]['speed'])
            adjacent_speeds[edge['head']].append(self.modes[-1]['speed'])
        for n in self.nodes:
            self.junction[n['id']]=len(self.modes)
            adjacent=adjacent_speeds[n['id']]
            self.modes.append(dict(kind='junction',id=n['id'],center=self.xy[n['id']],
                speed=min(adjacent,default=float(scene['joint_reference_profile']['vehicle']['speed_max_mps']))))
        for s in self.stops:
            self.stop[s['id']]=len(self.modes)
            self.modes.append(dict(kind='stop',id=s['id'],node=s['osm_node_id'],
                center=np.asarray(s['xy_m'])-self.origin))
        self.next=[]
        for i,mode in enumerate(self.modes):
            choices=[i]  # optional numerical subdivision, not a graph jump
            if mode['kind']=='edge':choices.append(self.junction[mode['head']])
            elif mode['kind']=='stop':choices.append(self.junction[mode['node']])
            else:
                choices+=self.outgoing[mode['id']]
                choices += [self.stop[s['id']] for s in self.stops if s['osm_node_id']==mode['id']]
            self.next.append(choices)
        self.max_options=max(map(len,self.next))

    def transition(self,a,b):return b in self.next[a]

    def roundtrip_distances(self,vehicle):
        reverse=[[] for _ in self.modes]
        for a,bs in enumerate(self.next):
            for b in bs:reverse[b].append(a)
        def bfs(graph,start):
            d=np.full(len(graph),len(graph)+1,dtype=np.int64);d[start]=0;q=deque([start])
            while q:
                a=q.popleft()
                for b in graph[a]:
                    if d[b]>d[a]+1:d[b]=d[a]+1;q.append(b)
            return d
        return bfs(self.next,self.stop[vehicle['depot_stop']]),bfs(reverse,self.stop[vehicle['terminal_depot_stop']])

    def minimum_all_stop_phases(self,vehicle):
        forward,backward=self.roundtrip_distances(vehicle)
        counts={sid:int(forward[k]+backward[k]+1) for sid,k in self.stop.items()}
        if any(n>2*len(self.modes) for n in counts.values()):raise ValueError('A source stop has no directed depot roundtrip')
        return max(counts.values()),counts

    def add_transition(self,model,relations,left,right,position,name):
        """Binary-indexed *all* legal mode changes and exact core portals."""
        option=relations.selector(name+'_option',self.max_options)
        pair=relations.selector(name+'_pair',len(self.modes)*self.max_options)
        model.addCons(pair.value==left.value*self.max_options+option.value)
        def rows(category):
            a,k=divmod(category,self.max_options)
            if k>=len(self.next[a]):return [AffineRow('invalid_transition_option',[],lo=1)]
            b=self.next[a][k];ans=[AffineRow('next_mode',[(1,right.value)],b,b)]
            if a!=b:
                edge=self.modes[a] if self.modes[a]['kind']=='edge' else (
                     self.modes[b] if self.modes[b]['kind']=='edge' else None)
                if edge is not None:
                    progress=edge['length']-edge['ell'] if self.modes[a]['kind']=='edge' else edge['ell']
                    offset=float(edge['direction']@edge['a'])+progress
                    ans.append(AffineRow('shared_edge_portal',[(float(edge['direction'][q]),position[q]) for q in range(2)],offset,offset))
            return ans
        relations.add(pair,rows,[left.value,right.value,*position],name=name)
        return pair


def bernstein(n,s):return [math.comb(n,k)*s**k*(1-s)**(n-k) for k in range(n+1)]


@dataclass
class RoadPhase:
    carrier: int
    index: int
    mode: object
    start: object
    end: object
    duration: object
    controls: list
    parked: object
    edge: object
    junction: object
    geometry: dict


class RegionalVehicles:
    def __init__(self,path,scene,phase_counts,degree=7,deferred_dynamics=False):
        self.path=path;self.scene=scene;self.m=path.m;self.rel=path.rel
        self.origin=np.asarray(scene['fleet']['vehicles'][0]['initial_xyz_m'][:2])
        self.domain=RegionalRoadDomain(scene,self.origin)
        self.phase_counts=phase_counts;self.degree=degree;self.phases=[];self.parked=[];self.extra_points=set()
        self.point_aux={};self.reachability=[]
        self.deferred_dynamics=deferred_dynamics
        if len(phase_counts)!=len(scene['fleet']['vehicles']) or any(n<1 for n in phase_counts):
            raise ValueError('Positive explicit numerical phase count for each vehicle required')

    def build(self):
        m,rel,H,n=self.m,self.rel,self.path.H,self.degree
        cfg=self.scene['joint_reference_profile']['vehicle'];dom=self.domain
        points=np.array(list(dom.xy.values()));lo=points.min(axis=0)-3;hi=points.max(axis=0)+3
        inv=np.linalg.inv(np.array([bernstein(n,s) for s in np.linspace(0,1,n+1)]))
        norms=np.sum(abs(inv),axis=1)
        yawbound=2*math.pi+cfg['speed_max_mps']*math.tan(math.radians(cfg['steering_max_deg']))/cfg['wheelbase_m']*H
        lower=np.array([*lo,-yawbound,0.,-math.radians(cfg['steering_max_deg'])])
        upper=np.array([*hi,yawbound,cfg['speed_max_mps'],math.radians(cfg['steering_max_deg'])])
        self.state_lower=lower;self.state_upper=upper
        mid=(lower+upper)/2;rad=(upper-lower)/2
        for carrier,count in enumerate(self.phase_counts):
            phases=[];vehicle=self.scene['fleet']['vehicles'][carrier]
            forward,backward=dom.roundtrip_distances(vehicle)
            admissible_pairs=0
            for k in range(count):
                key=f'vehicle{carrier}_phase{k}'
                mode=rel.selector(key+'_mode',len(dom.modes))
                eligible=np.flatnonzero((forward<=k)&(backward<=count-1-k))
                if not len(eligible):raise ValueError('Road phase grid has no depot roundtrip')
                admissible_pairs+=len(eligible)
                m.chgVarLb(mode.value,int(eligible[0]));m.chgVarUb(mode.value,int(eligible[-1]))
                common_one=int(np.bitwise_and.reduce(eligible));possible_one=int(np.bitwise_or.reduce(eligible))
                for bit,v in enumerate(mode.bits):
                    if common_one&(1<<bit):m.chgVarLb(v,1)
                    if not possible_one&(1<<bit):m.chgVarUb(v,0)
                start=m.addVar(key+'_start',lb=0,ub=H);end=m.addVar(key+'_end',lb=0,ub=H)
                T=m.addVar(key+'_duration',lb=0,ub=H);m.addCons(end==start+T)
                cp=[[m.addVar(key+f'_control{i}_{q}',lb=float(mid[q]-norms[i]*rad[q]),ub=float(mid[q]+norms[i]*rad[q])) for q in range(5)] for i in range(n+1)]
                parked,edge,junction=[m.addVar(key+'_'+s,vtype='B') for s in ['parked','edge','junction']]
                m.addCons(parked+edge+junction==1)
                geom={q:m.addVar(key+'_'+q,lb=lb,ub=ub) for q,lb,ub in [
                    ('ax',float(lo[0]),float(hi[0])),('ay',float(lo[1]),float(hi[1])),
                    ('dx',-1,1),('dy',-1,1),('ell',0,3),('length',0,float(np.linalg.norm(hi-lo))),
                    ('speed',0,cfg['speed_max_mps'])]}
                phase=RoadPhase(carrier,k,mode,start,end,T,cp,parked,edge,junction,geom)
                phases.append(phase)
                def mode_rows(category,phase=phase,forward=forward,backward=backward,count=count):
                    if forward[category]>phase.index or backward[category]>count-1-phase.index:
                        return [AffineRow('outside_necessary_roundtrip_domain',[],lo=1)]
                    z=dom.modes[category];is_edge=z['kind']=='edge';is_stop=z['kind']=='stop'
                    a=z['a'] if is_edge else z['center'];d=z['direction'] if is_edge else [0,0]
                    vals=[*a,*d,z['ell'] if is_edge else 0,z['length'] if is_edge else 0,
                          0 if is_stop else z['speed']]
                    ans=[AffineRow(q,[(1,phase.geometry[q])],float(v),float(v)) for q,v in zip(geom,vals)]
                    ans += [AffineRow('edge_flag',[(1,phase.edge)],int(is_edge),int(is_edge)),
                            AffineRow('stop_flag',[(1,phase.parked)],int(is_stop),int(is_stop)),
                            AffineRow('junction_flag',[(1,phase.junction)],int(not(is_edge or is_stop)),int(not(is_edge or is_stop)))]
                    return ans
                rel.add(mode,mode_rows,[parked,edge,junction,*geom.values()],name=key+'_source_geometry')
                # n+1 points ensure the zero-duration derivative polynomial
                # is identically zero. No hidden positive minimum travel time.
                for j,s in enumerate([] if self.deferred_dynamics else np.linspace(0,1,n+1)):
                    state,ds=self.shared_point(phase,float(s))
                    x,y,theta,v,steer=state
                    for q in range(5):m.addCons(state[q]>=float(lower[q]));m.addCons(state[q]<=float(upper[q]))
                    m.addCons(ds[0]==T*v*cos(theta),name=key+f'_ack_x{j}')
                    m.addCons(ds[1]==T*v*sin(theta),name=key+f'_ack_y{j}')
                    m.addCons(ds[2]*cos(steer)==T*v*sin(steer)/cfg['wheelbase_m'],name=key+f'_ack_yaw{j}')
                    m.addCons(ds[3]<=T*cfg['acceleration_max_mps2']);m.addCons(ds[3]>=-T*cfg['acceleration_max_mps2'])
                    rate=math.radians(cfg['steering_rate_max_deg_s'])
                    m.addCons(ds[4]<=T*rate);m.addCons(ds[4]>=-T*rate)
                    m.addCons(v<=geom['speed'])
                    m.addCons(v*v*sin(steer)<=cfg['lateral_acceleration_max_mps2']*cfg['wheelbase_m']*cos(steer))
                    m.addCons(v*v*sin(steer)>=-cfg['lateral_acceleration_max_mps2']*cfg['wheelbase_m']*cos(steer))
                    progress=(x-geom['ax'])*geom['dx']+(y-geom['ay'])*geom['dy']
                    lateral=-(x-geom['ax'])*geom['dy']+(y-geom['ay'])*geom['dx']
                    m.addCons(edge*(progress-geom['ell'])>=0);m.addCons(edge*(geom['length']-geom['ell']-progress)>=0)
                    m.addCons(edge*(9-lateral*lateral)>=0)
                    m.addCons(edge*(ds[0]*geom['dx']+ds[1]*geom['dy'])>=0)
                    m.addCons(edge*(cos(theta)*geom['dx']+sin(theta)*geom['dy'])>=0)
                    m.addCons(parked*(9-(x-geom['ax'])**2-(y-geom['ay'])**2)>=0)
                    # Full junction/rectangle union containment is deliberately
                    # not replaced by center/corner sampling here.
                if k:
                    prev=phases[-2];m.addCons(start==prev.end)
                    for q in range(5):m.addCons(cp[0][q]==prev.controls[-1][q])
                    dom.add_transition(m,rel,prev.mode,mode,cp[0][:2],key+'_transition')
                else:
                    m.addCons(start==0);m.addCons(mode.value==dom.stop[vehicle['depot_stop']])
                    initial=[vehicle['initial_xyz_m'][0]-self.origin[0],vehicle['initial_xyz_m'][1]-self.origin[1],
                             vehicle['initial_heading_rad'],vehicle['initial_speed_mps']]
                    for q,val in enumerate(initial):m.addCons(cp[0][q]==val)
                    m.addCons(cp[0][4]==0)
                self.parked.append(ParkedInterval(carrier,start,end,parked,phase))
            last=phases[-1];m.addCons(last.end==self.path.C)
            m.addCons(last.mode.value==dom.stop[vehicle['terminal_depot_stop']])
            m.addCons(last.controls[-1][3]==0)
            # Returning to the depot means the declared original position,
            # not merely choosing the same stop label at a distant location.
            for q in range(2):m.addCons(last.controls[-1][q]==vehicle['initial_xyz_m'][q]-self.origin[q])
            self.phases.append(phases)
            available=[sid for sid,i in dom.stop.items() if forward[i]+backward[i]+1<=count]
            self.reachability.append(dict(carrier=carrier,phase_count=count,available_stops=available,
                admissible_phase_mode_pairs=int(admissible_pairs),original_phase_mode_pairs=count*len(dom.modes),
                all_source_stops_topologically_available=len(available)==len(dom.stop),
                physical_route_feasibility_proved=False))
        rel.lock(m.getVars())
        return self

    def check_required_points(self,value,tolerance=1e-6):
        """Check EVERY original collocation condition before acceptance.

        Deferred construction changes storage only. Interval vehicle checks
        are still additionally required; these finite rows include the stop
        center and scalar state bounds that footprint checks alone do not.
        """
        cfg=self.scene['joint_reference_profile']['vehicle'];bad=[];checked=0
        for phases in self.phases:
            for p in phases:
                cp=np.array([[value(v) for v in row] for row in p.controls]);T=value(p.duration)
                mode=self.domain.modes[int(round(value(p.mode.value)))];is_edge=mode['kind']=='edge';is_stop=mode['kind']=='stop'
                for s in np.linspace(0,1,self.degree+1):
                    state=np.array(bernstein(self.degree,s))@cp
                    ds=np.array(bernstein(self.degree-1,s))@(self.degree*np.diff(cp,axis=0))
                    x,y,theta,v,steer=state;sn=math.sin(steer);cs=math.cos(steer)
                    equality=[ds[0]-T*v*math.cos(theta),ds[1]-T*v*math.sin(theta),ds[2]*cs-T*v*sn/cfg['wheelbase_m']]
                    margins=[*list(state-self.state_lower),*list(self.state_upper-state),
                        T*cfg['acceleration_max_mps2']-abs(ds[3]),T*math.radians(cfg['steering_rate_max_deg_s'])-abs(ds[4]),
                        mode.get('speed',0)-v,cfg['lateral_acceleration_max_mps2']*cfg['wheelbase_m']*cs-abs(v*v*sn)]
                    if is_edge:
                        delta=state[:2]-mode['a'];direction=mode['direction'];progress=delta@direction;lateral=delta@np.array([-direction[1],direction[0]])
                        margins += [progress-mode['ell'],mode['length']-mode['ell']-progress,9-lateral*lateral,
                            ds[:2]@direction,np.array([math.cos(theta),math.sin(theta)])@direction]
                    if is_stop:margins.append(9-float(np.sum((state[:2]-mode['center'])**2)))
                    checked+=1;error=max(max(abs(z) for z in equality),max(0.,-min(margins)))
                    if error>tolerance:bad.append(dict(kind='vehicle_physics',carrier=p.carrier,phase_index=p.index,s=float(s),original_point_residual=error))
        return dict(passed=not bad,points_checked=checked,requests=bad)

    def shared_point(self,phase,s):
        """Exact linear definitions shared by all nonlinear vehicle rows."""
        key=(phase.carrier,phase.index,float(s))
        if key in self.point_aux:return self.point_aux[key]
        n=self.degree;cp=phase.controls;B=bernstein(n,s);D=bernstein(n-1,s)
        state=[];derivative=[]
        for q in range(5):
            x=self.m.addVar(f'vehicle_point{len(self.point_aux)}_state{q}',
                lb=float(self.state_lower[q]),ub=float(self.state_upper[q]))
            self.m.addCons(x==quicksum(B[i]*cp[i][q] for i in range(n+1)))
            coeff=[n*((D[k-1] if k else 0)-(D[k] if k<n else 0)) for k in range(n+1)]
            lo=sum(min(c*cp[k][q].getLbOriginal(),c*cp[k][q].getUbOriginal()) for k,c in enumerate(coeff))
            hi=sum(max(c*cp[k][q].getLbOriginal(),c*cp[k][q].getUbOriginal()) for k,c in enumerate(coeff))
            dx=self.m.addVar(f'vehicle_point{len(self.point_aux)}_derivative{q}',lb=lo,ub=hi)
            self.m.addCons(dx==quicksum(c*cp[k][q] for k,c in enumerate(coeff)))
            state.append(x);derivative.append(dx)
        self.point_aux[key]=(state,derivative);return state,derivative

    def status(self):
        return dict(source_nodes=len(self.domain.nodes),source_directed_arcs=len(self.domain.edges),
            source_stops=len(self.domain.stops),all_source_modes_available=True,
            phase_counts=self.phase_counts,phase_count_is_physical_limit=False,
            collocated_ackermann=True,continuous_ackermann_verified=False,
            full_footprint_containment_attached=False,inter_vehicle_safety_attached=False,
            full_joint_model=False)

    def add_point(self,carrier,index,s):
        signature=(carrier,index,float(s))
        if signature in self.extra_points:return False
        phase=self.phases[carrier][index];m=self.m;cp=phase.controls;n=self.degree;T=phase.duration
        cfg=self.scene['joint_reference_profile']['vehicle'];key=f'vehicle_refine_{len(self.extra_points)}'
        state,ds=self.shared_point(phase,s)
        x,y,theta,v,steer=state;g=phase.geometry
        m.addCons(ds[0]==T*v*cos(theta),name=key+'_ack_x');m.addCons(ds[1]==T*v*sin(theta),name=key+'_ack_y')
        m.addCons(ds[2]*cos(steer)==T*v*sin(steer)/cfg['wheelbase_m'],name=key+'_ack_yaw')
        m.addCons(v>=0);m.addCons(v<=g['speed'])
        m.addCons(steer>=-math.radians(cfg['steering_max_deg']));m.addCons(steer<=math.radians(cfg['steering_max_deg']))
        for q,limit in [(3,cfg['acceleration_max_mps2']),(4,math.radians(cfg['steering_rate_max_deg_s']))]:
            m.addCons(ds[q]<=T*limit);m.addCons(ds[q]>=-T*limit)
        lateral=v*v*sin(steer);bound=cfg['lateral_acceleration_max_mps2']*cfg['wheelbase_m']*cos(steer)
        m.addCons(lateral<=bound);m.addCons(lateral>=-bound)
        progress=(x-g['ax'])*g['dx']+(y-g['ay'])*g['dy']
        lateral_offset=-(x-g['ax'])*g['dy']+(y-g['ay'])*g['dx']
        m.addCons(phase.edge*(progress-g['ell'])>=0);m.addCons(phase.edge*(g['length']-g['ell']-progress)>=0)
        m.addCons(phase.edge*(9-lateral_offset*lateral_offset)>=0)
        m.addCons(phase.parked*(9-(x-g['ax'])**2-(y-g['ay'])**2)>=0)
        m.addCons(phase.edge*(ds[0]*g['dx']+ds[1]*g['dy'])>=0)
        m.addCons(phase.edge*(cos(theta)*g['dx']+sin(theta)*g['dy'])>=0)
        self.extra_points.add(signature);return True
