"""Directed source-road routing and analytic Ackermann motion primitives.

No teleporting or zero-radius U-turns. Circular fillets and straight segments
are traversed from rest; steering changes take finite time at zero speed.
These are feasible motion primitives, not a globally optimal OCP certificate.
"""
import heapq,math
import numpy as np
from shapely.geometry import LineString,Polygon
from Agent.Publication.engineering_joint_execution import smooth

def angle(a):return (a+math.pi)%(2*math.pi)-math.pi
def rot90(v):return np.array([-v[1],v[0]])
def rate_basis(s):return 140*s**3-420*s**4+420*s**5-140*s**6
def accel_basis(s):return 420*s**2-1680*s**3+2100*s**4-840*s**5

class RoadMotion:
    def __init__(self,scene):
        self.scene=scene;self.cfg=scene['joint_reference_profile']['vehicle']
        self.nodes={n['id']:n for n in scene['roads']['nodes']};self.edges={(e['tail'],e['head']):e for e in scene['roads']['edges']}
        if len(self.edges)!=len(scene['roads']['edges']):
            raise ValueError('Parallel road arcs require an arc-identity adapter; do not silently merge')
        self.node_speed={}
        for (a,b),edge in self.edges.items():
            speed=float(edge['speed_limit_mps'])
            if not math.isfinite(speed) or speed<=0:raise ValueError('Road speed limit must be finite and positive')
            for node in (a,b):self.node_speed[node]=min(self.node_speed.get(node,math.inf),speed)
        self.adj={};self.stops={s['id']:s for s in scene['stops']};self.turn_cache={};self.cache={}
        self.R=self.cfg['wheelbase_m']/math.tan(math.radians(self.cfg['steering_max_deg']))+.01
        for a,b in self.edges:self.adj.setdefault(a,[]).append(b)
        if scene['roads']['turn_restrictions']:raise ValueError('Turn relation adapter required; do not silently ignore')
    def xy(self,n):return np.array([self.nodes[n]['easting_m'],self.nodes[n]['northing_m']],dtype=float)
    def bearing(self,a,b):
        p=self.xy(b)-self.xy(a);return math.atan2(p[1],p[0])
    def turn(self,a,b,c):
        key=(a,b,c)
        if key in self.turn_cache:return self.turn_cache[key]
        if a==c:answer=None
        else:
            p,q,r=self.xy(a),self.xy(b),self.xy(c);u=q-p;v=r-q;l1=np.linalg.norm(u);l2=np.linalg.norm(v);u/=l1;v/=l2
            theta=angle(math.atan2(v[1],v[0])-math.atan2(u[1],u[0]));cut=self.R*math.tan(abs(theta)/2)
            if abs(theta)<1e-7:answer=dict(theta=0.,cut=0.)
            elif cut>.45*min(l1,l2) or abs(theta)>math.radians(175):answer=None
            else:
                ta=q-cut*u;tb=q+cut*v;center=ta+math.copysign(1,theta)*self.R*rot90(u)
                corridor=LineString([p,q,r]).buffer(self.cfg['road_corridor_width_m']/2,cap_style=1,join_style=1)
                start=math.atan2(ta[1]-center[1],ta[0]-center[0]);inside=True
                for t in np.linspace(0,1,max(9,int(abs(theta)*self.R/.1)+1)):
                    th=start+theta*t;pt=center+self.R*np.array([math.cos(th),math.sin(th)])
                    yaw=self.bearing(a,b)+theta*t
                    # Continuous inclusion from a Lipschitz displacement bound
                    # for the corners between adjacent arc samples.
                    h=abs(theta)/(max(9,int(abs(theta)*self.R/.1)+1)-1)/2
                    margin=h*(self.R+math.hypot(self.cfg['length_m']/2,self.cfg['width_m']/2))+.001
                    if not corridor.covers(self.footprint(pt,yaw).buffer(margin)):inside=False;break
                answer=dict(theta=theta,cut=cut,center=center.tolist(),ta=ta.tolist(),tb=tb.tolist(),start_angle=start) if inside else None
        self.turn_cache[key]=answer;return answer
    def footprint(self,p,yaw):
        u=np.array([math.cos(yaw),math.sin(yaw)]);v=rot90(u);L=self.cfg['length_m']/2;W=self.cfg['width_m']/2
        return Polygon([p+L*u+W*v,p+L*u-W*v,p-L*u-W*v,p-L*u+W*v])
    def paths(self,source_stop,heading=None,incoming=None,targets=None):
        key=(source_stop,None if heading is None else round(float(heading),9),incoming,None if targets is None else tuple(sorted(targets)))
        if key in self.cache:return self.cache[key]
        start=self.stops[source_stop]['osm_node_id'];dist={(incoming,start):0.};previous={};heap=[(0.,incoming or '',start)];best={}
        destinations={s['osm_node_id'] for sid,s in self.stops.items() if targets is None or sid in targets}
        while heap:
            d,ap,b=heapq.heappop(heap);a=ap or None
            if d!=dist.get((a,b)):continue
            if b in destinations and b not in best:best[b]=(a,b)
            if targets is not None and len(best)==len(destinations):break
            for c in self.adj.get(b,[]):
                if a is None:
                    if heading is not None and abs(angle(self.bearing(b,c)-heading))>1e-4:continue
                elif self.turn(a,b,c) is None:continue
                edge=self.edges[b,c];nd=d+edge['seconds'];state=(b,c)
                if nd<dist.get(state,math.inf):dist[state]=nd;previous[state]=(a,b);heapq.heappush(heap,(nd,b,c))
        routes={}
        for sid,stop in self.stops.items():
            end=best.get(stop['osm_node_id'])
            if end is None:continue
            seq=[end[1]];state=end
            while state!=(incoming,start):state=previous[state];seq.append(state[1])
            seq.reverse();routes[sid]=seq
        self.cache[key]=routes;return routes
    def route(self,source_stop,target_stop,heading=None):
        nodes=self.paths(source_stop,heading).get(target_stop)
        if nodes is None:return None
        return self.motion(nodes,source_stop,target_stop,heading)
    def minimum_return_cycle(self,vehicle):
        """Dijkstra lower bound over the explicit forward fillet primitive graph.

        Counts fillet distance savings. Raw polyline length is NOT a lower
        bound on a smoothed trajectory. No global arbitrary-curve road claim.
        """
        start=self.stops[vehicle['depot_stop']]['osm_node_id'];dist={(None,start):0.};previous={};heap=[(0.,'',start)];visited=0
        while heap:
            distance,ap,b=heapq.heappop(heap);a=ap or None
            if distance!=dist.get((a,b)):continue
            visited+=1
            if b==start and a is not None:
                sequence=[b];state=(a,b)
                while state!=(None,start):state=previous[state];sequence.append(state[1])
                sequence.reverse()
                return dict(feasible=True,minimum_primitive_cycle_length_m=distance,driving_time_lower_bound_s=distance/self.cfg['speed_max_mps'],node_ids=sequence,
                    septic_primitive_driving_time_lower_bound_s=2.1875*distance/self.cfg['speed_max_mps'],
                    states_settled=visited,scope='all nonempty depot-return routes in declared directed straight/circular-fillet graph; no arbitrary-curvature global bound')
            for c in self.adj.get(b,[]):
                length=float(np.linalg.norm(self.xy(c)-self.xy(b)))
                if a is None:
                    if abs(angle(self.bearing(b,c)-vehicle['initial_heading_rad']))>1e-4:continue
                    weight=length
                else:
                    turn=self.turn(a,b,c)
                    if turn is None:continue
                    weight=length-2*turn['cut']+self.R*abs(turn['theta'])
                assert weight>=-1e-8
                nd=distance+weight;state=(b,c)
                if nd<dist.get(state,math.inf):dist[state]=nd;previous[state]=(a,b);heapq.heappush(heap,(nd,b,c))
        return dict(feasible=False,states_settled=visited,scope='declared directed fillet primitive graph only')
    def itinerary(self,vehicle,visits):
        """Preserve incoming road context; stop on a fillet within the pad region.

        A road vertex is not an instantaneous yaw reset. The complete route
        is smoothed before its visit events are separated into driving legs.
        """
        depot=vehicle['depot_stop'];order=[depot]+list(visits)+[depot];nodes=[self.stops[depot]['osm_node_id']];indices=[]
        for i,(source,target) in enumerate(zip(order,order[1:])):
            incoming=nodes[-2] if len(nodes)>1 else None
            path=self.paths(source,vehicle['initial_heading_rad'] if incoming is None else None,incoming,[target]).get(target)
            if path is None:return None
            nodes.extend(path[1:]);indices.append((len(nodes)-1,target))
        route=self.motion(nodes,depot,depot,vehicle['initial_heading_rad'],indices)
        route['visits']=list(visits);return route
    def motion(self,nodes,source_stop,target_stop,heading=None,visit_indices=()):
        if len(nodes)==1:return dict(source_stop=source_stop,target_stop=target_stop,node_ids=nodes,edge_ids=[],segments=[],duration_s=0.,length_m=0.,
            initial_yaw_rad=float(heading) if heading is not None else 0.,terminal_yaw_rad=float(heading) if heading is not None else 0.,
            visit_events=[dict(stop_id=sid,after_segment_count=0,position_xy_m=self.xy(nodes[0]).tolist(),yaw_rad=float(heading or 0),route_time_s=0.) for ix,sid in visit_indices])
        primitives=[];events=[];current=self.xy(nodes[0]);steering=0.;visits={}
        pending_limit=min(self.edges[nodes[0],nodes[1]]['speed_limit_mps'],self.node_speed[nodes[0]])
        for ix,sid in visit_indices:visits.setdefault(ix,[]).append(sid)
        def visit(ix,point,yaw):
            for sid in visits.get(ix,[]):
                offset=float(np.linalg.norm(point-np.array(self.stops[sid]['xy_m'])))
                if offset>self.cfg['pad_region_radius_m']+1e-8:raise ValueError('Smoothed road stop outside permitted source pad region')
                events.append(dict(stop_id=sid,after_segment_count=len(primitives),position_xy_m=point.tolist(),yaw_rad=yaw,offset_from_source_stop_m=offset))
        def motion(kind,length,speed_limit,**kwargs):
            if length<1e-7:return
            vmax=min(self.cfg['speed_max_mps'],speed_limit)
            if kind=='arc':vmax=min(vmax,math.sqrt(self.cfg['lateral_acceleration_max_mps2']*self.R))
            T=max(2.1875*length/vmax,math.sqrt(7.513188405*length/self.cfg['acceleration_max_mps2']),.01)
            primitives.append(dict(kind=kind,length_m=float(length),duration_s=T,effective_speed_limit_mps=vmax,**kwargs))
        def steer(value,p,yaw):
            nonlocal steering
            if abs(value-steering)>1e-10:
                T=2.1875*abs(value-steering)/math.radians(self.cfg['steering_rate_max_deg_s'])
                primitives.append(dict(kind='steer',duration_s=T,position_xy_m=p.tolist(),yaw_rad=yaw,steering_start_rad=steering,steering_end_rad=value,length_m=0.))
            steering=value
        visit(0,current,self.bearing(nodes[0],nodes[1]))
        for i in range(1,len(nodes)-1):
            a,b,c=nodes[i-1:i+2];turn=self.turn(a,b,c);assert turn is not None
            yaw=self.bearing(a,b)
            # A merged collinear primitive uses the minimum of every road
            # and junction it traverses; it may be conservative but cannot
            # accelerate through a lower source speed limit.
            pending_limit=min(pending_limit,self.edges[a,b]['speed_limit_mps'],self.node_speed[b])
            if not turn['cut']:
                if i in visits:
                    point=self.xy(b);motion('straight',np.linalg.norm(point-current),pending_limit,from_xy_m=current.tolist(),to_xy_m=point.tolist(),yaw_rad=yaw,steering_rad=0.)
                    visit(i,point,yaw);current=point
                    pending_limit=self.node_speed[b]
                pending_limit=min(pending_limit,self.edges[b,c]['speed_limit_mps'])
                continue
            ta,tb=np.array(turn['ta']),np.array(turn['tb'])
            motion('straight',np.linalg.norm(ta-current),pending_limit,from_xy_m=current.tolist(),to_xy_m=ta.tolist(),yaw_rad=yaw,steering_rad=0.)
            delta=math.atan(self.cfg['wheelbase_m']/self.R)*math.copysign(1,turn['theta']);steer(delta,ta,yaw)
            if i in visits:
                half=turn['theta']/2;middle=np.array(turn['center'])+self.R*np.array([math.cos(turn['start_angle']+half),math.sin(turn['start_angle']+half)])
                motion('arc',self.R*abs(half),self.node_speed[b],center_xy_m=turn['center'],radius_m=self.R,start_angle_rad=turn['start_angle'],sweep_rad=half,yaw_start_rad=yaw,steering_rad=delta)
                visit(i,middle,yaw+half)
                motion('arc',self.R*abs(half),self.node_speed[b],center_xy_m=turn['center'],radius_m=self.R,start_angle_rad=turn['start_angle']+half,sweep_rad=half,yaw_start_rad=yaw+half,steering_rad=delta)
            else:
                motion('arc',self.R*abs(turn['theta']),self.node_speed[b],center_xy_m=turn['center'],radius_m=self.R,start_angle_rad=turn['start_angle'],
                    sweep_rad=turn['theta'],yaw_start_rad=yaw,steering_rad=delta)
            steer(0.,tb,self.bearing(b,c));current=tb
            pending_limit=min(self.edges[b,c]['speed_limit_mps'],self.node_speed[b])
        end=self.xy(nodes[-1]);yaw=self.bearing(nodes[-2],nodes[-1])
        pending_limit=min(pending_limit,self.node_speed[nodes[-1]])
        motion('straight',np.linalg.norm(end-current),pending_limit,from_xy_m=current.tolist(),to_xy_m=end.tolist(),yaw_rad=yaw,steering_rad=0.)
        visit(len(nodes)-1,end,yaw)
        time=0.
        for seg in primitives:seg.update(start_s=time,end_s=time+seg['duration_s']);time=seg['end_s']
        for e in events:e['route_time_s']=primitives[e['after_segment_count']-1]['end_s'] if e['after_segment_count'] else 0.
        return dict(source_stop=source_stop,target_stop=target_stop,node_ids=nodes,edge_ids=[self.edges[a,b]['id'] for a,b in zip(nodes,nodes[1:])],segments=primitives,
            duration_s=time,length_m=sum(s['length_m'] for s in primitives),initial_yaw_rad=self.bearing(nodes[0],nodes[1]),terminal_yaw_rad=yaw,visit_events=events)

def state(seg,s):
    T=seg['duration_s'];s=float(s);d=seg['length_m'];v=d*rate_basis(s)/T;a=d*accel_basis(s)/T**2
    if seg['kind']=='straight':
        p=np.array(seg['from_xy_m'])+smooth(s)*(np.array(seg['to_xy_m'])-seg['from_xy_m']);yaw=seg['yaw_rad'];delta=0.;deltadot=0.;yawdot=0.
    elif seg['kind']=='arc':
        arc=seg['sweep_rad'];th=seg['start_angle_rad']+arc*smooth(s);p=np.array(seg['center_xy_m'])+seg['radius_m']*np.array([math.cos(th),math.sin(th)])
        yaw=seg['yaw_start_rad']+arc*smooth(s);delta=seg['steering_rad'];deltadot=0.;yawdot=arc*rate_basis(s)/T
    else:
        p=np.array(seg['position_xy_m']);yaw=seg['yaw_rad'];delta=seg['steering_start_rad']+smooth(s)*(seg['steering_end_rad']-seg['steering_start_rad'])
        deltadot=(seg['steering_end_rad']-seg['steering_start_rad'])*rate_basis(s)/T;yawdot=0.
    return dict(xy_m=p.tolist(),yaw_rad=yaw,speed_mps=v,acceleration_mps2=a,steering_rad=delta,steering_rate_rad_s=deltadot,yaw_rate_rad_s=yawdot)
