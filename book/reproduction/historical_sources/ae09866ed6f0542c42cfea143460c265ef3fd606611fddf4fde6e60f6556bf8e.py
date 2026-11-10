"""Contract5 safety rows and exact geometric violation witnesses.

Witnesses retain free positions, durations, vehicle modes and task assignment.
They are necessary constraints, not a continuous certificate. All237 source
cylinders and2568 parabolic conductors remain in the independent search domain.
"""
import math
import numpy as np
from pyscipopt import sin,cos,sqrt,quicksum
from Agent.Publication.engineering_joint_safety import cylinder_signed_distance,wire_point,bezier,positive,clip01
from Agent.Publication.engineering_unified_inspection import minimum,maximum


def wire_closest(point,wire):
    """Global distance to a full quadratic wire by its cubic stationary roots."""
    a,b=np.asarray(wire['endpoint_xyz_m'],float);s=float(wire['sag_m'])
    c0=a-np.asarray(point,float);c1=b-a+np.array([0,0,-4*s]);c2=np.array([0,0,4*s])
    polynomial=np.array([2*np.dot(c2,c2),3*np.dot(c1,c2),np.dot(c1,c1)+2*np.dot(c0,c2),np.dot(c0,c1)])
    nonzero=np.flatnonzero(abs(polynomial)>1e-15)
    roots=np.roots(polynomial[nonzero[0]:]) if len(nonzero) else []
    candidates=[0.,1.]+[float(z.real) for z in roots if abs(z.imag)<1e-9 and 0<=z.real<=1]
    scores=[np.linalg.norm(c0+c1*t+c2*t*t) for t in candidates]
    k=int(np.argmin(scores));return float(scores[k])-float(wire['radius_m']),candidates[k]


def cylinder_distance_numeric(p,c):
    radial=float(np.linalg.norm(np.asarray(p)[:2]-c['center_xy_m']))-c['radius_m']
    vertical=max(c['base_elevation_m']-p[2],p[2]-c['base_elevation_m']-c['height_m'])
    return math.hypot(max(radial,0),max(vertical,0))+min(max(radial,vertical),0)


def capsule_margin(point,a,b,radius=3.):
    a=np.asarray(a,float);b=np.asarray(b,float);d=b-a;den=float(d@d)
    t=clip01(sum((point[q]-a[q])*d[q] for q in range(2))/den) if den>0 else 0.
    return radius-sqrt(sum((point[q]-a[q]-t*d[q])**2 for q in range(2)))


class RegionalSafety:
    def __init__(self,curves,vehicles,rendezvous,physics):
        self.curves=curves;self.vehicles=vehicles;self.rendezvous=rendezvous;self.physics=physics
        self.m=curves.m;self.path=curves.path;self.rel=curves.rel;self.origin=curves.origin
        self.scene=curves.scene;self.applied=set();self.serial=0;self.control_aux={}
        self.cylinders=self.scene['obstacle_model']['tower_cylinders']
        self.wires=self.scene['obstacle_model']['conductor_curves']
        self.cylinder_boxes=[];self.wire_boxes=[]
        for c in self.cylinders:
            self.cylinder_boxes.append((np.r_[np.asarray(c['center_xy_m'])-c['radius_m'],c['base_elevation_m']],
                np.r_[np.asarray(c['center_xy_m'])+c['radius_m'],c['base_elevation_m']+c['height_m']]))
        for w in self.wires:
            a,b=np.asarray(w['endpoint_xyz_m']);lo=np.minimum(a,b);hi=np.maximum(a,b)
            lo[2]-=w['sag_m'];self.wire_boxes.append((lo-w['radius_m'],hi+w['radius_m']))

    def _name(self):
        name=f'regional_safety_{self.serial}';self.serial+=1;return name

    def segment(self,task,index):return self.curves.by_task[task]['segments'][index]

    def point(self,segment,s):
        # An overlap parameter contains min/max and a duration quotient.
        # Repeating that expression inside de Casteljau creates exponentially
        # duplicated trees. Shared variables are exact defining equalities.
        name=self._name();j=self.physics
        if not isinstance(s,(int,float,np.number)):s=j.aux(s,name+'_curve_fraction',0,1)
        identity=(segment.task,segment.index)
        if identity not in self.control_aux:
            cp=segment.controls()['C'];mid=(self.curves.lower+self.curves.upper)/2;rad=(self.curves.upper-self.curves.lower)/2
            self.control_aux[identity]=[[j.aux(cp[k][q],f'safety_task{segment.task}_segment{segment.index}_control{k}_{q}',
                float(mid[q]-self.curves.inverse_norm[k]*rad[q]),float(mid[q]+self.curves.inverse_norm[k]*rad[q])) for q in range(3)] for k in range(8)]
        values=bezier(self.control_aux[identity],s)
        return [j.aux(x+self.origin[q],name+f'_world_point{q}',float(self.curves.lower[q]+self.origin[q]),float(self.curves.upper[q]+self.origin[q])) for q,x in enumerate(values)]

    def add_static(self,task,index,s,kind,obstacle,wire_parameter=None,ray=None,surface_sample=None):
        signature=('static',task,index,s,kind,obstacle,wire_parameter,ray,surface_sample)
        if signature in self.applied:return False
        segment=self.segment(task,index);point=self.point(segment,s);clearance=4.6
        if ray is not None:
            if segment.role!='capture' or surface_sample is None:raise ValueError('LOS needs an actual capture patch')
            target=self.curves.corrected_targets[task]['target']['surface_samples_utm_m'][surface_sample]
            if not 0<=ray<=1:raise ValueError('Invalid sight-line fraction')
            point=[(1-ray)*p+ray*q for p,q in zip(point,target)];clearance=0.
        name=self._name()
        if kind=='cylinder':
            # Host cylinder is deliberately NOT excluded.
            distance=cylinder_signed_distance(point,self.cylinders[obstacle])
            self.physics.gate(distance,segment.active,lo=clearance,name=name,family='obstacle_visibility')
        elif kind=='wire':
            if wire_parameter is None or not 0<=wire_parameter<=1:raise ValueError('Wire witness needs source parameter')
            w=self.wires[obstacle];q=wire_point(w,wire_parameter)
            self.physics.gate(sum((x-y)**2 for x,y in zip(point,q)),segment.active,
                lo=(clearance+w['radius_m'])**2,name=name,family='obstacle_visibility')
        elif kind=='terrain':
            g,_,_=self.physics.ground(point,name)
            self.physics.gate(point[2]-g,segment.active,lo=clearance,name=name,family='terrain_visibility')
        else:raise ValueError('Unknown obstacle family '+kind)
        self.applied.add(signature);return True

    def add_pair(self,task_a,index_a,task_b,index_b,fraction):
        signature=('uav_pair',task_a,index_a,task_b,index_b,fraction)
        if signature in self.applied:return False
        a=self.segment(task_a,index_a);b=self.segment(task_b,index_b);j=self.physics;name=self._name()
        left=maximum(a.clock,b.clock);right=minimum(a.clock+a.duration,b.clock+b.duration)
        width=right-left;overlap=j.binary(name+'_overlap')
        j.gate(width,overlap,lo=0,name=name+'_overlap_yes')
        j.gate(width,1-overlap,hi=0,name=name+'_overlap_no')
        both=j.and2(j.and2(a.active,b.active,name+'_both'),overlap,name+'_overlap_active')
        delta=self.path.tasks[task_a].flight.value-self.path.tasks[task_b].flight.value
        less,equal,greater=[j.binary(name+'_'+s) for s in ['flight_less','flight_same','flight_greater']]
        self.m.addCons(less+equal+greater==1)
        j.gate(delta,less,hi=-1,name=name+'_different_left');j.gate(delta,greater,lo=1,name=name+'_different_right')
        j.gate(delta,equal,0,0,name=name+'_equal')
        on=j.and2(both,1-equal,name+'_distinct_active')
        t=left+fraction*positive(width)
        pa=self.point(a,clip01((t-a.clock)/(a.duration+1-a.active)))
        pb=self.point(b,clip01((t-b.clock)/(b.duration+1-b.active)))
        j.gate(sum((x-y)**2 for x,y in zip(pa,pb)),on,lo=5.2**2,name=name+'_distance',family='inter_uav_safety')
        self.applied.add(signature);return True

    def road_union(self,category):
        dom=self.vehicles.domain;mode=dom.modes[category]
        if mode['kind']=='edge':return [(mode['a'],mode['b'])]
        node=mode['node'] if mode['kind']=='stop' else mode['id'];center=dom.xy[node]
        segments=[(center,center)]
        # Only the adjacent ell-length connection portions form the junction,
        # not entire roads out to the neighboring node.
        for edge in dom.modes[:len(dom.edges)]:
            if edge['tail']==node:segments.append((edge['a'],edge['a']+edge['ell']*edge['direction']))
            if edge['head']==node:segments.append((edge['b']-edge['ell']*edge['direction'],edge['b']))
        return segments

    def add_vehicle_footprint(self,carrier,phase_index,category,s,body_u,body_v):
        if not(-1<=body_u<=1 and -1<=body_v<=1):raise ValueError('Footprint parameter outside full rectangle')
        signature=('road_footprint',carrier,phase_index,category,s,body_u,body_v)
        if signature in self.applied:return False
        phase=self.vehicles.phases[carrier][phase_index];state=bezier(phase.controls,s)
        x,y,theta=state[:3];cfg=self.scene['joint_reference_profile']['vehicle']
        u=body_u*cfg['length_m']/2;v=body_v*cfg['width_m']/2
        point=[x+u*cos(theta)-v*sin(theta),y+u*sin(theta)+v*cos(theta)]
        margins=[capsule_margin(point,a,b) for a,b in self.road_union(category)]
        margin=margins[0]
        for q in margins[1:]:margin=maximum(margin,q)
        # A mode-specific witness remains globally valid for every future
        # route: exact binary-code equality activates only this source mode.
        mismatch=phase.mode.mismatch(category);on=self.physics.binary(self._name()+'_selected_mode')
        self.m.addCons(mismatch<=len(phase.mode.bits)*(1-on));self.m.addCons(mismatch>=1-on)
        self.physics.gate(margin,on,lo=0,name=self._name(),family='whole_vehicle_footprint')
        self.applied.add(signature);return True

    def search_static_point(self,point,clearance=4.6):
        """All source obstacles searched; bbox pruning has a distance proof."""
        point=np.asarray(point,float);failures=[]
        for kind,objects,boxes in [('cylinder',self.cylinders,self.cylinder_boxes),('wire',self.wires,self.wire_boxes)]:
            for i,(obj,(lo,hi)) in enumerate(zip(objects,boxes)):
                lower=np.linalg.norm(np.maximum(0,np.maximum(lo-point,point-hi)))
                if lower>clearance+1e-8:continue
                if kind=='cylinder':distance=cylinder_distance_numeric(point,obj);parameter=None
                else:distance,parameter=wire_closest(point,obj)
                if distance<clearance-1e-7:
                    failures.append(dict(kind=kind,obstacle=i,distance_m=distance,required_m=clearance,wire_parameter=parameter))
        return failures

    def status(self):
        return dict(source_cylinders=len(self.cylinders),source_conductors=len(self.wires),
            added_witnesses=len(self.applied),host_tower_preserved=True,uav_separation_m=5.2,
            no_obstacle_crop=True,continuous_certificate=False,aircraft_vehicle_collision_attached=False,
            inter_vehicle_collision_attached=False,full_joint_model=False)

    def vehicle_point(self,phase,s,body_u=0.,body_v=0.,body_z=0.):
        name=self._name();j=self.physics
        if not isinstance(s,(int,float,np.number)):s=j.aux(s,name+'_vehicle_fraction',0,1)
        state=[j.aux(x,name+f'_vehicle_state{q}',float(self.vehicles.state_lower[q]),float(self.vehicles.state_upper[q])) for q,x in enumerate(bezier(phase.controls,s))]
        x,y,theta=state[:3]
        cfg=self.scene['joint_reference_profile']['vehicle']
        u=body_u*cfg['length_m']/2;v=body_v*cfg['width_m']/2
        world=[x+self.origin[0],y+self.origin[1],0.]
        ground,_,_=self.physics.ground(world,self._name()+'_vehicle_ground')
        return [world[0]+u*cos(theta)-v*sin(theta),world[1]+u*sin(theta)+v*cos(theta),
                ground+body_z*cfg['height_m']],state,ground

    def _overlap(self,start_a,T_a,active_a,start_b,T_b,active_b,fraction,name):
        j=self.physics;left=maximum(start_a,start_b);right=minimum(start_a+T_a,start_b+T_b)
        width=right-left;overlap=j.binary(name+'_overlap')
        j.gate(width,overlap,lo=0,name=name+'_yes');j.gate(width,1-overlap,hi=0,name=name+'_no')
        on=j.and2(j.and2(active_a,active_b,name+'_both'),overlap,name+'_on')
        t=left+fraction*positive(width)
        return t,on

    def add_vehicle_pair(self,phase_a,phase_b,fraction,ua,va,ub,vb):
        signature=('vehicle_pair',phase_a.carrier,phase_a.index,phase_b.carrier,phase_b.index,fraction,ua,va,ub,vb)
        if signature in self.applied:return False
        if phase_a.carrier==phase_b.carrier:raise ValueError('Different physical vehicles required')
        j=self.physics;name=self._name();one=j.binary(name+'_one',1);self.m.chgVarLb(one,1)
        t,on=self._overlap(phase_a.start,phase_a.duration,one,phase_b.start,phase_b.duration,one,fraction,name)
        # Degenerate vehicle phases have a fixed continuous endpoint. Use
        # max(T,1e-9) only to evaluate that single endpoint, not to consume time.
        sa=clip01((t-phase_a.start)/maximum(phase_a.duration,1e-9))
        sb=clip01((t-phase_b.start)/maximum(phase_b.duration,1e-9))
        pa,_,_=self.vehicle_point(phase_a,sa,ua,va);pb,_,_=self.vehicle_point(phase_b,sb,ub,vb)
        j.gate(sum((pa[q]-pb[q])**2 for q in range(2)),on,lo=.5**2,name=name+'_ground_net_distance',family='vehicle_safety')
        self.applied.add(signature);return True

    def add_aircraft_vehicle(self,task,index,carrier,phase_index,fraction):
        signature=('aircraft_vehicle',task,index,carrier,phase_index,fraction)
        if signature in self.applied:return False
        segment=self.segment(task,index);phase=self.vehicles.phases[carrier][phase_index]
        j=self.physics;name=self._name();one=j.binary(name+'_one',1);self.m.chgVarLb(one,1)
        t,on=self._overlap(segment.clock,segment.duration,segment.active,phase.start,phase.duration,one,fraction,name)
        sa=clip01((t-segment.clock)/(segment.duration+1-segment.active))
        sb=clip01((t-phase.start)/maximum(phase.duration,1e-9))
        air=self.point(segment,sa);vehicle,state,ground=self.vehicle_point(phase,sb)
        theta=state[2];dx=air[0]-vehicle[0];dy=air[1]-vehicle[1]
        local=[dx*cos(theta)+dy*sin(theta),-dx*sin(theta)+dy*cos(theta),air[2]-ground-.9]
        q=[abs(local[k])-half for k,half in enumerate([2.,.9,.9])]
        signed=sqrt(sum(positive(x)**2 for x in q))+minimum(maximum(maximum(q[0],q[1]),q[2]),0)
        contact_flag=None
        if segment.role=='in' and segment.index==0:contact_flag=self.path.tasks[task].root
        elif segment.role=='recovery' and segment.index==2*self.curves.transfers:contact_flag=self.path.tasks[task].last
        clearance=4.6
        if contact_flag is not None:
            own=j.binary(name+'_own_carrier')
            def rows(k):
                if not k:return [AffineRow('zero_inactive_carrier',[(1,own)],0,0)]
                match=int(self.path.flights[k-1].carrier==carrier)
                return [AffineRow('own_carrier',[(1,own)],match,match)]
            from Agent.Publication.engineering_indexed_constraints import AffineRow
            self.rel.add(self.path.tasks[task].flight,rows,[own],name=name+'_ownership')
            contact=j.and2(contact_flag,own,name+'_own_contact')
            ramp=.6+4*clip01((air[2]-ground-2.4)/4)
            clearance=4.6+contact*(ramp-4.6)
        j.gate(signed-clearance,on,lo=0,name=name+'_body_clearance',family='aircraft_vehicle_safety')
        self.applied.add(signature);return True

    def add_pad_point(self,task,index,s):
        if ('pad',task,index,s) in self.applied:return False
        segment=self.segment(task,index);t=self.path.tasks[task];j=self.physics;name=self._name()
        launch=segment.role=='in' and index==0
        landing=segment.role=='recovery' and index==2*self.curves.transfers
        if not (launch or landing):raise ValueError('Pad point must be on contact piece')
        flag=t.root if launch else t.last;point=self.point(segment,s)
        # Endogenous pad anchor selected by the same task-flight identity.
        pad=[j.var(name+f'_pad{q}',float(self.curves.lower[q]+self.origin[q]),
                   float(self.curves.upper[q]+self.origin[q])) for q in range(3)]
        def rows(k):
            from Agent.Publication.engineering_indexed_constraints import AffineRow
            if not k:return [AffineRow('nonzero_pad_flight',[],lo=1)]
            a=self.rendezvous.anchors[k-1];anchor=a.launch if launch else a.land
            return [AffineRow('same_pad'+str(q),[(1,pad[q]),(-1,anchor.position[0][q])],self.origin[q],self.origin[q]) for q in range(3)]
        self.rel.add(t.flight,rows,pad,flag,name+'_pad_anchor')
        j.gate(sum((point[q]-pad[q])**2 for q in range(2)),flag,hi=9,name=name+'_pad_cylinder')
        j.gate(point[2]-pad[2],flag,0,4,name=name+'_pad_height')
        if s in [0.,1.] and ((launch and s==0) or (landing and s==1)):
            data=j.physics_segment(segment,s,name+'_contact_physics')
            for q in [0,1]:j.gate(data['rotation'][2][q],flag,0,0,name=name+f'_contact_vertical{q}')
        self.applied.add(('pad',task,index,s));return True

    def add_vehicle_static(self,carrier,phase_index,s,body_u,body_v,body_z,kind,obstacle,wire_parameter=None):
        signature=('vehicle_static',carrier,phase_index,s,body_u,body_v,body_z,kind,obstacle,wire_parameter)
        if signature in self.applied:return False
        if not(-1<=body_u<=1 and -1<=body_v<=1 and 0<=body_z<=1):raise ValueError('Invalid vehicle solid point')
        phase=self.vehicles.phases[carrier][phase_index];p,_,_=self.vehicle_point(phase,s,body_u,body_v,body_z)
        name=self._name()
        if kind=='cylinder':self.physics.row(cylinder_signed_distance(p,self.cylinders[obstacle]),lo=0,name=name,family='ground_obstacles')
        elif kind=='wire':
            wire=self.wires[obstacle];q=wire_point(wire,wire_parameter)
            self.physics.row(sum((x-y)**2 for x,y in zip(p,q)),lo=wire['radius_m']**2,name=name,family='ground_obstacles')
        else:raise ValueError('Unknown ground obstacle')
        self.applied.add(signature);return True

    def add_parking_capacity(self,carrier_a,index_a,carrier_b,index_b,category):
        signature=('parking',carrier_a,index_a,carrier_b,index_b,category)
        if signature in self.applied:return False
        if self.vehicles.domain.modes[category]['kind']!='stop':raise ValueError('Capacity is per configured stop')
        a=self.vehicles.phases[carrier_a][index_a];b=self.vehicles.phases[carrier_b][index_b];name=self._name();j=self.physics
        ma=a.mode.mismatch(category);mb=b.mode.mismatch(category);M=self.path.H
        order=j.binary(name+'_order');off=ma+mb
        self.m.addCons(a.end<=b.start+M*(1-order+off));self.m.addCons(b.end<=a.start+M*(order+off))
        self.applied.add(signature);return True

    def add_vehicle_visibility(self,task,index,carrier,phase_index,fraction,ray,surface_sample):
        signature=('vehicle_los',task,index,carrier,phase_index,fraction,ray,surface_sample)
        if signature in self.applied:return False
        segment=self.segment(task,index);phase=self.vehicles.phases[carrier][phase_index]
        if segment.role!='capture':raise ValueError('LOS belongs to capture interval')
        j=self.physics;name=self._name();one=j.binary(name+'_one');self.m.chgVarLb(one,1)
        t,on=self._overlap(segment.clock,segment.duration,segment.active,phase.start,phase.duration,one,fraction,name)
        a=clip01((t-segment.clock)/(segment.duration+1-segment.active));b=clip01((t-phase.start)/maximum(phase.duration,1e-9))
        camera=self.point(segment,a);target=self.curves.corrected_targets[task]['target']['surface_samples_utm_m'][surface_sample]
        point=[(1-ray)*x+ray*y for x,y in zip(camera,target)];vehicle,state,ground=self.vehicle_point(phase,b)
        dx=point[0]-vehicle[0];dy=point[1]-vehicle[1];th=state[2]
        local=[dx*cos(th)+dy*sin(th),-dx*sin(th)+dy*cos(th),point[2]-ground-.9]
        q=[abs(local[k])-half for k,half in enumerate([2.,.9,.9])]
        signed=sqrt(sum(positive(x)**2 for x in q))+minimum(maximum(maximum(q[0],q[1]),q[2]),0)
        j.gate(signed,on,lo=0,name=name+'_visibility',family='vehicle_visibility')
        self.applied.add(signature);return True
