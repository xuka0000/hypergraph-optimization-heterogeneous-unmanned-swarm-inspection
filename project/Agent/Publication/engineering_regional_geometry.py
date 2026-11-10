"""Continuous geometric acceptance and replayable necessary-row witnesses.

Subdivision uses polynomial convex hulls and distance Lipschitz bounds. A
depth exhaustion is unresolved, never feasible. No source obstacle is removed.
"""
import math
import numpy as np
from Agent.Publication.engineering_free_trajectory import split_controls
from Agent.Publication.engineering_regional_safety import wire_closest,cylinder_distance_numeric
from Agent.Publication.engineering_interval_physics import I,hull,vector,norm


def evaluate(c,s):
    work=np.asarray(c,float).copy()
    while len(work)>1:work=(1-s)*work[:-1]+s*work[1:]
    return work[0]


def restrict(c,a,b):
    """Exact de Casteljau restriction including constant intervals."""
    c=np.asarray(c,float)
    if not -1e-10<=a<=b<=1+1e-10:raise ValueError('Invalid polynomial interval')
    a=max(0.,a);b=min(1.,b)
    if b-a<1e-14:return np.tile(evaluate(c,(a+b)/2),(len(c),1))
    if b<1:c=split_controls(c,b)[0]
    if a>0:c=split_controls(c,a/b)[1]
    return c


def capsule_distance(point,a,b):
    a=np.asarray(a);b=np.asarray(b);d=b-a;den=d@d
    t=np.clip((point-a)@d/den,0,1) if den>0 else 0.
    return float(np.linalg.norm(point-a-t*d))


def rectangle_closest(a,b,length=4.,width=1.8):
    """Exact distance and body coordinates for two closed ground rectangles."""
    from shapely.geometry import Polygon
    from shapely.ops import nearest_points
    def polygon(state):
        x,y,theta=state[:3];R=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
        return Polygon(np.array([[-length/2,-width/2],[length/2,-width/2],[length/2,width/2],[-length/2,width/2]])@R.T+[x,y]),R
    pa,ra=polygon(a);pb,rb=polygon(b);p,q=nearest_points(pa,pb)
    uv_a=(np.array(p.coords[0])-a[:2])@ra/[length/2,width/2]
    uv_b=(np.array(q.coords[0])-b[:2])@rb/[length/2,width/2]
    return pa.distance(pb),np.clip(uv_a,-1,1),np.clip(uv_b,-1,1)


class RegionalGeometry:
    def __init__(self,safety,enclosure,tolerance=1e-6,max_depth=22):
        self.safety=safety;self.enclosure=enclosure;self.tolerance=tolerance;self.max_depth=max_depth
        self.certified_leaves=0
        self.obstacle_boxes={kind:np.asarray(boxes,float) for kind,boxes in
            [('cylinder',safety.cylinder_boxes),('wire',safety.wire_boxes)]}

    def _obstacles(self,cp,clearance):
        lo=cp.min(axis=0);hi=cp.max(axis=0);center=(lo+hi)/2;radius=float(np.linalg.norm((hi-lo)/2))
        unresolved=[]
        for kind,objects,boxes in [('cylinder',self.safety.cylinders,self.safety.cylinder_boxes),('wire',self.safety.wires,self.safety.wire_boxes)]:
            box=self.obstacle_boxes[kind]
            lower=np.linalg.norm(np.maximum(0,np.maximum(box[:,0]-hi,lo-box[:,1])),axis=1)
            for index in np.flatnonzero(lower<clearance+self.tolerance):
                index=int(index);obstacle=objects[index]
                distance=(cylinder_distance_numeric(center,obstacle) if kind=='cylinder' else wire_closest(center,obstacle)[0])
                if distance-radius>=clearance-self.tolerance:continue
                unresolved.append((kind,index))
        return unresolved

    def _point_witness(self,p,clearance,candidates):
        for kind,i in candidates:
            if kind=='cylinder':d=cylinder_distance_numeric(p,self.safety.cylinders[i]);u=None
            else:d,u=wire_closest(p,self.safety.wires[i])
            if d<clearance-self.tolerance:return dict(kind=kind,obstacle=i,wire_parameter=u,distance_m=d)
        return None

    def segment(self,cp,contact=None):
        """Static body obstacles, source terrain, own-pad cylinder/ramp."""
        stack=[(np.asarray(cp,float),0.,1.,0)]
        while stack:
            c,a,b,depth=stack.pop();candidates=self._obstacles(c,4.6);mid=evaluate(c,.5)
            witness=self._point_witness(mid,4.6,candidates)
            if witness:return dict(passed=False,witness=dict(witness,s=(a+b)/2),unresolved=False)
            try:
                terrain,_=self.enclosure.terrain_enclosure(c)
                h=hull(c)[2]-terrain
                if contact is None:terrain_ok=float(h.lo)>=4.6-self.tolerance;pad_ok=True
                else:
                    pad=np.asarray(contact,float);relative=hull(c)-pad
                    ramp=.6+4*min(1,max(0,float(relative[2].hi)/4))
                    terrain_ok=float(h.lo)>=ramp-self.tolerance
                    pad_ok=float(norm(relative[:2]).hi)<=3+self.tolerance and float(relative[2].lo)>=-self.tolerance and float(relative[2].hi)<=4+self.tolerance
                g=float(self.enclosure.terrain(*mid[:2]))
                margin=4.6 if contact is None else .6+4*np.clip((mid[2]-contact[2])/4,0,1)
                if not np.isfinite(g) or mid[2]-g<margin-self.tolerance:
                    return dict(passed=False,witness=dict(kind='terrain_clearance',s=(a+b)/2),unresolved=False)
                if contact is not None and (np.linalg.norm(mid[:2]-contact[:2])>3+self.tolerance or not -self.tolerance<=mid[2]-contact[2]<=4+self.tolerance):
                    return dict(passed=False,witness=dict(kind='pad',s=(a+b)/2),unresolved=False)
            except (ValueError,ArithmeticError):terrain_ok=pad_ok=False
            if not candidates and terrain_ok and pad_ok:self.certified_leaves+=1;continue
            if depth>=self.max_depth:return dict(passed=False,witness=None,unresolved=True,interval=[a,b],families=['body_obstacles','terrain','pad'])
            l,r=split_controls(c);s=(a+b)/2;stack += [(r,s,b,depth+1),(l,a,s,depth+1)]
        return dict(passed=True,witness=None,unresolved=False)

    def sight_line(self,cp,target):
        """All t and ray fractions, including host tower and terrain."""
        stack=[(np.asarray(cp,float),0.,1.,0.,1.,0)]
        while stack:
            c,a,b,u,v,depth=stack.pop();cloud=np.vstack([(1-u)*c+u*target,(1-v)*c+v*target])
            candidates=self._obstacles(cloud,0.);s=(a+b)/2;ray=(u+v)/2;p=(1-ray)*evaluate(c,.5)+ray*target
            w=self._point_witness(p,0.,candidates)
            if w:return dict(passed=False,witness=dict(w,s=s,ray=ray),unresolved=False)
            try:
                terrain,_=self.enclosure.terrain_enclosure(cloud);terrain_ok=cloud[:,2].min()>=float(terrain.hi)-self.tolerance
                ground=float(self.enclosure.terrain(*p[:2]))
                if not np.isfinite(ground) or p[2]<ground-self.tolerance:
                    return dict(passed=False,witness=dict(kind='terrain',s=s,ray=ray),unresolved=False)
            except (ValueError,ArithmeticError):terrain_ok=False
            if not candidates and terrain_ok:self.certified_leaves+=1;continue
            if depth>=self.max_depth:return dict(passed=False,witness=None,unresolved=True,interval=[a,b,u,v],families=['visibility'])
            time_width=np.linalg.norm(np.ptp(c,axis=0))*(1-u)
            ray_width=(v-u)*np.linalg.norm(target-c.mean(axis=0))
            if time_width>=ray_width:
                l,r=split_controls(c);stack += [(r,s,b,u,v,depth+1),(l,a,s,u,v,depth+1)]
            else:stack += [(c,a,b,ray,v,depth+1),(c,a,b,u,ray,depth+1)]
        return dict(passed=True,witness=None,unresolved=False)

    def aircraft_pair(self,ca,ta,Ta,cb,tb,Tb,required=5.2):
        left=max(ta,tb);right=min(ta+Ta,tb+Tb)
        if right<left-self.tolerance:return dict(passed=True,overlap=False)
        right=max(left,right)
        a=restrict(ca,np.clip((left-ta)/Ta,0,1),np.clip((right-ta)/Ta,0,1))
        b=restrict(cb,np.clip((left-tb)/Tb,0,1),np.clip((right-tb)/Tb,0,1))
        stack=[(a-b,0.,1.,0)]
        while stack:
            c,l,r,depth=stack.pop();box=hull(c);lower=float(norm(box).lo);value=float(np.linalg.norm(evaluate(c,.5)))
            if value<required-self.tolerance:return dict(passed=False,witness=dict(fraction=(l+r)/2,distance_m=value),unresolved=False)
            if lower>=required-self.tolerance:self.certified_leaves+=1;continue
            if depth>=self.max_depth:return dict(passed=False,witness=None,unresolved=True,interval=[l,r])
            p,q=split_controls(c);mid=(l+r)/2;stack += [(q,mid,r,depth+1),(p,l,mid,depth+1)]
        return dict(passed=True,overlap=True)

    def footprint(self,cp,category):
        """Every point of the body rectangle, all time, within source union."""
        capsules=self.safety.road_union(category);stack=[(np.asarray(cp),0.,1.,-1.,1.,-1.,1.,0)]
        while stack:
            c,s0,s1,u0,u1,v0,v1,depth=stack.pop();state=hull(c);theta=state[2]
            u=I(2*u0,2*u1);v=I(.9*v0,.9*v1)
            x=state[0]+u*theta.cos()-v*theta.sin();y=state[1]+u*theta.sin()+v*theta.cos()
            center=np.array([(x.lo+x.hi)/2,(y.lo+y.hi)/2]);radius=float(np.linalg.norm([(x.hi-x.lo)/2,(y.hi-y.lo)/2]))
            margin=max(3-capsule_distance(center,a,b) for a,b in capsules)
            if margin-radius>=-self.tolerance:self.certified_leaves+=1;continue
            s=(s0+s1)/2;uu=(u0+u1)/2;vv=(v0+v1)/2;st=evaluate(c,.5)
            p=st[:2]+[2*uu*math.cos(st[2])-.9*vv*math.sin(st[2]),2*uu*math.sin(st[2])+.9*vv*math.cos(st[2])]
            actual=max(3-capsule_distance(p,a,b) for a,b in capsules)
            if actual < -self.tolerance:return dict(passed=False,witness=dict(s=s,body_u=uu,body_v=vv,margin_m=actual),unresolved=False)
            if depth>=self.max_depth:return dict(passed=False,witness=None,unresolved=True,interval=[s0,s1,u0,u1,v0,v1])
            extents=[np.linalg.norm(np.ptp(c[:,:2],axis=0))+2.2*np.ptp(c[:,2]),2*(u1-u0),.9*(v1-v0)]
            axis=int(np.argmax(extents))
            if axis==0:
                a,b=split_controls(c);stack += [(b,s,s1,u0,u1,v0,v1,depth+1),(a,s0,s,u0,u1,v0,v1,depth+1)]
            elif axis==1:stack += [(c,s0,s1,uu,u1,v0,v1,depth+1),(c,s0,s1,u0,uu,v0,v1,depth+1)]
            else:stack += [(c,s0,s1,u0,u1,vv,v1,depth+1),(c,s0,s1,u0,u1,v0,vv,depth+1)]
        return dict(passed=True)

    def vehicle_pair(self,ca,ta,Ta,cb,tb,Tb):
        left=max(ta,tb);right=min(ta+Ta,tb+Tb)
        if right<left-self.tolerance:return dict(passed=True,overlap=False)
        def sub(c,t,T):
            if T<1e-12:return np.tile(c[0],(len(c),1))
            return restrict(c,np.clip((left-t)/T,0,1),np.clip((max(left,right)-t)/T,0,1))
        stack=[(sub(ca,ta,Ta),sub(cb,tb,Tb),0.,1.,0)]
        while stack:
            a,b,l,r,depth=stack.pop();am=evaluate(a,.5);bm=evaluate(b,.5)
            d,ua,ub=rectangle_closest(am,bm)
            if d<.5-self.tolerance:return dict(passed=False,witness=dict(fraction=(l+r)/2,ua=float(ua[0]),va=float(ua[1]),ub=float(ub[0]),vb=float(ub[1]),distance_m=d),unresolved=False)
            def motion(c,mid):
                translation=float(np.linalg.norm(c[:,:2]-mid[:2],axis=1).max())
                angle=float(abs(c[:,2]-mid[2]).max())
                return translation+math.hypot(2,.9)*min(2,angle)
            if d-motion(a,am)-motion(b,bm)>=.5-self.tolerance:self.certified_leaves+=1;continue
            if depth>=self.max_depth:return dict(passed=False,witness=None,unresolved=True,interval=[l,r])
            al,ar=split_controls(a);bl,br=split_controls(b);s=(l+r)/2
            stack += [(ar,br,s,r,depth+1),(al,bl,l,s,depth+1)]
        return dict(passed=True,overlap=True)

    def aircraft_vehicle(self,air,ta,Ta,vehicle,tb,Tb,own_contact=False):
        left=max(ta,tb);right=min(ta+Ta,tb+Tb)
        if right<left-self.tolerance:return dict(passed=True,overlap=False)
        ca=restrict(air,np.clip((left-ta)/Ta,0,1),np.clip((max(left,right)-ta)/Ta,0,1))
        cb=(restrict(vehicle,np.clip((left-tb)/Tb,0,1),np.clip((max(left,right)-tb)/Tb,0,1))
            if Tb>1e-12 else np.tile(vehicle[0],(len(vehicle),1)))
        stack=[(ca,cb,0.,1.,0)]
        while stack:
            a,b,l,r,depth=stack.pop();A=hull(a);B=hull(b);theta=B[2]
            world=np.column_stack([b[:,:2]+self.safety.origin[:2],np.zeros(len(b))])
            try:ground,_=self.enclosure.terrain_enclosure(world)
            except (ValueError,ArithmeticError):return dict(passed=False,witness=None,unresolved=True,reason='vehicle_DEM')
            dx=A[0]-B[0]-self.safety.origin[0];dy=A[1]-B[1]-self.safety.origin[1]
            local=vector([dx*theta.cos()+dy*theta.sin(),-dx*theta.sin()+dy*theta.cos(),A[2]-ground-.9])
            # A lower bound on distance to the box. If the hull intersects it,
            # zero is conservative for the required positive exterior margin.
            outside=np.maximum(0,np.maximum(local.lo-np.array([2,.9,.9]),-local.hi-np.array([2,.9,.9])))
            lower=float(np.linalg.norm(outside))
            # For own-pad contact, height>=2.4 implies distance to the entire
            # vehicle box >= height-1.8 >= .6+4*clip((height-2.4)/4,0,1).
            # Keep the shared height expression; independently boxing the two
            # sides loses this exact equality and can subdivide indefinitely.
            if own_contact and float((A[2]-ground).lo)>=2.4-self.tolerance:
                self.certified_leaves+=1;continue
            margin=4.6 if not own_contact else .6+4*np.clip((float(A[2].hi)-float(ground.lo)-2.4)/4,0,1)
            am=evaluate(a,.5);bm=evaluate(b,.5);g=float(self.enclosure.terrain(*(bm[:2]+self.safety.origin[:2])))
            d=am[:2]-bm[:2]-self.safety.origin[:2];th=bm[2]
            q=abs(np.array([d[0]*math.cos(th)+d[1]*math.sin(th),-d[0]*math.sin(th)+d[1]*math.cos(th),am[2]-g-.9]))-[2,.9,.9]
            actual=float(np.linalg.norm(np.maximum(q,0))+min(max(q),0))
            needed=4.6 if not own_contact else .6+4*np.clip((am[2]-g-2.4)/4,0,1)
            if not np.isfinite(g) or actual<needed-self.tolerance:
                return dict(passed=False,witness=dict(fraction=(l+r)/2,distance_m=actual,required_m=float(needed)),unresolved=False)
            if lower>=margin-self.tolerance:self.certified_leaves+=1;continue
            if depth>=self.max_depth:return dict(passed=False,witness=None,unresolved=True,interval=[l,r])
            al,ar=split_controls(a);bl,br=split_controls(b);s=(l+r)/2;stack += [(ar,br,s,r,depth+1),(al,bl,l,s,depth+1)]
        return dict(passed=True,overlap=True)

    def vehicle_dynamics(self,cp,T,mode,equality_tolerance=1e-4):
        """Engineering residual tolerance in m/s and rad/s; no exact-OCP claim."""
        cp=np.asarray(cp,float);cfg=self.safety.scene['joint_reference_profile']['vehicle']
        if T<=1e-12:
            return dict(passed=bool(np.max(abs(cp-cp[0]))<1e-6),zero_duration=True,
                        unresolved=False,witness=dict(s=.5) if np.max(abs(cp-cp[0]))>=1e-6 else None)
        def bounds(c,duration,point=None):
            state,d=(hull(c),hull((len(c)-1)*np.diff(c,axis=0)/duration)) if point is None else (I(point[0]),I(point[1]))
            theta=state[2];v=state[3];steer=state[4]
            if steer.cos().lo<=0:raise ArithmeticError('steering cosine unresolved')
            residual=[d[0]-v*theta.cos(),d[1]-v*theta.sin(),d[2]-v*steer.sin()/steer.cos()/cfg['wheelbase_m']]
            checks=dict(ackermann=all(float(x.absmax())<=equality_tolerance for x in residual),
                speed=float(v.lo)>=-1e-6 and float(v.hi)<=mode.get('speed',0)+1e-6,
                acceleration=float(d[3].absmax())<=cfg['acceleration_max_mps2']+1e-6,
                steering=float(steer.absmax())<=math.radians(cfg['steering_max_deg'])+1e-6,
                steering_rate=float(d[4].absmax())<=math.radians(cfg['steering_rate_max_deg_s'])+1e-6,
                lateral_acceleration=float((v*v*steer.sin()/steer.cos()/cfg['wheelbase_m']).absmax())<=cfg['lateral_acceleration_max_mps2']+1e-6)
            if mode['kind']=='edge':
                progress=(state[0]-mode['a'][0])*mode['direction'][0]+(state[1]-mode['a'][1])*mode['direction'][1]
                forward=d[0]*mode['direction'][0]+d[1]*mode['direction'][1]
                heading=theta.cos()*mode['direction'][0]+theta.sin()*mode['direction'][1]
                checks.update(core_progress=float(progress.lo)>=mode['ell']-1e-6 and float(progress.hi)<=mode['length']-mode['ell']+1e-6,
                              no_reverse=float(forward.lo)>=-1e-6 and float(heading.lo)>=-1e-6)
            return checks
        stack=[(cp,T,0.,1.,0)]
        while stack:
            c,duration,a,b,depth=stack.pop()
            try:checks=bounds(c,duration)
            except ArithmeticError:checks={'interval_singular':False}
            if all(checks.values()):self.certified_leaves+=1;continue
            # A point violation can be materialized in the same joint model.
            state=evaluate(c,.5);d=evaluate((len(c)-1)*np.diff(c,axis=0)/duration,.5)
            try:actual=bounds(c,duration,(state,d))
            except ArithmeticError:actual={'point_singular':False}
            if not all(actual.values()):return dict(passed=False,witness=dict(s=(a+b)/2,checks=actual),unresolved=False)
            if depth>=self.max_depth:return dict(passed=False,witness=None,unresolved=True,interval=[a,b],checks=checks)
            l,r=split_controls(c);mid=(a+b)/2;stack += [(r,duration/2,mid,b,depth+1),(l,duration/2,a,mid,depth+1)]
        return dict(passed=True,equality_tolerance=equality_tolerance)

    def vehicle_static(self,cp):
        """Whole3D vehicle solid vs all source towers/conductors, all time."""
        stack=[(np.asarray(cp),0.,1.,-1.,1.,-1.,1.,0.,1.,0)]
        while stack:
            c,a,b,u0,u1,v0,v1,z0,z1,depth=stack.pop();state=hull(c);theta=state[2]
            world=np.column_stack([c[:,:2]+self.safety.origin[:2],np.zeros(len(c))])
            try:ground,_=self.enclosure.terrain_enclosure(world)
            except (ArithmeticError,ValueError):return dict(passed=False,witness=None,unresolved=True,reason='vehicle_DEM')
            u=I(2*u0,2*u1);v=I(.9*v0,.9*v1)
            xyz=vector([state[0]+self.safety.origin[0]+u*theta.cos()-v*theta.sin(),
                        state[1]+self.safety.origin[1]+u*theta.sin()+v*theta.cos(),ground+I(1.8*z0,1.8*z1)])
            candidates=self._obstacles(np.vstack([xyz.lo,xyz.hi]),0.)
            if not candidates:self.certified_leaves+=1;continue
            mid=evaluate(c,.5);uu=(u0+u1)/2;vv=(v0+v1)/2;zz=(z0+z1)/2
            g=float(self.enclosure.terrain(*(mid[:2]+self.safety.origin[:2])))
            point=np.r_[mid[:2]+self.safety.origin[:2]+[2*uu*math.cos(mid[2])-.9*vv*math.sin(mid[2]),2*uu*math.sin(mid[2])+.9*vv*math.cos(mid[2])],g+1.8*zz]
            w=self._point_witness(point,0.,candidates)
            if w:return dict(passed=False,witness=dict(w,s=(a+b)/2,body_u=uu,body_v=vv,body_z=zz),unresolved=False)
            if depth>=self.max_depth:return dict(passed=False,witness=None,unresolved=True,interval=[a,b,u0,u1,v0,v1,z0,z1])
            ext=[np.linalg.norm(np.ptp(c[:,:2],axis=0))+2.2*np.ptp(c[:,2])+float(ground.hi-ground.lo),2*(u1-u0),.9*(v1-v0),1.8*(z1-z0)]
            k=int(np.argmax(ext));d=depth+1
            if k==0:
                l,r=split_controls(c);s=(a+b)/2;stack += [(r,s,b,u0,u1,v0,v1,z0,z1,d),(l,a,s,u0,u1,v0,v1,z0,z1,d)]
            elif k==1:stack += [(c,a,b,uu,u1,v0,v1,z0,z1,d),(c,a,b,u0,uu,v0,v1,z0,z1,d)]
            elif k==2:stack += [(c,a,b,u0,u1,vv,v1,z0,z1,d),(c,a,b,u0,u1,v0,vv,z0,z1,d)]
            else:stack += [(c,a,b,u0,u1,v0,v1,zz,z1,d),(c,a,b,u0,u1,v0,v1,z0,zz,d)]
        return dict(passed=True)

    def vehicle_sight_line(self,air,ta,Ta,vehicle,tb,Tb,target):
        left=max(ta,tb);right=min(ta+Ta,tb+Tb)
        if right<left-self.tolerance:return dict(passed=True,overlap=False)
        ca=restrict(air,np.clip((left-ta)/Ta,0,1),np.clip((max(left,right)-ta)/Ta,0,1))
        cb=restrict(vehicle,np.clip((left-tb)/Tb,0,1),np.clip((max(left,right)-tb)/Tb,0,1)) if Tb>1e-12 else np.tile(vehicle[0],(len(vehicle),1))
        stack=[(ca,cb,0.,1.,0.,1.,0)]
        while stack:
            a,b,l,r,u,v,depth=stack.pop();cloud=np.vstack([(1-u)*a+u*target,(1-v)*a+v*target]);A=hull(cloud);B=hull(b);theta=B[2]
            world=np.column_stack([b[:,:2]+self.safety.origin[:2],np.zeros(len(b))])
            try:ground,_=self.enclosure.terrain_enclosure(world)
            except (ValueError,ArithmeticError):return dict(passed=False,witness=None,unresolved=True,reason='vehicle_DEM')
            dx=A[0]-B[0]-self.safety.origin[0];dy=A[1]-B[1]-self.safety.origin[1]
            local=vector([dx*theta.cos()+dy*theta.sin(),-dx*theta.sin()+dy*theta.cos(),A[2]-ground-.9])
            minabs=np.where((local.lo<=0)&(local.hi>=0),0,np.minimum(abs(local.lo),abs(local.hi)))
            qlo=minabs-[2,.9,.9];lower=float(np.linalg.norm(np.maximum(qlo,0))+min(max(qlo),0))
            if lower>=-self.tolerance:self.certified_leaves+=1;continue
            ray=(u+v)/2;am=(1-ray)*evaluate(a,.5)+ray*target;bm=evaluate(b,.5)
            g=float(self.enclosure.terrain(*(bm[:2]+self.safety.origin[:2])));d=am[:2]-bm[:2]-self.safety.origin[:2];th=bm[2]
            q=abs(np.array([d[0]*math.cos(th)+d[1]*math.sin(th),-d[0]*math.sin(th)+d[1]*math.cos(th),am[2]-g-.9]))-[2,.9,.9]
            actual=float(np.linalg.norm(np.maximum(q,0))+min(max(q),0))
            if actual< -self.tolerance:return dict(passed=False,witness=dict(fraction=(l+r)/2,ray=ray,distance_m=actual),unresolved=False)
            if depth>=self.max_depth:return dict(passed=False,witness=None,unresolved=True,interval=[l,r,u,v])
            time_span=np.linalg.norm(np.ptp(a,axis=0))+np.linalg.norm(np.ptp(b[:,:2],axis=0))+2.2*np.ptp(b[:,2])
            ray_span=(v-u)*np.linalg.norm(target-a.mean(axis=0));dep=depth+1
            if time_span>=ray_span:
                al,ar=split_controls(a);bl,br=split_controls(b);s=(l+r)/2;stack += [(ar,br,s,r,u,v,dep),(al,bl,l,s,u,v,dep)]
            else:stack += [(a,b,l,r,ray,v,dep),(a,b,l,r,u,ray,dep)]
        return dict(passed=True,overlap=True)
