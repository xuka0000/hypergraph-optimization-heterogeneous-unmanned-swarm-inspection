"""Contract5 point dynamics/optics on free regional trajectories.

These are necessary point constraints. Their presence is NOT continuous
acceptance, a rigorous energy-integral bound, or a completed regional solver.
The optical projection uses the existing verified ideal-H20 formulation while
the terrain/weather/contact clock adapters below replace the pilot adapters.
"""
import math
import numpy as np
from pyscipopt import Expr,quicksum,sin,cos,sqrt
from Agent.Publication.engineering_unified_inspection import JointModel,dot,cross,plus,maximum,minimum
from Agent.Publication.engineering_indexed_constraints import AffineRow


class RegionalPhysics(JointModel):
    def __init__(self,curves,terrain,forecast,rendezvous,seed_solution=None):
        # Intentionally do not invoke the pilot constructor: it creates a
        # frozen-route warm start and a one-component scope.
        self.curves=curves;self.terrain_index=terrain;self.forecast=forecast;self.rendezvous=rendezvous
        self.path=curves.path;self.m=self.path.m;self.rel=self.path.rel
        self.scene=curves.scene;self.targets=curves.corrected_targets
        self.cfg=self.scene['joint_reference_profile'];self.origin=curves.origin
        self.state_lower=curves.lower+self.origin;self.state_upper=curves.upper+self.origin
        self.yaw_state_bound=curves.pose_hi[0]
        self.vars={};self.groups={};self.rows=[];self.points={};self.energy={}
        self.materialized_segments=set();self.counter=0
        self.segment_energy={};self.energy_registered=False
        self.energy_links={};self.current_orders={};self.extra_points=set();self.exposure_points=set()
        self.seed_solution=seed_solution;self.seed_residuals=[]
        from Agent.Publication.engineering_regional_physics_bounds import physical_auxiliary_bounds
        self.auxiliary_bounds=physical_auxiliary_bounds(self.cfg,forecast,terrain)

    def var(self,name,lb=0,ub=None,initial=0,kind='C',group='auxiliary'):
        if name in self.vars:raise ValueError('Duplicate regional physical variable '+name)
        v=self.m.addVar(name=name,lb=lb,ub=ub,vtype=kind)
        self.vars[name]=v;self.groups[name]=group
        if self.seed_solution is not None:
            # Original solutions default newly added variables to zero, even
            # when zero is outside the domain (e.g. absolute DEM heights).
            # This initializes a candidate only; no model bound is changed.
            value=max(v.getLbOriginal(),min(v.getUbOriginal(),float(initial)))
            self.m.setSolVal(self.seed_solution,v,value)
        return v

    def row(self,e,lo=None,hi=None,name='',family='link'):
        if isinstance(e,(int,float,np.number)):
            if (lo is not None and e<lo-1e-9) or (hi is not None and e>hi+1e-9):
                raise ValueError('Inconsistent constant physical row '+name)
            return
        if self.seed_solution is not None:
            value=float(self.m.getSolVal(self.seed_solution,e))
            error=max(0.,lo-value if lo is not None else 0.,value-hi if hi is not None else 0.)
            if error>1e-7:self.seed_residuals.append(dict(name=name,family=family,value=value,lower=lo,upper=hi,error=error))
        if lo is not None and hi is not None and lo==hi:
            self.m.addCons(e==lo,name=name)
        else:
            if lo is not None:self.m.addCons(e>=lo,name=name+'_lo')
            if hi is not None:self.m.addCons(e<=hi,name=name+'_hi')
        # Constraint incidence will be exported from actual SCIP, not a second
        # enormous Python copy of expression trees.
        self.counter+=1

    def aux(self,e,name,lb=-1e15,ub=1e15,group='physical_auxiliary'):
        v=self.var(name,lb,ub,group=group)
        if self.seed_solution is not None:
            value=float(self.m.getSolVal(self.seed_solution,e)) if not isinstance(e,(int,float,np.number)) else float(e)
            self.m.setSolVal(self.seed_solution,v,value)
            if value<lb-1e-7 or value>ub+1e-7:self.seed_residuals.append(dict(name=name,family='variable_bound',value=value,lower=lb,upper=ub))
        self.row(v-e,0,0,name+'_def','definition');return v

    def quotient_aux(self,numerator,denominator,name,lb,ub):
        """Exact positive-denominator lifting; do not form reciprocal trees.

        Denominators below are sqrt(sum(square)+1e-12), hence >=1e-6.
        x*d=n is equivalent to x=n/d throughout their existing domains.
        Original output bounds and independent physical acceptance remain.
        """
        if denominator.getLbOriginal()<=0:
            raise ValueError('Positive denominator proof required for '+name)
        v=self.var(name,lb,ub,group='physical_auxiliary')
        if self.seed_solution is not None:
            value=self.m.getSolVal(self.seed_solution,numerator)/self.m.getSolVal(self.seed_solution,denominator)
            self.m.setSolVal(self.seed_solution,v,value)
            if value<lb-1e-7 or value>ub+1e-7:
                self.seed_residuals.append(dict(name=name,family='variable_bound',value=value,lower=lb,upper=ub))
        self.row(v*denominator-numerator,0,0,name+'_def','definition')
        return v

    def state_aux(self,z,name,field,derivative):
        if field=='C':
            lo,hi=(self.curves.lower,self.curves.upper) if derivative==0 else (
                [-[0,5,3,5][derivative]]*3,[[0,5,3,5][derivative]]*3)
        else:
            lo,hi=(self.curves.pose_lo,self.curves.pose_hi) if derivative==0 else (
                [-x for x in (self.curves.pose_rate if derivative==1 else self.curves.pose_acc)],
                self.curves.pose_rate if derivative==1 else self.curves.pose_acc)
        return [self.aux(x,name+f'_{q}',float(lo[q]),float(hi[q])) for q,x in enumerate(z)]

    def ground(self,world,key):
        # Coordinates supplied to IndexedTerrain are local XY, heights remain
        # the original absolute DEM reference. No local crop is introduced.
        xy=[self.aux(world[q]-self.origin[q],key+f'_local_ground_xy{q}',
                     float(self.curves.lower[q]),float(self.curves.upper[q])) for q in range(2)]
        data=self.terrain_index.add(self.m,self.rel,xy,key+'_terrain')
        if self.seed_solution is not None:
            loc=np.array([self.m.getSolVal(self.seed_solution,v) for v in xy]);worldxy=loc+self.origin[:2]
            k=int(self.terrain_index.terrain.tri.find_simplex(worldxy))
            if k<0:raise ValueError('Seed point outside source DEM')
            self._seed_selector(data['cell'],k)
            cell=self.terrain_index.cells[k];ids=cell['vertices']
            mat=np.vstack([self.terrain_index.xy[ids].T,np.ones(3)])
            weights=np.linalg.solve(mat,np.r_[loc,1.])
            for v,value in zip(data['weights'],weights):self.m.setSolVal(self.seed_solution,v,float(value))
            self.m.setSolVal(self.seed_solution,data['ground'],float(weights@self.terrain_index.z[ids]))
            for v,value in zip(data['slopes'],cell['gradient']):self.m.setSolVal(self.seed_solution,v,float(value))
        return data['ground'],*data['slopes']

    def _seed_selector(self,sel,k):
        self.m.setSolVal(self.seed_solution,sel.value,k)
        for j,b in enumerate(sel.bits):self.m.setSolVal(self.seed_solution,b,(k>>j)&1)

    def and2(self,a,b,name):
        # The old pilot's helper reads its pilot warm-start object. Regional
        # construction has no fixed-route warm start and must not invoke it.
        v=self.binary(name)
        self.row(v-a,hi=0,name=name+'_a');self.row(v-b,hi=0,name=name+'_b')
        self.row(v-a-b,lo=-1,name=name+'_ab')
        return v

    def optics(self,block,s,data,key):
        pixels=super().optics(block,s,data,key)
        # Contract5 specifies every patch sample and the 2D blur norm. The
        # pilot checks its center and per-axis blur; retain those necessary
        # rows and add the missing full contract conditions here only.
        task=self.targets[block['task']];target=task['target'];active=block['active']
        fx0=math.hypot(5184,3888)/(2*math.tan(math.radians(66.6/2)))
        fx=fx0*data['angles'][3]
        rc=[[self.vars[key+f'_cam_{k}_{q}'] for q in range(3)] for k in range(3)]
        rcd=[[self.vars[key+f'_cam_dot_{k}_{q}'] for q in range(3)] for k in range(3)]
        for k,point in enumerate(target['surface_samples_utm_m']):
            delta=[data['position'][q]-point[q] for q in range(3)]
            distance=self.aux(sqrt(dot(delta,delta)+1e-12),key+f'_sample{k}_distance',1e-9,30000)
            self.gate(distance,active,8,25,name=key+f'_sample{k}_focus',family='camera_fov')
            self.gate(dot(target['normal_world'],delta)-math.cos(math.radians(30))*distance,active,lo=0,
                      name=key+f'_sample{k}_incidence',family='camera_fov')
            qc=[self.vars[key+f'_q{k}_{q}'] for q in range(3)];depth=self.vars[key+f'_depth_safe{k}']
            qcd=[dot(cold,[-x for x in delta])-dot(col,data['velocity']) for col,cold in zip(rc,rcd)]
            rates=[.001*(fx0*data['rates'][3]*qc[q]/depth+fx*(qcd[q]*depth-qc[q]*qcd[2])/depth**2) for q in range(2)]
            self.gate(dot(rates,rates),active,hi=1,name=key+f'_sample{k}_blur_norm',family='motion_blur')
        return pixels

    def view_region(self,segment,data,key):
        """Whole-service geometric region; image constraints are exposure-only."""
        target=self.targets[segment.task]['target'];active=segment.active
        for k,point in enumerate(target['surface_samples_utm_m']):
            delta=[data['position'][q]-point[q] for q in range(3)]
            dist=self.aux(sqrt(dot(delta,delta)+1e-12),key+f'_view_distance{k}',1e-9,30000)
            self.gate(dist,active,8,25,name=key+f'_view_focus{k}',family='view_region')
            self.gate(dot(target['normal_world'],delta)-math.cos(math.radians(30))*dist,
                active,lo=0,name=key+f'_view_incidence{k}',family='view_region')

    def physics_segment(self,segment,s,key):
        active=segment.active;cfg=self.cfg;air=cfg['aircraft'];energy=cfg['energy']
        bound=self.auxiliary_bounds
        p,v,a,jerk=[self.state_aux(x,key+f'_state{r}','C',r) for r,x in enumerate(segment.evaluate(s,'C',3))]
        angles,rates,accels=[self.state_aux(x,key+f'_pose{r}','A',r) for r,x in enumerate(segment.evaluate(s,'A',2))]
        world=[p[q]+self.origin[q] for q in range(3)]
        ground,gx,gy=self.ground(world,key)
        agl=self.aux(world[2]-ground,key+'_agl',0,100)
        clock=self.aux(segment.clock+s*segment.duration,key+'_absolute_clock',0,self.path.H)
        W=self.forecast.add_point(self.m,self.rel,clock,agl,active,key)
        if self.seed_solution is not None:
            tt=float(self.m.getSolVal(self.seed_solution,clock));hh=float(self.m.getSolVal(self.seed_solution,agl))
            h,k=self.forecast.cell(tt,hh);lo,hi,*coeff=self.forecast.coefficients(h,k)
            self._seed_selector(W['selector'],h*3+k)
            for seedvar,value in zip(W['coefficient_variables']['origins'],[self.forecast.times[h],lo]):self.m.setSolVal(self.seed_solution,seedvar,float(value))
            for vr,cr in zip(W['coefficient_variables']['coefficients'],coeff):
                for seedvar,value in zip(vr,cr):self.m.setSolVal(self.seed_solution,seedvar,float(value))
        rho=self.aux(W['density_kg_m3'],key+'_density',max(.5,bound['weather_min'][2]-1e-8),min(2,bound['weather_max'][2]+1e-8))
        wind=[self.aux(W[f],key+'_'+f,max(-30,bound['weather_min'][q]-1e-8),min(30,bound['weather_max'][q]+1e-8))
              for q,f in enumerate(['wind_east_ms','wind_north_ms'])]+[0.]
        self.gate(dot(wind,wind),active,hi=100,name=key+'_wind_limit',family='weather')
        self.gate(W['temperature_C'],active,0,40,name=key+'_temperature',family='weather')
        task=self.curves.owner(segment)
        # Only the first piece of a root task and the last recovery piece may
        # use the own-pad contact ramp. Other terrain/obstacles keep4.6m.
        contact=None;is_launch=False
        if segment.role=='in' and segment.index==0:contact=task.root;is_launch=True
        elif segment.role=='recovery' and segment.index==2*self.curves.transfers:contact=task.last
        margin=4.6
        if contact is not None:
            dock=self.var(key+'_dock_ground',float(self.terrain_index.z.min()),float(self.terrain_index.z.max()))
            if self.seed_solution is not None:
                flag=float(contact) if isinstance(contact,(int,float,np.number)) else float(self.m.getSolVal(self.seed_solution,contact))
                if flag>.5:
                    k=int(round(self.m.getSolVal(self.seed_solution,task.flight.value)))
                    if not 1<=k<=len(self.rendezvous.anchors):raise ValueError('Active seed contact has no flight identity')
                    anchor=self.rendezvous.anchors[k-1]
                    g=anchor.launch_ground if is_launch else anchor.land_ground
                    self.m.setSolVal(self.seed_solution,dock,float(self.m.getSolVal(self.seed_solution,g)))
            def dock_rows(k):
                if not k:return [AffineRow('nonzero_contact_flight',[],lo=1)]
                anchor=self.rendezvous.anchors[k-1]
                g=anchor.launch_ground if is_launch else anchor.land_ground
                return [AffineRow('contact_ground',[(1,dock),(-1,g)],0,0)]
            self.rel.add(task.flight,dock_rows,[dock],contact,key+'_dock')
            ramp=.6+4*minimum(1,maximum(0,(world[2]-dock-2.4)/4))
            margin=4.6+contact*(ramp-4.6)
        self.gate(agl-margin,active,lo=0,name=key+'_terrain_clearance',family='safety')
        va=[self.aux(v[q]-wind[q],key+f'_air_velocity_{q}',-min(40,bound['air_velocity_abs'][q]),min(40,bound['air_velocity_abs'][q])) for q in range(3)]
        speed=self.aux(sqrt(dot(va,va)+1e-12),key+'_airspeed',1e-6,min(80,bound['airspeed_upper']))
        force=[self.aux(air['mass_kg']*(a[q]+(9.81 if q==2 else 0))+
            .5*air['drag_CdA_m2']*rho*speed*va[q],key+f'_force_{q}',-min(2000,bound['force_abs'][q]),min(2000,bound['force_abs'][q])) for q in range(3)]
        norm=self.aux(sqrt(dot(force,force)+1e-12),key+'_force_norm',bound['force_norm_lower'],min(4000,bound['force_norm_upper']))
        b3=[self.quotient_aux(x,norm,key+f'_body_z_{q}',-1,1) for q,x in enumerate(force)]
        cs,sn=cos(angles[0]),sin(angles[0])
        axis=[b3[2]*cs,b3[2]*sn,-b3[0]*cs-b3[1]*sn]
        n2=self.aux(sqrt(dot(axis,axis)+1e-12),key+'_heading_norm',bound['heading_norm_lower'],2)
        b1=[self.quotient_aux(x,n2,key+f'_body_x_{q}',-1,1) for q,x in enumerate(axis)]
        b2=self.avec(cross(b3,b1),key+'_body_y',-1,1)
        # Full material derivative: explicit forecast time change PLUS local
        # height change. Spatial gradient is zero only for the declared uniform
        # single-cell reference, not for an unimplemented multi-cell field.
        dh=v[2]-gx*v[0]-gy*v[1]
        winddot=[W['dt'][f]+W['dz'][f]*dh for f in ['wind_east_ms','wind_north_ms']]+[0.]
        rhodot=W['dt']['density_kg_m3']+W['dz']['density_kg_m3']*dh
        vad=[self.aux(a[q]-winddot[q],key+f'_air_velocity_dot_{q}',-min(1e15,bound['air_velocity_dot_abs'][q]),min(1e15,bound['air_velocity_dot_abs'][q])) for q in range(3)]
        # Cauchy: |va.vad|/sqrt(va.va+eps) <= ||vad|| <= sqrt(3)*1e15.
        speed_rate=min(math.sqrt(3)*1e15,bound['airspeed_dot_abs'])
        speeddot=self.quotient_aux(dot(va,vad),speed,key+'_airspeed_dot',-speed_rate,speed_rate)
        fd=[self.aux(air['mass_kg']*jerk[q]+.5*air['drag_CdA_m2']*(
            rhodot*speed*va[q]+rho*speeddot*va[q]+rho*speed*vad[q]),key+f'_force_dot_{q}',-min(1e18,bound['force_dot_abs'][q]),min(1e18,bound['force_dot_abs'][q])) for q in range(3)]
        bz=min(1e18,bound['body_z_dot_abs']);bx=min(1e18,bound['body_x_dot_abs']);by=min(1e18,bound['body_y_dot_abs'])
        b3d=[self.quotient_aux(fd[q]-b3[q]*dot(b3,fd),norm,key+f'_body_z_dot_{q}',-bz,bz) for q in range(3)]
        cd,sd=-sn*rates[0],cs*rates[0]
        axisdot=[b3d[2]*cs+b3[2]*cd,b3d[2]*sn+b3[2]*sd,
            -b3d[0]*cs-b3[0]*cd-b3d[1]*sn-b3[1]*sd]
        b1d=[self.quotient_aux(axisdot[q]-b1[q]*dot(b1,axisdot),n2,key+f'_body_x_dot_{q}',-bx,bx) for q in range(3)]
        b2d=self.avec(plus(cross(b3d,b1),cross(b3,b1d)),key+'_body_y_dot',-by,by)
        omega=[dot(b3,b2d),dot(b1,b3d),dot(b2,b1d)]
        for q,x in enumerate(omega):self.gate(x,active,-math.radians(30),math.radians(30),name=key+f'_body_rate{q}',family='attitude')
        for vector,limit,name in [(v,5,'ground_speed'),(va,12,'air_speed'),(a,3,'acceleration'),(jerk,5,'jerk')]:
            self.gate(dot(vector,vector),active,hi=limit**2,name=key+'_'+name,family='kinematics')
        self.gate(v[2],active,-1,1,name=key+'_vertical_speed',family='kinematics')
        self.gate(norm,active,0,137.34,name=key+'_thrust',family='attitude')
        self.gate(force[2]-math.cos(math.radians(25))*norm,active,lo=0,name=key+'_tilt',family='attitude')
        for q in range(3):
            self.gate(rates[q],active,-math.radians(30),math.radians(30),name=key+f'_rate{q}',family='attitude')
            self.gate(accels[q],active,-math.radians(30),math.radians(30),name=key+f'_angular_acc{q}',family='attitude')
        self.gate(rates[3]*6.83,active,-10,10,name=key+'_zoom_rate',family='camera')
        power=self.aux(energy['hover_power_W']*sqrt(1.225/rho)+.5*air['drag_CdA_m2']*rho*speed**3+
            air['mass_kg']*9.81*maximum(v[2],0)/energy['propulsion_efficiency'],key+'_power',0,200000)
        result=dict(position=world,velocity=v,acceleration=a,jerk=jerk,angles=angles,rates=rates,
            ground=ground,rotation=[b1,b2,b3],rotation_dot=[b1d,b2d,b3d],power=power,agl=agl,
            density=rho,wind=wind,clock=clock,active=active)
        self.points[key]=result
        if segment.role=='capture':
            self.view_region(segment,result,key)
        return result

    def materialize_segment(self,segment,order=5):
        key=(segment.task,segment.index,order)
        if key in self.materialized_segments:return self.energy[key]
        nodes,weights=np.polynomial.legendre.leggauss(order);nodes=(nodes+1)/2;weights=weights/2
        terms=[]
        for k,(s,w) in enumerate(zip(nodes,weights)):
            data=self.physics_segment(segment,float(s),f'physical_task{segment.task+1}_seg{segment.index}_q{order}_{k}')
            terms.append(float(w)*segment.duration*data['power']/3600)
        self.energy[key]=self.aux(quicksum(terms),f'physical_task{segment.task+1}_seg{segment.index}_q{order}_Wh',
            0,548-109.6,'aircraft_energy_quadrature')
        if self.energy_registered:
            identity=(segment.task,segment.index)
            if identity in self.energy_links:self.m.delCons(self.energy_links[identity])
            self.energy_links[identity]=self.m.addCons(self.segment_energy[identity]==self.energy[key],
                     name=f'energy_integral_link_task{segment.task+1}_seg{segment.index}_q{order}')
            self.current_orders[identity]=order
        self.materialized_segments.add(key)
        self.rel.lock(self.m.getVars())
        return self.energy[key]

    def add_point(self,segment,s):
        signature=(segment.task,segment.index,s)
        if signature in self.extra_points:return False
        self.physics_segment(segment,s,f'physics_refine_{len(self.extra_points)}')
        self.extra_points.add(signature);self.rel.lock(self.m.getVars());return True

    def add_exposure_point(self,segment,fraction):
        if segment.role!='capture' or not 0<=fraction<=1:raise ValueError('Capture exposure fraction required')
        identity=(segment.task,segment.index,float(fraction))
        if identity in self.exposure_points:return False
        self.m.addCons(segment.duration>=1.001*segment.active)
        # The abscissa follows the optimized duration; no frozen incumbent
        # timestamp is imposed on another trajectory.
        s=(1.+.001*fraction)*segment.active/segment.safe_duration
        key=f'exposure_task{segment.task+1}_{len(self.exposure_points)}'
        data=self.physics_segment(segment,s,key)
        self.optics(dict(task=segment.task,active=segment.active),s,data,key)
        self.exposure_points.add(identity);self.rel.lock(self.m.getVars());return True

    def register_energy(self,resources):
        if self.energy_registered:raise ValueError('Energy variables already registered')
        for s in self.curves.segments:
            v=self.var(f'segment_task{s.task+1}_index{s.index}_Wh',0,438.4,group='aircraft_energy')
            self.row(v-438.4*s.active,hi=0,name=v.name+'_active',family='energy')
            self.segment_energy[s.task,s.index]=v
        tasks=[];recovery=[]
        for d in self.curves.by_task:
            tasks.append(quicksum(self.segment_energy[s.task,s.index] for s in d['segments'] if s.role!='recovery'))
            recovery.append(quicksum(self.segment_energy[s.task,s.index] for s in d['segments'] if s.role=='recovery'))
        flight_recovery=None
        if self.curves.shared_recovery:
            flight_recovery=[quicksum(self.segment_energy[s.task,s.index] for s in d['segments']) for d in self.curves.recoveries]
        resources.attach_task_energy(tasks,recovery,flight_recovery)
        self.energy_registered=True
        return self

    def status(self):
        return dict(materialized_segments=len(self.materialized_segments),physical_points=len(self.points),
            energy_path_registered=self.energy_registered,
            every_segment_integral_materialized=len(self.current_orders)==len(self.curves.segments),
            exact_projection_and_surface_jacobian=True,weather_time_derivative_included=True,
            body_attitude_derived_from_force=True,quadrature_is_rigorous_integral_bound=False,
            continuous_certificate=False,all_obstacle_and_intervehicle_constraints_attached=False,
            full_joint_model=False)
