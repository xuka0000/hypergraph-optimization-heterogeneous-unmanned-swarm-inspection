"""Coupled free position/pose/zoom/time nonlinear refinement.

The outer resource master owns discrete choices. This local step retains that
choice and a trust region, but does not freeze observation positions, capture
motion, trajectory controls, or camera pose. Independent replay is mandatory.
No global MINLP bound or measured image-quality claim is produced here.
"""
import math,time
import numpy as np
from Agent.Publication.engineering_free_trajectory import load_solver,bernstein


def basis(x,n=7):
    return np.array([math.comb(n,i)*x**i*(1-x)**(n-i) for i in range(n+1)])


def initial_pose_controls(segment,keys):
    samples=[]
    for u in np.linspace(0,1,8):
        t=segment['start_s']+u*(segment['end_s']-segment['start_s'])
        k=min(len(keys)-2,max(0,np.searchsorted(keys[:,0],t,side='right')-1))
        a,b=keys[k:k+2];s=np.clip((t-a[0])/(b[0]-a[0]),0,1)
        smooth=35*s**4-84*s**5+70*s**6-20*s**7
        samples.append(np.r_[a[1:]+smooth*(b[1:]-a[1:]),1.])
    return np.linalg.solve(np.array([basis(x) for x in np.linspace(0,1,8)]),samples).T


def _equality(op,expression,ca):
    # All position/pose controls are independent, so only literal zero rows
    # can be removed. This is not numerical rank elimination.
    for row in ca.vertsplit(expression):
        if not row.is_zero():op.subject_to(row==0)


def optimize(candidates,poses,scene,terrain,weather,soc=1.,trust_m=.75,max_iter=500,hessian='exact',checkpoint=None,initial_scale=1.03):
    ca=load_solver();cfg=scene['joint_reference_profile'];air=cfg['aircraft'];cam=cfg['camera'];en=cfg['energy']
    from Agent.Publication.engineering_joint_execution import Geometry
    geo=Geometry(scene);op=ca.Opti();origin=np.array(candidates[0]['timeline'][0]['from_xyz_m'])
    wire_a=np.concatenate([z[0][:-1] for z in geo.wires]);wire_b=np.concatenate([z[0][1:] for z in geo.wires]);wire_radius=np.concatenate([np.full(len(z[0])-1,z[1]) for z in geo.wires]);wire_d=wire_b-wire_a;wire_den=np.sum(wire_d**2,axis=1)
    wire_planes=0;wire_checks=0
    clearance=sum(air[k] for k in ['body_radius_m','obstacle_clearance_m','tracking_error_m','map_error_m'])
    fx0=math.hypot(cam['image_width_px'],cam['image_height_px'])/(2*math.tan(math.radians(cam['diagonal_fov_deg_at_wide']/2)))
    nodes,weights=np.polynomial.legendre.leggauss(12);all_blocks=[];all_energies=[];launches=[];ends=[];reg=0.
    reference_reserve=en['minimum_SOC']*en['pair_capacity_Wh'];pre=cfg['recovery']['preparation_seconds'];post=cfg['recovery']['recovery_seconds']
    maximum=op.variable();op.set_initial(maximum,max(d['source_sortie']['land_s']+post+(initial_scale-1)*d['flight_seconds'] for d in candidates)+(750 if soc<.5 else 0))
    op.subject_to(op.bounded(0,maximum,3500.))
    for flight_index,(d,pose) in enumerate(zip(candidates,poses)):
        blocks=[];energy=0.;keys=np.asarray(pose['pose_keys_s_yaw_gp_gy']);targets={t['id']:t for t in d['targets']}
        capture_index=0;contact_anchor=np.array(d['timeline'][0]['from_xyz_m'])
        for k,seg in enumerate(d['timeline']):
            init=np.array(seg['control_points_m']);t0=seg['end_s']-seg['start_s'];a0=initial_pose_controls(seg,keys)
            c=op.variable(3,8);a=op.variable(4,8);scale=op.variable();T=t0*scale
            op.set_initial(c,(init-origin).T);op.set_initial(a,a0);op.set_initial(scale,initial_scale)
            # Trust regions are local search steps, not a fixed flight catalogue.
            op.subject_to(op.bounded((init-origin).T-trust_m,c,(init-origin).T+trust_m))
            op.subject_to(op.bounded(.65,scale,1.8));reg+=ca.sumsqr(c-(init-origin).T)+ca.sumsqr(a-a0)
            op.subject_to(op.bounded(math.radians(cam['gimbal_pitch_deg'][0]),a[1,:],math.radians(cam['gimbal_pitch_deg'][1])))
            op.subject_to(op.bounded(math.radians(cam['gimbal_yaw_deg'][0]),a[2,:],math.radians(cam['gimbal_yaw_deg'][1])))
            op.subject_to(op.bounded(1.,a[3,:],cam['focal_length_mm_bounds'][1]/6.83))
            # The exact fixed exposure bounds in this reference are 1ms/1ms.
            # There is no calibrated illumination law allowing shorter exposure.
            capture=None
            if seg['kind']=='capture':
                capture=d['captures'][capture_index];capture_index+=1;task=targets[capture['task_id']]
                op.subject_to(T>=task['observation_requirement']['min_dwell_s'])
                center=np.array([task[z] for z in ['easting_m','northing_m','altitude_m']])
                for j in range(8):op.subject_to(ca.sumsqr(c[:,j]+origin-center)<=task['observation_requirement']['viewpoint_ball_radius_m']**2)
            contact=seg.get('departure_contact',False) or seg.get('arrival_contact',False)
            if contact:
                # Vertical initial/terminal contact column remains a local
                # initializer; endpoints retain the actual stationary pad.
                op.subject_to(c[:2,:]==np.repeat((contact_anchor[:2]-origin[:2])[:,None],8,axis=1))
            tau=ca.MX.sym('tau');p,v,acc,jerk=bernstein(c,T,tau);angles,angle_rate,angle_acc,_=bernstein(a,T,tau);world=p+origin
            ref=init.mean(axis=0);g0=float(terrain(*ref[:2]));eps=.1
            grad=np.array([(float(terrain(*(ref[:2]+np.eye(2)[q]*eps)))-float(terrain(*(ref[:2]-np.eye(2)[q]*eps))))/(2*eps) for q in range(2)])
            ground=g0+ca.dot(grad,world[:2]-ref[:2]);agl=world[2]-ground
            wz=[];rz=[]
            for h in (30.,60.,99.999999):
                w,rho=weather(np.r_[ref[:2],g0+h],0.,g0);wz.append(ca.DM(w));rz.append(float(rho))
            low=ca.fmin(1.,ca.fmax(0.,(agl-30)/30));high=ca.fmin(1.,ca.fmax(0.,(agl-60)/39.999999))
            wind=ca.if_else(agl<=60,wz[0]+low*(wz[1]-wz[0]),wz[1]+high*(wz[2]-wz[1]))
            rho=ca.if_else(agl<=60,rz[0]+low*(rz[1]-rz[0]),rz[1]+high*(rz[2]-rz[1]))
            va=v-wind;force=air['mass_kg']*(acc+ca.DM([0,0,9.81]))+.5*rho*air['drag_CdA_m2']*ca.sqrt(ca.sumsqr(va)+1e-12)*va
            b3=force/ca.sqrt(ca.sumsqr(force));b2=ca.cross(b3,ca.vertcat(ca.cos(angles[0]),ca.sin(angles[0]),0));b2=b2/ca.sqrt(ca.sumsqr(b2));rb=ca.horzcat(ca.cross(b2,b3),b2,b3)
            drb=ca.reshape(ca.jacobian(ca.reshape(rb,9,1),tau),3,3)/T;omega_matrix=rb.T@drb;omega=ca.vertcat(omega_matrix[2,1],omega_matrix[0,2],omega_matrix[1,0])
            gp,gy=angles[1],angles[2];optical=ca.vertcat(ca.cos(gp)*ca.cos(gy),ca.cos(gp)*ca.sin(gy),ca.sin(gp));right=ca.vertcat(ca.sin(gy),-ca.cos(gy),0);rc=rb@ca.horzcat(right,ca.cross(optical,right),optical)
            power=en['hover_power_W']*ca.sqrt(1.225/rho)+.5*air['drag_CdA_m2']*rho*(ca.sumsqr(va)+1e-12)**1.5+air['mass_kg']*9.81*ca.fmax(v[2],0)/en['propulsion_efficiency']
            # Keep each physical segment as an atomic AD function. Inlining
            # thousands of substitutions made the global Jacobian exceed4GiB.
            derivative_options={'max_num_dir':1}
            physics_name=f'physics_{flight_index}_{k}'
            physics_fn=ca.Function(physics_name,[c,a,scale,tau],[v,acc,jerk,force,va,omega,agl,angle_rate,angle_acc,world,power],derivative_options).expand(physics_name+'_sx',derivative_options)
            for u in np.linspace(0,1,13):
                vv,aa,jj,ff,vvair,om,alt,rates,accels,wpos,pow_u=physics_fn(c,a,scale,float(u))
                op.subject_to(ca.sumsqr(vv)<=(air['ground_speed_max_mps']*.985)**2)
                op.subject_to(op.bounded(-air['vertical_speed_max_mps']*.985,vv[2],air['vertical_speed_max_mps']*.985))
                op.subject_to(ca.sumsqr(vvair)<=(air['air_speed_max_mps']*.985)**2)
                op.subject_to(ca.sumsqr(aa)<=(air['acceleration_max_mps2']*.98)**2);op.subject_to(ca.sumsqr(jj)<=(air['jerk_max_mps3']*.98)**2)
                op.subject_to(ca.sumsqr(ff)<=(air['thrust_max_N']*.99)**2);op.subject_to(ff[2]>=math.cos(math.radians(air['tilt_max_deg']-.1))*ca.sqrt(ca.sumsqr(ff)))
                op.subject_to(op.bounded(-math.radians(29.),om,math.radians(29.)))
                op.subject_to(alt>=(.61 if contact else clearance)+.002);op.subject_to(alt<=99.9)
                op.subject_to(op.bounded(-math.radians(29.),rates[:3],math.radians(29.)))
                op.subject_to(op.bounded(-math.radians(29.),accels[:3],math.radians(29.)))
                op.subject_to(op.bounded(-cam['zoom_rate_max_mm_s']*.99/6.83,rates[3],cam['zoom_rate_max_mm_s']*.99/6.83))
            # Nearby source obstacles constrain the actual 3D trajectory at
            # collocation. Whole-curve acceptance is a separate operation.
            mid=init.mean(axis=0);rad=np.linalg.norm(init-mid,axis=1).max()+trust_m*2+clearance
            cyls=[geo.cyl[i] for i in geo.tree.query_ball_point(mid[:2],rad+3)]
            for u in np.linspace(0,1,13):
                wpos=physics_fn(c,a,scale,float(u))[9]
                for obstacle in cyls:
                    radial=ca.sqrt(ca.sumsqr(wpos[:2]-obstacle['center_xy_m'])+1e-12)-obstacle['radius_m']
                    vertical=ca.fmax(obstacle['base_elevation_m']-wpos[2],wpos[2]-obstacle['base_elevation_m']-obstacle['height_m'])
                    distance2=ca.fmax(radial,0)**2+ca.fmax(vertical,0)**2
                    op.subject_to(distance2>=(clearance+.01)**2)
            # All source conductor chords are inflated by their source
            # parabola approximation error in Geometry. Trust-region pruning
            # cannot omit a reachable chord at these collocation times;
            # whole-curve acceptance still uses the independent certificate.
            nearby=set()
            for u in np.linspace(0,1,13):
                initial_point=basis(float(u))@init;fraction=np.clip(np.sum((initial_point-wire_a)*wire_d,axis=1)/wire_den,0,1)
                distances=np.linalg.norm(initial_point-wire_a-fraction[:,None]*wire_d,axis=1)
                indices=np.flatnonzero(distances<=wire_radius+clearance+trust_m*math.sqrt(3)+.02);nearby.update(indices.tolist());wpos=physics_fn(c,a,scale,float(u))[9]
                for wi in indices:
                    frac=ca.fmin(1,ca.fmax(0,ca.dot(wpos-wire_a[wi],wire_d[wi])/wire_den[wi]));delta=wpos-wire_a[wi]-frac*wire_d[wi]
                    op.subject_to(ca.sumsqr(delta)>=(clearance+wire_radius[wi]+.01)**2);wire_checks+=1
            # Where the initializer supports a separating plane, constrain
            # every control point for an all-time local collision certificate.
            for wi in nearby:
                center=init.mean(axis=0);fraction=np.clip(np.dot(center-wire_a[wi],wire_d[wi])/wire_den[wi],0,1);normal=center-wire_a[wi]-fraction*wire_d[wi];normal/=np.linalg.norm(normal)
                bound=max(np.dot(wire_a[wi],normal),np.dot(wire_b[wi],normal))+clearance+wire_radius[wi]
                if np.min(init@normal)>=bound+.006:
                    op.subject_to(ca.DM(normal).T@c>=bound+.005-float(normal@origin));wire_planes+=1
            if capture:
                target=task['target'];q=np.array([target[z] for z in ['easting_m','northing_m','altitude_m']]);normal=ca.DM(target['normal_world']);tan=ca.DM(np.array(target['tangent_basis']).T)
                delta=world-q;distance=ca.sqrt(ca.sumsqr(delta))
                # Enforce the optics through the whole captured dwell, including
                # movement, force-induced body tilt, zoom and image derivatives.
                for point_index,point in enumerate(target['surface_samples_utm_m']):
                    qc=rc.T@(ca.DM(point)-world);fx=fx0*angles[3]
                    pixel=ca.vertcat(fx*qc[0]/qc[2]+cam['image_width_px']/2,fx*qc[1]/qc[2]+cam['image_height_px']/2)
                    pixel_rate=ca.jacobian(pixel,tau)/T
                    jproj=ca.vertcat(ca.horzcat(fx/qc[2],0,-fx*qc[0]/qc[2]**2),ca.horzcat(0,fx/qc[2],-fx*qc[1]/qc[2]**2))@rc.T@tan
                    metric=(jproj.T@jproj)*cam['gsd_max_m_per_px']**2-ca.DM.eye(2)*1.002
                    optic_name=f'optics_{flight_index}_{k}_{point_index}'
                    optic_fn=ca.Function(optic_name,[c,a,scale,tau],[pixel,metric,distance,qc[2],pixel_rate,ca.dot(normal,delta)],derivative_options).expand(optic_name+'_sx',derivative_options)
                    for u in np.linspace(0,1,9):
                        px,mm,dist,depth,prate,incidence=optic_fn(c,a,scale,float(u))
                        op.subject_to(depth>=.1)
                        op.subject_to(op.bounded(cam['frame_border_px']+1,px[0],cam['image_width_px']-cam['frame_border_px']-1))
                        op.subject_to(op.bounded(cam['frame_border_px']+1,px[1],cam['image_height_px']-cam['frame_border_px']-1))
                        op.subject_to(mm[0,0]>=0);op.subject_to(mm[1,1]>=0);op.subject_to(mm[0,0]*mm[1,1]-mm[0,1]*mm[1,0]>=0)
                        op.subject_to(op.bounded(-.98,cam['exposure_bounds_s'][0]*prate,.98))
                        op.subject_to(dist>=cam['focus_min_m']+.002);op.subject_to(dist<=cam['observation_max_m']-.002)
                        op.subject_to(incidence>=math.cos(math.radians(cam['incidence_angle_max_deg']-.05))*dist)
            for u,w in zip((nodes+1)/2,weights/2):energy+=w*T*physics_fn(c,a,scale,float(u))[10]/3600
            blocks.append(dict(C=c,A=a,T=T,source=seg,capture=capture))
        for before,after in zip(blocks,blocks[1:]):
            for left,right in zip(bernstein(before['C'],before['T'],1.),bernstein(after['C'],after['T'],0.)):_equality(op,left-right,ca)
            for left,right in zip(bernstein(before['A'],before['T'],1.)[:3],bernstein(after['A'],after['T'],0.)[:3]):_equality(op,left-right,ca)
        for k,u in [(0,0.),(-1,1.)]:
            block=blocks[k];state=bernstein(block['C'],block['T'],u);boundary=np.array(d['timeline'][k]['control_points_m'][0 if k==0 else -1])-origin
            _equality(op,state[0]-boundary,ca)
            for expr in state[1:]:_equality(op,expr,ca)
            ang=bernstein(block['A'],block['T'],u)
            initial=keys[0 if k==0 else -1,1:]
            _equality(op,ang[0][:3]-initial,ca)
            for expr in ang[1:3]:_equality(op,expr,ca)
        delay=op.variable();op.subject_to(op.bounded(0,delay,100.));op.set_initial(delay,0.)
        # Both source pairs/chargers remain independent because this candidate
        # uses one original aircraft per carrier. The outer master handles
        # additional sortie/pair reuse; this local step does not invent a pack.
        required=energy+reference_reserve
        charge=ca.fmax(0.,required-soc*en['pair_capacity_Wh'])
        op.subject_to(required<=.9*en['pair_capacity_Wh'])
        charge_time=charge*1800/(.7*en['pair_capacity_Wh'])
        launch=pre+charge_time+delay;end=launch+sum(b['T'] for b in blocks)+post
        op.subject_to(maximum>=end);op.subject_to(end<=3500.)
        all_blocks.append(blocks);all_energies.append(energy);launches.append(launch);ends.append(end)
    # A tiny regularizer resolves flat nuisance directions; reported objectives
    # are replayed makespan and energy, not this numerical regularizer.
    op.minimize(maximum+1e-7*reg)
    print('NLP dimensions',op.nx,op.ng,hessian,'Hessian; local SX functions',flush=True)
    stage_name='makespan'
    def iterate(iteration):
        if iteration%20:return
        g=np.asarray(op.debug.value(op.g)).ravel();lo=np.asarray(op.debug.value(op.lbg)).ravel();hi=np.asarray(op.debug.value(op.ubg)).ravel();violation=float(np.maximum(0,np.maximum(lo-g,g-hi)).max())
        row=dict(iteration=int(iteration),stage=stage_name,variables=int(op.nx),constraints=int(op.ng),objective=float(op.debug.value(op.f)),unscaled_max_constraint_violation=violation,status='solver_iterate_not_a_validated_result',x=np.asarray(op.debug.value(op.x)).ravel().tolist())
        print('NLP iteration',iteration,'objective',row['objective'],'constraint',violation,flush=True)
        if checkpoint:checkpoint(row)
    op.callback(iterate)
    op.solver('ipopt',{'expand':False,'max_num_dir':1},{'print_level':3,'max_iter':max_iter,'tol':1e-6,'acceptable_tol':1e-5,'sb':'yes','max_cpu_time':900.,'hessian_approximation':hessian})
    begin=time.perf_counter();stages=[];result=None;converged=False
    class FrozenSolution:
        def __init__(self,x):self.x=np.asarray(x).ravel()
        def value(self,expression):return np.asarray(ca.Function('frozen_value',[op.x],[expression])(self.x)).squeeze()
    try:
        raw=op.solve();result=FrozenSolution(raw.value(op.x));del raw;converged=True;stages.append(dict(stage='makespan',status=op.stats()['return_status'],seconds=time.perf_counter()-begin,objective_s=float(result.value(maximum))))
        if checkpoint:
            g=np.asarray(result.value(op.g)).ravel();lo=np.asarray(result.value(op.lbg)).ravel();hi=np.asarray(result.value(op.ubg)).ravel();violation=float(np.maximum(0,np.maximum(lo-g,g-hi)).max())
            checkpoint(dict(iteration=op.stats().get('iter_count'),stage='makespan_converged',variables=int(op.nx),constraints=int(op.ng),objective=float(result.value(maximum)),unscaled_max_constraint_violation=violation,status='NLP_converged_requires_physical_replay',x=result.x.tolist()))
        cap=float(result.value(maximum))+.001;initial_x=result.x.copy();op.subject_to(maximum<=cap);op.set_initial(op.x,initial_x);stage_name='energy'
        op.minimize(sum(all_energies)+1e-8*reg)
        energy_begin=time.perf_counter();raw=op.solve();result=FrozenSolution(raw.value(op.x));del raw;stages.append(dict(stage='energy',status=op.stats()['return_status'],seconds=time.perf_counter()-energy_begin,objective_Wh=float(result.value(sum(all_energies)))))
    except RuntimeError as exc:
        try:solver_status=op.stats().get('return_status','solver_failure')
        except RuntimeError:solver_status='construction_failure'
        stages.append(dict(stage='failed',status=solver_status,error=str(exc)[-1800:]))
        # Do not export a failed iterate as a solved plan. A valid previous
        # makespan-stage solution may still be exported for independent replay.
        if result is None and solver_status!='construction_failure':
            try:
                candidate_x=np.asarray(op.debug.value(op.x)).ravel()
                if np.isfinite(candidate_x).all():result=FrozenSolution(candidate_x)
            except RuntimeError:pass
    receipt=dict(success=converged,candidate_available=result is not None,stages=stages,seconds=time.perf_counter()-begin,variables=op.nx,constraints=op.ng,initial_SOC=soc,trust_m=trust_m,hessian=hessian,initial_time_scale=initial_scale,source_wire_collocation_constraints=wire_checks,source_wire_separating_planes=wire_planes,
        free_blocks=['position_controls','segment_times','moving_capture_position','body_yaw','gimbal_pitch_yaw','focal_curve','charge_amount','launch_delay'],
        fixed_outer_choices='current task order, aircraft and battery identities, stationary original carriers',
        exposure_s=.001,exposure_source='original profile equal lower/upper bounds',global_gap=None,independent_replay_pending=True)
    if result is None:return None,receipt
    exports=[]
    for d,blocks,energy,launch,end in zip(candidates,all_blocks,all_energies,launches,ends):
        timeline=[];captures=[];clock=0.
        for b in blocks:
            dt=float(result.value(b['T']));cp=np.asarray(result.value(b['C'])).T+origin;ap=np.asarray(result.value(b['A'])).T
            row=dict(b['source'],start_s=clock,end_s=clock+dt,control_points_m=cp.tolist(),from_xyz_m=cp[0].tolist(),to_xyz_m=cp[-1].tolist(),pose_controls_yaw_gp_gy_focal_ratio=ap.tolist())
            if b['capture']:
                capture=dict(b['capture'],start_s=clock,end_s=clock+dt,position_xyz_m=(basis(.5)@cp).tolist(),moving_capture=True)
                row['task_id']=capture['task_id'];captures.append(capture)
            timeline.append(row);clock+=dt
        exports.append(dict(d,timeline=timeline,captures=captures,flight_seconds=clock,aircraft_energy_Wh=float(result.value(energy)),
            optimized_launch_s=float(result.value(launch)),optimized_recovery_end_s=float(result.value(end)),initial_SOC=soc,
            status='nonlinear_candidate_requires_independent_replay' if converged else 'failed_solver_iterate_not_accepted',full_optical_trajectory_coupling=True))
    return exports,receipt
