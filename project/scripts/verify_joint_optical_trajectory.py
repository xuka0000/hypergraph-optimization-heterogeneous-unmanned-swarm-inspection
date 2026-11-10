"""Independent continuous enclosure replay for a coupled optics candidate.

Uses interval operations and exported Bernstein controls, never NLP residuals
as acceptance evidence. All failed/unresolved checks remain in the receipt.
"""
from run_engineering_full_joint import *
from Agent.Publication.engineering_interval_physics import I,hull,vector,dot,norm,cross,ForceEnclosure
from Agent.Publication.engineering_free_trajectory import split_controls
from verify_free_trajectory_block import geometry_certificate
from verify_engineering_joint_pilot import solid_cylinder_intersection
from Agent.Publication.engineering_joint_execution import closest_segments
from check_joint_airspace import crop,separate
import math

def split_any(c):
    rows=[np.asarray(c,float)]
    for _ in range(len(c)-1):rows.append((rows[-1][:-1]+rows[-1][1:])/2)
    return np.array([r[0] for r in rows]),np.array([r[-1] for r in rows[::-1]])

def derivatives(c,T,count):
    out=[]
    for _ in range(count):c=(len(c)-1)*np.diff(c,axis=0)/T;out.append(c)
    return out

def wire_cone_separated(cp,point,a,b,r):
    """A separating plane proves the entire camera-hull sight cone is clear."""
    center=cp.mean(axis=0);u=point-center;v=b-a;w=center-a;aa=u@u;bb=u@v;cc=v@v;dd=u@w;ee=v@w;den=aa*cc-bb*bb;candidates=[]
    if den>1e-20:
        s,t=(bb*ee-cc*dd)/den,(aa*ee-bb*dd)/den
        if 0<=s<=1 and 0<=t<=1:candidates.append((center+s*u,a+t*v))
    for p in [center,point]:candidates.append((p,a+np.clip((p-a)@v/max(cc,1e-30),0,1)*v))
    for q in [a,b]:candidates.append((center+np.clip((q-center)@u/max(aa,1e-30),0,1)*u,q))
    p,q=min(candidates,key=lambda pair:np.linalg.norm(pair[0]-pair[1]));normal=p-q;length=np.linalg.norm(normal)
    if length<1e-12:return False
    normal/=length;cone_min=min(0.,float(np.min((cp-point)@normal)));wire_max=max(float((a-point)@normal),float((b-point)@normal))+r
    return cone_min-wire_max>1e-8

def cylinder_cone_separated(cp,point,cylinder):
    center=cp.mean(axis=0);xy=np.array(cylinder['center_xy_m']);lo=cylinder['base_elevation_m'];hi=lo+cylinder['height_m'];radius=cylinder['radius_m']
    for s in [0.,.25,.5,.75,1.]:
        p=point+s*(center-point);radial=p[:2]-xy;length=np.linalg.norm(radial);closest=np.r_[xy+radial*min(1.,radius/max(length,1e-30)),np.clip(p[2],lo,hi)];normal=p-closest;nl=np.linalg.norm(normal)
        if nl<1e-12:continue
        normal/=nl;cone_min=min(0.,float(np.min((cp-point)@normal)))
        support=float((xy-point[:2])@normal[:2])+radius*np.linalg.norm(normal[:2])+normal[2]*((hi if normal[2]>=0 else lo)-point[2])
        if cone_min-support>1e-8:return True
    return False

def optics_bound(cp,T,ac,task,body,scene,geo,angular_derivative=None):
    cfg=scene['joint_reference_profile']['camera'];angles=hull(ac);rates=hull(7*np.diff(ac,axis=0)/T if angular_derivative is None else angular_derivative)
    gp,gy=angles[1],angles[2];gpd,gyd=rates[1],rates[2]
    opt=vector([gp.cos()*gy.cos(),gp.cos()*gy.sin(),gp.sin()]);right=vector([gy.sin(),-gy.cos(),0])
    optd=vector([-gp.sin()*gpd*gy.cos()-gp.cos()*gy.sin()*gyd,-gp.sin()*gpd*gy.sin()+gp.cos()*gy.cos()*gyd,gp.cos()*gpd])
    rightd=vector([gy.cos()*gyd,gy.sin()*gyd,0]);down=cross(opt,right);downd=cross(optd,right)+cross(opt,rightd)
    cols=body['body_columns'];dcols=body['body_column_rates']
    def rotate(a):return cols[0]*a[0]+cols[1]*a[1]+cols[2]*a[2]
    def rotate_rate(a,ad):return dcols[0]*a[0]+dcols[1]*a[1]+dcols[2]*a[2]+rotate(ad)
    rc=[rotate(a) for a in [right,down,opt]];rd=[rotate_rate(a,ad) for a,ad in zip([right,down,opt],[rightd,downd,optd])]
    fx0=math.hypot(cfg['image_width_px'],cfg['image_height_px'])/(2*math.tan(math.radians(cfg['diagonal_fov_deg_at_wide']/2)));f=fx0*angles[3];fd=fx0*rates[3]
    target=task['target'];q=np.array([target[k] for k in ['easting_m','northing_m','altitude_m']]);delta=body['position']-q;distance=norm(delta)
    nominal=np.array([task[k] for k in ['easting_m','northing_m','altitude_m']]);viewradius_upper=float(np.linalg.norm(cp-nominal,axis=1).max())
    inc=dot(I(target['normal_world']),delta)/distance
    if distance.lo<cfg['focus_min_m']-1e-7 or distance.hi>cfg['observation_max_m']+1e-7 or inc.lo<math.cos(math.radians(cfg['incidence_angle_max_deg']))-1e-7 or viewradius_upper>task['observation_requirement']['viewpoint_ball_radius_m']+1e-7:return False,dict(reason='view_region',distance=[float(distance.lo),float(distance.hi)],incidence_cos_lower=float(inc.lo),view_radius_upper_m=viewradius_upper)
    center=np.mean(cp,axis=0);radius=float(np.linalg.norm(cp-center,axis=1).max());blur=0.;minimum_psd=math.inf;pixel_margin=math.inf
    for point in target['surface_samples_utm_m']:
        point=np.array(point);rel=I(point)-body['position'];vrel=-body['velocity'];qc=[dot(col,rel) for col in rc];qcd=[dot(cold,rel)+dot(col,vrel) for col,cold in zip(rc,rd)]
        if qc[2].lo<=0:return False,dict(reason='camera_depth')
        px=f*qc[0]/qc[2]+cfg['image_width_px']/2;py=f*qc[1]/qc[2]+cfg['image_height_px']/2
        pixel_margin=min(pixel_margin,float(px.lo-cfg['frame_border_px']),float(cfg['image_width_px']-cfg['frame_border_px']-px.hi),float(py.lo-cfg['frame_border_px']),float(cfg['image_height_px']-cfg['frame_border_px']-py.hi))
        dx=fd*qc[0]/qc[2]+f*(qcd[0]*qc[2]-qc[0]*qcd[2])/qc[2].square();dy=fd*qc[1]/qc[2]+f*(qcd[1]*qc[2]-qc[1]*qcd[2])/qc[2].square()
        blur=max(blur,float(max(dx.absmax(),dy.absmax()))*cfg['exposure_bounds_s'][0])
        jac=[]
        for tangent in target['tangent_basis']:
            r=[dot(col,I(tangent)) for col in rc]
            jac.append([f*(r[0]*qc[2]-qc[0]*r[2])/qc[2].square(),f*(r[1]*qc[2]-qc[1]*r[2])/qc[2].square()])
        m00=(jac[0][0].square()+jac[0][1].square())*cfg['gsd_max_m_per_px']**2-1
        m11=(jac[1][0].square()+jac[1][1].square())*cfg['gsd_max_m_per_px']**2-1
        m01=(jac[0][0]*jac[1][0]+jac[0][1]*jac[1][1])*cfg['gsd_max_m_per_px']**2
        determinant=m00*m11-m01.square();minimum_psd=min(minimum_psd,float(m00.lo),float(m11.lo),float(determinant.lo))
        # The camera curve lies in the hull ball; all sightlines lie within
        # radius of the center-to-surface ray. Inflate the source obstacles.
        mid=(center+point)/2;length=np.linalg.norm(point-center)
        for idx in geo.tree.query_ball_point(mid[:2],length/2+radius+3):
            if cylinder_cone_separated(cp,point,geo.cyl[idx]):continue
            obstacle=dict(geo.cyl[idx]);obstacle['radius_m']+=radius+1e-8;obstacle['base_elevation_m']-=radius;obstacle['height_m']+=2*radius
            if solid_cylinder_intersection(center,point,obstacle):return False,dict(reason='sight_cylinder_hull')
        close=np.flatnonzero(np.linalg.norm(geo.centers-mid,axis=1)<=geo.radii+length/2+radius+.2)
        for idx in close:
            points,r,wid=geo.wires[idx]
            # Disjoint expanded axis boxes prove segment distance > r+radius;
            # reuse the source geometry's broad phase before exact distances.
            lo=np.minimum(center,point)-r-radius-1e-8;hi=np.maximum(center,point)+r+radius+1e-8
            selected=np.flatnonzero(np.all(np.maximum(points[:-1],points[1:])>=lo,axis=1)&np.all(np.minimum(points[:-1],points[1:])<=hi,axis=1))
            if any(closest_segments(center,point,points[j],points[j+1])<r+radius and not wire_cone_separated(cp,point,points[j],points[j+1],r) for j in selected):return False,dict(reason='sight_wire_hull',wire=wid)
    values=dict(blur_upper_px=blur,FOV_border_margin_px=pixel_margin,GSD_PSD_margin=minimum_psd)
    return bool(blur<=cfg['blur_max_px']+1e-7 and pixel_margin>=-1e-7 and minimum_psd>=-1e-7),values


def evaluate(tag,sortie_files=None,context_data=None):
    started=time.perf_counter()
    old_path=OUT/(tag+'_validation.json')
    if old_path.exists():
        history=OUT/(tag+'_validation_attempts.json');old=read(history) if history.exists() else [];old.append(read(old_path));save(history,old)
    scene,pilot,terrain,weather=context() if context_data is None else context_data
    cfg=scene['joint_reference_profile'];geo=Geometry(scene);force=ForceEnclosure(scene,terrain,weather)
    air=cfg['aircraft'];cam=cfg['camera'];clearance=sum(air[k] for k in ['body_radius_m','obstacle_clearance_m','tracking_error_m','map_error_m'])
    tri=terrain.tri.simplices;coords=terrain.points[tri];tb=dict(lo=coords.min(axis=1),hi=coords.max(axis=1),max_z=terrain.values[:,0][tri].max(axis=1))
    rows=[];flights=[]
    files=sortie_files if sortie_files is not None else [f'{tag}_sortie{k}.json' for k in [0,1]]
    for slot,filename in enumerate(files):
        path=OUT/filename;d=read(path);flights.append(d);failed=[];stats=dict(accepted_intervals=0,evaluations=0,max_depth=0);boundrows=[];previous=None;targets={t['id']:t for t in d['targets']}
        max_join=0.;max_pose_join=0.;optical_tasks=set();clock=0.
        for index,seg in enumerate(d['timeline']):
            cp=np.array(seg['control_points_m']);ac=np.array(seg['pose_controls_yaw_gp_gy_focal_ratio']);T=seg['end_s']-seg['start_s'];t0=seg['start_s']
            states=eval_septic(cp,T,0.);pstates=eval_septic(ac,T,0.)
            if previous:
                max_join=max(max_join,max(np.linalg.norm(x-y) for x,y in zip(previous[0],states)))
                max_pose_join=max(max_pose_join,max(np.linalg.norm(x-y) for x,y in zip(previous[1][:3],pstates[:3])))
            previous=(eval_septic(cp,T,1.),eval_septic(ac,T,1.))
            if abs(t0-clock)>.001:failed.append(dict(segment=index,reason='time_gap'))
            clock=seg['end_s'];contact=seg.get('departure_contact',False) or seg.get('arrival_contact',False)
            geometry_ok,depth=geometry_certificate(cp,geo,clearance,terrain,tb,contact)
            if not geometry_ok:failed.append(dict(segment=index,reason='continuous_geometry'))
            task=targets[seg['task_id']] if seg['kind']=='capture' else None;local_origin=cp[0].copy()
            segment_evaluations=0
            def recurse(c,a,dt,start,position_derivatives,angle_derivatives,depth=0):
                nonlocal segment_evaluations
                segment_evaluations+=1
                if segment_evaluations>2048:
                    failed.append(dict(segment=index,reason='interval_budget_inconclusive',evaluations=segment_evaluations));return False
                stats['evaluations']+=1;stats['max_depth']=max(stats['max_depth'],depth);der=[hull(z) for z in position_derivatives]
                dr,ddr=[hull(z) for z in angle_derivatives];angles=hull(a);good=True;values={}
                for name,z,limit in [('speed',norm(der[0]).hi,air['ground_speed_max_mps']),('vertical',der[0][2].absmax(),air['vertical_speed_max_mps']),('acceleration',norm(der[1]).hi,air['acceleration_max_mps2']),('jerk',norm(der[2]).hi,air['jerk_max_mps3'])]:
                    if z>limit+1e-6:good=False;values[name]=float(z)
                if np.max(dr[:3].absmax())>math.radians(30)+1e-6 or np.max(ddr[:3].absmax())>math.radians(30)+1e-6 or dr[3].absmax()*6.83>cam['zoom_rate_max_mm_s']+1e-6:good=False;values['pose_rate']='interval exceeds'
                if angles[3].lo<1-1e-7 or angles[3].hi>cam['focal_length_mm_bounds'][1]/6.83+1e-7:good=False;values['focal']='bounds'
                for col,key in [(1,'gimbal_pitch_deg'),(2,'gimbal_yaw_deg')]:
                    if angles[col].lo<math.radians(cam[key][0])-1e-6 or angles[col].hi>math.radians(cam[key][1])+1e-6:good=False;values[key]='bounds'
                try:
                    physics,pvalues,body=force.check(c,dt,a[:,0],start+d['optimized_launch_s']+dt/2,local_origin,details=True,state_derivatives=position_derivatives,yaw_derivative=angle_derivatives[0][:,0]);good=good and physics;values.update(pvalues)
                    if task:
                        optics,ovalues=optics_bound(c+local_origin,dt,a,task,body,scene,geo,angle_derivatives[0]);good=good and optics;values.update(ovalues)
                except (ArithmeticError,ValueError) as exc:good=False;values['reason']=str(exc)
                if good:stats['accepted_intervals']+=1;boundrows.append(values);return True
                if depth>=18 or dt<2e-5:
                    failed.append(dict(segment=index,start_s=start,end_s=start+dt,reason='interval_failed_or_unresolved',bounds=values));return False
                l,r=split_controls(c);al,ar=split_controls(a)
                pds=[split_any(z) for z in position_derivatives];ads=[split_any(z) for z in angle_derivatives]
                if not recurse(l,al,dt/2,start,[z[0] for z in pds],[z[0] for z in ads],depth+1):return False
                return recurse(r,ar,dt/2,start+dt/2,[z[1] for z in pds],[z[1] for z in ads],depth+1)
            absolute=t0+d['optimized_launch_s'];end=absolute+T
            cuts=[absolute]+[float(h) for h in range((int(absolute//3600)+1)*3600,int(end)+1,3600) if absolute<h<end]+[end]
            passed=True;position_derivatives=derivatives(cp-local_origin,T,3);angle_derivatives=derivatives(ac,T,2)
            for left,right in zip(cuts,cuts[1:]):
                lo,hi=(left-absolute)/T,(right-absolute)/T
                piece=recurse(crop(cp-local_origin,lo,hi),crop(ac,lo,hi),right-left,left-d['optimized_launch_s'],
                    [crop(z,lo,hi) for z in position_derivatives],[crop(z,lo,hi) for z in angle_derivatives])
                passed=passed and piece
            if task and passed:optical_tasks.add(task['id'])
            print('verify',tag,slot,index,'geometry',geometry_ok,'physics_optics',passed,'evaluations',segment_evaluations,flush=True)
            save(OUT/(tag+'_validation_live.json'),dict(observed_at=stamp(),slot=slot,segment=index,geometry_pass=geometry_ok,physics_optics_pass=passed,segment_evaluations=segment_evaluations,failures=failed,stats=stats))
            # Retain failures across all segments; an early violation must
            # not hide later optics/geometry defects during engineering repair.
        if max_join>1e-5:failed.append(dict(reason='C3',error=max_join))
        if max_pose_join>1e-5:failed.append(dict(reason='pose_C2',error=max_pose_join))
        # Piecewise-constant source wind can jump. A bounded body-rate model
        # also requires continuous orientation at these jumps; checking the
        # two smooth traces alone would miss an instantaneous attitude change.
        launch=d['optimized_launch_s'];landing=launch+d['flight_seconds'];weather_jumps=[]
        for boundary in range((int(launch//3600)+1)*3600,int(landing)+1,3600):
            if not launch<boundary<landing:continue
            local=boundary-launch
            seg=next(s for s in d['timeline'] if s['start_s']-1e-7<=local<=s['end_s']+1e-7)
            T=seg['end_s']-seg['start_s'];u=np.clip((local-seg['start_s'])/T,0,1)
            p,v,acc,_=eval_septic(seg['control_points_m'],T,u);yaw=eval_septic(seg['pose_controls_yaw_gp_gy_focal_ratio'],T,u)[0][0]
            rotations=[]
            for trace in [-1e-5,1e-5]:
                wind,rho=weather(p,boundary+trace,float(terrain(*p[:2])));va=v-wind
                F=air['mass_kg']*(acc+[0,0,9.81])+.5*rho*air['drag_CdA_m2']*np.linalg.norm(va)*va
                b3=F/np.linalg.norm(F);b2=np.cross(b3,[math.cos(yaw),math.sin(yaw),0]);b2/=np.linalg.norm(b2)
                rotations.append(np.column_stack([np.cross(b2,b3),b2,b3]))
            jump=float(np.linalg.norm(rotations[0]-rotations[1]));weather_jumps.append(dict(time_s=boundary,rotation_jump_norm=jump))
            if jump>1e-6:failed.append(dict(reason='source_weather_attitude_jump',time_s=boundary,rotation_jump_norm=jump))
        # Reintegrate with the exact source-hour DEM-dependent wind function.
        copied=copy.deepcopy(d['timeline']);energy=energy_of(copied,cfg,terrain,weather,d['optimized_launch_s']);delta=abs(energy-d['aircraft_energy_Wh'])
        if delta>.01:failed.append(dict(reason='energy_exact_weather',difference_Wh=delta))
        row=dict(slot=slot,source_file=filename,source_sha256=sha(path),all_pass=not failed,failures=failed,stats=stats,max_C3_error=max_join,max_pose_C2_error=max_pose_join,
            energy_Wh=energy,NLP_energy_Wh=d['aircraft_energy_Wh'],energy_difference_Wh=delta,optical_passed_tasks=sorted(optical_tasks),capture_count=len(d['captures']),
            weather_boundary_traces=weather_jumps,
            accepted_bounds={key:max(v[key] for v in boundrows if key in v) for key in ['airspeed_upper_mps','thrust_upper_N','body_rate_component_upper_deg_s','blur_upper_px'] if any(key in v for v in boundrows)})
        rows.append(row);save(OUT/(tag+'_validation.json'),dict(observed_at=stamp(),complete=False,rows=rows,all_pass=False))
    separation=[];required=2*air['body_radius_m']+air['inter_uav_clearance_m']
    for ia,A in enumerate(flights):
      for ib,B in enumerate(flights[ia+1:],ia+1):
       for a in A['timeline']:
        for b in B['timeline']:
            a0=A['optimized_launch_s']+a['start_s'];a1=A['optimized_launch_s']+a['end_s'];b0=B['optimized_launch_s']+b['start_s'];b1=B['optimized_launch_s']+b['end_s'];left,right=max(a0,b0),min(a1,b1)
            if right<=left:continue
            ac=crop(np.array(a['control_points_m']),(left-a0)/(a1-a0),(right-a0)/(a1-a0));bc=crop(np.array(b['control_points_m']),(left-b0)/(b1-b0),(right-b0)/(b1-b0));ok,bound,violation,depth=separate(ac,bc,required,left,right)
            separation.append(dict(flights=[ia,ib],passed=ok,bound_m=bound,violation=violation,
                model_segment_keys=[a.get('model_segment_key'),b.get('model_segment_key')],overlap_s=[left,right]))
    result=dict(observed_at=stamp(),seconds=time.perf_counter()-started,evaluator_sha256=sha(Path(__file__)),force_enclosure_sha256=sha(ROOT/'Agent/Publication/engineering_interval_physics.py'),complete=True,rows=rows,all_pass=all(r['all_pass'] for r in rows) and all(r['passed'] for r in separation),
        pairwise_separation=separation,min_separation_bound_m=min((r['bound_m'] for r in separation),default=None),
        scope='numerical interval/collision-hull enclosures; source reference physics; not measured image/flight validation')
    save(OUT/(tag+'_validation.json'),result);print(json.dumps({k:v for k,v in result.items() if k not in ['rows','pairwise_separation']},ensure_ascii=False));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--tag',default='joint_optical_SOC1');args=p.parse_args();r=evaluate(args.tag)
    if not r['all_pass']:raise SystemExit(1)
