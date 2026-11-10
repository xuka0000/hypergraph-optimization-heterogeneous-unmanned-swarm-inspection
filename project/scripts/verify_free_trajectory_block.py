"""Independent dense and convex-hull checks of exported free controls."""
from run_engineering_full_joint import *
import math
from Agent.Publication.engineering_free_trajectory import split_controls


def derivative_controls(c,T,order):
    c=np.asarray(c,float)
    for _ in range(order):c=(len(c)-1)*np.diff(c,axis=0)/T
    return c


def norm_upper(c,depth=0):
    # Bezier derivative convex hull gives a rigorous upper bound. Subdivision
    # avoids mistaking a loose original control hull for the actual maximum.
    c=np.asarray(c);n=len(c)-1
    if depth==7:return float(np.linalg.norm(c,axis=1).max())
    rows=[c]
    for _ in range(n):rows.append((rows[-1][:-1]+rows[-1][1:])/2)
    left=np.array([r[0] for r in rows]);right=np.array([r[-1] for r in rows[::-1]])
    return max(norm_upper(left,depth+1),norm_upper(right,depth+1))


def geometry_certificate(c,geo,clearance,terrain,L,contact,depth=0):
    a,b=c[0],c[-1];d=b-a;den=d@d
    fraction=np.clip((c-a)@d/max(den,1e-30),0,1);radius=float(np.linalg.norm(c-a-fraction[:,None]*d,axis=1).max())
    obstacle=geo.segment_margin(a,b,clearance+radius)
    center=c[:,:2].mean(axis=0);R=float(np.linalg.norm(c[:,:2]-center,axis=1).max());ground=float(terrain(*center))
    in_hull=bool((terrain.tri.find_simplex(c[:,:2])>=0).all())
    required=.61 if contact else clearance
    if R<1e-7:upper_ground=ground
    else:
        lo=c[:,:2].min(axis=0);hi=c[:,:2].max(axis=0)
        selected=np.all(L['hi']>=lo-1e-8,axis=1)&np.all(L['lo']<=hi+1e-8,axis=1)
        upper_ground=float(L['max_z'][selected].max()) if selected.any() else math.inf
    ground_margin=float(c[:,2].min()-upper_ground-required)
    if in_hull and obstacle>=-1e-7 and ground_margin>=-1e-7:return True,depth
    if depth>=15:return False,depth
    left,right=split_controls(c);one,d1=geometry_certificate(left,geo,clearance,terrain,L,contact,depth+1)
    if not one:return False,d1
    two,d2=geometry_certificate(right,geo,clearance,terrain,L,contact,depth+1);return two,max(d1,d2)


def evaluate_leg(row,scene,terrain,weather,geo,L):
    cfg=scene['joint_reference_profile'];air=cfg['aircraft'];clearance=sum(air[k] for k in ['body_radius_m','obstacle_clearance_m','tracking_error_m','map_error_m']);timeline=leg_timeline(row,row['i']==9,row['j']==10)
    failure=[];previous=None;join=0.;samples=0;depth=0
    maxima=dict(ground_speed_mps=0.,vertical_speed_mps=0.,acceleration_mps2=0.,jerk_mps3=0.,air_speed_mps=0.,tilt_deg=0.,thrust_N=0.)
    for seg in timeline:
        c=np.array(seg['control_points_m']);T=seg['end_s']-seg['start_s'];left=eval_septic(c,T,0);right=eval_septic(c,T,1)
        if previous is not None:join=max(join,max(float(np.linalg.norm(a-b)) for a,b in zip(previous,left)))
        previous=right
        contact=seg.get('departure_contact',False) or seg.get('arrival_contact',False)
        if contact and np.max(np.linalg.norm(c[:,:2]-c[0,:2],axis=1))>1e-6:failure.append('nonvertical_contact')
        passed,d=geometry_certificate(c,geo,clearance,terrain,L,contact);depth=max(depth,d)
        if not passed:failure.append('continuous_geometry_certificate_failed')
        for order,key in [(1,'ground_speed_mps'),(2,'acceleration_mps2'),(3,'jerk_mps3')]:maxima[key]=max(maxima[key],norm_upper(derivative_controls(c,T,order)))
        maxima['vertical_speed_mps']=max(maxima['vertical_speed_mps'],norm_upper(derivative_controls(c,T,1)[:,2:3]))
        for x in np.linspace(0,1,int(math.ceil(T/.25))+1):
            p,v,ac,j=eval_septic(c,T,x);wind,rho=weather(p,seg['start_s']+x*T,float(terrain(*p[:2])));va=v-wind
            force=air['mass_kg']*(ac+[0,0,9.81])+.5*rho*air['drag_CdA_m2']*np.linalg.norm(va)*va;samples+=1
            maxima['air_speed_mps']=max(maxima['air_speed_mps'],float(np.linalg.norm(va)));maxima['thrust_N']=max(maxima['thrust_N'],float(np.linalg.norm(force)))
            maxima['tilt_deg']=max(maxima['tilt_deg'],math.degrees(math.acos(np.clip(force[2]/np.linalg.norm(force),-1,1))))
    limits=dict(ground_speed_mps=air['ground_speed_max_mps'],vertical_speed_mps=air['vertical_speed_max_mps'],acceleration_mps2=air['acceleration_max_mps2'],jerk_mps3=air['jerk_max_mps3'],air_speed_mps=air['air_speed_max_mps'],tilt_deg=air['tilt_max_deg'],thrust_N=air['thrust_max_N'])
    for key,limit in limits.items():
        if maxima[key]>limit+1e-5:failure.append(key)
    if join>1e-6:failure.append('C3_join')
    wh=energy_of(timeline,cfg,terrain,weather);ediff=abs(wh-row['energy_Wh'])
    if ediff>.01:failure.append('energy')
    return dict(vehicle=row['vehicle'],i=row['i'],j=row['j'],passed=not failure,failures=failure,maxima=maxima,limits=limits,C3_join_max_error=join,
        physical_samples=samples,continuous_geometry_pass='continuous_geometry_certificate_failed' not in failure,geometry_subdivision_depth=depth,
        kinematic_bounds_method='7-level Bernstein derivative convex-hull subdivision',aerodynamics_sampled=True,pose_checks_pending=True,energy_difference_Wh=ediff)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,default=OUT/'full_joint_free_input.json');parser.add_argument('--output',type=Path,default=OUT/'free_trajectory_independent_validation.json');args=parser.parse_args()
    source=args.input;data=read(source);scene,pilot,terrain,weather=context();geo=Geometry(scene)
    vertices=terrain.tri.simplices;vals=terrain.values[:,0];xy=terrain.points[vertices]
    L=dict(lo=xy.min(axis=1),hi=xy.max(axis=1),max_z=vals[vertices].max(axis=1))
    cache={};rows=[]
    for row in data['legs']:
        key=(0 if row['i']<9 and row['j']<9 else row['vehicle'],row['i'],row['j'])
        if key not in cache:cache[key]=evaluate_leg(row,scene,terrain,weather,geo,L);print(key,cache[key]['failures'],flush=True)
        result=dict(cache[key],vehicle=row['vehicle']);rows.append(result)
        save(args.output,dict(source_sha256=sha(source),observed_at=stamp(),completed=len(rows),expected=len(data['legs']),
            terrain_bound_method='maximum vertex elevation of all source DEM triangles whose boxes intersect the Bezier control hull box; exact height for vertical contact',rows=rows,all_checked_pass=all(r['passed'] for r in rows),all_pose_checks_complete=False))

if __name__=='__main__':main()
