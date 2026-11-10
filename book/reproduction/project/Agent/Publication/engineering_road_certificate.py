"""Independent continuous certificate for exported road-motion primitives.

This certifies an exported primitive trajectory, not completeness/optimality
of that primitive family within the full Ackermann road-motion domain.
"""
import math
import numpy as np
from shapely.geometry import LineString,Polygon
from shapely.ops import unary_union


def certify_stationary(scene,vehicle,site):
    """A zero-speed choice must still fit the real source road corridor."""
    cfg=scene['joint_reference_profile']['vehicle'];p=np.asarray(site['xyz_m'][:2]);yaw=site['yaw_rad']
    nodes={n['id']:np.array([n['easting_m'],n['northing_m']]) for n in scene['roads']['nodes']}
    u=np.array([math.cos(yaw),math.sin(yaw)]);v=np.array([-u[1],u[0]])
    footprint=Polygon([p+a*cfg['length_m']/2*u+b*cfg['width_m']/2*v for a,b in [(1,1),(1,-1),(-1,-1),(-1,1)]])
    radius=math.hypot(cfg['length_m']/2,cfg['width_m']/2)+cfg['road_corridor_width_m']/2
    pieces=[];edge_ids=[]
    for edge in scene['roads']['edges']:
        a,b=nodes[edge['tail']],nodes[edge['head']]
        if np.any(np.maximum(a,b)<p-radius) or np.any(np.minimum(a,b)>p+radius):continue
        pieces.append(LineString([a,b]).buffer(cfg['road_corridor_width_m']/2,cap_style=1,join_style=1));edge_ids.append(edge['id'])
    covered=bool(pieces and unary_union(pieces).covers(footprint.buffer(1e-8)))
    source=scene['fleet']['vehicles'][vehicle]
    at_depot=bool(site['stop_id']==source['depot_stop'] and np.linalg.norm(p-np.asarray(source['initial_xyz_m'][:2]))<1e-7)
    return dict(passed=covered and at_depot,stationary=True,footprint_inside_source_corridor=covered,
                source_depot=at_depot,source_edge_ids=edge_ids,scope='full rectangular parked footprint, fixed source depot')


def certify(scene,route,leg_scales=(1.,1.)):
    if route is None:return dict(passed=True,stationary=True,checks=[],scope='stationary exact')
    cfg=scene['joint_reference_profile']['vehicle'];nodes={n['id']:n for n in scene['roads']['nodes']}
    edges={e['id']:e for e in scene['roads']['edges']};checks=[]
    def check(name,ok,**detail):checks.append(dict(name=name,passed=bool(ok),**detail))
    def xy(n):return np.array([nodes[n]['easting_m'],nodes[n]['northing_m']],float)
    ids=route['node_ids'];allowed={(e['tail'],e['head']) for e in edges.values()}
    check('directed source edges',all((a,b) in allowed for a,b in zip(ids,ids[1:])))
    check('source turn relations supported',not scene['roads']['turn_restrictions'])
    corridors=unary_union([LineString([xy(a),xy(b)]).buffer(cfg['road_corridor_width_m']/2,cap_style=1,join_style=1) for a,b in zip(ids,ids[1:])])
    corner_radius=math.hypot(cfg['length_m']/2,cfg['width_m']/2)
    def geometry(seg,s):
        q=35*s**4-84*s**5+70*s**6-20*s**7
        if seg['kind']=='straight':
            p=np.asarray(seg['from_xy_m'])+q*(np.asarray(seg['to_xy_m'])-seg['from_xy_m']);yaw=seg['yaw_rad']
        elif seg['kind']=='arc':
            angle=seg['start_angle_rad']+seg['sweep_rad']*q
            p=np.asarray(seg['center_xy_m'])+seg['radius_m']*np.array([math.cos(angle),math.sin(angle)])
            yaw=seg['yaw_start_rad']+seg['sweep_rad']*q
        else:p=np.asarray(seg['position_xy_m']);yaw=seg['yaw_rad']
        return p,yaw
    def footprint(seg,s):
        p,yaw=geometry(seg,s);u=np.array([math.cos(yaw),math.sin(yaw)]);v=np.array([-u[1],u[0]])
        return Polygon([p+a*cfg['length_m']/2*u+b*cfg['width_m']/2*v for a,b in [(1,1),(1,-1),(-1,-1),(-1,1)]])
    def enclosure(seg,lo=0.,hi=1.,depth=0):
        sweep=abs(seg.get('sweep_rad',0.));L=2.1875*(seg['length_m']+sweep*corner_radius)
        margin=L*(hi-lo)/2
        polygon=footprint(seg,(lo+hi)/2)
        if corridors.covers(polygon.buffer(margin+1e-9)):return True,depth
        if depth>=18:return False,depth
        mid=(lo+hi)/2;one,d1=enclosure(seg,lo,mid,depth+1)
        if not one:return False,d1
        two,d2=enclosure(seg,mid,hi,depth+1);return two,max(d1,d2)
    split=route['visit_events'][0]['after_segment_count'] if route.get('visit_events') else len(route['segments'])
    previous=None
    for k,seg in enumerate(route['segments']):
        scale=leg_scales[0 if k<split else 1];T=seg['duration_s']*scale;length=seg['length_m']
        check(f'segment{k} positive duration',T>0 and scale>=1-1e-9)
        vmax=2.1875*length/T;amax=7.513188405*length/T**2
        check(f'segment{k} speed and acceleration',vmax<=cfg['speed_max_mps']+1e-8 and amax<=cfg['acceleration_max_mps2']+1e-8,vmax=vmax,amax=amax)
        delta=abs(seg.get('steering_rad',0.));rate=0.;lateral=0.
        if seg['kind']=='steer':
            delta=max(abs(seg['steering_start_rad']),abs(seg['steering_end_rad']))
            rate=2.1875*abs(seg['steering_end_rad']-seg['steering_start_rad'])/T
        if seg['kind']=='arc':
            lateral=vmax**2/seg['radius_m']
            check(f'segment{k} Ackermann identity',abs(math.tan(seg['steering_rad'])/cfg['wheelbase_m']-math.copysign(1.,seg['sweep_rad'])/seg['radius_m'])<1e-9)
        check(f'segment{k} steering and lateral acceleration',delta<=math.radians(cfg['steering_max_deg'])+1e-9 and rate<=math.radians(cfg['steering_rate_max_deg_s'])+1e-9 and lateral<=cfg['lateral_acceleration_max_mps2']+1e-8)
        first,last=geometry(seg,0),geometry(seg,1)
        if previous:
            angle=math.atan2(math.sin(first[1]-previous[1]),math.cos(first[1]-previous[1]))
            check(f'segment{k} state continuity',np.linalg.norm(first[0]-previous[0])<1e-6 and abs(angle)<1e-6)
        previous=last;ok,depth=enclosure(seg)
        check(f'segment{k} continuous footprint',ok,depth=depth)
    return dict(passed=all(x['passed'] for x in checks),checks=checks,stationary=False,
                scope='source-directed exported primitives; analytic extrema and continuous footprint enclosure; full vehicle domain not certified')
