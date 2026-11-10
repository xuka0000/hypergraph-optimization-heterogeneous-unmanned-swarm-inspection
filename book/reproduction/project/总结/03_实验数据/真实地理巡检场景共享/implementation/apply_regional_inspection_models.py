"""Apply source-bound nine-view and joint reference models in place.

Only the three authorized derived scene JSONs change. Map objects, all prior
optimizer results, and the five already configured scene files are preserved.
This configures reference simulation inputs; it is not a joint optimizer.
"""
from pathlib import Path
import copy, datetime, hashlib, json, math, sys
from collections import defaultdict
import networkx as nx
import numpy as np
from pyproj import Transformer
from scipy.interpolate import LinearNDInterpolator
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from export_geographic_scene_pack import add_views, operational_parameters
from Agent.Publication.inspection_charging import profile as charge_profile

PACK=ROOT/'总结/03_实验数据/真实地理巡检场景共享'
NAMES=['rommerskirchen','brauweiler','villevaude']
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,d):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(d,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n',encoding='utf-8')
def source(sid,p,url,title,kind='project_implementation'):
    return {'id':sid,'title':title,'url':url,'locator':'complete file','sha256':sha(p),'bytes':p.stat().st_size,
        'package_path':p.relative_to(PACK).as_posix(),'original_workspace_relative_path':p.relative_to(ROOT).as_posix(),
        'evidence_kind':kind,'retrieved_at':None,'source_hash_verified':True,'hash_verification_basis':'current file bytes'}

class Terrain:
    def __init__(self,rows,crs):
        tr=Transformer.from_crs(4326,crs,always_xy=True)
        x,y=tr.transform([r['location']['lng'] for r in rows],[r['location']['lat'] for r in rows])
        self.f=LinearNDInterpolator(np.column_stack([x,y]),[r['elevation'] for r in rows])
    def __call__(self,xy):
        a=float(self.f(*xy))
        if not math.isfinite(a):raise ValueError('Outside acquired DEM hull; no silent fallback')
        return a

def build_roads(s):
    roads=s['roads'];nodes={x['id']:x for x in roads['nodes']};restr=[];excluded_ways=set();unsupported=[]
    for rel in roads.get('source_relations',[]):
        tags=rel.get('tags',{})
        if tags.get('type')!='restriction':continue
        ms=rel.get('members',[]);frm=[m for m in ms if m.get('role')=='from'];to=[m for m in ms if m.get('role')=='to'];via=[m for m in ms if m.get('role')=='via']
        rule=tags.get('restriction:motorcar',tags.get('restriction:motor_vehicle',tags.get('restriction','')))
        if len(frm)!=1 or len(to)!=1 or len(via)!=1 or via[0]['type']!='node' or not rule.startswith(('no_','only_')) or any('conditional' in k for k in tags):
            excluded_ways.update(str(m['ref']) for m in frm);unsupported.append(str(rel['id']));continue
        restr.append(dict(id=str(rel['id']),from_way=str(frm[0]['ref']),to_way=str(to[0]['ref']),via=str(via[0]['ref']),rule=rule))
    allowed={'motorway','motorway_link','trunk','trunk_link','primary','primary_link','secondary','secondary_link','tertiary','tertiary_link','unclassified','residential','living_street','service','track'}
    barriers={'bollard','block','bus_trap','stile','turnstile','cycle_barrier','chain','fence','wall','jersey_barrier','gate','lift_gate','swing_gate'}
    permitted={'yes','permissive','designated'}
    def permission(t,default=None):return next((t[k] for k in ['motorcar','motor_vehicle','vehicle','access'] if k in t),default)
    blocked={n for n,v in nodes.items() if permission(v['original_tags']) not in permitted|{None} or (v['original_tags'].get('barrier') in barriers and permission(v['original_tags']) not in permitted)}
    g=nx.DiGraph();excluded=[]
    for seg in roads['segments']:
        t=seg['original_tags'];wid=seg['osm_way_id'];a,b=seg['tail'],seg['head'];kind=t.get('highway');perm=permission(t,'yes')
        reason=None
        if kind not in allowed:reason='nonmotor_highway'
        elif wid in excluded_ways:reason='unsupported_turn_origin'
        elif perm not in permitted or any('conditional' in k for k in t if k.startswith(('access','vehicle','motor','oneway'))):reason='restricted_or_conditional'
        elif a in blocked or b in blocked:reason='barrier_or_node_access'
        one=t.get('oneway','yes' if t.get('junction')=='roundabout' else 'no')
        if one not in {'yes','1','true','-1','no','0','false'}:reason='unsupported_direction'
        if reason:excluded.append({'segment':seg['id'],'reason':reason});continue
        speed=10.
        try:
            limit=float(t.get('maxspeed','').split()[0]);limit*=1.609344 if 'mph' in t.get('maxspeed','') else 1
            if limit>0:speed=min(speed,limit)
        except (ValueError,IndexError):pass
        for u,v in ([(b,a)] if one=='-1' else [(a,b)] if one in {'yes','1','true'} else [(a,b),(b,a)]):
            e=dict(tail=u,head=v,length_m=seg['length_m'],seconds=seg['length_m']/(speed/3.6),speed_limit_mps=speed/3.6,
                way_id=wid,source_segment_id=seg['id'],highway=kind,access=perm,original_oneway=one,source_ids=seg['source_ids'])
            if not g.has_edge(u,v) or e['seconds']<g[u][v]['seconds']:g.add_edge(u,v,**e)
    components=sorted(nx.strongly_connected_components(g),key=lambda c:(-len(c),min(c)))
    assert components
    g=g.subgraph(components[0]).copy()
    for n in g:g.nodes[n].update(nodes[n])
    rs=[r for r in restr if r['via'] in g]
    ri=defaultdict(list)
    for r in rs:ri[(r['via'],r['from_way'])].append(r)
    edges=sorted(g.edges());edge_ids={uv:f'RE{i:06}' for i,uv in enumerate(edges)}
    transitions=[]
    for a,b in edges:
        incoming=g[a][b]['way_id']
        for c in g.successors(b):
            outgoing=g[b][c]['way_id'];valid=True
            for r in ri[(b,incoming)]:
                match=outgoing==r['to_way']
                if r['rule']=='no_u_turn':match=match and c==a
                if (r['rule'].startswith('no_') and match) or (r['rule'].startswith('only_') and not match):valid=False
            if valid:transitions.append([edge_ids[(a,b)],edge_ids[(b,c)]])
    # Restriction-aware edge-state graph: do not infer reachability from a node graph alone.
    state=nx.DiGraph();state.add_nodes_from(edge_ids.values());state.add_edges_from(transitions)
    sc=sorted(nx.strongly_connected_components(state),key=lambda c:(-len(c),min(c)))
    retained=sc[0];kept=[(a,b) for a,b in edges if edge_ids[(a,b)] in retained]
    used={n for uv in kept for n in uv};g=g.edge_subgraph(kept).copy()
    for n in g:g.nodes[n].update(nodes[n])
    out=[]
    for a,b in kept:out.append(dict(g[a][b],id=edge_ids[(a,b)]))
    roads.update(representation='mapped_geometry_plus_directed_reference_routing',directed=True,edges=out,
        routing_graph={'node_ids':sorted(used),'edge_ids':[e['id'] for e in out],
            'allowed_edge_transitions':[t for t in transitions if t[0] in retained and t[1] in retained],
            'turn_state_strongly_connected':True,'source_node_component_sizes':[len(c) for c in components],
            'source_turn_component_sizes':[len(c) for c in sc]},
        turn_restrictions=rs,turn_restrictions_applied=True,access_policy_applied=True,
        access_policy='motorcar precedence; restrictive/conditional access and unpermitted barriers excluded; mapped untagged roads are planning assumptions',
        excluded_segments=excluded,unsupported_turn_relations=unsupported,reference_speed_kmph=10.,
        field_access_verified=False,parallel_edges='minimum-time source segment retained per ordered node pair; no invented connection')
    return g,{r['via'] for r in rs}

def select_stops(s,g,forbidden,terrain):
    kinds={'tertiary','tertiary_link','unclassified','residential','living_street','service','track'}
    allowed=sorted(n for n in g if n not in forbidden and any(e['highway'] in kinds for _,_,e in g.out_edges(n,data=True)))
    assert allowed,'No model stop candidates on allowed road classes'
    xy=np.array([[g.nodes[n]['easting_m'],g.nodes[n]['northing_m']] for n in allowed]);tree=cKDTree(xy)
    if s['substations']:
        tr=Transformer.from_crs(4326,s['coordinate_reference']['projected'],always_xy=True)
        from shapely.geometry import Polygon
        polys=[(Polygon([tr.transform(*p) for p in v['geometry_lon_lat']]),v['id']) for v in s['substations']]
        poly,pid=max(polys,key=lambda x:x[0].area);center=np.array([poly.centroid.x,poly.centroid.y]);basis='largest_mapped_substation_centroid';ref=pid
    else:center=np.mean([[t['easting_m'],t['northing_m']] for t in s['towers']],axis=0);basis='tower_centroid';ref=None
    n=allowed[int(tree.query(center)[1])];selected=[]
    def add(n,reason):
        if n in {v['osm_node_id'] for v in selected}:return
        v=g.nodes[n];x,y=v['easting_m'],v['northing_m']
        selected.append(dict(id=f'P{len(selected)+1:03}',osm_node_id=n,xy_m=[x,y],lon=v['longitude'],lat=v['latitude'],
            ground_m=terrain([x,y]),reason=reason,parking_status='model_candidate_not_permission',source_ids=[f'OSM_{s["scene_id"]}_roads','MODEL_REGIONAL']))
    add(n,'model_depot_access')
    depotxy=np.asarray(selected[0]['xy_m'])
    second=min((q for q in allowed if np.linalg.norm(np.array([g.nodes[q]['easting_m'],g.nodes[q]['northing_m']])-depotxy)>=12.),
        key=lambda q:np.linalg.norm(np.array([g.nodes[q]['easting_m'],g.nodes[q]['northing_m']])-depotxy))
    add(second,'second_vehicle_depot_access')
    for tower in s['towers']:
        n=allowed[int(tree.query([tower['easting_m'],tower['northing_m']])[1])]
        pos=[g.nodes[n]['easting_m'],g.nodes[n]['northing_m']]
        if min(math.dist(pos,p['xy_m']) for p in selected)>=120:add(n,'near_'+tower['id'])
    s['stops']=selected;s['bases']=[dict(id=f'B{i+1:03}',lon=depot['lon'],lat=depot['lat'],xy_m=depot['xy_m'],ground_m=depot['ground_m'],
        road_stop=depot['id'],osm_node_id=depot['osm_node_id'],snap_distance_m=float(np.linalg.norm(np.asarray(depot['xy_m'])-center)),
        reference_geometry_basis=basis,reference_substation_id=ref,vehicle_ids=[f'V{i+1}'],evidence_kind='model_selected_road_access_not_actual_depot') for i,depot in enumerate(selected[:2])]

def orientation(s):
    by={n['id']:n for n in s['power_nodes']};directions=defaultdict(list)
    for seg in s['power_segments']:
        a,b=seg['tail'],seg['head'];v=np.array([by[b]['easting_m']-by[a]['easting_m'],by[b]['northing_m']-by[a]['northing_m']]);norm=np.linalg.norm(v)
        if norm>0:directions[a].append(v/norm);directions[b].append(-v/norm)
    towers=[]
    for t in s['towers']:
        vs=np.asarray(directions[t['source_object_id']]);assert len(vs)>0
        covariance=vs.T@vs;values,vectors=np.linalg.eigh(covariance);axis=vectors[:,-1]
        if axis[0]<-1e-12 or (abs(axis[0])<=1e-12 and axis[1]<0):axis=-axis
        t['orientation_reference']={'method':'principal axis of unit incident source power segments; sign east then north','incident_segments':len(vs),
            'anisotropy':float((values[-1]-values[0])/max(values.sum(),1e-12)),'as_built_verified':False}
        towers.append(dict(id=t['id'],xy_m=[t['easting_m'],t['northing_m']],line_direction_reference=axis.tolist(),orientation_rule=t['orientation_reference']['method']))
    return towers

def joint_profile():
    # All new numerical choices are declared simulation assumptions, not measured hardware.
    return dict(profile_id='regional_joint_reference_1',role='complete_declared_reference_simulation_not_field_calibration',
        geometry=dict(target_patch_width_m=.5,target_patch_height_m=.5,target_surface_samples=5,viewpoint_ball_radius_m=2.,
            tower_cylinder_radius_m=2.,tower_height_m=35.7378,conductor_radius_m=.05,conductor_sag_reference_m=5.,
            conductor_layers_m=[34.2138,31.0134,25.908,20.8026],conductor_lateral_offsets_m=[-4.8768,4.8768],
            conductor_curve='q(t)=(1-t)*endpoint_a+t*endpoint_b-[0,0,4*sag*t*(1-t)]; endpoints at tower/source node DEM plus layer and incident-axis lateral offset',
            vegetation_model='none; not surveyed'),
        aircraft=dict(reference_family='M300 geometry/endurance reference; no imported M100 learned model',mass_kg=7.,body_radius_m=.6,
            ground_speed_max_mps=5.,air_speed_max_mps=12.,vertical_speed_max_mps=1.,acceleration_max_mps2=3.,jerk_max_mps3=5.,
            tilt_max_deg=25.,body_rate_max_deg_s=30.,thrust_min_N=0.,thrust_max_N=2*7*9.81,drag_CdA_m2=.25,
            tracking_error_m=.5,map_error_m=.5,inter_uav_clearance_m=3.,obstacle_clearance_m=3.),
        camera=dict(image_width_px=5184,image_height_px=3888,diagonal_fov_deg_at_wide=66.6,
            focal_length_mm_bounds=[6.83,119.94],projection='ideal square-pixel pinhole; fx(f)=hypot(width,height)/(2*tan(66.6deg/2))*f/6.83',
            principal_point_px=[2592,1944],distortion=[0,0,0,0,0],mount_offset_m=[0,0,0],gimbal_pitch_deg=[-120,30],gimbal_yaw_deg=[-320,320],
            gimbal_rate_max_deg_s=30.,gimbal_acceleration_max_deg_s2=30.,yaw_rate_max_deg_s=30.,yaw_acceleration_max_deg_s2=30.,
            focus_min_m=8.,observation_max_m=25.,incidence_angle_max_deg=30.,frame_border_px=32,
            gsd_max_m_per_px=.005,blur_max_px=1.,exposure_bounds_s=[.001,.001],zoom_rate_max_mm_s=10.,required_frames=1,
            settle_focus_readout_min_s=1.,exposure_role='fixed declared daylight reference; not calibrated low-light guarantee'),
        vehicle=dict(type='reference_Ackermann',wheelbase_m=2.5,length_m=4.,width_m=1.8,height_m=1.8,
            speed_max_mps=10/3.6,reverse_speed_max_mps=0.,acceleration_max_mps2=1.,steering_max_deg=35.,steering_rate_max_deg_s=20.,
            lateral_acceleration_max_mps2=1.5,road_corridor_width_m=6.,pad_offset_m=[0,0,1.8],pad_region_radius_m=3.,
            moving_landing=False,charging_on_road=False),
        energy=dict(pair_capacity_Wh=548.,initial_SOC=1.,minimum_SOC=.2,terminal_SOC=.2,charge_efficiency=.9,
            charge_tail_minutes=20.,cooling_seconds=0.,supply_max_W_per_vehicle=1000.,idle_power_W=0.,
            temperature_capacity_model='constant within declared 0..40C reference domain',
            power_model='reference_hover_plus_drag_and_positive_climb',hover_power_W=548*3600/2580,
            power_formula='P=hover_W*sqrt(1.225/rho)+0.5*CdA*rho*norm(v_air)^3+mass*g*max(vz,0)/0.7',
            propulsion_efficiency=.7,energy_boundary='aircraft only; vehicle traction excluded',hardware_charge_control_verified=False),
        recovery=dict(candidate_sites='configured stops reached by assigned carrier before landing',
            reserve_rule='E(t)>=energy_of_collision_free_recovery_trajectory(t,site)+0.2*548Wh',
            trajectory_conditions='same flight, weather, terrain, obstacle and landing constraints as planned motion',
            preparation_seconds=120.,recovery_seconds=240.,initial_aircraft_state='stationary on carrier pad',
            terminal_vehicle_state='stationary at initial depot; all assigned aircraft recovered',
            verification='must provide recovery trajectory witness; distance-only estimate is insufficient'),
        weather=dict(source='retained ERA5 replay; not an issued forecast',time_step_s=3600,height_layers_agl_m=[30,60,100],
            temporal_interpolation='left-hour piecewise constant; split integration at hour boundaries',
            vertical_interpolation='linear between available reference heights; below30 use30, above100 forbidden',
            height_reference='source layers above ERA5 grid terrain; local AGL mapped to these reference layers without terrain downscaling',
            missing_density_policy='no flight in a layer/hour whose density is unresolved; interpolation requires both valid endpoints',
            horizontal_interpolation='nearest retained grid cell',wind_vertical_ms=0.,wind_vertical_kind='unresolved -> declared zero reference',
            wind_limit_mps=10.,gust10_limit_mps=10.,air_temperature_C=[0.,40.],charging_temperature_C=[0.,40.],
            maximum_previous_hour_precipitation_mm=0.,require_daylight=True,forecast_error_bound_mps=0.,
            uncertainty_role='deterministic hindcast reference, not robust weather guarantee',ignore_original_M100_pass_flags=True),
        numerical=dict(position_basis='degree7_Bernstein',max_subsegments_per_transfer=3,samples_per_segment=9,
            integration='Gauss-Legendre order9 per subsegment; split at weather/charge state boundaries',position_tolerance_m=.01,
            time_tolerance_s=.001,energy_tolerance_Wh=.01,angle_tolerance_rad=.001,
            full_trajectory_verification='interval/adaptive checks required before claiming continuous feasibility'),
        planning=dict(horizon_seconds=28800.,default_start_utc='2025-01-01T08:00:00Z',reference_periods=['2025-01-01/2025-01-07','2025-07-01/2025-07-07'],
            sortie_slots='one optional slot per inspection task; not preassigned flights',fixed_six_tower_limit=False,
            successor_domain='all distinct task pairs plus launch/recovery anchors; lazy symbolic indexing, no fixed catalogue',
            maximum_vehicle_visits='2*number_of_sortie_slots+2',objective=['weighted_incomplete_towers','fleet_return_makespan_s','aircraft_energy_Wh'],
            complete_tower_rule='all9 planned image requirements; execution requires image acceptance',pure_reposition_sorties=False),
        provenance={'default_kind':'declared_project_reference_assumption',
            'transferred_geometry':'KPLC25; not the local tower as-built geometry',
            'camera_hardware_bounds':'H20; ideal intrinsics and image thresholds are assumptions',
            'battery_nominal_capacity':'M300 TB60 274Wh per battery; pair548Wh; usable capacity assumed equal nominal',
            'charging':'M30020to90percent1800s; other intervals/efficiency/control are reference assumptions',
            'weather':'existing public ERA5 source records; no M100 acceptance flags reused',
            'not_a_measured_profile':True})

def populate(s):
    name=s['scene_id'];crs=s['coordinate_reference']['projected'];epsg=int(crs.split(':')[-1])
    demfile=PACK/'sources/regional_dem'/name/'terrain.json';dem=read(demfile);terrain=Terrain(dem['results'],crs)
    g,forbidden=build_roads(s);select_stops(s,g,forbidden,terrain)
    for collection in [s['power_nodes'],s['roads']['nodes']]:
        zs=terrain.f(np.asarray([[v['easting_m'],v['northing_m']] for v in collection]))
        assert np.isfinite(zs).all()
        for v,z in zip(collection,zs):v['ground_reference_elevation_m']=float(z)
    towers=orientation(s);s['inspection_tasks']=[];s['transfer_waypoints']=[];s['tower_routes']=[]
    add_views(s,towers,[],terrain,True,epsg=epsg)
    s['parameters']=operational_parameters()
    overrides={'stop_separation_m':120.,'recipe_tower_limit':None,'uav_energy_scale':[1.,1.],
        'uav_payloads':['M300_H20_reference','M300_H20_reference'],
        'objective':['weighted_incomplete_towers','fleet_return_makespan_s','aircraft_energy_Wh'],
        'normalized_energy_equation':'not used; Wh power model in joint_reference_profile.energy'}
    initialization_only={'local_speed_mps','local_acceleration_mps2','obstacle_envelope_above_terrain_m',
        'terrain_sample_step_m','junction_candidate_radius_m','local_movement_seconds_per_tower','net_tower_seconds'}
    for p in s['parameters']:
        p['status']='applied_reference';p['application_scope']='regional_reference_simulation'
        if p['key'] in overrides:
            p.update(value=overrides[p['key']],evidence_kind='design',source_ids=['MODEL_REGIONAL'])
        if p['key'] in initialization_only:p['status']='initialization_reference_only'
        if p['key'] in {'recipe_tower_limit','normalized_energy_equation'}:p['status']='not_used_by_joint_model'
        if p['key']=='full_charge_reference_seconds':p['status']='manufacturer_reference_not_active_curve_total'
    s['reference_profiles'].update(applied=True,applied_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        purpose='nine photo tasks, fleet, time, energy and charging reference applied; real tower shape remains unknown')
    s['scene_kind']='real_geography_configured_reference_model'
    s['terrain']=dict(dataset=dem['dataset'],source_resolution_m=90,sampling_grid_spacing_m=250,
        samples=dem['results'],requests=dem['requests'],interpolation='UTM LinearNDInterpolator; extrapolation forbidden',
        source_ids=[f'TERRAIN_{name}'],status='acquired_and_bound',terrain_role=dem.get('terrain_role','DEM reference, not surveyed tower base'),
        source_url=dem['source_url'],vertical_datum=dem['vertical_datum'])
    profile=joint_profile();s['joint_reference_profile']=profile
    s['charging_profiles']=dict(active_profile='tail20_cool0',selection_status='declared_reference_selection',
        sensitivity_profiles=[dict(id=f'tail{tail}_cool0',**charge_profile(tail),source_ids=['M300','MODEL_CHARGE']) for tail in [10,20,30]],
        active_curve=charge_profile(20),partial_charging_in_model=True,hardware_control_verified=False)
    active=s['charging_profiles']['active_curve'];active['charging_queue_policy']='optimization_decision'
    active['input_power_by_SOC_interval_W']=[548*3600*(b-a)/(u-t)/.9 for a,b,t,u in zip(active['charge_soc_knots'],active['charge_soc_knots'][1:],active['charge_time_knots_seconds'],active['charge_time_knots_seconds'][1:])]
    active['power_bound_rule']='0<=P_in<=piecewise_SOC_input_power; occupied channel; supply bound; partial charge allowed'
    s['motion_and_energy_model']=dict(profile='joint_reference_profile',reference_local_net_tower_seconds=300.,
        fixed_route_role='initialization_only',power_model=profile['energy']['power_formula'],capacity_Wh=548.,
        calibration_status='explicit_reference_surrogate_not_high_accuracy_M300_prediction')
    for task in s['inspection_tasks']:
        cam=np.array([task['easting_m'],task['northing_m'],task['altitude_m']]);target=task['target'];q=np.array([target['easting_m'],target['northing_m'],target['altitude_m']])
        normal=(cam-q)/np.linalg.norm(cam-q);t1=np.array([-normal[1],normal[0],0.]);t2=np.array([0.,0.,1.])
        samples=[q]+[q+a*.25*t1+b*.25*t2 for a,b in [(-1,-1),(-1,1),(1,-1),(1,1)]]
        target.update(surface_id=task['id']+'_surface',normal_world=normal.tolist(),tangent_basis=[t1.tolist(),t2.tolist()],surface_samples_utm_m=[v.tolist() for v in samples],
            geometry_kind='declared0.5m_reference_patch_not_surveyed_component')
        task['observation_requirement']=dict(camera_model='joint_reference_profile.camera',nominal_viewpoint_role='initialization_center_not_fixed_decision',
            viewpoint_ball_radius_m=2.,min_dwell_s=task['effective_service_seconds'],weight=1.,
            weather_window_rule='all occupied UTC hours satisfy regional_reference_profile weather limits',planned_status='required_not_completed')
        task['source_ids']=sorted(set(task['source_ids']+['MODEL_REGIONAL',f'TERRAIN_{name}']))
    weather=read(ROOT/f'总结/03_实验数据/天气因素建模/scenes/{name}.json')
    weatherpath=ROOT/weather['weather_record_file']['path'];assert sha(weatherpath)==weather['weather_record_file']['sha256']
    wr=read(weatherpath);cells={tuple(x) for x in weather['requested_weather_cells']}
    keys=['grid_id','grid_latitude','grid_longitude','grid_elevation_m','season_window','time_utc','height_agl_m','wind_east_ms','wind_north_ms','wind_speed_ms','wind_gusts_10m_ms','temperature_2m_C','dew_point_2m_C','surface_pressure_hPa','pressure_proxy_hPa','temperature_proxy_C','precipitation_previous_hour_mm','is_day','air_density_proxy_kg_m3','status','vertical_model','weather_code']
    records=[{k:v[k] for k in keys} for v in wr['records'] if (v['grid_latitude'],v['grid_longitude']) in cells]
    assert sorted({v['height_agl_m'] for v in records})==profile['weather']['height_layers_agl_m']
    for r in records:r['reference_flight_allowed']=bool(r['air_density_proxy_kg_m3'] is not None and r['air_density_proxy_kg_m3']>0 and r['wind_speed_ms']<=10 and r['wind_gusts_10m_ms']<=10 and 0<=r['temperature_2m_C']<=40 and r['precipitation_previous_hour_mm']==0 and r['is_day']==1 and r['weather_code'] in {0,1,2,3})
    s['weather']=dict(mode='historical_replay',requested_cells=[list(c) for c in sorted(cells)],records=records,
        acceptance_profile='joint_reference_profile.weather',source_ids=['WEATHER_REGIONAL'],
        original_M100_acceptance_flags_not_used=True,available_at_issued_forecast=None)
    for tower in s['towers']:
        cell=min(cells,key=lambda c:(c[0]-tower['latitude'])**2+((c[1]-tower['longitude'])*math.cos(math.radians(tower['latitude'])))**2)
        tower['weather_grid_lat_lon']=list(cell)
    s['fleet']=dict(vehicles=[dict(id=f'V{v}',depot_stop=s['bases'][v-1]['road_stop'],uav_ids=[f'U{v}_{i}' for i in [1,2]],battery_pair_ids=[f'B{v}_{i}' for i in range(1,5)],charger_channels=1,handling_positions=1,supply_max_W=1000.) for v in [1,2]],
        aircraft=[dict(id=f'U{v}_{i}',carrier_id=f'V{v}',profile='joint_reference_profile.aircraft',camera='joint_reference_profile.camera') for v in [1,2] for i in [1,2]],
        battery_pairs=[dict(id=f'B{v}_{i}',carrier_id=f'V{v}',capacity_Wh=548.,initial_Wh=548.,minimum_Wh=109.6,terminal_Wh=109.6) for v in [1,2] for i in range(1,5)])
    for v,depot in zip(s['fleet']['vehicles'],s['bases']):
        node=depot['osm_node_id'];nxt=next(iter(sorted(g.successors(node))))
        yaw=math.atan2(g.nodes[nxt]['northing_m']-g.nodes[node]['northing_m'],g.nodes[nxt]['easting_m']-g.nodes[node]['easting_m'])
        v.update(initial_xyz_m=[*depot['xy_m'],depot['ground_m']],initial_heading_rad=yaw,initial_speed_mps=0.,terminal_depot_stop=depot['road_stop'])
        for u in s['fleet']['aircraft']:
            if u['carrier_id']==v['id']:u.update(launch_anchor_xyz_m=[*depot['xy_m'],depot['ground_m']+1.8],initial_velocity_mps=[0.,0.,0.],initial_yaw_rad=yaw,
                initial_gimbal_pitch_yaw_rad=[0.,0.],initial_focal_length_mm=6.83,initial_exposure_s=.001,initial_mode='stowed; flight-position constraints activate on launch')
    contract=read(ROOT/'Agent/Publication/engineering_inspection_contract.json')
    s['joint_model']=dict(model_id=contract['model_id'],equation_source='implementation/engineering_inspection_model.tex',
        equation_sha256=sha(ROOT/'Agent/Publication/engineering_inspection_model.tex'),decision_blocks=contract['variables'],constraint_families=contract['constraint_families'],
        parameter_profile='joint_reference_profile',sets=dict(towers=len(s['towers']),observations=len(s['inspection_tasks']),sortie_slots=len(s['inspection_tasks']),
            aircraft=4,vehicles=2,battery_pairs=8,stops=len(s['stops']),road_arcs=len(s['roads']['edges'])),
        data_complete_for_declared_reference_model=True,joint_solver_implemented=False,full_joint_graph_materialized=False,
        full_joint_scalar_metrics=None,field_deployment_ready=False,plan_status='not_solved',
        input_binding=dict(target_geometry='/inspection_tasks/*/target',nominal_viewpoints='/inspection_tasks',roads='/roads/routing_graph',
            vehicle_candidates='/stops',initial_state='/fleet',weather='/weather',parameters='/joint_reference_profile'),
        physical_scope='reference patch geometry, template conductors, simplified road corridors; no actual survey/aircraft calibration')
    s['obstacle_model']=dict(towers='cylinders at source tower locations with reference radius2m and height35.7378m',
        conductors='parallel reference layers along original source segments; parabolic sag5m; not measured wire geometry',
        ground='acquired DEM piecewise-linear terrain',vegetation='not present in declared reference scene; field vegetation unknown',
        reference_parameters='joint_reference_profile.geometry',source_ids=['MODEL_REGIONAL',f'OSM_{name}_power',f'TERRAIN_{name}'])
    sn={n['id']:n for n in s['power_nodes']};axes={t['source_object_id']:np.asarray(t['line_direction_utm']) for t in s['towers']}
    cylinders=[dict(id='CYL_'+t['id'],center_xy_m=[t['easting_m'],t['northing_m']],base_elevation_m=t['ground_elevation_m'],
        radius_m=2.,height_m=35.7378,source_tower_id=t['id']) for t in s['towers']]
    wires=[]
    for seg in s['power_segments']:
        delta=np.array([sn[seg['head']]['easting_m']-sn[seg['tail']]['easting_m'],sn[seg['head']]['northing_m']-sn[seg['tail']]['northing_m']]);axis=delta/np.linalg.norm(delta)
        if axis[0]<0:axis=-axis
        for k,h in enumerate(profile['geometry']['conductor_layers_m']):
            for side in [-1,1]:
                endpoints=[]
                for nid in [seg['tail'],seg['head']]:
                    n=sn[nid];a=axes.get(nid,axis);normal=np.array([-a[1],a[0]])
                    endpoints.append([*(np.array([n['easting_m'],n['northing_m']])+side*4.8768*normal),n['ground_reference_elevation_m']+h])
                wires.append(dict(id=f'WIRE_{seg["id"]}_{k}_{side}',source_segment_id=seg['id'],endpoint_xyz_m=endpoints,
                    sag_m=5.,radius_m=.05,parameter_interval=[0.,1.],curve='(1-t)*a+t*b-[0,0,4*sag*t*(1-t)]'))
    s['obstacle_model'].update(tower_cylinders=cylinders,conductor_curves=wires,source_segment_offset_correspondence='template sign alignment; actual circuit and phase geometry not surveyed')
    s['missing_parameters']=['local parking and airspace permissions','as-built tower/component/conductor and vegetation survey',
        'voltage-specific operator safety clearance','camera and mounting calibration; defect-specific image acceptance',
        'loaded M300 dynamics and power calibration','vehicle hardware/corridor survey and charger control validation',
        'issued forecast and validated weather errors','joint optimizer implementation and solved mission validation']
    s['scientific_status']='complete_declared_reference_scene_not_field_validated_not_solved'
    s['routing_adapter_status']='directed access/oneway/turn-aware graph and depot/stops configured; continuous vehicle trajectory not solved'
    s['construction']=[v for v in s['construction'] if v.get('step',0)<7]+[
        dict(step=7,input=dem['dataset'],operation='cached250m grid plus exact tower queries; UTM linear interpolation',output='/terrain,/towers/*/ground_elevation_m'),
        dict(step=8,input='OSM roads and turn restrictions',operation='access/oneway/barrier filter; largest restriction-aware strongly connected component; no invented edges',output='/roads/routing_graph'),
        dict(step=9,input='source roads and substations',operation='road access depot; nearest-tower stop candidates separated120m',output='/bases,/stops'),
        dict(step=10,input='LI23/KPLC25 and incident source lines',operation='principal-axis orientation; nine photo tasks per tower; target patches and initialization route',output='/inspection_tasks,/tower_routes'),
        dict(step=11,input='declared reference profile and retained weather',operation='bind all19 variable blocks and16 constraint families; independent M300 reference weather screen',output='/joint_model,/joint_reference_profile,/weather,/fleet')]
    s['field_provenance']=[p for p in s['field_provenance'] if p.get('path')!='/reference_profiles']+[
        dict(path='/inspection_tasks',kind='transferred_template_plus_derived',source_ids=['LI23','KPLC25','MODEL_REGIONAL',f'TERRAIN_{name}'],method='UTM template rotated with source-line principal axis, inverse-projected to WGS84'),
        dict(path='/joint_reference_profile',kind='declared_reference_parameters',source_ids=['MODEL_REGIONAL','M300','H20'],method='each profile distinguishes inherited source bounds and new numerical assumptions'),
        dict(path='/weather',kind='public_reanalysis_replay',source_ids=['WEATHER_REGIONAL'],method='raw meteorological fields; original M100 acceptance flags excluded')]
    s['statistics'].update(road_edges=len(s['roads']['edges']),routing_nodes=len(s['roads']['routing_graph']['node_ids']),candidate_stops=len(s['stops']),
        photo_tasks=len(s['inspection_tasks']),nonphoto_waypoints=len(s['transfer_waypoints']),terrain_samples=len(s['terrain']['samples']))
    return s

def main():
    impl=PACK/'implementation';impl.mkdir(exist_ok=True)
    import shutil
    for f in ['scripts/apply_regional_inspection_models.py','scripts/prepare_regional_terrain.py','scripts/export_geographic_scene_pack.py','Agent/Publication/engineering_inspection_model.tex']:
        shutil.copyfile(ROOT/f,impl/Path(f).name)
    wp=ROOT/'总结/03_实验数据/天气因素建模/weather_records.json'
    weather=read(wp)
    # The scenes embed selected records; a single pinned full source remains in the package.
    dst=PACK/'sources/regional_weather/weather_records.json';dst.parent.mkdir(parents=True,exist_ok=True)
    if not dst.exists() or sha(dst)!=sha(wp):shutil.copyfile(wp,dst)
    srcs=[source('MODEL_REGIONAL',impl/'apply_regional_inspection_models.py',None,'Regional nine-view and joint reference model implementation'),
        source('MODEL_REGIONAL_HELPER',impl/'export_geographic_scene_pack.py',None,'Extended projection-aware nine-view helper; historical exporter retained separately'),
        source('MODEL_REGIONAL_TERRAIN',impl/'prepare_regional_terrain.py',None,'Cached public terrain acquisition'),
        source('WEATHER_REGIONAL',dst,'https://open-meteo.com/en/docs/historical-weather-api','Retained ERA5-derived weather fields; source manifest and profile provenance in original record','public_reanalysis')]
    cat=read(PACK/'catalog.json');new=[]
    for name in NAMES:
        s=populate(read(PACK/'scenes'/f'{name}.json'))
        dem=PACK/'sources/regional_dem'/name/'terrain.json'
        extra=srcs+[source(f'TERRAIN_{name}',dem,read(dem)['source_url'],'Copernicus DEM 2021 GLO90; DOI10.5270/ESA-c5d3d65; per-scene API or public COG source hashes retained','public_source')]
        sd={x['id']:x for x in cat['source_catalog']};sd.update({x['id']:x for x in extra});s['source_catalog']=list(sd.values())
        s['configured_at']=datetime.datetime.now(datetime.timezone.utc).isoformat()
        out=PACK/'scenes'/f'{name}.json';write(out,s)
        c=next(c for c in cat['scenes'] if c['id']==name);c.update(sha256=sha(out),statistics=s['statistics'],configuration_status=s['scientific_status'])
        new.append(dict(scene_id=name,sha256=sha(out),statistics=s['statistics'],configured_at=s['configured_at']))
        print('CONFIGURED',name,s['statistics'],flush=True)
        srcs+=extra[-1:]
    sd={x['id']:x for x in cat['source_catalog']};sd.update({x['id']:x for x in srcs})
    cat['source_catalog']=list(sd.values());cat['updated_at']=datetime.datetime.now(datetime.timezone.utc).isoformat();cat['regional_reference_models_applied']=True
    write(PACK/'catalog.json',cat)
    rec=read(ROOT/'research_quality/regional_inspection_completion.json');rec['configured_scenes']=new;rec['status']='configured_validation_pending'
    write(ROOT/'research_quality/regional_inspection_completion.json',rec)
if __name__=='__main__':main()
