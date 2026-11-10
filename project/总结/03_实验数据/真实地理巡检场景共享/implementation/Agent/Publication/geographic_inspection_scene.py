"""Source-bound geographic inspection instance; no proposed optimizer is called.

WGS84 source geometry, local metric calculations, seconds in the interchange
file. The legacy adapter explicitly converts to km/minute layers. Terrain is
SRTM, tower shape and operational allowances are declared planning assumptions.
"""
from collections import Counter, defaultdict
from hashlib import sha256
from itertools import product
from math import ceil, cos, dist, radians, sin
from pathlib import Path
from types import SimpleNamespace as NS
import json
import xml.etree.ElementTree as ET

import networkx as nx
import numpy as np
from pyproj import Transformer
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ROOT / '总结/07_证据与复现索引/综合报告检索/场景设置文献/caballero2025_data'
OUTPUT = ROOT / 'Data/Scenarios/geographic_inspection'
VERSION = 'geographic-inspection-1'
NS_KML = {'k': 'http://www.opengis.net/kml/2.2'}
PROJECT = Transformer.from_crs(4326, 32630, always_xy=True)

DEFAULTS = dict(
    horizon_minutes=120, vehicle_count=2, uavs_per_vehicle=2,
    operation_capacity=1, preparation_seconds=60, recovery_seconds=60,
    turnaround_seconds=180, transit_speed_mps=8., inspection_speed_mps=2.,
    climb_speed_mps=2., capture_seconds=20., tower_height_m=36.,
    inspection_heights_m=[36., 30., 24., 18.], side_offset_m=15.,
    terrain_clearance_m=50., reserve_fraction=.2,
    flight_allowance_minutes=[20., 30.], nearest_stops_per_endpoint=3,
    max_towers_per_sortie=6, merge_tower_tolerance_m=5.,
    stop_separation_m=120., road_speeds_kmph={'track': 15., 'service': 15., 'unclassified': 30.},
)


def digest(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def xy(lon, lat):
    return tuple(float(v) for v in PROJECT.transform(lon, lat))


def kml_sequences(path):
    root = ET.parse(path).getroot()
    return [[tuple(map(float, token.split(',')[:2])) for token in c.text.split()]
            for c in root.findall('.//k:coordinates', NS_KML)]


def source_geometry(config=None):
    config = {**DEFAULTS, **(config or {})}
    root = ET.parse(SOURCES / 'osm_map.xml').getroot()
    nodes = {n.attrib['id']: dict(lon=float(n.attrib['lon']), lat=float(n.attrib['lat']))
             for n in root.findall('node')}
    graph = nx.Graph()
    excluded = []
    for way in root.findall('way'):
        tags = {t.attrib['k']: t.attrib['v'] for t in way.findall('tag')}
        kind = tags.get('highway')
        if kind not in config['road_speeds_kmph']:
            continue
        if any(tags.get(k) in {'no', 'private'} for k in ('access', 'vehicle', 'motor_vehicle')):
            excluded.append(way.attrib['id'])
            continue
        if tags.get('oneway') in {'yes', '1', '-1'}:
            raise ValueError('Directed-road adapter required; do not silently ignore one-way roads')
        ids = [n.attrib['ref'] for n in way.findall('nd')]
        for a, b in zip(ids, ids[1:]):
            if a not in nodes or b not in nodes:
                continue
            pa, pb = xy(**nodes[a]), xy(**nodes[b])
            length = dist(pa, pb)
            if length <= 0:
                continue
            seconds = length / (config['road_speeds_kmph'][kind] / 3.6)
            data = dict(length_m=length, seconds=seconds, way_id=way.attrib['id'],
                        highway=kind, access=tags.get('access', 'not_tagged'),
                        surface=tags.get('surface', 'unknown'))
            if not graph.has_edge(a, b) or graph[a][b]['seconds'] > seconds:
                graph.add_edge(a, b, **data)
    for n in graph:
        graph.nodes[n].update(nodes[n], xy_m=list(xy(**nodes[n])))
    towers, lines, merges = [], [], []
    for sequence in kml_sequences(SOURCES / 'pylons.kml'):
        line = []
        for lon, lat in sequence:
            p = xy(lon, lat)
            close = [t for t in towers if dist(t['xy_m'], p) <= config['merge_tower_tolerance_m']]
            if close:
                t = close[0]
                merges.append(dict(source_lon_lat=[lon, lat], canonical=t['id'], distance_m=dist(t['xy_m'], p)))
            else:
                t = dict(id=f'T{len(towers)+1:02d}', lon=lon, lat=lat, xy_m=list(p))
                towers.append(t)
            if not line or line[-1] != t['id']:
                line.append(t['id'])
        lines.append(line)
    bases = [dict(id=f'B{i+1}', lon=p[0][0], lat=p[0][1], xy_m=list(xy(*p[0])))
             for i, p in enumerate(kml_sequences(SOURCES / 'stations.kml'))]
    return graph, towers, lines, bases, merges, excluded


def terrain_requests():
    graph, towers, _, bases, _, _ = source_geometry()
    coords = [(n['lat'], n['lon']) for n in graph.nodes.values()]
    coords += [(t['lat'], t['lon']) for t in towers + bases]
    lo = np.min([t['xy_m'] for t in towers], axis=0) - 800
    hi = np.max([t['xy_m'] for t in towers], axis=0) + 800
    inverse = Transformer.from_crs(32630, 4326, always_xy=True)
    for x, y in product(np.linspace(lo[0], hi[0], 13), np.linspace(lo[1], hi[1], 13)):
        lon, lat = inverse.transform(x, y)
        coords.append((lat, lon))
    return list(dict.fromkeys((round(lat, 7), round(lon, 7)) for lat, lon in coords))


class Terrain:
    def __init__(self, path=None):
        rows = json.loads(Path(path or SOURCES / 'terrain_srtm90m.json').read_text(encoding='utf-8'))['results']
        points = [xy(r['location']['lng'], r['location']['lat']) for r in rows]
        values = [r['elevation'] for r in rows]
        if any(v is None for v in values):
            raise ValueError('Terrain contains missing elevations')
        self.linear = LinearNDInterpolator(points, values)
        self.nearest = NearestNDInterpolator(points, values)

    def __call__(self, p):
        v = float(self.linear(p[0], p[1]))
        return float(self.nearest(p[0], p[1])) if not np.isfinite(v) else v


def select_stops(graph, towers, bases, config):
    selected = []
    forbidden=graph.graph.get('forbidden_stops',set())
    allowed=[n for n in graph if n not in forbidden]
    # All stops are actual OSM nodes; snapping does not assert lawful parking.
    def add(n, reason):
        if n not in [s['osm_node_id'] for s in selected]:
            selected.append(dict(id=f'P{len(selected)+1:02d}', osm_node_id=n,
                                 xy_m=graph.nodes[n]['xy_m'], lon=graph.nodes[n]['lon'],
                                 lat=graph.nodes[n]['lat'], reason=reason,
                                 parking_status='planning_candidate_not_field_verified'))
    for b in bases:
        n = min(allowed, key=lambda n: dist(graph.nodes[n]['xy_m'], b['xy_m']))
        add(n, 'nearest_road_node_to_author_base_' + b['id'])
        b['road_stop'] = next(s['id'] for s in selected if s['osm_node_id'] == n)
        b['snap_distance_m'] = dist(graph.nodes[n]['xy_m'], b['xy_m'])
    for t in towers:
        n = min(allowed, key=lambda n: dist(graph.nodes[n]['xy_m'], t['xy_m']))
        if min(dist(graph.nodes[n]['xy_m'], s['xy_m']) for s in selected) >= config['stop_separation_m']:
            add(n, 'road_access_near_' + t['id'])
    topology=graph.to_undirected() if graph.is_directed() else graph
    for n, degree in sorted(topology.degree()):
        if n in forbidden:continue
        if degree >= 3 and min(dist(graph.nodes[n]['xy_m'], t['xy_m']) for t in towers) <= 600:
            if min(dist(graph.nodes[n]['xy_m'], s['xy_m']) for s in selected) >= config['stop_separation_m']:
                add(n, 'road_junction')
    return selected


def build(config=None, tower_subset=None, geometry=None, terrain=None, case_id=None):
    config = {**DEFAULTS, **(config or {})}
    graph, towers, lines, bases, merges, excluded = geometry or source_geometry(config)
    terrain = terrain or Terrain()
    if tower_subset is not None:
        allowed = set(tower_subset)
        towers = [t for t in towers if t['id'] in allowed]
        runs=[]
        for line in lines:
            run=[]
            for t in line:
                if t in allowed:
                    run.append(t)
                elif run:
                    runs.append(run); run=[]
            if run: runs.append(run)
        lines=runs
    stops = select_stops(graph, towers, bases, config)
    if not (nx.is_strongly_connected(graph) if graph.is_directed() else nx.is_connected(graph)):
        raise ValueError('Disconnected source roads require an explicit access decision')
    for obj in towers + stops:
        obj['ground_m'] = terrain(obj['xy_m'])
    by_tower = {t['id']: t for t in towers}
    neighbors = defaultdict(list)
    for line in lines:
        for a, b in zip(line, line[1:]):
            neighbors[a].append(b); neighbors[b].append(a)
    observations, components = [], []
    for t in towers:
        near = neighbors[t['id']]
        if not near:
            direction = np.array([1., 0.])
        else:
            delta = np.array(by_tower[near[-1]]['xy_m']) - np.array(by_tower[near[0]]['xy_m'] if len(near)>1 else t['xy_m'])
            direction = delta / np.linalg.norm(delta)
        normal = np.array([-direction[1], direction[0]])
        t['line_direction'] = direction.tolist()
        t['adjacent_towers'] = list(dict.fromkeys(near))
        t['template_status'] = 'generic_nine_view_template_not_measured_tower_type'
        t['orientation_rule'] = 'first_to_last_listed_neighbor; sole_neighbor_bearing; isolated_east'
        tasks = []
        # Li2023 Fig3: point1 + points2-5 + points8-11; 6/7 are transit only.
        roles = [('top', 0., config['tower_height_m'])]
        roles += [(f'left_{name}', -1., h) for name, h in zip(['ground_wire','upper','middle','lower'], config['inspection_heights_m'])]
        roles += [(f'right_{name}', 1., h) for name, h in zip(['ground_wire','upper','middle','lower'], config['inspection_heights_m'])]
        for role, side, h in roles:
            offset = normal * config['side_offset_m'] * (side or 1.)
            if side == 0:
                offset = direction * -config['side_offset_m']
            p = np.array(t['xy_m']) + offset
            i = len(observations) + 1
            observations.append(dict(id=i, tower=t['id'], role=role,
                                     xyz_m=[float(p[0]), float(p[1]), t['ground_m'] + h],
                                     target_xyz_m=[*t['xy_m'], t['ground_m'] + h],
                                     capture_seconds=config['capture_seconds'], modality='rgb',
                                     geometry_source='Li2023 workflow + declared generic tower template'))
            tasks.append(i)
        t['observations'] = tasks
        components.append(dict(id=t['id'], required_observations=tasks, completion='ALL', priority=1.))
    by_obs = {o['id']: o for o in observations}
    road_paths = []
    for a in stops:
        shortest=graph.graph.get('shortest_paths')
        lengths, paths = shortest(graph,a['osm_node_id'],{s['osm_node_id'] for s in stops}) if shortest else nx.single_source_dijkstra(graph, a['osm_node_id'], weight='seconds')
        for b in stops:
            if a == b:
                continue
            nodes = paths[b['osm_node_id']]
            road_paths.append(dict(tail=a['id'], head=b['id'], seconds=lengths[b['osm_node_id']],
                                   length_m=sum(graph[u][v]['length_m'] for u,v in zip(nodes,nodes[1:])),
                                   osm_nodes=nodes))
    fleet = []
    for vehicle in range(1, config['vehicle_count']+1):
        for kind in range(config['uavs_per_vehicle']):
            allowance = config['flight_allowance_minutes'][kind % 2]
            fleet.append(dict(id=len(fleet)+1, vehicle=vehicle,
                              profile=['short_endurance','long_endurance'][kind%2],
                              planning_flight_allowance_minutes=allowance,
                              usable_airborne_seconds=allowance*60*(1-config['reserve_fraction']),
                              sensor_types=['rgb']))
    # Reuse safe above-template transit for every link. Not a field-certified path.
    transition_cache = {}
    def transition(a, b):
        key = (tuple(a), tuple(b))
        if key in transition_cache:
            return transition_cache[key]
        horizontal = dist(a[:2], b[:2])
        samples = np.linspace(a[:2], b[:2], max(2, int(ceil(horizontal/50))+1))
        height = max(a[2], b[2], max(terrain(p) for p in samples) + config['terrain_clearance_m'])
        path = [list(a), [a[0], a[1], height], [b[0], b[1], height], list(b)]
        seconds = (height-a[2]+height-b[2])/config['climb_speed_mps'] + horizontal/config['transit_speed_mps']
        transition_cache[key] = (seconds, path)
        return seconds, path
    def observation_sequence(tasks):
        seconds = 0.; events=[]; path=[]; previous=None
        for task in tasks:
            o=by_obs[task]; p=o['xyz_m']
            if previous is not None:
                last=by_obs[previous]
                # Same side vertical move, otherwise cross above the tower/terrain.
                if last['tower']==o['tower'] and last['role'].split('_')[0]==o['role'].split('_')[0]:
                    seconds += dist(last['xyz_m'],p)/config['inspection_speed_mps'];path.append(p)
                else:
                    dt, segment=transition(last['xyz_m'],p);seconds+=dt;path.extend(segment[1:])
            else:
                path.append(p)
            seconds += o['capture_seconds'];events.append([task,seconds]);previous=task
        return seconds,events,path
    recipes=set()
    for t in towers:
        tasks=tuple(t['observations'])
        recipes.update([tasks, tasks[:5], tasks[5:]])
    # Contiguous line sequences, both directions. This is a finite candidate
    # catalogue, not a physical ban on longer/more general tours.
    for line in lines:
        for size in range(2,min(len(line),config['max_towers_per_sortie'])+1):
            for start in range(len(line)-size+1):
                seq=line[start:start+size]
                for order in (seq,list(reversed(seq))):
                    recipes.add(tuple(i for t in order for i in by_tower[t]['observations']))
    flights=[]; rejected=Counter(); recipe_rows=[]
    for ri,tasks in enumerate(sorted(recipes)):
        body,events,path=observation_sequence(tasks)
        launch_stops=sorted(stops,key=lambda s:dist([*s['xy_m'],s['ground_m']],by_obs[tasks[0]]['xyz_m']))[:config['nearest_stops_per_endpoint']]
        recovery_stops=sorted(stops,key=lambda s:dist([*s['xy_m'],s['ground_m']],by_obs[tasks[-1]]['xyz_m']))[:config['nearest_stops_per_endpoint']]
        recipe_rows.append(dict(id=ri,tasks=list(tasks),body_seconds=body,path_xyz_m=path,
                                events_seconds=events))
        for launch,recovery in product(launch_stops,recovery_stops):
            out, p_out=transition([*launch['xy_m'],launch['ground_m']],by_obs[tasks[0]]['xyz_m'])
            back, p_back=transition(by_obs[tasks[-1]]['xyz_m'],[*recovery['xy_m'],recovery['ground_m']])
            duration=out+body+back
            for uav in fleet:
                if ceil(duration/60)*60 > uav['usable_airborne_seconds']+1e-9:
                    rejected[uav['profile']]+=1;continue
                fid=f'F{len(flights)+1:05d}'
                flights.append(dict(id=fid,recipe=ri,vehicle=uav['vehicle'],uav=uav['id'],
                                    launch=launch['id'],recovery=recovery['id'],duration_seconds=duration,
                                    task_ids=list(tasks),completion_offsets_seconds=[[i,out+t] for i,t in events],
                                    normalized_airtime=duration/(60*uav['planning_flight_allowance_minutes']),
                                    outbound_xyz_m=p_out,inbound_xyz_m=p_back))
    graph_nodes=[dict(id=n,**d,ground_m=terrain(d['xy_m'])) for n,d in graph.nodes(data=True)]
    graph_edges=[dict(tail=a,head=b,**d) for a,b,d in graph.edges(data=True)]
    scene=dict(schema=VERSION,case_id=case_id or ('caballero_osm_full' if tower_subset is None else 'caballero_osm_small'),
               data_role='development_scenario_validation',coordinate_reference='EPSG:32630',
               source_kind='public_geography_with_declared_inspection_and_operational_assumptions',
               config=config,towers=towers,lines=lines,bases=bases,stops=stops,observations=observations,
               components=components,fleet=fleet,roads=dict(nodes=graph_nodes,edges=graph_edges,paths=road_paths),
               recipes=recipe_rows,flights=flights,merges=merges,excluded_access_ways=excluded,
               rejected_airtime_by_profile=dict(rejected),
               source_hashes={name:digest(SOURCES/name) for name in
                              ('pylons.kml','stations.kml','UAV_database.m','osm_map.xml','terrain_srtm90m.json')},
               operational_assumptions=dict(battery_supply='sufficient fully charged spare packs; replace each sortie',
                   turnaround='180 seconds for pack replacement and preparation; may overlap vehicle motion',
                   depot='both vehicles start/end at B1-snapped road stop; B2 is a stop only',
                   base_access='off-road access from original B1/B2 coordinates is not modelled',
                   tower_template='nine generic required RGB views per tower, including branch tower T10; not as-built survey'),
               limitations=['Parking permission and road driveability are unverified.',
                            'Tower type, height, camera dwell, speeds and handling times are declared assumptions.',
                            'SRTM90m is a coarse terrain model, not a structure/vegetation point cloud.',
                            'Finite flight catalogue is not an exhaustive continuous trajectory space.',
                            'No wind, failure, communication or arbitrary time-window constraints were added.',
                            'No proposed-algorithm advantage has been measured.'])
    return scene


def native_instance(scene, horizon_minutes=None):
    """Adapter to existing domain; no old single-observation decoder is invoked."""
    from Mask.Geometry import Point3D, Polyline3D
    from Mask.Scenario import RoadNode,RoadEdge,RoadNetwork,CandidateStop,UavSpec,VehicleSpec,PhysicalServiceCandidate
    from Mask.Vehicle import VehicleState
    from Mask.UAV import UavState
    from Env.State import RollingState
    from Agent.Publication.task_groups import ObservationRequirement,TaskGroup,TaskGroupModel
    cfg=scene['config']; horizon=horizon_minutes or cfg['horizon_minutes']
    depot=scene['bases'][0]['road_stop']; rename=lambda n:'sink' if n==depot else n
    p3=lambda a:Point3D(*(float(v)/1000 for v in a))
    stops={s['id']:s for s in scene['stops']}; rn={n['id']:n for n in scene['roads']['nodes']}
    nodes=tuple(RoadNode(rename(s['id']),p3([*s['xy_m'],s['ground_m']])) for s in stops.values())
    edges=[]
    for i,path in enumerate(scene['roads']['paths']):
        points=tuple(p3([*rn[n]['xy_m'],rn[n]['ground_m']]) for n in path['osm_nodes'])
        geometry=Polyline3D(points)
        # Metric closure preserves static shortest road travel time and actual
        # geometry, avoiding rounding every tiny OSM geometry segment separately.
        speed=geometry.length_3d/(path['seconds']/3600)
        edges.append(RoadEdge(f'osm_path_{i}',rename(path['tail']),rename(path['head']),geometry,
                              'osm_shortest_path',speed,'static',True))
    vehicles=tuple(VehicleSpec(v,'sink',float(horizon),cfg['operation_capacity'],1.)
                   for v in range(1,cfg['vehicle_count']+1))
    uavs=tuple(UavSpec(u['id'],u['vehicle'],cfg['transit_speed_mps']*3.6,1.,cfg['reserve_fraction'],('rgb',))
               for u in scene['fleet'])
    tasks=tuple(NS(tower_id=str(o['id']),location=p3(o['xyz_m']),service_minutes=cfg['capture_seconds']/60,risk=0.)
                for o in scene['observations'])
    candidates=tuple(PhysicalServiceCandidate(f['id'],f['vehicle'],f['uav'],rename(f['launch']),rename(f['recovery']),
                        tuple(map(str,f['task_ids'])),f['duration_seconds']/60,
                        tuple((str(i),t/60) for i,t in f['completion_offsets_seconds']),
                        f['normalized_airtime'],0.,f['normalized_airtime'],True) for f in scene['flights'])
    physical=NS(road=RoadNetwork(nodes,tuple(edges),'sink'),
                stops=tuple(CandidateStop(n.node_id,n.location,road_node_id=n.node_id) for n in nodes),
                power=NS(towers=tasks),vehicles=vehicles,uavs=uavs,candidates=candidates)
    task_ids=frozenset(o['id'] for o in scene['observations'])
    state=RollingState(0,1,frozenset(),task_ids,
                       {v.vehicle_id:VehicleState(v.vehicle_id,'sink',1,remaining_duty_time=float(horizon)) for v in vehicles},
                       {u.uav_id:UavState(u.uav_id,u.owner_vehicle_id,1,remaining_energy=1.) for u in uavs})
    instance=NS(split='validation',seed=0,instance_id=scene['case_id'],physical_scenario=physical,rolling_state=state,
                miss_penalty={i:1. for i in task_ids},energy_alpha=.9,road_beta_w=0.,road_beta_eta=0.,
                preparation_layers=ceil(cfg['preparation_seconds']/60),
                recovery_service_layers=ceil(cfg['recovery_seconds']/60),
                turnaround_layers=ceil(cfg['turnaround_seconds']/60))
    groups=TaskGroupModel(tuple(TaskGroup(c['id'],tuple(ObservationRequirement(f"obs_{i}",(i,)) for i in c['required_observations']),
                                        c['priority'],1+horizon,source='Li2023 fine-inspection workflow; generic tower dimensions declared')
                                for c in scene['components']),1)
    return instance,groups


def save(scene):
    OUTPUT.mkdir(parents=True,exist_ok=True)
    path=OUTPUT/(scene['case_id']+'.json')
    path.write_text(json.dumps(scene,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
    return path
