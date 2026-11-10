"""Build and exercise source-bounded operational datasets, without old flight costs."""
import argparse
import copy
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import networkx as nx
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from Agent.Run.build_ubeda_100_scene import turn_aware_shortest_paths,turn_allowed
from Agent.Publication.geographic_inspection_scene import Terrain,SOURCES
from Agent.Run.complete_inspection_reference_parameters import leg_seconds
from Agent.Publication.reference_inspection import VehicleClock,flight_cost,validate_events
OUT=ROOT/'Data/Scenarios/geographic_inspection'
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,d):p.write_text(json.dumps(d,ensure_ascii=False,separators=(',',':'))+'\n',encoding='utf8')


def generate(n):
    started=time.perf_counter();oldpath=OUT/f'ubeda_osm_{n}.json';oldhash=sha(oldpath);old=read(oldpath)
    geometrypath=OUT/f'ubeda_{n}_nine_view_reference.json';geometry=read(geometrypath)
    servicepath=OUT/f'ubeda_{n}_nine_view_service_parameters.json';service=read(servicepath)
    parameterpath=OUT/'inspection_operating_reference.json';parameters=read(parameterpath)
    if service['parent_sha256']!=sha(geometrypath):raise ValueError('stale service/geometry binding')
    cfg=dict(vehicles=2,uavs_per_vehicle=2,battery_pairs_per_vehicle=4,charging_pairs_per_vehicle=1,
             full_charge_seconds=3600.,preparation_seconds=120.,recovery_seconds=240.,
             reserve_fraction=.2,hover_reference_seconds=2580.,forward_reference_seconds=2700.,
             local_speed_mps=1.,local_acceleration_mps2=3.,transit_speed_mps=5.,
             road_speed_kmph=10.,terrain_sample_step_m=50.,
             uav_energy_scale=[1.,45/43],uav_payloads=['M300_H20_reference','M300_H20T_reference'],
             obstacle_envelope_above_terrain_m=35.7378+8.,endpoint_limit='all',
             recipe_tower_limit=6,objective='max_completed_towers_within_working_day',horizon_seconds=28800.,charging_on_road=False)
    basis={
      'vehicles':'resource design shared by methods','uavs_per_vehicle':'resource design shared by methods',
      'battery_pairs_per_vehicle':'resource design;2 installed+2 spare; finite stock',
      'charging_pairs_per_vehicle':'M300 manual p30, two cells as one pair',
      'full_charge_seconds':'M300 manual p30,220–240V room-temperature full-charge reference',
      'preparation_seconds':'BY26 p24 manual-operation reference; includes resetting/swapping allowance by model definition',
      'recovery_seconds':'BY26 p24 handling reference; airborne descent counted separately; transferred boundary assumption',
      'reserve_fraction':'research margin, sensitivity required; not guaranteed safe reserve',
      'hover_reference_seconds':'M300 manual p95,700g load/sea level43min maximum reference',
      'forward_reference_seconds':'M300 manual p95,700g load/sea level45min maximum reference',
      'local_speed_mps':'RISS26 preprint TableIII field limit transferred to reference platform',
      'local_acceleration_mps2':'RISS26 TableIII; rest-to-rest corner stops are model assumption',
      'transit_speed_mps':'WA20 Table1 simulation speed',
      'road_speed_kmph':'BY26 p24 simulation speed; existing stricter mapped speed limits retained',
      'uav_energy_scale':'H20T reported43min /700g reference45min ratio;45/43 applied to normalized-time energy is an explicit surrogate, not measured hover physics',
      'uav_payloads':'H20 and H20T share M300 and TB60; official H20 page and DJI comparison; nine tasks requireRGB so both compatible',
      'terrain_sample_step_m':'numerical sampling,25/50/100m convergence checked separately',
      'obstacle_envelope_above_terrain_m':'KPLC peak35.7378m plus LI23 local8m margin; conservative model extension, not obstacle survey',
      'endpoint_limit':'all retained road stops are available, no nearest3 truncation',
      'recipe_tower_limit':'finite line-consecutive catalogue up to6; LI23 reported6 is reference scale, not universal physical limit',
      'objective':'design: maximize complete equal-weight towers within the declared working day; full-coverage replay is a separate feasibility diagnostic',
      'horizon_seconds':'BY26 p24 Table5 uses8h working day; adopted reference shift, not local work order',
      'charging_on_road':'design: charging allowed only stationary, reference power/thermal availability assumed'}
    assert set(basis)==set(cfg)
    g=nx.DiGraph()
    for row in old['roads']['edges']:
        e=copy.copy(row);e['seconds']=max(e['seconds'],e['length_m']/(cfg['road_speed_kmph']/3.6))
        g.add_edge(e.pop('tail'),e.pop('head'),**e)
    g.graph['turn_restrictions']=old['geography_source']['turn_restrictions']
    stops=old['stops'];targets={s['osm_node_id'] for s in stops};roads=[]
    for i,a in enumerate(stops):
        lengths,paths=turn_aware_shortest_paths(g,a['osm_node_id'],targets)
        for b in stops:
            if a['id']==b['id']:continue
            walk=paths[b['osm_node_id']]
            assert all(turn_allowed(g,u,v,w) for u,v,w in zip(walk,walk[1:],walk[2:]))
            assert abs(sum(g[u][v]['seconds'] for u,v in zip(walk,walk[1:]))-lengths[b['osm_node_id']])<1e-6
            roads.append(dict(tail=a['id'],head=b['id'],seconds=lengths[b['osm_node_id']],osm_nodes=walk))
        if i%25==0:print(n,'ROAD',i+1,'/',len(stops),flush=True)
    terrain=Terrain(SOURCES/f'ubeda_{100 if n==60 else n}/terrain.json')
    def transition(a,b,step=None):
        horizontal=math.dist(a[:2],b[:2]);steps=max(2,math.ceil(horizontal/(step or cfg['terrain_sample_step_m']))+1)
        xy=np.linspace(a[:2],b[:2],steps);zs=terrain.linear(xy)
        if not np.all(np.isfinite(zs)):raise ValueError('transit outside terrain interpolation hull')
        ceiling=max(a[2],b[2],float(np.max(zs))+cfg['obstacle_envelope_above_terrain_m'])
        vertical=leg_seconds(ceiling-a[2])+leg_seconds(ceiling-b[2])
        cruise=leg_seconds(horizontal,cfg['transit_speed_mps'],cfg['local_acceleration_mps2'])
        return dict(seconds=vertical+cruise,energy=vertical/2580+cruise/2700,ceiling_m=ceiling)
    observations={o['id']:o for o in geometry['observations']}
    dwell={r['observation_id']:r['effective_service_seconds'] for r in service['records']}
    tower_ids={t['id']:t['observations'] for t in old['towers']}
    routes={r['tower']:r['path'] for r in service['tower_routes']}
    recipes=[]
    def body(tasks):
        clock=0.;energy=0.;events=[];previous=None
        for tower in dict.fromkeys(observations[i]['tower'] for i in tasks):
            selected=[i for i in tasks if observations[i]['tower']==tower]
            indices=[i for i,p in enumerate(routes[tower]) if p['photo'] and any(observations[o]['li2023_point_number']==p['point_number'] for o in selected)]
            path=routes[tower][min(indices):max(indices)+1]
            if previous is not None:
                hop=transition(previous,path[0]['xyz_m']);clock+=hop['seconds'];energy+=hop['energy']
            for i,p in enumerate(path):
                if i:
                    dt=leg_seconds(math.dist(path[i-1]['xyz_m'],p['xyz_m']));clock+=dt;energy+=dt/2580
                if p['photo']:
                    obs=next(o for o in selected if observations[o]['li2023_point_number']==p['point_number'])
                    clock+=dwell[obs];energy+=dwell[obs]/2580;events.append([obs,clock])
            previous=path[-1]['xyz_m']
        return dict(tasks=list(tasks),seconds=clock,energy=energy,events=events)
    # Preserve declared task catalogue identity while recomputing every cost.
    for old_recipe in old['recipes']:recipes.append(body(old_recipe['tasks']))
    endpoints=sorted({r['tasks'][0] for r in recipes}|{r['tasks'][-1] for r in recipes})
    endpoint_costs={};convergence=[]
    for j,obs in enumerate(endpoints):
        target=observations[obs]['xyz_m'];rows={}
        for s in stops:
            start=[*s['xy_m'],s['ground_m']];rows[s['id']]=transition(start,target)
        endpoint_costs[str(obs)]=rows
        # Longest link for every endpoint: check both finer and coarser sampling.
        far=max(stops,key=lambda s:math.dist(s['xy_m'],target[:2]));start=[*far['xy_m'],far['ground_m']]
        fine=transition(start,target,25);coarse=transition(start,target,100);mid=rows[far['id']]
        convergence.append(dict(observation=obs,stop=far['id'],seconds25=fine['seconds'],seconds50=mid['seconds'],seconds100=coarse['seconds']))
        if j%100==0:print(n,'ENDPOINT',j+1,'/',len(endpoints),flush=True)
    result=dict(schema='operational_reference_inspection_1',case_id=f'ubeda_reference_{n}',
                scope='public geography plus transferred engineering template; synthetic operational benchmark, not actual site flight plan',
                config=cfg,config_basis=basis,depot=old['bases'][0]['road_stop'],
                observations=[dict(o,capture_seconds=dwell[o['id']],effective_service_seconds=dwell[o['id']],
                                   time_evidence_kind='calibrated_equal_residual_not_measured',
                                   capture_field_semantics='all non-translational service overhead, not shutter time') for o in geometry['observations']],
                components=old['components'],towers=geometry['towers'],stops=stops,
                recipes=recipes,endpoint_costs=endpoint_costs,roads=roads,
                geography_parent=oldpath.name,geography_sha256=oldhash,
                geometry_parent=geometrypath.name,geometry_sha256=sha(geometrypath),
                service_parent=servicepath.name,service_sha256=sha(servicepath),
                parameter_parent=parameterpath.name,parameter_sha256=sha(parameterpath),
                payload_capacity_source=dict(path=str((ROOT/'总结/07_证据与复现索引/综合报告检索/场景设置文献/dji_m300_h20t_comparison.html').relative_to(ROOT)),
                    sha256=sha(ROOT/'总结/07_证据与复现索引/综合报告检索/场景设置文献/dji_m300_h20t_comparison.html'),
                    url='https://enterprise-insights.dji.com/blog/dji-thermal-night-vision-drones-comparison-side-by-side?hs_amp=true',
                    locator='M300RTK+H20T flight time43min; class energy rescaling is our declared approximation'),
                flight_representation='outbound[observation,stop]+recipe+inbound[observation,stop]; symmetric air transfer, all stop pairs',
                charging_model='4 full pairs initially per vehicle; full3600s charge after each sortie; one pair at a time, stationary only, chargeable-temperature assumption',
                old_solver_compatible=False,solver_api='Agent.Publication.reference_inspection.flight_cost/validate_events',
                limitations=['OSM position is not an as-built asset geometry survey.',
                             'SRTM/reference clearance is not field collision or electrical certification.',
                             'Time and normalized energy are calibrated/transferred models, not flight logs.',
                             'Finite recipe catalogue and candidate parking set define the benchmark domain; no continuous-space global-optimum claim.',
                             'H20/H20T energy heterogeneity uses a declared45/43 scaling; thermal modality is not required by theseRGB tasks.'])
    # Validate every factorized recipe has exactly the requested event list.
    for r in recipes:assert [i for i,t in r['events']]==r['tasks'] and r['events'][-1][1]==r['seconds']
    dest=OUT/f'ubeda_reference_{n}.json';write(dest,result)
    checked=run_witness(result,n,bounded=False)
    bounded_checked=run_witness(result,n,bounded=True)
    audit=dict(scene=dest.name,scene_sha256=sha(dest),counts=dict(towers=n,observations=len(observations),stops=len(stops),recipes=len(recipes),road_paths=len(roads),endpoint_entries=len(endpoints)*len(stops)),
               all_stops_available_as_endpoints=True,full_coverage_witness=checked,bounded_shift_witness=bounded_checked,
               terrain_sampling_scope='longest stop link per endpoint; not exhaustive all-link convergence',
               terrain_sampling_max_delta_25_vs_50_seconds=max(abs(x['seconds25']-x['seconds50']) for x in convergence),terrain_sampling=convergence,
               old_scene_unchanged=sha(oldpath)==oldhash,wall_seconds=time.perf_counter()-started,
               build_code_sha256=sha(Path(__file__)),model_code_sha256=sha(ROOT/'Agent/Publication/reference_inspection.py'))
    write(OUT/f'ubeda_reference_{n}_audit.json',audit)
    print('DONE',n,json.dumps(checked),json.dumps(bounded_checked),round(audit['wall_seconds'],2),flush=True)
    return audit


def run_witness(result,n,bounded):
    cfg=result['config'];stops=result['stops'];recipes=result['recipes']
    tower_ids={t['id']:t['observations'] for t in result['towers']}
    road_lookup={(r['tail'],r['head']):r for r in result['roads']}
    clocks=[VehicleClock(i+1,result['depot'],cfg) for i in range(cfg['vehicles'])]
    assignments=[];unreachable=[]
    singles={tuple(r['tasks']):i for i,r in enumerate(recipes)}
    for k,tower in enumerate(result['towers']):
        rid=singles[tuple(tower_ids[tower['id']])];c=clocks[k%len(clocks)]
        uav=len([e for e in c.events if e['kind']=='flight'])%cfg['uavs_per_vehicle']
        feasible=[s for s in stops if flight_cost(result,rid,s['id'],s['id'],uav)['energy']<=.8+1e-9]
        if not feasible:unreachable.append(tower['id']);continue
        stop=min(feasible,key=lambda s:(0 if s['id']==c.stop else road_lookup[c.stop,s['id']]['seconds'])+flight_cost(result,rid,s['id'],s['id'])['seconds'])['id']
        trial=copy.deepcopy(c);trial.inspect(result,rid,stop,road_lookup)
        home=0 if stop==result['depot'] else road_lookup[stop,result['depot']]['seconds']
        if bounded and trial.t+home>cfg['horizon_seconds']+1e-7:continue
        clocks[k%len(clocks)]=trial
        assignments.append(dict(tower=tower['id'],vehicle=c.vehicle,recipe=rid,stop=stop))
    for c in clocks:
        if c.stop!=result['depot']:c.advance(road_lookup[c.stop,result['depot']]['seconds'],'road',tail=c.stop,head=result['depot']);c.stop=result['depot']
    events=[e for c in clocks for e in c.events]
    checked=validate_events(result,events,require_all=not bounded and not unreachable,enforce_horizon=bounded)
    if unreachable:raise ValueError('Reference energy model cannot serve towers: '+str(unreachable))
    dest=OUT/f'ubeda_reference_{n}.json'
    witness=dict(scene=dest.name,scene_sha256=sha(dest),role='bounded8h feasibility witness' if bounded else 'full-coverage feasibility diagnostic outside working-day constraint',
                 not_algorithm_comparison=True,events=events,assignments=assignments,checks=checked)
    write(OUT/f'ubeda_reference_{n}_{"bounded_witness" if bounded else "witness"}.json',witness)
    return checked


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--counts',type=int,nargs='+',default=[60,100,150]);ap.add_argument('--replay-only',action='store_true');args=ap.parse_args()
    for count in args.counts:
        if args.replay_only:
            path=OUT/f'ubeda_reference_{count}.json';scene=read(path)
            scene['config'].update(objective='max_completed_towers_within_working_day',horizon_seconds=28800.)
            scene['config']['endpoint_limit']='all'
            scene['config'].update(uav_energy_scale=[1.,45/43],uav_payloads=['M300_H20_reference','M300_H20T_reference'])
            scene['config_basis'].update(uav_energy_scale='Official H20T43min/700g-reference45min ratio transferred as45/43 normalized energy scale; explicit approximation, not measured hover effect',uav_payloads='M300 withH20/H20T; shared pairedTB60; bothRGB-compatible')
            payload=ROOT/'总结/07_证据与复现索引/综合报告检索/场景设置文献/dji_m300_h20t_comparison.html'
            scene['payload_capacity_source']=dict(path=str(payload.relative_to(ROOT)),sha256=sha(payload),url='https://enterprise-insights.dji.com/blog/dji-thermal-night-vision-drones-comparison-side-by-side?hs_amp=true',locator='M300RTK+H20T43min; scaling is a declared model approximation')
            scene['limitations']=[x for x in scene['limitations'] if not x.startswith('H20 reference uses')]
            if not any(x.startswith('H20/H20T') for x in scene['limitations']):scene['limitations'].append('H20/H20T heterogeneity uses declared45/43 energy scaling, not measured hover physics; bothRGB-compatible')
            services=read(OUT/scene['service_parent']);dwell={r['observation_id']:r['effective_service_seconds'] for r in services['records']}
            for o in scene['observations']:o.update(capture_seconds=dwell[o['id']],effective_service_seconds=dwell[o['id']],time_evidence_kind='calibrated_equal_residual_not_measured',capture_field_semantics='all non-translational service overhead, not shutter time')
            scene['config_basis'].update(objective='design: maximize complete equal-weight towers in reference working day',horizon_seconds='BY26 p24 Table5:8h working day reference')
            write(path,scene)
            a=read(OUT/f'ubeda_reference_{count}_audit.json')
            a.update(scene_sha256=sha(path),full_coverage_witness=run_witness(scene,count,False),bounded_shift_witness=run_witness(scene,count,True),build_code_sha256=sha(Path(__file__)),model_code_sha256=sha(ROOT/'Agent/Publication/reference_inspection.py'))
            write(OUT/f'ubeda_reference_{count}_audit.json',a)
            print('REPLAYED',count,a['bounded_shift_witness'],flush=True)
        else:generate(count)
