"""Independent data/model audit; no mission solve or baseline comparison."""
from pathlib import Path
from collections import Counter
import datetime,hashlib,json,math
import numpy as np
import networkx as nx
from pyproj import Transformer
from scipy.spatial import Delaunay
ROOT=Path(__file__).resolve().parents[1]
PACK=ROOT/'总结/03_实验数据/真实地理巡检场景共享'
NAMES=['rommerskirchen','brauweiler','villevaude']
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    record=read(ROOT/'research_quality/regional_inspection_completion.json');cat=read(PACK/'catalog.json');rows=[]
    for name in NAMES:
        s=read(PACK/'scenes'/f'{name}.json');G=len(s['towers']);I=len(s['inspection_tasks']);assert I==9*G
        count=Counter(t['tower_id'] for t in s['inspection_tasks']);assert set(count.values())=={9}
        by={t['id']:t for t in s['towers']};tasks={t['id']:t for t in s['inspection_tasks']}
        assert len(tasks)==I
        proj=Transformer.from_crs(4326,s['coordinate_reference']['projected'],always_xy=True)
        maxerr=0.;gsd=[];clearance=[];focus=[]
        dem=read(PACK/f'sources/regional_dem/{name}/terrain.json')
        for ref in dem['requests']:
            p=PACK/ref['path'];assert sha(p)==ref['sha256']
            if p.suffix=='.tif':
                import rasterio
                with rasterio.open(p) as ds:
                    grid=ds.read(1);tf=ds.transform
                for sample in dem['results']:
                    c,r=(~tf)*(sample['location']['lng'],sample['location']['lat']);c-=.5;r-=.5
                    j,i=math.floor(c),math.floor(r);u,v=c-j,r-i
                    z=(1-u)*(1-v)*float(grid[i,j])+u*(1-v)*float(grid[i,j+1])+(1-u)*v*float(grid[i+1,j])+u*v*float(grid[i+1,j+1])
                    assert abs(z-sample['elevation'])<1e-8
                continue
            raw=read(p)
            assert len(raw['request_coordinates_lon_lat'])==len(raw['response']['elevation'])
        demby={(r['location']['lng'],r['location']['lat']):r['elevation'] for r in dem['results']}
        for ref in dem['requests']:
            if Path(ref['path']).suffix=='.tif':continue
            raw=read(PACK/ref['path'])
            for (x,y),h in zip(raw['request_coordinates_lon_lat'],raw['response']['elevation']):assert demby[x,y]==h
        p=s['joint_reference_profile'];fx=math.hypot(5184,3888)/(2*math.tan(math.radians(66.6)/2))
        for tower in s['towers']:
            tt=[tasks[x] for x in tower['photo_task_ids']]
            assert {x['paper_point_number'] for x in tt}=={1,2,3,4,5,8,9,10,11}
            assert abs(np.linalg.norm(tower['line_direction_utm'])-1)<1e-9
        for t in s['inspection_tasks']:
            tower=by[t['tower_id']];q=t['target']
            for z in [t,q]:
                xy=proj.transform(z['longitude'],z['latitude']);maxerr=max(maxerr,math.dist(xy,[z['easting_m'],z['northing_m']]))
            assert abs(t['altitude_m']-tower['ground_elevation_m']-t['height_above_tower_ground_m'])<1e-8
            cam=np.array([t['easting_m'],t['northing_m'],t['altitude_m']]);target=np.array([q['easting_m'],q['northing_m'],q['altitude_m']])
            view=target-cam;distance=np.linalg.norm(view);focus.append(float(distance));assert 8<=distance<=25
            normal=np.array(q['normal_world']);tangent=np.array(q['tangent_basis']);assert abs(np.dot(normal,view/distance)+1)<1e-9
            assert np.max(np.abs(tangent@tangent.T-np.eye(2)))<1e-9 and np.max(np.abs(tangent@normal))<1e-9
            yaw=math.degrees(math.atan2(view[0],view[1]))%360;assert abs((yaw-t['camera_azimuth_deg']+180)%360-180)<1e-8
            samples=np.array(q['surface_samples_utm_m']);assert samples.shape==(5,3)
            local=(samples-cam)@np.array([*tangent,view/distance]).T
            uv=local[:,:2]/local[:,2,None]*fx+np.array([2592,1944]);assert (uv>32).all() and (uv<np.array([5184,3888])-32).all()
            gsd.append(float(distance/fx));assert gsd[-1]<=p['camera']['gsd_max_m_per_px']
        assert maxerr<1e-6
        nodes={x['id']:x for x in s['roads']['nodes']};segments={e['id']:e for e in s['roads']['segments']}
        edges={e['id']:e for e in s['roads']['edges']}
        for e in edges.values():
            src=segments[e['source_segment_id']];assert {src['tail'],src['head']}=={e['tail'],e['head']}
            assert abs(e['length_m']-src['length_m'])<1e-10
            if e['original_oneway'] in {'yes','true','1'}:assert e['tail']==src['tail']
            if e['original_oneway']=='-1':assert e['head']==src['tail']
            assert abs(e['seconds']*e['speed_limit_mps']-e['length_m'])<1e-8
        state=nx.DiGraph();state.add_nodes_from(edges)
        for a,b in s['roads']['routing_graph']['allowed_edge_transitions']:
            e,f=edges[a],edges[b];assert e['head']==f['tail']
            for rule in s['roads']['turn_restrictions']:
                if rule['via']!=e['head'] or rule['from_way']!=e['way_id']:continue
                match=f['way_id']==rule['to_way']
                if rule['rule']=='no_u_turn':match=match and f['head']==e['tail']
                assert not (rule['rule'].startswith('no_') and match)
                assert not (rule['rule'].startswith('only_') and not match)
            state.add_edge(a,b)
        assert nx.is_strongly_connected(state)
        for stop in s['stops']:
            n=nodes[stop['osm_node_id']];assert [n['longitude'],n['latitude']]==[stop['lon'],stop['lat']]
            assert stop['osm_node_id'] in s['roads']['routing_graph']['node_ids']
        assert len(s['fleet']['aircraft'])==4 and len(s['fleet']['vehicles'])==2 and len(s['fleet']['battery_pairs'])==8
        assert math.dist(*[v['initial_xyz_m'] for v in s['fleet']['vehicles']])>=12.
        curve=s['charging_profiles']['active_curve'];assert abs(curve['charge_time_knots_seconds'][2]-curve['charge_time_knots_seconds'][1]-1800)<1e-8
        assert max(curve['input_power_by_SOC_interval_W'])<=1000
        for a,b,t,u,power in zip(curve['charge_soc_knots'],curve['charge_soc_knots'][1:],curve['charge_time_knots_seconds'],curve['charge_time_knots_seconds'][1:],curve['input_power_by_SOC_interval_W']):
            assert abs((u-t)*power*.9/3600-(b-a)*548)<1e-9
        assert abs(p['energy']['hover_power_W']*2580/3600-548)<1e-9
        assert len(s['obstacle_model']['tower_cylinders'])==G and len(s['obstacle_model']['conductor_curves'])==8*len(s['power_segments'])
        weather=s['weather']['records'];assert weather and all(not any(k.startswith('pass') or 'eligible' in k for k in r) for r in weather)
        assert sorted({r['height_agl_m'] for r in weather})==p['weather']['height_layers_agl_m']
        for r in weather:
            expected=r['air_density_proxy_kg_m3'] is not None and r['air_density_proxy_kg_m3']>0 and r['wind_speed_ms']<=p['weather']['wind_limit_mps'] and r['wind_gusts_10m_ms']<=p['weather']['gust10_limit_mps'] and 0<=r['temperature_2m_C']<=40 and r['is_day']==1 and r['precipitation_previous_hour_mm']==0 and r['weather_code'] in {0,1,2,3}
            assert r['reference_flight_allowed']==expected
        assert all((r['air_density_proxy_kg_m3'] is not None and r['air_density_proxy_kg_m3']>0) or not r['reference_flight_allowed'] for r in weather)
        model=s['joint_model'];assert len(model['decision_blocks'])==19 and len(model['constraint_families'])==16
        assert model['data_complete_for_declared_reference_model'] and not model['joint_solver_implemented'] and not model['field_deployment_ready']
        assert model['equation_sha256']==sha(ROOT/'Agent/Publication/engineering_inspection_model.tex')
        rows.append(dict(scene_id=name,towers=G,photo_tasks=I,stops=len(s['stops']),road_arcs=len(edges),
            turn_transitions=state.number_of_edges(),terrain_samples=len(dem['results']),weather_rows=len(weather),
            unavailable_density_rows=sum(r['air_density_proxy_kg_m3'] is None for r in weather),
            allowed_reference_weather_rows=sum(r['reference_flight_allowed'] for r in weather),
            max_coordinate_error_m=maxerr,nominal_GSD_m_per_px_range=[min(gsd),max(gsd)],nominal_focus_m_range=[min(focus),max(focus)],
            all_nine_tasks_bound=True,turn_aware_connectivity=True,charge_Wh_consistency=True,status='PASS',
            nominal_viewpoint_checks='ideal reference optics only; no continuous mission feasibility claim'))
    preserved=[]
    for c in cat['scenes']:
        if c['id'] in NAMES:continue
        old=record['before'][c['id']];assert sha(PACK/c['file'])==old['sha256'];preserved.append(c['id'])
    result=dict(task_id='regional_inspection_completion',observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='PASS',scenes=rows,
        preserved_scene_hashes=preserved,scope='reference input and geometry consistency; no joint solve, field calibration or algorithm result',solver_runs=0)
    (ROOT/'research_quality/regional_inspection_validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
