"""Export pinned geographic inputs, explicit model parameters and provenance.

No optimizer, witness schedule, API query or active Data directory is used.
"""
from __future__ import annotations

import ast
import copy
import hashlib
import json
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from pyproj import Transformer
from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from Agent.Publication import geographic_inspection_scene as geo
from Agent.Run import build_ubeda_100_scene as ub
from Agent.Run.complete_inspection_reference_parameters import timing, tower_route
from Agent.Publication.inspection_charging import profile as charge_profile

BASE = ROOT / '总结/07_证据与复现索引/综合报告检索/场景设置文献'
OUT = ROOT / '总结/03_实验数据/真实地理巡检场景共享'
STAMP = datetime.now(timezone.utc).isoformat()
SOURCES = {}
COPY_MAP = {}


def load(p):
    return json.loads(Path(p).read_text(encoding='utf-8-sig'))


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def save(p, value, compact=False):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False,
                           indent=None if compact else 2,
                           separators=(',', ':') if compact else None) + '\n', encoding='utf-8')


def source(sid, path, url, title, locator='', recorded=None, expected=None,
           license=None, query=None, copy_raw=False, evidence_kind='public_source'):
    path = Path(path)
    h = sha(path)
    if expected and expected.lower() != h:
        raise ValueError(f'Source hash mismatch: {sid}')
    rel = path.relative_to(ROOT).as_posix()
    item = dict(id=sid, title=title, url=url, locator=locator,
                retrieved_at=recorded, sha256=h, bytes=path.stat().st_size,
                original_workspace_relative_path=rel, evidence_kind=evidence_kind,
                license=license, query=query, source_hash_verified=bool(expected),
                hash_verification_basis='saved provenance' if expected else 'current retained source identity')
    if copy_raw:
        relative = Path('sources') / path.relative_to(BASE)
        item['package_path'] = relative.as_posix()
        COPY_MAP[relative.as_posix()] = path
    else:
        item['package_path'] = None
    SOURCES[sid] = item
    return sid


def param(key, value, unit, kind, refs, label, locator='', status='reference', formula=None):
    return dict(key=key, label=label, value=value, unit=unit, evidence_kind=kind,
                source_ids=refs, locator=locator, status=status, formula=formula)


def init_sources():
    refs = [
        ('LI23','li2023_pdf.pdf','https://doi.org/10.3390/rs15030865',
         'Li Z, Zhang Y, Wu H, et al. Design and Application of a UAV Autonomous Inspection System for High-Voltage Power Transmission Lines. Remote Sensing, 2023, 15(3):865.',
         'Section 3.2.1, Figure 3, PDF pp5–6; Table 7, p21'),
        ('WA20','wang2020.pdf','https://doi.org/10.3390/en13020326',
         'Wang J, Wang G, Hu X, et al. Cooperative Transmission Tower Inspection with a Vehicle and a UAV in Urban Areas. Energies, 2020, 13(2):326.', 'Table 1, PDF p11'),
        ('BY26','bychkov2026.pdf','https://doi.org/10.3390/drones10070511',
         'Bychkov A, Eroshenko S, Romanov A. A Complex Analysis of Geoinformation Data for Automatic Aerial Inspection Mission Planning. Drones, 2026, 10(7):511.', 'PDF p24, Table 5; published simulation parameters'),
        ('RISS26','riss2026_insulators.pdf','https://arxiv.org/abs/2602.24011',
         'Riss V, Kratky V, Penicka R, Saska M. Autonomous Inspection of Power Line Insulators with UAV on an Unmapped Transmission Tower. arXiv:2602.24011, 2026. [Preprint]', 'PDF pp6–7, Tables I–III'),
        ('KPLC25','kplc_2025_tower_specification.pdf','https://kplc.co.ke/storage/01JQXMQ0QWCHBZKRQ1MBHAJADM.pdf',
         'Kenya Power and Lighting Company. 132 kV suspension tower standard-height engineering drawing, 2025.', 'PDF p73, Appendix 1A-1'),
        ('M300','dji_m300_manual.pdf','https://dl.djicdn.com/downloads/matrice-300/20230526UM/M300_RTK_User_Manual_EN_v3.4.pdf',
         'DJI. Matrice 300 RTK User Manual, v3.4, 2023.', 'PDF pp30,32,35,95; 220–240 V; 700 g payload reference'),
        ('H20','dji_h20_specs.html','https://enterprise.dji.com/zenmuse-h20-series/specs',
         'DJI. Zenmuse H20 Series Specifications.', 'H20: 678±5 g; 5184×3888; diagonal FOV 66.6–4 degrees'),
        ('H20T','dji_m300_h20t_comparison.html','https://enterprise-insights.dji.com/blog/dji-thermal-night-vision-drones-comparison-side-by-side?hs_amp=true',
         'DJI. Thermal and Night Vision Drones Comparison.', 'M300 RTK with H20T: reference flight time 43 minutes')]
    for sid, name, url, title, loc in refs:
        source(sid, BASE/name, url, title, loc)
    SOURCES['CAB25']=dict(id='CAB25',title='Caballero A, Roman-Escorza F J, Maza I, Ollero A. A Multi-UAV Approach for Fast Inspection of Overhead Power Lines: From Route Planning to Field Operation. Journal of Intelligent & Robotic Systems, 2025, 111:67.',url='https://doi.org/10.1007/s10846-025-02277-6',locator='Code availability; linked author repository; sample KML is not identified as the complete field-trial dataset',retrieved_at=STAMP,sha256=None,bytes=None,package_path=None,evidence_kind='publisher_metadata_verified',license='CC BY 4.0 (article)',source_hash_verified=False,hash_verification_basis='publisher page metadata checked; article PDF not bundled')
    for sid, name in [('MODEL_GEO','Agent/Publication/geographic_inspection_scene.py'),
                      ('MODEL_UBEDA','Agent/Run/build_ubeda_100_scene.py'),
                      ('MODEL_SELECTION','Agent/Run/prepare_expanded_inspection_geography.py'),
                      ('MODEL_VIEWS','Agent/Run/build_nine_view_reference.py'),
                      ('MODEL_TIMING','Agent/Run/complete_inspection_reference_parameters.py'),
                      ('MODEL_OPERATION','Agent/Run/build_operational_inspection_dataset.py'),
                      ('MODEL_CHARGE','Agent/Publication/inspection_charging.py'),
                      ('MODEL_EXPORT','scripts/export_geographic_scene_pack.py')]:
        source(sid, ROOT/name, None, name, evidence_kind='project_implementation')
        rel='implementation/'+name
        SOURCES[sid]['package_path']=rel
        COPY_MAP[rel]=ROOT/name
    original=load(geo.SOURCES/'source_manifest.json')
    for f in original['files']:
        sid={'pylons.kml':'AUTHOR_TOWERS','stations.kml':'AUTHOR_BASES','osm_map.xml':'OSM_SMALL',
             'terrain_srtm90m.json':'TERRAIN_SMALL'}.get(f['file'])
        if sid:
            source(sid,geo.SOURCES/f['file'],f['url'],f['file'],
                   'Repository commit '+original['repository_commit'] if sid.startswith('AUTHOR') else '',
                   f.get('recorded_utc',original['retrieved_utc']),f['sha256'],
                   'MIT (author repository)' if sid.startswith('AUTHOR') else 'ODbL' if sid=='OSM_SMALL' else None,
                   copy_raw=True)
            if sid.startswith('AUTHOR'):
                SOURCES[sid]['related_paper_doi']=original['paper_doi']
                SOURCES[sid]['repository_commit']=original['repository_commit']
    for name in ('source_manifest.json','LICENSE'):
        COPY_MAP['sources/caballero2025_data/'+name]=geo.SOURCES/name
    p=geo.SOURCES/'osm_ubeda_junction_source.json';m=load(p)
    source('OSM_UBEDA',geo.SOURCES/'osm_ubeda_junction_map.xml',m['url'],
           'OpenStreetMap Úbeda 132 kV lines and substation',recorded=m.get('retrieved_unix'),
           expected=m['sha256'],license='ODbL',copy_raw=True)
    COPY_MAP['sources/caballero2025_data/osm_ubeda_junction_source.json']=p
    for n in (100,150):
        folder=geo.SOURCES/f'ubeda_{n}';manifest=load(folder/'road_manifest.json')
        records={}
        f=folder/'retrieval.jsonl'
        if f.exists():
            for line in f.read_text(encoding='utf-8').splitlines():
                x=json.loads(line)
                if x.get('status')=='ok':records[x['file']]=x
        for i,tile in enumerate(manifest['tiles']):
            f=folder/tile['file'];r=records.get(f.name,{})
            url=r.get('url') or 'https://api.openstreetmap.org/api/0.6/map?bbox='+','.join(map(str,tile['bbox']))
            source(f'OSM_ROADS_{n}_{i:02}',f,url,f'Úbeda roads {n}, tile {i}',
                   recorded=r.get('retrieved_unix'),expected=r.get('sha256'),license='ODbL',copy_raw=True)
        source(f'TERRAIN_{n}',folder/'terrain.json','https://api.opentopodata.org/v1/srtm90m',
               f'Úbeda SRTM90m sampled terrain {n}', 'Pinned 500 m sampling grid plus asset samples; source dataset 90 m',copy_raw=True)
        for name in ('road_manifest.json','retrieval.jsonl'):
            f=folder/name
            if f.exists():COPY_MAP[f'sources/caballero2025_data/ubeda_{n}/'+name]=f
    for key,label,bbox in load(BASE/'topology_osm/areas.json'):
        for typ in ('power','roads'):
            prov=load(BASE/'topology_osm'/f'{key}_{typ}_provenance.json')
            source(f'OSM_{key}_{typ}',BASE/'topology_osm'/f'{key}_{typ}.json',prov['url'],
                   f'OpenStreetMap {label} {typ}',recorded=prov.get('retrieved_utc'),
                   expected=prov['sha256'],license=prov['license'],query=prov['query'],copy_raw=True)
            COPY_MAP[f'sources/topology_osm/{key}_{typ}_provenance.json']=BASE/'topology_osm'/f'{key}_{typ}_provenance.json'
    COPY_MAP['sources/topology_osm/areas.json']=BASE/'topology_osm/areas.json'


def operational_parameters():
    rows=[
        ('vehicles',2,'count','design',['MODEL_OPERATION'],'车辆数'),
        ('uavs_per_vehicle',2,'count','design',['MODEL_OPERATION'],'每车无人机数'),
        ('battery_pairs_per_vehicle',4,'pair','design',['MODEL_OPERATION'],'每车电池对数'),
        ('charging_pairs_per_vehicle',1,'pair','source',['M300'],'每车同时充电对数'),
        ('cells_per_pair',2,'cell','source',['M300'],'每对电池块数'),
        ('station_storage_cells',8,'cell','source',['M300'],'BS60储存电池块数'),
        ('launch_service_capacity_per_vehicle',1,'operation','design',['MODEL_GEO'],'每车起降作业容量'),
        ('preparation_seconds',120.,'s','transfer',['BY26','MODEL_OPERATION'],'起飞准备'),
        ('recovery_seconds',240.,'s','transfer',['BY26','MODEL_OPERATION'],'回收作业'),
        ('horizon_seconds',28800.,'s','transfer',['BY26'],'工作时段'),
        ('hover_reference_seconds',2580.,'s','source',['M300'],'700g载荷悬停参考续航'),
        ('forward_reference_seconds',2700.,'s','source',['M300'],'700g载荷前飞参考续航'),
        ('reserve_fraction',.2,'fraction','design',['MODEL_OPERATION'],'能量预留比例'),
        ('uav_energy_scale',[1.,45/43],'ratio','model',['M300','H20T','MODEL_OPERATION'],'两类载荷能耗倍率'),
        ('local_speed_mps',1.,'m/s','transfer',['RISS26'],'塔内速度'),
        ('local_acceleration_mps2',3.,'m/s2','transfer',['RISS26'],'塔内加速度'),
        ('transit_speed_mps',5.,'m/s','transfer',['WA20'],'塔间飞行速度'),
        ('road_speed_kmph',10.,'km/h','transfer',['BY26'],'道路参考速度'),
        ('net_tower_seconds',300.,'s','transfer',['LI23'],'单塔净作业参考时间'),
        ('camera_center_offset_m',15.,'m','design',['MODEL_VIEWS'],'拍摄位至塔中心水平距离'),
        ('tower_peak_height_m',35.7378,'m','transfer',['KPLC25'],'参考塔顶相对高度'),
        ('crossarm_heights_m',[34.2138,31.0134,25.908,20.8026],'m','transfer',['KPLC25'],'地线和三相横担高度'),
        ('phase_crossarm_half_length_m',4.8768,'m','transfer',['KPLC25'],'相线横担半长'),
        ('groundwire_crossarm_half_length_m',4.8768,'m','inferred',['KPLC25','MODEL_VIEWS'],'地线横担半长 对齐推定'),
        ('transfer_above_groundwire_m',8.,'m','transfer',['LI23'],'跨塔过渡相对地线高度'),
        ('obstacle_envelope_above_terrain_m',43.7378,'m','model',['KPLC25','LI23','MODEL_OPERATION'],'区域障碍包络假设'),
        ('terrain_sample_step_m',50.,'m','numerical',['MODEL_OPERATION'],'航段地形检查步长'),
        ('stop_separation_m',350.,'m','numerical',['MODEL_UBEDA'],'候选停靠点间距'),
        ('junction_candidate_radius_m',600.,'m','numerical',['MODEL_GEO'],'路口候选距塔半径'),
        ('recipe_tower_limit',6,'count','catalogue_design',['LI23','MODEL_OPERATION'],'连续杆塔任务组合上限'),
        ('endpoint_limit','all','category','design',['MODEL_OPERATION'],'任务端点候选停靠点'),
        ('full_charge_reference_seconds',3600.,'s','source',['M300'],'全充参考耗时'),
        ('charge_20_to_90_reference_seconds',1800.,'s','source',['M300'],'20至90百分比充电参考耗时'),
        ('charging_on_road',False,'boolean','design',['MODEL_OPERATION'],'行驶中充电'),
        ('battery_supply_voltage_v',[220,240],'V','reference_condition',['M300'],'充电供电参考范围'),
        ('camera_resolution_px',[5184,3888],'pixel','source',['H20'],'H20变焦相机分辨率'),
        ('camera_diagonal_fov_deg',[66.6,4.0],'degree','source',['H20'],'广角和长焦端对角视场'),
        ('camera_min_telephoto_focus_m',8.,'m','source',['H20'],'长焦端最近对焦距离'),
        ('camera_payload_mass_g',678.,'g','source',['H20'],'H20名义质量'),
        ('photo_tasks_per_tower',9,'count','workflow_transfer',['LI23'],'每塔拍摄作业数'),
        ('tower_completion','ALL_9','category','design',['MODEL_OPERATION'],'整塔完成规则'),
        ('tower_weight',1.,'weight','design',['MODEL_OPERATION'],'杆塔权重'),
        ('uav_payloads',['M300_H20_reference','M300_H20T_reference'],'category','reference',['MODEL_OPERATION','H20','H20T'],'无人机载荷配置'),
        ('objective','max_completed_towers_within_working_day','category','design',['MODEL_OPERATION'],'优化目标定义'),
        ('initial_battery_state',1.,'SOC fraction','design',['MODEL_OPERATION'],'全部电池对初始满电'),
        ('time_representation','continuous','seconds','numerical',['MODEL_TIMING'],'时间表达'),
        ('additional_automatic_swap_seconds',0.,'s','model_boundary',['MODEL_TIMING'],'额外自动换电耗时'),
        ('component_standoff_m',10.1232,'m','derived',['KPLC25','MODEL_VIEWS'],'侧向拍摄点至横担端点距离'),
        ('camera_mass_tolerance_g',5.,'g','source',['H20'],'H20质量公差'),
        ('groundwire_transfer_height_m',42.2138,'m','derived',['KPLC25','LI23','MODEL_TIMING'],'6、7号过渡点离塔基高度'),
    ]
    result=[param(*r) for r in rows]
    t=timing()
    result.append(param('nontranslation_service_seconds_per_photo',t['effective_service_seconds'],'s','calibrated',['LI23','RISS26','MODEL_TIMING'],'每项非平移服务时间',formula='(300 - sum(rest-to-rest route motion seconds))/9'))
    result.append(param('normalized_energy_equation','scale*(local_and_vertical_seconds/2580 + transit_seconds/2700)','fraction','model',['M300','MODEL_OPERATION'],'归一化能耗代理公式'))
    result.append(param('local_movement_seconds_per_tower',t['moving_seconds'],'s','derived',['MODEL_TIMING'],'单塔路线平移总时间'))
    return result


def base_scene(sid,label,country,epsg,kind):
    return dict(schema_version='1.0.0',scene_id=sid,name=label,country=country,
                export_observed_at=STAMP,source_snapshot='2026-11-05/06 retained downloads',
                scene_kind=kind,coordinate_reference=dict(geographic='EPSG:4326',
                projected=f'EPSG:{epsg}',lon_lat_order=['longitude','latitude'],
                longitude_unit='degree_east',latitude_unit='degree_north',xy_unit='m',z_unit='m',
                projected_axis_order=['easting','northing'],vertical_datum=None,
                vertical_datum_status='not independently verified in retained source metadata'),
                towers=[],power_nodes=[],power_ways=[],power_segments=[],substations=[],roads={},terrain={},
                stops=[],bases=[],inspection_tasks=[],transfer_waypoints=[],tower_routes=[],
                parameters=[],construction=[],field_provenance=[],missing_parameters=[],
                source_catalog=[],statistics={},scientific_status='development_data_not_field_validated',
                licenses=[dict(dataset='OpenStreetMap',attribution='© OpenStreetMap contributors',
                license='ODbL 1.0',url='https://www.openstreetmap.org/copyright')])


def finish(scene, source_ids):
    source_ids=set(source_ids)
    for p in scene['parameters']:source_ids.update(p['source_ids'])
    if 'reference_profiles' in scene:
        for p in scene['reference_profiles'].get('available_parameters',[]):source_ids.update(p['source_ids'])
    scene['source_catalog']=[copy.deepcopy(SOURCES[i]) for i in sorted(source_ids)]
    scene['statistics'].update(towers=len(scene['towers']),power_nodes=len(scene['power_nodes']),
        power_ways=len(scene['power_ways']),power_segments=len(scene['power_segments']),
        road_nodes=len(scene['roads'].get('nodes',[])),road_edges=len(scene['roads'].get('edges',[])),
        road_segments=len(scene['roads'].get('segments',[])),candidate_stops=len(scene['stops']),
        photo_tasks=len(scene['inspection_tasks']),nonphoto_waypoints=len(scene['transfer_waypoints']),
        terrain_samples=len(scene['terrain'].get('samples',[])))
    points=np.array([[t['longitude'],t['latitude']] for t in scene['towers']])
    scene['tower_bounds_lon_lat']=[*points.min(axis=0).tolist(),*points.max(axis=0).tolist()]
    scene['field_provenance'] += [
        dict(path='/coordinate_reference/projected',kind='derived',source_ids=['MODEL_EXPORT'],method='UTM zone of study area; pyproj always_xy=True'),
        dict(path='/statistics',kind='derived',source_ids=['MODEL_EXPORT'],method='Length and set operations on exported entities'),
        dict(path='/parameters/*',kind='per_field',source_ids=[],method='Each parameter carries evidence_kind, unit and source_ids'),
        dict(path='/towers/*/actual_height_m',kind='source_or_unknown',source_ids=[],method='Only numeric original OSM height; otherwise null; reference template never written here'),
        dict(path='/towers/*/actual_tower_type',kind='source_or_unknown',source_ids=[],method='Only original source design/type tag; otherwise null')]
    scene['missing_parameters'] += ['local parking permission','site-validated safe clearance',
        'field image acceptance and defect detection rate','actual flight logs and calibrated energy model']
    save(OUT/'scenes'/f"{scene['scene_id']}.json",scene,compact=True)
    print('EXPORTED',scene['scene_id'],scene['statistics'],flush=True)
    return scene


def decimal_tag(tags,key):
    try:return float(tags[key].strip().removesuffix('m').strip())
    except (KeyError,ValueError,AttributeError):return None


def add_views(scene,towers,lines,terrain,modern):
    by={t['id']:t for t in towers};neighbors=defaultdict(list)
    for line in lines:
        for a,b in zip(line,line[1:]):neighbors[a].append(b);neighbors[b].append(a)
    inv=Transformer.from_crs(32630,4326,always_xy=True)
    model=timing();lookup={x['number']:x for x in model['route']}
    heights=[34.2138,31.0134,25.908,20.8026] if modern else [36.,30.,24.,18.]
    numbers=[1,2,3,4,5,8,9,10,11]
    roles=['overview']+[f'{side}_{level}' for side in ('left','right') for level in ('groundwire','upper','middle','lower')]
    for t in towers:
        near=neighbors[t['id']]
        if near:
            delta=np.array(by[near[-1]]['xy_m'])-np.array(by[near[0]]['xy_m'] if len(near)>1 else t['xy_m'])
            direction=delta/np.linalg.norm(delta)
        else:direction=np.array([1.,0.])
        normal=np.array([-direction[1],direction[0]])
        t['line_direction']=direction.tolist();t['adjacent_towers']=list(dict.fromkeys(near))
        xyz0=[*t['xy_m'],float(terrain(t['xy_m']))]
        t['ground_m']=xyz0[2]
        out=next(x for x in scene['towers'] if x['id']==t['id'])
        out.update(ground_elevation_m=xyz0[2],line_direction_utm=direction.tolist(),
                   orientation_rule='first-to-last listed neighbour; sole-neighbour bearing; isolated east',
                   reference_geometry_profile='KPLC132kV_template' if modern else 'legacy_generic_template',photo_task_ids=[])
        for k,(num,role) in enumerate(zip(numbers,roles)):
            if k==0:
                cam=np.array(t['xy_m'])-15*direction;target=np.array(t['xy_m']);h=heights[0]
            else:
                sign=(1 if k<=4 else -1) if modern else (-1 if k<=4 else 1)
                cam=np.array(t['xy_m'])+sign*15*normal
                target=np.array(t['xy_m'])+(sign*4.8768*normal if modern else 0)
                h=heights[(k-1)%4]
            lon,lat=inv.transform(*cam);tlon,tlat=inv.transform(*target)
            obs=f"{t['id']}_P{num:02d}";out['photo_task_ids'].append(obs)
            yaw=(math.degrees(math.atan2(target[0]-cam[0],target[1]-cam[1]))+360)%360
            scene['inspection_tasks'].append(dict(id=obs,tower_id=t['id'],paper_point_number=num,role=role,
                longitude=lon,latitude=lat,easting_m=float(cam[0]),northing_m=float(cam[1]),
                altitude_m=xyz0[2]+h,height_above_tower_ground_m=h,
                target=dict(longitude=tlon,latitude=tlat,easting_m=float(target[0]),northing_m=float(target[1]),altitude_m=xyz0[2]+h),
                camera_azimuth_deg=yaw,camera_elevation_deg=0.,modality='RGB',
                effective_service_seconds=model['effective_service_seconds'] if modern else 20.,
                time_evidence_kind='calibrated_equal_residual' if modern else 'legacy_design',
                geometry_evidence_kind='transferred_engineering_template' if modern else 'declared_generic_template',
                source_ids=['LI23','KPLC25','MODEL_VIEWS','MODEL_TIMING'] if modern else ['LI23','MODEL_GEO']))
        if modern:
            route=[]
            for rp in model['route']:
                lx,ly,h=rp['xyz'];p=np.array(t['xy_m'])+lx*direction+ly*normal;lon,lat=inv.transform(*p)
                row=dict(point_number=rp['number'],longitude=lon,latitude=lat,easting_m=float(p[0]),northing_m=float(p[1]),
                         altitude_m=xyz0[2]+h,photo=rp['photo'],incoming_seconds=rp.get('incoming_seconds',0.))
                route.append(row)
                if not rp['photo']:scene['transfer_waypoints'].append(dict(id=f"{t['id']}_X{rp['number']}",tower_id=t['id'],**row,
                    evidence_kind='project_detour' if rp['number']=='overview_detour' else 'LI23_local_cross_tower_reference'))
            scene['tower_routes'].append(dict(tower_id=t['id'],points=route,net_reference_seconds=model['total_seconds']))


def export_geo_common(scene,g,towers,lines,bases,terrain_file,metadata,modern,source_ids,selection):
    terrain=geo.Terrain(terrain_file)
    cfg=ub.CONFIG if modern else geo.DEFAULTS
    stops=geo.select_stops(g,towers,bases,cfg)
    for t in towers:
        tags=t.get('tags',{})
        scene['towers'].append(dict(id=t['id'],source_object_id=str(t.get('osm_id',t['id'])),
            source_system='OSM' if modern else 'author_KML',longitude=t['lon'],latitude=t['lat'],
            easting_m=float(t['xy_m'][0]),northing_m=float(t['xy_m'][1]),original_tags=tags,
            actual_height_m=decimal_tag(tags,'height'),actual_tower_type=tags.get('design'),
            source_ids=['OSM_UBEDA'] if modern else ['AUTHOR_TOWERS']))
    add_views(scene,towers,lines,terrain,modern)
    for s in stops:
        s=copy.deepcopy(s);s['ground_m']=float(terrain(s['xy_m']));scene['stops'].append(s)
    scene['bases']=bases
    scene['power_nodes']=[dict(id=t['id'],longitude=t['lon'],latitude=t['lat'],source_object_id=str(t.get('osm_id',t['id']))) for t in towers]
    for i,line in enumerate(lines):
        scene['power_ways'].append(dict(id=f'L{i+1:03}',node_ids=line))
        for j,(a,b) in enumerate(zip(line,line[1:])):
            scene['power_segments'].append(dict(id=f'L{i+1:03}_E{j+1:03}',tail=a,head=b,
                length_m=math.dist(next(t['xy_m'] for t in towers if t['id']==a),next(t['xy_m'] for t in towers if t['id']==b)),
                adjacency='source_consecutive',source_ids=['OSM_UBEDA'] if modern else ['AUTHOR_TOWERS']))
    edges=[]
    for a,b,e in g.edges(data=True):
        row=dict(tail=str(a),head=str(b),**copy.deepcopy(e))
        if modern:row['seconds']=max(row['seconds'],row['length_m']/(10/3.6))
        edges.append(row)
        if not g.is_directed():edges.append(dict(row,tail=str(b),head=str(a)))
    scene['roads']=dict(representation='directed_model_graph',directed=True,
        nodes=[dict(id=str(n),longitude=d['lon'],latitude=d['lat'],easting_m=d['xy_m'][0],northing_m=d['xy_m'][1]) for n,d in g.nodes(data=True)],
        edges=edges,turn_restrictions=metadata.get('turn_restrictions',[]),filter_metadata=metadata,
        travel_time_evidence='transferred 10 km/h cap plus original stricter mapped/design speed' if modern else 'legacy highway-class assumed speed',
        all_pairs_paths=None,all_pairs_paths_status='not materialized; compute on this directed graph with turn restrictions',
        original_source_elements_in_package=True)
    terr=load(terrain_file)
    scene['terrain']=dict(dataset='SRTM90m',source_resolution_m=90,
        interpolation='scipy LinearNDInterpolator; NearestND fallback is detectable and audited',
        sampling_grid_spacing_m=terr.get('grid_spacing_m'),samples=terr['results'],
        request_urls=terr.get('requests',[]),source_ids=[f'TERRAIN_{100 if len(towers)==60 else len(towers)}'] if modern else ['TERRAIN_SMALL'])
    if modern:
        scene['parameters']=operational_parameters()
        scene['motion_and_energy_model']=dict(source_ids=['MODEL_TIMING','MODEL_OPERATION'],rest_to_rest_seconds='2*sqrt(d/a) if d<=v*v/a else d/v+v/a',distance_unit='m',speed_unit='m/s',acceleration_unit='m/s2',transit_ceiling='max(start_altitude,end_altitude,max(sampled_terrain)+43.7378)',transit_vertical_seconds='leg_seconds(ceiling-start_altitude,1,3)+leg_seconds(ceiling-end_altitude,1,3)',transit_horizontal_seconds='leg_seconds(horizontal_distance,5,3)',energy='scale*(local_and_vertical_seconds/2580+horizontal_transit_seconds/2700)',allowable_energy_fraction=.8,terrain_sample_count='max(2,ceil(horizontal_distance/50)+1)',air_transfer_symmetric=True,active_flight_catalogue=None,model_status='reference_inputs_only; uncalibrated physical energy')
        scene['charging_profiles']=dict(active_profile=None,selection_status='no unique sensitivity profile selected',
            historical_fixed_full_charge=dict(full_charge_seconds=3600.,status='historical model not residual dependent',source_ids=['M300','MODEL_OPERATION']),
            sensitivity_profiles=[dict(id=f'tail{x}_cool0',**charge_profile(x),source_ids=['M300','MODEL_CHARGE']) for x in (10,20,30)])
        scene['missing_parameters']+=['selected charging sensitivity profile','calibrated TB60 charge curve','per-tower as-built height and geometry']
        source_ids+=['KPLC25','LI23','M300','H20','H20T','WA20','BY26','RISS26','MODEL_CHARGE','MODEL_OPERATION','MODEL_VIEWS','MODEL_TIMING']
    else:
        labels={'horizon_minutes':'工作时段','vehicle_count':'车辆数','uavs_per_vehicle':'每车无人机数','tower_height_m':'通用塔高设定','inspection_heights_m':'通用侧面层高'}
        for k,v in geo.DEFAULTS.items():
            unit='m' if k.endswith('_m') else 's' if k.endswith('_seconds') else 'min' if 'minutes' in k else 'm/s' if k.endswith('_mps') else 'km/h' if k=='road_speeds_kmph' else 'count_or_category'
            scene['parameters'].append(param(k,v,unit,'legacy_design',['MODEL_GEO'],labels.get(k,k),status='historical_reference_only'))
        scene['parameters'] += [param('photo_tasks_per_tower',9,'count','workflow_transfer',['LI23'],'每塔拍摄作业数'),
            param('photo_point_numbers',[1,2,3,4,5,8,9,10,11],'index','source',['LI23'],'拍摄航点编号')]
        source_ids+=['LI23','MODEL_GEO']
        scene['charging_profiles']=None
        scene['missing_parameters']+=['finite battery and charging adaptation to historical prototype','as-built tower geometry']
    scene['selection']=selection
    scene['construction']=[
        dict(step=1,input='retained source snapshots',operation='verify SHA256 and source identity',output='/source_catalog'),
        dict(step=2,input='tower coordinates and source adjacency',operation=selection,output='/towers,/power_segments'),
        dict(step=3,input='WGS84 coordinates',operation='EPSG:32630 projection, always_xy=True',output='easting_m,northing_m'),
        dict(step=4,input='OSM roads',operation='original road/access/oneway/turn filter; no invented connection',output='/roads'),
        dict(step=5,input='retained SRTM samples',operation='piecewise linear ground interpolation',output='/towers/*/ground_elevation_m'),
        dict(step=6,input='source roads and towers',operation='original candidate-stop rule; mapped location, no parking permission inferred',output='/stops,/bases'),
        dict(step=7,input='LI23 plus explicit geometry profile',operation='nine model photo positions per tower; WGS84 camera and target coordinates',output='/inspection_tasks'),
        dict(step=8,input='existing model code and public references',operation='export values, units, evidence type and applicability',output='/parameters')]
    scene['field_provenance']=[
        dict(path='/towers/*/{longitude,latitude}',kind='source',source_ids=['OSM_UBEDA'] if modern else ['AUTHOR_TOWERS'],method='unchanged source coordinate; original KML 5 m merge for prototypes'),
        dict(path='/towers/*/ground_elevation_m',kind='derived',source_ids=scene['terrain']['source_ids'],method='projected linear terrain interpolation'),
        dict(path='/roads',kind='source_plus_filter',source_ids=[i for i in source_ids if i.startswith('OSM_ROADS') or i=='OSM_SMALL']+['MODEL_UBEDA' if modern else 'MODEL_GEO'],method='retained original geometry/filter'),
        dict(path='/inspection_tasks',kind='model_geometry',source_ids=['LI23','KPLC25','MODEL_VIEWS'] if modern else ['LI23','MODEL_GEO'],method='camera/target positions are derived reference template, not measured point coordinates'),
        dict(path='/stops',kind='derived_candidate',source_ids=['MODEL_GEO'],method='actual OSM nodes; site stopping permission unknown')]
    scene['routing_adapter_status']='geography and model inputs exported; no solver or candidate-flight catalogue exported'
    return finish(scene,source_ids+['MODEL_EXPORT'])


def author_scenes():
    for size in (6,27):
        g,towers,lines,bases,merges,excluded=geo.source_geometry()
        if size==6:
            selected={'T01','T02','T03','T11','T12','T23'}
            towers=[t for t in towers if t['id'] in selected];runs=[]
            for line in lines:
                run=[]
                for n in line:
                    if n in selected:run.append(n)
                    elif run:runs.append(run);run=[]
                if run:runs.append(run)
            lines=runs
        assert len(towers)==size
        scene=base_scene(f'ubeda_author_{size:03}',f'作者KML地理样例 {size}塔（西班牙）','Spain',32630,'author_geographic_sample_with_legacy_model')
        scene['name_scope']='ID ubeda_author is a package alias; the author KML area differs from the OSM Úbeda 60/100/150 area; no municipality attribution asserted'
        scene['coordinate_merges']=merges
        selection='author KML coordinates; <=5 m canonical merge; fixed six IDs T01,T02,T03,T11,T12,T23' if size==6 else 'author KML coordinates; <=5 m canonical merge; all 27 canonical towers'
        yield export_geo_common(scene,g,towers,lines,bases,geo.SOURCES/'terrain_srtm90m.json',
            dict(excluded_road_way_ids=excluded,turn_restrictions=[],undirected_source=True),False,
            ['AUTHOR_TOWERS','AUTHOR_BASES','OSM_SMALL','TERRAIN_SMALL','MODEL_GEO','CAB25'],selection)


def ubeda_sources():
    root=ET.parse(geo.SOURCES/'osm_ubeda_junction_map.xml').getroot()
    tags=lambda x:{t.attrib['k']:t.attrib['v'] for t in x.findall('tag')}
    nodes={x.attrib['id']:dict(osm_id=x.attrib['id'],lon=float(x.attrib['lon']),lat=float(x.attrib['lat']),tags=tags(x)) for x in root.findall('node')}
    ways={x.attrib['id']:dict(osm_id=x.attrib['id'],nodes=[a.attrib['ref'] for a in x.findall('nd')],tags=tags(x)) for x in root.findall('way')}
    station=ways['415865753'];center=np.mean([geo.xy(nodes[n]['lon'],nodes[n]['lat']) for n in station['nodes'][:-1]],axis=0)
    lines=[w for w in ways.values() if w['tags'].get('power')=='line' and w['tags'].get('voltage')=='132000']
    assert len(lines)==3
    ids={n for w in lines for n in w['nodes'] if nodes[n]['tags'].get('power')=='tower'}
    for n in ids:
        nodes[n]['xy_m']=list(geo.xy(nodes[n]['lon'],nodes[n]['lat']))
        nodes[n]['distance_to_station_m']=float(np.linalg.norm(np.array(nodes[n]['xy_m'])-center))
    ordered=sorted(ids,key=lambda n:(nodes[n]['distance_to_station_m'],n))
    return nodes,lines,station,center,ordered


def ubeda_scenes():
    nodes,source_lines,station,center,ordered=ubeda_sources()
    inv=Transformer.from_crs(32630,4326,always_xy=True)
    for size in (60,100,150):
        chosen=set(ordered[:size]);case_lines=[]
        for w in source_lines:
            case_lines.append(dict(osm_way_id=w['osm_id'],name=w['tags']['name'],voltage_v=132000,
                tower_ids=[n for n in w['nodes'] if n in chosen],source_adjacency_segments=[[a,b] for a,b in zip(w['nodes'],w['nodes'][1:]) if a in chosen and b in chosen]))
        sequence=list(dict.fromkeys(n for w in case_lines for n in w['tower_ids']));rename={n:f'T{i+1:03}' for i,n in enumerate(sequence)}
        towers=[dict(id=rename[n],**copy.deepcopy(nodes[n])) for n in sequence];lines=[]
        for w in case_lines:
            run=[]
            for a,b in w['source_adjacency_segments']:
                if run and run[-1]!=rename[a]:lines.append(run);run=[]
                if not run:run=[rename[a]]
                run.append(rename[b])
            if run:lines.append(run)
        lon,lat=inv.transform(*center);bases=[dict(id='B1',lon=lon,lat=lat,xy_m=center.tolist(),source_osm_way_id='415865753',role='mean of mapped substation polygon vertices')]
        bbox=[min(t['lon'] for t in towers)-.012,min(t['lat'] for t in towers)-.012,max(t['lon'] for t in towers)+.012,max(t['lat'] for t in towers)+.012]
        # Use the existing road filter unchanged, with the original deterministic selection supplied in memory.
        ub.configure(size)
        ub.selection=lambda:(copy.deepcopy(towers),copy.deepcopy(lines),copy.deepcopy(bases),bbox,dict(lines=case_lines))
        (g,ts,ls,bs,_,_),metadata=ub.geometry()
        scene=base_scene(f'ubeda_osm_{size:03}',f'Úbeda {size}塔','Spain',32630,'real_geography_with_transferred_operating_reference')
        scene['selected_source_lines']=case_lines;scene['study_bbox_w_s_e_n']=bbox
        scene['substations']=[dict(id='415865753',original_tags=station['tags'],longitude=lon,latitude=lat,
            geometry_lon_lat=[[nodes[n]['lon'],nodes[n]['lat']] for n in station['nodes']],source_ids=['OSM_UBEDA'])]
        scene['statistics']['eligible_source_towers']=len(ordered)
        source_n=100 if size==60 else size
        ids=['OSM_UBEDA',f'TERRAIN_{source_n}','MODEL_GEO','MODEL_UBEDA','MODEL_SELECTION']+[k for k in SOURCES if k.startswith(f'OSM_ROADS_{source_n}_')]
        yield export_geo_common(scene,g,ts,ls,bs,geo.SOURCES/f'ubeda_{source_n}/terrain.json',metadata,True,ids,
            'nearest N mapped 132 kV towers to mean of substation polygon vertices; distance in EPSG:32630; tie by OSM node ID string; preserve original way order and adjacency')


def regional_geometry(raw,bbox,epsg,source_id,kind):
    s,w,n,e=bbox;project=Transformer.from_crs(4326,epsg,always_xy=True)
    nodes={str(x['id']):x for x in raw['elements'] if x['type']=='node'}
    inside=lambda i:s<=nodes[i]['lat']<=n and w<=nodes[i]['lon']<=e
    ways=[];segments=[];used=set()
    for way in raw['elements']:
        if way['type']!='way':continue
        tags=way.get('tags',{})
        if kind=='power' and (tags.get('power')!='line' or tags.get('line') in ('bay','busbar')):continue
        if kind=='roads' and 'highway' not in tags:continue
        segids=[]
        for k,(a,b) in enumerate(zip(way['nodes'],way['nodes'][1:])):
            a,b=str(a),str(b)
            if a not in nodes or b not in nodes or not inside(a) or not inside(b):continue
            pa=project.transform(nodes[a]['lon'],nodes[a]['lat']);pb=project.transform(nodes[b]['lon'],nodes[b]['lat'])
            if pa==pb:continue
            eid=f"{way['id']}:{k}";segids.append(eid);used.update((a,b))
            segments.append(dict(id=eid,tail=a,head=b,osm_way_id=str(way['id']),source_segment_index=k,
                length_m=math.dist(pa,pb),source_ids=[source_id],original_tags=copy.deepcopy(tags)))
        if segids:ways.append(dict(id=str(way['id']),original_tags=copy.deepcopy(tags),segment_ids=segids,source_ids=[source_id]))
    rows=[]
    for i in sorted(used,key=int):
        x=nodes[i];xx,yy=project.transform(x['lon'],x['lat'])
        rows.append(dict(id=i,longitude=x['lon'],latitude=x['lat'],easting_m=xx,northing_m=yy,
                         original_tags=x.get('tags',{}),source_ids=[source_id]))
    return rows,ways,segments,nodes


def crossings(scene,polygons,epsg):
    by={x['id']:x for x in scene['power_nodes']};unique={}
    for seg in scene['power_segments']:
        key=tuple(sorted((seg['tail'],seg['head'])))
        unique.setdefault(key,seg)
    segments=list(unique.values())
    gs=[LineString([(by[s['tail']]['easting_m'],by[s['tail']]['northing_m']),
                    (by[s['head']]['easting_m'],by[s['head']]['northing_m'])]) for s in segments]
    tree=STRtree(gs);old={id(g):i for i,g in enumerate(gs)};inv=Transformer.from_crs(epsg,4326,always_xy=True)
    pts={}
    for i,g in enumerate(gs):
        for candidate in tree.query(g):
            j=int(candidate) if isinstance(candidate,(int,np.integer)) else old[id(candidate)]
            if j<=i or not g.crosses(gs[j]):continue
            p=g.intersection(gs[j])
            if p.geom_type!='Point':continue
            key=(round(p.x,3),round(p.y,3))
            row=pts.setdefault(key,dict(easting_m=p.x,northing_m=p.y,segment_pairs=[],inside_mapped_substation=any(q.covers(p) for q in polygons)))
            row['segment_pairs'].append([segments[i]['id'],segments[j]['id']])
    result=[]
    for i,(_,row) in enumerate(sorted(pts.items())):
        lon,lat=inv.transform(row['easting_m'],row['northing_m'])
        result.append(dict(id=f'X{i+1:04}',longitude=lon,latitude=lat,**row))
    return result


def complex_scenes():
    for key,label,bbox in load(BASE/'topology_osm/areas.json'):
        epsg=32631 if key=='villevaude' else 32632
        sid=f'OSM_{key}_power';rid=f'OSM_{key}_roads'
        raw=load(BASE/'topology_osm'/f'{key}_power.json')
        pnodes,pways,psegs,allnodes=regional_geometry(raw,bbox,epsg,sid,'power')
        scene=base_scene(key,label,'France' if key=='villevaude' else 'Germany',epsg,'real_geographic_candidate')
        scene['study_bbox_s_w_n_e']=bbox
        scene['power_nodes'],scene['power_ways'],scene['power_segments']=pnodes,pways,psegs
        for node in pnodes:
            tags=node['original_tags']
            if tags.get('power')!='tower':continue
            scene['towers'].append(dict(id='OSM_'+node['id'],source_object_id=node['id'],source_system='OSM',
                longitude=node['longitude'],latitude=node['latitude'],easting_m=node['easting_m'],northing_m=node['northing_m'],
                original_tags=tags,actual_height_m=decimal_tag(tags,'height'),actual_tower_type=tags.get('design'),
                ground_elevation_m=None,reference_geometry_profile=None,photo_task_ids=None,source_ids=[sid]))
        project=Transformer.from_crs(4326,epsg,always_xy=True);polygons=[]
        for x in raw['elements']:
            if x['type']!='way' or x.get('tags',{}).get('power')!='substation':continue
            ids=[str(i) for i in x['nodes']]
            if len(ids)<4 or ids[0]!=ids[-1] or any(i not in allnodes for i in ids):continue
            coords=[[allnodes[i]['lon'],allnodes[i]['lat']] for i in ids]
            poly=Polygon([project.transform(*p) for p in coords])
            if poly.is_valid:polygons.append(poly)
            scene['substations'].append(dict(id=str(x['id']),geometry_lon_lat=coords,original_tags=x.get('tags',{}),source_ids=[sid]))
        roadraw=load(BASE/'topology_osm'/f'{key}_roads.json')
        rn,rw,rs,_=regional_geometry(roadraw,bbox,epsg,rid,'roads')
        scene['roads']=dict(representation='mapped_geometry_not_routing_graph',nodes=rn,ways=rw,segments=rs,
            routing_graph=None,turn_restrictions_applied=False,access_policy_applied=False,
            source_relations=[x for x in roadraw['elements'] if x['type']=='relation'],
            original_tags_preserved=True,source_ids=[rid])
        scene['terrain']=dict(dataset=None,samples=[],status='not_acquired_for_this_region')
        scene['geometric_crossings']=crossings(scene,polygons,epsg)
        scene['statistics']['geometric_crossings_outside_mapped_substations']=sum(not x['inside_mapped_substation'] for x in scene['geometric_crossings'])
        scene['statistics']['geometric_crossings_all']=len(scene['geometric_crossings'])
        scene['historical_crossing_count']={'rommerskirchen':24,'brauweiler':2,'villevaude':46}[key]
        scene['crossing_recovery_status']='recomputed from retained raw geometry; historical derived files and scripts absent; no bitwise reconstruction claim'
        scene['crossing_semantics']='proper planar segment intersection; not electrical connectivity or a UAV collision; duplicate location rounded to 1 mm'
        scene['parameters']=[param('photo_tasks_per_tower',None,'count','unknown',[],'当前每塔拍摄任务数',status='not_assigned'),
            param('terrain_elevation_m',None,'m','unknown',[],'地面高程',status='not_acquired'),
            param('vehicles',None,'count','unknown',[],'车辆资源',status='not_assigned'),
            param('horizon_seconds',None,'s','unknown',[],'作业时段',status='not_assigned')]
        scene['reference_profiles']=dict(applied=False,profile_id='ubeda_operating_reference',
            available_parameters=operational_parameters(),purpose='reference only; no per-tower geometry or timing assigned to this region')
        scene['missing_parameters']=['per-tower validated geometry and orientation','DEM terrain samples',
            'candidate parking and depot selection','fleet and task assignment','road routing/access adaptation','per-tower photo coordinates']
        scene['construction']=[
            dict(step=1,input='retained Overpass queries and JSON',operation='verify source SHA256',output='/source_catalog'),
            dict(step=2,input='fixed S,W,N,E rectangle',operation='retain non-bay/non-busbar power-line segments with both endpoints in rectangle',output='/power_segments'),
            dict(step=3,input='retained line endpoints',operation='unique power=tower OSM nodes; no invented connections',output='/towers'),
            dict(step=4,input='WGS84 coordinates',operation=f'UTM projection EPSG:{epsg}',output='easting_m,northing_m'),
            dict(step=5,input='retained road ways',operation='clip at endpoint selection; preserve access, direction, surface and maxspeed tags',output='/roads'),
            dict(step=6,input='line segments and mapped substation polygons',operation='proper geometric crossings; 1 mm location deduplication; flag station interior',output='/geometric_crossings'),
            dict(step=7,input='existing known parameters',operation='unknown operational fields null; literature reference profile unapplied',output='/parameters,/reference_profiles')]
        scene['field_provenance']=[dict(path='/towers/*/{longitude,latitude,original_tags}',kind='source',source_ids=[sid],method='OSM node identity; exact source coordinate'),
            dict(path='/power_segments',kind='source_plus_filter',source_ids=[sid,'MODEL_EXPORT'],method='consecutive source nodes, both endpoints within bbox'),
            dict(path='/roads',kind='source_plus_filter',source_ids=[rid,'MODEL_EXPORT'],method='consecutive mapped road nodes; no routing suitability inference'),
            dict(path='/geometric_crossings',kind='derived',source_ids=[sid,'MODEL_EXPORT'],method='Shapely proper intersections; station polygon flag; 1 mm deduplication'),
            dict(path='/reference_profiles',kind='unapplied_reference',source_ids=['LI23','KPLC25','MODEL_OPERATION'],method='not assigned to these towers')]
        yield finish(scene,[sid,rid,'MODEL_EXPORT'])


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    init_sources()
    scenes=list(author_scenes())+list(ubeda_scenes())+list(complex_scenes())
    by={s['scene_id']:s for s in scenes}
    germany=len({t['source_object_id'] for t in by['rommerskirchen']['towers']} & {t['source_object_id'] for t in by['brauweiler']['towers']})
    for rel,src in COPY_MAP.items():
        dst=OUT/rel;dst.parent.mkdir(parents=True,exist_ok=True)
        if not dst.exists() or sha(dst)!=sha(src):shutil.copy2(src,dst)
    catalog=dict(schema_version='1.0.0',created_at=STAMP,scenes=[dict(id=s['scene_id'],name=s['name'],file=f"scenes/{s['scene_id']}.json",sha256=sha(OUT/'scenes'/f"{s['scene_id']}.json"),statistics=s['statistics']) for s in scenes],
                 dependencies=dict(ubeda_author_006_subset_of='ubeda_author_027',ubeda_osm_nested=[60,100,150],german_shared_tower_count=germany),
                 source_catalog=list(SOURCES.values()),unknown_value=None,
                 raw_source_change=False,optimizer_executed=False,field_validation=False)
    save(OUT/'catalog.json',catalog)
    print('CATALOG',len(scenes),'GERMAN_OVERLAP',germany,flush=True)


if __name__=='__main__':
    main()
