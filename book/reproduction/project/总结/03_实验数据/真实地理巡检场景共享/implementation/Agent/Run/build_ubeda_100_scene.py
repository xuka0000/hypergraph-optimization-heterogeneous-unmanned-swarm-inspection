"""Build the 60/100/150-tower OSM scenes with directional, pinned geography.

The historical filename and default count remain compatible with the 100-tower
entry point. The 60-tower subset reuses the pinned 100-tower source envelope.
"""
import argparse,json,sys,time,urllib.parse,urllib.request,xml.etree.ElementTree as ET,heapq,itertools
from pathlib import Path
from math import dist,ceil
from itertools import product
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import networkx as nx
import numpy as np
from pyproj import Transformer
from Agent.Publication.geographic_inspection_scene import SOURCES,OUTPUT,DEFAULTS,xy,digest,select_stops,Terrain,build,save

RAW=SOURCES/'ubeda_100';RAW.mkdir(exist_ok=True)
TOWER_COUNT=100
CONFIG={**DEFAULTS,'horizon_minutes':240,'stop_separation_m':350.,
        'road_speeds_kmph':{'track':15.,'service':15.,'unclassified':30.,'residential':25.,
                           'tertiary':40.,'secondary':50.,'primary':50.,'living_street':10.,
                           'tertiary_link':30.,'secondary_link':30.,'primary_link':30.}}


def configure(tower_count):
    global TOWER_COUNT,RAW
    if tower_count not in (60,100,150):
        raise ValueError('Supported tower counts are 60, 100 and 150')
    TOWER_COUNT=tower_count
    RAW=SOURCES/f'ubeda_{100 if tower_count==60 else tower_count}'
    RAW.mkdir(exist_ok=True)


def selection():
    src=json.loads((OUTPUT/'expanded_geography_candidates.json').read_text(encoding='utf-8'))
    case=next(c for c in src['cases'] if c['tower_count']==TOWER_COUNT)
    by={t['osm_id']:t for t in src['tower_coordinates']}
    # Stable tower IDs follow line order, not method-dependent outcomes.
    ordered=list(dict.fromkeys(n for line in case['lines'] for n in line['tower_ids']))
    assert len(ordered)==TOWER_COUNT
    rename={n:f'T{i+1:03d}' for i,n in enumerate(ordered)}
    towers=[dict(id=rename[n],osm_id=n,lon=by[n]['lon'],lat=by[n]['lat'],xy_m=by[n]['xy_m']) for n in ordered]
    lines=[]
    for line in case['lines']:
        segments=line['source_adjacency_segments'];run=[]
        for a,b in segments:
            if run and run[-1]!=rename[a]:lines.append(run);run=[]
            if not run:run=[rename[a]]
            run.append(rename[b])
        if run:lines.append(run)
    inverse=Transformer.from_crs(32630,4326,always_xy=True)
    lon,lat=inverse.transform(*src['station']['xy_m'])
    base=[dict(id='B1',lon=lon,lat=lat,xy_m=src['station']['xy_m'],role='planning depot near mapped substation; not operator permission')]
    bbox=[min(t['lon'] for t in towers)-.012,min(t['lat'] for t in towers)-.012,
          max(t['lon'] for t in towers)+.012,max(t['lat'] for t in towers)+.012]
    return towers,lines,base,bbox,case


def fetch(url,path,kind):
    if path.exists():return
    for attempt in range(1,3):
        try:
            b=urllib.request.urlopen(urllib.request.Request(url,headers={'User-Agent':'paper11-research-scene/1.0'}),timeout=40).read()
            if kind=='xml':ET.fromstring(b)
            else:
                j=json.loads(b)
                if j.get('status')!='OK':raise ValueError(j)
            path.write_bytes(b)
            record=dict(status='ok',url=url,file=path.name,sha256=digest(path),bytes=len(b),retrieved_unix=time.time())
            with (RAW/'retrieval.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(record)+'\n')
            print('FETCHED',path.name,len(b),flush=True);return
        except Exception as e:
            if hasattr(e,'read'):
                e.detail=e.read().decode('utf-8',errors='replace')
            with (RAW/'retrieval.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(dict(status='failed',url=url,attempt=attempt,error=str(e),time=time.time()))+'\n')
            print('RETRY',path.name,attempt,str(e),getattr(e,'detail','')[:200],flush=True)
            if getattr(e,'code',None)==400:raise
            if attempt==2:raise


def fetch_roads():
    *_,bbox,_=selection();x0,y0,x1,y1=bbox
    existing=RAW/'road_manifest.json'
    if existing.exists():
        manifest=json.loads(existing.read_text())
        a,b,c,d=manifest['bbox']
        if manifest['status']!='complete' or not (a<=x0 and b<=y0 and c>=x1 and d>=y1):
            raise ValueError('Pinned road envelope does not cover requested scene')
        if not all((RAW/t['file']).exists() for t in manifest['tiles']):
            raise ValueError('Pinned road manifest has missing files')
        print('REUSE pinned road envelope',RAW,flush=True);return
    splitfile=RAW/'road_splits.json';splits=json.loads(splitfile.read_text()) if splitfile.exists() else {}
    files=[]
    def tile(box,name,depth=0):
        if name not in splits:
            try:
                fetch('https://api.openstreetmap.org/api/0.6/map?bbox='+','.join(map(str,box)),RAW/(name+'.xml'),'xml')
                files.append(dict(file=name+'.xml',bbox=box));return
            except urllib.error.HTTPError as e:
                if e.code!=400 or not any(s in getattr(e,'detail','') for s in ('50000','50,000','too many nodes')) or depth>=3:raise
                splits[name]=box;splitfile.write_text(json.dumps(splits,indent=2),encoding='utf-8')
        a,b,c,d=box
        for ix,iy in product(range(2),range(2)):
            tile([a+(c-a)*ix/2,b+(d-b)*iy/2,a+(c-a)*(ix+1)/2,b+(d-b)*(iy+1)/2],f'{name}_{ix}_{iy}',depth+1)
    for ix,iy in product(range(2),range(2)):
        box=[x0+(x1-x0)*ix/2,y0+(y1-y0)*iy/2,x0+(x1-x0)*(ix+1)/2,y0+(y1-y0)*(iy+1)/2]
        tile(box,f'roads_{ix}_{iy}')
    (RAW/'road_manifest.json').write_text(json.dumps(dict(bbox=bbox,tiles=files,status='complete'),indent=2),encoding='utf-8')


def geometry(allow_unverified_gates=False):
    towers,lines,bases,bbox,case=selection();nodes={};ways={};relations={};node_tags={}
    manifest=json.loads((RAW/'road_manifest.json').read_text());assert manifest['status']=='complete'
    for item in manifest['tiles']:
        p=RAW/item['file']
        r=ET.parse(p).getroot()
        for n in r.findall('node'):
            nodes[n.attrib['id']]=dict(lon=float(n.attrib['lon']),lat=float(n.attrib['lat']))
            node_tags[n.attrib['id']]={t.attrib['k']:t.attrib['v'] for t in n.findall('tag')}
        for w in r.findall('way'):ways[w.attrib['id']]=w
        for rel in r.findall('relation'):relations[rel.attrib['id']]=rel
    restrictions=[];excluded_turn_ways=set();unhandled=[]
    for rid,rel in relations.items():
        tags={t.attrib['k']:t.attrib['v'] for t in rel.findall('tag')}
        if tags.get('type')!='restriction':continue
        members=rel.findall('member');frm=[m.attrib['ref'] for m in members if m.attrib.get('role')=='from'];to=[m.attrib['ref'] for m in members if m.attrib.get('role')=='to'];via=[m for m in members if m.attrib.get('role')=='via']
        rule=tags.get('restriction','')
        if len(frm)!=1 or len(to)!=1 or len(via)!=1 or via[0].attrib['type']!='node' or not rule.startswith(('no_','only_')) or 'restriction:conditional' in tags:
            excluded_turn_ways.update(frm);unhandled.append(rid);continue
        restrictions.append(dict(id=rid,from_way=frm[0],to_way=to[0],via=via[0].attrib['ref'],rule=rule))
    blocked_nodes={}
    physical_barriers={'bollard','block','bus_trap','stile','turnstile','cycle_barrier','chain','fence','wall','jersey_barrier','gate','lift_gate','swing_gate'}
    for n,tags in node_tags.items():
        permission=next((tags[k] for k in ('motorcar','motor_vehicle','vehicle','access') if k in tags),None)
        gate_sensitivity=allow_unverified_gates and permission is None and tags.get('barrier') in {'gate','lift_gate','swing_gate'}
        if permission in {'no','private','agricultural','forestry','delivery','customers'} or (tags.get('barrier') in physical_barriers and permission not in {'yes','permissive','designated'} and not gate_sensitivity):
            blocked_nodes[n]=tags
    g=nx.DiGraph();excluded=[];directional=0
    for wid,w in sorted(ways.items()):
        tags={t.attrib['k']:t.attrib['v'] for t in w.findall('tag')};kind=tags.get('highway')
        if kind not in CONFIG['road_speeds_kmph']:continue
        if wid in excluded_turn_ways:
            excluded.append(dict(way=wid,reason='unsupported turn relation: exclude originating way'));continue
        # Do not silently turn conditional access into unrestricted access.
        restriction=next((tags[k] for k in ('motorcar','motor_vehicle','vehicle','access') if k in tags),'yes')
        if restriction not in {'yes','permissive','designated'} or any('conditional' in k for k in tags if k.startswith(('access','vehicle','motor_vehicle','oneway'))):
            excluded.append(dict(way=wid,reason='restricted/conditional access',tags=tags));continue
        one=tags.get('oneway','yes' if tags.get('junction')=='roundabout' else 'no')
        if one not in {'yes','1','true','-1','no','0','false'}:
            excluded.append(dict(way=wid,reason='unhandled direction',tags=tags));continue
        directional+=one in {'yes','1','true','-1'}
        ids=[n.attrib['ref'] for n in w.findall('nd')]
        for a,b in zip(ids,ids[1:]):
            if a not in nodes or b not in nodes:raise ValueError('Missing road geometry node')
            if a in blocked_nodes or b in blocked_nodes:continue
            # The map API returns whole ways beyond its requested rectangle.
            # Clip to the declared study envelope, never import a distant road tail.
            if not all(bbox[0]<=nodes[n]['lon']<=bbox[2] and bbox[1]<=nodes[n]['lat']<=bbox[3] for n in (a,b)):
                continue
            length=dist(xy(**nodes[a]),xy(**nodes[b]))
            if length<=0:continue
            speed=CONFIG['road_speeds_kmph'][kind]
            speed_tag=tags.get('maxspeed','')
            if speed_tag.isdigit():speed=min(speed,float(speed_tag))
            data=dict(length_m=length,seconds=length/(speed/3.6),way_id=wid,highway=kind,
                      access=restriction,surface=tags.get('surface','unknown'),oneway=one)
            pairs=[(b,a)] if one=='-1' else [(a,b)] if one in {'yes','1','true'} else [(a,b),(b,a)]
            for u,v in pairs:
                if not g.has_edge(u,v) or data['seconds']<g[u][v]['seconds']:g.add_edge(u,v,**data)
    for n in g:g.nodes[n].update(nodes[n],xy_m=list(xy(**nodes[n])))
    components=sorted(nx.strongly_connected_components(g),key=len,reverse=True)
    total=len(g);g=g.subgraph(components[0]).copy()
    restrictions=[r for r in restrictions if r['via'] in g]
    g.graph['turn_restrictions']=restrictions;g.graph['shortest_paths']=turn_aware_shortest_paths
    g.graph['forbidden_stops']={r['via'] for r in restrictions}
    # Disconnected mapped fragments are recorded, not bridged by invented edges.
    nearest=[min(dist(t['xy_m'],d['xy_m']) for _,d in g.nodes(data=True)) for t in towers]
    metadata=dict(directed=True,source_bbox=bbox,source_nodes=total,retained_nodes=len(g),
                  excluded_component_sizes=[len(c) for c in components[1:]],oneway_ways=directional,
                  tower_to_nearest_road_max_m=max(nearest),source_line_names=[l['name'] for l in case['lines']],
                  turn_restrictions=restrictions,unsupported_turn_relations=unhandled,
                  blocked_or_unpermitted_gate_nodes=blocked_nodes,
                  access_policy='motorcar overrides motor_vehicle/vehicle/access; restrictive/conditional ways excluded; hard barriers and gates without explicit vehicle permission excluded; otherwise untagged access remains planning-only')
    return (g,towers,lines,bases,[],excluded),metadata


def turn_allowed(g,previous,node,nxt):
    if previous is None:return True
    incoming=g[previous][node]['way_id'];outgoing=g[node][nxt]['way_id']
    for r in g.graph.get('restriction_index',{}).get((node,incoming),[]):
        matches=outgoing==r['to_way']
        if r['rule'].endswith('u_turn') and r['from_way']==r['to_way']:matches=matches and nxt==previous
        if (r['rule'].startswith('no_') and matches) or (r['rule'].startswith('only_') and not matches):return False
    return True


def turn_aware_shortest_paths(g,source,targets):
    if 'restriction_index' not in g.graph:
        index={}
        for r in g.graph['turn_restrictions']:index.setdefault((r['via'],r['from_way']),[]).append(r)
        g.graph['restriction_index']=index
    counter=itertools.count();queue=[(0.,next(counter),(None,source))];cost={(None,source):0.};parent={};found={};paths={}
    while queue and len(found)<len(targets):
        value,_,state=heapq.heappop(queue);previous,node=state
        if value!=cost[state]:continue
        if node in targets and node not in found:
            found[node]=value;p=[];cur=state
            while True:
                p.append(cur[1])
                if cur not in parent:break
                cur=parent[cur]
            paths[node]=p[::-1]
        for nxt,e in g[node].items():
            if not turn_allowed(g,previous,node,nxt):continue
            target=(node,nxt);nv=value+e['seconds']
            if nv<cost.get(target,float('inf')):
                cost[target]=nv;parent[target]=state;heapq.heappush(queue,(nv,next(counter),target))
    if len(found)!=len(targets):raise ValueError('Turn-aware reachability incomplete: do not invent road links')
    return found,paths


def fetch_terrain():
    if (RAW/'terrain.json').exists():
        print('REUSE pinned terrain grid; selected stop elevations use interpolation',digest(RAW/'terrain.json'),flush=True);return
    geo,_=geometry();g,towers,_,bases,*_=geo;stops=select_stops(g,towers,bases,CONFIG)
    positions=[t['xy_m'] for t in towers+stops]+[n['xy_m'] for n in g.nodes.values()]
    low=np.min(positions,axis=0)-100;high=np.max(positions,axis=0)+100
    # Grid covers the retained road network as well as every flight corridor.
    grid=list(product(np.arange(low[0],high[0]+500,500),np.arange(low[1],high[1]+500,500)))
    inverse=Transformer.from_crs(32630,4326,always_xy=True)
    coords=[]
    for x,y in grid+[t['xy_m'] for t in towers+stops]:
        lon,lat=inverse.transform(x,y);coords.append((round(lat,7),round(lon,7)))
    coords=list(dict.fromkeys(coords));results=[];queries=[]
    for start in range(0,len(coords),100):
        batch=coords[start:start+100]
        url='https://api.opentopodata.org/v1/srtm90m?'+urllib.parse.urlencode({'locations':'|'.join(f'{a},{b}' for a,b in batch)})
        path=RAW/f'terrain_batch_{start//100:03d}.json';fetch(url,path,'json')
        payload=json.loads(path.read_text());assert len(payload['results'])==len(batch)
        for row,(lat,lon) in zip(payload['results'],batch):
            assert abs(row['location']['lat']-lat)<1e-6 and abs(row['location']['lng']-lon)<1e-6
            assert row['elevation'] is not None
        results+=payload['results'];queries.append(url)
        print('TERRAIN',len(results),'/',len(coords),flush=True)
    (RAW/'terrain.json').write_text(json.dumps(dict(source='OpenTopoData SRTM90m',grid_spacing_m=500,requests=queries,results=results),separators=(',',':')),encoding='utf-8')


def generate():
    geo,metadata=geometry();scene=build(CONFIG,geometry=geo,terrain=Terrain(RAW/'terrain.json'),case_id=f'ubeda_osm_{TOWER_COUNT}')
    scene['roads']['directed']=True;scene['geography_source']=metadata
    scene['source_hashes']={p.name:digest(p) for p in sorted(RAW.glob('roads_*.xml'))}
    scene['source_hashes']['terrain.json']=digest(RAW/'terrain.json')
    scene['source_hashes']['expanded_geography_candidates.json']=digest(OUTPUT/'expanded_geography_candidates.json')
    scene['operational_assumptions']['depot']='both vehicles at road stop nearest mapped Ubeda substation; no parking/utility access permission inferred'
    scene['operational_assumptions']['tower_template']=f'all {TOWER_COUNT} towers use generic nine-view RGB template; actual tower types are not surveyed'
    scene['operational_assumptions']['duty_window']='240 minutes planning window fixed before outcomes; not a measured shift requirement'
    scene['operational_assumptions']['stop_heading']='Turn-restriction via nodes excluded as stops; paths preserve direction across internal junctions. Stopping and heading changes at other candidate stops remain planning assumptions.'
    samples=len(json.loads((RAW/'terrain.json').read_text(encoding='utf-8'))['results'])
    scene['operational_assumptions']['terrain_sampling']=f'Pinned {samples} SRTM90m samples: 500m grid plus tower and initial-stop samples; interpolate current stop positions. Not a fine obstacle/terrain survey.'
    scene['geography_source']['raw_source_directory']=str(RAW.relative_to(OUTPUT.parents[2]))
    scene['parameter_evidence']='inspection_parameter_evidence.json; same development configuration as ubeda_osm_100; physical and numerical evidence gaps remain'
    for stop in scene['stops']:
        if 'author_base' in stop['reason']:stop['reason']='road_stop_near_mapped_substation'
    path=save(scene);print('BUILT',path,'towers',len(scene['towers']),'stops',len(scene['stops']),'flights',len(scene['flights']),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--tower-count',type=int,choices=(60,100,150),default=100);ap.add_argument('--fetch-roads',action='store_true');ap.add_argument('--fetch-terrain',action='store_true');ap.add_argument('--build',action='store_true');ap.add_argument('--reference',action='store_true',help='Build source-bounded nine-view operating dataset and finite-battery witnesses');a=ap.parse_args()
    configure(a.tower_count)
    if a.fetch_roads:fetch_roads()
    if a.fetch_terrain:fetch_terrain()
    if a.build:
        if a.reference:
            from Agent.Run.build_operational_inspection_dataset import generate as generate_reference
            generate_reference(a.tower_count)
        else:generate()
