"""Extract an OSM-backed larger geography, not an executable flight scenario."""
import json,sys,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
import networkx as nx
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from Agent.Publication.geographic_inspection_scene import ROOT,SOURCES,OUTPUT,xy


def main():
    source=SOURCES/'osm_ubeda_junction_map.xml';root=ET.parse(source).getroot()
    tags=lambda e:{t.attrib['k']:t.attrib['v'] for t in e.findall('tag')}
    nodes={n.attrib['id']:dict(osm_id=n.attrib['id'],lon=float(n.attrib['lon']),lat=float(n.attrib['lat']),tags=tags(n)) for n in root.findall('node')}
    ways={w.attrib['id']:dict(osm_id=w.attrib['id'],nodes=[n.attrib['ref'] for n in w.findall('nd')],tags=tags(w)) for w in root.findall('way')}
    station=ways['415865753']
    assert station['tags']['name']=='Subestación de Úbeda'
    polygon=[nodes[i] for i in station['nodes'][:-1]]
    center=np.mean([xy(n['lon'],n['lat']) for n in polygon],axis=0)
    lines=[w for w in ways.values() if w['tags'].get('power')=='line' and w['tags'].get('voltage')=='132000']
    assert len(lines)==3
    ids={n for w in lines for n in w['nodes'] if nodes[n]['tags'].get('power')=='tower'}
    for n in ids:
        nodes[n]['xy_m']=list(xy(nodes[n]['lon'],nodes[n]['lat']))
        nodes[n]['distance_to_station_m']=float(np.linalg.norm(np.array(nodes[n]['xy_m'])-center))
    ordered=sorted(ids,key=lambda n:(nodes[n]['distance_to_station_m'],n))
    cases=[]
    for size in (60,100,150):
        chosen=set(ordered[:size]);chosen_lines=[]
        for line in lines:
            # Keep source order and only source-adjacent selected towers.
            selected=[n for n in line['nodes'] if n in chosen]
            segments=[[a,b] for a,b in zip(line['nodes'],line['nodes'][1:]) if a in chosen and b in chosen]
            chosen_lines.append(dict(osm_way_id=line['osm_id'],name=line['tags']['name'],voltage_v=132000,
                                     tower_ids=selected,source_adjacency_segments=segments))
        pts=np.array([nodes[n]['xy_m'] for n in ordered[:size]])
        sv=np.linalg.svd(pts-pts.mean(axis=0),compute_uv=False)
        cases.append(dict(tower_count=size,tower_ids=ordered[:size],lines=chosen_lines,
                          extent_east_north_km=((pts.max(axis=0)-pts.min(axis=0))/1000).tolist(),
                          spatial_second_to_first_singular_value=float(sv[1]/sv[0])))
    old=json.loads((OUTPUT/'caballero_osm_full.json').read_text(encoding='utf-8'))
    g=nx.Graph();g.add_nodes_from(t['id'] for t in old['towers'])
    for line in old['lines']:g.add_edges_from(zip(line,line[1:]))
    payload=dict(status='candidate_geography_only_not_executable_or_validated_inspection_scenario',
                 source=dict(path=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                             attribution='OpenStreetMap contributors',license='ODbL',
                             notice='Map API returns complete intersecting ways: many towers lie beyond requested bbox'),
                 station=dict(osm_way_id=station['osm_id'],name=station['tags']['name'],xy_m=center.tolist()),
                 selection='nearest N mapped 132kV-line towers to mean of substation polygon vertices; tie by OSM node ID; no solver outcomes',
                 original_27_topology=dict(towers=len(g),edges=g.number_of_edges(),junctions=[n for n,d in g.degree() if d>=3],cycles=len(nx.cycle_basis(g)),
                                          interpretation='three mainly corridor-like sequences; one branch; not a complex mesh'),
                 raw_mapped_towers=sum(n['tags'].get('power')=='tower' for n in nodes.values()),
                 eligible_132kv_towers=len(ids),tower_coordinates=[nodes[n] for n in ordered],cases=cases,
                 missing=['road coverage across entire selected extent','terrain coverage','parking and access candidates',
                          'tower-type-aware inspection templates','duty/fleet configuration and feasibility replay'],
                 claims_excluded=['electrical connectivity through substation terminals','algorithm difficulty','algorithm superiority','field validation'])
    path=OUTPUT/'expanded_geography_candidates.json';path.write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
    case=next(c for c in cases if c['tower_count']==100)
    plt.rcParams.update({'font.sans-serif':['Microsoft YaHei'],'axes.unicode_minus':False,'font.size':10})
    fig,ax=plt.subplots(figsize=(10,6))
    fig.subplots_adjust(left=.10,right=.98,top=.86,bottom=.15)
    for i,line in enumerate(case['lines']):
        color=['#2b6b84','#ba562c','#467945'][i]
        for a,b in line['source_adjacency_segments']:
            p=(np.array([nodes[a]['xy_m'],nodes[b]['xy_m']])-center)/1000
            ax.plot(*p.T,c=color,lw=1.2)
        p=(np.array([nodes[n]['xy_m'] for n in line['tower_ids']])-center)/1000
        name=line['name'].replace('Línea de alta tensión ','')
        ax.scatter(*p.T,s=13,color=color,label=f'{name}：{len(p)}塔')
    ax.scatter(0,0,s=160,c='black',marker='*',label='Úbeda变电站（仅定位，未设定车辆基地）')
    ax.set_aspect('equal');ax.set(xlabel='相对变电站东向距离 / km',ylabel='相对变电站北向距离 / km',
                                 title='100塔候选区域：实际线路走向与杆塔坐标\n道路、地形与完整巡检调度尚待补齐')
    ax.grid(alpha=.2);ax.legend(fontsize=8,loc='lower left');ax.spines[['top','right']].set_visible(False)
    fig.text(.5,.025,'© OpenStreetMap contributors · ODbL | 按地理距离选取，不依据算法结果筛选',ha='center',fontsize=8)
    fig.savefig(OUTPUT/'figures/expanded_100_geography.png',dpi=180);plt.close(fig)
    print(json.dumps({'path':str(path),'raw_towers':payload['raw_mapped_towers'],'eligible_towers':len(ids),'old_topology':payload['original_27_topology'],
                      'cases':[dict(n=c['tower_count'],per_line=[len(l['tower_ids']) for l in c['lines']],extent_km=c['extent_east_north_km']) for c in cases]},ensure_ascii=False))


if __name__=='__main__':main()
