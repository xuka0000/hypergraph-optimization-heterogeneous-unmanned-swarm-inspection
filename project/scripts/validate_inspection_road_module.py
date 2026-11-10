"""Bounded motion-module experiment; no regional solve or algorithm comparison.

Tests the actual production primitive generator against kinematic derivatives,
independent geometric lengths and all supplied segment speed limits. Synthetic
inputs are counterexamples, never substituted for the retained regional scene.
"""
from pathlib import Path
import copy, datetime, hashlib, json, math, os, sys, time
import numpy as np
from pyscipopt import Model,quicksum
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from Agent.Publication.engineering_road_motion import RoadMotion,state,angle
OUT=ROOT/'总结/03_实验数据/工程化联合求解/module_validation/road'
BOOK=ROOT/'总结/超图优化与无人集群巡检建模专著'
SCENE=ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes/rommerskirchen.json'
RAW=ROOT/'总结/07_证据与复现索引/综合报告检索/场景设置文献/topology_osm/rommerskirchen_roads.json'
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,d):
 p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def bind(p):return dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p))
def now():return datetime.datetime.now(datetime.timezone.utc).isoformat()
def make_scene(cfg,points,limit=None):
 ids=[str(i) for i in range(len(points))]
 nodes=[dict(id=i,easting_m=float(p[0]),northing_m=float(p[1])) for i,p in zip(ids,points)]
 speed=cfg['speed_max_mps'] if limit is None else limit
 edges=[dict(tail=a,head=b,id='test_'+a+'_'+b,speed_limit_mps=speed,
             seconds=math.dist(points[int(a)],points[int(b)])/speed)
        for a,b in zip(ids,ids[1:])]
 return dict(joint_reference_profile=dict(vehicle=copy.deepcopy(cfg)),
             roads=dict(nodes=nodes,edges=edges,turn_restrictions=[]),
             stops=[dict(id='S'+i,osm_node_id=i,xy_m=list(p)) for i,p in zip(ids,points)])
def audit_motion(road,route,speed_limit):
 cfg=road.cfg; maxv=maxa=maxlat=maxsteer=maxrate=residual=continuity=0.
 last=None
 for seg in route['segments']:
  left=state(seg,0.);right=state(seg,1.)
  if last is not None:
   continuity=max(continuity,float(np.linalg.norm(np.array(left['xy_m'])-last['xy_m'])),
       abs(angle(left['yaw_rad']-last['yaw_rad'])),abs(left['speed_mps']-last['speed_mps']),
       abs(left['steering_rad']-last['steering_rad']))
  last=right
  for s in np.linspace(0,1,501):
   q=state(seg,s);v=q['speed_mps'];de=q['steering_rad']
   maxv=max(maxv,abs(v));maxa=max(maxa,abs(q['acceleration_mps2']))
   maxlat=max(maxlat,v*v*abs(math.tan(de))/cfg['wheelbase_m'])
   maxsteer=max(maxsteer,abs(de));maxrate=max(maxrate,abs(q['steering_rate_rad_s']))
   if 1e-5<s<1-1e-5:
    h=1e-5;qa=state(seg,s-h);qb=state(seg,s+h);dt=2*h*seg['duration_s']
    observed=np.array([(qb['xy_m'][0]-qa['xy_m'][0])/dt,(qb['xy_m'][1]-qa['xy_m'][1])/dt,
                       angle(qb['yaw_rad']-qa['yaw_rad'])/dt])
    expected=np.array([v*math.cos(q['yaw_rad']),v*math.sin(q['yaw_rad']),v*math.tan(de)/cfg['wheelbase_m']])
    residual=max(residual,float(np.abs(observed-expected).max()))
 return dict(length_m=route['length_m'],duration_s=route['duration_s'],segments=len(route['segments']),
    speed_peak_mps=maxv,speed_limit_mps=speed_limit,acceleration_peak_mps2=maxa,
    lateral_acceleration_peak_mps2=maxlat,steering_peak_deg=math.degrees(maxsteer),
    steering_rate_peak_deg_s=math.degrees(maxrate),kinematic_residual=residual,continuity_residual=continuity,
    passed=bool(maxv<=speed_limit+1e-8 and maxa<=cfg['acceleration_max_mps2']+1e-7
       and maxlat<=cfg['lateral_acceleration_max_mps2']+1e-7
       and maxsteer<=math.radians(cfg['steering_max_deg'])+1e-8
       and maxrate<=math.radians(cfg['steering_rate_max_deg_s'])+1e-8
       and residual<2e-5 and continuity<1e-7))
def main():
 OUT.mkdir(parents=True,exist_ok=True);scene=read(SCENE);cfg=scene['joint_reference_profile']['vehicle']
 protocol=dict(experiment_id='inspection-road-primitive-module',code=bind(Path(__file__)),
   production=bind(ROOT/'Agent/Publication/engineering_road_motion.py'),scene=bind(SCENE),
   purpose='verify actual motion generator against source speeds and continuous kinematic identities',
   samples_per_primitive=501,finite_difference_normalized_step=1e-5,paid_cost=0,
   source_road_tags=bind(RAW),solver_time_limit_s=None,solver_threads=1,
   regional_solve=False,algorithm_comparison=False,max_peak_write_GiB=.01)
 result_path=OUT/'results.json'
 if result_path.exists():
  old=read(result_path)
  if old.get('protocol')==protocol and old.get('passed'):
   print(json.dumps(dict(status='reused_compatible_complete',path=str(result_path)),ensure_ascii=False));return
  save(OUT/'attempts'/('result_'+sha(result_path)[:12]+'.json'),old)
 started=now();t0=time.perf_counter();cases=[]
 definitions=[('straight',[(0,0),(40,0)],None),('left90',[(0,0),(40,0),(40,40)],None),
  ('right90',[(0,0),(40,0),(40,-40)],None),('slow_straight',[(0,0),(40,0)],5/3.6),
  ('slow_turn',[(0,0),(40,0),(40,40)],5/3.6)]
 for name,points,limit in definitions:
  model=RoadMotion(make_scene(cfg,points,limit));route=model.motion([str(i) for i in range(len(points))],'S0','S'+str(len(points)-1),0.)
  row=audit_motion(model,route,cfg['speed_max_mps'] if limit is None else limit)
  expected=40. if len(points)==2 else 80-2*model.R+math.pi*model.R/2
  row.update(id=name,expected_length_m=expected,length_error_m=abs(row['length_m']-expected),route=route)
  row['passed']=row['passed'] and row['length_error_m']<1e-8;cases.append(row)
 mixed=make_scene(cfg,[(0,0),(40,0),(80,0)])
 mixed['roads']['edges'][1]['speed_limit_mps']=5/3.6
 model=RoadMotion(mixed);route=model.motion(['0','1','2'],'S0','S2',0.)
 row=audit_motion(model,route,5/3.6);row.update(id='collinear_mixed_limits',route=route);cases.append(row)
 # A retained real edge with the lowest source speed; deterministic longest
 # at that speed avoids an acceleration-limited segment masking the defect.
 edge=min(scene['roads']['edges'],key=lambda e:(e['speed_limit_mps'],-e['length_m'],e['id']))
 nodes={n['id']:n for n in scene['roads']['nodes']}
 points=[(nodes[n]['easting_m'],nodes[n]['northing_m']) for n in (edge['tail'],edge['head'])]
 model=RoadMotion(make_scene(cfg,points,edge['speed_limit_mps']));route=model.motion(['0','1'],'S0','S1')
 real=audit_motion(model,route,edge['speed_limit_mps']);real.update(id='source_low_speed_edge',source_edge=edge,source_xy_m=points,route=route);cases.append(real)
 raw_way=next(w for w in read(RAW)['elements'] if w.get('type')=='way' and str(w['id'])==edge['way_id'])
 real['retained_osm_tags']=raw_way['tags']
 checks=[dict(id=r['id'],passed=r['passed']) for r in cases]
 checks.append(dict(id='source_speed_provenance',passed=float(raw_way['tags']['maxspeed'])/3.6==edge['speed_limit_mps']))
 # Solve the finite timing problem for the fixed five-piece turn. The
 # derivative extrema are independently derived from polynomial roots.
 turn_case=next(c for c in cases if c['id']=='left90');m=Model();m.hideOutput();m.setIntParam('parallel/maxnthreads',1)
 variables=[];analytic=[];c1=35/16;c2=84*math.sqrt(5)/25
 for j,seg in enumerate(turn_case['route']['segments']):
  T=m.addVar(name='duration_'+str(j),lb=.01);variables.append(T)
  if seg['kind']=='steer':
   bound=c1*abs(seg['steering_end_rad']-seg['steering_start_rad'])/math.radians(cfg['steering_rate_max_deg_s'])
   m.addCons(T>=bound);analytic.append(max(.01,bound))
  else:
   length=seg['length_m'];limit=cfg['speed_max_mps'];m.addCons(T*limit>=c1*length)
   m.addCons(T*T*cfg['acceleration_max_mps2']>=c2*length)
   limits=[.01,c1*length/limit,math.sqrt(c2*length/cfg['acceleration_max_mps2'])]
   if seg['kind']=='arc':
    m.addCons(T*T*seg['radius_m']*cfg['lateral_acceleration_max_mps2']>=c1*c1*length*length)
    limits.append(c1*length/math.sqrt(seg['radius_m']*cfg['lateral_acceleration_max_mps2']))
   analytic.append(max(limits))
 m.setObjective(quicksum(variables),'minimize');m.optimize()
 timing=dict(status=str(m.getStatus()),objective_s=m.getObjVal() if m.getNSols() else None,
   durations_s=[m.getVal(v) for v in variables] if m.getNSols() else None,
   analytic_objective_s=sum(analytic),analytic_durations_s=analytic,solver_time_s=m.getSolvingTime(),
   scope='minimum time within a fixed five-piece septic straight/turn family; not optimal route or joint mission')
 m.freeProb();checks.append(dict(id='fixed_primitive_timing_SCIP',passed=timing['status']=='optimal' and abs(timing['objective_s']-timing['analytic_objective_s'])<1e-5))
 production=RoadMotion(scene)
 checks.append(dict(id='retained_source_arc_identity',passed=len(production.edges)==len(scene['roads']['edges']),arcs=len(production.edges)))
 integer=make_scene(cfg,[(0,0),(40,0),(40,40)])
 for node in integer['roads']['nodes']:
  node['easting_m']=int(node['easting_m']);node['northing_m']=int(node['northing_m'])
 checks.append(dict(id='integer_coordinates_accepted',passed=RoadMotion(integer).turn('0','1','2') is not None))
 model=RoadMotion(make_scene(cfg,[(0,0),(40,0),(40,40)]))
 checks.append(dict(id='u_turn_rejected',passed=model.turn('0','1','0') is None))
 short=RoadMotion(make_scene(cfg,[(0,0),(2,0),(2,2)]))
 checks.append(dict(id='short_junction_rejected',passed=short.turn('0','1','2') is None))
 checks.append(dict(id='initial_heading_preserved',passed='S2' not in model.paths('S0',math.pi)))
 checks.append(dict(id='reverse_arc_absent',passed='S0' not in model.paths('S2')))
 duplicate=make_scene(cfg,[(0,0),(40,0)]);duplicate['roads']['edges'].append({**duplicate['roads']['edges'][0],'id':'parallel_other'})
 try:RoadMotion(duplicate);rejected=False
 except ValueError:rejected=True
 checks.append(dict(id='unsupported_parallel_arc_not_silently_lost',passed=rejected))
 restricted=make_scene(cfg,[(0,0),(40,0),(40,40)]);restricted['roads']['turn_restrictions']=[{'test':True}]
 try:RoadMotion(restricted);rejected=False
 except ValueError:rejected=True
 checks.append(dict(id='unsupported_turn_rule_rejected',passed=rejected))
 sensitivity=[]
 for width in (2.,3.,4.,6.,8.):
  modified=copy.deepcopy(cfg);modified['road_corridor_width_m']=width
  m=RoadMotion(make_scene(modified,[(0,0),(40,0),(40,40)]))
  sensitivity.append(dict(corridor_width_m=width,primitive_admitted=m.turn('0','1','2') is not None))
 checks.append(dict(id='width_sensitivity_nontrivial',passed=not sensitivity[0]['primitive_admitted'] and sensitivity[-1]['primitive_admitted']))
 result=dict(protocol=protocol,started_at=started,completed_at=now(),pid=os.getpid(),passed=all(x['passed'] for x in checks),
   checks=checks,cases=cases,timing_optimization=timing,width_sensitivity=sensitivity,parameters=cfg,wall_seconds=time.perf_counter()-t0,
   limits=['reference planning kinematics, not calibrated real-vehicle dynamics',
           'analytic path primitives are a subset of continuous trajectories',
           'no joint task selection, UAV, battery or weather decisions in this module',
           'derivative/continuity sampling is numeric evidence, not an interval proof',
           'no multigraph adapter: parallel arcs must be rejected, never silently merged'])
 save(result_path,result);save(OUT/'protocol.json',protocol)
 save(BOOK/'evidence/road_module_validation.json',dict(result=bind(result_path),**result))
 print(json.dumps(dict(passed=result['passed'],checks=len(checks),failed=[c for c in checks if not c['passed']],
  cases=[{k:r[k] for k in ('id','length_m','duration_s','speed_peak_mps','speed_limit_mps','passed')} for r in cases],
  width_sensitivity=sensitivity,wall_seconds=result['wall_seconds']),ensure_ascii=False))
 raise SystemExit(0 if result['passed'] else 1)
if __name__=='__main__':main()
