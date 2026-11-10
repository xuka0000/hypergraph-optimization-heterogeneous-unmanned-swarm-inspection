"""Source-preserving geography/task module validation, not regional planning.

Independent oracles: retained OSM node values, PROJ published control point,
binary truth-table enumeration, and directed reachability. No cloud or workers.
"""
from pathlib import Path
import datetime, hashlib, itertools, json, math, os, sys, time
import numpy as np
from pyproj import Transformer
from pyscipopt import Model, quicksum

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from Agent.Publication.engineering_regional_structure import add_complete_tower_constraints
SHARE = ROOT / '总结/03_实验数据/真实地理巡检场景共享'
BOOK = ROOT / '总结/超图优化与无人集群巡检建模专著'
OUT = ROOT / '总结/03_实验数据/工程化联合求解/module_validation/geography'
ROLES = (1, 2, 3, 4, 5, 8, 9, 10, 11)

def read(p): return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def now(): return datetime.datetime.now(datetime.timezone.utc).isoformat()
def save(p, data):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
def binding(p): return {'path':p.relative_to(ROOT).as_posix(), 'sha256':sha(p)}

def validate_roles(towers, tasks):
    """No task can substitute for another tower or repeat an existing role."""
    ids = [t['id'] for t in tasks]
    if len(ids) != len(set(ids)): return False
    by_tower = {t['id']: [] for t in towers}
    for task in tasks:
        if task['tower_id'] not in by_tower: return False
        by_tower[task['tower_id']].append(task['paper_point_number'])
    return all(sorted(v) == list(ROLES) for v in by_tower.values())

def source_file(item):
    candidates = [ROOT / (item.get('original_workspace_relative_path') or '__absent__'),
                  SHARE / (item.get('package_path') or '__absent__')]
    return next((p for p in candidates if p.is_file()), None)

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    script_hash = sha(Path(__file__))
    sources = [SHARE/'scenes'/f'{name}.json' for name in
        ['ubeda_osm_060','ubeda_osm_100','ubeda_osm_150','rommerskirchen','brauweiler','villevaude']]
    protocol = dict(experiment_id='inspection-geography-task-module',
        purpose='Verify source identities, coordinate representation and complete-tower logical constraints',
        algorithm_comparison=False, regional_optimization=False, code_sha256=script_hash,
        production_builder=binding(ROOT/'Agent/Publication/engineering_regional_structure.py'),
        current_derived_geometry=binding(ROOT/'research_quality/inspection_regional_model_data.json'),
        input_bindings=[binding(p) for p in sources],
        acceptance=dict(projected_recompute_m=1e-6, inverse_degree=1e-9,
            published_control_point_m=.02, target_plane_m=2e-9,
            target_unit_normal=1e-9, identity_mismatches=0, truth_table_mismatches=0),
        acceptance_interpretation='numerical consistency, not geodetic survey accuracy',
        full_truth_table_states=512, objective_directions=['minimize','maximize'],
        solver_runs=1024, solver_time_limit_s=None, solver_threads=1,
        scope='component verification authorized; no crossing of full-joint review node',
        max_peak_write_GiB=.02, paid_cost=0)
    result_path=OUT/'results.json'
    if result_path.exists():
        previous=read(result_path)
        if previous.get('protocol')==protocol and previous.get('status')=='passed':
            print(json.dumps({'status':'reused_compatible_complete','result':str(result_path)},ensure_ascii=False));return
        save(OUT/'attempts'/('result_'+sha(result_path)[:12]+'.json'), previous)
        if (OUT/'truth_table.json').exists():
            save(OUT/'attempts'/('truth_'+sha(OUT/'truth_table.json')[:12]+'.json'),read(OUT/'truth_table.json'))
    save(OUT/'protocol.json',protocol)
    started=now();t0=time.perf_counter();checks=[]
    def check(name, passed, **values):
        checks.append(dict(id=name, passed=bool(passed), **values))
    scenes=[]
    for path in sources:
        d=read(path);towers=d['towers'];tasks=d['inspection_tasks'];crs=d['coordinate_reference']
        forward=Transformer.from_crs('EPSG:4326',crs['projected'],always_xy=True)
        inverse=Transformer.from_crs(crs['projected'],'EPSG:4326',always_xy=True)
        xy=np.array(forward.transform([t['longitude'] for t in towers],[t['latitude'] for t in towers])).T
        stored=np.array([[t['easting_m'],t['northing_m']] for t in towers])
        xy_error=float(np.abs(xy-stored).max())
        ll=np.array(inverse.transform(stored[:,0],stored[:,1])).T
        ll0=np.array([[t['longitude'],t['latitude']] for t in towers])
        ll_error=float(np.abs(ll-ll0).max())
        row=dict(scene_id=d['scene_id'],towers=len(towers),tasks=len(tasks),
                 unique_task_ids=len({t['id'] for t in tasks}),projection_max_error_m=xy_error,
                 inverse_max_error_degree=ll_error,roles_exact=validate_roles(towers,tasks),
                 geographic_crs=crs['geographic'],projected_crs=crs['projected'])
        scenes.append(row)
        check(d['scene_id']+'_identity_and_coordinates',row['roles_exact'] and xy_error<=1e-6 and ll_error<=1e-9,**row)
    scene=read(SHARE/'scenes/rommerskirchen.json');towers=scene['towers'];tasks=scene['inspection_tasks']
    # Published independent control point, rounded to centimetres (GRS80).
    control=Transformer.from_crs('EPSG:4326','+proj=utm +zone=32 +ellps=GRS80',always_xy=True).transform(12,56)
    control_error=max(abs(control[0]-687071.44),abs(control[1]-6210141.33))
    check('published_UTM_control',control_error<.02,actual=list(control),expected=[687071.44,6210141.33],error_m=control_error,
          source='https://proj.org/en/stable/operations/projections/utm.html')
    # Raw source binding and exact tower provenance, without fetching changing OSM data.
    source_bindings=[];raw_nodes={}
    for item in scene['source_catalog']:
        if item['id'] not in ['OSM_rommerskirchen_power','OSM_rommerskirchen_roads','LI23','KPLC25','TERRAIN_rommerskirchen']:continue
        p=source_file(item)
        ok=p is not None and sha(p)==item['sha256']
        check('source_'+item['id'],ok,source_id=item['id'],path=str(p) if p else None)
        source_bindings.append({**item,'verified_current_sha256':sha(p) if p else None,'verified_file':p.relative_to(ROOT).as_posix() if p else None})
        if item['id']=='OSM_rommerskirchen_power' and p:
            raw_nodes={str(x['id']):x for x in read(p)['elements'] if x['type']=='node'}
    mismatches=[]
    for tower in towers:
        original=raw_nodes.get(tower['source_object_id'])
        if not original or original['lon']!=tower['longitude'] or original['lat']!=tower['latitude']:
            mismatches.append(tower['id'])
    check('raw_OSM_tower_coordinates',not mismatches,checked=len(towers),mismatches=mismatches)
    # Five surface samples lie in their prescribed plane; normals have unit length.
    max_plane=max_normal=0.
    for task in tasks:
        target=task['target'];c=np.array([target[k] for k in ('easting_m','northing_m','altitude_m')]);n=np.array(target['normal_world'])
        q=np.array(target['surface_samples_utm_m'])
        max_plane=max(max_plane,float(np.max(np.abs((q-c)@n))))
        max_normal=max(max_normal,abs(float(n@n)-1.))
    check('target_patch_geometry',max_plane<=2e-9 and max_normal<=1e-9,samples=sum(len(t['target']['surface_samples_utm_m']) for t in tasks),max_plane_residual_m=max_plane,max_unit_normal_squared_residual=max_normal)
    derived_path=ROOT/'research_quality/inspection_regional_model_data.json'
    derived=read(derived_path);corrections={x['task_id']:x for x in derived['corrections']}
    translation_error=0.
    for task in tasks:
        c=corrections[task['id']];shift=c['host_cylinder_correction_m']+c['wire_surface_correction_m']
        delta=np.array(c['normal_world'])*shift
        translation_error=max(translation_error,float(np.abs(np.array(c['surface_samples_utm_m'])-np.array(c['source_surface_samples_utm_m'])-delta).max()))
    check('current_derived_geometry_translation',translation_error<=2e-9,corrected_tasks=len(corrections),maximum_residual_m=translation_error,source=binding(derived_path),scope='translation consistency only; clearance is validated in the safety module')
    # Mutated inputs must be rejected. These are synthetic counterexamples, never saved as scene data.
    first=towers[0];nine=[t for t in tasks if t['tower_id']==first['id']]
    mutations={
        'missing_one_role':nine[:-1],
        'duplicate_role':nine[:-1]+[dict(nine[0],id='counterexample')],
        'transfer_point_as_photo':nine[:-1]+[dict(nine[-1],paper_point_number=6)],
        'foreign_tower_substitution':nine[:-1]+[dict(nine[-1],tower_id=towers[1]['id'])]}
    for name,bad in mutations.items():check(name,not validate_roles([first],bad),expected='rejected')
    fwd=Transformer.from_crs('EPSG:4326','EPSG:32632',always_xy=True)
    wrong=np.array(fwd.transform(first['latitude'],first['longitude']))
    axis_error=float(np.linalg.norm(wrong-[first['easting_m'],first['northing_m']]))
    check('axis_swap_counterexample',axis_error>1000,error_m=axis_error)
    # Exhaustive logical validation. SCIP solves every fixed binary pattern;
    # Python all(bits) is an independent semantic evaluator, not another MILP builder.
    truth=[];solver_t0=time.perf_counter()
    for bits in itertools.product((0,1),repeat=9):
        outcomes={}
        for direction in ('minimize','maximize'):
            m=Model();m.hideOutput();m.setIntParam('parallel/maxnthreads',1)
            ys=[m.addVar(vtype='B',lb=b,ub=b,name=f'y_{k}') for k,b in zip(ROLES,bits)]
            z=m.addVar(vtype='B',name='tower_complete')
            add_complete_tower_constraints(m,z,ys)
            m.setObjective(z,direction);m.optimize()
            outcomes[direction]=dict(value=int(round(m.getVal(z))) if m.getNSols() else None,status=str(m.getStatus()),solve_s=m.getSolvingTime())
            m.freeProb()
        truth.append(dict(mask=sum(b<<j for j,b in enumerate(bits)),photos=sum(bits),expected=int(all(bits)),actual=outcomes['maximize']['value'],minimum_actual=outcomes['minimize']['value'],outcomes=outcomes))
    truth_elapsed=time.perf_counter()-solver_t0
    check('complete_tower_truth_table',all(all(o['value']==r['expected'] and o['status']=='optimal' for o in r['outcomes'].values()) for r in truth),states=len(truth),solver_runs=2*len(truth),mismatches=sum(r['actual']!=r['expected'] or r['minimum_actual']!=r['expected'] for r in truth),wall_seconds=truth_elapsed)
    by_count=[]
    for n in range(10):
        rows=[r for r in truth if r['photos']==n]
        by_count.append(dict(photos=n,patterns=len(rows),complete_tower=sum(r['actual'] for r in rows),mismatches=sum(r['actual']!=r['expected'] or r['minimum_actual']!=r['expected'] for r in rows)))
    # Inspect directed reachability of all candidate stops from each stored depot.
    adj={str(n['id']):[] for n in scene['roads']['nodes']}
    for edge in scene['roads']['edges']:adj[str(edge['tail'])].append(str(edge['head']))
    reached_counts=[]
    for base in scene['bases']:
        seen={str(base['osm_node_id'])};stack=list(seen)
        while stack:
            for other in adj[stack.pop()]:
                if other not in seen:seen.add(other);stack.append(other)
        reachable=sum(str(p['osm_node_id']) in seen for p in scene['stops'])
        reached_counts.append(dict(base=base['id'],reachable_stops=reachable,total_stops=len(scene['stops'])))
    check('directed_depot_to_stop_reachability',all(x['reachable_stops']==x['total_stops'] for x in reached_counts),bases=reached_counts,turn_restrictions=len(scene['roads']['turn_restrictions']),scope='graph reachability only, not vehicle-body or permission validation')
    # Deterministic first-tower example for a completely bound teaching subsection.
    origin=np.array([first['easting_m'],first['northing_m'],first['ground_elevation_m']])
    example=dict(tower_id=first['id'],longitude=first['longitude'],latitude=first['latitude'],easting_m=first['easting_m'],northing_m=first['northing_m'],surface_reference_elevation_m=first['ground_elevation_m'],roles=list(ROLES),target_patch_side_m=.5,normal=nine[0]['target']['normal_world'],target=corrections[nine[0]['id']],min_dwell_s=nine[0]['observation_requirement']['min_dwell_s'],role_rows=[dict(k=t['paper_point_number'],role=t['role'],target_offset_m=(np.array(corrections[t['id']]['target_xyz_m'])-origin).tolist()) for t in nine],geometry_basis='current translated reference patches; OSM tower coordinates unchanged',geometry_binding=binding(derived_path))
    result=dict(protocol=protocol,started_at=started,completed_at=now(),pid=os.getpid(),status='passed' if all(x['passed'] for x in checks) else 'failed',checks=checks,scenes=scenes,numeric_example=example,source_bindings=source_bindings,truth_table_by_count=by_count,wall_seconds=time.perf_counter()-t0,limits=['source geometry audit, not proof of survey precision','reference target geometry, not as-built tower measurements','SCIP truth table validates tower-completion logic only','Copernicus GLO90 is a DSM; numeric ground field is an elevation proxy','target corrections belong to a separately bound derived model, not original map'],full_regional_solver_run=False)
    save(OUT/'truth_table.json',truth);save(result_path,result)
    # Electronic source remains in original project; the book binds it by hash.
    save(BOOK/'evidence/geography_module_validation.json',dict(result=binding(result_path),truth_table=binding(OUT/'truth_table.json'),script=binding(Path(__file__)),**{k:v for k,v in result.items() if k not in ['protocol','source_bindings']}))
    print(json.dumps(dict(status=result['status'],checks_passed=sum(x['passed'] for x in checks),checks=len(checks),wall_seconds=result['wall_seconds'],scenes=scenes,truth_table=by_count,failed=[c for c in checks if not c['passed']]),ensure_ascii=False))
    raise SystemExit(0 if result['status']=='passed' else 1)

if __name__=='__main__':main()
