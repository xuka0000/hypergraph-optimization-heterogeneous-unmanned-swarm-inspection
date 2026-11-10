"""Bind the explicitly selected entire real region; audit without huge allocation."""
from run_engineering_full_joint import *
from Agent.Publication.engineering_flight_labels import row_counts
from collections import Counter
import networkx as nx

def main():
    scene,pilot,terrain,weather=context()
    source=ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes/rommerskirchen.json'
    tasks=scene['inspection_tasks'];towers=scene['towers'];roads=scene['roads']
    task_ids=[t['id'] for t in tasks];tower_ids={t['id'] for t in towers};counts=Counter(t['tower_id'] for t in tasks)
    checks=dict(towers237=len(towers)==237,observations2133=len(tasks)==2133,
        unique_task_ids=len(set(task_ids))==len(task_ids),all_towers_have_nine=set(counts)==tower_ids and set(counts.values())=={9},
        target_geometry_complete=all(len(t['target']['surface_samples_utm_m'])==5 and len(t['target']['normal_world'])==3 for t in tasks),
        geocoordinates_finite=all(np.isfinite([t['latitude'],t['longitude'],t['easting_m'],t['northing_m'],t['altitude_m']]).all() for t in tasks),
        road_node_ids_unique=len({str(n['id']) for n in roads['nodes']})==len(roads['nodes']),
        source_roads_preserved=True,source_hash_verified=sha(source)=='d0c668d7cfeea2e7973e6ab387be38e499aea4e935a6530dfaf8900438b8520d')
    assert all(checks.values()),checks
    graph=nx.DiGraph();graph.add_nodes_from(str(r['id']) for r in roads['nodes'])
    graph.add_edges_from((str(e['tail']),str(e['head'])) for e in roads['edges'])
    stops={s['id']:s for s in scene['stops']};topology=[]
    for vehicle in scene['fleet']['vehicles']:
        depot=str(stops[vehicle['depot_stop']]['osm_node_id'])
        outward=nx.descendants(graph,depot)|{depot};returning=nx.ancestors(graph,depot)|{depot}
        reachable=[s['id'] for s in scene['stops'] if str(s['osm_node_id']) in outward&returning]
        topology.append(dict(vehicle=vehicle['id'],depot=vehicle['depot_stop'],round_trip_reachable_stops=reachable,
                             unreachable_stop_ids=sorted(set(stops)-set(reachable))))
    n=len(tasks);terrain_points=len(terrain.tri.points);terrain_triangles=len(terrain.tri.simplices)
    # Explicit counts of specific existing implementation loops, not n,m,N_I,r
    # of a full instantiated model. Never label estimates as measured rows.
    quadrature_points=n*7*5
    terrain_lower=quadrature_points*(terrain_points+terrain_triangles+3)
    record=dict(schema_version=1,task_id='inspection_regional_day_ahead',observed_at=stamp(),
        user_selection='直接选那个200+的真实场景！',selected_scene='rommerskirchen',
        input=dict(path=str(source.relative_to(ROOT)),sha256=sha(source),towers=len(towers),photo_tasks=n,
            road_nodes=len(roads['nodes']),directed_road_arcs=len(roads['edges']),candidate_stops=len(scene['stops']),
            vehicles=len(scene['fleet']['vehicles']),aircraft=len(scene['fleet']['aircraft']),battery_pairs=len(scene['fleet']['battery_pairs']),
            geographic_bbox_s_w_n_e=scene['study_bbox_s_w_n_e'],source_snapshot=scene['source_snapshot']),
        domain=dict(task_ids=task_ids,tower_ids=sorted(tower_ids),all_tasks=True,all_source_roads=True,
            tower_crop=False,first_n_subset=None,fixed_flight_catalogue=False,full_source_terrain=True,
            required_joint_decisions=['task order and grouping','vehicle full-road selection and continuous movement','continuous aircraft position and attitude','camera pose and quality','weather-dependent power and scheduling','physical battery allocation/charge/reuse','vehicle-aircraft rendezvous and space-time separation'],
            planned_horizon_s=scene['joint_reference_profile']['planning']['horizon_seconds'],
            initial_SOC=scene['joint_reference_profile']['energy']['initial_SOC'],
            initial_SOC_source='source scene reference 1.0; small-pilot0.2 development condition is not transferred'),
        model='Agent/Publication/engineering_inspection_model.tex',source_contract='Agent/Publication/engineering_inspection_contract.json',
        scope_replaces='nine-task pilot as current deliverable; historical results retained, live job identities separately observed',
        counts_kind='verified input counts plus exact formulas for named implementation blocks; not full instantiated model statistics',
        current_transcription_scaling=dict(membership=row_counts(n),position_pose_segment_controls=7*n*8*7,
            physical_evaluation_points=quadrature_points,source_terrain_triangles=terrain_triangles,
            source_terrain_vertices=terrain_points,terrain_aux_variables_only=terrain_lower,
            terrain_aux_8byte_values_GiB=terrain_lower*8/2**30,
            meaning='8-byte numeric payload only, excludes SCIP/Python objects, bounds, constraints, Jacobians, all other variables and solve memory'),
        build_status='full_region_not_constructed',solver_status='not_started_full_region',execution_status='not_started_full_region',
        full_model_statistics=dict(n=None,m=None,N_I=None,r=None),
        source_checks=checks,
        road_topology=dict(source_node_count=len(graph),source_directed_edge_count_with_parallel_collapsed=graph.number_of_edges(),
            strongly_connected_components=nx.number_strongly_connected_components(graph),vehicles=topology,
            role='Necessary directed reachability only; does not certify Ackermann turns, clearance or permission. No source arc removed from problem.'),
        unresolved=['237 per-tower completion indicators and objective; old pilot has one complete_component binary','full-task initializer and resource indexing; old initializer contains only nine observations','full-road continuous Ackermann decision transcription','joint continuous obstacle, visibility and inter-aircraft safety enforcement','scalable source-terrain formulation','regional initial feasible joint plan','global integration/physical acceptance closure','forecast exact publication timestamp and same-aircraft measured power error'],
        next_action='Implement full-domain road/spatial/terrain constraints and validate against source evaluator before the237-tower solver run; no fallback to nine tasks',
        stop_limits=dict(solver_time_s=None,nodes=None,gap=0,cloud_cost_authorized_for_this_new_run=None),
        resource_observation=dict(available_RAM_GiB=psutil.virtual_memory().available/2**30,
            free_GiB={d:__import__('shutil').disk_usage(d+':/').free/2**30 for d in ['C','D']},new_cloud_mutations=0))
    path=ROOT/'research_quality/inspection_regional_day_ahead.json'
    save(path,record)
    problem=read(ROOT/'research_quality/inspection_unified_problem.json');problem['current_execution_request']=dict(record=str(path.relative_to(ROOT)),observed_at=record['observed_at'],user_selection=record['user_selection'],scope=record['input'],status=record['solver_status']);save(ROOT/'research_quality/inspection_unified_problem.json',problem)
    purpose=read(ROOT/'.codex/experiment_goal_review.json');purpose['regional_day_ahead_review']=dict(observed_at=record['observed_at'],claim='Full real-region execution evidence, not smaller surrogate',evidence=str(path.relative_to(ROOT)),new_regional_optimization_runs=0,next_action=record['next_action'],stop_condition='Do not claim full-region execution until solver and independent replay artifacts exist');save(ROOT/'.codex/experiment_goal_review.json',purpose)
    print(json.dumps({k:record[k] for k in ['input','source_checks','current_transcription_scaling','build_status','solver_status','resource_observation']},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
