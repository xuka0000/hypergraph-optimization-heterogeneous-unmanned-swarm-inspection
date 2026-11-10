"""Build a source-bound model contract without rewriting any original scene."""
import datetime
import hashlib
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
MODEL = ROOT/'Agent/Publication/engineering_inspection_contract.json'
sys.path.insert(0,str(ROOT/'Agent/Publication'))
from engineering_replanning import contract_extension


def main():
    def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
    previous=json.loads(MODEL.read_text(encoding='utf-8-sig')) if MODEL.exists() else {}
    names=['ubeda_osm_060','ubeda_osm_100','ubeda_osm_150',
           'rommerskirchen','brauweiler','villevaude']
    scene_records=[]
    for name in names:
        p=ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes'/f'{name}.json'
        d=json.loads(p.read_text(encoding='utf-8-sig'))
        scene_records.append(dict(scene_id=d['scene_id'],path=p.relative_to(ROOT).as_posix(),
            sha256=sha(p),statistics=d['statistics'],coordinate_reference=d['coordinate_reference'],
            source_status='public_geography_with_declared_templates',
            observations_present=bool(d.get('inspection_tasks')),
            observed_geometry_is_as_built=False,
            original_scene_modified=bool(d.get('joint_reference_profile')),
            task_geometry_locator='/inspection_tasks/*/target',
            legacy_viewpoint_locator='/inspection_tasks/*/(easting_m,northing_m,altitude_m)',
            legacy_camera_angles='initialization_only_not_fixed_new_decisions',
            target_surface_mesh='/inspection_tasks/*/target/surface_samples_utm_m' if d.get('joint_reference_profile') else None,
            target_surface_normals='/inspection_tasks/*/target/normal_world' if d.get('joint_reference_profile') else None,
            conductors_and_vegetation_mesh='/obstacle_model' if d.get('joint_reference_profile') else None,
            inspection_acceptance_profile='/joint_reference_profile/camera' if d.get('joint_reference_profile') else None,
            reference_inputs_complete=bool(d.get('joint_model',{}).get('data_complete_for_declared_reference_model')),
            reference_profile='/joint_reference_profile' if d.get('joint_reference_profile') else None,
            field_calibration_complete=False))
    variables=[
        ('task_assignment','binary','observation x sortie','x_i_l'),
        ('task_success_planned','binary','observation','a_i'),
        ('complete_component','binary','component','z_g'),
        ('sortie_active','binary','uncommitted sortie slot','b_l'),
        ('task_successor','binary','sortie x ordered observation/endpoint pair','h_i_j_l'),
        ('aircraft_assignment','binary','aircraft x sortie','y_u_l'),
        ('battery_pair_assignment','binary','battery pair x sortie','beta_p_l'),
        ('event_times','continuous_seconds','launch capture recovery charge','t_event'),
        ('aircraft_position_control_points','continuous_m','sortie x segment x coefficient x xyz','c_l_k_r'),
        ('segment_duration','continuous_seconds','sortie x segment','Delta_l_k'),
        ('aircraft_yaw','continuous_rad','trajectory','psi_l'),
        ('gimbal_angles','continuous_rad','trajectory x gimbal axis','gamma_l'),
        ('camera_focal_length','continuous_mm_or_discrete_verified_modes','capture','f_i'),
        ('exposure','continuous_seconds_or_discrete_verified_modes','capture','t_exp_i'),
        ('vehicle_road_arcs','binary','vehicle x road arc x visit','r_v_a_visit'),
        ('vehicle_motion','continuous_SI','x y yaw speed steering acceleration','q_v_v_v_delta_v'),
        ('battery_energy','continuous_Wh','battery pair x time','E_p'),
        ('charging_channel','binary','battery pair x time','c_p'),
        ('charging_input_power','continuous_W','battery pair x time','P_in_p'),
    ]
    families=[
        ('assign','task_successor_aircraft_battery','assign,fleet,sequence'),
        ('completion','task_success_planned_complete_component','complete'),
        ('time_window','capture_times_dwell_weather','window'),
        ('kinematics','position_control_points_durations_wind','kinematic'),
        ('attitude','acceleration_drag_mass_yaw_rotation','attitude'),
        ('camera_fov','position_rotation_gimbal_focal_length_target','fov,viewangle'),
        ('surface_resolution','position_rotation_gimbal_intrinsics_surface_tangent','gsd'),
        ('motion_blur','position_derivative_rotation_rate_gimbal_zoom_exposure_light','blur,exposure'),
        ('continuous_collision','aircraft_trajectories_vehicle_geometry_obstacles_error_margin','collision'),
        ('weather_power','trajectory_air_velocity_weather_payload_model_history','power'),
        ('emergency_recovery','energy_position_weather_verified_recovery_trajectory','return'),
        ('road_vehicle_motion','road_arc_pose_speed_steering_corridor','ugv'),
        ('rendezvous','aircraft_endpoint_vehicle_pose_event_times','rendezvous'),
        ('handling_capacity','aircraft_assignment_event_intervals_handling_capacity','pad'),
        ('battery_conservation','battery_assignment_flight_power_charge_power_energy','battery'),
        ('battery_availability','sortie_intervals_battery_assignment_charge_cooling','batteryavailability'),
        ('replanning_initial_state','current_vehicle_aircraft_pack_and_task_state','initialstate'),
        ('executed_task_history','image_acceptance_evidence_and_remaining_assignments','taskhistory'),
        ('execution_commitment','issued_actions_and_ongoing_aircraft_pack_identity','commitment,takeover'),
        ('nonanticipativity','same_observed_history_across_weather_and_power_scenarios','nonanticipativity'),
    ]
    required=[
      ('target_surface_and_normals','mesh/m/unit_normal','survey_or_verified_engineering_model'),
      ('conductor_sag_vegetation_obstacles','m','survey_with_uncertainty'),
      ('energized_clearance_by_voltage','m','applicable_operator_rule'),
      ('camera_intrinsics_and_mount','px/rad/m','camera_calibration'),
      ('image_quality_acceptance','m_per_px/px/frame_count','inspection_defect_requirement'),
      ('airframe_loaded_envelope','m/s,m/s2,m/s3,N,rad/s','manufacturer_mode_and_validation'),
      ('airframe_power_model_and_domain','W','same_platform_calibration_or_declared_bound'),
      ('trajectory_tracking_error_bound','m','controller_and_conditions_validation'),
      ('vehicle_chassis_and_corridor','m/rad/m/s','actual_vehicle_and_road_survey'),
      ('battery_usable_capacity','Wh','actual_pack_and_temperature_domain'),
      ('charger_partial_charge_curve','W/efficiency','actual_charger_verified_control'),
      ('vehicle_power_supply','W','actual_vehicle_electrical_spec'),
      ('battery_terminal_energy','Wh','next_shift_operating_requirement'),
      ('forecast_available_at_and_error','UTC,m/s','forecast_issue_and_validation'),
      ('solver_discretization_tolerances','m/s/rad/Wh','declared_numerical_accuracy'),
      ('forecast_issue_archive','UTC and weather fields','issue-and-receipt-timestamped forecast archive; reanalysis cannot substitute'),
      ('power_error_scenarios','W','held-out same-platform calibration residuals'),
      ('replanning_period_and_event_thresholds','s and event-specific SI','declared experiment configuration'),
      ('state_estimation_and_takeover_error','m/rad/Wh/s','controller and telemetry validation'),
    ]
    contract=dict(schema_version=1,model_id='inspection_engineering_joint_model',
        revision='4',observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        user_request='考虑能耗、连续无人机位姿航迹、天气、车-无人机协同的带重规划的大规模电力巡检决策（高细腻度决策）优化模型',
        status='rolling_joint_mathematical_specification_and_causal_input_interface; full_solver_unfinished',
        joint_solver_implemented=False,deployment_ready=False,
        mathematical_source='Agent/Publication/engineering_inspection_model.tex',
        mathematical_source_sha256=sha(ROOT/'Agent/Publication/engineering_inspection_model.tex'),
        report='总结/06_论文初稿/工程化巡检联合数学模型.pdf',
        variables=[dict(id=i,domain=d,indexing=n,symbol=s) for i,d,n,s in variables],
        constraint_families=[dict(id=i,scope=s,equation_labels=['eng:eq:'+x for x in labels.split(',')],
            implementation='mathematical_definition_only; selected primitives separately checked')
            for i,s,labels in families],
        objective={'order':['weighted_incomplete_components','fleet_return_makespan_seconds',
                            'aircraft_energy_Wh'],
                   'scenario_aggregation':'componentwise worst case over declared finite scenarios',
                   'probability_guarantee':False,
                   'hard_constraints':['safety','image_quality','energy_reserve'],
                   'battery_ageing_objective':False,'success_threshold':None},
        planning_scope={'fixed_flight_catalogue_required':False,'fixed_six_tower_limit':False,
            'continuous_trajectory_control_points':True,'segment_durations_are_decisions':True,
            'camera_viewpoints_are_decisions':True,'partial_charge_allowed_if_hardware_verified':True,
            'vehicle_path_and_stop_sequence_are_decisions':True,'carrier_transfer':False,
            'moving_platform_landing':False,'direct_motor_command_optimization':False,
            'replanning':True,'all_configured_regional_tasks':True,
            'full_source_road_and_stop_domain':True,
            'state_resolution':'adaptive_event_anchored_piecewise_polynomial',
            'weather_resolution':'retain_source_resolution_not_claimed_equal_to_trajectory_grid',
            'old_candidates_role':'warm_start_only_not_feasible_domain'},
        source_scenes=scene_records,
        missing_required_parameters=[dict(id=i,value=None,unit=u,required_source=s,
            scope='field calibration; regional declared reference substitutes are bound separately') for i,u,s in required],
        regional_reference_input_profiles=[dict(scene_id=s['scene_id'],scene_sha256=s['sha256'],
            profile=s['reference_profile'],complete_for_declared_reference_simulation=True,
            calibrated_for_deployment=False,
            forecast_issue_archive_available=False,
            rolling_joint_feasibility_verified=False) for s in scene_records if s['reference_inputs_complete']],
        manufacturer_boundaries={'source':'https://enterprise.dji.com/zenmuse-h20-series/specs',
            'zoom_photo_pixels':[5184,3888],'physical_focal_length_mm':[6.83,119.94],
            'gimbal_pitch_deg':[-120,30],'gimbal_yaw_deg':[-320,320],
            'calibrated_fx_fy_px':None,'field_image_quality_thresholds':None,
            'role':'hardware_bounds_not_inspection_acceptance'},
        inherited_reference_assumptions={'reserve_fraction':.2,'horizon_seconds':28800,
            'vehicles':2,'uavs_per_vehicle':2,'battery_pairs_per_vehicle':4,
            'concurrent_charging_pairs_per_vehicle':1,'handling_positions_per_vehicle':1,
            'source':'shared_scene_parameters; design assumptions, not measured fleet inventory'},
        output_contract={'trajectory':['vehicle_or_aircraft_id','time_UTC','local_xyz_m',
            'geographic_lon_lat_alt_with_datum','velocity_mps','acceleration_mps2',
            'body_orientation','gimbal_angles','segment_id','continuous_verification_status'],
            'capture':['task_id','target_region_id','time_UTC','camera_intrinsics_id','exposure_s',
                       'planned_quality_margins','post_execution_acceptance'],
            'resources':['aircraft_id','carrier_id','battery_pair_id','charge_interval',
                         'charge_power_W','SOC_or_Wh_trace','terminal_energy_requirement'],
            'validation':['minimum_constraint_slack','source_identity','assumption_flags',
                          'continuous_vs_sampled_check','failure_reason','forecast_issue_time'],
            'replanning':['information_time','state_revision_and_hash','source_scene_hash',
                'forecast_issue_and_receipt_time','remaining_and_accepted_tasks','committed_prefix',
                'solve_and_communication_delay','actual_takeover_state','candidate_acceptance_or_rejection'],
            'mission_evaluation':['actual_accepted_components','actual_return_time',
                'nonduplicated_actual_energy_Wh','violations','replanning_delays','failures']},
        replanning=contract_extension(),
        hypergraph={'vertices':'scalar decision variables or explicitly declared blocks',
            'hyperedges':'actual constraint scopes retaining functions bounds units activation',
            'resource_candidate_dual':'only after specifying identical incidence, not automatic new complexity',
            'rank_and_degree':None,'hypergraph_advantage_established':False,
            'matching_alone_is_full_solver':False},
        evidence=['research_quality/inspection_engineering_model_validation.json',
            'research_quality/inspection_granularity_hypergraph_audit.json',
            '总结/03_实验数据/电池能耗公开机型验证/cmu_pretrained/results.json',
            '总结/03_实验数据/天气因素建模/航段能耗联算/results.json',
            'research_quality/inspection_replanning_validation.json'],
        next_stage='Implement full roads, continuous-domain acceptance, uncertainty and solver-driven closed-loop execution; current local interface checks are not algorithm comparison')
    for key in ['history','completion_record','optimality_request']:
        if key in previous:contract[key]=previous[key]
    if previous and previous.get('revision')!=contract['revision']:
        contract.setdefault('history',[]).append({k:previous.get(k) for k in
            ['revision','observed_at','mathematical_source_sha256','status']})
    contract.setdefault('optimality_request',{})['scope']='uncapped offline reference; not a promise of zero-delay online replanning'
    MODEL.write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'scenes':len(scene_records),'decision_blocks':len(variables),
                      'constraint_families':len(families),'unfilled_parameter_groups':len(required),
                      'joint_solver_implemented':False}))


if __name__=='__main__':main()
