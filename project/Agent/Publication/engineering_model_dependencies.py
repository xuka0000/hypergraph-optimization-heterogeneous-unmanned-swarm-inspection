"""Conservative variable-block dependencies for the integrated definition.

This is a teaching/implementation ledger, NOT scalar n, m, N_I, r statistics.
Actual solver expressions and activation variables must supply scalar incidence.
"""

REFINED_DEPENDENCIES = {
    'battery_conservation': ['pack_SOC','pack_current','pack_voltage','pack_core_temperature',
        'terminal_energy_counters','charge_channel','charge_power','flight_power','pack_assignment','event_time'],
    'planned_recovery_reserve': ['pack_SOC','pack_voltage','pack_current','pack_core_temperature',
        'flight_power','event_time','pack_assignment','position','vehicle_state'],
    'rigid_body_rotation': ['position','velocity','acceleration','attitude','body_rate','thrust',
        'rotor_thrust','rotor_command','rotor_speed','landing_contact_force','landing_contact_moment',
        'vehicle_state','event_time','flight_power'],
    'tracking_execution': ['controller_reference','position','velocity','acceleration','attitude',
        'body_rate','rotor_thrust','rotor_command','event_time'],
    'electrothermal_battery': ['pack_SOC','pack_voltage','pack_current','pack_polarization',
        'pack_core_temperature','pack_cooling_duration','charge_channel','charge_power',
        'flight_power','pack_assignment','pack_mode','event_time','rotor_thrust','rotor_speed'],
    'weather_joint_scenarios': ['position','vehicle_state','event_time','flight_power',
        'pack_SOC','pack_core_temperature'],
    'temporal_energy': ['position','velocity','attitude','body_rate','power_history',
        'flight_power','event_time','aircraft_assignment'],
    'state_handover': ['position','velocity','acceleration','attitude','body_rate','rotor_thrust',
        'gimbal','focal_length','vehicle_state','vehicle_control','road_mode','road_progress',
        'pack_SOC','pack_voltage','pack_current','pack_polarization','pack_core_temperature',
        'pack_mode','charge_power','power_history','task_success','event_time','controller_reference',
        'aircraft_assignment','pack_assignment'],
    'platform_model_binding': ['attitude','body_rate','rotor_thrust','rotor_command','rotor_speed',
        'flight_power','power_history','pack_SOC','pack_voltage','pack_current','pack_polarization',
        'pack_core_temperature','charge_power','gimbal','focal_length'],
}

def bind_integrated_dependencies(contract):
    ids={v['id'] for v in contract['variables']}
    for family in contract['constraint_families']:
        deps=['pack_SOC' if v=='pack_energy' else v for v in family.get('depends_on',[])]
        if family['id'] in REFINED_DEPENDENCIES:
            deps=REFINED_DEPENDENCIES[family['id']][:]
        if family['id'] in {'initial_and_terminal','execution_takeover','commitment'}:
            deps += ['pack_polarization','pack_core_temperature','rotor_thrust','body_rate','power_history']
        if family['id']=='causal_information':
            deps += ['rotor_command','controller_reference','charge_channel']
        if family['id']=='force_and_attitude':deps += ['rotor_thrust']
        if family['id']=='weather_power':deps += ['attitude','power_history']
        family['depends_on']=list(dict.fromkeys(deps))
        family['dependency_semantics']='conservative variable-block support; scalar expression/activation extraction still required'
        if not deps or set(deps)-ids:raise ValueError('Invalid integrated dependencies: '+family['id'])
    contract.setdefault('hypergraph',{})['dependency_registration']={
        'status':'variable_block_support_registered',
        'scalar_incidence_verified':False,
        'conservative_block_dependencies_are_not_hypergraph_size_statistics':True,
        'objective_variables':['tower_complete','makespan','flight_power','objective_epigraph'],
        'next_requirement':'extract indexed scalar constraint and objective factors, including conditional activation, from the same solver expressions'}
    return contract
