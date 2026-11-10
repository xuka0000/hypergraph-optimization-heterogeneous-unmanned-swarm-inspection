"""Shared ECM transcription for an event-aligned charge/rest interval.

This block is not a thermal-event scheduler. The caller supplies the hot-lock
memory and event-aligned interval. Continuous guard crossing must be checked
in execution and refined before accepting a mission. Charging is controllable
within CC, CV and input-power ceilings; maximum current is not compulsory.
"""
from Agent.Publication.engineering_joint_transcription import battery_rhs, ScipOps
from Agent.Publication.engineering_battery_electrothermal import BatteryParameters


def add_charge_interval(tr, name, b0, b1, dt, ambient, connected,
                        hot_blocked, parameters=None):
    p=(parameters or BatteryParameters()).validate()
    if len(b0)!=6 or len(b1)!=6:
        raise ValueError('Six battery states required')
    if connected.vtype()!='BINARY' or hot_blocked.vtype()!='BINARY':
        raise ValueError('Binary charger occupancy and observed thermal memory required')
    if isinstance(dt,(int,float)) and dt<=0:
        raise ValueError('Positive interval duration required')
    midpoint=[(a+b)/2 for a,b in zip(b0,b1)]
    charge=tr.var(name+'_charge_A',0.,p.charge_current_max_A)
    grid=tr.var(name+'_input_W',0.,p.charger_input_max_W)
    rhs,voltage,_,_=battery_rhs(midpoint,-charge,ambient,p,ScipOps)
    tr.row(name+'_CC',charge-p.charge_current_max_A*connected,'charge_current_ceiling','le')
    tr.row(name+'_thermal_lock',connected+hot_blocked-1,'thermal_memory','le')
    tr.row(name+'_supply',grid-p.charger_input_max_W*connected,'charger_input_ceiling','le')
    tr.row(name+'_terminal_power',p.charge_efficiency*grid-voltage*charge,'charger_ECM_coupling')
    tr.row(name+'_CV',voltage-p.terminal_voltage_max_V,'charge_voltage_ceiling','le',activation=connected)
    tr.row(name+'_voltage_positive',p.terminal_voltage_min_V-voltage,'charge_voltage_ceiling','le',activation=connected)
    # Endpoints plus midpoint are necessary sampled bounds, not a certificate
    # for the unobserved interior or a replacement for thermal latch events.
    for j,state in enumerate([b0,midpoint,b1]):
        tr.row(f'{name}_temperature_low_{j}',p.charge_temperature_min_C-state[2],
               'charge_temperature','le',activation=connected)
        tr.row(f'{name}_temperature_high_{j}',state[2]-p.charge_temperature_max_C,
               'charge_temperature','le',activation=connected)
    for j,d in enumerate(rhs):
        tr.row(f'{name}_state_{j}',b1[j]-b0[j]-dt*d,'charge_rest_ECM_midpoint')
    return dict(charge_current=charge,grid_power=grid,midpoint_voltage=voltage,
                connected=connected,hot_blocked=hot_blocked,
                continuous_thermal_guard_verified=False,complete_joint_problem=False)


def add_charger_capacity(tr,name,blocks,channels,input_limit_W):
    if not isinstance(channels,int) or channels<0 or input_limit_W<0:
        raise ValueError('Nonnegative charger capacity required')
    tr.row(name+'_channels',sum(b['connected'] for b in blocks)-channels,
           'shared_charger_capacity','le')
    tr.row(name+'_power',sum(b['grid_power'] for b in blocks)-input_limit_W,
           'shared_vehicle_power','le')
