"""Shared nonlinear physics for finite joint optimization.

This kernel transcribes the executable reference plant, including the rotor
power/ECM coupling. It does not by itself implement mission routing, contact,
camera acceptance, charging allocation, or scenario-tree decisions. Callers
must declare those constraint families separately before claiming a complete
joint model. No historical solver is relabelled by importing this module.
"""
from __future__ import annotations

import math
from dataclasses import asdict
import numpy as np
import pyscipopt as scip
from Agent.Publication.engineering_reference_flight import FlightParameters
from Agent.Publication.engineering_battery_electrothermal import BatteryParameters


class NumericOps:
    sqrt = staticmethod(math.sqrt)
    exp = staticmethod(math.exp)


class ScipOps:
    sqrt = staticmethod(scip.sqrt)
    exp = staticmethod(scip.exp)


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def matvec(a, v):
    return [sum(x*y for x, y in zip(row, v)) for row in a]


def transpose(a):
    return list(map(list, zip(*a)))


def rotation(q, ops):
    norm = ops.sqrt(sum(x*x for x in q))
    w, x, y, z = [a/norm for a in q]
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def flight_rhs(y, command, wind, rho, p, ops):
    """Smooth-domain transcription; command slew is imposed as constraints.

    The numerical plant clips thrust rates. Here a command is constrained to
    the unclipped admissible region, so the same rate is obtained exactly.
    Ground contact uses another mode and must not call this free-flight RHS.
    """
    R = rotation(y[6:10], ops)
    va = matvec(transpose(R), [y[3+i]-wind[i] for i in range(3)])
    area = [p.drag_area_x_m2, p.drag_area_y_m2, p.drag_area_z_m2]
    drag = matvec(R, [-.5*rho*area[i]*ops.sqrt(va[i]*va[i])*va[i] for i in range(3)])
    f = y[13:17]
    acc = [(R[i][2]*sum(f)+drag[i])/p.mass_kg-(p.gravity_mps2 if i == 2 else 0) for i in range(3)]
    q, omega = y[6:10], y[10:13]
    v = cross(q[1:], omega)
    qdot = [-.5*sum(a*b for a, b in zip(q[1:], omega))]+[.5*(q[0]*omega[i]+v[i]) for i in range(3)]
    a = p.arm_radius_m/math.sqrt(2)
    c = p.torque_coefficient/p.thrust_coefficient
    moment = [a*(f[0]+f[1]-f[2]-f[3]), a*(-f[0]+f[1]+f[2]-f[3]), c*(f[0]-f[1]+f[2]-f[3])]
    J = [p.inertia_x, p.inertia_y, p.inertia_z]
    gyro = cross(omega, [J[i]*omega[i] for i in range(3)])
    wdot = [(moment[i]-gyro[i])/J[i] for i in range(3)]
    fdot = [(command[i]-f[i])/p.thrust_lag_s for i in range(4)]
    return list(y[3:6])+acc+qdot+wdot+fdot


def rotor_power(y, fdot, p, ops):
    shaft = [p.torque_coefficient*(y[13+i]/p.thrust_coefficient)**1.5
             +p.rotor_inertia_kg_m2*fdot[i]/(2*p.thrust_coefficient) for i in range(4)]
    # Positive shaft power only: no fictitious regenerative battery credit.
    return sum((x+ops.sqrt(x*x))/2 for x in shaft)/p.motor_efficiency+p.auxiliary_power_W


def battery_rhs(y, current, ambient, p, ops):
    """SOC, polarization, core temperature, and three Wh counters."""
    ocv = p.ocv_empty_V+p.ocv_slope_V*y[0]
    factor = ops.exp(p.resistance_temperature_coefficient_per_K*(p.reference_temperature_C-y[2]))
    r0, r1 = p.R0_ohm*factor, p.R1_ohm*factor
    voltage = ocv-y[1]-r0*current
    heat = r0*current*current+y[1]*y[1]/r1
    power = voltage*current
    positive = (power+ops.sqrt(power*power))/2
    negative = (-power+ops.sqrt(power*power))/2
    rhs = [-current/(3600*p.capacity_Ah), -y[1]/(r1*p.C1_F)+current/p.C1_F,
           (heat-p.thermal_conductance_W_per_K*(y[2]-ambient))/p.thermal_capacity_J_per_K,
           negative/(3600*p.charge_efficiency), positive/3600, heat/3600]
    return rhs, voltage, ocv-y[1], r0


class Transcription:
    """Original scalar constraints and conditional incidence, before presolve."""
    def __init__(self, name):
        self.model = scip.Model(name)
        self.model.hideOutput()
        self.variables = {}
        self.rows = []

    def var(self, name, lb, ub, vtype='C'):
        if name in self.variables:
            raise ValueError('Duplicate scalar variable: '+name)
        v = self.model.addVar(name, vtype=vtype, lb=lb, ub=ub)
        self.variables[name] = v
        return v

    def row(self, name, expression, family, sense='eq', activation=None):
        if any(r['name'] == name for r in self.rows):
            raise ValueError('Duplicate scalar constraint: '+name)
        # Multiplication is exact for a binary gate. No unexplained big-M.
        if activation is not None:
            if activation.vtype() != 'BINARY':
                raise ValueError('Activation must be binary')
            expression = activation*expression
        cons = self.model.addCons(expression == 0 if sense == 'eq' else expression <= 0, name=name)
        # Query the actual original solver row, not guessed block dependencies.
        variables = sorted({v.name for v in self.model.getConsVars(cons)})
        self.rows.append(dict(name=name, family=family, sense=sense, variables=variables,
                             activation=None if activation is None else activation.name))
        return cons

    def hypergraph(self):
        incidence = {n: [] for n in self.variables}
        fixed = {n for n,v in self.variables.items() if v.getLbOriginal() == v.getUbOriginal()}
        for row in self.rows:
            for name in row['variables']:
                incidence[name].append(row['name'])
        free_edges = [[v for v in r['variables'] if v not in fixed] for r in self.rows]
        return dict(n=len(self.variables), m=len(self.rows), N_I=sum(len(r['variables']) for r in self.rows),
                    r=max((len(r['variables']) for r in self.rows), default=0),
                    d_max=max(map(len, incidence.values()), default=0),
                    n_fixed=len(fixed), n_free=len(self.variables)-len(fixed),
                    free_variable_incidence=sum(map(len,free_edges)),
                    maximum_free_variables_per_row=max(map(len,free_edges),default=0),
                    bound_convention='original explicit scalar rows; variable bounds are separate unary domains; before presolve',
                    structural_incidence_not_effective_rank=True,
                    variables=list(self.variables), edges=self.rows, variable_edges=incidence)


def add_flight_battery_interval(transcription, name, y0, y1, b0, b1, command,
                               dt, wind, rho, ambient, flight=None, battery=None,
                               contact=None, power_residual_fraction=0.,
                               actuator_mode='thrust_lag'):
    """Implicit midpoint for one free-flight interval of the joint state.

    State endpoints, command and dt may be variables of a larger optimization.
    Weather values must come from that same interval and information branch.
    This routine has no weather or trajectory search hidden in its constants.
    It enforces midpoint physical envelopes; dense execution verification and
    mesh refinement are necessary before accepting continuous feasibility.
    """
    p, bp = flight or FlightParameters(), battery or BatteryParameters()
    p.validate(); bp.validate()
    if not all(len(v)==17 for v in [y0,y1]) or not all(len(v)==6 for v in [b0,b1]) or len(command)!=4:
        raise ValueError('Joint interval state dimensions')
    if isinstance(dt,(int,float)) and (not math.isfinite(dt) or dt<=0):
        raise ValueError('Positive finite interval duration required')
    tr = transcription
    y = [(a+b)/2 for a, b in zip(y0, y1)]
    b = [(a+c)/2 for a, c in zip(b0, b1)]
    if actuator_mode not in ['thrust_lag','ground_angular_acceleration']:
        raise ValueError('Unknown actuator mode')
    angular_block=None
    if actuator_mode=='ground_angular_acceleration':
        if contact is None:raise ValueError('Ground motor drive requires explicit pad mode')
        from Agent.Publication.engineering_platform_contact import ContactParameters
        gp=contact.get('parameters') or ContactParameters()
        tr.row(name+'_ground_drive_mode',contact['active']-1.,'ground_drive_mode')
        angular_block=[]
        for r in range(4):
            cap=math.sqrt(p.thrust_max_per_rotor_N/p.thrust_coefficient)
            om0=tr.var(f'{name}_Omega_start_{r}',0.,cap)
            om1=tr.var(f'{name}_Omega_end_{r}',0.,cap)
            om=(om0+om1)/2;alpha=command[r]
            tr.row(f'{name}_thrust_start_{r}',y0[13+r]-p.thrust_coefficient*om0**2,'ground_rotor_state')
            tr.row(f'{name}_thrust_end_{r}',y1[13+r]-p.thrust_coefficient*om1**2,'ground_rotor_state')
            tr.row(f'{name}_angular_step_{r}',om1-om0-dt*alpha,'ground_rotor_midpoint')
            tr.row(f'{name}_angular_hi_{r}',alpha-gp.ground_angular_acceleration_max_radps2,'ground_drive_domain','le')
            tr.row(f'{name}_angular_lo_{r}',-alpha-gp.ground_angular_acceleration_max_radps2,'ground_drive_domain','le')
            torque=p.torque_coefficient*om**2+p.rotor_inertia_kg_m2*alpha
            tr.row(f'{name}_motor_hi_{r}',torque-gp.ground_motor_torque_max_Nm,'ground_drive_domain','le')
            tr.row(f'{name}_motor_lo_{r}',-torque-gp.ground_motor_torque_max_Nm,'ground_drive_domain','le')
            y[13+r]=p.thrust_coefficient*om**2
            angular_block.append(dict(start=om0,end=om1,midpoint=om,acceleration=alpha))
        dy=flight_rhs(y,y[13:17],wind,rho,p,ScipOps)
        for r,block in enumerate(angular_block):
            dy[13+r]=2*p.thrust_coefficient*block['midpoint']*block['acceleration']
    else:
        dy = flight_rhs(y, command, wind, rho, p, ScipOps)
    contact_block=None
    if contact is not None:
        from Agent.Publication.engineering_platform_contact import add_contact
        contact_block=add_contact(tr,name+'_contact',y,**contact)
        if actuator_mode=='ground_angular_acceleration':
            gaps=[x[2]-contact['pad_xyz'][2] for x in contact_block['positions']]
            tr.row(name+'_ground_drive_touch',gaps[0]*gaps[1]*gaps[2]*gaps[3],'ground_drive_contact')
        inertia=[p.inertia_x,p.inertia_y,p.inertia_z]
        for i in range(3):
            dy[3+i]+=contact_block['force_world'][i]/p.mass_kg
            dy[10+i]+=contact_block['moment_body'][i]/inertia[i]
    current = tr.var(name+'_current_A', 0., bp.discharge_current_max_A)
    power = tr.var(name+'_power_W', 0., bp.discharge_current_max_A*bp.terminal_voltage_max_V)
    db, voltage, emf, r0 = battery_rhs(b, current, ambient, bp, ScipOps)
    tr.row(name+'_rotor_power', power-(1+power_residual_fraction)*rotor_power(y, dy[13:], p, ScipOps), 'actuator_power')
    tr.row(name+'_electrical_power', voltage*current-power, 'battery_flight_coupling')
    tr.row(name+'_high_voltage_branch', 2*r0*current-emf, 'battery_domain', 'le')
    tr.row(name+'_voltage_lower', bp.terminal_voltage_min_V-voltage, 'battery_domain', 'le')
    tr.row(name+'_voltage_upper', voltage-bp.terminal_voltage_max_V, 'battery_domain', 'le')
    for i, deriv in enumerate(dy):
        tr.row(f'{name}_flight_{i}', y1[i]-y0[i]-dt*deriv, 'rigid_rotor_midpoint')
    for i, deriv in enumerate(db):
        tr.row(f'{name}_battery_{i}', b1[i]-b0[i]-dt*deriv, 'battery_midpoint')
    for i in range(4):
        tr.row(f'{name}_slew_upper_{i}', dy[13+i]-p.thrust_rate_max_Nps, 'rotor_command_domain', 'le')
        tr.row(f'{name}_slew_lower_{i}', -dy[13+i]-p.thrust_rate_max_Nps, 'rotor_command_domain', 'le')
    for node in [y0, y1]:
        # A strict nonzero quaternion domain is required by normalized rotation.
        # Exact unit norm at endpoints is compatible with midpoint q dynamics.
        tag='start' if node is y0 else 'end'
        tr.row(name+'_unit_q_'+tag, sum(q*q for q in node[6:10])-1., 'quaternion_domain')
    return dict(current=current, power=power, midpoint_voltage=voltage,contact=contact_block,
                actuator_mode=actuator_mode,ground_angular_states=angular_block,
                reference_parameters=dict(flight=asdict(p), battery=asdict(bp)),
                complete_joint_problem=False)
