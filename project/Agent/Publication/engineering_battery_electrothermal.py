"""Executable reference-pair electrical/thermal model for revision 8.

Positive current discharges. SOC, polarization and temperature persist across
sorties, charging and replanning. This is an explicitly assumed reference
plant, NOT a fitted TB60 model. Historical Wh-only solvers remain historical.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import math
import numpy as np
from scipy.integrate import solve_ivp


@dataclass(frozen=True)
class BatteryParameters:
    capacity_Ah: float = 11.87
    ocv_empty_V: float = 40.8
    ocv_slope_V: float = 12.0
    R0_ohm: float = 0.08
    R1_ohm: float = 0.04
    C1_F: float = 2500.0
    resistance_temperature_coefficient_per_K: float = 0.02
    reference_temperature_C: float = 25.0
    thermal_capacity_J_per_K: float = 2700.0
    thermal_conductance_W_per_K: float = 6.0
    charge_efficiency: float = 0.9
    charge_current_max_A: float = 17.8
    discharge_current_max_A: float = 50.0
    terminal_voltage_max_V: float = 52.8
    terminal_voltage_min_V: float = 36.0
    charger_input_max_W: float = 1000.0
    soc_min: float = 0.2
    soc_max: float = 1.0
    charge_temperature_min_C: float = 5.0
    charge_temperature_max_C: float = 40.0
    charge_restart_temperature_C: float = 39.0
    flight_temperature_min_C: float = -20.0
    flight_temperature_max_C: float = 50.0

    def validate(self):
        if any(not math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('Nonfinite battery parameter')
        for name in ['capacity_Ah', 'ocv_empty_V', 'ocv_slope_V', 'R0_ohm',
                     'R1_ohm', 'C1_F', 'thermal_capacity_J_per_K',
                     'thermal_conductance_W_per_K', 'charge_current_max_A',
                     'discharge_current_max_A', 'charger_input_max_W']:
            if getattr(self, name) <= 0: raise ValueError('Positive parameter required: '+name)
        if not 0 < self.charge_efficiency <= 1: raise ValueError('Invalid efficiency')
        if not 0 <= self.soc_min < self.soc_max <= 1: raise ValueError('Invalid SOC domain')
        if not self.charge_temperature_min_C < self.charge_restart_temperature_C < self.charge_temperature_max_C:
            raise ValueError('Invalid thermal hysteresis')
        if not 0 < self.terminal_voltage_min_V < self.terminal_voltage_max_V:
            raise ValueError('Invalid voltage limits')
        return self


# y = [SOC, V_RC, T_C, grid_Wh, load_Wh, resistive_heat_Wh].
def initial_state(soc=1.0, temperature_C=25.0, polarization_V=0.0):
    y = np.array([soc, polarization_V, temperature_C, 0., 0., 0.], dtype=float)
    if not np.all(np.isfinite(y)) or not 0 <= soc <= 1:
        raise ValueError('Invalid initial battery state')
    return y


def electrical_parameters(y, p):
    factor = math.exp(p.resistance_temperature_coefficient_per_K *
                      (p.reference_temperature_C-float(y[2])))
    return p.ocv_empty_V+p.ocv_slope_V*y[0], p.R0_ohm*factor, p.R1_ohm*factor


def terminal_voltage(y, current_A, p):
    ocv, r0, _ = electrical_parameters(y, p)
    return ocv-y[1]-r0*current_A


def current_for_power(y, terminal_power_W, p):
    """Stable high-voltage quadratic branch, including P=0 and charging."""
    ocv, r0, _ = electrical_parameters(y, p)
    a = ocv-y[1]
    discriminant = a*a-4*r0*terminal_power_W
    if a <= 0 or discriminant <= 0 or not math.isfinite(discriminant):
        raise ValueError('Power request outside high-voltage branch')
    return 2*terminal_power_W/(a+math.sqrt(discriminant))


def charging_current(y, p, input_limit_W=None, fraction=1.0):
    """Controllable reference charger; returns discharge-signed current, mode.

    Temperature enable/hysteresis is an event mode in simulate_charge, not an
    ambient-temperature proxy. A CC or CV ceiling may also be supply limited.
    """
    if not 0 <= fraction <= 1: raise ValueError('Charge command outside [0,1]')
    limit = p.charger_input_max_W if input_limit_W is None else float(input_limit_W)
    if not math.isfinite(limit) or limit < 0: raise ValueError('Invalid supply limit')
    limit = min(limit, p.charger_input_max_W)
    ocv, r0, _ = electrical_parameters(y, p)
    a = ocv-y[1]
    caps = {'CC': p.charge_current_max_A,
            'CV': max(0., (p.terminal_voltage_max_V-a)/r0),
            'supply': -current_for_power(y, -p.charge_efficiency*limit, p)}
    mode = min(caps, key=caps.get)
    return -fraction*caps[mode], mode if fraction == 1 else 'partial_'+mode


def derivative(y, current_A, ambient_C, p):
    ocv, r0, r1 = electrical_parameters(y, p)
    voltage = ocv-y[1]-r0*current_A
    heat = r0*current_A**2+y[1]**2/r1
    power = voltage*current_A
    return np.array([-current_A/(3600*p.capacity_Ah),
                     -y[1]/(r1*p.C1_F)+current_A/p.C1_F,
                     (heat-p.thermal_conductance_W_per_K*(y[2]-ambient_C))/p.thermal_capacity_J_per_K,
                     max(-power, 0.)/(3600*p.charge_efficiency),
                     max(power, 0.)/3600, heat/3600])


def chemical_energy_Wh(y, p):
    s = y[0]
    return p.capacity_Ah*(p.ocv_empty_V*s+0.5*p.ocv_slope_V*s*s)


def polarization_energy_Wh(y, p):
    return 0.5*p.C1_F*y[1]**2/3600


def energy_balance_residual_Wh(before, after, p):
    # OCV temperature coefficient and Coulombic loss are zero in this profile.
    stored_before = chemical_energy_Wh(before,p)+polarization_energy_Wh(before,p)
    stored_after = chemical_energy_Wh(after,p)+polarization_energy_Wh(after,p)
    return (stored_after-stored_before + after[4]-before[4] + after[5]-before[5]
            -p.charge_efficiency*(after[3]-before[3]))


def simulate_segment(y0, duration_s, command, ambient_C, p, *, max_step_s=2.0,
                     rtol=1e-9, atol=1e-11):
    """Current/power/rest segment with physical bound events (never clip SOC)."""
    p.validate()
    y0=np.asarray(y0,dtype=float)
    if y0.shape != (6,) or not np.all(np.isfinite(y0)): raise ValueError('State shape')
    if not math.isfinite(duration_s) or duration_s < 0: raise ValueError('Duration')
    if not math.isfinite(ambient_C) or max_step_s <= 0: raise ValueError('Ambient/step')
    kind=command['kind']; value=float(command.get('value',0))
    if kind not in {'current','power','rest'} or not math.isfinite(value):raise ValueError('Command')
    def current(y):
        return current_for_power(y,value,p) if kind=='power' else value if kind=='current' else 0.
    def margins(y):
        i=current(y);v=terminal_voltage(y,i,p)
        return [y[0]-p.soc_min,p.soc_max-y[0],v-p.terminal_voltage_min_V,
                p.terminal_voltage_max_V-v,p.discharge_current_max_A-i,
                i+p.charge_current_max_A,y[2]-p.flight_temperature_min_C,
                p.flight_temperature_max_C-y[2]]
    if min(margins(y0)) < -1e-8:raise ValueError('Initial state/command violates battery domain')
    events=[]
    for k in range(8):
        def event(t,y,k=k):return margins(y)[k]+1e-9
        event.terminal=True;event.direction=-1;events.append(event)
    if duration_s==0:return dict(t=np.array([0.]),y=y0[:,None],completed=True,stop_reason=None)
    sol=solve_ivp(lambda t,y:derivative(y,current(y),ambient_C,p),(0,duration_s),y0,
                  rtol=rtol,atol=atol,max_step=max_step_s,events=events)
    if not sol.success:raise ArithmeticError(sol.message)
    hit=next((i for i,a in enumerate(sol.t_events) if len(a)),None)
    names=['SOC_min','SOC_max','voltage_min','voltage_max','discharge_current',
           'charge_current','temperature_min','temperature_max']
    return dict(t=sol.t,y=sol.y,completed=hit is None,
                stop_reason=None if hit is None else names[hit])


def simulate_power_trace(y0,times_s,power_W,ambient_C,p,*,max_step_s=.2):
    """Feed an actual simulated actuator trace into the same electrical plant.

    Piecewise-linear interpolation is declared here. This is execution replay,
    not a replacement for co-optimizing power and trajectory states.
    """
    t=np.asarray(times_s,dtype=float);power=np.asarray(power_W,dtype=float)
    if t.ndim!=1 or len(t)<2 or power.shape!=t.shape or not np.all(np.isfinite(t)) or not np.all(np.diff(t)>0) or not np.all(np.isfinite(power)) or np.any(power<0):
        raise ValueError('Invalid power trace')
    p.validate()
    def rhs(at,y):return derivative(y,current_for_power(y,float(np.interp(at,t,power)),p),ambient_C,p)
    sol=solve_ivp(rhs,(float(t[0]),float(t[-1])),y0,t_eval=t,rtol=1e-9,atol=1e-11,max_step=max_step_s)
    if not sol.success:raise ArithmeticError(sol.message)
    violations=[]
    for at,z,P in zip(sol.t,sol.y.T,power):
        i=current_for_power(z,P,p);v=terminal_voltage(z,i,p)
        if not(p.soc_min-1e-8<=z[0]<=p.soc_max+1e-8 and p.terminal_voltage_min_V-1e-8<=v<=p.terminal_voltage_max_V+1e-8
               and 0<=i<=p.discharge_current_max_A+1e-8 and p.flight_temperature_min_C-1e-8<=z[2]<=p.flight_temperature_max_C+1e-8):
            violations.append(float(at))
    return dict(t=sol.t,y=sol.y,completed=not violations,violating_sample_times_s=violations,
                validation_scope='sampled trajectory replay; interval safety requires separate certification')


def simulate_charge(y0, target_soc, ambient_C, p, *, horizon_s=14400.,
                    input_limit_W=None, fraction=1., max_step_s=2., thermal_memory=None):
    """Charge/rest hybrid execution with CC/CV, supply cap and thermal events."""
    p.validate();y=np.asarray(y0,dtype=float).copy()
    if y.shape!=(6,) or not np.all(np.isfinite(y)):raise ValueError('State shape')
    if not p.soc_min <= y[0] <= target_soc < p.soc_max:raise ValueError('Charge target/domain')
    if not math.isfinite(ambient_C) or not math.isfinite(horizon_s) or horizon_s<=0 or max_step_s<=0:
        raise ValueError('Invalid charge duration/environment')
    charging_current(y,p,input_limit_W,fraction)  # validate request even if cooling
    times=[0.];states=[y.copy()];phases=[];now=0.
    memory=thermal_memory or {}
    if any(type(v) is not bool for v in memory.values()):raise ValueError('Thermal memory must be boolean')
    cool=y[2]>p.charge_temperature_max_C or (memory.get('hot_blocked',False) and y[2]>p.charge_restart_temperature_C)
    warm=y[2]<p.charge_temperature_min_C
    while now<horizon_s and y[0]<target_soc-1e-9:
        if cool or warm:
            threshold=p.charge_restart_temperature_C if cool else p.charge_temperature_min_C
            if (cool and ambient_C>=threshold) or (warm and ambient_C<=threshold and abs(y[1])<1e-12):
                return dict(t=np.array(times),y=np.array(states).T,phases=phases,
                            thermal_memory=dict(hot_blocked=bool(cool)),
                            completed=False,stop_reason='ambient_cannot_reach_charge_window')
            def ready(t,z):return z[2]-threshold
            ready.terminal=True;ready.direction=-1 if cool else 1
            mode='cooling' if cool else 'passive_warming'
            current=lambda z:0.
            events=[ready]
        else:
            mode='charging'
            current=lambda z:charging_current(z,p,input_limit_W,fraction)[0]
            def target(t,z):return z[0]-target_soc
            def hot(t,z):return z[2]-p.charge_temperature_max_C
            def cold(t,z):return z[2]-p.charge_temperature_min_C
            target.terminal=hot.terminal=cold.terminal=True
            target.direction=1;hot.direction=1;cold.direction=-1
            events=[target,hot,cold]
        start=now
        sol=solve_ivp(lambda t,z:derivative(z,current(z),ambient_C,p),
                      (now,horizon_s),y,rtol=1e-9,atol=1e-11,
                      max_step=max_step_s,events=events)
        if not sol.success:raise ArithmeticError(sol.message)
        now=float(sol.t[-1]);y=sol.y[:,-1]
        if now-start<1e-8:raise ArithmeticError('Thermal event failed to advance')
        times.extend(sol.t[1:].tolist());states.extend(sol.y[:,1:].T)
        phases.append(dict(mode=mode,start_s=start,end_s=now,ending_SOC=float(y[0]),ending_temperature_C=float(y[2])))
        if mode=='charging':
            if len(sol.t_events[0]):break
            cool=bool(len(sol.t_events[1]));warm=bool(len(sol.t_events[2]))
        elif len(sol.t_events[0]):cool=False;warm=False
        if not any(len(e) for e in sol.t_events):break
    completed=bool(y[0]>=target_soc-1e-8)
    return dict(t=np.array(times),y=np.array(states).T,phases=phases,
                thermal_memory=dict(hot_blocked=bool(cool)),
                completed=completed,stop_reason=None if completed else 'physical_horizon_exhausted')
