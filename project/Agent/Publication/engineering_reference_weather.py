"""Causal synthetic forecast experiment anchored to retained real geography.

ERA5 records remain hindcast data. Issuance, residual blocks, visibility and
near-ground/vertical wind are explicitly declared simulation assumptions.
There are no fitted probabilities or forecast-skill claims.
"""
from __future__ import annotations
import copy,datetime,math
import numpy as np
from Agent.Publication.weather_factors import density_surface
from Agent.Publication.engineering_replanning import available_forecast


def build_reference_packets(scene,start_utc='2025-07-01T08:00:00+00:00',horizon_s=28800):
    start=datetime.datetime.fromisoformat(start_utc)
    lookup={(r['time_utc'],r['height_agl_m']):r for r in scene['weather']['records']}
    records=[]
    for hour in range(math.ceil(horizon_s/3600)+1):
        stamp=(start+datetime.timedelta(hours=hour)).strftime('%Y-%m-%dT%H:00Z')
        rows=[]
        for h in [30,60,100]:
            if (stamp,h) not in lookup:raise ValueError('Missing retained weather anchor: '+stamp)
            rows.append(copy.deepcopy(lookup[stamp,h]))
        records.append(dict(time_s=hour*3600.,rows=rows))
    common=dict(data_role='forecast',source_id='declared_synthetic_forecast_from_retained_ERA5_anchor',
       actual_issued_forecast=False,epoch_utc=start_utc,valid_from_s=0.,valid_until_s=float(horizon_s),
       source_scene_id=scene['scene_id'],spatial_model='one uniform grid cell; no downscaling claim',
       scenario_kind='deterministic bounded reference blocks, not empirical residual distribution',
       near_ground=dict(model='linear velocity ramp from zero at0m to retained30m value',height_m=[0,30],
                        horizontal_bound_mps=15.,vertical_amplitude_mps=.4),
       visibility_m=5000.,minimum_capture_visibility_m=1000.,
       wind_limit_mps=10.,gust10_limit_mps=10.,temperature_range_C=[0.,40.],
       records=records)
    scenarios=[]
    for sid,sign in [('nominal',0.),('east_hot',1.),('west_cool',-1.)]:
        scenarios.append(dict(id=sid,residual_sign=sign,wind_residual_amplitude_mps=2.,
           temperature_residual_C=2.,power_residual_fraction=.05*(sign+1.),
           available_observations=[dict(available_at_s=660.,key='observed_residual_sign',value=sign)]))
    # Synthetic forecast update at t=600 only becomes accessible at t=630.
    first=dict(common,issued_at_s=-3600.,available_at_s=-3300.,packet_id='reference_initial',scenarios=scenarios)
    update=copy.deepcopy(first);update.update(issued_at_s=600.,available_at_s=630.,packet_id='reference_update')
    for s in update['scenarios']:s['wind_residual_amplitude_mps']=1.
    return [first,update]


def information_group(scenario,now_s):
    return tuple((x['key'],str(x['value'])) for x in sorted(scenario['available_observations'],key=lambda x:x['available_at_s']) if x['available_at_s']<=now_s)


def nonanticipativity_pairs(packet,now_s):
    scenarios=packet['scenarios']
    return [(a['id'],b['id']) for i,a in enumerate(scenarios) for b in scenarios[i+1:]
            if information_group(a,now_s)==information_group(b,now_s)]


def weather_at(packet,scenario_id,time_s,height_agl_m):
    if not packet['valid_from_s']<=time_s<=packet['valid_until_s']:raise ValueError('Weather outside time domain')
    if not 0<=height_agl_m<=100:raise ValueError('Weather outside height domain')
    scenario=next((s for s in packet['scenarios'] if s['id']==scenario_id),None)
    if scenario is None:raise ValueError('Unknown scenario')
    # Continuous atmospheric fields interpolate in time; flags and hourly
    # accumulation use their original left-supported hour without smoothing.
    knots=packet['records'];times=[k['time_s'] for k in knots]
    left=min(len(knots)-2,max(0,int(np.searchsorted(times,time_s,side='right')-1)))
    frac=(time_s-times[left])/(times[left+1]-times[left])
    h=height_agl_m
    def vertical(rows,field):
        vals=[float(r[field]) for r in rows]
        if h<30 and field.startswith('wind_'):return vals[0]*h/30.
        return float(np.interp(max(h,30.),[30,60,100],vals))
    def field(name):return (1-frac)*vertical(knots[left]['rows'],name)+frac*vertical(knots[left+1]['rows'],name)
    sign=scenario['residual_sign']
    reveal=1.-math.exp(-max(time_s-600.,0.)/300.)
    # A smooth shared temporal block. No aircraft-specific resampling.
    phase=math.sin(2*math.pi*time_s/1800.)
    scale=min(h/30.,1.)
    we=field('wind_east_ms')+reveal*scale*sign*scenario['wind_residual_amplitude_mps']*phase
    wn=field('wind_north_ms')+reveal*scale*sign*.5*scenario['wind_residual_amplitude_mps']*math.sin(2*math.pi*time_s/2400.)
    wz=reveal*scale*sign*packet['near_ground']['vertical_amplitude_mps']*math.sin(2*math.pi*time_s/60.)
    Ta=field('temperature_2m_C')+reveal*sign*scenario['temperature_residual_C']
    dew=min(field('dew_point_2m_C'),Ta)
    pressure=field('surface_pressure_hPa')
    # Explicit near-ground temperature uses the surface thermodynamic data.
    rho=density_surface(Ta,dew,pressure)*math.exp(-9.80665*h/(287.05*(Ta+273.15)))
    # The terminal knot has its own observation. Do not borrow the preceding
    # hour's categorical fields merely because interpolation needs two knots.
    flag_index=min(len(knots)-1,max(0,int(np.searchsorted(times,time_s,side='right')-1)))
    flags=knots[flag_index]['rows'][0]
    gust=float(flags['wind_gusts_10m_ms'])+reveal*abs(sign)*scenario['wind_residual_amplitude_mps']
    permitted=(np.linalg.norm([we,wn,wz])<=packet['wind_limit_mps'] and gust<=packet['gust10_limit_mps']
       and packet['temperature_range_C'][0]<=Ta<=packet['temperature_range_C'][1]
       and flags['precipitation_previous_hour_mm']==0 and bool(flags['is_day'])
       and packet['visibility_m']>=packet['minimum_capture_visibility_m'])
    return dict(wind_enu_mps=[we,wn,wz],temperature_C=Ta,density_kg_m3=rho,
       gust10_mps=gust,previous_hour_precipitation_mm=float(flags['precipitation_previous_hour_mm']),
       daylight=bool(flags['is_day']),visibility_m=packet['visibility_m'],flight_screen_pass=bool(permitted),
       power_residual_fraction=reveal*scenario['power_residual_fraction'],
       source_hour=flags['time_utc'],synthetic_weather_scenario=True)


def planning_packet(packets,now_s,end_s):
    return available_forecast(packets,now_s,end_s)
