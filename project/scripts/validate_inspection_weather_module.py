"""Weather data semantics, nonanticipativity and resource-window optimization."""
from pathlib import Path
import sys,json,hashlib,datetime,itertools,time,copy
import numpy as np
from pyscipopt import Model,quicksum
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from Agent.Publication.engineering_reference_weather import *
from Agent.Publication.weather_factors import wind_uv
from Agent.Publication.engineering_replanning import replan_reasons
OUT=ROOT/'总结/03_实验数据/工程化联合求解/module_validation/weather'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    sp=ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes/rommerskirchen.json'
    scene=json.loads(sp.read_text(encoding='utf-8'));packets=build_reference_packets(scene)
    fp=ROOT/'Agent/Publication/inspection_reference_weather_profile.json'
    profile=dict(profile_id='rommerskirchen_causal_reference_weather',packets=packets,
      source_scene_sha256=sha(sp),forecast_status='synthetic reference experiment; retained ERA5 never relabelled an issued forecast',
      reference_uncertainty='three fixed common residual blocks; no probability, empirical forecast skill or safety confidence',
      near_ground_and_visibility='explicit numerical assumptions, not site measurements',
      observation_model='residual sign observed without noise at660s; earlier commands remain nonanticipative',
      replanning_period_s=300.,forecast_update_delay_s=30.,
      tracking_error_trigger_m=.5,SOC_prediction_error_trigger=.03,minimum_takeover_lead_s=1.)
    write(fp,profile)
    protocol=dict(experiment_id='inspection_weather_information_component',profile_sha256=sha(fp),
      source_sha256=sha(ROOT/'Agent/Publication/engineering_reference_weather.py'),validator_sha256=sha(Path(__file__)),
      purpose='verify common field, causal packet availability, near-ground domain and nonanticipative decisions',
      scope='component reference experiments; not full mission or empirical forecast verification')
    write(OUT/'protocol.json',protocol)
    checks=[];begin=time.perf_counter()
    def check(name,ok,**kw):checks.append(dict(name=name,passed=bool(ok),**kw))
    check('wind_from_direction_conversion',np.linalg.norm(wind_uv(5,270)-[5,0])<1e-10 and np.linalg.norm(wind_uv(5,0)-[0,-5])<1e-10)
    check('future_forecast_not_available',planning_packet(packets,629,28800)['packet_id']=='reference_initial')
    check('received_update_becomes_available',planning_packet(packets,630,28800)['packet_id']=='reference_update')
    fake=copy.deepcopy(packets);fake[0]['data_role']=fake[1]['data_role']='reanalysis'
    try:planning_packet(fake,0,28800);rejected=False
    except ValueError:rejected=True
    check('hindcast_rejected_as_online_forecast',rejected)
    bad=copy.deepcopy(packets);bad[0]['available_at_s']=-4000
    try:planning_packet(bad,0,28800);rejected=False
    except ValueError:rejected=True
    check('receipt_before_issue_rejected',rejected)
    domain_bad=0
    for t,h in [(-1,30),(28801,30),(0,-.1),(0,101)]:
        try:weather_at(packets[0],'nominal',t,h)
        except ValueError:domain_bad+=1
    check('outside_weather_domain_rejected',domain_bad==4)
    low=weather_at(packets[0],'east_hot',1400,0)
    near=weather_at(packets[0],'east_hot',1400,15)
    high=weather_at(packets[0],'east_hot',1400,30)
    check('near_ground_ramp_continuous',np.linalg.norm(low['wind_enu_mps'])<1e-12 and np.linalg.norm(np.array(near['wind_enu_mps'])*2-high['wind_enu_mps'])<1e-10)
    prefix=[]
    for t in [0,150,599,600]:
        vals=[weather_at(packets[0],s['id'],t,50) for s in packets[0]['scenarios']]
        prefix.append(max(np.linalg.norm(np.array(v['wind_enu_mps'])-vals[0]['wind_enu_mps']) for v in vals))
    check('common_observation_prefix',max(prefix)<1e-10 and len(nonanticipativity_pairs(packets[0],659))==3 and len(nonanticipativity_pairs(packets[0],660))==0)
    sample=weather_at(packets[0],'nominal',3600,30)
    raw=next(r for r in scene['weather']['records'] if r['time_utc']=='2025-07-01T09:00Z' and r['height_agl_m']==30)
    check('source_knot_and_hourly_flags',np.linalg.norm(np.array(sample['wind_enu_mps'])[:2]-[raw['wind_east_ms'],raw['wind_north_ms']])<1e-10
          and sample['previous_hour_precipitation_mm']==raw['precipitation_previous_hour_mm'])
    endpoint_packet=copy.deepcopy(packets[0])
    endpoint_packet['records'][-1]['rows'][0]['is_day']=0
    endpoint_packet['records'][-1]['rows'][0]['precipitation_previous_hour_mm']=2.
    endpoint=weather_at(endpoint_packet,'nominal',28800,30)
    prior=weather_at(endpoint_packet,'nominal',28799,30)
    check('terminal_hour_uses_own_flags',not endpoint['daylight'] and endpoint['previous_hour_precipitation_mm']==2.
          and not endpoint['flight_screen_pass'] and prior['daylight'] and prior['previous_hour_precipitation_mm']==0.)
    rows=[]
    for s,t,h in itertools.product(packets[0]['scenarios'],[0,600,900,1800,3600,14400,28799],[0,15,30,60,100]):
        v=weather_at(packets[0],s['id'],t,h)
        rows.append(dict(scenario=s['id'],time_s=t,height_agl_m=h,**v))
    check('positive_density_and_shared_bounded_residual',all(x['density_kg_m3']>0 and abs(x['wind_enu_mps'][2])<=.4+1e-9 for x in rows))
    check('event_availability_filters_future',replan_reasons({'time_s':629},600,300,[dict(type='forecast_update',available_at_s=630)])==[]
          and replan_reasons({'time_s':630},600,300,[dict(type='forecast_update',available_at_s=630)])==['forecast_update'])
    # Two-stage binary decision example with exact enumeration. Outcome value
    # coefficients are synthetic teaching data, unrelated to inspection score.
    # Choose route before branch; recourse may select a later slot after660s.
    costs=np.array([[4.,10.],[9.,5.],[6.,7.]])
    outputs=[]
    for causal in [False,True]:
        m=Model('weather_nonanticipativity');m.hideOutput();x={}
        for s in range(3):
            for r in range(2):x[s,r]=m.addVar(f'route_s{s}_{r}',vtype='B')
            m.addCons(x[s,0]+x[s,1]==1)
        if causal:
            for s in [1,2]:m.addCons(x[s,0]==x[0,0])
        worst=m.addVar('worst_cost',lb=0)
        for s in range(3):m.addCons(worst>=quicksum(float(costs[s,r])*x[s,r] for r in range(2)))
        m.setObjective(worst);t0=time.perf_counter();m.optimize();elapsed=time.perf_counter()-t0
        choices=[int(round(m.getVal(x[s,1]))) for s in range(3)]
        candidates=[v for v in itertools.product([0,1],repeat=3) if not causal or len(set(v))==1]
        oracle=min(max(costs[s,r] for s,r in enumerate(v)) for v in candidates)
        outputs.append(dict(nonanticipativity=causal,status=str(m.getStatus()),objective=m.getObjVal(),
           route_choices=choices,oracle_objective=oracle,wall_s=elapsed,online_executable_before_observation=len(set(choices))==1))
        check('nonanticipative_SCIP_enumeration_'+str(causal),str(m.getStatus())=='optimal' and abs(m.getObjVal()-oracle)<1e-8)
    check('anticipative_solution_is_rejected',outputs[0]['objective']<outputs[1]['objective'] and not outputs[0]['online_executable_before_observation'] and outputs[1]['online_executable_before_observation'])
    result=dict(protocol=protocol,observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),checks=checks,
      all_checks_passed=all(x['passed'] for x in checks),decision_examples=outputs,sampled_weather=rows,
      wall_s=time.perf_counter()-begin,actual_forecast_calibration=False,complete_joint_model_verified=False)
    (OUT/'attempts').mkdir(exist_ok=True);attempt=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S')
    write(OUT/'attempts'/f'{attempt}.json',result);write(OUT/'results.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='sampled_weather'},ensure_ascii=False,indent=2))
    if not result['all_checks_passed']:raise SystemExit(1)

if __name__=='__main__':main()
