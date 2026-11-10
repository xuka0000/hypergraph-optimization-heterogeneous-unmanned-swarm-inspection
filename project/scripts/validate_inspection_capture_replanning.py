"""Image geometry and physical-state takeover evidence, with adversarial cases."""
from pathlib import Path
import sys,json,hashlib,datetime,copy,math
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from Agent.Publication.engineering_image_acceptance import evaluate_capture
from Agent.Publication.engineering_replanning import validate_candidate_takeover
OUT=ROOT/'总结/03_实验数据/工程化联合求解/module_validation/capture_replanning'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    sp=ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes/rommerskirchen.json'
    scene=json.loads(sp.read_text(encoding='utf-8'));cam=scene['joint_reference_profile']['camera']
    profile=dict(profile_id='inspection_reference_image_acceptance',parameters=cam,minimum_visibility_m=1000.,
      source_scene_sha256=sha(sp),hardware_source='https://enterprise.dji.com/zenmuse-h20-series/specs',
      manufacturer_fields=['image dimensions','focal range','wide diagonal FOV'],
      declared_assumptions=['ideal distortion-free projection','principal point at center','zero mounting offset',
       'GSD0.005m/px','blur1px','exposure0.001s','visibility1000m','0.5m template patch'],
      image_quality_role='simulated geometry; real image content/illumination acceptance remains a measured validation requirement')
    fp=ROOT/'Agent/Publication/inspection_reference_image_profile.json';write(fp,profile)
    checks=[]
    def check(name,ok,**kw):checks.append(dict(name=name,passed=bool(ok),**kw))
    protocol=dict(experiment_id='inspection_capture_and_takeover_components',profile_sha256=sha(fp),
        camera_source_sha256=sha(ROOT/'Agent/Publication/engineering_image_acceptance.py'),
        replanning_source_sha256=sha(ROOT/'Agent/Publication/engineering_replanning.py'),validator_sha256=sha(Path(__file__)),
        purpose='verify 2D exposure blur and forbid physical state resets during takeover',full_mission=False)
    write(OUT/'protocol.json',protocol)
    points=np.array([[0,0,15],[-.25,-.25,15],[.25,-.25,15],[-.25,.25,15],[.25,.25,15]])
    tangents=np.array([[1,0],[0,1],[0,0]])
    poses=[dict(time_s=t,position_m=[0,0,0],rotation=np.eye(3),focal_length_mm=6.83) for t in [0,.0005,.001]]
    fixtures=[]
    def run(name,pp=poses,v=5000,ray=lambda p,q:True,expected=True):
        ans=evaluate_capture(points,[0,0,-1],tangents,pp,cam,v,ray)
        fixtures.append(dict(name=name,**ans));check(name,ans['accepted']==expected,details=ans)
        return ans
    still=run('stationary_reference_image')
    F=math.hypot(5184,3888)/(2*math.tan(math.radians(66.6)/2))
    check('pinhole_gsd_matches_closed_form',abs(still['worst_gsd_m_per_px']-15/F)<1e-10)
    moving=copy.deepcopy(poses)
    for row in moving:row['position_m']=[.8*15/F*row['time_s']/.001]*2+[0.]
    blur=run('diagonal_blur_rejected',moving,expected=False)
    check('euclidean_blur_oracle',abs(blur['blur_px']-math.sqrt(2)*.8)<1e-8 and 'blur' in blur['reasons'],expected_px=math.sqrt(2)*.8)
    run('unknown_visibility_ray_rejected',ray=lambda p,q:None,expected=False)
    run('occlusion_rejected',ray=lambda p,q:False,expected=False)
    run('low_visibility_rejected',v=500,expected=False)
    cropped=copy.deepcopy(poses)
    for row in cropped:row['position_m']=[100,0,0]
    run('outside_frame_and_range_rejected',cropped,expected=False)
    # A real physical trace is used for the takeover state, not an invented
    # full-battery snapshot. This uses the already-run reference flight.
    traces=np.load(ROOT/'总结/03_实验数据/工程化联合求解/module_validation/flight/tracking_traces.npz')
    y=traces['time_varying_3d_wind_y'][:,-1]
    state=dict(time_s=40.,revision=2,
      vehicles={'V1':dict(position_m=[0,0,0])},
      aircraft={'U1':dict(position_m=y[:3].tolist(),velocity_mps=y[3:6].tolist(),body_quaternion_wxyz=y[6:10].tolist(),
                         body_rate_radps=y[10:13].tolist(),rotor_thrust_N=y[13:17].tolist(),mode='airborne',battery_pair_id='B1',sortie_id='L1')},
      battery_pairs={'B1':dict(SOC=.8862740967320168,polarization_V=.25,temperature_C=25.248219097935767,charge_hot_blocked=False,
                              grid_input_Wh=0.,load_output_Wh=8.186145358005076,resistive_heat_Wh=.1,mode='in_flight',aircraft_id='U1')},
      committed_actions=[dict(action_id='hold_L1',start_s=39.,end_s=41.)])
    # Battery values must come from a replay, including polarization and heat.
    from Agent.Publication.engineering_battery_electrothermal import initial_state,simulate_power_trace,BatteryParameters
    b=simulate_power_trace(initial_state(.9,25),traces['time_varying_3d_wind_t'],traces['time_varying_3d_wind_power_W'],25,BatteryParameters())['y'][:,-1]
    for key,val in zip(['SOC','polarization_V','temperature_C','grid_input_Wh','load_output_Wh','resistive_heat_Wh'],b):state['battery_pairs']['B1'][key]=float(val)
    candidate=dict(state_revision=2,information_time_s=30.,ready_at_s=39.,takeover_state=copy.deepcopy(state),committed_actions=copy.deepcopy(state['committed_actions']))
    check('actual_takeover_state_accepted',validate_candidate_takeover(candidate,state,40.)['state_compatible'])
    failures=[]
    mutations={
       'SOC_reset':lambda c:c['takeover_state']['battery_pairs']['B1'].update(SOC=1.),
       'polarization_reset':lambda c:c['takeover_state']['battery_pairs']['B1'].update(polarization_V=0.),
       'temperature_reset':lambda c:c['takeover_state']['battery_pairs']['B1'].update(temperature_C=25.),
       'thermal_protection_memory_reset':lambda c:c['takeover_state']['battery_pairs']['B1'].update(charge_hot_blocked=True),
       'rotor_reset':lambda c:c['takeover_state']['aircraft']['U1'].update(rotor_thrust_N=[0]*4),
       'lost_commitment':lambda c:c.update(committed_actions=[]),
       'stale_revision':lambda c:c.update(state_revision=1),
       'future_candidate':lambda c:c.update(ready_at_s=41.),
       'position_reset':lambda c:c['takeover_state']['aircraft']['U1'].update(position_m=[0,0,0])}
    for name,mutate in mutations.items():
        c=copy.deepcopy(candidate);mutate(c)
        try:validate_candidate_takeover(c,state,40.);rejected=False
        except ValueError as e:rejected=True;failures.append(dict(case=name,reason=str(e)))
        check('reject_'+name,rejected)
    c=copy.deepcopy(candidate);c['takeover_state']['aircraft']['U1']['body_quaternion_wxyz']=(-y[6:10]).tolist()
    check('quaternion_sign_equivalence_preserved',validate_candidate_takeover(c,state,40.)['state_compatible'])
    result=dict(protocol=protocol,observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),checks=checks,
        all_checks_passed=all(c['passed'] for c in checks),image_fixtures=fixtures,takeover_state=state,
        negative_takeover_cases=failures,full_joint_solver_verified=False,real_image_quality_calibrated=False)
    (OUT/'attempts').mkdir(exist_ok=True);attempt=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S')
    write(OUT/'attempts'/f'{attempt}.json',result);write(OUT/'results.json',result)
    print(json.dumps(dict(passed=sum(c['passed'] for c in checks),total=len(checks),failed=[c for c in checks if not c['passed']],images=fixtures),ensure_ascii=False,indent=2))
    if not result['all_checks_passed']:raise SystemExit(1)

if __name__=='__main__':main()
