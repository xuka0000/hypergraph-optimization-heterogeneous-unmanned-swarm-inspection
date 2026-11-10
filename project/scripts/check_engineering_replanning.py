"""Known-case checks of causal inputs; NOT an inspection optimization run."""
import copy
import datetime
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'Agent/Publication'))
from engineering_replanning import prepare_joint_replanning, replan_reasons, validate_snapshot
from engineering_joint_optimality import REQUIRED_CLOSURES, require_complete_model


def check_actual_entry_scope():
    checks=[]
    with tempfile.TemporaryDirectory(prefix='inspection_scope_check_') as temp:
        root=Path(temp);e=root/'evidence.txt';e.write_text('known-case only',encoding='utf-8')
        ev=dict(path=e.name,sha256=hashlib.sha256(e.read_bytes()).hexdigest())
        data=dict(closures={k:dict(status='verified',evidence=[ev]) for k in REQUIRED_CLOSURES},
                  implementation_files=[ev])
        path=root/'record.json';path.write_text(json.dumps(data),encoding='utf-8')
        require_complete_model(root,path.name);checks.append('legacy-known-case-preserved')
        data['rolling_scope_extension']={'mathematical_definition':True}
        path.write_text(json.dumps(data),encoding='utf-8')
        try:require_complete_model(root,path.name)
        except RuntimeError as error:
            for key in ['full_regional_task_domain','causal_forecast_and_nonanticipativity','execution_state_and_takeover']:
                assert key in str(error)
            checks.append('legacy-completeness-cannot-substitute-for-rolling-scope')
        else:raise AssertionError('rolling scope omitted from actual entry')
    return dict(passed=2,total=2,checks=checks,
        module_sha256=hashlib.sha256((ROOT/'Agent/Publication/engineering_joint_optimality.py').read_bytes()).hexdigest(),
        full_solver_executed=False)


def fixture(scene):
    now=600.0
    state=dict(scene_id=scene['scene_id'],epoch_utc='2025-07-01T08:00:00Z',time_s=now,
        revision=1,vehicles={},aircraft={},battery_pairs={},tasks={},committed_actions=[])
    for v in scene['fleet']['vehicles']:
        state['vehicles'][v['id']]=dict(position_m=v['initial_xyz_m'],yaw_rad=v['initial_heading_rad'],
            speed_mps=0.,steering_rad=0.,observed_at_s=now)
    for u in scene['fleet']['aircraft']:
        yaw=u['initial_yaw_rad']
        state['aircraft'][u['id']]=dict(mode='stowed',position_m=u['launch_anchor_xyz_m'],
            velocity_mps=[0.,0.,0.],acceleration_mps2=[0.,0.,0.],
            body_quaternion_wxyz=[math.cos(yaw/2),0.,0.,math.sin(yaw/2)],
            gimbal_rad=[0.,0.],yaw_rad=yaw,observed_at_s=now)
    for b in scene['fleet']['battery_pairs']:
        state['battery_pairs'][b['id']]=dict(mode='available',energy_Wh=b['initial_Wh'],
            available_after_s=now,observed_at_s=now)
    for t in scene['inspection_tasks']:state['tasks'][t['id']]=dict(status='pending')
    # Deliberately synthetic state, never labelled measured or dynamically feasible.
    u=scene['fleet']['aircraft'][0]['id'];p=scene['fleet']['battery_pairs'][0]['id']
    state['aircraft'][u].update(mode='airborne',position_m=[338500.,5653300.,120.],
        velocity_mps=[1.,0.,0.],battery_pair_id=p,sortie_id='fixture-sortie')
    state['battery_pairs'][p].update(mode='in_flight',energy_Wh=321.,aircraft_id=u)
    keys=list(state['tasks'])
    state['tasks'][keys[0]]=dict(status='accepted',image_quality_accepted=True,
        accepted_at_s=590.,evidence_id='synthetic-fixture-image')
    state['tasks'][keys[1]]=dict(status='failed',image_quality_accepted=False)
    state['committed_actions']=[dict(action_id='fixture-prefix',start_s=599.,end_s=610.,aircraft_id=u),
                                dict(action_id='fixture-past',start_s=580.,end_s=590.,aircraft_id=u)]
    packet=dict(data_role='forecast',source_id='synthetic-known-case-NOT-real-forecast',
        issued_at_s=0.,available_at_s=5.,valid_from_s=0.,valid_until_s=28800.,
        scenarios=[dict(id='fixture-low'),dict(id='fixture-high')])
    return state,packet


def main():
    checks=[];source=[]
    def check(name,cond):
        checks.append(dict(id=name,passed=bool(cond)))
        if not cond:raise AssertionError(name)
    def rejects(name,fn):
        try:fn()
        except ValueError:check(name,True)
        else:check(name,False)
    for name in ['rommerskirchen','brauweiler','villevaude']:
        path=ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes'/f'{name}.json'
        scene=json.loads(path.read_text(encoding='utf-8-sig'));state,forecast=fixture(scene)
        inp=prepare_joint_replanning(scene,state,[forecast],28800.)
        check(name+'_all_remaining_tasks',len(inp['remaining_task_ids'])==len(scene['inspection_tasks'])-1)
        check(name+'_full_stop_domain',inp['full_stop_domain']['count']==len(scene['stops']))
        check(name+'_full_road_locator',inp['full_road_domain']['locator']=='/roads')
        source.append(dict(scene_id=name,towers=len(scene['towers']),tasks=len(scene['inspection_tasks']),
            stops=len(scene['stops']),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        if name=='rommerskirchen':base=(scene,state,forecast,inp)
    scene,state,forecast,inp=base
    u=scene['fleet']['aircraft'][0]['id'];p=scene['fleet']['battery_pairs'][0]['id'];ids=list(state['tasks'])
    check('no_battery_reset',inp['initial_state']['battery_pairs'][p]['energy_Wh']==321.)
    check('no_aircraft_teleport',inp['initial_state']['aircraft'][u]['position_m']==[338500.,5653300.,120.])
    check('ongoing_battery_identity',inp['ongoing_sorties'][u]==dict(sortie_id='fixture-sortie',battery_pair_id=p))
    check('rejected_image_still_pending',ids[1] in inp['remaining_task_ids'])
    check('only_unexpired_commitments_carried',[a['action_id'] for a in inp['committed_actions']]==['fixture-prefix'])
    future=copy.deepcopy(forecast);future.update(issued_at_s=601.,available_at_s=602.)
    unavailable=copy.deepcopy(forecast);unavailable.update(issued_at_s=500.,available_at_s=601.)
    truth=copy.deepcopy(forecast);truth.update(data_role='reanalysis',issued_at_s=550.,available_at_s=551.)
    selected=prepare_joint_replanning(scene,state,[truth,future,unavailable,forecast],28800.)
    check('no_future_or_reanalysis_leak',selected['forecast']==forecast)
    rejects('missing_causal_forecast_fails',lambda:prepare_joint_replanning(scene,state,[truth,future,unavailable],28800.))
    bad=copy.deepcopy(state);bad['tasks'][ids[0]]['image_quality_accepted']=False
    rejects('planned_photo_not_accepted',lambda:validate_snapshot(scene,bad))
    bad=copy.deepcopy(state);bad['tasks'][ids[0]]['accepted_at_s']=601.
    rejects('future_image_not_accepted',lambda:validate_snapshot(scene,bad))
    bad=copy.deepcopy(state);bad['aircraft'][u]['observed_at_s']=599.
    rejects('unsynchronized_state_fails',lambda:validate_snapshot(scene,bad))
    bad=copy.deepcopy(state);other=scene['fleet']['aircraft'][1]['id']
    bad['aircraft'][other]=copy.deepcopy(bad['aircraft'][u])
    rejects('duplicate_inflight_pack_fails',lambda:validate_snapshot(scene,bad))
    recovering=copy.deepcopy(state);recovering['aircraft'][u]['mode']='recovering'
    check('recovering_keeps_battery',u in prepare_joint_replanning(scene,recovering,[forecast],28800.)['ongoing_sorties'])
    low=copy.deepcopy(state);low['battery_pairs'][p]['energy_Wh']=10.
    check('low_energy_truth_retained',prepare_joint_replanning(scene,low,[forecast],28800.)['initial_state']['battery_pairs'][p]['energy_Wh']==10.)
    check('causal_trigger_clock',replan_reasons(state,590.,30.,[
        dict(type='forecast_update',available_at_s=590.),dict(type='road_closure',available_at_s=595.),
        dict(type='safety_alarm',available_at_s=601.)])==['road_closure'])
    check('periodic_trigger',replan_reasons(state,500.,30.,[])==['periodic'])
    rejects('future_previous_clock_fails',lambda:replan_reasons(state,601.,30.,[]))
    bad=copy.deepcopy(state);bad['epoch_utc']='2025-07-01T08:00:00'
    rejects('naive_epoch_fails',lambda:validate_snapshot(scene,bad))
    inp['initial_state']['battery_pairs'][p]['energy_Wh']=0.
    check('input_not_mutated',state['battery_pairs'][p]['energy_Wh']==321.)
    check('not_claimed_joint_solver',selected['solver_implemented'] is False)
    record=dict(observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope='causal input interface on complete source scenes, synthetic state/forecast fixtures only',
        physical_feasibility_evaluated=False,optimization_runs=0,large_scale_joint_solve_verified=False,
        source_scenes=source,checks=checks,passed=sum(x['passed'] for x in checks),total=len(checks),
        module_sha256=hashlib.sha256((ROOT/'Agent/Publication/engineering_replanning.py').read_bytes()).hexdigest())
    record['engineering_attempt_history']=[dict(attempt=1,status='failed',reason='test fixture used initial_energy_Wh; source field is initial_Wh'),
        dict(attempt=2,status='passed',checks=27),dict(attempt=3,status='passed',checks=len(checks),reason='explicit UTC epoch validation added')]
    record['actual_entry_scope_checks']=check_actual_entry_scope()
    dest=ROOT/'research_quality/inspection_replanning_validation.json'
    dest.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'passed':record['passed'],'total':record['total'],'optimization_runs':0,
                      'evidence':str(dest)},ensure_ascii=False))


if __name__=='__main__':main()
