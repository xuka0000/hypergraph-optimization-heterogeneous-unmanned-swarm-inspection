"""Fixed event examples for ongoing service, receipt latency and repeat identity."""
from pathlib import Path
import copy,datetime,hashlib,json,sys,time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from Agent.Publication.engineering_service_boundary import service_boundary,check_service_plan,remaining_event_clock

def main():
    start=time.perf_counter();checks=[]
    def check(name,ok,detail=None):checks.append(dict(name=name,passed=bool(ok),detail=detail))
    def rejects(name,fn):
        try:fn();check(name,False)
        except ValueError as e:check(name,True,str(e))
    a=dict(attempt_id='a1',task_id='P01',start_s=10.,end_s=30.,aircraft_id='U1',battery_pair_id='B1',sortie_id='L1',
       exposure_intervals_s=[[11.,11.001]],prefix_evidence_id='trajectory-a1',
       acceptance_event_at_s=30.,acceptance_received_at_s=35.,quality_passed=True,image_evidence_id='image-a1')
    b=service_boundary(['P01','P02'],[a],11.0005)
    check('split_exposure_without_restart',abs(b['ongoing']['P01']['remaining_s']-18.9995)<1e-10 and
      b['ongoing']['P01']['executed_exposure_intervals_s']==[[11.,11.0005]] and
      b['ongoing']['P01']['remaining_exposure_intervals_s']==[[11.0005,11.001]],b['ongoing']['P01'])
    check('future_result_not_leaked','quality_passed' not in json.dumps(b) and not b['accepted_task_ids'])
    check('ongoing_task_not_new_demand','P01' not in b['new_service_eligible_task_ids'])
    clock=remaining_event_clock(10.,30.,11.0005)
    check('source_arc_zero_duration_after_cut',clock['start_s']==11.0005)
    check('remaining_path_time_and_energy_partition',abs(clock['duration_s']+15-(45.-11.0005))<1e-12,
          dict(remaining_flight_s=clock['duration_s']+15,past_prefix_excluded_s=1.0005,
               constant_1200W_remaining_Wh=1200*(clock['duration_s']+15)/3600))
    check('valid_continuation',check_service_plan(b,[],copy.deepcopy(b['ongoing']))['bookkeeping_valid'])
    for field,value in [('remaining_s',20.),('battery_pair_id','B2'),('original_start_s',11.0005),('executed_exposure_intervals_s',[])]:
        c=copy.deepcopy(b['ongoing']);c['P01'][field]=value
        rejects('reject_reset_'+field,lambda c=c:check_service_plan(b,[],c))
    rejects('reject_missing_ongoing',lambda:check_service_plan(b,[],{}))
    pending=service_boundary(['P01','P02'],[a],30.)
    check('half_open_service_end_is_pending',not pending['ongoing'] and len(pending['pending_acceptance'])==1 and not pending['accepted_task_ids'])
    check('received_acceptance_credits_once',service_boundary(['P01','P02'],[a],35.)['accepted_task_ids']==['P01'])
    failed=copy.deepcopy(a);failed['quality_passed']=False
    f=service_boundary(['P01'],[failed],35.)
    check('failed_image_remains_demand',not f['accepted_task_ids'] and len(f['failed_attempts'])==1 and f['new_service_eligible_task_ids']==['P01'])
    repeat=dict(attempt_id='a2',task_id='P01',start_s=35.,end_s=55.)
    check('repeat_gets_distinct_identity',check_service_plan(f,[repeat],{})['bookkeeping_valid'])
    repeat['attempt_id']='a1';rejects('reject_recycled_attempt_id',lambda:check_service_plan(f,[repeat],{}))
    bad=copy.deepcopy(a);bad['image_evidence_id']='';rejects('reject_no_image_evidence',lambda:service_boundary(['P01'],[bad],35.))
    bad=copy.deepcopy(a);bad.update(attempt_id='a2',task_id='P02',start_s=20.,end_s=40.,exposure_intervals_s=[[21.,21.001]],acceptance_event_at_s=40.,acceptance_received_at_s=45.)
    rejects('reject_same_aircraft_service_overlap',lambda:service_boundary(['P01','P02'],[a,bad],25.))
    source=ROOT/'Agent/Publication/engineering_service_boundary.py'
    result=dict(experiment_id='inspection_service_boundary_component',observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
       source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),checks=checks,all_checks_passed=all(c['passed'] for c in checks),
       examples=dict(input_attempt=a,split_boundary=b,pending_boundary=pending),wall_s=time.perf_counter()-start,
       scope='event bookkeeping only; no mission optimization, exposure-geometry proof or field validation')
    out=ROOT/'总结/03_实验数据/工程化联合求解/module_validation/service_boundary';out.mkdir(parents=True,exist_ok=True)
    (out/'results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(passed=sum(c['passed'] for c in checks),total=len(checks),failures=[c for c in checks if not c['passed']]),ensure_ascii=False))
    if not result['all_checks_passed']:raise SystemExit(1)
if __name__=='__main__':main()
