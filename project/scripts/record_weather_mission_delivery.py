"""Record a completed conditional diagnostic without expanding research scope."""
import os,pathlib,shutil,subprocess,sys,time
import psutil
from run_weather_mission_coupling import ROOT,OUT,TASK,read,save,ref,now
from record_m100_validation_delivery import check_record

def main():
    at=now();v=read(OUT/'delivery_validation.json');r=read(OUT/'results.json');pdf=read(OUT/'module_pdf_sync.json')
    assert v['passed_for_current_scope'] and pdf['status']=='compiled_and_new_pages_visually_verified'
    q=ROOT/'research_quality';bridge=pathlib.Path('D:/博士研究生/具身智能大论文架构/cross_work_bridge')
    files=[OUT/n for n in ['protocol.json','mission.json','cases.json','regression.json','execution_binding.json',
      'stage_scope_receipt.json','execution.json','results.json','trajectory_diagnosis.json','delivery_validation.json',
      'table_reference_mapping.json','天气_航段_能耗联算报告.txt','天气_航段_能耗联算报告.pdf','module_pdf_sync.json']]
    files += [ROOT/'Agent/Publication/weather_mission_adapter.py',ROOT/'scripts/verify_weather_mission_coupling.py',ROOT/'scripts/diagnose_weather_mission_coupling.py']
    materials=[{**ref(p),'locator':'whole artifact'} for p in files]
    reasons={
      'C01':('PASS','Five immutable public energy models with author checkpoint/source hashes and frozen train+validation scalers. Necessary body-yaw and signed-roll feature alignment recorded. Author dynamics/controller reused with exact-hover zero limit; original propagate path-advance policy explicitly NOT reproduced by fixed-time caller. No algorithm improvement or competition.'),
      'C02':('PASS','All60 saved energies and phase sums independently recomputed; all12 state trajectories checked against position/force recurrences, relative-air velocity, body rotation and common time. Numerical recurrence maximum error below1e-9; largest independent energy difference2.77e-7Wh. This verifies equations and implementation, not real flight accuracy.'),
      'C03':('PASS','One mapped T001/P02 mission with9 literature-template photo points;12 weather cases frozen before execution;5 models. No60 independent flights, no random-seed statistics or held-out field experiment.533.7s airborne interval common; warmup20s excluded; takeoff/landing excluded.'),
      'C04':('PASS','All cases, input-envelope violations, negative weather control and model disagreements retained. No post-hoc success threshold. Source-bound nominal79.92Wh screen is not mission feasibility. Phase diagnosis explicitly post-hoc and does not rerun or replace primary outcomes.'),
      'C05':('UNKNOWN','No measured power or tower-scale wind truth. Calm0C/35C and historical cases are outside source density range; calm15C has62.55percent rows below source airspeed envelope. Input bounds alone do not establish validity. Battery thermal/ageing and usable-capacity effects absent.'),
      'C06':('PASS','Protected author files, original scene and immutable protocol preserved. Independent saved-output verification passed. Five PDFs generated from complete results and visually reviewed. IEEE/CMU primary citation title and2021 conference verified. No source or power data modified for favorable findings.'),
      'C07':('PASS','One finite localCPU2 batch with900s internal/930s process guard completed60 predictions in230.11s. Zero training, GPU requirement, cloud expense or extra workers. All reservations/drive floors checked; existing SCIP identity preserved, not duplicated or restarted.'),
      'C08':('UNKNOWN','Weather-to-state-to-power-to-Wh interface executed with real new results. Fixed-time target/control lag reaches17.83m calm and90.69m headwind8m/s; photo-end errors separately retained. Task-completion-aware timing and source-bound photo acceptance needed before scheduling. No full-flight accuracy or hypergraph benefit claim.')}
    def put(s,d):p=q/f'{TASK}_{s}.json';save(p,d);return ref(p)
    manual={k:{'risk_id':k,'task_id':TASK,'stage':'research','verdict':value[0],
      'scope':'Bounded weather-mission interface diagnostic; full calibrated inspection model incomplete',
      'reviewer':'Current original Codex; not user acceptance or independent peer review','reason':value[1],'reviewed_materials':materials} for k,value in reasons.items()}
    mr=put('manual',manual);expected=put('report_expected',{'pairs':60,'cases':12,'models':5,'passed':True})
    checks=[{'id':'pairs','kind':'derived_consistency','scope':'All saved model-condition results','data':{**ref(OUT/'results.json'),'pointer':'/rows'},
      'operation':'count','reported':{**expected,'pointer':'/pairs'}},
      {'id':'verification','kind':'identity','scope':'Independent complete state and energy verification','expected':expected,'actual':ref(OUT/'delivery_validation.json'),
       'fields':[{'expected':'passed','actual':'passed_for_current_scope'},{'expected':'pairs','actual':'pairs'},{'expected':'cases','actual':'cases'},{'expected':'models','actual':'models'}]}]
    content=put('report_content',{'schema_version':1,'task_id':TASK,'stage':'research','checks':checks,
      'coverage':{k:{'checks':['pairs','verification'],'manual_review':{**mr,'locator':'/'+k,'pointer':'/'+k}} for k in reasons}})
    mappings={'R01':'C08','R02':'C01','R03':'C02','R04':'C05','R05':'C06','R06':'C04','R07':'C05','R08':'C07','R09':'C06','R10':'C08'}
    items=[{'id':k,'status':reasons[j][0],'reason':reasons[j][1],'evidence':[{**mr,'locator':'/'+j,'pointer':'/'+j}]} for k,j in mappings.items()]
    risks=[{'id':k,'status':value[0],'finding_status':'no_defect' if value[0]=='PASS' else 'review_lead','reason':value[1],
      'impact':'Applicability of conditional weather energy to completion of a real inspection','evidence':[{**mr,'locator':'/'+k,'pointer':'/'+k}]} for k,value in reasons.items()]
    next_action='建立按实际到达及有来源的拍摄验收推进的任务接口，核对完成时长/能耗；继续保留局地风场、实飞功率和电池温度标定缺口，未进入调度或算法比较。'
    put('self_check',{'policy_id':'RESEARCH-STAGE-SELF-CHECK-01','policy_version':'1.2','stage':'research','task_id':TASK,'checked_at':at,
      'subject':{'scope':'60 conditional weather-mission predictions completed; calibrated mission-feasibility model remains incomplete','materials':materials},
      'items':items,'task_completion':'partial','stage_outcome':'NOT_READY','risk_triggers':list(reasons),'risk_checks':risks,'content_manifest':content,
      'limitations':v['science_limits'],'next_action':next_action})
    check_record(TASK)
    evidence=[ref(p)['path'] for p in files]
    record={'id':TASK,'title':'Úbeda单塔九点任务的天气—航段—能耗联算',
      'purpose':'检验风和气温如何传递到实际状态、模型功率、空中段能量和参照可行性',
      'method':'同一533.7秒规定航段、12固定天气、5冻结作者模型；逐行状态递推、全部60能量独立核验，108拍摄时段事后定位偏差。',
      'result':'b-TCN无风15°C70.815Wh；去程逆风8m/s71.466Wh(+0.918%)；侧风8m/s62.411Wh(-11.867%)，b-LSTM同条件+0.960%。无风/逆风8m/s转移最大偏差17.828/90.693m；拍摄结束最大偏差近0/0.581m。',
      'remaining':next_action,'boundary':'条件仿真预测；没有实飞功率真值、完整起降、拍摄验收或电池温度衰减；不宣称原3.51%误差移植成功或超图优势。',
      'status':'conditional_interface_completed_full_mission_validation_pending','observed_at':time.time(),'execution_observed_at':r['observed_at'],
      'evidence':evidence,'progress':{'unit':'model-condition predictions','total':60,'completed':60,'running':0,'failed':0,'unique_missions':1,'conditions':12,'models':5,'photo_phases_diagnosed':108},
      'cost':0,'training':False,'elapsed_batch_seconds':r['seconds']}
    p=ROOT/'.codex/paper_progress.json';d=read(p)
    ex=d['modules']['results'].setdefault('experiments',[]);ex[:]=[x for x in ex if x.get('id')!=TASK];ex.append(record)
    for name in ['modeling','setup','results']:
        d['modules'][name].setdefault('content',{})[TASK]=record
        ev=d['modules'][name].setdefault('evidence',[])
        for e in evidence:
            if e not in ev:ev.append(e)
    d['modules']['algorithm'].setdefault('content',{})[TASK]={'status':'inspection_solver_unchanged','interface':'Weather through source dynamics to five immutable power models; no optimization experiment or stage transition.'}
    for rec in pdf['records']:
        if rec.get('module') not in ['modeling','algorithm','setup','results']:continue
        doc=d['modules'][rec['module']]['document'];doc.setdefault('source_files',[])
        if rec['shared_content']['path'] not in doc['source_files']:doc['source_files'].append(rec['shared_content']['path'])
        doc['latest_local_compile']={'observed_at':pdf['observed_at'],'pdf_sha256':rec['pdf']['sha256'],'pages':rec['pages'],'new_appendix_visual_review':'passed','shared_builder':'missing; native XeLaTeX fallback'}
    save(p,d)
    p=ROOT/'project_layout.json';d=read(p);d['geographic_inspection_scene'][TASK]={'root':ref(OUT/'results.json')['path'].rsplit('/',1)[0],
      'model':'Agent/Publication/weather_mission_adapter.py','report':ref(OUT/'天气_航段_能耗联算报告.pdf')['path'],'role':'Conditional coupling follow-up; one authoritative result set, prior scenarios and results unchanged'};save(p,d)
    live=[]
    try:
        proc=psutil.Process(40284);match=abs(proc.create_time()-1794046886.26545)<.1
        live.append({'pid':40284,'identity_matched':match,'status':proc.status() if match else 'PID_reused','CPU_seconds':sum(proc.cpu_times()[:2]) if match else None,'action':'preserved_no_restart'})
    except psutil.NoSuchProcess:live.append({'pid':40284,'status':'absent_at_readback','action':'no_restart'})
    p=ROOT/'.codex/experiment_goal_review.json';d=read(p);d[TASK]={'observed_at':at,'owner_thread_id':os.environ.get('CODEX_THREAD_ID'),'claim':record['purpose'],
      'new_evidence':evidence,'cost':0,'deadline':'Finite local batch completed; no live weather task or unattended supervision','next_action':next_action,
      'stop_condition':'Current fixed protocol completed; no full-field accuracy or optimizer stage inferred; existing pauses retained',
      'cloud_inspection':'First startup failure: ModuleNotFoundError cloud_services; provider list not reached, no mutation or blind retry',
      'cloud_counts':{k:None for k in ['in_use','idle','retained','unknown','pending_release']},'local_experiments':live};save(p,d)
    p=ROOT/'.codex/storage_manifest.json';d=read(p);d[TASK].update(observed_at=at,status='completed_conditional_outputs_retained',peak_write_reservation_bytes=0,C_peak_temporary_bytes=0,
      retained_logical_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),free_bytes={s:shutil.disk_usage(s+':/').free for s in ['C','D']},reclaimed_bytes=0);save(p,d)
    ev=ROOT/'.codex/evaluation_events';registered=read(ev/f'{TASK}-task.json')['body']['checks']
    why=[reasons['C03'][1],reasons['C01'][1],reasons['C02'][1],reasons['C02'][1],reasons['C05'][1]+' '+reasons['C08'][1]]
    for kind,body in [('work_end',{'task':TASK,'session':os.environ.get('CODEX_THREAD_ID')}),('review',{'task':TASK,'mode':'automatic-self-review-v2','evaluation_version':'1',
      'checks':dict(zip(registered,['pass','pass','pass','pass','unknown'])),'rationale':dict(zip(registered,why)),'evidence':evidence})]:
        save(ev/f'{TASK}-{kind}.json',{'id':f'{TASK}-{kind}','kind':kind,'at':at,'body':body})
    publications=[]
    for kind in ['task','work_start','work_end','review']:
        c=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'scripts/publish_evaluation.py'),'--input',str(ev/f'{TASK}-{kind}.json')],capture_output=True,text=True,encoding='utf8',errors='replace',timeout=40)
        publications.append({'kind':kind,'exit':c.returncode,'stdout':c.stdout[-1600:],'stderr':c.stderr[-1600:]})
        if c.returncode:break
    save(OUT/'station_publication.json',{'observed_at':now(),'attempts':publications,'remaining_events':'local_pending_if_not_submitted'})
    c=subprocess.run([sys.executable,'-X','utf8',str(bridge/'tools/codex_control_center/publish_paper_status.py'),'--workspace','paper11','--sync'],capture_output=True,text=True,encoding='utf8',errors='replace',timeout=30)
    save(OUT/'publication_status.json',{'observed_at':now(),'exit':c.returncode,'stdout':c.stdout[-1500:],'stderr':c.stderr[-1500:],'local_modules_compiled_and_checked':True,'remote_published':c.returncode==0})
    save(OUT/'delivery_status.json',{'observed_at':now(),'component_status':record['status'],'report':ref(OUT/'天气_航段_能耗联算报告.pdf'),'local_experiments':live,
      'layout_check':'Inherited external staging registration escapes workspace; no active files reorganized or deleted',
      'cloud':'Access failed missing cloud_services; live provider counts unknown; no mutation','local_cost':0,'no_new_workers':True,'prior_pauses_preserved':True})
    with (q/'task_checks.md').open('a',encoding='utf8') as f:
        f.write('\n\n'+at+' | '+TASK+' | 12天气×5冻结模型60组已完成、全量独立核验；108拍摄时段事后诊断。固定时间转移滞后、模型响应分歧与越界保留；全任务可行性/实飞精度未知，无新成功阈值、未跨求解器比较节点。证据：'+ref(OUT/'delivery_validation.json')['path']+'\n')
    print({'recorded':True,'live_experiments':live,'station_exit':publications[0]['exit'],'publisher_exit':c.returncode})

if __name__=='__main__':main()
