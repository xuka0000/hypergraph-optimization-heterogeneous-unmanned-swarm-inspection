"""Close regional-weather evidence bookkeeping without launching research jobs."""
import datetime,json,os,pathlib,shutil,subprocess,sys,time
import psutil
from prepare_weather_factors import ROOT,OUT,TASK,save,ref,now
from record_m100_validation_delivery import check_record

def read(p):return json.loads(p.read_text('utf8'))
def main():
    at=now();validation=read(OUT/'delivery_validation.json');result=read(OUT/'results.json');pdf=read(OUT/'module_pdf_sync.json')
    assert validation['passed_for_current_scope'] and pdf['status']=='compiled_and_new_pages_visually_verified'
    q=ROOT/'research_quality';bridge=pathlib.Path('D:/博士研究生/具身智能大论文架构/cross_work_bridge')
    files=[OUT/n for n in ['protocol.json','source_manifest.json','source_audit.json','model.json','results.json','validation.json','delivery_validation.json','table_reference_mapping.json','天气因素建模报告.txt','天气因素建模报告.pdf','module_pdf_sync.json']]
    files += [ROOT/'Agent/Publication/weather_factors.py',ROOT/'scripts/verify_weather_factors.py']
    materials=[{**ref(p),'locator':'whole artifact'} for p in files]
    reasons={
      'C01':('PASS','ERA5-only raw responses, official aircraft manual, corrected Gao2021 framework, unchanged CMU density and original GeoWind5kg energy function source-bound. Adaptations explicitly labelled; no public solver improvement or algorithm comparison.'),
      'C02':('PASS','17 deterministic known cases and independent raw-output recomputation of all5040 rows,36 summaries and6 mappings passed. Original coefficient-vs-ideal difference explained analytically. Unknowns not replaced by zero.'),
      'C03':('PASS','January/July2025 first seven days fixed before download as development examples.1680 unique grid-hours, not5040 independent samples;1075 tower entries represent901 unique source objects. No formal held-out evaluation or annual-statistics claim.'),
      'C04':('PASS','All6 scenes and3 heights retained, including identical shared-grid outcomes,757 density-envelope outlier grid-hours and16 invalid lapse rows. No success threshold, significance or hypergraph-benefit claim.'),
      'C05':('UNKNOWN','Regional weather screen implemented; tower-scale gusts, vertical wind, forecast-error probabilities, aircraft attitude response and battery-temperature curves remain uncalibrated. Existing3.51percent energy error cannot be transferred.'),
      'C06':('PASS','All16 source-asset hashes and6 original scene hashes unchanged; every saved row/unit/window and geographic mapping checked independently. 425byte DJI page classified redirect-only, official manual used as evidence. PDF tables generated from saved results and visually reviewed.'),
      'C07':('PASS','Local deterministic model/data construction, zero training/paid-cloud/CFD runs. Acquisition finite600s guard and256MiB peak reservation; drive floors preserved. Existing SCIP process not restarted or duplicated.'),
      'C08':('UNKNOWN','Weather component supplies regional inputs and availability screen. No current solver integration, calibrated weather energy prediction, high-order coupling effect or full high-accuracy local weather validation.')}
    def put(s,d):p=q/f'{TASK}_{s}.json';save(p,d);return ref(p)
    manual={k:{'risk_id':k,'task_id':TASK,'stage':'research','verdict':v[0],'scope':'Regional-weather model/data/evaluator node; high-accuracy mission-weather task incomplete',
      'reviewer':'Current original Codex; not user acceptance or independent peer review','reason':v[1],'reviewed_materials':materials} for k,v in reasons.items()}
    mr=put('manual',manual);expected=put('report_expected',{'rows':5040,'grids':1680,'scenes':6,'passed':True})
    checks=[{'id':'rows','kind':'derived_consistency','scope':'All saved regional weather rows','data':{**ref(OUT/'weather_records.json'),'pointer':'/records'},'operation':'count','reported':{**expected,'pointer':'/rows'}},
      {'id':'verification','kind':'identity','scope':'Independent full raw-to-output verification','expected':expected,'actual':ref(OUT/'delivery_validation.json'),'fields':[{'expected':'passed','actual':'passed_for_current_scope'},{'expected':'grids','actual':'unique_grid_hours'},{'expected':'scenes','actual':'scene_count'}]}]
    content=put('report_content',{'schema_version':1,'task_id':TASK,'stage':'research','checks':checks,'coverage':{k:{'checks':['rows','verification'],'manual_review':{**mr,'locator':'/'+k,'pointer':'/'+k}} for k in reasons}})
    mappings={'R01':'C08','R02':'C01','R03':'C02','R04':'C05','R05':'C06','R06':'C04','R07':'C05','R08':'C07','R09':'C06','R10':'C08'}
    items=[{'id':k,'status':reasons[v][0],'reason':reasons[v][1],'evidence':[{**mr,'locator':'/'+v,'pointer':'/'+v}]} for k,v in mappings.items()]
    risks=[{'id':k,'status':v[0],'finding_status':'no_defect' if v[0]=='PASS' else 'review_lead','reason':v[1],'impact':'Supported weather accuracy and downstream energy/optimization claims','evidence':[{**mr,'locator':'/'+k,'pointer':'/'+k}]} for k,v in reasons.items()]
    put('self_check',{'policy_id':'RESEARCH-STAGE-SELF-CHECK-01','policy_version':'1.2','stage':'research','task_id':TASK,'checked_at':at,
      'subject':{'scope':'Regional-weather construction and validation completed; high-accuracy weather/inspection inference remains incomplete','materials':materials},
      'items':items,'task_completion':'partial','stage_outcome':'NOT_READY','risk_triggers':list(reasons),'risk_checks':risks,'content_manifest':content,
      'limitations':validation['limitations'],'next_action':'Review the concrete weather model and source-bound missing local-wind/energy calibration before crossing to optimizer comparison, training or CFD. Existing unrelated approvals and pauses preserved.'})
    check_record(TASK)
    evidence=[ref(p)['path'] for p in files]
    record={'id':TASK,'title':'真实巡检场景天气因素建模','purpose':'建立天气对空速、密度、可用时段与电池能耗的可复用接口',
      'method':'固定冬夏各7天；ERA5小时0.25度；6场景5网格3高度；M100官方边界；17已知算例及5040行独立复核',
      'result':'60m冬/夏基础通过小时：Úbeda144/159、德国两个场景44/125、Villevaudé80/127；每周168小时。密度超原训练验证范围757/1680；16行高度密度为空。',
      'remaining':'塔级风场、真实高度阵风/垂直风、预报误差、天气下姿态和电池温度响应；巡检求解器尚未接入。',
      'boundary':'两周开发示例不是全年可飞率；筛选不是飞行许可；原3.51%能耗误差不能移用于新天气；无超图优势结论。',
      'status':'regional_component_completed_full_weather_accuracy_unvalidated','observed_at':time.time(),'execution_observed_at':result['observed_at'],
      'evidence':evidence,'progress':{'unit':'unique regional grid-hours','total':1680,'completed':1680,'running':0,'failed':0,'height_rows':5040,'unknown_height_density_rows':16},'cost':0,'training':False}
    p=ROOT/'.codex/paper_progress.json';d=read(p)
    ex=d['modules']['results'].setdefault('experiments',[]);ex[:]=[x for x in ex if x.get('id')!=TASK];ex.append(record)
    for n in ['modeling','setup','results']:
        d['modules'][n].setdefault('content',{})[TASK]=record
        ev=d['modules'][n].setdefault('evidence',[])
        for e in evidence:
            if e not in ev:ev.append(e)
    d['modules']['algorithm'].setdefault('content',{})[TASK]={'status':'inspection_solver_unchanged','weather_interface':'Shared weather sample identity, availability and energy equations; no new optimization or claim of hypergraph benefit.'}
    for rec in pdf['records'][1:]:
        if rec['module']=='report':continue
        doc=d['modules'][rec['module']]['document'];doc.setdefault('source_files',[])
        shared=rec['shared_content']['path']
        if shared not in doc['source_files']:doc['source_files'].append(shared)
        doc['latest_local_compile']={'observed_at':pdf['observed_at'],'pdf_sha256':rec['pdf']['sha256'],'pages':rec['pages'],'new_appendix_visual_review':'passed','shared_builder':'missing; installed native XeLaTeX fallback'}
    save(p,d)
    p=ROOT/'project_layout.json';d=read(p);d['geographic_inspection_scene'][TASK]={'root':ref(OUT/'results.json')['path'].rsplit('/',1)[0],'model':'Agent/Publication/weather_factors.py','report':ref(OUT/'天气因素建模报告.pdf')['path'],'role':'Bound regional-weather overlay; original scene data unchanged'};save(p,d)
    live=[]
    try:
        proc=psutil.Process(40284);match=abs(proc.create_time()-1794046886.26545)<.1
        live.append({'pid':40284,'identity_matched':match,'status':proc.status() if match else 'PID_reused','CPU_seconds':sum(proc.cpu_times()[:2]) if match else None,'action':'preserved_no_restart'})
    except psutil.NoSuchProcess:live.append({'pid':40284,'status':'absent_at_readback','action':'no_restart'})
    p=ROOT/'.codex/experiment_goal_review.json';d=read(p);d[TASK]={'observed_at':at,'owner_thread_id':os.environ.get('CODEX_THREAD_ID'),'claim':record['purpose'],'new_evidence':evidence,
      'cost':0,'deadline':'Bounded regional-data construction complete; no unattended weather job','next_action':record['remaining'],'stop_condition':'No automatic paid/training/CFD or optimizer transition; missing fields remain unknown',
      'cloud_inspection':'First failure missing cloud_services; provider list not reached; no mutation or blind retry','cloud_counts':{k:None for k in ['in_use','idle','retained','unknown','pending_release']},'local_experiments':live};save(p,d)
    p=ROOT/'.codex/storage_manifest.json';d=read(p);d[TASK].update(observed_at=at,status='regional_component_completed_assets_retained',peak_write_reservation_bytes=0,C_peak_temporary_bytes=0,
      retained_logical_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),free_bytes={s:shutil.disk_usage(s+':/').free for s in ['C','D']},reclaimed_bytes=0);save(p,d)
    ev=ROOT/'.codex/evaluation_events';registered=read(ev/f'{TASK}-task.json')['body']['checks']
    why=[reasons['C01'][1],reasons['C02'][1],reasons['C02'][1],reasons['C03'][1],reasons['C05'][1]]
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
    save(OUT/'delivery_status.json',{'observed_at':now(),'component_status':record['status'],'report':ref(OUT/'天气因素建模报告.pdf'),'local_experiments':live,
      'layout_check':{'exit':1,'error':'Legacy registered path escapes workspace: C:/Users/Out/AppData/Local/Paper11StorageStaging/legacy_baseline_results','files_moved':0,'files_deleted':0,'disposition':'Existing external staging registration retained; no active files reorganized'},
      'cloud':'access check failed missing cloud_services; counts unknown; no mutation','local_cost':0,'no_new_workers':True,'prior_pauses_preserved':True})
    with (q/'task_checks.md').open('a',encoding='utf8') as f:f.write('\n\n'+at+' | '+TASK+' | 区域天气组件完成；1680网格小时、5040高度行、6场景映射；17已知算例与全行独立复核通过。塔级风场和新天气能耗精度仍未验证；未跨优化比较节点。证据：'+ref(OUT/'delivery_validation.json')['path']+'\n')
    print(json.dumps({'recorded':True,'local_experiments':live,'station':publications[0]['exit'],'publisher':c.returncode},ensure_ascii=False))

if __name__=='__main__':main()
