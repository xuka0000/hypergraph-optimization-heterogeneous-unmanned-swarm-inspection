"""Record observed results and remaining gaps; do not authorize or launch experiments."""
import datetime,hashlib,json,os,pathlib,shutil,subprocess,sys,time
import psutil
from run_m100_energy_validation import ROOT,OUT,TASK,ref,save

BRIDGE=pathlib.Path('D:/博士研究生/具身智能大论文架构/cross_work_bridge')
Q=ROOT/'research_quality'

def refresh_refs(value):
    if isinstance(value,dict):
        if 'path' in value and 'sha256' in value:
            p=ROOT/value['path']
            if p.exists() and p.is_file():value['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
        for v in value.values():refresh_refs(v)
    elif isinstance(value,list):
        for v in value:refresh_refs(v)

def check_record(task):
    policy=BRIDGE/'.codex/research_stage_self_check_policy.json'
    cmd=[sys.executable,'-X','utf8','-B',str(BRIDGE/'scripts/check_research_stage_self_check_v1_2.py'),
      '--root',str(ROOT),'--record',str(Q/f'{task}_self_check.json'),'--stage','research','--mode','task','--policy',str(policy)]
    c=subprocess.run(cmd,capture_output=True,text=True,encoding='utf-8',timeout=30)
    (Q/f'{task}_self_check_receipt.json').write_text(c.stdout,encoding='utf-8')
    d=json.loads(c.stdout)
    print(task,{'exit':c.returncode,'record_valid':d.get('record_valid'),'stage_ready':d.get('stage_ready'),'errors':d.get('errors')})
    assert c.returncode==0 and d['record_valid']

def main():
    now=datetime.datetime.now(datetime.timezone.utc).isoformat()
    results=json.loads((OUT/'results.json').read_text(encoding='utf-8'))
    diag=json.loads((OUT/'interval_diagnostic.json').read_text(encoding='utf-8'))
    def put(s,x):
        p=Q/f'{TASK}_{s}.json';save(p,x);return ref(p)
    materials=[{**ref(p),'locator':'whole artifact'} for p in [OUT/'protocol.json',OUT/'source_manifest.json',
        OUT/'input_audit.json',OUT/'results.json',OUT/'interval_diagnostic.json',OUT/'delivery_validation.json',
        OUT/'首轮测试报告.txt',ROOT/'scripts/run_m100_energy_validation.py',ROOT/'scripts/summarize_m100_energy_validation.py']]
    reasons={
      'C01':('PASS','Two frozen physical models, exact source hashes and published constants; no learning changes. Advanced model weights/scaler gaps explicit.'),
      'C02':('PASS','Finite positive integrated energies and finite predictions; low-current ground tails diagnosed, not clipped or removed from primary statistics. No iterative solver used.'),
      'C03':('UNKNOWN','Eight IDs are reserved tests in CMU source; no local fitting. Complete original accepted-flight list and scaler missing, so exact pretrained-model reproduction remains unverified.'),
      'C04':('PASS','Eight paired flights, sixteen model-flight pairs; 9582 rows are not independent flights. Flight-macro APE and all per-flight Wh recomputed. Missing278/279 retained as missing.'),
      'C05':('UNKNOWN','Frozen physical code and rounded publication parameters compared on identical records. This does not substitute for the requested 2026 hybrid voltage model or verify its claimed gains.'),
      'C06':('PASS','Original predictions independently reintegrated, model identities and source units verified. Full-span wording corrected after detecting near-ground tails; primary numbers unchanged.'),
      'C07':('PASS','Batch wall time excludes acquisition/preparation/review, and is not used as a speed superiority claim; cost0 and no training. Wh error, not inference milliseconds, is evaluated.'),
      'C08':('UNKNOWN','Physical power-to-energy replay completed; full high-accuracy battery/voltage/SOC/thermal/ageing/return-feasibility validation incomplete. No threshold, universal accuracy, or hypergraph benefit asserted.')}
    manual={k:{'risk_id':k,'task_id':TASK,'stage':'research','verdict':st,'scope':'Initial public-platform component validation',
       'reviewer':'Current original Codex; not independent or user acceptance','reason':why,'reviewed_materials':materials} for k,(st,why) in reasons.items()}
    mr=put('manual',manual)
    er=put('report_expected',{'flights':8,'pairs':16,'verified_sources':87,'passed':True})
    checks=[{'id':'pairs','kind':'derived_consistency','scope':'All physical-model replay pairs',
       'data':{**ref(OUT/'results.json'),'pointer':'/rows'},'operation':'count','reported':{**er,'pointer':'/pairs'}},
       {'id':'verification','kind':'identity','scope':'Independent full-result integration and immutable sources',
       'expected':er,'actual':ref(OUT/'delivery_validation.json'),
       'fields':[{'expected':'passed','actual':'passed_for_completed_scope'},{'expected':'flights','actual':'flights'},
                 {'expected':'verified_sources','actual':'source_files_verified'}]}]
    content=put('report_content',{'schema_version':1,'task_id':TASK,'stage':'research','checks':checks,
       'coverage':{k:{'checks':['pairs','verification'],'manual_review':{**mr,'locator':'/'+k,'pointer':'/'+k}} for k in reasons}})
    mappings={'R01':'C08','R02':'C01','R03':'C02','R04':'C05','R05':'C06','R06':'C04','R07':'C05','R08':'C07','R09':'C06','R10':'C08'}
    items=[{'id':k,'status':reasons[v][0],'reason':reasons[v][1],'evidence':[{**mr,'locator':'/'+v,'pointer':'/'+v}]} for k,v in mappings.items()]
    risk=[{'id':k,'status':v[0],'finding_status':'no_defect' if v[0]=='PASS' else 'review_lead','reason':v[1],
       'impact':'Battery model reproducibility and supported accuracy claim','evidence':[{**mr,'locator':'/'+k,'pointer':'/'+k}]} for k,v in reasons.items()]
    put('self_check',{'policy_id':'RESEARCH-STAGE-SELF-CHECK-01','policy_version':'1.2','stage':'research','task_id':TASK,
       'checked_at':now,'subject':{'scope':'Physical component results delivered; advanced battery validation incomplete','materials':materials},
       'items':items,'task_completion':'partial','stage_outcome':'NOT_READY','risk_triggers':list(reasons),'risk_checks':risk,
       'content_manifest':content,'limitations':json.loads((OUT/'delivery_validation.json').read_text(encoding='utf-8'))['limitations'],
       'next_action':'Obtain exact CMU normalization/accepted-flight sheet and verified 2026 implementation/checkpoints before learned-model accuracy claims. No training launched.'})
    check_record(TASK)
    # Refresh previously completed SCIP report, leaving failed author runs unresolved.
    old='hypergraph_paper_pipeline_original'
    base=ROOT/'总结/03_实验数据/超图与SCIP组合任务对比/论文完整流程原始场景对比'
    old_results=json.loads((base/'results.json').read_text(encoding='utf-8'))
    live=[]
    for pid,created in [(13184,1794059455.1895182),(40284,1794046886.26545)]:
        try:
            p=psutil.Process(pid);matched=abs(p.create_time()-created)<.1
            live.append({'pid':pid,'identity_matched':matched,'status':p.status() if matched else 'pid_reused',
                         'CPU_seconds':sum(p.cpu_times()[:2]) if matched else None})
        except psutil.NoSuchProcess:live.append({'pid':pid,'identity_matched':False,'status':'absent'})
    oldobs=json.loads((base/'execution_observation.json').read_text(encoding='utf-8'))
    oldobs['previous_observation']={k:v for k,v in oldobs.items() if k!='previous_observation'}
    oldobs.update(observed_at=now,status='completed_gaplimit_independently_verified',pid=13184,
        current_process_observations=live,completion_evidence=ref(base/'raw/paper2026_dac2012_superblue19_SCIP_cold_gap05.json'))
    save(base/'execution_observation.json',oldobs)
    e=json.loads((Q/f'{old}_report_expected.json').read_text(encoding='utf-8'));e['verified']=72;save(Q/f'{old}_report_expected.json',e)
    md=json.loads((Q/f'{old}_manual.json').read_text(encoding='utf-8'))
    md['C07']['reason']='Full pipeline timings retained; seven SCIP outputs verified. Largest cold SCIP reachedgap0.566215% in2099.025s total, not optimal; initial RAM failure retained; paidcost0.'
    md['C08']['reason']='All77 planned outcomes accounted for:65 valid author runs,5 failures,7 valid SCIP outputs. Largest SCIP objective11642395 versus author10-run mean11405360.9 (-2.03596%). Five baseline failures remain unresolved; no next-scale readiness.'
    for v in md.values():v['scope']='All planned outcomes observed; original author failures remain unresolved'
    refresh_refs(md);save(Q/f'{old}_manual.json',md)
    rc=json.loads((Q/f'{old}_report_content.json').read_text(encoding='utf-8'));refresh_refs(rc);save(Q/f'{old}_report_content.json',rc)
    ck=json.loads((Q/f'{old}_self_check.json').read_text(encoding='utf-8'))
    ck['checked_at']=now;ck['subject']['scope']='All77 fixed outcomes recorded;5 author failures remain open'
    for arr in ['items','risk_checks']:
        for it in ck[arr]:
            ev=it.get('evidence',[])
            k=ev[0].get('pointer','').lstrip('/') if ev else ''
            if k in ['C07','C08']:it['reason']=md[k]['reason']
    ck['limitations']=[x for x in ck['limitations'] if 'not finished' not in x]
    refresh_refs(ck);save(Q/f'{old}_self_check.json',ck);check_record(old)
    # Record the new task without replacing existing manuscript statements.
    pg=ROOT/'.codex/paper_progress.json';d=json.loads(pg.read_text(encoding='utf-8-sig'))
    record={'id':TASK,'title':'M100公开机型电池能耗组件验证','purpose':'核验公开模型与真实电压电流积分能耗的偏差',
       'method':'8个原作者保留航次；2冻结物理模型；同记录时段；实际时间戳VI积分；高度筛选仅事后诊断',
       'result':'完整记录区间平均APE11.5407%/6.1683%；近似高度筛选诊断8.8987%/3.1382%。',
       'remaining':'TCN原归一化、2026模型代码/权重及动态电池/返航验证仍缺失。',
       'boundary':'8航次而非9582独立样本；不等于整航次高精度电池验证；无达标阈值。',
       'status':'partial','observed_at':time.time(),'evidence':[str((OUT/n).relative_to(ROOT)).replace('\\','/') for n in ['results.json','interval_diagnostic.json','delivery_validation.json','首轮测试报告.txt']],
       'progress':{'unit':'completed physical model-flight pairs','total':16,'completed':16,'running':0,'failed':0,'blocked_advanced_components':2}}
    ex=d['modules']['results'].setdefault('experiments',[]);ex[:]=[x for x in ex if x.get('id')!=TASK];ex.append(record)
    for mod in ['modeling','setup','results']:
        d['modules'][mod]['content'][TASK]=record
        for path in record['evidence']:
            if path not in d['modules'][mod]['evidence']:d['modules'][mod]['evidence'].append(path)
    d['modules']['algorithm']['content'][TASK]={'status':'unchanged','reason':'No inspection solver or hypergraph algorithm changed.'}
    save(pg,d)
    selection=ROOT/'总结/07_证据与复现索引/电池能耗建模选型.json'
    d=json.loads(selection.read_text(encoding='utf-8'));d['local_test_followup']=record
    d['local_test_followup']['advanced_model_status']='not_reproduced';save(selection,d)
    storage=ROOT/'.codex/storage_manifest.json';d=json.loads(storage.read_text(encoding='utf-8-sig'))
    d[TASK].update(observed_at=now,status='physical_batch_complete_advanced_dependencies_pending',live_peak_reservation_bytes=0,C_temporary_peak_bytes=0,
       retained_logical_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),
       free_bytes={x:shutil.disk_usage(x+':/').free for x in ['C','D']},reclaimed_bytes=0)
    d[old].update(observed_at=now,status='completed_with_author_failures',active_peak_extra_writes_bytes=0)
    save(storage,d)
    purpose=ROOT/'.codex/experiment_goal_review.json';d=json.loads(purpose.read_text(encoding='utf-8-sig'))
    d[TASK]={'observed_at':now,'claim':'Local physical-model energy accuracy only; advanced battery model pending',
       'new_evidence':record['evidence'],'cost':0,'deadline':'local batch finished','next_action':'Resolve pretrained normalization/code dependencies',
       'stop_condition':'No repeated acquisition without new source evidence; no training or cloud action',
       'cloud_inspection':'Previously stopped on missing cloud_services; no new evidence to revalidate',
       'cloud_counts':{'in_use':None,'idle':None,'retained':None,'unknown':None,'pending_release':None},
       'local_experiments':live}
    save(purpose,d)
    checks=Q/'task_checks.md'
    with checks.open('a',encoding='utf-8') as f:
        f.write('\n\n'+now+' | '+TASK+' | 两个物理模型16组回放及独立复算完成；动态电池主任务partial、stage NOT_READY。主表11.5407%/6.1683%，高度筛选事后诊断8.8987%/3.1382%；无预设达标线。来源/失败/缺失和原始记录保留。证据：'+str((OUT/'delivery_validation.json').relative_to(ROOT))+'\n')
    ev=ROOT/'.codex/evaluation_events'
    status={'primary_source_identity':'pass','flight_split_audit':'unknown','frozen_model_execution':'unknown',
            'independent_energy_metrics':'pass','limitations_and_failures':'pass'}
    rationale={'primary_source_identity':'87 primary-source files git-blob and SHA256 verified.',
       'flight_split_audit':'Eight explicit reserved testIDs, but original complete accepted-flight list and scaler unavailable.',
       'frozen_model_execution':'Two physical models executed; selected TCN and2026 dynamic voltage model not reproduced.',
       'independent_energy_metrics':'All16 paired energies independently recomputed from saved prediction files, with actual timestamps.',
       'limitations_and_failures':'Two missingflights,403/timeouts,ground-tail issue and advanced-model blockers retained; no fullbattery accuracy claim.'}
    save(ev/(TASK+'-review.json'),{'id':TASK+'-review','kind':'review','at':now,'body':{'task':TASK,
       'mode':'automatic-self-review-v2','evaluation_version':'1','checks':status,'rationale':rationale,'evidence':record['evidence']}})
    save(ev/(TASK+'-end.json'),{'id':TASK+'-end','kind':'work_end','at':now,'body':{'task':TASK,'session':os.environ.get('CODEX_THREAD_ID')}})
    print(json.dumps({'recorded':True,'live_processes':live,'report':str(OUT/'首轮测试报告.txt')},ensure_ascii=False))

if __name__=='__main__':main()
