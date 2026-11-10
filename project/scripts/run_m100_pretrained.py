"""Original CMU checkpoints and preprocessing, in an isolated Python3.8/TF2.2 runtime.

No training or parameter selection. Gate orchestration uses the main interpreter.
The original author Python files remain unchanged.
"""
import argparse, datetime, hashlib, json, os, pathlib, subprocess, sys, time

ROOT=pathlib.Path(__file__).resolve().parents[1]
BASE=ROOT/'总结/03_实验数据/电池能耗公开机型验证'
SRC=BASE/'sources/cmu2021'
OUT=BASE/'cmu_pretrained'
TASK='battery_cmu_pretrained_validation'
BRIDGE=pathlib.Path('D:/博士研究生/具身智能大论文架构/cross_work_bridge/scripts')
PY38=ROOT/'.codex/venvs/m100_cmu2021/python.exe'
MODELS=['b-TCN','s-TCN','b-LSTM','s-LSTM','Liu']

def save(p,x):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')

def ref(p):
    return {'path':p.relative_to(ROOT).as_posix(),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}

def runtime():
    os.environ['CUDA_VISIBLE_DEVICES']='-1'
    os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
    os.environ['OMP_NUM_THREADS']='2'
    sys.path.insert(0,str(SRC))
    import tensorflow as tf
    tf.config.threading.set_intra_op_parallelism_threads(2)
    tf.config.threading.set_inter_op_parallelism_threads(1)
    import data_utils as du, deep_energy_model as dm, evaluation as ev, liu_model
    return tf,du,dm,ev,liu_model

def prepare():
    tf,du,dm,ev,liu=runtime()
    import numpy as np, pandas as pd
    all_data=du.load_dataset(str(SRC/'data'))
    data,lo,hi,test,train,val=du.process_data(all_data,**dm.load_inputs())
    files=[ref(SRC/'data/Flight Sheet.xlsx')]+[ref(SRC/'data'/str(f)/'processed.csv') for f in sorted(all_data)]
    counts={'all_flights':len(all_data),'train':len(train),'val':len(val),
            'test_entries':len(test),'test_unique':len(set(test)),'random':len(du.get_random_flights())}
    assert not (set(train)&set(val) or set(train)&set(test) or set(val)&set(test))
    assert set(du.get_random_flights()).issubset(test)
    # Exact original min-max computation, with an independently selected table.
    cols=['airspeed_x','airspeed_y','vertspd','aoa','airspeed','power']
    dev=pd.concat([all_data[int(f)] for f in list(train)+list(val)])
    assert np.allclose(lo,dev[cols].min().values) and np.allclose(hi,dev[cols].max().values)
    alignment=[]
    for f in sorted(set(test)):
        a=all_data[f]; x,y=data[f]
        assert len(x)==len(a)-19
        assert np.allclose(y*(hi[-1]-lo[-1])+lo[-1],a.power.values[19:])
        assert np.all(np.diff(a.time)>0)
        alignment.append({'flight':int(f),'samples':len(a),'prediction_samples':len(x),
                          'from_s':float(a.time.values[19]),'to_s':float(a.time.values[-1])})
    # Verify all trainable model variables restore; optimizer slots are immaterial at inference.
    restored=[]
    for name in MODELS[:-1]:
        tf.keras.backend.clear_session()
        cfg=ev.get_pre_def_configs()[name]; inputs=dm.load_inputs(**cfg)
        model=dm.create_advanced_net((20,5),**inputs)
        status=model.load_weights(str(SRC/'models'/name/'cp.ckpt'))
        status.assert_existing_objects_matched();status.expect_partial()
        restored.append({'model':name,'params':model.count_params(),'trainable_tensors':len(model.trainable_weights),
                         'checkpoint_all_existing_objects_matched':True,'config':cfg})
    reg={'passed':True,'all_original_checkpoints_restored':True,'window_alignment_verified':True,
         'normalization_independently_verified':True,'train_validation_test_disjoint':True,
         'constant_integral_Wh':float(np.trapz([600,600],[0,3600])/3600)}
    assert reg['constant_integral_Wh']==600
    save(OUT/'regression.json',reg)
    audit={'counts':counts,'splits':{k:[int(x) for x in v] for k,v in [('test',test),('train',train),('val',val),('random',du.get_random_flights())]},
       'normalized_columns':cols,'data_min':lo.tolist(),'data_max':hi.tolist(),
       'normalization_scope':'Original author train+validation; original test excluded. Never fitted to test.',
       'cached_preprocessing':'Exact author processed.csv from author data_min.zip; no reconstructed density or crop.',
       'alignment':alignment,'files':files,'checkpoints':restored,
       'source_manifest':ref(BASE/'recovery_source_manifest.json'),'tensorflow':tf.__version__,'numpy':np.__version__,'pandas':pd.__version__}
    save(OUT/'input_audit.json',audit)
    protocol={'task_id':TASK,'created_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
      'purpose':'Validate all four public trained energy models against original Liu model on original held-out flights.',
      'models':MODELS,'training':False,'tuning':False,'threshold':None,
      'author_commit':'d613ba37bde8dcdaa5a9e395de4cc4029c24a39d','split_source':'Unchanged get_flights_and_ranges(seed42)',
      'interval':'Author preprocessed >7m relative-altitude records; discard first19 records for lookback20; same points for all methods.',
      'metrics':['whole_available_crop_energy_APE_pct_trapezoid','power_MAPE_pct','power_RMSE_W','author_regime_energy_RE_pct_simpson_even_avg'],
      'aggregation':'Flight macro mean; original test entry multiplicity retained in paper-replay column; unique-flight result also reported. Random10 reported separately.',
      'runtime':'Python3.8.10 TensorFlow2.2.0 keras-tcn3.1.1 original models. ruptures1.0.6 original change-point implementation.',
      'no_full_flight_or_voltage_claim':True,'test_includes_previous_eight':'Additional fixed-model replay; no tuning based on prior results.',
      'resources':{'local_CPU_threads':2,'cloud_cost':0,'RAM_limit_GiB':4,'wall_guard_seconds':1800},
      'sources':audit['files'],'original_protocol_retained':'../protocol.json'}
    save(OUT/'protocol.json',protocol)
    print(json.dumps({'prepared':True,'counts':counts,'normalization_min':lo.tolist(),'normalization_max':hi.tolist(),'restored':restored},ensure_ascii=False),flush=True)

def batch():
    tf,du,dm,ev,liu=runtime()
    import numpy as np,pandas as pd
    assert os.environ.get('PAPER11_CMU_ADMITTED')==ref(OUT/'protocol.json')['sha256']
    assert not (OUT/'results.json').exists(),'Existing results retained; refuse duplicate execution'
    start=time.perf_counter(); all_data=du.load_dataset(str(SRC/'data'))
    audit=json.loads((OUT/'input_audit.json').read_text(encoding='utf8'))
    for f in audit['files']:assert ref(ROOT/f['path'])['sha256']==f['sha256']
    data,lo,hi,test,train,val=du.process_data(all_data,**dm.load_inputs())
    assert list(test)==audit['splits']['test'] and np.array_equal(lo,audit['data_min']) and np.array_equal(hi,audit['data_max'])
    unique=sorted(set(int(f) for f in test));cp=du.find_regimes({f:all_data[f] for f in unique})
    rows=[];predframes={}
    for f in unique:
        a=all_data[f].iloc[19:];predframes[f]=pd.DataFrame({'time_s':a.time.values,'measured_W':a.power.values})
    for name in MODELS:
        tf.keras.backend.clear_session(); load_start=time.perf_counter()
        if name!='Liu':
            inputs=dm.load_inputs(**ev.get_pre_def_configs()[name]);model=dm.create_advanced_net((20,5),**inputs)
            status=model.load_weights(str(SRC/'models'/name/'cp.ckpt'));status.assert_existing_objects_matched();status.expect_partial()
            tensors=du.create_tensors(data,unique,**inputs)
        load_sec=time.perf_counter()-load_start
        for f in unique:
            a=all_data[f]; y=a.power.values[19:];ts=a.time.values[19:]
            ts0=time.perf_counter()
            if name=='Liu':pr=liu.power(a[['vertspd','airspeed','aoa','payload','density']].T,*liu.optimum_values())[19:]
            else:
                model.reset_states();pr=model.predict(tensors[f],verbose=0).reshape(-1)*(hi[-1]-lo[-1])+lo[-1]
            sec=time.perf_counter()-ts0
            assert len(pr)==len(y) and np.isfinite(pr).all()
            truth=float(np.trapz(y,ts)/3600);pred=float(np.trapz(pr,ts)/3600)
            gt_reg=ev.calc_energy(y,ts,cp[f]);pr_reg=ev.calc_energy(pr,ts,cp[f])
            row={'flight':f,'model':name,'samples':len(y),'measured_Wh':truth,'predicted_Wh':pred,
               'delta_Wh':pred-truth,'energy_APE_pct':abs(pred/truth-1)*100,
               'power_MAPE_pct':float(np.mean(abs((pr-y)/y))*100),'power_RMSE_W':float(np.sqrt(np.mean((pr-y)**2))),
               'author_regime_RE_pct':float(np.mean(abs((pr_reg-gt_reg)/gt_reg))*100),
               'prediction_seconds':sec,'model_load_seconds':load_sec,'regime_change_points':cp[f]}
            rows.append(row);predframes[f][name+'_W']=pr
            assert time.perf_counter()-start<1800,'Finite local batch wall guard'
        print(json.dumps({'model_completed':name,'flights':len(unique),'elapsed_s':time.perf_counter()-start},ensure_ascii=False),flush=True)
    summaries=[]
    for group,ids in [('test_author_entries',list(test)),('test_unique_flights',unique),('random10',list(du.get_random_flights()))]:
        for name in MODELS:
            by={r['flight']:r for r in rows if r['model']==name};rr=[by[int(f)] for f in ids]
            summaries.append({'group':group,'model':name,'entries':len(ids),'unique_flights':len(set(ids)),
              **{k:float(np.mean([r[k] for r in rr])) for k in ['energy_APE_pct','power_MAPE_pct','power_RMSE_W','author_regime_RE_pct']},
              'max_energy_APE_pct':max(r['energy_APE_pct'] for r in rr),
              'prediction_seconds_unique':sum(by[int(f)]['prediction_seconds'] for f in set(ids))})
    (OUT/'predictions').mkdir(exist_ok=True)
    for f,p in predframes.items():p.to_csv(OUT/'predictions'/('flight_%d.csv'%f),index=False)
    pd.DataFrame(rows).to_csv(OUT/'per_flight_results.csv',index=False,encoding='utf-8-sig')
    save(OUT/'results.json',{'observed_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'task_id':TASK,
        'protocol':ref(OUT/'protocol.json'),'input_audit':ref(OUT/'input_audit.json'),'status':'completed',
        'training':False,'rows':rows,'summary':summaries,'batch_seconds':time.perf_counter()-start,
        'scope':'Fixed author checkpoint replay on original cropped held-out data; no dynamic-voltage accuracy or new-plane transfer claim.'})
    print(json.dumps({'summary':summaries,'seconds':time.perf_counter()-start},ensure_ascii=False,indent=2),flush=True)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['prepare','batch','runtime_prepare','runtime_batch']);args=ap.parse_args()
    if args.mode.startswith('runtime_'):
        prepare() if args.mode=='runtime_prepare' else batch();return
    if args.mode=='batch':
        sys.path.insert(0,str(BRIDGE));from check_research_stage import require_transition
        save(OUT/'transition_receipt.json',require_transition(ROOT,'research_quality/'+TASK+'_review_nodes.json',TASK))
        p=subprocess.run([sys.executable,'-X','utf8','-B',str(BRIDGE/'check_experiment_preflight.py'),'--root',str(ROOT),
           '--record',str(ROOT/'research_quality'/(TASK+'_preflight.json')),'--next-step','public_baselines'],capture_output=True,text=True,encoding='utf8',timeout=30)
        (OUT/'preflight_receipt.json').write_text(p.stdout,encoding='utf8');assert p.returncode==0 and json.loads(p.stdout)['preflight_allowed'],p.stdout+p.stderr
        os.environ['PAPER11_CMU_ADMITTED']=ref(OUT/'protocol.json')['sha256']
    r=subprocess.run([str(PY38),'-X','utf8','-B',str(pathlib.Path(__file__).resolve()),'runtime_'+args.mode],timeout=1900)
    assert r.returncode==0,r.returncode

if __name__=='__main__':main()
