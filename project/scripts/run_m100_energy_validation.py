"""Frozen public-model replay on eight available author-held-out M100 flights.

No fitting, retuning, flight selection by error, or synthetic flight validation.
TCN/2026 GRU are blocked separately; this batch tests two physical power models.
"""
import argparse, datetime, hashlib, importlib.util, json, pathlib, sys, time
import numpy as np
import pandas as pd
from scipy.integrate import simpson
from scipy.spatial.transform import Rotation

ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'总结/03_实验数据/电池能耗公开机型验证'
BRIDGE=pathlib.Path('D:/博士研究生/具身智能大论文架构/cross_work_bridge/scripts')
TASK='battery_energy_public_validation'
FLIGHTS=[268,269,270,271,272,273,276,277]
MODELS=['CMU_Liu_frozen','Rodrigues2022_Table1']
COEFF={'takeoff':[80.4,13.8],'cruise':[68.9,16.8],'landing':[71.5,-24.3]}

def save(p,x):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')

def ref(p):
    p=pathlib.Path(p)
    return {'path':p.relative_to(ROOT).as_posix(),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}

def read_data():
    return pd.read_csv(OUT/'sources/rodrigues_data/MLModel/data/flights_validation.csv')

def trap_wh(y,t):
    y=np.asarray(y);t=np.asarray(t)
    assert len(t)>1 and np.all(np.diff(t)>0) and np.isfinite(y).all()
    return float(np.trapz(y,t)/3600.)

def simps_old(y,t):
    """Equivalent to historical scipy simps(..., even='avg') on 1D arrays."""
    y=np.asarray(y);t=np.asarray(t)
    if len(y)%2:return float(simpson(y,x=t))
    if len(y)==2:return float((y[0]+y[1])*(t[1]-t[0])/2)
    a=simpson(y[:-1],x=t[:-1])+(y[-2]+y[-1])*(t[-1]-t[-2])/2
    b=simpson(y[1:],x=t[1:])+(y[0]+y[1])*(t[1]-t[0])/2
    return float((a+b)/2)

def get_liu():
    path=OUT/'sources/cmu2021/liu_model.py'
    spec=importlib.util.spec_from_file_location('cmu_liu_frozen',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

def features(d):
    # Exact quaternion component selection and units used by CMU source.
    euler=Rotation.from_quat(d[['x.1','y.1','z.1','w']].to_numpy()).as_euler('xyz')
    return pd.DataFrame({'vertspd':d['z.2'].to_numpy(),
        'airspeed':d.wind_speed.to_numpy(),'aoa':euler[:,0],
        'payload':d.payload.to_numpy()/1000.,'density':d.density.to_numpy()})

def physical_predictions(d):
    liu=get_liu();f=features(d)
    a=time.perf_counter();p1=liu.power(f.T,*liu.optimum_values());t1=time.perf_counter()-a
    a=time.perf_counter();mr=(3.08+0.635+d.payload.to_numpy()/1000.)**1.5/np.sqrt(d.density.to_numpy())
    p2=np.array([COEFF[r][0]*x+COEFF[r][1] for r,x in zip(d.regime,mr)])
    t2=time.perf_counter()-a
    return [(MODELS[0],p1,t1),(MODELS[1],p2,t2)]

def prepare():
    data=read_data();ids=sorted(data.flight.astype(int).unique().tolist());assert ids==FLIGHTS
    required=['time','battery_voltage','battery_current','wind_speed','wind_angle',
              'x.1','y.1','z.1','w','z.2','payload','density','regime']
    assert not data[required].isna().any().any()
    assert set(data.regime)==set(COEFF)
    rows=[]
    for f,d in data.groupby('flight',sort=True):
        t=d.time.to_numpy();assert np.all(np.diff(t)>0)
        assert d.payload.nunique()==d.density.nunique()==1
        rows.append({'flight':int(f),'rows':len(d),'start_s':float(t[0]),'end_s':float(t[-1]),
            'duration_s':float(t[-1]-t[0]),'dt_median_s':float(np.median(np.diff(t))),
            'dt_max_s':float(np.diff(t).max()),'payload_g':float(d.payload.iloc[0]),
            'programmed_speed_m_s':float(d.speed.iloc[0]),'programmed_altitude_m':float(d.altitude.iloc[0]),
            'density_kg_m3':float(d.density.iloc[0]),'regimes':d.regime.value_counts().to_dict(),
            'negative_current_rows':int((d.battery_current<0).sum()),
            'quaternion_norm_max_deviation':float(abs(np.linalg.norm(d[['x.1','y.1','z.1','w']],axis=1)-1).max())})
    reg={'passed':True,'constant_power_Wh':trap_wh([600,600,600],[0,1800,3600]),
         'ramp_power_Wh':trap_wh([0,300,600],[0,1800,3600]),
         'simps_constant_Wh':simps_old([600]*4,[0,1000,2000,3600])/3600}
    assert reg['constant_power_Wh']==600 and reg['ramp_power_Wh']==300 and abs(reg['simps_constant_Wh']-600)<1e-10
    # Independent algebra check against direct call to the unchanged author code.
    d=data.iloc[:20];f=features(d);k1,k2,c2,c4,c5=get_liu().optimum_values()
    mg=(3.71+f.payload.to_numpy())*9.81;v=f.airspeed.to_numpy();a=f.aoa.to_numpy();vz=f.vertspd.to_numpy()
    thrust=np.sqrt((mg-c5*(v*np.cos(a))**2)**2+(c4*v**2)**2)
    expected=k1*thrust*(vz/2+np.sqrt((vz/2)**2+thrust/k2**2))+c2*thrust**1.5+c4*v**3
    actual=get_liu().power(f.T,k1,k2,c2,c4,c5)
    reg['liu_algebra_max_abs_W']=float(abs(expected-actual).max());assert reg['liu_algebra_max_abs_W']<1e-9
    save(OUT/'regression.json',reg)
    save(OUT/'input_audit.json',{'flights':rows,'available_test_flights':ids,'original_random_test_flights':[268,269,270,271,272,273,276,277,278,279],
        'missing_flights':[278,279],'records':len(data),'units_checked':True,
        'scope':'All rows in author flights_validation.csv; postprocessed airborne record, not full raw bag or idle-inclusive battery discharge.',
        'density_source':'Column published by original data author; no substitute standard atmosphere.',
        'TCN_scaler':'Original accepted-flight sheet and fitted input scaler unavailable. Do not estimate scaler from this test set.',
        'training_test_disjoint':'Eight IDs explicitly reserved as test in CMU source; publication-specific complete membership not reconstructed.',
        'raw_file':ref(OUT/'sources/rodrigues_data/MLModel/data/flights_validation.csv')})
    protocol={'task_id':TASK,'created_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
      'question':'How accurately do frozen publicly sourced physical energy models replay measured M100 flight energy, and which advanced components are reproducible?',
      'batch':'bounded_public_platform_component_validation','data_role':'author-held-out flight replay; not final inspection benchmark',
      'flight_ids':FLIGHTS,'models':MODELS,'training':False,'tuning':False,'acceptance_threshold':None,
      'common_interval':'All available timestamped records per author-held-out flight; identical for both models.',
      'truth':'Recorded V times I; actual irregular timestamps; no clipping negative measurements.',
      'primary_metric':'flight-macro mean abs(predicted_Wh - measured_Wh) / measured_Wh; trapezoid',
      'other_metrics':['signed_energy_error_pct','energy_abs_error_Wh','power_RMSE_W','power_MAE_W','per_regime_energy_error_pct','prediction_seconds'],
      'robustness':'Historical Simpson even=avg integration on same whole span; original-paper regime-separated sum reported separately.',
      'models_and_adapters':{
       MODELS[0]:{'source':'sources/cmu2021/liu_model.py','commit':'d613ba37bde8dcdaa5a9e395de4cc4029c24a39d','params':get_liu().optimum_values(),
                  'changes':'None to original power function; column/unit/Rotation adapter only. Evaluated on full available airborne span, not CMU >7m crop.'},
       MODELS[1]:{'paper':'https://doi.org/10.1016/j.patter.2022.100569','equation':3,'table':1,
                 'coefficients':COEFF,'coefficients_precision':'Published rounded 1-decimal coefficients; not refitted.',
                 'mass_kg':3.715,'mass_source':'Author inflightcomponents.py: 3.08+0.635',
                 'phase_labels':'Author published takeoff/cruise/landing labels; known-state replay, not preflight trajectory prediction.'}},
      'blocked_components':{
       'CMU_TCN_pretrained':'weights verified, but exact original normalization/split/crop assets inaccessible; no substitute normalization and no accuracy claim',
       'Baldo2026_voltage':'No verified author source/checkpoints; article data availability links only M100 data. No reproduced voltage accuracy.',
       'return_feasibility':'Initial usable battery energy, calibrated SOC/cutoff, ageing/temperature and inspection mission validation not supplied.'},
      'resources':{'local_CPU':1,'cloud_cost':0,'ram_limit_GiB':2,'wall_guard_seconds':300,'duplicate_live_experiments':False}}
    save(OUT/'protocol.json',protocol)
    print(json.dumps({'prepared':True,'flights':len(rows),'records':len(data),'regression':reg},ensure_ascii=False))

def batch():
    sys.path.insert(0,str(BRIDGE));from check_research_stage import require_transition
    save(OUT/'transition_receipt.json',require_transition(ROOT,f'research_quality/{TASK}_review_nodes.json',TASK))
    import subprocess
    pre=subprocess.run([sys.executable,'-X','utf8','-B',str(BRIDGE/'check_experiment_preflight.py'),'--root',str(ROOT),
           '--record',str(ROOT/f'research_quality/{TASK}_preflight.json'),'--next-step','public_baselines'],capture_output=True,text=True,encoding='utf-8',timeout=30)
    (OUT/'preflight_receipt.json').write_text(pre.stdout,encoding='utf-8');assert pre.returncode==0 and json.loads(pre.stdout)['preflight_allowed'],pre.stdout+pre.stderr
    target=OUT/'results.json'
    assert not target.exists(),'Existing results retained; no duplicate batch'
    start=time.perf_counter();data=read_data();results=[];regimes=[]
    for flight in FLIGHTS:
        d=data[data.flight==flight].copy();t=d.time.to_numpy();true=(d.battery_voltage*d.battery_current).to_numpy()
        pred_frame=pd.DataFrame({'time_s':t,'voltage_V':d.battery_voltage.to_numpy(),'current_A':d.battery_current.to_numpy(),
                                  'measured_W':true,'regime':d.regime.to_numpy()})
        for name,pred,seconds in physical_predictions(d):
            assert len(pred)==len(true) and np.isfinite(pred).all()
            actual=trap_wh(true,t);predicted=trap_wh(pred,t)
            manual=sum((float(true[i])+float(true[i+1]))*.5*float(t[i+1]-t[i]) for i in range(len(t)-1))/3600
            assert abs(manual-actual)<1e-9
            e_sim=simps_old(true,t)/3600;p_sim=simps_old(pred,t)/3600
            row={'flight':flight,'model':name,'n':len(t),'duration_s':float(t[-1]-t[0]),'measured_Wh':actual,'predicted_Wh':predicted,
               'delta_Wh':predicted-actual,'signed_error_pct':(predicted/actual-1)*100,'abs_error_pct':abs(predicted/actual-1)*100,
               'power_MAE_W':float(abs(pred-true).mean()),'power_RMSE_W':float(np.sqrt(np.mean((pred-true)**2))),
               'model_seconds':seconds,'measured_simps_Wh':e_sim,'predicted_simps_Wh':p_sim,
               'abs_error_simps_pct':abs(p_sim/e_sim-1)*100,'manual_integration_error_Wh':abs(manual-actual),
               'min_measured_voltage_V':float(d.battery_voltage.min()),'payload_g':float(d.payload.iloc[0])}
            for regime in COEFF:
                mask=(d.regime==regime).to_numpy();ts=t[mask];gt=true[mask];pr=pred[mask]
                ea=trap_wh(gt,ts);ep=trap_wh(pr,ts)
                regimes.append({'flight':flight,'model':name,'regime':regime,'n':int(mask.sum()),'duration_s':float(ts[-1]-ts[0]),
                    'measured_Wh':ea,'predicted_Wh':ep,'abs_error_pct':abs(ep/ea-1)*100,
                    'measured_simps_Wh':simps_old(gt,ts)/3600,'predicted_simps_Wh':simps_old(pr,ts)/3600})
            results.append(row);pred_frame[name+'_W']=pred
        (OUT/'predictions').mkdir(exist_ok=True)
        pred_frame.to_csv(OUT/'predictions'/f'flight_{flight}.csv',index=False,encoding='utf-8')
        assert time.perf_counter()-start<300
    summary=[]
    for name in MODELS:
        rr=[x for x in results if x['model']==name]
        summary.append({'model':name,'flights':len(rr),'mean_energy_APE_pct':float(np.mean([x['abs_error_pct'] for x in rr])),
             'median_energy_APE_pct':float(np.median([x['abs_error_pct'] for x in rr])),
             'max_energy_APE_pct':max(x['abs_error_pct'] for x in rr),
             'mean_signed_error_pct':float(np.mean([x['signed_error_pct'] for x in rr])),
             'mean_abs_energy_error_Wh':float(np.mean([abs(x['delta_Wh']) for x in rr])),
             'mean_power_RMSE_W':float(np.mean([x['power_RMSE_W'] for x in rr])),
             'mean_energy_simps_APE_pct':float(np.mean([x['abs_error_simps_pct'] for x in rr])),
             'sum_model_seconds':sum(x['model_seconds'] for x in rr)})
    output={'observed_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'task_id':TASK,
      'status':'two_frozen_physical_models_completed_advanced_components_blocked','protocol':ref(OUT/'protocol.json'),
      'rows':results,'regimes':regimes,'summary':summary,'batch_seconds':time.perf_counter()-start,
      'missing_flights':[278,279],'no_threshold_or_success_claim':True,'voltage_prediction_results':None,
      'runtime':{'python':sys.version,'numpy':np.__version__,'pandas':pd.__version__}}
    save(target,output)
    pd.DataFrame(results).to_csv(OUT/'per_flight_results.csv',index=False,encoding='utf-8-sig')
    pd.DataFrame(regimes).to_csv(OUT/'per_regime_results.csv',index=False,encoding='utf-8-sig')
    print(json.dumps({'summary':summary,'seconds':output['batch_seconds']},ensure_ascii=False,indent=2))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['prepare','batch']);a=ap.parse_args()
    if a.mode=='prepare':prepare()
    else:batch()
