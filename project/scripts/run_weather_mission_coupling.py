"""Fixed real-geography mission, prescribed weather, immutable public predictors.

This is an authorized bounded model/interface diagnostic, not a solver benchmark
or a field-accuracy test. No formal research node is advanced by this runner.
"""
import argparse,datetime,hashlib,json,os,pathlib,subprocess,sys,time,shutil
ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'总结/03_实验数据/天气因素建模/航段能耗联算'
BASE=ROOT/'总结/03_实验数据/电池能耗公开机型验证'
SRC=BASE/'sources/cmu2021';WEATHER=OUT.parent
PY38=ROOT/'.codex/venvs/m100_cmu2021/python.exe'
TASK='weather_mission_coupling';MODELS=['b-TCN','s-TCN','b-LSTM','s-LSTM','Liu']
sys.path.insert(0,str(ROOT/'Agent/Publication'))

def now():return datetime.datetime.now(datetime.timezone.utc).isoformat()
def save(p,d):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
def ref(p):return {'path':str(p.relative_to(ROOT)).replace('\\','/'),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
def read(p):return json.loads(p.read_text('utf8'))

def prepare():
    import numpy as np
    from pyproj import Geod
    import weather_mission_adapter as ma
    import weather_factors as wf
    assert not (OUT/'protocol.json').exists(),'Frozen protocol already exists; inspect rather than overwrite'
    assert shutil.disk_usage('D:/').free-512*2**20-256*2**20>30*2**30
    assert shutil.disk_usage('C:/').free-64*2**20>10*2**30
    OUT.mkdir(parents=True,exist_ok=True)
    path=ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes/ubeda_osm_060.json';s=read(path)
    tower=s['towers'][0];route=next(r for r in s['tower_routes'] if r['tower_id']==tower['id'])
    stop=next(x for x in s['stops'] if x['reason']=='road_access_near_T001')
    tasks={x['paper_point_number']:x for x in s['inspection_tasks'] if x['tower_id']==tower['id']}
    geod=Geod(ellps='WGS84')
    def point(lon,lat,alt):
        az,_,dist=geod.inv(stop['lon'],stop['lat'],lon,lat);a=np.radians(az)
        return [float(dist*np.cos(a)),float(-dist*np.sin(a)),float(alt-stop['ground_m'])]
    cruise=stop['ground_m']+60
    start={'time_s':0.,'nwu_m':[0.,0.,60.],'longitude':stop['lon'],'latitude':stop['lat'],'altitude_m':cruise,'phase':'transit_start','source_id':stop['id']}
    points=[start];elapsed=0.;phases=[]
    def append(lon,lat,alt,duration,phase,sourceid):
        nonlocal elapsed
        assert duration>0
        begin=elapsed;elapsed=round(elapsed+round(duration/.1)*.1,10)
        points.append({'time_s':elapsed,'nwu_m':point(lon,lat,alt),'longitude':lon,'latitude':lat,'altitude_m':alt,'phase':phase,'source_id':sourceid})
        phases.append({'begin_s':begin,'end_s':elapsed,'phase':phase,'source_id':sourceid,'unrounded_duration_s':duration})
    first=route['points'][0];last=route['points'][-1]
    horizontal=np.linalg.norm(np.array(point(first['longitude'],first['latitude'],cruise))[:2])
    append(first['longitude'],first['latitude'],cruise,horizontal/5,'outbound','P02_to_T001')
    append(first['longitude'],first['latitude'],first['altitude_m'],cruise-first['altitude_m'],'tower_approach','T001_P01')
    local_begin=elapsed
    for i,p in enumerate(route['points']):
        if i:append(p['longitude'],p['latitude'],p['altitude_m'],p['incoming_seconds'],'local_transfer',str(p['point_number']))
        if p['photo']:append(p['longitude'],p['latitude'],p['altitude_m'],tasks[p['point_number']]['effective_service_seconds'],'photo',tasks[p['point_number']]['id'])
    local_end=elapsed
    append(last['longitude'],last['latitude'],cruise,cruise-last['altitude_m'],'tower_departure',str(last['point_number']))
    return_distance=np.linalg.norm(np.array(point(last['longitude'],last['latitude'],cruise))[:2])
    append(stop['lon'],stop['lat'],cruise,return_distance/5,'return','T001_to_P02')
    append(stop['lon'],stop['lat'],cruise,10,'end_hover','P02')
    trajectory={'source_scene':ref(path),'tower_id':tower['id'],'tower_OSM_id':tower['source_object_id'],'home':stop,'points':points,'phases':phases,
      'duration_s':elapsed,'local_inspection_duration_s':local_end-local_begin,'original_local_duration_s':route['net_reference_seconds'],
      'photo_count':len(tasks),'coordinate_frame':'Local geodesic north/west/up about P02; WGS84 geodesic distance/azimuth; up relative to P02 model ground',
      'path_role':'Prescribed airborne development mission; mapped tower/road geography with transferred photo geometry; collision/terrain/parking and actual M100 flight not field validated',
      'excluded':'Initial takeoff and final landing; energy and budget screen refer only to airborne segment',
      'time_adapter':'Each existing service/transfer duration independently rounded to nearest0.1s; original300s reference retained; additional approach/departure1m/s, transit5m/s,20s excluded stabilization and10s included end-hover explicitly configured'}
    save(OUT/'mission.json',trajectory)
    outdir=np.array(point(first['longitude'],first['latitude'],cruise));outdir[2]=0;outdir/=np.linalg.norm(outdir)
    cross=np.array([-outdir[1],outdir[0],0])
    cases=[]
    def make(name,temp,wind,role,dew=None,p=1013.25,precip=0,code=0,gust=None,source=None):
        if dew is None:
            vapor=.5*wf.TETENS(temp);a=np.log10(vapor/611);dew=float(237.3*a/(7.5-a))
        wind=np.asarray(wind);screen=wf.weather_screen(temp,float(np.linalg.norm(wind[:2])),precip,code,gust)
        cases.append({'id':name,'temperature_C':temp,'dewpoint_C':dew,'pressure_hPa':p,'density_kgm3':wf.density_surface(temp,dew,p),
          'wind_nwu_ms':wind.tolist(),'wind_enu_ms':[-float(wind[1]),float(wind[0]),float(wind[2])],
          'role':role,'weather_screen':screen,'source':source,'vertical_wind_assumption':'explicit0 for conditional diagnostic, not observed0'})
    make('calm_15C',15,[0,0,0],'controlled baseline: sea-level pressure1013.25hPa, RH50%; not an observed weather event')
    for speed in [4,8]:
        for kind,unit in [('outbound_tail',outdir),('outbound_head',-outdir),('cross',cross)]:make(f'{kind}_{speed}ms',15,speed*unit,'controlled wind sensitivity; global wind fixed, reverses role on return')
    for temp in [0,35]:make(f'calm_{temp}C',temp,[0,0,0],'controlled temperature sensitivity, constant pressure1013.25hPa/RH50%; no battery derating')
    make('outbound_head_12ms',15,-12*outdir,'negative weather boundary control above manufacturer10m/s')
    rows=read(WEATHER/'weather_records.json')['records']
    for season,ts in [('winter','2025-01-01T12:00Z'),('summer','2025-07-01T12:00Z')]:
        w=next(x for x in rows if x['grid_latitude']==38 and x['grid_longitude']==-3.25 and x['height_agl_m']==60 and x['time_utc']==ts)
        make(season+'_actual_noon',w['temperature_2m_C'],[w['wind_north_ms'],-w['wind_east_ms'],0],
          'Observed API reanalysis background at frozen timestamp; uniform60m background over mission, no tower-scale downscaling',
          dew=w['dew_point_2m_C'],p=w['surface_pressure_hPa'],precip=w['precipitation_previous_hour_mm'],code=w['weather_code'],gust=w['wind_gusts_10m_ms'],
          source={'file':ref(WEATHER/'weather_records.json'),'sample_id':w['weather_sample_id'],'height_agl_m':60})
    save(OUT/'cases.json',cases)
    reg=ma.regression();reg['reference_photo_count']=len(tasks);reg['reference_local_duration_s']=route['net_reference_seconds']
    assert len(tasks)==9 and abs(route['net_reference_seconds']-300)<1e-6
    save(OUT/'regression.json',reg)
    protected=[ref(SRC/f) for f in ['QuadSim.py','data_utils.py','deep_energy_model.py','evaluation.py','liu_model.py','MetarHandler.py']]
    for m in MODELS[:-1]:protected.extend(ref(p) for p in sorted((SRC/'models'/m).glob('cp.ckpt*')))
    protocol={'task_id':TASK,'frozen_at':now(),'authorization':{'quote':'嗯 开始！','context':'User accepted proposed weather–real mission–energy diagnostic before full scheduling','thread_id':os.environ.get('CODEX_THREAD_ID'),
      'source':'Current conversation; exact new-message JSONL receipt unavailable in current local session snapshot; no fabricated line approval'},
      'stage':'Same model/data/evaluator node: bounded deterministic sensitivity, not formal method/solver comparison; no stage expansion',
      'question':'Under one frozen real-geography inspection path, how do wind and temperature change modelled energy, tracking and energy/weather screening?',
      'cases':[x['id'] for x in cases],'models':MODELS,'mission':ref(OUT/'mission.json'),'weather_conditions':ref(OUT/'cases.json'),'protected_author_assets':protected,
      'original_model_config_and_normalization':ref(BASE/'cmu_pretrained/input_audit.json'),
      'adapter':{'dynamics':'Unchanged author force/controller with exact-hover0/0 continuous limit; originaldist_std=0 explicit; Dryden zero; fixed20s warmup excluded from outcomes',
        'features':'Match original measured-data feature definitions: horizontal air components rotated by actual body yaw; aoa is signed first Euler angle(roll), as calc_aoa(index=0). Do not substitute track heading or unsigned tilt from original sim exporter.',
        'normalization':'Immutable original train+validation min/max; no fitting on synthetic mission inputs; no clipping',
        'density':'Constant per case as required by original network fixed-input contract; historical case uses original grid surface-density proxy, not terrain corrected'},
      'budget_screen':{'battery':'M100 TB47D nominal99.9Wh from DJI manual printed61','nominal_Wh':99.9,'reserve_fraction':.2,'reserve_source':'Existing scene MODEL_OPERATION design value; unchanged','available_reference_Wh':79.92,'initial_energy_assumption':'Reference fully charged nominal battery; not measured usable energy or temperature-adjusted capacity','operational_feasibility':None},
      'metrics':['same airborne interval Wh and deltaWh','phase energy','feature envelope out-of-domain rows','reference tracking RMS/max/end error','weather screen','nominal energy-budget screen'],
      'scientific_success_threshold':None,'formal_statistics':False,'training':False,'paid_cloud':False,'scope_limit':'No accuracy claim without measured flight power; no CFD, turbulence law, battery-temperature curve, optimizer or hypergraph comparison',
      'resources':{'CPU_threads':2,'RAM_GiB':4,'wall_guard_seconds':900,'peak_write_bytes':512*2**20,'cloud_cost':0}}
    save(OUT/'protocol.json',protocol)
    p=ROOT/'.codex/storage_manifest.json';st=read(p);st[TASK]={'observed_at':now(),'peak_write_reservation_bytes':512*2**20,'C_peak_temporary_bytes':64*2**20,'other_live_reservations_bytes':256*2**20,'cloud_cost':0,'status':'fixed_mission_prepared'};save(p,st)
    ev=ROOT/'.codex/evaluation_events'
    for kind,body in [('task',{'task':TASK,'title':'真实航段天气能耗联算','type':'experiment','stage':'1','checks':['frozen_mission_and_weather','author_model_integrity','coordinates_and_time_alignment','saved_energy_verification','validity_and_feasibility_limits'],'evaluation_version':'1','execution_version':ref(pathlib.Path(__file__))['sha256']}),('work_start',{'task':TASK,'session':os.environ.get('CODEX_THREAD_ID')})]:save(ev/f'{TASK}-{kind}.json',{'id':f'{TASK}-{kind}','kind':kind,'at':now(),'body':body})
    print(json.dumps({'prepared':True,'cases':len(cases),'models':len(MODELS),'mission_s':elapsed,'photo_count':len(tasks),'tower_s':local_end-local_begin,'regression':reg['passed']},ensure_ascii=False))

def batch():
    import numpy as np,pandas as pd
    import weather_mission_adapter as ma
    import run_m100_pretrained as original
    tf,du,dm,ev,liu=original.runtime()
    proto=read(OUT/'protocol.json');assert os.environ.get('PAPER11_WEATHER_ADMITTED')==ref(OUT/'protocol.json')['sha256']
    assert not (OUT/'results.json').exists(),'Completed run preserved, no duplicate'
    for a in proto['protected_author_assets']:assert ref(ROOT/a['path'])==a
    mission=read(OUT/'mission.json');cases=read(OUT/'cases.json');audit=read(BASE/'cmu_pretrained/input_audit.json')
    lo=np.array(audit['data_min']);hi=np.array(audit['data_max']);start=time.perf_counter()
    drange=read(WEATHER/'model.json')['CMU_trainval_density_range_kgm3']
    states={};inputs_data={};paths={};case_info=[]
    (OUT/'predictions').mkdir(exist_ok=True)
    for c in cases:
        s=ma.simulate(mission,c)
        # Necessary frame/type adapter matches original measured-data preprocessing.
        s['airspeed_x']=s.air_n*np.cos(s.yaw)+s.air_w*np.sin(s.yaw)
        s['airspeed_y']=-s.air_n*np.sin(s.yaw)+s.air_w*np.cos(s.yaw)
        s['aoa_original_sim_export']=s.aoa;s['aoa']=s['roll']
        old_x=s.airspeed*np.cos(np.radians(s.airspeed_angle-s.heading));old_y=s.airspeed*np.sin(np.radians(s.airspeed_angle-s.heading))
        feature_cols=['airspeed_x','airspeed_y','vertspd','aoa','airspeed','density','payload']
        features=s[feature_cols].values.copy();norm=features.copy();norm[:,:5]=(norm[:,:5]-lo[:5])/(hi[:5]-lo[:5])
        x,y=du.multivariate_data(norm,np.zeros(len(norm)),0,None,20,-1,1,True)
        inputs_data[c['id']]=(x,y);states[c['id']]=s
        target_s=s.iloc[19:];mask=target_s.time.values>=0
        raw=features[19:][mask];ood=((raw[:,:5]<lo[:5])|(raw[:,:5]>hi[:5]));density_bad=not drange[0]<=c['density_kgm3']<=drange[1]
        track=s[s.time>=0];dist=track.tracking_error_m.values
        case_info.append({'case_id':c['id'],'predicted_rows':len(track),'duration_s':float(track.time.iloc[-1]-track.time.iloc[0]),
          'density_ood':density_bad,'varying_ood_rows':int(ood.any(axis=1).sum()),'any_ood_rows':int(len(raw) if density_bad else ood.any(axis=1).sum()),
          'feature_ood_rows':dict(zip(feature_cols[:5],[int(i) for i in ood.sum(axis=0)])),
          'feature_min':dict(zip(feature_cols,map(float,raw.min(axis=0)))),'feature_max':dict(zip(feature_cols,map(float,raw.max(axis=0)))),
          'tracking_rms_m':float(np.sqrt(np.mean(dist**2))),'tracking_max_m':float(dist.max()),'end_error_m':float(dist[-1]),
          'original_sim_vs_measured_adapter_max_air_component_diff_ms':float(np.maximum(abs(s.airspeed_x-old_x),abs(s.airspeed_y-old_y)).max()),
          'original_sim_vs_measured_adapter_max_aoa_diff_rad':float(abs(s.aoa-s.aoa_original_sim_export).max())})
        s.to_csv(OUT/'predictions'/(c['id']+'_states.csv'),index=False)
        paths[c['id']]=pd.DataFrame({'time_s':track.time.values})
        print('dynamics',c['id'],'rows',len(track),flush=True)
    results=[]
    for model_name in MODELS:
        tf.keras.backend.clear_session()
        if model_name!='Liu':
            cfg=ev.get_pre_def_configs()[model_name];config=dm.load_inputs(**cfg);model=dm.create_advanced_net((20,5),**config)
            status=model.load_weights(str(SRC/'models'/model_name/'cp.ckpt'));status.assert_existing_objects_matched();status.expect_partial()
            tensors=du.create_tensors(inputs_data,[c['id'] for c in cases],**config)
        for c in cases:
            s=states[c['id']];s_target=s.iloc[19:];mask=s_target.time.values>=0;ts=s_target.time.values[mask]
            t0=time.perf_counter()
            if model_name=='Liu':pred=liu.power(s[['vertspd','airspeed','aoa','payload','density']].T,*liu.optimum_values())[19:][mask]
            else:
                model.reset_states();full=model.predict(tensors[c['id']],verbose=0).reshape(-1)*(hi[-1]-lo[-1])+lo[-1]
                assert len(full)==len(s_target);pred=full[mask]
            elapsed=time.perf_counter()-t0
            assert np.isfinite(pred).all();energy=float(np.trapz(pred,ts)/3600)
            interval_energy=(pred[:-1].astype('float64')+pred[1:].astype('float64'))/2*np.diff(ts)/3600
            mid=(ts[:-1]+ts[1:])/2;phase_energy={}
            for phase in mission['phases']:
                sel=(mid>=phase['begin_s'])&(mid<phase['end_s']);phase_energy[phase['phase']]=phase_energy.get(phase['phase'],0)+float(interval_energy[sel].sum())
            assert abs(sum(phase_energy.values())-energy)<1e-4
            row={'case_id':c['id'],'model':model_name,'energy_Wh':energy,'phase_energy_Wh':phase_energy,'mean_power_W':float(pred.mean()),'min_power_W':float(pred.min()),'max_power_W':float(pred.max()),
              'reference_budget_Wh':79.92,'reference_margin_Wh':79.92-energy,'reference_budget_pass':energy<=79.92,
              'weather_base_pass':c['weather_screen']['base_screen_pass'],'joint_reference_screen':energy<=79.92 and c['weather_screen']['base_screen_pass'],
              'operational_flight_feasible':None,'prediction_seconds':elapsed,'samples':len(pred),'ground_truth_Wh':None,'accuracy_error_pct':None}
            results.append(row);paths[c['id']][model_name+'_W']=pred
            assert time.perf_counter()-start<900
        print('predictions',model_name,'completed',flush=True)
    for p in results:
        e0=next(x['energy_Wh'] for x in results if x['case_id']=='calm_15C' and x['model']==p['model']);p['delta_vs_calm15_Wh']=p['energy_Wh']-e0;p['delta_vs_calm15_pct']=(p['energy_Wh']/e0-1)*100
    for cid,d in paths.items():d.to_csv(OUT/'predictions'/(cid+'_power.csv'),index=False)
    for a in proto['protected_author_assets']:assert ref(ROOT/a['path'])==a
    save(OUT/'results.json',{'task_id':TASK,'observed_at':now(),'status':'complete_conditional_mission_diagnostic','protocol':ref(OUT/'protocol.json'),'rows':results,'case_diagnostics':case_info,
      'pairs':len(results),'cases':len(cases),'models':len(MODELS),'seconds':time.perf_counter()-start,'source_assets_unchanged':True,
      'scope':'Simulated states on real mapped geography with prescribed task geometry; conditional predictions, not observed accuracy, full-flight feasibility or solver evidence'})
    print(json.dumps({'complete':True,'pairs':len(results),'seconds':time.perf_counter()-start},ensure_ascii=False),flush=True)

def launch():
    import psutil
    p=OUT/'execution.json'
    if p.exists():
        old=read(p);pid=old.get('pid')
        if pid and psutil.pid_exists(pid) and abs(psutil.Process(pid).create_time()-old['create_time'])<.1:raise RuntimeError('Same task process still live; refusing duplicate')
    assert not (OUT/'results.json').exists(),'Completed task retained'
    protocol=read(OUT/'protocol.json');assert read(OUT/'regression.json')['passed']
    save(OUT/'stage_scope_receipt.json',{'observed_at':now(),'scope':protocol['stage'],'review_gate_policy':ref(ROOT/'.codex/review_gate_policy.json'),
      'basis':'.codex/paper_rules/reviewed_benchmark_workflow.md section0: bounded model/data/evaluator validation allowed; same authorized node work is not a stage transition',
      'formal_stage_transition_attempted':False,'authorization':protocol['authorization'],'preflight_type':'Model interface regression and exact frozen-input identity; no new proposed-method comparison'})
    env=os.environ.copy();env['PAPER11_WEATHER_ADMITTED']=ref(OUT/'protocol.json')['sha256']
    log=OUT/'execution.log'
    with log.open('w',encoding='utf8') as out:
        proc=subprocess.Popen([str(PY38),'-X','utf8','-B',str(pathlib.Path(__file__).resolve()),'runtime'],cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT)
        identity=psutil.Process(proc.pid)
        save(p,{'task_id':TASK,'pid':proc.pid,'create_time':identity.create_time(),'started_at':now(),'status':'running','protocol':ref(OUT/'protocol.json'),'wall_guard_s':930,'log':str(log.relative_to(ROOT))})
        try:rc=proc.wait(timeout=930)
        except subprocess.TimeoutExpired:proc.terminate();rc=proc.wait(timeout=20);save(OUT/'timeout.json',{'at':now(),'reason':'finite diagnostic guard930s','exit':rc});raise
    d=read(p);d.update(status='completed' if rc==0 else 'failed',exit_code=rc,ended_at=now());save(p,d)
    print(log.read_text('utf8')[-7000:]);assert rc==0

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['prepare','run','runtime']);a=ap.parse_args()
    {'prepare':prepare,'run':launch,'runtime':batch}[a.mode]()
