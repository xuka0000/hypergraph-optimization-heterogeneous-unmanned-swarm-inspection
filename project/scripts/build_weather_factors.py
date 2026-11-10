"""Construct traceable scene weather overlays and perform known-case validation.

This is the model/data/evaluator node, not an optimization-method comparison.
"""
import ast,datetime,hashlib,importlib.util,json,math,pathlib,sys,time
import numpy as np,pandas as pd
from prepare_weather_factors import ROOT,OUT,TASK,NAMES,PERIODS,VARIABLES,ref,save,now
sys.path.insert(0,str(ROOT/'Agent/Publication'))
import weather_factors as wf

def load_original(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def known_checks():
    checks=[]
    def check(name,condition,details):
        assert bool(condition),(name,details);checks.append({'name':name,'passed':True,'evidence':details})
    check('cardinal_wind_from_convention',all(np.allclose(wf.wind_uv(4,a),x,atol=1e-12) for a,x in [(0,[0,-4]),(90,[-4,0]),(180,[0,4]),(270,[4,0])]),'ECMWF meteorological from-angle; east/north components')
    check('wind_speed_preservation',all(abs(np.linalg.norm(wf.wind_uv(7,a))-7)<1e-12 for a in range(0,361,5)),'73 angular cases')
    check('height_endpoints',np.allclose(wf.height_wind_uv(1,2,3,4,10),[1,2]) and np.allclose(wf.height_wind_uv(1,2,3,4,100),[3,4]),'Original10m/100m values preserved')
    a=wf.wind_uv(4,359);b=wf.wind_uv(4,1);mid=wf.height_wind_uv(*a,*b,math.sqrt(1000))
    check('angular_wrap_no_false_reversal',mid[1]<-3.99 and abs(mid[0])<1e-12,mid.tolist())
    rejected=False
    try:wf.height_wind_uv(1,2,3,4,9)
    except ValueError:rejected=True
    check('no_below10m_extrapolation',rejected,'Below10m unsupported, not silently extrapolated')
    check('relative_velocity_identity',np.allclose(wf.air_relative_enu([6,0,0],[4,0,0]),[2,0,0]),'Ground6east minus tailwind4east equals air2east')
    rho=wf.density_surface(20,10,1013.25);e=wf.TETENS(10);ideal=(101325-e)/(287.05*293.15)+e/(461.5*293.15)
    coefficient_difference=((.0034848-1/287.05)*101325+((1/287.05-1/461.5)-.0034848*.37960)*e)/293.15
    check('density_independent_formula_and_author_coefficients',abs(rho-ideal-coefficient_difference)<1e-12,
      {'CMU_kgm3':rho,'ideal_kgm3':ideal,'relative_difference':abs(rho-ideal)/ideal,
       'analytic_coefficient_difference':coefficient_difference,'note':'Original author constants differ slightly from modern ideal-gas constants; original source preserved, not tuned to match.'})
    check('density_temperature_pressure_humidity_signs',wf.density_surface(30,10,1013.25)<rho and wf.density_surface(20,10,900)<rho and wf.density_surface(20,15,1013.25)<rho,'At otherwise fixed conditions temperature/humidity up or pressure down reduces gas density; no battery-capacity inference')
    check('saturation_profile_not_silently_clipped',wf.density_height_proxy(10,10,1013,100)['air_density_proxy_kg_m3'] is None,'Unsaturated-lapse inconsistency remains unknown')
    check('manufacturer_boundaries',wf.weather_screen(-10,10,0,0)['base_screen_pass'] and wf.weather_screen(40,10,0,0)['base_screen_pass'] and not wf.weather_screen(40.1,10,0,0)['base_screen_pass'],'Inclusive published operating limits')
    check('precip_and_gust_separate',not wf.weather_screen(20,4,.1,0)['base_screen_pass'] and wf.weather_screen(20,4,0,0,11)['base_screen_pass'] and not wf.weather_screen(20,4,0,0,11)['gust_screen_pass'],'Rain threshold and additional10m gust sensitivity are distinct')
    check('unknown_hazard_not_zero',wf.weather_screen(None,4,0,0)['base_screen_pass'] is None and wf.weather_screen(20,4,0,0)['gust_screen_pass'] is None,'Missing core/gust inputs stay unknown')
    testrows=[{'time_utc':t+'Z','base_screen_pass':True} for t in ['2025-01-01T00:00','2025-01-01T01:00','2025-07-01T00:00']]
    windows=wf.contiguous_windows(testrows);check('disconnected_periods_not_one_window',[x['screened_hours'] for x in windows]==[2,1],windows)
    # Exact public energy function: reference aircraft5kg; no M100 calibration claim.
    m=load_original(OUT/'sources/geowind_energy_model.py','author_geowind_energy')
    x=np.linspace(0,1000,101);P=np.column_stack([x,np.full_like(x,60),np.zeros_like(x)]);V=np.tile([6.,0.,0.],(101,1));cases=[]
    for name,wind in [('calm',[0,0,0]),('tailwind4',[4,0,0]),('headwind4',[-4,0,0]),('crosswind4',[0,0,4])]:
        energy=m.get_energy_xv(P,V,lambda p,w=np.array(wind,dtype=float):w)[0]
        va=np.array([6.,0.,0.])-np.array(wind);eps=1e-9;speed=np.sqrt(np.dot(va,va)+eps)
        drag=-.5*1.18*.16*speed*va;thrust=np.array([0,5*9.81,0])-drag;tn=np.sqrt(np.dot(thrust,thrust)+eps)
        vc=np.dot(va,thrust/(tn+eps));vi=-.5*vc+np.sqrt((.5*vc)**2+tn/(2*1.18*.5)+eps)
        power=np.dot(thrust,va)+tn*vi+100*(tn/(5*9.81))**1.5;manual=power*1000/6
        check('public_energy_steady_'+name,abs(energy-manual)<1e-7,{'author_J':energy,'independent_J':manual})
        cases.append({'case':name,'wind_author_axes_east_up_north':wind,'ground_speed_ms':6,'airspeed_ms':float(np.linalg.norm(va)),
          'distance_m':1000,'seconds':1000/6,'predicted_Wh':float(energy/3600),'source':'unchanged GeoWind2Plan2026 author energy_model.py','aircraft':'author5kg reference, not M100'})
    return checks,cases

def build():
    start=time.perf_counter();proto=json.loads((OUT/'protocol.json').read_text('utf8'))
    assert proto['stage'].startswith('model/data/evaluator')
    mapping=json.loads((OUT/'scene_weather_mapping.json').read_text('utf8'));manifest=json.loads((OUT/'source_manifest.json').read_text('utf8'))
    for m in mapping:assert ref(ROOT/m['source_scene']['path'])['sha256']==m['source_scene']['sha256']
    for a in manifest['assets']:assert ref(ROOT/a['path'])['sha256']==a['sha256']
    checks,diagnostic=known_checks()
    cmu=ROOT/'总结/03_实验数据/电池能耗公开机型验证';audit=json.loads((cmu/'cmu_pretrained/input_audit.json').read_text('utf8'))
    trainval=audit['splits']['train']+audit['splits']['val'];dens=[]
    for f in trainval:dens.extend(pd.read_csv(cmu/'sources/cmu2021/data'/str(f)/'processed.csv',usecols=['density']).density.unique())
    drange=[float(min(dens)),float(max(dens))]
    rows=[];groups={};raw_count=0;raw_keys=set()
    for season,begin,end in PERIODS:
        raw=OUT/'sources'/('era5_'+season+'.json');data=json.loads(raw.read_text('utf8'));assert len(data)==5
        for (lat,lon),r in zip(proto['requested_cells'],data):
            assert abs(r['latitude']-lat)<1e-6 and abs(r['longitude']-lon)<1e-6
            assert r['utc_offset_seconds']==0 and r['hourly_units']['wind_speed_10m']=='m/s' and r['hourly_units']['surface_pressure']=='hPa'
            d=r['hourly'];assert len(d['time'])==168 and all(len(v)==168 for v in d.values())
            assert d['time'][0]==begin+'T00:00' and d['time'][-1]==end+'T23:00'
            assert all(v is not None for key in VARIABLES for v in d[key])
            grid_id=('lat%.2f_lon%.2f'%(lat,lon)).replace('-','m')
            for i,ts in enumerate(d['time']):
                key=(lat,lon,ts);assert key not in raw_keys;raw_keys.add(key);raw_count+=1
                u10,v10=wf.wind_uv(d['wind_speed_10m'][i],d['wind_direction_10m'][i]);u100,v100=wf.wind_uv(d['wind_speed_100m'][i],d['wind_direction_100m'][i])
                t=d['temperature_2m'][i];dew=d['dew_point_2m'][i];p=d['surface_pressure'][i];rho=wf.density_surface(t,dew,p)
                for h in proto['diagnostic_heights_agl_m']:
                    uv=wf.height_wind_uv(u10,v10,u100,v100,h);v=float(np.linalg.norm(uv));hp=wf.density_height_proxy(t,dew,p,h)
                    sc=wf.weather_screen(t,v,d['precipitation'][i],d['weather_code'][i],d['wind_gusts_10m'][i],d['is_day'][i])
                    row={'grid_id':grid_id,'grid_latitude':lat,'grid_longitude':lon,'grid_elevation_m':r['elevation'],
                      'season_window':season,'time_utc':ts+'Z','height_agl_m':h,'wind_east_ms':float(uv[0]),'wind_north_ms':float(uv[1]),'wind_speed_ms':v,
                      'wind_vertical_ms':None,'gust_at_height_ms':None,'wind_gusts_10m_ms':d['wind_gusts_10m'][i],
                      'temperature_2m_C':t,'dew_point_2m_C':dew,'surface_pressure_hPa':p,'relative_humidity_2m_pct':d['relative_humidity_2m'][i],
                      'precipitation_previous_hour_mm':d['precipitation'][i],'weather_code':d['weather_code'][i],'is_day':d['is_day'][i],
                      'air_density_surface_proxy_kgm3':rho,**hp,'density_outside_CMU_trainval_envelope':not drange[0]<=rho<=drange[1],
                      **sc,'weather_sample_id':grid_id+'_'+ts+'Z','data_role':'historical reanalysis development; no issue-time forecast available'}
                    rows.append(row);groups.setdefault((lat,lon,season,h),[]).append(row)
    assert raw_count==1680 and len(rows)==5040
    save(OUT/'weather_records.json',{'schema_version':'1.0','protocol':ref(OUT/'protocol.json'),'source_manifest':ref(OUT/'source_manifest.json'),
      'unique_grid_hours':raw_count,'height_rows':len(rows),'records':rows})
    summaries=[];overlays=[]
    for m in mapping:
        cells=sorted({(t['requested_weather_grid_latitude'],t['requested_weather_grid_longitude']) for t in m['towers']})
        scene_rows=[];season_rows=[]
        for season,_,_ in PERIODS:
            for h in proto['diagnostic_heights_agl_m']:
                series=[groups[(*cell,season,h)] for cell in cells];merged=[]
                for i in range(168):
                    selected=[s[i] for s in series];x={'time_utc':selected[0]['time_utc'],
                      'base_screen_pass':all(y['base_screen_pass'] for y in selected),'gust_screen_pass':all(y['gust_screen_pass'] for y in selected)};merged.append(x)
                available=sum(x['base_screen_pass'] for x in merged);gust=sum(x['gust_screen_pass'] for x in merged)
                flattened=[x for s in series for x in s]
                summary={'scene_id':m['scene_id'],'name':m['name'],'season':season,'height_agl_m':h,'tower_entries':len(m['towers']),'weather_cells':len(cells),
                  'hour_bins':168,'all_scene_cells_base_pass_hours':available,'all_scene_cells_gust_pass_hours':gust,
                  'pass_pct':available/168*100,'gust_pass_pct':gust/168*100,
                  'temperature_range_C':[min(x['temperature_2m_C'] for x in flattened),max(x['temperature_2m_C'] for x in flattened)],
                  'density_range_kgm3':[min(x['air_density_surface_proxy_kgm3'] for x in flattened),max(x['air_density_surface_proxy_kgm3'] for x in flattened)],
                  'max_mean_wind_ms':max(x['wind_speed_ms'] for x in flattened),'max_gust10_ms':max(x['wind_gusts_10m_ms'] for x in flattened),
                  'longest_base_screen_run_hours':max([x['screened_hours'] for x in wf.contiguous_windows(merged)] or [0])}
                summaries.append(summary);scene_rows.append({'season':season,'height_agl_m':h,'summary':summary,
                  'base_screen_windows':wf.contiguous_windows(merged),'gust_screen_windows':wf.contiguous_windows(merged,'gust_screen_pass')})
        overlay={'schema_version':'weather-overlay-1.0','scene_id':m['scene_id'],'name':m['name'],'source_scene':m['source_scene'],
          'profile':wf.M100_PROFILE,'weather_record_file':ref(OUT/'weather_records.json'),'requested_weather_cells':cells,'towers':m['towers'],
          'periods':PERIODS,'weather_selection':'Query record by grid_latitude/grid_longitude/time_utc/height_agl_m; shared sample_id across linked missions',
          'per_period_height_screening':scene_rows,'aggregate_semantics':'All weather cells must pass for scene-wide screen; individual missions must evaluate only actually traversed cells and all occupied time bins.',
          'precision':{'horizontal':'ERA5 0.25degree regional background','temporal':'hourly','precipitation':'provider0.1mm reporting; zero does not exclude trace precipitation',
            'wind_vertical_ms':None,'gust_at_flight_height_ms':None,'tower_local_wind_correction':None,'forecast_issue_time':None,'weather_sample_probability':None},
          'operational_flight_clearance':None,'is_attached_to_live_optimizer':False}
        op=OUT/'scenes'/(m['scene_id']+'.json');save(op,overlay);overlays.append(ref(op))
    # Explicit source-to-model adaptation; no claim of untouched Gao reproduction.
    model={'task_id':TASK,'profile':wf.M100_PROFILE,'frame':'ENU east/north/up; meteorological direction FROM north clockwise',
      'equations':{'wind':'u=-U sin(theta); v=-U cos(theta)','air_velocity':'v_air=v_ground-w',
        'density':'Original CMU MetarHandler.density(T,Td,p_hPa), extracted unchanged; independent moist ideal-gas check',
        'height_wind':'alpha=ln(h/10)/ln(10); (u_h,v_h)=(1-alpha)*(u10,v10)+alpha*(u100,v100)',
        'energy':'E_k(s)[Wh]=(1/3600)*integral P(v_air,attitude,payload,rho)[W] dt[s]; no additional weather multiplier',
        'mission_weather':'a_k(t,s)=AND of selected weather screening fields over all occupied cells/time bins; x_k<=a_k',
        'battery_coupling':'B_next(s)=B_now(s)-E_k(s)+charge_k(s); use shared weather scenario s across all missions, not independent per-task draws',
        'chance_constraint':'P[shared resource/energy constraints hold across scenario s]>=1-alpha; alpha and probabilities unset pending evidence'},
      'references':{'weather_windows':'Gao et al2021 DOI10.1038/s41598-021-91325-w corrected2021; adapted regional/M100 profile, not global-result reproduction',
        'density':'CMU2021 original source + independent moist-air gas law','wind_frame':'ECMWF official wind-components documentation',
        'wind_energy':'GeoWind2Plan2026 arXiv2609.36056 AppendixE + unchanged author energy_model.py;5kg reference only'},
      'CMU_trainval_density_range_kgm3':drange,'CMU_training_flights':114,'CMU_validation_flights':38,
      'integration_audit':{'TCN':'density and two body-frame relative-airspeed components are active inputs; new geographic rollout also needs attitude/heading dynamics and out-of-domain checks',
        'Liu':'Current original power() reads density but does not use it in active return-value calculation; source unchanged',
        'avoid_double_wind':'Published M100 wind_speed is treated as measured airspeed by author source; do not subtract ambient wind again from that channel'},
      'missing_calibration':['tower-scale terrain/building/tower CFD or observations','gusts at actual flight height','vertical wind','weather forecast error and issue times',
        'flight attitude response for forecast inputs','battery thermal/capacity/charging curves','precise visibility and lightning','local-height thermodynamic sounding'],
      'hypergraph_boundary':'Fixed weather windows alone only prune candidate missions; higher-order advantage requires measured shared battery/charging/meeting constraints that alter combined decisions. Not demonstrated here.'}
    model['units']={'power':'W','time':'s','energy_and_battery_state':'Wh','wind':'m/s ENU','density':'kg/m^3','weather_height':'m above ERA5 grid terrain; not tower-local adjusted AGL'}
    save(OUT/'model.json',model)
    results={'task_id':TASK,'observed_at':now(),'status':'completed_model_data_known_case_validation','scope':'Regional weather component; no optimizer/training/CFD run',
      'unique_grid_hours':raw_count,'height_rows':len(rows),'scene_overlays':overlays,'summary':summaries,'wind_energy_known_cases':diagnostic,
      'weather_density_OOD_grid_hours':sum(x['height_agl_m']==60 and x['density_outside_CMU_trainval_envelope'] for x in rows),
      'lapse_profile_unknown_rows':sum(x['air_density_proxy_kg_m3'] is None for x in rows),'seconds':time.perf_counter()-start,'cloud_cost':0}
    save(OUT/'results.json',results)
    save(OUT/'validation.json',{'observed_at':now(),'known_case_checks':checks,'source_files_hash_verified':len(manifest['assets']),
      'source_scene_files_unchanged':True,'unique_grid_hours':1680,'height_rows':5040,'scene_count':6,'passed_for_current_scope':True,
      'temporal_independence_claim':False,'high_accuracy_field_weather_validated':False,'algorithm_comparison_run':False})
    pd.DataFrame(summaries).to_csv(OUT/'scene_period_summary.csv',index=False,encoding='utf-8-sig')
    print(json.dumps({'counts':[raw_count,len(rows),len(overlays)],'density_training_range':drange,'density_OOD_grid_hours':results['weather_density_OOD_grid_hours'],
      'known_case_checks':len(checks),'wind_examples':diagnostic,'height60_summary':[x for x in summaries if x['height_agl_m']==60]},ensure_ascii=False,indent=2))

if __name__=='__main__':build()
