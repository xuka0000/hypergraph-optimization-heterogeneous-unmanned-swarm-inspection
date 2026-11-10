"""Acquire bounded primary weather evidence; no training, optimizer or cloud actions."""
import datetime,hashlib,json,os,pathlib,shutil,subprocess,sys,time
import requests

ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'总结/03_实验数据/天气因素建模'
SCENES=ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes'
TASK='weather_factor_modeling'
NAMES=['ubeda_osm_060','ubeda_osm_100','ubeda_osm_150','rommerskirchen','brauweiler','villevaude']
PERIODS=[('winter','2025-01-01','2025-01-07'),('summer','2025-07-01','2025-07-07')]
VARIABLES=['temperature_2m','dew_point_2m','relative_humidity_2m','surface_pressure','wind_speed_10m','wind_direction_10m','wind_speed_100m','wind_direction_100m','wind_gusts_10m','precipitation','rain','snowfall','weather_code','is_day']
def now():return datetime.datetime.now(datetime.timezone.utc).isoformat()
def save(p,d):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
def ref(p):return {'path':str(p.relative_to(ROOT)).replace('\\','/'),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
def main():
    # Conservative total peak includes all downloaded sources, JSON overlays and PDFs.
    reserve=256*2**20;ctmp=64*2**20;other=256*2**20
    assert shutil.disk_usage('D:/').free-reserve-other>=30*2**30
    assert shutil.disk_usage('C:/').free-ctmp>=10*2**30
    sp=ROOT/'.codex/storage_manifest.json';st=json.loads(sp.read_text('utf-8-sig'))
    st[TASK]={'observed_at':now(),'drive':'D','peak_write_reservation_bytes':reserve,'C_peak_temporary_bytes':ctmp,'other_live_reservations_bytes':other,'cloud_cost':0,'status':'source_acquisition_and_model_validation'};save(sp,st)
    OUT.mkdir(exist_ok=True);(OUT/'sources').mkdir(exist_ok=True)
    mappings=[];cells=set()
    for n in NAMES:
        p=SCENES/(n+'.json');s=json.loads(p.read_text('utf8'));t=[]
        for x in s['towers']:
            cell=(round(x['latitude']*4)/4,round(x['longitude']*4)/4);cells.add(cell)
            t.append({'tower_id':x['id'],'source_object_id':x['source_object_id'],'latitude':x['latitude'],'longitude':x['longitude'],
              'ground_elevation_m':x.get('ground_elevation_m'),'requested_weather_grid_latitude':cell[0],'requested_weather_grid_longitude':cell[1]})
        mappings.append({'scene_id':n,'name':s['name'],'source_scene':ref(p),'towers':t})
    cells=sorted(cells)
    protocol={'task_id':TASK,'created_at':now(),'authorization':'User: OK！ 继续下一步就是天气因素建模！',
      'stage':'model/data/evaluator construction and bounded deterministic known-case validation; not algorithm comparison',
      'scope':'Regional weather background attached to all six existing geographic cases; retained M100 experimental aircraft profile. No change to original M300 scene files.',
      'periods':PERIODS,'period_selection':'First complete seven-day windows of January and July2025, frozen before retrieving weather; development demonstrations, not annual/climate or unseen formal evaluation.',
      'weather_model':'ERA5 only, no automatic best-match blend','grid_degrees':0.25,'requested_cells':cells,'variables':VARIABLES,'timezone':'UTC',
      'elevation_parameter':'nan: provider downscaling disabled; model grid elevation retained; tower-level adjustments kept separate',
      'interpolation':'Horizontal nearest ERA5 grid; log-height interpolation of u/v between10m and100m, explicitly a development assumption; no sub-grid turbulence generated',
      'diagnostic_heights_agl_m':[30,60,100],'primary_height_agl_m':60,
      'models_not_trained':True,'cost':0,'wall_guard_seconds':600,'max_download_file_bytes':32*2**20,
      'limits':'No claimed high-resolution local wind accuracy, field flight certification, battery cold-capacity curve, new TCN energy accuracy, optimizer superiority, or hypergraph advantage.',
      'next_review':'Review weather model/inputs and missing local wind before any optimization comparison or paid CFD/training.'}
    if (OUT/'protocol.json').exists():
        old=json.loads((OUT/'protocol.json').read_text('utf8'));assert old['periods']==[list(x) for x in PERIODS] and old['requested_cells']==[list(x) for x in cells]
    else:save(OUT/'protocol.json',protocol)
    save(OUT/'scene_weather_mapping.json',mappings)
    ev=ROOT/'.codex/evaluation_events'
    checks=['source_identity','weather_units_and_frames','deterministic_model_validation','scene_mapping_and_shared_cells','weather_uncertainty_and_limits']
    for kind,body in [('task',{'task':TASK,'title':'真实巡检场景天气因素建模','type':'experiment','stage':'1','checks':checks,'evaluation_version':'1','execution_version':'weather-model-development-1'}),
      ('work_start',{'task':TASK,'session':os.environ.get('CODEX_THREAD_ID')})]:
        p=ev/(TASK+'-'+kind+'.json')
        if not p.exists():save(p,{'id':TASK+'-'+kind,'kind':kind,'at':now(),'body':body})
    manifest=json.loads((OUT/'source_manifest.json').read_text('utf8')) if (OUT/'source_manifest.json').exists() else {'observed_at':now(),'assets':[],'attempts':[]}
    session=requests.Session();session.trust_env=False;session.proxies={'https':'http://127.0.0.1:15556'}
    start=time.monotonic()
    def get(key,url,filename,params=None):
        p=OUT/'sources'/filename
        old=next((x for x in manifest['assets'] if x['id']==key),None)
        if old and p.exists():assert ref(p)['sha256']==old['sha256'];return p
        assert time.monotonic()-start<600
        try:
            response=session.get(url,params=params,timeout=(15,45));response.raise_for_status()
            body=response.content;assert len(body)<32*2**20,('file_exceeds_bound',len(body))
            if filename.endswith('.json'):response.json()
            if filename.endswith('.pdf'):assert body.startswith(b'%PDF'),response.headers.get('Content-Type')
            p.write_bytes(body);rec={'id':key,'requested_url':response.request.url,'final_url':response.url,'status':'verified_acquisition','bytes':len(body),'content_type':response.headers.get('Content-Type'),**ref(p)}
            manifest['assets'].append(rec);print(key,'saved',len(body),flush=True)
        except Exception as e:
            manifest['attempts'].append({'id':key,'at':now(),'url':url,'status':'failed','error':repr(e),'response_excerpt':getattr(getattr(e,'response',None),'text','')[:1000]});print(key,'FAILED',repr(e),flush=True);p=None
        save(OUT/'source_manifest.json',manifest);return p
    for season,begin,end in PERIODS:
        params={'latitude':','.join(str(c[0]) for c in cells),'longitude':','.join(str(c[1]) for c in cells),
          'start_date':begin,'end_date':end,'hourly':','.join(VARIABLES),'models':'era5','timezone':'UTC','wind_speed_unit':'ms','elevation':','.join(['nan']*len(cells)),'cell_selection':'nearest'}
        get('ERA5_'+season,'https://archive-api.open-meteo.com/v1/archive','era5_'+season+'.json',params)
    docs=[('Gao2021','https://www.nature.com/articles/s41598-021-91325-w.pdf','gao2021.pdf'),
      ('Gao2021correction','https://www.nature.com/articles/s41598-021-00537-7','gao2021_correction.html'),
      ('GeoWind2Plan2026_fulltext','https://arxiv.org/html/2609.36056v1','geowind2plan2026.html'),
      ('WESPR2026','https://arxiv.org/pdf/2603.09194','wespr2026.pdf'),
      ('DJI_M100_safety','https://dl.djicdn.com/downloads/dev/Matrice/en/M100_Disclaimer_and_Safety_Guidelines_en_160304.pdf','m100_safety.pdf'),
      ('DJI_M100_spec','https://www.dji.com/matrice100','m100_specs.html'),
      ('DJI_M100_manual','https://dl.djicdn.com/downloads/m100/M100_User_Manual_EN.pdf','m100_manual.pdf'),
      ('OpenMeteo_archive','https://open-meteo.com/en/docs/historical-weather-api','openmeteo_archive.html'),
      ('ECMWF_wind','https://confluence.ecmwf.int/pages/viewpage.action?pageId=242074496','ecmwf_wind.html'),
      ('MetPy_density','https://unidata.github.io/MetPy/latest/api/generated/metpy.calc.density.html','metpy_density.html')]
    for entry in docs:get(*entry)
    p=get('GeoWind2Plan_repo','https://api.github.com/repos/DUAL-Xiao/GeoWind2Plan/commits/main','geowind_commit.json')
    if p:
        sha=json.loads(p.read_text('utf8'))['sha']
        for name in ['README.md','LICENSE','geowind2plan/energy_model.py']:
            get('GeoWind_'+name,'https://raw.githubusercontent.com/DUAL-Xiao/GeoWind2Plan/'+sha+'/'+name,'geowind_'+pathlib.Path(name).name)
    print(json.dumps({'complete_acquisitions':len(manifest['assets']),'failed_attempts':manifest['attempts'],'cells':cells,'protocol':ref(OUT/'protocol.json')},ensure_ascii=False))

if __name__=='__main__':main()
