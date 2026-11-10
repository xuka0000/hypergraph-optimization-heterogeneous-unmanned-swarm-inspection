"""Independently recompute saved weather records from provider responses.

Does not import the weather implementation or rerun an experiment.
"""
import collections, datetime, hashlib, json, math, pathlib
from prepare_weather_factors import ROOT, OUT, TASK, ref, save, now

def read(p): return json.loads(p.read_text('utf8'))
def close(a,b): assert math.isclose(a,b,rel_tol=1e-11,abs_tol=1e-11),(a,b)
def runs(times):
    answer=[]
    for t in times:
        if not answer or (t-answer[-1][-1]).total_seconds()!=3600:answer.append([])
        answer[-1].append(t)
    return answer

def main():
    proto=read(OUT/'protocol.json');result=read(OUT/'results.json');model=read(OUT/'model.json')
    data=read(OUT/'weather_records.json');mapping=read(OUT/'scene_weather_mapping.json')
    manifest=read(OUT/'source_manifest.json');raw={};bykey={};sources=0
    for x in manifest['assets']:
        assert ref(ROOT/x['path'])['sha256']==x['sha256'];sources+=1
    for season,begin,end in proto['periods']:
        for g in read(OUT/'sources'/f'era5_{season}.json'):
            assert g['utc_offset_seconds']==0
            assert g['hourly_units']['wind_speed_10m']=='m/s' and g['hourly_units']['surface_pressure']=='hPa'
            d=g['hourly'];expected=[(datetime.datetime.fromisoformat(begin)+datetime.timedelta(hours=i)).isoformat(timespec='minutes') for i in range(168)]
            assert d['time']==expected
            for i,t in enumerate(d['time']):
                k=(g['latitude'],g['longitude'],t+'Z');assert k not in raw
                raw[k]=({v:d[v][i] for v in proto['variables']},g['elevation'],season)
    fieldmap={'temperature_2m_C':'temperature_2m','dew_point_2m_C':'dew_point_2m','surface_pressure_hPa':'surface_pressure',
      'wind_gusts_10m_ms':'wind_gusts_10m','relative_humidity_2m_pct':'relative_humidity_2m','precipitation_previous_hour_mm':'precipitation','weather_code':'weather_code','is_day':'is_day'}
    unknown=0;outliers=set();passed={};rho_deltas=[]
    for row in data['records']:
        k=(row['grid_latitude'],row['grid_longitude'],row['time_utc']);h=row['height_agl_m'];key=(*k,h)
        assert key not in bykey;bykey[key]=row
        d,elev,season=raw[k];assert row['grid_elevation_m']==elev and row['season_window']==season
        for dest,src in fieldmap.items():assert row[dest]==d[src]
        a=math.log10(h/10);vec=[]
        for fun in [math.sin,math.cos]:
            val=-sum(weight*d[f'wind_speed_{level}m']*fun(math.radians(d[f'wind_direction_{level}m'])) for weight,level in [(1-a,10),(a,100)])
            vec.append(val)
        speed=math.hypot(*vec)
        for actual,expected in zip([row['wind_east_ms'],row['wind_north_ms'],row['wind_speed_ms']],[*vec,speed]):close(actual,expected)
        t=d['temperature_2m'];td=d['dew_point_2m'];p=d['surface_pressure']
        e=611*10**(7.5*td/(td+237.3))
        rho=.0034848*(100*p-.3796*e)/(t+273.15);close(rho,row['air_density_surface_proxy_kgm3'])
        ideal=(p*100-e)/(287.05*(t+273.15))+e/(461.5*(t+273.15));rho_deltas.append(abs(rho-ideal)/ideal)
        outside=not model['CMU_trainval_density_range_kgm3'][0]<=rho<=model['CMU_trainval_density_range_kgm3'][1]
        assert row['density_outside_CMU_trainval_envelope']==outside
        if outside:outliers.add(k)
        # Equivalent hydrostatic expression independent of implementation call chain.
        ph=p*math.exp(-9.80665*h*rho/(p*100));th=t-.0065*(h-2);eh=e*ph/p
        loge=math.log10(eh/611);tdh=237.3*loge/(7.5-loge)
        close(ph,row['pressure_proxy_hPa']);close(th,row['temperature_proxy_C'])
        if tdh>th+.2:
            assert row['air_density_proxy_kg_m3'] is None and row['status']=='unsaturated_lapse_assumption_invalid';unknown+=1
        else:close(.0034848*(100*ph-.3796*eh)/(th+273.15),row['air_density_proxy_kg_m3'])
        reasons=[]
        if t< -10 or t>40:reasons.append('temperature')
        if speed>10:reasons.append('mean_wind_at_height')
        if d['precipitation']>0:reasons.append('reported_precipitation')
        if d['weather_code'] not in (0,1,2,3):reasons.append('reported_weather_hazard')
        base=not reasons;gust=base and d['wind_gusts_10m']<=10
        assert row['base_reasons']==reasons and row['base_screen_pass']==base and row['gust_screen_pass']==gust
        assert row['charging_ambient_screen_pass']==(0<=t<=40)
        assert row['daylight_screen_pass']==(base and bool(d['is_day']))
        assert row['wind_vertical_ms'] is None and row['gust_at_height_ms'] is None and row['flight_clearance'] is None
        assert row['weather_sample_id']==row['grid_id']+'_'+row['time_utc']
        passed[key]=(base,gust,reasons)
    assert len(raw)==1680 and len(bykey)==5040
    tower_entries=0;physical=set();scene_checks=[];reasons_output=[];ground_unknown=0
    for scene in mapping:
        src=ROOT/scene['source_scene']['path'];assert ref(src)==scene['source_scene']
        original=read(src);o={t['id']:t for t in original['towers']}
        overlay=read(OUT/'scenes'/(scene['scene_id']+'.json'))
        assert overlay['towers']==scene['towers'] and overlay['weather_record_file']==ref(OUT/'weather_records.json')
        assert len(o)==len(scene['towers'])
        for t in scene['towers']:
            original_t=o[t['tower_id']]
            for f in ['latitude','longitude','source_object_id','ground_elevation_m']:assert t[f]==original_t.get(f)
            lat,lon=t['latitude'],t['longitude']
            assert abs(t['requested_weather_grid_latitude']-lat)<=.125 and abs(t['requested_weather_grid_longitude']-lon)<=.125
            physical.add((original_t.get('source_system','OSM'),str(t['source_object_id'])))
            tower_entries+=1;ground_unknown+=t['ground_elevation_m'] is None
        cells=overlay['requested_weather_cells']
        for item in overlay['per_period_height_screening']:
            season=item['season'];h=item['height_agl_m'];summary=item['summary']
            start=next(x[1] for x in proto['periods'] if x[0]==season)
            base_times=[];gust_times=[];reasons=collections.Counter();flat=[]
            for i in range(168):
                dt=datetime.datetime.fromisoformat(start).replace(tzinfo=datetime.timezone.utc)+datetime.timedelta(hours=i)
                ts=dt.isoformat(timespec='minutes').replace('+00:00','Z')
                selected=[bykey[(lat,lon,ts,h)] for lat,lon in cells];flat.extend(selected)
                if all(x['base_screen_pass'] for x in selected):base_times.append(dt)
                if all(x['gust_screen_pass'] for x in selected):gust_times.append(dt)
                reasons.update(set(r for x in selected for r in x['base_reasons']))
            for field,times,countkey in [('base_screen_windows',base_times,'all_scene_cells_base_pass_hours'),('gust_screen_windows',gust_times,'all_scene_cells_gust_pass_hours')]:
                assert summary[countkey]==len(times)
                expected=[{'first_hour_endpoint':s[0].isoformat(),'last_hour_endpoint':s[-1].isoformat(),'screened_hours':len(s)} for s in runs(times)]
                assert item[field]==expected
            assert summary['longest_base_screen_run_hours']==max([len(r) for r in runs(base_times)] or [0])
            close(summary['pass_pct'],100*len(base_times)/168);close(summary['gust_pass_pct'],100*len(gust_times)/168)
            for f,rf in [('temperature_range_C','temperature_2m_C'),('density_range_kgm3','air_density_surface_proxy_kgm3')]:
                close(summary[f][0],min(x[rf] for x in flat));close(summary[f][1],max(x[rf] for x in flat))
            close(summary['max_mean_wind_ms'],max(x['wind_speed_ms'] for x in flat));close(summary['max_gust10_ms'],max(x['wind_gusts_10m_ms'] for x in flat))
            assert summary==next(s for s in result['summary'] if s['scene_id']==scene['scene_id'] and s['season']==season and s['height_agl_m']==h)
            if h==60:reasons_output.append({'scene_id':scene['scene_id'],'season':season,'nonexclusive_rejected_hour_counts':dict(reasons)})
        scene_checks.append({'scene_id':scene['scene_id'],'tower_entries':len(o),'source_unchanged':True,'overlay_verified':True})
    assert len(outliers)==result['weather_density_OOD_grid_hours']==757 and unknown==result['lapse_profile_unknown_rows']==16
    assert all(x['passed'] for x in read(OUT/'validation.json')['known_case_checks'])
    save(OUT/'delivery_validation.json',{'observed_at':now(),'scope':'Saved raw weather, all derived rows, source-bound scene mapping and summary windows',
      'independent_verifier':ref(pathlib.Path(__file__)),'passed_for_current_scope':True,'source_assets_verified':sources,
      'unique_grid_hours':len(raw),'height_rows':len(bykey),'summary_conditions':len(result['summary']),'scene_count':len(mapping),
      'tower_entries':tower_entries,'unique_source_towers':len(physical),'ground_elevation_unknown_entries':ground_unknown,
      'density_OOD_grid_hours':len(outliers),'density_unknown_height_rows':unknown,'max_original_vs_ideal_density_relative_difference':max(rho_deltas),
      'scenes':scene_checks,'weather_rejection_reasons':reasons_output,
      'high_accuracy_local_weather_validated':False,'optimizer_comparison':False,'no_training_or_paid_actions':True,
      'limitations':['Two fixed example weeks, not annual climate statistics','Hourly0.25degree background not tower-local wind',
        'No issue-time forecast or calibrated probabilities','CMU density envelope check is not observed energy error',
        'Null vertical wind, height gust and16 saturated-lapse density rows preserved','Known-case numerical agreement does not establish field accuracy']})
    print(json.dumps({'passed':True,'rows':len(bykey),'grid_hours':len(raw),'scene_count':6,'tower_entries':tower_entries,'unique_source_towers':len(physical),'density_unknown_rows':unknown},ensure_ascii=False))

if __name__=='__main__':main()
