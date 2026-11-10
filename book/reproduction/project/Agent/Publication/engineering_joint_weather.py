"""Retained deterministic weather tables, without reusing M100 pass flags."""
import datetime


def hourly_table(scene,start_utc,horizon_seconds):
    start=datetime.datetime.fromisoformat(start_utc)
    assert start.minute==0 and start.second==0
    grids={r['grid_id'] for r in scene['weather']['records']}
    if len(grids)!=1:
        raise ValueError('Multiple weather cells require the declared nearest-cell spatial adapter')
    lookup={(r['time_utc'],r['height_agl_m']):r for r in scene['weather']['records']}
    table=[]
    for hour in range(int(horizon_seconds/3600)):
        stamp=(start+datetime.timedelta(hours=hour)).strftime('%Y-%m-%dT%H:00Z')
        rows=[lookup.get((stamp,h)) for h in [30,60,100]]
        if any(r is None for r in rows):
            raise ValueError('Missing retained weather hour/layer: '+stamp)
        table.append(dict(hour=hour,start_s=hour*3600,end_s=(hour+1)*3600,
                          source_time=stamp,rows=rows))
    return table


def sampled_weather(j,block,s,key,agl,low,high):
    """Necessary point constraints; continuous hour boundaries need replay.

Every coefficient is chosen by the current optimized clock. Both boundary
traces remain possible in this relaxation; acceptance must check the declared
left-hour convention and all crossings. This is not a continuous certificate.
"""
    i=block['task'];role=block['role'];k=block['segment']
    if role=='in':
        clock=j.t[i]-j.duration_in[i]+sum(j.segments[f'in_{i}_{r}']['T'] for r in range(k))
    elif role=='cap':clock=j.t[i]
    else:clock=j.t[i]+j.duration_cap[i]+sum(j.segments[f'rec_{i}_{r}']['T'] for r in range(k))
    clock=clock+s*block['T'];warm=max(0,min(len(j.weather_table)-1,int(j.val(clock)//3600)))
    selectors=[]
    for h,row in enumerate(j.weather_table):
        select=j.binary(key+f'_weather_hour{h}',h==warm,'weather_auxiliary');selectors.append(select)
        on=j.and2(select,block['active'],key+f'_weather_active{h}')
        j.gate(clock,on,row['start_s'],row['end_s'],name=key+f'_weather_clock{h}',family='weather_power')
        # Invalid layers cannot acquire positive interpolation weight.
        for r,weight in zip(row['rows'],[1-low,low-high,high]):
            if not r['reference_flight_allowed'] or r['air_density_proxy_kg_m3'] is None:
                j.gate(weight,on,0,0,name=key+f'_invalid_height{h}_{r["height_agl_m"]}',family='weather_power')
    j.row(sum(selectors),1,1,key+'_one_weather_hour','weather_power')
    def coefficient(layer,field):
        return sum(selectors[h]*float(row['rows'][layer][field] or 0.) for h,row in enumerate(j.weather_table))
    winds=[[coefficient(k,f) for f in ['wind_east_ms','wind_north_ms']]+[0.] for k in range(3)]
    density=[coefficient(k,'air_density_proxy_kg_m3') for k in range(3)]
    return winds,density,clock
