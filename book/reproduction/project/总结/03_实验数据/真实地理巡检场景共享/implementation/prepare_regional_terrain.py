"""Bounded cached public DEM acquisition for the three authorized regions."""
import datetime, hashlib, json, math, sys, time, urllib.parse, urllib.request
from pathlib import Path
import numpy as np
import requests as http_requests
from pyproj import Transformer

ROOT=Path(__file__).resolve().parents[1]
PACK=ROOT/'总结/03_实验数据/真实地理巡检场景共享'
NAMES=['rommerskirchen','brauweiler','villevaude']
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def write(p,d):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(d,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n',encoding='utf-8')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    last=0.;deadline=time.monotonic()+1800;calls=0;session=http_requests.Session()
    for name in NAMES:
        s=read(PACK/'scenes'/f'{name}.json');out=PACK/'sources/regional_dem'/name
        tr=Transformer.from_crs(s['coordinate_reference']['projected'],4326,always_xy=True)
        xy=np.array([[n['easting_m'],n['northing_m']] for n in s['roads']['nodes']+s['towers']])
        lo=np.floor((xy.min(0)-150)/250)*250;hi=np.ceil((xy.max(0)+150)/250)*250
        locations=[tr.transform(float(x),float(y)) for x in np.arange(lo[0],hi[0]+1,250) for y in np.arange(lo[1],hi[1]+1,250)]
        locations += [(t['longitude'],t['latitude']) for t in s['towers']]
        locations=sorted({(round(lon,9),round(lat,9)) for lon,lat in locations})
        cached={};requests=[]
        for p in sorted(out.glob('*.json')):
            entry=read(p)
            if 'response' not in entry:continue
            for r in entry['response']['results']:cached[(round(r['location']['lng'],9),round(r['location']['lat'],9))]=r
            requests.append({'path':p.relative_to(PACK).as_posix(),'sha256':sha(p),'url':entry['request_url'],'retrieved_at':entry['retrieved_at']})
        needed=[v for v in locations if v not in cached];rows=[]
        # Diagnostic: 100-point GET/POST responses stall; a 20-point GET completed.
        for i in range(0,len(needed),20):
            batch=needed[i:i+20]
            query=urllib.parse.urlencode({'locations':'|'.join(f'{lat:.9f},{lon:.9f}' for lon,lat in batch),'interpolation':'bilinear'})
            url='https://api.opentopodata.org/v1/srtm90m?'+query
            key=hashlib.sha256(url.encode()).hexdigest()[:18];p=out/f'{key}.json'
            if p.exists():entry=read(p);assert entry['request_url']==url
            else:
                if calls>=450 or time.monotonic()>deadline:raise RuntimeError('Bounded terrain acquisition exhausted')
                time.sleep(max(0,1.1-(time.monotonic()-last)))
                last=time.monotonic();calls+=1
                payload={'locations':'|'.join(f'{lat:.9f},{lon:.9f}' for lon,lat in batch),'interpolation':'bilinear'}
                # The first run received headers but timed out reading a response body.
                # Preserve successful batches and allow a bounded longer socket read.
                try:
                    # A fresh Connection: close request succeeded for the failed
                    # keep-alive batch; do not reuse the stalled connection.
                    res=http_requests.get('https://api.opentopodata.org/v1/srtm90m',params=payload,headers={'Connection':'close'},timeout=(12,25))
                    res.raise_for_status();body=res.json()
                except Exception as exc:
                    with (PACK/'sources/regional_dem/acquisition_failures.jsonl').open('a',encoding='utf-8') as f:
                        f.write(json.dumps({'at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'url':url,'error':repr(exc)},ensure_ascii=False)+'\n')
                    raise
                if body.get('status')!='OK':raise RuntimeError(str(body)[:500])
                assert len(body['results'])==len(batch) and all(x['elevation'] is not None for x in body['results'])
                entry={'request_url':url,'request_method':'GET','retrieved_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'response':body}
                write(p,entry)
            results=entry['response']['results']
            for (lon,lat),v in zip(batch,results):
                assert abs(lon-v['location']['lng'])<1e-8 and abs(lat-v['location']['lat'])<1e-8
            for v in results:cached[(round(v['location']['lng'],9),round(v['location']['lat'],9))]=v
            requests.append({'path':p.relative_to(PACK).as_posix(),'sha256':sha(p),'url':url,'retrieved_at':entry['retrieved_at']})
            if i%200==0:print(name,len(cached),'/',len(locations),flush=True)
        rows=[cached[v] for v in locations]
        target=out/'terrain.json'
        write(target,{'dataset':'SRTM90m','dataset_version':'OpenTopoData SRTM version3','source_resolution_m':90,'grid_spacing_m':250,'interpolation':'API bilinear; local projected piecewise-linear','bounds_utm_m':[lo.tolist(),hi.tolist()],'source_url':'https://www.opentopodata.org/datasets/srtm/','vertical_datum':'source_native_elevation; not independently surveyed','results':rows,'requests':requests})
        print('COMPLETE',name,len(rows),sha(target),flush=True)
def from_rasters():
    """Offline bilinear sampling from retained CGIAR-CSI SRTM90 GeoTIFFs.

    The incomplete OpenTopoData responses remain as acquisition history only.
    They are not mixed with the authoritative v4.1 raster values.
    """
    import zipfile
    from rasterio.io import MemoryFile
    from scipy.ndimage import map_coordinates
    for name in NAMES:
        s=read(PACK/'scenes'/f'{name}.json');out=PACK/'sources/regional_dem'/name
        tile='srtm_37_03' if name=='villevaude' else 'srtm_38_02'
        zpath=PACK/'sources/regional_dem/rasters'/f'{tile}.zip';meta=read(zpath.with_suffix('.source.json'))
        assert sha(zpath)==meta['sha256']
        with zipfile.ZipFile(zpath) as z:
            tif=next(x for x in z.namelist() if x.lower().endswith('.tif'))
            with MemoryFile(z.read(tif)) as mem:
                with mem.open() as ds:
                    assert ds.crs.to_epsg()==4326
                    a=ds.read(1);transform=ds.transform;nodata=ds.nodata
                    gridmeta=dict(crs=str(ds.crs),transform=list(transform)[:6],width=ds.width,height=ds.height,nodata=nodata)
        tr=Transformer.from_crs(s['coordinate_reference']['projected'],4326,always_xy=True)
        xy=np.array([[n['easting_m'],n['northing_m']] for n in s['roads']['nodes']+s['towers']])
        lo=np.floor((xy.min(0)-150)/250)*250;hi=np.ceil((xy.max(0)+150)/250)*250
        locations=[tr.transform(float(x),float(y)) for x in np.arange(lo[0],hi[0]+1,250) for y in np.arange(lo[1],hi[1]+1,250)]
        locations += [(t['longitude'],t['latitude']) for t in s['towers']]
        locations=sorted(set(locations));lon,lat=np.asarray(locations).T
        col,row=(~transform)*(lon,lat);col-=.5;row-=.5
        assert min(row)>=0 and max(row)<a.shape[0]-1 and min(col)>=0 and max(col)<a.shape[1]-1
        valid=map_coordinates((a!=nodata).astype(float),[row,col],order=1,prefilter=False)
        assert np.min(valid)>.999999,'No silent interpolation across missing DEM pixels'
        heights=map_coordinates(a.astype(float),[row,col],order=1,prefilter=False)
        rows=[dict(dataset='CGIAR-CSI_SRTM90_v4.1',elevation=float(h),location={'lng':float(x),'lat':float(y)}) for (x,y),h in zip(locations,heights)]
        target=out/'terrain.json'
        write(target,dict(dataset='CGIAR-CSI_SRTM90_v4.1',dataset_version='CGIAR-CSI SRTM v4.1 GeoTIFF',source_resolution_m=90,grid_spacing_m=250,
            interpolation='raster pixel-center bilinear; projected piecewise-linear for model assets',bounds_utm_m=[lo.tolist(),hi.tolist()],
            source_url='https://srtm.csi.cgiar.org/srtmdata/',vertical_datum='source GeoTIFF elevation; local datum tie not surveyed',
            raster_metadata=gridmeta,results=rows,requests=[dict(path=zpath.relative_to(PACK).as_posix(),sha256=meta['sha256'],url=meta['url'],retrieved_at=meta['retrieved_at'])],
            acquisition_note='API batches incomplete; retained for failure history only. All active samples use one pinned raster source per scene.'))
        print('COMPLETE',name,len(rows),sha(target),flush=True)

def from_copernicus():
    """Pinned public Copernicus GLO90 2021, delivered by Open-Meteo.

    Source is changed explicitly after repeated incomplete SRTM transport;
    no cached SRTM values are mixed into these scene elevations.
    """
    deadline=time.monotonic()+1500;last_request=0.
    for name in NAMES:
        s=read(PACK/'scenes'/f'{name}.json');out=PACK/'sources/regional_dem'/name
        tr=Transformer.from_crs(s['coordinate_reference']['projected'],4326,always_xy=True)
        xy=np.array([[n['easting_m'],n['northing_m']] for n in s['roads']['nodes']+s['towers']])
        lo=np.floor((xy.min(0)-150)/250)*250;hi=np.ceil((xy.max(0)+150)/250)*250
        locations=[tr.transform(float(x),float(y)) for x in np.arange(lo[0],hi[0]+1,250) for y in np.arange(lo[1],hi[1]+1,250)]
        locations += [(t['longitude'],t['latitude']) for t in s['towers']]
        locations=sorted({(round(x,9),round(y,9)) for x,y in locations});rows=[];refs=[]
        for i in range(0,len(locations),100):
            batch=locations[i:i+100];query=urllib.parse.urlencode(dict(latitude=','.join(f'{y:.9f}' for x,y in batch),longitude=','.join(f'{x:.9f}' for x,y in batch)))
            url='https://api.open-meteo.com/v1/elevation?'+query;p=out/('cop90_'+hashlib.sha256(url.encode()).hexdigest()[:18]+'.json')
            if p.exists():entry=read(p);assert entry['request_url']==url
            else:
                while True:
                    assert time.monotonic()<deadline,'Bounded Copernicus acquisition deadline'
                    time.sleep(max(0,11-(time.monotonic()-last_request)));last_request=time.monotonic()
                    res=http_requests.get(url,timeout=(8,15))
                    if res.status_code==429:
                        with (PACK/'sources/regional_dem/acquisition_failures.jsonl').open('a',encoding='utf-8') as f:
                            f.write(json.dumps(dict(at=datetime.datetime.now(datetime.timezone.utc).isoformat(),url=url,error=res.text,action='respect provider rate limit; wait60s in short intervals'))+'\n')
                        if 'hour' in res.text.lower():raise RuntimeError('Hourly provider quota: stop this API; no automatic minute retries')
                        for _ in range(6):time.sleep(10)
                        continue
                    res.raise_for_status();body=res.json();break
                assert len(body['elevation'])==len(batch) and all(v is not None and math.isfinite(v) for v in body['elevation'])
                entry=dict(request_url=url,retrieved_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),response=body,request_coordinates_lon_lat=batch)
                write(p,entry);time.sleep(.2)
                print('COP90',name,min(i+100,len(locations)),'/',len(locations),flush=True)
            rows.extend(dict(dataset='Copernicus_GLO90_2021',elevation=float(h),location={'lng':x,'lat':y}) for (x,y),h in zip(batch,entry['response']['elevation']))
            refs.append(dict(path=p.relative_to(PACK).as_posix(),sha256=sha(p),url=url,retrieved_at=entry['retrieved_at']))
        write(out/'terrain.json',dict(dataset='Copernicus_GLO90_2021',dataset_version='Copernicus DEM 2021 release GLO-90',source_resolution_m=90,grid_spacing_m=250,
            source_url='https://open-meteo.com/en/docs/elevation-api',doi='https://doi.org/10.5270/ESA-c5d3d65',
            bounds_utm_m=[lo.tolist(),hi.tolist()],interpolation='API elevations; local projected piecewise-linear interpolation',
            vertical_datum='source elevation; local datum tie unverified',terrain_role='DSM elevation proxy; not bare-earth or tower-base survey',
            results=rows,requests=refs,acquisition_note='Active elevations exclusively Copernicus; failed/incomplete SRTM acquisition retained separately.'))
        print('COMPLETE',name,len(rows),flush=True)

def from_glo_cog():
    import rasterio
    from scipy.ndimage import map_coordinates
    name='villevaude';s=read(PACK/'scenes'/f'{name}.json');out=PACK/'sources/regional_dem'/name
    raster=PACK/'sources/regional_dem/rasters/Copernicus_DSM_COG_30_N48_00_E002_00_DEM.tif';meta=read(raster.with_suffix('.source.json'))
    assert sha(raster)==meta['sha256']
    with rasterio.open(raster) as ds:
        assert ds.crs.to_epsg()==4326
        a=ds.read(1);tf=ds.transform;nd=ds.nodata;tags=ds.tags()
    tr=Transformer.from_crs(s['coordinate_reference']['projected'],4326,always_xy=True)
    xy=np.array([[n['easting_m'],n['northing_m']] for n in s['roads']['nodes']+s['towers']])
    lo=np.floor((xy.min(0)-150)/250)*250;hi=np.ceil((xy.max(0)+150)/250)*250
    locations=[tr.transform(float(x),float(y)) for x in np.arange(lo[0],hi[0]+1,250) for y in np.arange(lo[1],hi[1]+1,250)]
    locations += [(t['longitude'],t['latitude']) for t in s['towers']]
    locations=sorted(set(locations));lon,lat=np.asarray(locations).T;col,row=(~tf)*(lon,lat);col-=.5;row-=.5
    assert min(row)>=0 and max(row)<a.shape[0]-1 and min(col)>=0 and max(col)<a.shape[1]-1
    valid=np.isfinite(a)&(a!=nd);assert np.min(map_coordinates(valid.astype(float),[row,col],order=1,prefilter=False))>.999999
    heights=map_coordinates(a.astype(float),[row,col],order=1,prefilter=False)
    rows=[dict(dataset='Copernicus_GLO90_2021',elevation=float(h),location={'lng':x,'lat':y}) for (x,y),h in zip(locations,heights)]
    write(out/'terrain.json',dict(dataset='Copernicus_GLO90_2021',dataset_version='Copernicus DEM 2021 release GLO-90 public COG',source_resolution_m=90,grid_spacing_m=250,
        source_url='https://registry.opendata.aws/copernicus-dem/',doi='https://doi.org/10.5270/ESA-c5d3d65',bounds_utm_m=[lo.tolist(),hi.tolist()],
        interpolation='GeoTIFF pixel-center bilinear, then UTM piecewise linear for model assets',vertical_datum='native Copernicus DSM; local datum tie unverified',
        terrain_role='DSM elevation proxy; not bare-earth or tower-base survey',raster_metadata=dict(transform=list(tf)[:6],shape=list(a.shape),nodata=nd,tags=tags),
        requests=[dict(path=raster.relative_to(PACK).as_posix(),sha256=meta['sha256'],url=meta['url'],retrieved_at=meta['retrieved_at'])],results=rows,
        acquisition_note='Complete active scene uses pinned public COG; partial API samples retained as acquisition history only.'))
    print('COMPLETE',name,len(rows),float(min(heights)),float(max(heights)),flush=True)

if __name__=='__main__':
    if '--copernicus-cog' in sys.argv:from_glo_cog()
    elif '--copernicus' in sys.argv:from_copernicus()
    elif '--raster' in sys.argv:from_rasters()
    else:main()
