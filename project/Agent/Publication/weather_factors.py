"""Traceable regional weather-to-flight inputs, not operational flight clearance.

Frame: east/north/up. Meteorological angles describe where wind comes FROM.
No local gust/vertical wind, battery-temperature response or uncertainty law is
invented when data are absent. Source functions are loaded without modification.
"""
import ast,math,pathlib
import numpy as np

ROOT=pathlib.Path(__file__).resolve().parents[2]
CMU=ROOT/'总结/03_实验数据/电池能耗公开机型验证/sources/cmu2021/MetarHandler.py'

def _original_density_functions():
    tree=ast.parse(CMU.read_text('utf8'))
    nodes=[x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name in {'tetens','density'}]
    assert len(nodes)==2
    ns={'math':math};exec(compile(ast.Module(body=nodes,type_ignores=[]),str(CMU),'exec'),ns)
    return ns['tetens'],ns['density']

TETENS,CMU_DENSITY=_original_density_functions()
M100_PROFILE={'aircraft':'DJI Matrice100 experimental reference; not M300',
    'min_air_temperature_C':-10.0,'max_air_temperature_C':40.0,'max_wind_speed_ms':10.0,
    'max_reported_hour_precipitation_mm':0.0,'min_charge_ambient_C':0.0,'max_charge_ambient_C':40.0,
    'sources':{'wind':'DJI M100 User Manual printedp59/PDFp61','temperature':'printedp60/PDFp62',
      'charge':'printedp61/PDFp63','precipitation':'M100 Safety Guidelines printedp12/PDFp14: no rain/snow'},
    'gust_use':'Separate conservative development screen using10m gust and same10m/s bound; manufacturer does not specify averaging period or gust quantile.',
    'screen_pass_is_operational_clearance':False}

def wind_uv(speed_ms,from_degrees):
    speed=float(speed_ms);angle=float(from_degrees)
    if not math.isfinite(speed) or speed<0 or not math.isfinite(angle):raise ValueError('Invalid wind')
    a=math.radians(angle)
    return np.array([-speed*math.sin(a),-speed*math.cos(a)])

def height_wind_uv(u10,v10,u100,v100,height_agl_m):
    h=float(height_agl_m)
    if not 10<=h<=100:raise ValueError('No supported extrapolation beyond10–100m AGL')
    alpha=math.log(h/10)/math.log(10)
    return (1-alpha)*np.array([u10,v10])+alpha*np.array([u100,v100])

def air_relative_enu(ground_velocity_enu,wind_velocity_enu):
    g=np.asarray(ground_velocity_enu,dtype=float);w=np.asarray(wind_velocity_enu,dtype=float)
    if g.shape!=(3,) or w.shape!=(3,) or not np.isfinite(g).all() or not np.isfinite(w).all():
        raise ValueError('Need explicit finite ground and wind ENU vectors; vertical wind may not silently default to zero')
    return g-w

def density_surface(temp_C,dew_C,pressure_hPa):
    if not all(math.isfinite(float(x)) for x in [temp_C,dew_C,pressure_hPa]):raise ValueError('Missing thermodynamic input')
    if temp_C<=-237.3 or dew_C<=-237.3 or pressure_hPa<=0:raise ValueError('Outside density formula domain')
    # Rounded reported dew points can slightly exceed rounded temperature.
    if dew_C>temp_C+0.2:raise ValueError('Dew point materially exceeds air temperature')
    return float(CMU_DENSITY(float(temp_C),float(dew_C),float(pressure_hPa)))

def density_height_proxy(temp_C,dew_C,pressure_hPa,height_agl_m):
    """Declared hydrostatic/lapse approximation above model-grid terrain.

    2m weather is not a measured vertical sounding. Surface-pressure hydrostatic
    integration uses constant virtual temperature; vapor mixing ratio is fixed.
    """
    h=float(height_agl_m);rho0=density_surface(temp_C,dew_C,pressure_hPa)
    tv=pressure_hPa*100/(287.05*rho0)
    p=pressure_hPa*math.exp(-9.80665*h/(287.05*tv))
    t=temp_C-0.0065*(h-2)
    vapor=TETENS(dew_C)*p/pressure_hPa
    a=math.log10(vapor/611);dew=237.3*a/(7.5-a)
    saturated=dew>t+0.2
    return {'temperature_proxy_C':t,'pressure_proxy_hPa':p,'air_density_proxy_kg_m3':None if saturated else density_surface(t,dew,p),
      'status':'unsaturated_lapse_assumption_invalid' if saturated else 'approximation_only',
      'vertical_model':'hydrostatic constant Tv; lapse0.0065K/m; fixed vapor mixing ratio; above ERA5 terrain, not local tower survey'}

def weather_screen(temp_C,wind_speed_ms,precipitation_mm,weather_code,gust10_ms=None,daylight=None):
    """Hourly retrospective screen. Unknowns are not silently treated as pass."""
    fields=[temp_C,wind_speed_ms,precipitation_mm,weather_code]
    if any(x is None or not math.isfinite(float(x)) for x in fields):return {'base_screen_pass':None,'gust_screen_pass':None,'unknown':'missing core input'}
    reasons=[]
    if not -10<=temp_C<=40:reasons.append('temperature')
    if wind_speed_ms>10:reasons.append('mean_wind_at_height')
    if precipitation_mm>0:reasons.append('reported_precipitation')
    if int(weather_code) not in [0,1,2,3]:reasons.append('reported_weather_hazard')
    gust_unknown=gust10_ms is None or not math.isfinite(float(gust10_ms))
    gust_bad=False if gust_unknown else gust10_ms>10
    base=not reasons
    return {'base_screen_pass':base,'gust_screen_pass':False if not base or gust_bad else None if gust_unknown else True,
      'gust10_exceeds_reference':None if gust_unknown else gust_bad,'base_reasons':reasons,
      'daylight_screen_pass':None if daylight is None else base and bool(daylight),
      'charging_ambient_screen_pass':0<=temp_C<=40,'flight_clearance':None,
      'unresolved_hazards':['tower-scale gusts and turbulence','vertical wind','icing beyond weather-code indicators','visibility and smog','lightning absent from ERA5 weather-code estimate']}

def contiguous_windows(records,field='base_screen_pass'):
    """Return runs of consecutive *screened hourly proxy* bins, never flyability guarantees."""
    import datetime
    out=[];start=None;prev=None
    for x in records:
        t=datetime.datetime.fromisoformat(x['time_utc'].replace('Z','+00:00'))
        ok=x[field] is True
        if start is not None and (not ok or (t-prev).total_seconds()!=3600):
            out.append({'first_hour_endpoint':start.isoformat(),'last_hour_endpoint':prev.isoformat(),'screened_hours':int((prev-start).total_seconds()/3600)+1});start=None
        if ok and start is None:start=t
        if ok:prev=t
    if start is not None:out.append({'first_hour_endpoint':start.isoformat(),'last_hour_endpoint':prev.isoformat(),'screened_hours':int((prev-start).total_seconds()/3600)+1})
    return out
