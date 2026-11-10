"""Contract5 continuous forecast field, separate from execution weather.

The retained single forecast cell is explicitly a spatially uniform reference.
Time and height interpolation are continuous; rain/daylight/gust retain their
source interval meaning. No future execution record enters this constructor.
"""
from __future__ import annotations
from datetime import datetime, timedelta, timezone
from pathlib import Path
import hashlib
import json
import math
import numpy as np
from Agent.Publication.engineering_indexed_constraints import AffineRow


FIELDS = ('wind_east_ms', 'wind_north_ms', 'density_kg_m3', 'temperature_C')


def utc(value):
    value = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if value.tzinfo is None:
        value = value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc)


class RegionalForecast:
    def __init__(self, times, values, intervals, provenance):
        self.times = np.asarray(times, float)
        self.values = np.asarray(values, float)
        self.intervals, self.provenance = intervals, provenance
        if self.values.shape != (len(times), 3, 4):
            raise ValueError('All time endpoints, heights and continuous fields required')
        if not np.isfinite(self.values).all() or not np.all(np.diff(self.times) > 0):
            raise ValueError('Missing or nonmonotone forecast data')
        if len(intervals) != len(times)-1:
            raise ValueError('One original discrete-field interval required per time cell')

    def cell(self, time, height):
        if time < self.times[0]-1e-9 or time > self.times[-1]+1e-9:
            raise ValueError('No temporal extrapolation')
        if height < 0 or height > 100:
            raise ValueError('Outside declared height domain')
        h = min(len(self.times)-2, int(np.searchsorted(self.times, time, side='right')-1))
        k = 0 if height < 30 else (1 if height < 60 else 2)
        return h, k

    def coefficients(self, h, k):
        lo, hi, lower, upper = [(0.,30.,0,0),(30.,60.,0,1),(60.,100.,1,2)][k]
        dt = self.times[h+1]-self.times[h]
        a = self.values[h,lower]
        b = (self.values[h+1,lower]-a)/dt
        c = (self.values[h,upper]-a)/(hi-lo)
        d = ((self.values[h+1,upper]-self.values[h+1,lower])/(hi-lo)-c)/dt
        return lo, hi, a, b, c, d

    def evaluate(self, time, height):
        h, k = self.cell(time, height)
        lo, hi, a, b, c, d = self.coefficients(h,k)
        x, y = time-self.times[h], height-lo
        return dict(values=dict(zip(FIELDS,(a+b*x+c*y+d*x*y).tolist())),
                    time_derivative=dict(zip(FIELDS,(b+d*y).tolist())),
                    height_derivative=dict(zip(FIELDS,(c+d*x).tolist())),
                    interval=self.intervals[h], cell=[h,k])

    def add_point(self, model, relations, clock, agl, active, name, operation='flight'):
        """Exact bilinear field on one selected time/height cell.

        Cell coefficients are affine indexed lookups. The interpolation and
        both derivative directions are shared expressions. This prevents the
        old omission of time-varying wind in attitude-rate constraints.
        """
        selector = relations.selector(name+'_weather_cell', 3*(len(self.times)-1))
        data = [self.coefficients(h,k) for h in range(len(self.times)-1) for k in range(3)]
        t0 = model.addVar(name+'_time_origin', lb=float(self.times[0]), ub=float(self.times[-2]))
        z0 = model.addVar(name+'_height_origin', lb=0, ub=60)
        coeff = []
        for q in range(4):
            coeff.append([model.addVar(name+f'_coefficient{q}_{f}',
                lb=float(min(row[q+2][f] for row in data)),
                ub=float(max(row[q+2][f] for row in data))) for f in range(4)])
        def rows(category):
            h,k=divmod(category,3);lo,hi,a,b,c,d=data[category]
            ans=[AffineRow('clock',[(1,clock)],float(self.times[h]),float(self.times[h+1])),
                 AffineRow('height',[(1,agl)],lo,hi),
                 AffineRow('time_origin',[(1,t0)],float(self.times[h]),float(self.times[h])),
                 AffineRow('height_origin',[(1,z0)],lo,lo)]
            for q, row in enumerate((a,b,c,d)):
                ans.extend(AffineRow(f'coefficient{q}_{f}',[(1,coeff[q][f])],float(row[f]),float(row[f])) for f in range(4))
            if operation=='flight' and not self.intervals[h]['flight_allowed']:
                ans.append(AffineRow('weather_no_flight',[(1,active)],0,0))
            return ans
        relations.add(selector,rows,[clock,agl,t0,z0,active,*sum(coeff,[])],name=name+'_forecast')
        x,y=clock-t0,agl-z0
        result={f:coeff[0][j]+coeff[1][j]*x+coeff[2][j]*y+coeff[3][j]*x*y for j,f in enumerate(FIELDS)}
        result['dt']={f:coeff[1][j]+coeff[3][j]*y for j,f in enumerate(FIELDS)}
        result['dz']={f:coeff[2][j]+coeff[3][j]*x for j,f in enumerate(FIELDS)}
        result['selector']=selector
        result['coefficient_variables']=dict(origins=[t0,z0],coefficients=coeff)
        return result


def retained_regional_forecast(root):
    root=Path(root)
    folder=root/'总结/03_实验数据/工程化联合求解/day_ahead_pilot'
    raw_path=folder/'forecast_raw.json'; source_path=folder/'forecast_source.json'
    raw=json.loads(raw_path.read_text('utf-8-sig'))
    source=json.loads(source_path.read_text('utf-8-sig'))
    digest=hashlib.sha256(raw_path.read_bytes()).hexdigest()
    if digest != source['sha256'] or source['role'] != 'forecast':
        raise ValueError('Forecast identity/role mismatch')
    # Reuse the date frozen before the pilot data acquisition, not a date
    # selected after viewing forecast/actual outcomes.
    start=utc('2026-07-01T08:00:00Z'); cutoff=utc('2026-06-30T18:00:00Z')
    init=utc('2026-06-30T00:00:00Z')
    if init+timedelta(hours=18)>cutoff:
        raise ValueError('Reference availability later than planning cutoff')
    h=raw['hourly']; lookup={utc(t):i for i,t in enumerate(h['time'])}
    values=[]; intervals=[]
    for k in range(9):
        i=lookup[start+timedelta(hours=k)]
        fields=['temperature_2m','surface_pressure','precipitation','is_day','wind_speed_10m',
                'wind_direction_10m','wind_speed_100m','wind_direction_100m','wind_gusts_10m']
        if any(h[f][i] is None for f in fields):
            raise ValueError('Missing required forecast value')
        def vector(speed,direction):
            r=math.radians(direction);return np.array([-speed*math.sin(r),-speed*math.cos(r)])
        w10=vector(h['wind_speed_10m'][i],h['wind_direction_10m'][i])
        w100=vector(h['wind_speed_100m'][i],h['wind_direction_100m'][i])
        layers=[]
        for z in (30.,60.,100.):
            w=w10+(z-10)/90*(w100-w10)
            temp=h['temperature_2m'][i]-.0065*(z-2)
            pressure=h['surface_pressure'][i]*100*math.exp(-9.80665*z/(287.05*(h['temperature_2m'][i]+273.15)))
            rho=pressure/(287.05*(temp+273.15))
            layers.append([*w,rho,temp])
        values.append(layers)
        if k<8:
            intervals.append(dict(start_s=k*3600,end_s=(k+1)*3600,
                source_time=h['time'][i],precipitation_mm=h['precipitation'][i],
                gust10_mps=h['wind_gusts_10m'][i],is_day=h['is_day'][i],
                flight_allowed=bool(h['precipitation'][i]<=0 and h['wind_gusts_10m'][i]<=10 and h['is_day'][i]==1),
                semantics='retain source hourly discrete values; continuous wind/temp checked separately'))
    provenance=dict(source_path=str(raw_path.relative_to(root)),sha256=digest,url=source['url'],
        start_utc=start.isoformat(),planning_cutoff_utc=cutoff.isoformat(),model_initialization_utc=init.isoformat(),
        publication_time_utc=None,availability_assumption_hours=18,causality='conditional_on_declared_availability_assumption',
        spatial_interpolation='explicit_single_cell_uniform_reference',
        returned_latitude=raw['latitude'],returned_longitude=raw['longitude'],
        field_calibrated=False,uses_execution_truth=False,grid_height_downscaling=False)
    return RegionalForecast(np.arange(9)*3600,values,intervals,provenance)
