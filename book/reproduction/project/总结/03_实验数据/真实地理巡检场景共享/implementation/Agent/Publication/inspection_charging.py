"""Residual-dependent charge work; explicit sensitivity curves, NOT TB60 calibration."""
import math


def residual_model(cfg):
    mode = cfg.get('charging_mode', 'legacy_fixed')
    if mode not in ('legacy_fixed', 'residual_piecewise'):
        raise ValueError('unknown charging mode')
    return mode == 'residual_piecewise'


def profile(tail_minutes=20, cooling_seconds=0, queue_policy='highest_soc'):
    if tail_minutes <= 0 or cooling_seconds < 0:
        raise ValueError('invalid charging sensitivity parameters')
    return dict(charging_mode='residual_piecewise',
                charge_soc_knots=[0., .2, .9, 1.],
                charge_time_knots_seconds=[0., 1800*.2/.7, 1800*.9/.7, 1800*.9/.7+60*tail_minutes],
                cooling_after_recovery_seconds=float(cooling_seconds),
                charging_queue_policy=queue_policy,
                charging_evidence='20%-90%=1800s at220V:DJI; linear interpolation, tail and cooling are sensitivity assumptions; normalized energy approximates SOC',
                charging_model_version='residual_piecewise_1')


def cumulative(cfg, q):
    xs, ys = cfg['charge_soc_knots'], cfg['charge_time_knots_seconds']
    if not (len(xs) == len(ys) >= 2 and xs[0] == 0 and xs[-1] == 1 and ys[0] == 0):
        raise ValueError('invalid charge curve endpoints')
    if any(b <= a for a, b in zip(xs, xs[1:])) or any(b <= a for a,b in zip(ys,ys[1:])):
        raise ValueError('charge curve must increase strictly')
    if not math.isfinite(q) or not -1e-9 <= q <= 1+1e-9:
        raise ValueError('SOC out of range')
    q = min(1., max(0., q))
    for a,b,t,u in zip(xs,xs[1:],ys,ys[1:]):
        if q <= b:
            return t+(u-t)*(q-a)/(b-a)
    raise AssertionError('curve interpolation')


def charge_work(cfg, energy):
    if not math.isfinite(energy) or not 0 <= energy <= 1+1e-9:
        raise ValueError('invalid flight energy')
    if not residual_model(cfg):
        return cfg['full_charge_seconds']
    return cumulative(cfg,1.)-cumulative(cfg,1.-min(1.,energy))


def remaining_soc(cfg, work):
    target=cumulative(cfg,1.)-work
    xs,ys=cfg['charge_soc_knots'],cfg['charge_time_knots_seconds']
    if target < -1e-7 or target > ys[-1]+1e-7:
        raise ValueError('invalid remaining charge work')
    for a,b,t,u in zip(xs,xs[1:],ys,ys[1:]):
        if target <= u+1e-8:
            return min(1.,max(0.,a+(b-a)*(target-t)/(u-t)))
    return 1.


def cooling(cfg):
    return cfg.get('cooling_after_recovery_seconds',0.) if residual_model(cfg) else 0.


def validate_priority(scene, events):
    """Independent chronological work replay; does not repair schedules."""
    cfg=scene['config']
    if not residual_model(cfg):return
    policy=cfg.get('charging_queue_policy')
    if policy not in ('highest_soc','controlled_insertion'):
        raise ValueError('explicit charger insertion policy required')
    from Agent.Publication.reference_inspection import flight_cost
    for v in range(1,cfg['vehicles']+1):
        es=[e for e in events if e['vehicle']==v]
        work={p:0. for p in range(cfg['battery_pairs_per_vehicle'])}
        available={p:0. for p in work};busy=set();points={}
        def at(t,typ,e):points.setdefault(t,[]).append((typ,e))
        for e in es:
            if e['kind']=='prepare':at(e['start'],'consume',e)
            if e['kind']=='recover':
                f=next(f for f in es if f['kind']=='flight' and f['pair']==e['pair'] and abs(f['end']-e['start'])<1e-7)
                c=flight_cost(scene,f['recipe'],f['launch'],f['recovery'],f['uav'])
                at(e['end'],'return',dict(e,energy=c['energy']))
                at(e['end']+cooling(cfg),'thermal',e)
            if e['kind']=='charge':at(e['start'],'charge_start',e);at(e['end'],'charge_end',e)
        active=None;last=0.
        for t,ops in sorted(points.items()):
            if active is not None and t>last:
                if active in busy or available[active]>last+1e-7:raise ValueError('charging unavailable pack')
                if policy=='highest_soc':
                    eligible=[p for p in work if p not in busy and available[p]<=last+1e-7 and work[p]>1e-7]
                    if eligible and work[active]>min(work[p] for p in eligible)+1e-5:
                        raise ValueError('charger priority violation')
                work[active]-=t-last
                if work[active]<-1e-5:raise ValueError('charge beyond full')
                work[active]=max(0.,work[active])
            # Simultaneous completion/recovery precedes a new operation.
            order={'charge_end':0,'return':1,'thermal':2,'consume':3,'charge_start':4}
            for typ,e in sorted(ops,key=lambda x:order[x[0]]):
                p=e['pair']
                if typ=='charge_end':
                    if active!=p:raise ValueError('charger end mismatch')
                    active=None
                elif typ=='return':
                    if p not in busy:raise ValueError('return without flight')
                    busy.remove(p);work[p]=charge_work(cfg,e['energy']);available[p]=t+cooling(cfg)
                elif typ=='consume':
                    if p in busy or work[p]>1e-5 or active==p:raise ValueError('non-full battery departure')
                    busy.add(p)
                elif typ=='charge_start':
                    if active is not None or p in busy or available[p]>t+1e-7 or work[p]<=1e-9:raise ValueError('invalid charge start')
                    active=p
            last=t
        if busy or active is not None:raise ValueError('unfinished battery operation')
