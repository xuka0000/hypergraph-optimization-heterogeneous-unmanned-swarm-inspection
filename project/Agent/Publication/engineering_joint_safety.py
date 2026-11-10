"""Globally valid point-witness constraints for the shared trajectory model.

A witness selects a normalized curve/ray parameter, never a frozen waypoint
or a fixed time ordering. Adding these rows is necessary, not sufficient,
for continuous feasibility; an independent interval oracle must close every
remaining interval before a complete-model certificate can be issued.
"""
import json
import math
import numpy as np
from pyscipopt import sqrt


def positive(x):return (x+abs(x))/2
def clip01(x):return (x+abs(x))/2-positive(x-1)
def bezier(controls,s):
    level=[list(row) for row in controls]
    for _ in range(len(level)-1):
        level=[[(1-s)*a+s*b for a,b in zip(x,y)] for x,y in zip(level,level[1:])]
    return level[0]


def block_start(j,b):
    i,k,role=b['task'],b['segment'],b['role']
    if role=='cap':return j.t[i]
    if role=='in':return j.t[i]-j.duration_in[i]+sum(j.segments[f'in_{i}_{q}']['T'] for q in range(k))
    return j.t[i]+j.duration_cap[i]+sum(j.segments[f'rec_{i}_{q}']['T'] for q in range(k))


def cylinder_signed_distance(point,cylinder):
    radial=sqrt(sum((point[q]-cylinder['center_xy_m'][q])**2 for q in range(2)))-cylinder['radius_m']
    low=cylinder['base_elevation_m']-point[2]
    high=point[2]-cylinder['base_elevation_m']-cylinder['height_m']
    vertical=(low+high+abs(low-high))/2
    largest=(radial+vertical+abs(radial-vertical))/2
    return sqrt(positive(radial)**2+positive(vertical)**2)+(largest-abs(largest))/2


def wire_point(wire,t):
    a,b=np.asarray(wire['endpoint_xyz_m'],float)
    q=(1-t)*a+t*b;q[2]-=4*wire['sag_m']*t*(1-t)
    return q


def witness_name(j,prefix):
    # The runner can discard expression rows after exporting incidence. Keep
    # a separate monotonically increasing name counter across refinements.
    number=getattr(j,'_witness_serial',len(j.rows))
    j._witness_serial=number+1
    return prefix+str(number)


def point_witness(j,witness):
    """Add an exact source-obstacle point constraint at normalized parameters."""
    b=j.segments[witness['block']];s=float(witness['curve_parameter'])
    if not 0<=s<=1:raise ValueError('Witness outside the source curve')
    point=[x+j.origin[q] for q,x in enumerate(bezier(b['C'],s))]
    kind=witness['kind'];name=witness_name(j,'witness_')
    if kind.startswith('visibility_'):
        if b['role']!='cap':raise ValueError('Visibility needs an active capture')
        ray=float(witness['ray_parameter']);target=j.targets[b['task']]['target']['surface_samples_utm_m'][witness['surface_sample']]
        if not 0<=ray<=1:raise ValueError('Sight parameter outside the line segment')
        point=[(1-ray)*p+ray*q for p,q in zip(point,target)];clearance=0.
    else:
        air=j.cfg['aircraft'];clearance=sum(air[k] for k in ['body_radius_m','obstacle_clearance_m','tracking_error_m','map_error_m'])
    if kind.endswith('cylinder'):
        obs=j.scene['obstacle_model']['tower_cylinders'][witness['obstacle_index']]
        j.gate(cylinder_signed_distance(point,obs),b['active'],lo=clearance,name=name,family='continuous_obstacle_witness')
    elif kind.endswith('wire'):
        obs=j.scene['obstacle_model']['conductor_curves'][witness['obstacle_index']]
        parameter=float(witness['wire_parameter'])
        if not 0<=parameter<=1:raise ValueError('Wire parameter outside its source curve')
        target=wire_point(obs,parameter)
        j.gate(sum((p-q)**2 for p,q in zip(point,target)),b['active'],lo=(clearance+obs['radius_m'])**2,name=name,family='continuous_obstacle_witness')
    elif kind=='physics':
        data=j.physics(b,s,name)
        if b['role']=='cap':j.optics(b,s,data,name)
    else:raise ValueError('Unknown witness kind '+kind)


def pair_witness(j,key_a,key_b,fraction):
    """Couple the same absolute time after any retiming of either segment."""
    if key_a==key_b or not 0<=fraction<=1:raise ValueError('Invalid segment pair/witness')
    a,b=j.segments[key_a],j.segments[key_b];ta,tb=block_start(j,a),block_start(j,b)
    left=(ta+tb+abs(ta-tb))/2
    ea,eb=ta+a['T'],tb+b['T'];right=(ea+eb-abs(ea-eb))/2
    name=witness_name(j,'pair_witness_');width=right-left
    overlap=j.binary(name+'_overlap',j.val(width)>=0)
    j.gate(width,overlap,lo=0,name=name+'_overlap_yes',family='continuous_pairwise_airspace')
    j.gate(width,1-overlap,hi=0,name=name+'_overlap_no',family='continuous_pairwise_airspace')
    active=j.and2(j.and2(a['active'],b['active'],name+'_both'),overlap,name+'_on')
    time=left+float(fraction)*positive(width)
    pa=bezier(a['C'],clip01((time-ta)/a['T']))
    pb=bezier(b['C'],clip01((time-tb)/b['T']))
    required=2*j.cfg['aircraft']['body_radius_m']+j.cfg['aircraft']['inter_uav_clearance_m']
    # Segments of the same flight meet at common boundaries; those are not
    # two aircraft. Exclude this case using the actual task-to-flight labels.
    same=sum(j.x[a['task'],l]*j.x[b['task'],l] for l in range(j.n))
    j.gate((1-same)*(sum((x-y)**2 for x,y in zip(pa,pb))-required**2),active,lo=0,name=name+'_distance',family='continuous_pairwise_airspace')


def apply_witnesses(j,rows):
    applied=getattr(j,'_applied_witnesses',set())
    j._applied_witnesses=applied;added=[]
    for row in rows:
        signature=json.dumps(row,sort_keys=True,allow_nan=False,separators=(',',':'))
        if signature in applied:continue
        if row['kind']=='pair':pair_witness(j,row['block_a'],row['block_b'],float(row['overlap_fraction']))
        else:point_witness(j,row)
        applied.add(signature);added.append(row)
    return added


def witnesses_from_replay(flights,physical):
    """Map exact exported segment identities back to necessary model rows.

    Interval inconclusiveness is NOT a demonstrated violation. Its endpoint
    and midpoint rows are valid necessary constraints, but may fail to remove
    a candidate. Deduplication then returns unfinished, not a false pass.
    Geometry, energy and weather-jump failures require other oracles and are
    deliberately not converted into guessed exclusions.
    """
    requests=[];unmapped=[]
    for row in physical.get('rows',[]):
        flight=flights[row['slot']]
        for failure in row.get('failures',[]):
            index=failure.get('segment');mapped=False
            if index is not None and 'start_s' in failure and 'end_s' in failure:
                segment=flight['timeline'][index];key=segment.get('model_segment_key')
                duration=segment['end_s']-segment['start_s']
                if key and duration>0 and failure.get('reason')=='interval_failed_or_unresolved':
                    lo=(failure['start_s']-segment['start_s'])/duration
                    hi=(failure['end_s']-segment['start_s'])/duration
                    if all(math.isfinite(t) and -1e-8<=t<=1+1e-8 for t in (lo,hi)):
                        for t in (lo,(lo+hi)/2,hi):
                            requests.append(dict(kind='physics',block=key,curve_parameter=min(1.,max(0.,float(t)))))
                        mapped=True
            if not mapped:unmapped.append(dict(slot=row['slot'],**failure))
    for row in physical.get('pairwise_separation',[]):
        if row['passed']:continue
        keys=row.get('model_segment_keys');window=row.get('overlap_s');violation=row.get('violation') or {}
        if keys and all(keys) and window and window[1]>window[0]:
            time=violation.get('time_s')
            if time is None and violation.get('interval_s'):time=sum(violation['interval_s'])/2
            if time is not None and math.isfinite(time):
                fraction=(time-window[0])/(window[1]-window[0])
                if -1e-8<=fraction<=1+1e-8:
                    requests.append(dict(kind='pair',block_a=keys[0],block_b=keys[1],overlap_fraction=min(1.,max(0.,float(fraction)))))
                    continue
        unmapped.append(dict(reason='pair_witness_unavailable',pair=row))
    unique={json.dumps(w,sort_keys=True,allow_nan=False):w for w in requests}
    return dict(witnesses=list(unique.values()),unmapped_failures=unmapped,
                scope='necessary point constraints; no independent acceptance implied')
