"""Independent candidate acceptance for the unified regional transcription.

Every continuous family is checked before a candidate may count as feasible.
Unresolved enclosures and missing physics are rejected with explicit requests.
This is numerical engineering validation, not formal interval certification.
"""
import math
import numpy as np
from Agent.Publication.engineering_regional_geometry import RegionalGeometry,evaluate,restrict
from Agent.Publication.engineering_regional_curves import control_points


def polynomial_range(c):
    c=np.asarray(c,float);n=len(c)-1;power=np.zeros(n+1)
    for i in range(n+1):
        for k in range(i,n+1):power[k]+=c[i]*math.comb(n,i)*math.comb(n-i,k-i)*(-1)**(k-i)
    roots=np.polynomial.Polynomial(power).deriv().roots()
    s=[0.,1.]+[float(x.real) for x in roots if abs(x.imag)<1e-9 and 0<x.real<1]
    values=[float(evaluate(c[:,None],x)[0]) for x in s]
    return min(values),max(values),s[int(np.argmin(values))],s[int(np.argmax(values))]


class RegionalAcceptance:
    def __init__(self,model,energy_tolerance_Wh=1e-4,max_depth=22):
        self.j=model;self.geometry=RegionalGeometry(model.safety,model.enclosure,max_depth=max_depth)
        self.energy_tolerance=energy_tolerance_Wh;self.max_depth=max_depth;self.calls=0
        self.physics_cache={}

    def check(self,solution):
        self.calls+=1;j=self.j;m=j.path.m
        def resource_checkpoint():
            guard=getattr(j,'resource_ok',None)
            if guard is not None and not guard():
                raise MemoryError(str(getattr(j,'resource_stop','Acceptance resource reserve')))
        resource_checkpoint()
        def value(x):return float(x) if isinstance(x,(int,float,np.number)) else float(m.getSolVal(solution,x))
        def array(x):return np.array([[value(y) for y in row] for row in x])
        requests=[];active=[];vehicles=[];energies={};unresolved=[]
        # Per-candidate caches only. Names/phase IDs are attached after lookup,
        # so rejected copies still materialize their own indexed witnesses.
        vehicle_cache={};vehicle_pair_cache={};vehicle_pair_checks=0
        # Indexed exact relations own all discrete/resource algebra. Do not
        # evaluate trajectory objects whose selected relation is invalid.
        if next(j.path.rel.violations(solution),None) is not None:return dict(passed=False,indexed_relations_invalid=True,requests=[],unresolved=[])
        road_points=None
        if getattr(j.vehicles,'deferred_dynamics',False):
            road_points=j.vehicles.check_required_points(value)
            if not road_points['passed']:return dict(passed=False,requests=road_points['requests'],unresolved=[],original_road_points_checked=road_points['points_checked'])
        for seg in j.curves.segments:
            if value(seg.active)<.5:continue
            identity=(seg.task,seg.index)
            if identity not in j.physics.current_orders and not getattr(j,'oracle_first',False):
                requests.append(dict(kind='materialize',task=seg.task,index=seg.index,order=5))
        if requests:return dict(passed=False,requests=requests,unresolved=[])
        for seg in j.curves.segments:
            if value(seg.active)<.5:continue
            if len(active)%32==0:resource_checkpoint()
            identity=(seg.task,seg.index)
            cp=seg.controls();c=array(cp['C'])+j.curves.origin;ac=array(cp['A']);T=value(seg.duration);start=value(seg.clock)
            if T<=0:return dict(passed=False,requests=[],unresolved=[dict(reason='nonpositive_active_segment',task=seg.task,index=seg.index)])
            task=j.curves.owner(seg);flight=int(round(value(task.flight.value)))-1
            target=j.curves.corrected_targets[seg.task] if seg.role=='capture' else None
            identity_cache=(seg.task,seg.index,c.tobytes(),ac.tobytes(),T,start,self.energy_tolerance,self.max_depth)
            if identity_cache not in self.physics_cache:
                self.physics_cache[identity_cache]=j.enclosure.integrate_and_check(c,T,ac,start,target,self.energy_tolerance,self.max_depth)
            out=self.physics_cache[identity_cache]
            if not out['passed']:
                failed=out['failures'][0];s=(failed['s0']+failed['s1'])/2
                request=dict(kind='physics',task=seg.task,index=seg.index,s=s)
                exposed=failed.get('checks',{}).get('exposure',False)
                if exposed:
                    s=float(np.clip((s*T-1)/.001,0,1))
                    request=dict(kind='exposure',task=seg.task,index=seg.index,s=s)
                registry=j.physics.exposure_points if exposed else j.physics.extra_points
                if (seg.task,seg.index,s) in registry:unresolved.append(dict(family='force_optics_interval',task=seg.task,index=seg.index,result=out))
                else:requests.append(request)
                continue
            E=value(j.physics.segment_energy[seg.task,seg.index]);lo,hi=out['lower_Wh'],out['upper_Wh']
            # A finite quadrature equation never by itself certifies energy.
            if not lo-1e-6<=E<=hi+1e-6:
                order=j.physics.current_orders.get((seg.task,seg.index),1)
                requests.append(dict(kind='materialize',task=seg.task,index=seg.index,order=order+4));continue
            energies[identity]=(lo,hi);contact=None;own=False
            if seg.role=='in' and seg.index==0 and value(task.root)>.5:
                contact=np.array([value(x) for x in j.rendezvous.anchors[flight].launch.position[0]])+j.curves.origin;own=True;endpoint=0.
            elif seg.role=='recovery' and seg.index==2*j.curves.transfers and value(task.last)>.5:
                contact=np.array([value(x) for x in j.rendezvous.anchors[flight].land.position[0]])+j.curves.origin;own=True;endpoint=1.
            if own and ('pad',seg.task,seg.index,endpoint) not in j.safety.applied:
                requests.append(dict(kind='pad',task=seg.task,index=seg.index,s=endpoint))
            out=self.geometry.segment(c,contact)
            self._geometric(out,dict(task=seg.task,index=seg.index),requests,unresolved,'static')
            if target:
                exposure_curve=restrict(c,1./T,1.001/T)
                for q,point in enumerate(target['target']['surface_samples_utm_m']):
                    out=self.geometry.sight_line(exposure_curve,np.asarray(point))
                    self._geometric(out,dict(task=seg.task,index=seg.index,surface_sample=q,exposure=True),requests,unresolved,'static')
            active.append(dict(segment=seg,c=c,ac=ac,T=T,start=start,end=start+T,flight=flight,carrier=j.path.flights[flight].carrier,own_contact=own,target=target))
        if requests or unresolved:return dict(passed=False,requests=requests,unresolved=unresolved)
        for carrier,phases in enumerate(j.vehicles.phases):
            for phase in phases:
                if phase.index%64==0:resource_checkpoint()
                cp=array(phase.controls);T=value(phase.duration);start=value(phase.start);category=int(round(value(phase.mode.value)))
                base=dict(carrier=carrier,phase_index=phase.index,category=category)
                mode=j.vehicles.domain.modes[category]
                geometric_key=(cp.tobytes(),T,category)
                if geometric_key not in vehicle_cache:
                    vehicle_cache[geometric_key]=[
                        ('vehicle_physics',self.geometry.vehicle_dynamics(cp,T,mode)),
                        ('footprint',self.geometry.footprint(cp,category)),
                        ('vehicle_static',self.geometry.vehicle_static(cp))]
                for family,out in vehicle_cache[geometric_key]:
                    self._geometric(out,base,requests,unresolved,family)
                vehicles.append(dict(phase=phase,c=cp,T=T,start=start,end=start+T,category=category,carrier=carrier,
                                     collision_key=(cp.tobytes(),start,T)))
        if requests or unresolved:return dict(passed=False,requests=requests,unresolved=unresolved)
        active.sort(key=lambda x:x['start'])
        for ai,a in enumerate(active):
            for b in active[ai+1:]:
                if b['start']>a['end']+1e-6:break
                if a['flight']==b['flight']:continue
                out=self.geometry.aircraft_pair(a['c'],a['start'],a['T'],b['c'],b['start'],b['T'])
                self._geometric(out,dict(task_a=a['segment'].task,index_a=a['segment'].index,task_b=b['segment'].task,index_b=b['segment'].index),requests,unresolved,'uav_pair')
            for b in vehicles:
                if b['start']>a['end']+1e-6 or a['start']>b['end']+1e-6:continue
                out=self.geometry.aircraft_vehicle(a['c'],a['start'],a['T'],b['c'],b['start'],b['T'],a['own_contact'] and a['carrier']==b['carrier'])
                base=dict(task=a['segment'].task,index=a['segment'].index,carrier=b['carrier'],phase_index=b['phase'].index)
                self._geometric(out,base,requests,unresolved,'aircraft_vehicle')
                if a['target']:
                    for q,point in enumerate(a['target']['target']['surface_samples_utm_m']):
                        out=self.geometry.vehicle_sight_line(restrict(a['c'],1/a['T'],1.001/a['T']),a['start']+1,.001,b['c'],b['start'],b['T'],np.asarray(point))
                        self._geometric(out,dict(base,surface_sample=q),requests,unresolved,'vehicle_los')
        for ai,a in enumerate(vehicles):
            for b in vehicles[ai+1:]:
                if a['carrier']==b['carrier'] or b['start']>a['end']+1e-6 or a['start']>b['end']+1e-6:continue
                base=dict(carrier_a=a['carrier'],index_a=a['phase'].index,carrier_b=b['carrier'],index_b=b['phase'].index)
                key=(a['collision_key'],b['collision_key']);vehicle_pair_checks+=1
                if vehicle_pair_checks%16384==0:resource_checkpoint()
                if key not in vehicle_pair_cache:
                    vehicle_pair_cache[key]=self.geometry.vehicle_pair(a['c'],a['start'],a['T'],b['c'],b['start'],b['T'])
                out=vehicle_pair_cache[key]
                self._geometric(out,base,requests,unresolved,'vehicle_pair')
                if a['category']==b['category'] and j.vehicles.domain.modes[a['category']]['kind']=='stop' and min(a['end'],b['end'])-max(a['start'],b['start'])>1e-6:
                    requests.append(dict(kind='parking',category=a['category'],**base))
        # Whole-shift physical-camera polynomial extrema, including waiting.
        for index,cam in enumerate(j.rendezvous.ground_camera):
            if value(cam['active'])<.5:continue
            T=value(cam['duration']);cp=array(control_points(cam['start'],cam['end'],cam['duration'],cam['middle']))
            if T<=0:unresolved.append(dict(family='ground_camera',index=index,reason='nonpositive_active_camera_duration'));continue
            for derivative in range(3):
                for q in range(3):
                    if derivative==0:lo,hi=j.curves.pose_lo[q+1],j.curves.pose_hi[q+1]
                    elif derivative==1:lo,hi=-j.curves.pose_rate[q+1],j.curves.pose_rate[q+1]
                    elif q<2:lo,hi=-math.radians(30),math.radians(30)
                    else:continue
                    mn,mx,sn,sx=polynomial_range(cp[:,q])
                    if mn<lo-1e-6 or mx>hi+1e-6:requests.append(dict(kind='ground_camera',index=index,s=sn if mn<lo-1e-6 else sx))
                cp=(len(cp)-1)*np.diff(cp,axis=0)/T
        # Independently accumulate conservative energy intervals by actual
        # flight and physical pack chains. This is separate from SCIP's Wh.
        flight_energy={}
        for a in active:
            lo,hi=energies[a['segment'].task,a['segment'].index]
            if a['flight'] not in flight_energy:flight_energy[a['flight']]=[0.,0.]
            flight_energy[a['flight']][0]+=lo;flight_energy[a['flight']][1]+=hi
        ledger=[]
        for carrier,ids in j.resources.by_carrier.items():
            packs=j.scene['fleet']['vehicles'][carrier]['battery_pair_ids']
            for identity,pid in enumerate(packs,1):
                initial=next(x['initial_Wh'] for x in j.scene['fleet']['battery_pairs'] if x['id']==pid);lower=upper=float(initial)
                flights=sorted([i for i in ids if value(j.path.flights[i].active)>.5 and int(round(value(j.resources.flights[i].battery.resource.value)))==identity],key=lambda i:value(j.path.flights[i].start))
                for i in flights:
                    res=j.resources.flights[i];gain=value(res.charged);lower+=gain;upper+=gain
                    if upper>j.resources.capacity+1e-6:unresolved.append(dict(family='physical_pack_energy',pack=pid,flight=i,upper_Wh=upper,capacity_Wh=j.resources.capacity,reason='interval upper launch energy exceeds capacity'))
                    elo,ehi=flight_energy.get(i,[0.,0.]);lower-=ehi;upper-=elo
                    if lower<j.resources.reserve-1e-6:unresolved.append(dict(family='physical_pack_energy',pack=pid,flight=i,lower_Wh=lower,reserve_Wh=j.resources.reserve,reason='interval lower energy crosses reserve; candidate not certified'))
                ledger.append(dict(pack=pid,lower_terminal_Wh=lower,upper_terminal_Wh=upper))
        return dict(passed=not requests and not unresolved,requests=requests,unresolved=unresolved,
                    active_segments=len(active),vehicle_phases=len(vehicles),energy_intervals=ledger,
                    original_road_points_checked=road_points['points_checked'] if road_points is not None else None,
                    vehicle_geometry_unique_evaluations=len(vehicle_cache),
                    vehicle_pair_checks=vehicle_pair_checks,vehicle_pair_unique_evaluations=len(vehicle_pair_cache),
                    energy_tolerance_per_segment_Wh=self.energy_tolerance,
                    formal_directed_rounding_certificate=False,geometric_tolerance_m=self.geometry.tolerance,
                    ackermann_residual_tolerance_SI=1e-4)

    @staticmethod
    def _geometric(out,base,requests,unresolved,kind):
        if out.get('passed'):return
        if out.get('witness') is not None:
            witness=dict(out['witness']);obstacle_kind=witness.pop('kind',None)
            if obstacle_kind is not None:witness['obstacle_kind']=obstacle_kind
            requests.append(dict(kind=kind,**base,**witness))
        else:unresolved.append(dict(family=kind,**base,result=out))
