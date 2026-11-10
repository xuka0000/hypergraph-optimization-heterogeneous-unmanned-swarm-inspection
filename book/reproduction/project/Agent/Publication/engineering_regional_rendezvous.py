"""Joint endogenous parked-vehicle / continuous-UAV endpoint coupling."""
from dataclasses import dataclass
import math
from pyscipopt import quicksum
from Agent.Publication.engineering_indexed_constraints import AffineRow
from Agent.Publication.engineering_regional_curves import control_points


@dataclass
class FlightAnchors:
    launch: object
    land: object
    launch_phase: object
    land_phase: object
    pad_exit: object
    pad_entry: object
    launch_ground: object
    land_ground: object


class RegionalRendezvous:
    def __init__(self,curves,vehicles,resources,terrain):
        self.curves=curves;self.vehicles=vehicles;self.resources=resources;self.terrain=terrain
        self.path=curves.path;self.m=self.path.m;self.rel=self.path.rel
        if not(curves.m is vehicles.m is resources.m):raise ValueError('Separate solved stages are not permitted')
        self.anchors=[];self.ground_camera=[];self.extra_camera_points=set();self.deferred_camera=False

    def build(self):
        p,m,r=self.path,self.m,self.resources
        for i,f in enumerate(p.flights):
            launch=self.curves.node(f'flight{i+1}_launch');land=self.curves.node(f'flight{i+1}_land')
            phases=self.vehicles.phases[f.carrier]
            ls=self.rel.selector(f'flight{i+1}_launch_phase',len(phases));rs=self.rel.selector(f'flight{i+1}_land_phase',len(phases))
            ex=r.var(f'flight{i+1}_pad_exit',0,p.H);entry=r.var(f'flight{i+1}_pad_entry',0,p.H)
            m.addCons(ex<=p.H*f.active);m.addCons(entry<=p.H*f.active)
            m.addCons(ex>=f.start+4*f.active);m.addCons(entry<=f.end-4*f.active);m.addCons(entry>=ex)
            grounds=[]
            for name,anchor,sel in [('launch',launch,ls),('land',land,rs)]:
                g=self.terrain.add(m,self.rel,anchor.position[0][:2],f'flight{i+1}_{name}_ground')
                grounds.append(g['ground'])
                m.addCons(anchor.position[0][2]==g['ground']+2.4-self.curves.origin[2])
                for q in range(3):m.addCons(anchor.position[1][q]==0)
                m.addCons(anchor.pose[1][0]==0);m.addCons(anchor.pose[2][0]==0)
                def rows(k,name=name,anchor=anchor,i=i,phases=phases,ex=ex,entry=entry):
                    phase=phases[k];f=p.flights[i]
                    ans=[r.eq('vehicle_parked',[(1,phase.parked)],1)]
                    ans += [r.eq('same_position'+str(q),[(1,anchor.position[0][q]),(-1,phase.controls[0][q])]) for q in range(2)]
                    ans.append(r.eq('same_heading',[(1,anchor.pose[0][0]),(-1,phase.controls[0][2])]))
                    if name=='launch':
                        ans += [AffineRow('parked_through_preparation',[(1,f.start),(-1,phase.start)],lo=p.pre),
                                AffineRow('parked_until_pad_exit',[(1,phase.end),(-1,ex)],lo=0)]
                    else:
                        ans += [AffineRow('parked_before_pad_entry',[(1,entry),(-1,phase.start)],lo=0),
                                AffineRow('parked_through_recovery',[(1,phase.end),(-1,f.end)],lo=p.post)]
                    return ans
                self.rel.add(sel,rows,[*anchor.position[0],*anchor.pose[0]],f.active,f'flight{i+1}_{name}_vehicle')
            self.anchors.append(FlightAnchors(launch,land,ls,rs,ex,entry,*grounds))
        self.rel.lock(m.getVars())
        for i,t in enumerate(p.tasks):
            data=self.curves.by_task[i];first_segment=data['segments'][0];last_segment=data['segments'][-1]
            for role,flag in [('launch',t.root),('land',t.last)]:
                def link(k,i=i,role=role,first_segment=first_segment,last_segment=last_segment):
                    if not k:return [AffineRow('nonzero_flight',[],lo=1)]
                    a=self.anchors[k-1];d=self.curves.by_task[i]
                    if role=='land' and self.curves.shared_recovery:
                        d=dict(d,recovery_end=self.curves.recoveries[k-1]['nodes'][-1])
                        last_segment=self.curves.recoveries[k-1]['segments'][-1]
                    anchor=a.launch if role=='launch' else a.land
                    curve=d['in_start'] if role=='launch' else d['recovery_end']
                    ans=[]
                    # Contact acceleration/jerk are one-sided. Position,
                    # velocity and the entire camera/yaw state are connected.
                    for rr in [0,1]:
                        ans += [r.eq('contact_position_d'+str(rr)+'_'+str(q),[(1,curve.position[rr][q]),(-1,anchor.position[rr][q])]) for q in range(3)]
                    for rr in range(3):
                        ans += [r.eq('contact_pose_d'+str(rr)+'_'+str(q),[(1,curve.pose[rr][q]),(-1,anchor.pose[rr][q])]) for q in range(4)]
                    if role=='launch':
                        ans += [r.eq('positive_departure_piece',[(1,first_segment.active)],1),
                                r.eq('pad_exit_clock',[(1,a.pad_exit),(-1,p.flights[k-1].start),(-1,first_segment.duration)]),
                                r.eq('pad_exit_height',[(1,first_segment.right.position[0][2]),(-1,a.launch_ground)],6.4-self.curves.origin[2])]
                    else:
                        ans += [r.eq('positive_arrival_piece',[(1,last_segment.active)],1),
                                r.eq('pad_entry_clock',[(1,a.pad_entry),(-1,p.flights[k-1].end),(1,last_segment.duration)]),
                                r.eq('pad_entry_height',[(1,last_segment.left.position[0][2]),(-1,a.land_ground)],6.4-self.curves.origin[2])]
                    return ans
                self.rel.add(t.flight,link,[],flag,f'task{i+1}_{role}_anchor')
        # One handling position per carrier. Preparation/ascent and
        # descent/recovery share the same physical pad, including overlapping
        # flights using different UAVs/batteries.
        for carrier,ids in r.by_carrier.items():
            intervals=[]
            for i in ids:
                f=p.flights[i];a=self.anchors[i]
                intervals += [(i,'departure',f.start-p.pre,a.pad_exit,f.active),
                              (i,'arrival',a.pad_entry,f.end+p.post,f.active)]
            M=p.H+p.pre+p.post
            for u,ia in enumerate(intervals):
                for ib in intervals[:u]:
                    if ia[0]==ib[0]:continue
                    before=r.var(f'pad_v{carrier}_{ia[0]}_{ia[1]}_{ib[0]}_{ib[1]}_order',0,1,True)
                    off=2-ia[4]-ib[4]
                    m.addCons(ia[3]<=ib[2]+M*(1-before+off))
                    m.addCons(ib[3]<=ia[2]+M*(before+off))
        self.rel.lock(m.getVars());self.curves.anchors_attached=True
        return self

    def _camera_curve(self,start,end,T,active,name):
        """Same degree7 camera state through ground waiting, no pose reset."""
        lower=self.curves.pose_lo[1:];upper=self.curves.pose_hi[1:]
        middle=[]
        for j in [3,4]:
            middle.append([self.m.addVar(name+f'_control{j}_{q}',
                lb=(lower[q]+upper[q])/2-self.curves.inverse_norm[j]*(upper[q]-lower[q])/2,
                ub=(lower[q]+upper[q])/2+self.curves.inverse_norm[j]*(upper[q]-lower[q])/2) for q in range(3)])
        # A candidate is checked over the entire ground polynomial by
        # RegionalAcceptance. Only allocate shared controls/point rows when
        # requested; the polynomial decision itself is never removed.
        safe=self.m.addVar(name+'_safe_duration',lb=1,ub=self.path.H+1)
        self.m.addCons(safe==T+1-active)
        z=dict(name=name,active=active,duration=T,start=start,end=end,middle=middle,
               shared_controls=None,safe_duration=safe)
        self.ground_camera.append(z)
        if not self.deferred_camera:
            for s in [j/7 for j in range(8)]:self._camera_point_rows(z,s)

    def _camera_controls(self,z):
        if z['shared_controls'] is not None:return z['shared_controls']
        name=z['name'];lower=self.curves.pose_lo[1:];upper=self.curves.pose_hi[1:]
        expressions=control_points(z['start'],z['end'],z['duration'],z['middle'])
        # Exact shared definitions prevent SCIP from repeatedly expanding the
        # endpoint/duration polynomial inside every camera inequality. Bounds
        # follow from the existing eight interpolation-point value bounds.
        cp=[]
        for k,row in enumerate(expressions):
            cp.append([])
            for q,expr in enumerate(row):
                mid=(lower[q]+upper[q])/2;rad=(upper[q]-lower[q])/2
                v=self.m.addVar(name+f'_shared_control{k}_{q}',
                    lb=mid-self.curves.inverse_norm[k]*rad,ub=mid+self.curves.inverse_norm[k]*rad)
                self.m.addCons(v==expr);cp[-1].append(v)
        # Active waiting lasts at least preparation (120s), and an active
        # terminal interval lasts at least recovery (240s). Thus safe>=1 is
        # already implied by the physical resource chains, also when unused.
        z['shared_controls']=cp
        return cp

    def _camera_point_rows(self,z,s):
        current=self._camera_controls(z);active=z['active'];safe=z['safe_duration']
        for derivative in range(3):
            n=len(current)-1
            values=[quicksum(math.comb(n,k)*s**k*(1-s)**(n-k)*row[q] for k,row in enumerate(current)) for q in range(3)]
            scale=safe**derivative if derivative else 1
            for q,x in enumerate(values):
                if derivative==0:lo,hi=self.curves.pose_lo[q+1],self.curves.pose_hi[q+1]
                elif derivative==1:lo,hi=-self.curves.pose_rate[q+1],self.curves.pose_rate[q+1]
                elif q<2:lo,hi=-math.radians(30),math.radians(30)
                else:continue
                # Multiplication by the strictly positive shared denominator
                # is algebraically equivalent; no derivative bound is relaxed.
                self.m.addCons(active*(x-lo*scale)>=0)
                self.m.addCons(active*(hi*scale-x)>=0)
            current=[[n*(current[k+1][q]-current[k][q]) for q in range(3)] for k in range(n)]

    def attach_global_camera(self,deferred=False):
        self.deferred_camera=deferred
        p,r,m=self.path,self.resources,self.m
        for i,(f,res,anchor) in enumerate(zip(p.flights,r.flights,self.anchors)):
            before=self.curves.node(f'flight{i+1}_ground_camera_before')
            ts=r.var(f'flight{i+1}_ground_camera_start_s',0,p.H)
            T=r.var(f'flight{i+1}_ground_camera_duration_s',0,p.H)
            m.addCons(T==f.start-ts);m.addCons(ts<=p.H*f.active)
            local=r.by_carrier[f.carrier]
            def prev(k,i=i,local=local,before=before,ts=ts):
                if not k:return [AffineRow('nonzero_previous_aircraft_flight',[],lo=1)]
                prev=local[k-1];a=self.anchors[prev].land
                ans=[r.eq('previous_landing_time',[(1,ts),(-1,p.flights[prev].end)])]
                ans += [r.eq(f'camera_carry_d{d}_{q}',[(1,before.pose[d][q]),(-1,a.pose[d][q])]) for d in range(3) for q in [1,2,3]]
                return ans
            self.rel.add(res.aircraft.pred,prev,[],res.aircraft.has_pred,f'flight{i+1}_camera_previous')
            def initial(k,i=i,before=before,ts=ts):
                if not k:return [AffineRow('nonzero_aircraft',[],lo=1)]
                f=p.flights[i];uid=self.curves.scene['fleet']['vehicles'][f.carrier]['uav_ids'][k-1]
                aircraft=next(u for u in self.curves.scene['fleet']['aircraft'] if u['id']==uid)
                v=[*aircraft['initial_gimbal_pitch_yaw_rad'],aircraft['initial_focal_length_mm']/6.83]
                ans=[r.eq('initial_camera_time',[(1,ts)],0)]
                ans += [r.eq(f'initial_camera_d{d}_{q}',[(1,before.pose[d][q+1])],v[q] if d==0 else 0) for d in range(3) for q in range(3)]
                return ans
            self.rel.add(res.aircraft.resource,initial,[],res.aircraft.first,f'flight{i+1}_camera_initial')
            start=[a[1:] for a in before.pose];end=[a[1:] for a in anchor.launch.pose]
            self._camera_curve(start,end,T,f.active,f'flight{i+1}_camera_ground_before')
            tail=self.curves.node(f'flight{i+1}_camera_terminal')
            duration=r.var(f'flight{i+1}_camera_tail_duration',0,p.H)
            # T can be zero for unused flight slots; avoid imposing a fixed
            # end-time on an inactive identity.
            r.gated([(1,duration),(1,f.end),(-1,p.C)],res.aircraft.last,f'flight{i+1}_camera_tail_clock',0,0)
            m.addCons(duration<=p.H*res.aircraft.last)
            self._camera_curve([a[1:] for a in anchor.land.pose],[a[1:] for a in tail.pose],duration,
                               res.aircraft.last,f'flight{i+1}_camera_ground_tail')
        self.rel.lock(m.getVars());r.pose_attached=True
        return self

    def status(self):
        return dict(flights=len(self.anchors),same_model_vehicle_endpoints=True,
            stopped_preparation_pad_exit_entry_recovery=True,pad_occupancy_attached=True,
            camera_ground_curves=len(self.ground_camera),camera_continuity=self.resources.pose_attached,
            continuous_pad_zone_constraints_attached=False,continuous_camera_verified=False,
            flight_attitude_endpoint_force_attached=False,full_joint_model=False)

    def add_camera_point(self,index,s):
        if (index,s) in self.extra_camera_points:return False
        self._camera_point_rows(self.ground_camera[index],s)
        self.extra_camera_points.add((index,s));return True
