"""Whole-region UAV/physical-pack continuity in the *same* SCIP model.

No resource is refreshed between flights. Indexed predecessor chains retain all
orders; their clocks rule out cycles. Charging credit is fixed to zero until a
charging ledger is attached, so an unfinished adapter cannot create free Wh.
This component does not itself constitute a complete inspection optimizer.
"""
from dataclasses import dataclass
from pyscipopt import quicksum
from Agent.Publication.engineering_indexed_constraints import AffineRow


@dataclass
class ResourceChain:
    resource: object
    assignment: list
    pred: object
    succ: object
    first: object
    last: object
    has_pred: object
    has_succ: object


@dataclass
class FlightResource:
    aircraft: ResourceChain
    battery: ResourceChain
    before_charge: object
    charged: object
    launch_energy: object
    landing_energy: object
    energy: object
    available_after: object


class RegionalResources:
    def __init__(self, path, scene):
        self.path=path;self.m=path.m;self.rel=path.rel;self.scene=scene
        self.H=path.H;self.flights=[];self.charging_attached=False
        self.energy_attached=False;self.pose_attached=False
        self.capacity=float(scene['joint_reference_profile']['energy']['pair_capacity_Wh'])
        self.reserve=self.capacity*float(scene['joint_reference_profile']['energy']['minimum_SOC'])
        self.by_carrier={v:[i for i,f in enumerate(path.flights) if f.carrier==v]
                         for v in range(len(path.slots_per_carrier))}

    def var(self,name,lb=0,ub=None,binary=False):
        return self.m.addVar(name,lb=lb,ub=ub,vtype='B' if binary else 'C')

    @staticmethod
    def eq(name,terms,value=0):return AffineRow(name,terms,value,value)

    def gated(self,terms,active,name,lo=None,hi=None,constant=0):
        row=AffineRow(name,terms,lo,hi,constant);low,high=row.bounds();expr=row.expression()
        if lo is not None:self.m.addCons(expr+max(0,lo-low)*(1-active)>=lo,name=name+'_lo')
        if hi is not None:self.m.addCons(expr-max(0,high-hi)*(1-active)<=hi,name=name+'_hi')

    def _chain(self,i,count,kind):
        f=self.path.flights[i];m=self.m;prefix=f'{kind}_flight{i+1}'
        resource=self.rel.selector(prefix+'_identity',count+1)
        assignment=[self.var(prefix+f'_is{k+1}',ub=1,binary=True) for k in range(count)]
        m.addCons(quicksum(assignment)==f.active,name=prefix+'_one_identity')
        m.addCons(resource.value==quicksum((k+1)*a for k,a in enumerate(assignment)),name=prefix+'_identity_code')
        # The domain is carrier-local; foreign resource identities cannot be
        # accidentally interpreted as local indices.
        size=len(self.by_carrier[f.carrier])+1
        pred=self.rel.selector(prefix+'_pred',size);succ=self.rel.selector(prefix+'_succ',size)
        first,last,hp,hs=[self.var(prefix+'_'+s,ub=1,binary=True) for s in ['first','last','has_pred','has_succ']]
        m.addCons(first+hp==f.active);m.addCons(last+hs==f.active)
        for sel,has in [(pred,hp),(succ,hs)]:
            m.addCons(sel.value>=has);m.addCons(sel.value<=(size-1)*has)
        return ResourceChain(resource,assignment,pred,succ,first,last,hp,hs)

    def build(self):
        m=self.m;Q=self.capacity
        for i,f in enumerate(self.path.flights):
            vehicle=self.scene['fleet']['vehicles'][f.carrier]
            u=self._chain(i,len(vehicle['uav_ids']),'aircraft')
            b=self._chain(i,len(vehicle['battery_pair_ids']),'pack')
            before=self.var(f'flight{i+1}_before_charge_Wh',self.reserve,Q)
            charged=self.var(f'flight{i+1}_charged_Wh',0,Q-self.reserve)
            launch=self.var(f'flight{i+1}_launch_Wh',self.reserve,Q)
            landing=self.var(f'flight{i+1}_landing_Wh',self.reserve,Q)
            energy=self.var(f'flight{i+1}_aircraft_Wh',0,Q-self.reserve)
            available=self.var(f'flight{i+1}_pack_available_s',0,self.H)
            m.addCons(launch==before+charged,name=f'flight{i+1}_charge_conservation')
            m.addCons(landing==launch-energy,name=f'flight{i+1}_flight_conservation')
            m.addCons(energy<=(Q-self.reserve)*f.active)
            m.addCons(charged<=(Q-self.reserve)*f.active)
            self.gated([(1,f.start),(-1,available)],f.active,f'flight{i+1}_pack_ready',lo=self.path.pre)
            # Until the actual ledger is linked, no charging is possible.
            m.chgVarUb(charged,0.)
            self.flights.append(FlightResource(u,b,before,charged,launch,landing,energy,available))
        self.rel.lock(m.getVars())
        for i,(f,r) in enumerate(zip(self.path.flights,self.flights)):
            local=self.by_carrier[f.carrier];local_id=local.index(i)+1
            for kind in ['aircraft','battery']:
                chain=getattr(r,kind)
                def pred_rows(k,i=i,kind=kind,local=local,local_id=local_id):
                    if k==0 or local[k-1]==i:return [AffineRow('invalid_predecessor',[],lo=1)]
                    p=local[k-1];cur=self.flights[i];prev=self.flights[p]
                    c=getattr(cur,kind);pc=getattr(prev,kind);f=self.path.flights[i];pf=self.path.flights[p]
                    rows=[self.eq('predecessor_active',[(1,pf.active)],1),
                          self.eq('reciprocal_successor',[(1,pc.succ.value)],local_id),
                          self.eq('same_physical_identity',[(1,c.resource.value),(-1,pc.resource.value)]),
                          AffineRow('prepare_after_previous_recovery',[(1,f.start),(-1,pf.end)],lo=self.path.pre+self.path.post)]
                    if kind=='battery':
                        rows += [self.eq('carry_previous_remaining_Wh',[(1,cur.before_charge),(-1,prev.landing_energy)]),
                                 self.eq('available_after_recovery',[(1,cur.available_after),(-1,pf.end)],self.path.post)]
                    return rows
                def succ_rows(k,i=i,kind=kind,local=local,local_id=local_id):
                    if k==0 or local[k-1]==i:return [AffineRow('invalid_successor',[],lo=1)]
                    p=local[k-1];c=getattr(self.flights[i],kind);pc=getattr(self.flights[p],kind)
                    return [self.eq('successor_active',[(1,self.path.flights[p].active)],1),
                            self.eq('reciprocal_predecessor',[(1,pc.pred.value)],local_id),
                            self.eq('same_physical_identity',[(1,c.resource.value),(-1,pc.resource.value)])]
                self.rel.add(chain.pred,pred_rows,[],chain.has_pred,f'{kind}_{i+1}_pred')
                self.rel.add(chain.succ,succ_rows,[],chain.has_succ,f'{kind}_{i+1}_succ')
            # Each true physical pack has its own initial energy, not an
            # average fleet SOC. Factory selects the actual source identity.
            def initial(k,i=i):
                if not k:return [AffineRow('invalid_initial_pack',[],lo=1)]
                f=self.path.flights[i];r=self.flights[i]
                pid=self.scene['fleet']['vehicles'][f.carrier]['battery_pair_ids'][k-1]
                p=next(x for x in self.scene['fleet']['battery_pairs'] if x['id']==pid)
                return [self.eq('source_initial_Wh',[(1,r.before_charge)],float(p['initial_Wh'])),
                        self.eq('source_available_time',[(1,r.available_after)],0)]
            self.rel.add(r.battery.resource,initial,[],r.battery.first,f'pack_initial_{i+1}')
        # At most one root per identity forbids disconnected chains silently
        # obtaining several copies of the same initial battery/UAV.
        for carrier,ids in self.by_carrier.items():
            for kind in ['aircraft','battery']:
                count=len(getattr(self.flights[ids[0]],kind).assignment) if ids else 0
                for k in range(count):
                    roots=[]
                    for i in ids:
                        c=getattr(self.flights[i],kind)
                        z=self.var(f'{kind}_{i+1}_root_identity{k+1}',ub=1,binary=True)
                        m.addCons(z<=c.first);m.addCons(z<=c.assignment[k]);m.addCons(z>=c.first+c.assignment[k]-1)
                        roots.append(z)
                    m.addCons(quicksum(roots)<=1,name=f'{kind}_carrier{carrier}_identity{k+1}_one_chain')
        self.total_energy=quicksum(r.energy for r in self.flights)
        return self

    def attach_task_energy(self, task_energy, recovery_energy, flight_recovery_energy=None):
        """Exact path accumulation of the *trajectory-derived* Wh expressions.

        This replaces O(tasks*flights) products. A free flight-energy variable
        is never sufficient to declare this attachment complete.
        """
        if self.energy_attached:raise ValueError('Energy already attached')
        if len(task_energy)!=len(self.path.tasks) or len(recovery_energy)!=len(self.path.tasks):
            raise ValueError('Every regional task and recovery expression required')
        cap=self.capacity-self.reserve
        self.flight_recovery_energy=[]
        if flight_recovery_energy is not None:
            if len(flight_recovery_energy)!=len(self.path.flights):raise ValueError('One return integral per flight required')
            for k,(f,expr) in enumerate(zip(self.path.flights,flight_recovery_energy)):
                e=self.var(f'flight{k+1}_recovery_Wh',0,cap)
                self.m.addCons(e==expr);self.m.addCons(e<=cap*f.active)
                self.flight_recovery_energy.append(e)
        self.prefix=[]
        for i,t in enumerate(self.path.tasks):
            before=self.var(f'task{i+1}_prefix_before_Wh',0,cap)
            after=self.var(f'task{i+1}_prefix_after_Wh',0,cap)
            self.m.addCons(after==before+task_energy[i],name=f'task{i+1}_path_energy')
            self.m.addCons(before<=cap*t.active);self.m.addCons(after<=cap*t.active)
            self.gated([(1,before)],t.root,f'task{i+1}_root_energy',0,0)
            self.prefix.append((before,after))
        self.rel.lock(self.m.getVars())
        for i,t in enumerate(self.path.tasks):
            def predecessor(k,i=i):
                if not k:return [AffineRow('no_null_predecessor',[],lo=1)]
                return [self.eq('path_energy_carry',[(1,self.prefix[i][0]),(-1,self.prefix[k-1][1])])]
            self.rel.add(t.pred,predecessor,[],t.has_pred,f'task{i+1}_energy_pred')
            # A bounded recovery-energy auxiliary makes this selected lookup
            # affine while its defining nonlinear trajectory integral stays
            # in this very same SCIP model.
            e=self.var(f'task{i+1}_recovery_Wh',0,cap)
            if flight_recovery_energy is None:self.m.addCons(e==recovery_energy[i])
            else:
                def return_rows(k,e=e):
                    if not k:return [AffineRow('nonzero_return_flight',[],lo=1)]
                    return [self.eq('selected_flight_return_energy',[(1,e),(-1,self.flight_recovery_energy[k-1])])]
                self.rel.add(t.flight,return_rows,[e],t.last,f'task{i+1}_shared_return_energy')
            self.m.addCons(e<=cap*t.last)
            def last(k,i=i,e=e):
                if not k:return [AffineRow('no_null_flight',[],lo=1)]
                return [self.eq('flight_energy_sum',[(1,self.flights[k-1].energy),(-1,self.prefix[i][1]),(-1,e)])]
            self.rel.add(t.flight,last,[e],t.last,f'task{i+1}_last_energy')
        self.energy_attached=True

    def status(self):
        return dict(flights=len(self.flights),physical_aircraft=len(self.scene['fleet']['aircraft']),
                    physical_battery_pairs=len(self.scene['fleet']['battery_pairs']),
                    energy_attached=self.energy_attached,charging_attached=self.charging_attached,
                    global_camera_pose_attached=self.pose_attached,full_joint_model=False,
                    unbound_charging_credit_fixed_zero=not self.charging_attached)
