"""Unified regional model, fail-closed candidate oracle and traceable refinement.

All decisions remain in one SCIP model. Original-stage additions after
freeTransform materialize necessary rows requested by rejected candidates.
No physical state or route is frozen between these implementation passes.
Finite grids and quadrature are explicit; bounds are never called continuous-
domain optimality certificates. No wall-time/node/gap cap is set here.
"""
from pathlib import Path
import datetime,json,time,math,hashlib,os
import numpy as np
import psutil
from scipy.interpolate import LinearNDInterpolator
from pyproj import Transformer
from pyscipopt import Conshdlr,SCIP_RESULT,Eventhdlr,SCIP_EVENTTYPE,SCIP_PARAMSETTING
from Agent.Publication.engineering_regional_structure import load_regional,RegionalPathBlock,pad_slot_bound
from Agent.Publication.engineering_regional_resources import RegionalResources
from Agent.Publication.engineering_regional_roads import RegionalVehicles
from Agent.Publication.engineering_regional_charging import RegionalCharging
from Agent.Publication.engineering_regional_curves import RegionalCurves
from Agent.Publication.engineering_indexed_terrain import IndexedTerrain
from Agent.Publication.engineering_regional_rendezvous import RegionalRendezvous
from Agent.Publication.engineering_regional_weather import retained_regional_forecast
from Agent.Publication.engineering_regional_physics import RegionalPhysics
from Agent.Publication.engineering_regional_safety import RegionalSafety
from Agent.Publication.engineering_regional_enclosure import RegionalEnclosure
from Agent.Publication.engineering_regional_acceptance import RegionalAcceptance


def stamp():return datetime.datetime.now(datetime.timezone.utc).isoformat()


class CandidateGuard(Conshdlr):
    def __init__(self,owner):self.owner=owner;self.pending=None;self.accepted_checks=0;self.rejected_checks=0;self.error=None;self.in_algebra_check=False

    def inspect(self,solution=None):
        j=self.owner;m=j.path.m
        if self.pending is not None or self.error is not None:return False
        try:
            if solution is not None:
                self.in_algebra_check=True
                try:algebra_ok=m.checkSol(solution,printreason=False,completely=False)
                finally:self.in_algebra_check=False
                if not algebra_ok:return False
            out=j.acceptance.check(solution)
            if out.get('indexed_relations_invalid'):return False
            if out['passed']:
                self.accepted_checks+=1;j.last_acceptance=out;return True
            self.rejected_checks+=1;self.pending=out;m.interruptSolve();return False
        except Exception as exc:
            self.error=dict(type=type(exc).__name__,message=str(exc));m.interruptSolve();return False

    def conscheck(self,constraints,solution,check_integrality,check_lp_rows,print_reason,completely,**kwargs):
        if self.in_algebra_check:return {'result':SCIP_RESULT.FEASIBLE}
        return {'result':SCIP_RESULT.FEASIBLE if self.inspect(solution) else SCIP_RESULT.INFEASIBLE}

    def consenfolp(self,constraints,n_useful_conss,sol_infeasible):
        if sol_infeasible:return {'result':SCIP_RESULT.FEASIBLE}
        m=self.owner.path.m
        # Fractional indexed choices belong to the relaxation, not a physical
        # trajectory candidate. Integrality remains SCIP's responsibility.
        if any(abs(m.getSolVal(None,s.value)-round(m.getSolVal(None,s.value)))>1e-6 for s in self.owner.path.rel.selectors):
            return {'result':SCIP_RESULT.FEASIBLE}
        for var in m.getVars():
            if var.vtype()!='CONTINUOUS' and not m.isFeasIntegral(m.getSolVal(None,var)):
                return {'result':SCIP_RESULT.FEASIBLE}
        return {'result':SCIP_RESULT.FEASIBLE if self.inspect() else SCIP_RESULT.INFEASIBLE}

    def consenfops(self,constraints,n_useful_conss,sol_infeasible,obj_infeasible):
        # Demand an LP to avoid accepting unchecked pseudo solutions.
        return {'result':SCIP_RESULT.SOLVELP}

    def conslock(self,constraint,locktype,nlockspos,nlocksneg):pass


class RegionalTrace(Eventhdlr):
    def __init__(self,owner):self.owner=owner;self.last=0.
    def mask(self):return SCIP_EVENTTYPE.BESTSOLFOUND|SCIP_EVENTTYPE.NODEFOCUSED|SCIP_EVENTTYPE.PRESOLVEROUND|SCIP_EVENTTYPE.LPSOLVED|SCIP_EVENTTYPE.FIRSTLPSOLVED
    def eventinit(self):self.model.catchEvent(self.mask(),self)
    def eventexit(self):self.model.dropEvent(self.mask(),self)
    def eventexec(self,event):
        now=time.monotonic()
        if now-self.last>=10 or event.getType()&SCIP_EVENTTYPE.BESTSOLFOUND:
            self.owner.trace('SCIP_event');self.last=now
            if not self.owner.resource_ok():self.model.interruptSolve()


class RegionalJointModel:
    def __init__(self,root,output,road_phases=(256,256),charging_events=(156,156),disable_nlp_relaxation=False,compact_memory=False,disable_primal_heuristics=False):
        self.root=Path(root);self.output=Path(output);self.road_phases=list(road_phases);self.charging_events=list(charging_events)
        self.started=time.monotonic();self.phase=1;self.round=0;self.last_acceptance=None;self.resource_stop=None
        self.scene,self.record=load_regional(root);self.forecast=retained_regional_forecast(root)
        self.registry=[];self.guard=None;self.event_handler=None
        self.run_id=None
        self.disable_nlp_relaxation=disable_nlp_relaxation
        self.compact_memory=compact_memory
        self.disable_primal_heuristics=disable_primal_heuristics

    def resource_ok(self):
        memory=psutil.virtual_memory().available
        disks={d:psutil.disk_usage(d+':/').free/2**30 for d in ['C','D']}
        if memory<.75*2**30 or disks['C']<10.25 or disks['D']<30.25:
            self.resource_stop=dict(observed_at=stamp(),available_RAM_GiB=memory/2**30,free_disks_GiB=disks,
                reason='live RAM/storage floor protection; no elapsed-time optimization cutoff')
            return False
        return True

    def build(self):
        if not self.resource_ok():raise MemoryError(str(self.resource_stop))
        scene=self.scene;record=self.record
        tr=Transformer.from_crs(4326,scene['coordinate_reference']['projected'],always_xy=True)
        samples=scene['terrain']['samples'];x,y=tr.transform([z['location']['lng'] for z in samples],[z['location']['lat'] for z in samples])
        self.terrain=LinearNDInterpolator(np.column_stack([x,y]),[z['elevation'] for z in samples])
        self.path=RegionalPathBlock(scene['inspection_tasks'],record['domain']['tower_tasks'],pad_slot_bound(scene,record)['per_carrier']).build()
        self.path.m.hideOutput();self.path.m.setRealParam('limits/gap',0.);self.path.m.setRealParam('limits/absgap',0.)
        self.path.m.setBoolParam('nlp/disable',self.disable_nlp_relaxation)
        if self.disable_primal_heuristics:self.path.m.setHeuristics(SCIP_PARAMSETTING.OFF)
        if self.compact_memory:
            self.path.m.setRealParam('memory/arraygrowfac',1.05)
            self.path.m.setRealParam('memory/savefac',.5)
            self.path.m.setBoolParam('misc/usevartable',False)
            self.path.m.setBoolParam('misc/useconstable',False)
        self.resources=RegionalResources(self.path,scene).build()
        self.vehicles=RegionalVehicles(self.path,scene,self.road_phases).build()
        self.charging=RegionalCharging(self.resources,self.vehicles.parked,self.charging_events).build().attach_temperature(self.forecast)
        self.curves=RegionalCurves(self.path,scene,record,self.terrain).build()
        self.terrain_index=IndexedTerrain(self.terrain,self.curves.origin[:2])
        self.rendezvous=RegionalRendezvous(self.curves,self.vehicles,self.resources,self.terrain_index).build().attach_global_camera()
        self.physics=RegionalPhysics(self.curves,self.terrain_index,self.forecast,self.rendezvous).register_energy(self.resources)
        self.safety=RegionalSafety(self.curves,self.vehicles,self.rendezvous,self.physics)
        self.enclosure=RegionalEnclosure(scene,self.terrain,self.forecast)
        self.acceptance=RegionalAcceptance(self)
        self.guard=CandidateGuard(self)
        self.path.m.includeConshdlr(self.guard,'regional_continuous_acceptance','all physical families checked before accepting candidates',
            enfopriority=-1000000,chckpriority=-1000000,needscons=False)
        self.event_handler=RegionalTrace(self);self.path.m.includeEventhdlr(self.event_handler,'regional_trace','accepted quality and resource readbacks')
        self.path.rel.lock(self.path.m.getVars());return self

    def trace(self,reason,extra=None):
        m=self.path.m;sol=m.getBestSol();values=None
        if sol is not None:
            values=[float(m.getSolVal(sol,self.path.objective)),float(m.getSolVal(sol,self.path.C)),float(m.getSolVal(sol,self.resources.total_energy))]
        def finite(x):
            try:x=float(x);return x if np.isfinite(x) and abs(x)<1e19 else None
            except (ValueError,TypeError):return None
        row=dict(observed_at=stamp(),run_id=self.run_id,elapsed_s=time.monotonic()-self.started,phase=self.phase,materialization_round=self.round,
            reason=reason,SCIP_status=str(m.getStatus()),accepted_objective_vector=values,
            finite_transcription_dual=finite(m.getDualbound()) if m.getStage()>=9 else None,
            full_continuous_domain_dual=None,original_continuous_optimality_proved=False,
            nodes=int(m.getNNodes()) if m.getStage()>=9 else 0,variables=m.getNVars(),rows=m.getNConss(),
            accepted_checks=self.guard.accepted_checks,rejected_checks=self.guard.rejected_checks,
            rss_GiB=psutil.Process().memory_info().rss/2**30,available_RAM_GiB=psutil.virtual_memory().available/2**30,
            numerical_grids=dict(road_phases=self.road_phases,charging_events=self.charging_events,transfer_pieces=3,position_degree=7),
            source_sha256=self.record['source']['sha256'],model_sha256=self.record['model']['sha256'],extra=extra)
        row['SCIP_memory_limit_MiB']=m.getParam('limits/memory')
        row['SCIP_allocator_MiB']=m.getMemUsed()/2**20
        row['SCIP_allocated_total_MiB']=m.getMemTotal()/2**20
        row['SCIP_external_estimate_MiB']=m.getMemExternEstim()/2**20
        row['NLP_relaxation_disabled']=self.disable_nlp_relaxation
        row['compact_memory_profile']=self.compact_memory
        row['primal_heuristics_disabled']=self.disable_primal_heuristics
        row['materialized_energy_orders']={str(k):sum(order==k for order in self.physics.current_orders.values()) for k in sorted(set(self.physics.current_orders.values()))}
        self.output.mkdir(parents=True,exist_ok=True)
        with (self.output/'progress.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        (self.output/'status.json').write_text(json.dumps(row,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        return row

    def exercise_required_row_hooks(self):
        """Materialize valid necessary rows from every deferred family.

        This checks real symbolic integration on the full source model. It is
        not a feasibility assertion. The decisions stay entirely free.
        """
        s=self.safety;v=self.vehicles;r=self.rendezvous;checks=[]
        def put(name,fn):
            if not self.resource_ok():raise MemoryError(str(self.resource_stop))
            p=self.output/'row_hook_progress.json'
            p.write_text(json.dumps(dict(observed_at=stamp(),run_id=self.run_id,current=name,status='starting',completed=checks),ensure_ascii=False,indent=2),encoding='utf-8')
            before=self.path.m.getNConss();clock=time.perf_counter();fn();checks.append(dict(family=name,added_rows=self.path.m.getNConss()-before,seconds=time.perf_counter()-clock,rss_GiB=psutil.Process().memory_info().rss/2**30))
            p.write_text(json.dumps(dict(observed_at=stamp(),run_id=self.run_id,current=name,status='materialized',completed=checks),ensure_ascii=False,indent=2),encoding='utf-8')
            if not self.resource_ok():raise MemoryError(str(self.resource_stop))
        put('force_optics_energy',lambda:self.physics.materialize_segment(self.curves.by_task[0]['segments'][3],5))
        put('launch_pad_contact',lambda:s.add_pad_point(0,0,0.))
        put('land_pad_contact',lambda:s.add_pad_point(0,6,1.))
        put('static_body_cylinder',lambda:s.add_static(0,3,.5,'cylinder',0))
        put('static_wire_visibility',lambda:s.add_static(0,3,.5,'wire',0,.5,.5,0))
        put('uav_pair',lambda:s.add_pair(0,3,1,3,.5))
        put('vehicle_footprint',lambda:s.add_vehicle_footprint(0,0,v.domain.stop[self.scene['fleet']['vehicles'][0]['depot_stop']],.5,1.,1.))
        put('vehicle_pair',lambda:s.add_vehicle_pair(v.phases[0][0],v.phases[1][0],.5,1.,1.,-1.,-1.))
        put('aircraft_vehicle',lambda:s.add_aircraft_vehicle(0,0,0,0,.5))
        put('ground_obstacles',lambda:s.add_vehicle_static(0,0,.5,0.,0.,.5,'cylinder',0))
        put('parking_capacity',lambda:s.add_parking_capacity(0,0,1,0,v.domain.stop[self.scene['fleet']['vehicles'][0]['depot_stop']]))
        put('vehicle_visibility',lambda:s.add_vehicle_visibility(0,3,0,0,.5,.5,0))
        put('continuous_vehicle',lambda:v.add_point(0,0,.5))
        put('ground_camera',lambda:r.add_camera_point(0,.5))
        self.path.rel.lock(self.path.m.getVars())
        return dict(observed_at=stamp(),checks=checks,all_hooks_materialized=all(x['added_rows']>0 for x in checks),
            same_SCIP_instance=True,tasks=len(self.path.tasks),feasibility_claim=False)

    def materialize(self,requests,batch=16):
        """Batch changes the insertion workload only, never stops optimization."""
        added=0
        for r in requests[:batch]:
            kind=r['kind'];seg=self.curves.by_task[r['task']]['segments'][r['index']] if 'task' in r and kind!='ground_camera' else None
            if kind=='materialize':
                before=len(self.physics.materialized_segments);self.physics.materialize_segment(seg,r['order']);changed=len(self.physics.materialized_segments)>before
            elif kind=='physics':changed=self.physics.add_point(seg,r['s'])
            elif kind=='pad':changed=self.safety.add_pad_point(r['task'],r['index'],r['s'])
            elif kind=='static':
                obstacle=r['obstacle_kind']
                if obstacle=='terrain_clearance':changed=self.physics.add_point(seg,r['s'])
                elif obstacle=='pad':changed=self.safety.add_pad_point(r['task'],r['index'],r['s'])
                else:changed=self.safety.add_static(r['task'],r['index'],r['s'],obstacle,r.get('obstacle',0),r.get('wire_parameter'),r.get('ray'),r.get('surface_sample'))
            elif kind=='vehicle_physics':changed=self.vehicles.add_point(r['carrier'],r['phase_index'],r['s'])
            elif kind=='footprint':changed=self.safety.add_vehicle_footprint(r['carrier'],r['phase_index'],r['category'],r['s'],r['body_u'],r['body_v'])
            elif kind=='vehicle_static':changed=self.safety.add_vehicle_static(r['carrier'],r['phase_index'],r['s'],r['body_u'],r['body_v'],r['body_z'],r['obstacle_kind'],r['obstacle'],r.get('wire_parameter'))
            elif kind=='uav_pair':changed=self.safety.add_pair(r['task_a'],r['index_a'],r['task_b'],r['index_b'],r['fraction'])
            elif kind=='aircraft_vehicle':changed=self.safety.add_aircraft_vehicle(r['task'],r['index'],r['carrier'],r['phase_index'],r['fraction'])
            elif kind=='vehicle_pair':changed=self.safety.add_vehicle_pair(self.vehicles.phases[r['carrier_a']][r['index_a']],self.vehicles.phases[r['carrier_b']][r['index_b']],r['fraction'],r['ua'],r['va'],r['ub'],r['vb'])
            elif kind=='parking':changed=self.safety.add_parking_capacity(r['carrier_a'],r['index_a'],r['carrier_b'],r['index_b'],r['category'])
            elif kind=='vehicle_los':changed=self.safety.add_vehicle_visibility(r['task'],r['index'],r['carrier'],r['phase_index'],r['fraction'],r['ray'],r['surface_sample'])
            elif kind=='ground_camera':changed=self.rendezvous.add_camera_point(r['index'],r['s'])
            else:raise ValueError('Unimplemented required family '+kind)
            added+=int(changed);self.registry.append(r)
            if not self.resource_ok():break
        self.path.rel.lock(self.path.m.getVars())
        (self.output/'materialized_requests.json').write_text(json.dumps(self.registry,ensure_ascii=False,indent=2),encoding='utf-8')
        return added

    def solve(self):
        m=self.path.m;self.trace('unified_constraint_generation_started')
        while self.resource_ok():
            available=psutil.virtual_memory().available
            # Actual SCIP allocator/LP-solver memory guard also applies while
            # a long root LP has not returned a Python event. This is resource
            # admission, not a scientific elapsed-time/node/solution cutoff.
            # Budget from actual allocated memory and current free RAM. Keep
            # .75GiB host floor plus .5GiB for Python-side additions. The earlier
            # percentage reserve stopped with2.56GiB still free; measurements
            # under the compact allocator justify this absolute guard.
            cap=m.getMemTotal()+m.getMemExternEstim()+max(0,available-1.25*2**30)
            m.setRealParam('limits/memory',cap/2**20)
            self.guard.pending=None;self.guard.error=None;m.optimize()
            self.trace('SCIP_returned',dict(error=self.guard.error,requests=len((self.guard.pending or {}).get('requests',[]))))
            if self.guard.error:
                return dict(status='engineering_error',error=self.guard.error,complete=False)
            if self.guard.pending:
                pending=self.guard.pending
                if pending.get('unresolved'):
                    (self.output/'unresolved_acceptance.json').write_text(json.dumps(pending,ensure_ascii=False,indent=2),encoding='utf-8')
                    return dict(status='continuous_acceptance_unresolved',complete=False)
                m.freeTransform();self.round+=1
                if not self.materialize(pending['requests']):return dict(status='no_new_valid_rows',complete=False)
                continue
            status=str(m.getStatus())
            if status!='optimal' or m.getNSols()==0:return dict(status=status,complete=False)
            sol=m.getBestSol();accept=self.acceptance.check(sol)
            if not accept['passed']:return dict(status='postsolve_acceptance_failed',complete=False,acceptance=accept)
            (self.output/f'phase{self.phase}_acceptance.json').write_text(json.dumps(accept,ensure_ascii=False,indent=2),encoding='utf-8')
            previous_directory=Path.cwd()
            try:
                os.chdir(self.output);m.writeBestSol(f'phase{self.phase}.sol')
            finally:os.chdir(previous_directory)
            if self.phase==3:return dict(status='finite_grid_lexicographic_optimal',complete=True,original_continuous_optimality_proved=False)
            objective=self.path.objective if self.phase==1 else self.path.C;opt=float(m.getSolVal(sol,objective))
            m.freeTransform()
            m.addCons(objective<=round(opt) if self.phase==1 else objective<=opt+1e-6,name=f'lexicographic_phase{self.phase}')
            self.phase+=1;m.setObjective(self.path.C if self.phase==2 else self.resources.total_energy)
        return dict(status='resource_protection',complete=False,resource=self.resource_stop)
