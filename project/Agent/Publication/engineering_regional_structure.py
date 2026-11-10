"""Compact exact task/flight path block for the whole regional joint model.

This block alone is a relaxation, NOT the full inspection optimizer. Full
physics, roads, charging and continuous acceptance must be attached before an
entry point can call it a regional solve. All 2133 tasks and 237 tower completion
variables are retained. Flight slots are bounded by actual pad occupancy, not
chosen flights. Task predecessors/successors remain unrestricted.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

from pyscipopt import Model, quicksum
from Agent.Publication.engineering_indexed_constraints import IndexedRelations, AffineRow


SOURCE_SHA = 'd0c668d7cfeea2e7973e6ab387be38e499aea4e935a6530dfaf8900438b8520d'


def add_complete_tower_constraints(model, complete, task_active):
    """Exact binary AND used by both the regional model and module validation."""
    active = list(task_active)
    if not active:
        raise ValueError('A tower must have at least one required observation')
    for value in active:
        model.addCons(complete <= value)
    model.addCons(complete >= 1-len(active)+quicksum(active))


def load_regional(root):
    root = Path(root)
    record = json.loads((root/'research_quality/inspection_regional_model_data.json').read_text('utf-8-sig'))
    source_path = root/record['source']['path']
    if hashlib.sha256(source_path.read_bytes()).hexdigest() != SOURCE_SHA:
        raise ValueError('Source scene identity changed')
    scene = json.loads(source_path.read_text('utf-8-sig'))
    if record['revision'] not in {'5','6','7'}:
        raise ValueError('Contract revision requires a reviewed adapter')
    if hashlib.sha256((root/record['model']['path']).read_bytes()).hexdigest() != record['model']['sha256']:
        raise ValueError('Mathematical source changed since its data binding')
    if record['revision']=='7':
        mode=record['parameters']['overrides']['geometry.capture_domain']
        if mode!='physical_optics_visibility_clearance':raise ValueError('Unknown capture-domain revision')
        # Derived model override only. The immutable geographic source retains
        # the original engineering-template ball for historical reproduction.
        for task in scene['inspection_tasks']:
            task['observation_requirement']['capture_domain']=mode
            task['observation_requirement']['viewpoint_ball_radius_m']=None
            task['observation_requirement']['nominal_viewpoint_role']='initialization_only'
    return scene, record


def pad_slot_bound(scene, record):
    """Sound for new day-ahead flights only, not partly completed replans.

    Each pad is occupied for pre + ascent to protection ceiling + descent +
    post. The carrier is stopped during these intervals. |vz| <= vzmax implies
    both vertical movements take at least delta_h/vzmax. Summing disjoint
    occupancy over one pad's H-second shift gives the integer upper bound.
    """
    cfg = scene['joint_reference_profile']
    overrides = record['parameters']['overrides']
    H = cfg['planning']['horizon_seconds']
    rise = (overrides['aircraft.pad_full_clearance_height_above_vehicle_ground_m']
            - overrides['aircraft.docking_center_height_above_vehicle_ground_m'])
    pre = cfg['recovery']['preparation_seconds']
    post = cfg['recovery']['recovery_seconds']
    minimum = pre + post + 2*rise/cfg['aircraft']['vertical_speed_max_mps']
    if minimum <= 0 or rise < 0:
        raise ValueError('Invalid handling duration/geometry')
    capacities = [int(math.floor(v['handling_positions']*H/minimum))
                  for v in scene['fleet']['vehicles']]
    return dict(horizon_s=H, preparation_s=pre, recovery_s=post,
                dock_to_protection_height_m=rise, minimum_occupancy_s=minimum,
                per_carrier=capacities, total=sum(capacities),
                day_ahead_only=True, removes_feasible_new_sorties=False,
                proof_equation='L_v * (hpre+hpost+2*delta_h/vzmax) <= pads_v*H',
                equation_sources=['eng:eq:pad', 'eng:eq:rendezvous', 'eng:eq:kinematic'],
                counting='optional unassigned slots, not a fixed flight catalogue')


@dataclass
class Task:
    id: str
    active: object
    flight: object
    pred: object
    succ: object
    root: object
    last: object
    has_pred: object
    has_succ: object
    time: object
    dwell: object
    incoming: object
    recovery: object


@dataclass
class Flight:
    carrier: int
    active: object
    first: object
    last: object
    start: object
    end: object


class RegionalPathBlock:
    def __init__(self, task_rows, tower_tasks, slots_per_carrier, horizon=28800.,
                 model=None, relations=None, pre=120., post=240.):
        self.m = model if model is not None else Model('regional_path_block')
        self.rel = relations if relations is not None else IndexedRelations(self.m)
        self.H = horizon
        self.pre, self.post = pre, post
        self.task_rows = task_rows
        self.tower_tasks = tower_tasks
        self.slots_per_carrier = slots_per_carrier
        self.tasks, self.flights, self.towers = [], [], {}
        self.full_model = False

    def var(self, name, upper, binary=False):
        return self.m.addVar(name, vtype='B' if binary else 'C', lb=0, ub=upper)

    @staticmethod
    def eq(name, terms, value=0.):
        return AffineRow(name, terms, value, value)

    def build(self):
        m, rel, H = self.m, self.rel, self.H
        n, L = len(self.task_rows), sum(self.slots_per_carrier)
        self.C = self.var('fleet_return_makespan_s', H)
        for carrier, count in enumerate(self.slots_per_carrier):
            for slot in range(count):
                k = len(self.flights)+1
                b = self.var(f'flight_{k}_active', 1, True)
                first = rel.selector(f'flight_{k}_first', n+1)
                last = rel.selector(f'flight_{k}_last', n+1)
                s, e = self.var(f'flight_{k}_start', H), self.var(f'flight_{k}_end', H)
                for sel in (first, last):
                    m.addCons(sel.value >= b)
                    m.addCons(sel.value <= n*b)
                m.addCons(s >= self.pre*b)
                m.addCons(s <= H*b)
                m.addCons(e >= s)
                m.addCons(e <= H*b)
                m.addCons(e+self.post*b <= self.C)
                self.flights.append(Flight(carrier, b, first, last, s, e))
        for i, row in enumerate(self.task_rows, 1):
            a, root, last, hp, hs = [self.var(f'task_{i}_{name}', 1, True)
                                   for name in ('active', 'root', 'last', 'has_pred', 'has_succ')]
            fid = rel.selector(f'task_{i}_flight', L+1)
            pred, succ = [rel.selector(f'task_{i}_{name}', n+1) for name in ('pred', 'succ')]
            t, d, tin, trec = [self.var(f'task_{i}_{name}', H)
                             for name in ('time', 'dwell', 'incoming', 'recovery')]
            m.addCons(fid.value >= a)
            m.addCons(fid.value <= L*a)
            m.addCons(root+hp == a)
            m.addCons(last+hs == a)
            for sel, has in ((pred, hp), (succ, hs)):
                m.addCons(sel.value >= has)
                m.addCons(sel.value <= n*has)
            m.addCons(d >= float(row['observation_requirement']['min_dwell_s'])*a)
            for var in (t, d, tin):
                m.addCons(var <= H*a)
            m.addCons(trec <= H*last)
            self.tasks.append(Task(row['id'], a, fid, pred, succ, root, last, hp, hs, t, d, tin, trec))

        by_id = {t.id: t for t in self.tasks}
        for tower, ids in self.tower_tasks.items():
            z = self.var(f'tower_{tower}_complete', 1, True)
            aa = [by_id[i].active for i in ids]
            add_complete_tower_constraints(m, z, aa)
            self.towers[tower] = z
        self.objective = len(self.towers)-quicksum(self.towers.values())

        # Lock all variables before deferred rows are considered. Factories
        # below reference arrays of original variables, never incumbent data.
        rel.lock(m.getVars())
        for i, t in enumerate(self.tasks, 1):
            def membership(k, t=t):
                if not k:
                    return [AffineRow('nonzero_flight', [], lo=1.)]
                f = self.flights[k-1]
                return [self.eq('active_flight', [(1, f.active)], 1.),
                        AffineRow('capture_after_launch', [(1, t.time), (-1, f.start)], lo=0.),
                        AffineRow('capture_before_landing', [(1, f.end), (-1, t.time), (-1, t.dwell)], lo=0.)]
            rel.add(t.flight, membership, [], t.active, f'membership_{i}')

            def root_rows(k, i=i, t=t):
                if not k:
                    return [AffineRow('nonzero_root_flight', [], lo=1.)]
                f = self.flights[k-1]
                return [self.eq('first_identity', [(1, f.first.value)], i),
                        self.eq('launch_clock', [(1, t.time), (-1, t.incoming), (-1, f.start)])]
            rel.add(t.flight, root_rows, [], t.root, f'root_{i}')

            def last_rows(k, i=i, t=t):
                if not k:
                    return [AffineRow('nonzero_last_flight', [], lo=1.)]
                f = self.flights[k-1]
                return [self.eq('last_identity', [(1, f.last.value)], i),
                        self.eq('landing_clock', [(1, f.end), (-1, t.time), (-1, t.dwell), (-1, t.recovery)])]
            rel.add(t.flight, last_rows, [], t.last, f'last_{i}')

            def predecessor(k, i=i, t=t):
                if k == 0 or k == i:
                    return [AffineRow('invalid_predecessor', [], lo=1.)]
                p = self.tasks[k-1]
                return [self.eq('active_predecessor', [(1, p.active)], 1.),
                        self.eq('reciprocal_successor', [(1, p.succ.value)], i),
                        self.eq('same_flight', [(1, p.flight.value), (-1, t.flight.value)]),
                        self.eq('transfer_clock', [(1, t.time), (-1, p.time), (-1, p.dwell), (-1, t.incoming)])]
            rel.add(t.pred, predecessor, [], t.has_pred, f'predecessor_{i}')

            def successor(k, i=i, t=t):
                if k == 0 or k == i:
                    return [AffineRow('invalid_successor', [], lo=1.)]
                q = self.tasks[k-1]
                return [self.eq('active_successor', [(1, q.active)], 1.),
                        self.eq('reciprocal_predecessor', [(1, q.pred.value)], i),
                        self.eq('same_flight', [(1, q.flight.value), (-1, t.flight.value)])]
            rel.add(t.succ, successor, [], t.has_succ, f'successor_{i}')

        for k, f in enumerate(self.flights, 1):
            for role, selector in [('first', f.first), ('last', f.last)]:
                def endpoint(i, k=k, role=role):
                    if not i:
                        return [AffineRow('nonzero_endpoint', [], lo=1.)]
                    t = self.tasks[i-1]
                    endpoint_flag = t.root if role == 'first' else t.last
                    return [self.eq('endpoint_active', [(1, t.active)], 1.),
                            self.eq('endpoint_flight', [(1, t.flight.value)], k),
                            self.eq('endpoint_flag', [(1, endpoint_flag)], 1.)]
                rel.add(selector, endpoint, [], f.active, f'flight_{k}_{role}')
        m.setObjective(self.objective, 'minimize')
        return self

    def solve(self):
        raise RuntimeError('This is the path block only. Attach every contract5 family and continuous acceptance before a regional solve.')

    def counts(self):
        return dict(towers=len(self.towers), tasks=len(self.tasks), flights=len(self.flights),
                    original_variables=self.m.getNVars(), explicit_rows=self.m.getNConss(),
                    indexed_relations=len(self.rel.relations),
                    full_joint_model=False, full_region_optimization_started=False,
                    n_m_NI_r_full_model=None)
