"""In-process SCIP evidence: every incumbent and a phase-aware time curve.

No observer thread/process and no solver stopping criterion is introduced.
Heartbeat rows are opportunistic solver events, not a wall-clock guarantee.
Raw solver feasibility is kept distinct from independent physical acceptance.
"""
from __future__ import annotations

import csv
import datetime as dt
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import time

from pyscipopt import Eventhdlr, SCIP_EVENTTYPE


def unrestricted_stopping(model):
    """Reset all early stopping limits, including inherited non-time limits."""
    for key in ('time', 'nodes', 'totalnodes', 'stallnodes', 'memory', 'gap',
                'absgap', 'primal', 'dual', 'solutions', 'bestsol', 'restarts',
                'autorestartnodes', 'softtime'):
        model.resetParam('limits/'+key)
    model.setRealParam('limits/gap', 0.)
    model.setRealParam('limits/absgap', 0.)
    return {k: v for k, v in model.getParams().items() if k.startswith('limits/')}


class SolveTrace(Eventhdlr):
    columns = ('sequence', 'observed_at', 'event', 'priority', 'objective_name', 'refinement',
               'elapsed_wall_s', 'solve_elapsed_s', 'phase_solve_s', 'nodes',
               'primal_bound', 'dual_bound', 'absolute_gap', 'relative_gap',
               'incomplete_components', 'fleet_makespan_seconds',
               'aircraft_energy_Wh', 'solution_id', 'acceptance', 'bound_scope')

    def __init__(self, directory, identity, objectives, variables=None,
                 begin=None, heartbeat_s=1., enforce_storage=True):
        super().__init__()
        self.directory=Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        # Never truncate the evidence of a previous attempt.
        self.jsonl=(self.directory/'progress.jsonl').open('x', encoding='utf-8')
        self.csvfile=(self.directory/'progress.csv').open('x', encoding='utf-8', newline='')
        self.csv=csv.DictWriter(self.csvfile, fieldnames=self.columns)
        self.csv.writeheader(); self.csvfile.flush()
        self.identity=identity; self.objectives=objectives; self.variables=variables
        self.begin=time.perf_counter() if begin is None else begin
        self.heartbeat_s=heartbeat_s; self.last_event=-math.inf
        self.enforce_storage=enforce_storage; self.last_storage=-math.inf
        # Eventhdlr.name is SCIP-owned and locates the native plugin.
        self.priority=None; self.objective_name=None; self.refinement=0; self.sequence=0; self.solution_number=0
        self.solver_offset=0.; self.phase_started=False; self.error=None
        self.current_solution_id=None; self.last_solution_fingerprint=None
        self.mask=(SCIP_EVENTTYPE.BESTSOLFOUND | SCIP_EVENTTYPE.DUALBOUNDIMPROVED |
                   SCIP_EVENTTYPE.LPSOLVED | SCIP_EVENTTYPE.NODESOLVED |
                   SCIP_EVENTTYPE.NODEFOCUSED | SCIP_EVENTTYPE.PRESOLVEROUND)
        self.bound_scope='admitted finite relaxation; full physical bound only after complete acceptance'
        self.write('trace_created', extra={'identity': identity, 'objectives': [x[0] for x in objectives],
            'heartbeat': 'on solver events; may be absent during a long solver operation',
            'storage_floors_GiB': {'C': 10, 'D': 30},
            'official_api': 'https://pyscipopt.readthedocs.io/en/v6.3.0/tutorials/eventhandler.html'})

    def eventinit(self):
        self.model.catchEvent(self.mask, self)

    def eventexit(self):
        self.model.dropEvent(self.mask, self)

    def _number(self, value):
        value=float(value)
        return value if math.isfinite(value) and abs(value)<self.model.infinity() else None

    def _storage(self):
        if not self.enforce_storage or time.perf_counter()-self.last_storage<5:
            return
        self.last_storage=time.perf_counter()
        # The reservation includes the next compressed incumbent and flush.
        for volume, floor in [('C:/',10), ('D:/',30)]:
            if shutil.disk_usage(volume).free < floor*2**30+64*2**20:
                raise OSError('storage floor plus64MiB next-write reservation: '+volume)

    def write(self, event, sol=None, snapshot=False, acceptance='not_independently_checked', extra=None):
        self._storage()
        now=time.perf_counter(); row={k: None for k in self.columns}
        row.update(sequence=self.sequence, observed_at=dt.datetime.now(dt.timezone.utc).isoformat(),
                   event=event, priority=self.priority, objective_name=self.objective_name,refinement=self.refinement,
                   elapsed_wall_s=now-self.begin, acceptance=acceptance,
                   bound_scope=self.bound_scope, solution_id=self.current_solution_id)
        self.sequence+=1
        if self.phase_started:
            seconds=float(self.model.getSolvingTime())
            row.update(phase_solve_s=seconds, solve_elapsed_s=self.solver_offset+seconds,
                       nodes=int(self.model.getNNodes()),
                       primal_bound=self._number(self.model.getPrimalbound()),
                       dual_bound=self._number(self.model.getDualbound()),
                       relative_gap=self._number(self.model.getGap()))
            if row['primal_bound'] is not None and row['dual_bound'] is not None:
                row['absolute_gap']=abs(row['primal_bound']-row['dual_bound'])
        if sol is not None:
            row['objective_vector']=[float(self.model.getSolVal(sol,e)) for _,e,_ in self.objectives]
            for (name,_,_),value in zip(self.objectives,row['objective_vector']):
                if name in self.columns: row[name]=value
            if snapshot:
                values=({name:float(self.model.getSolVal(sol,v)) for name,v in self.variables.items()}
                        if self.variables is not None else
                        {v.name:float(self.model.getSolVal(sol,v)) for v in self.model.getVars(transformed=False)})
                payload=json.dumps(values,sort_keys=True,separators=(',',':'),allow_nan=False).encode('utf-8')
                fingerprint=hashlib.sha256(payload).hexdigest()
                if fingerprint!=self.last_solution_fingerprint:
                    leaf=f'incumbent_{self.solution_number:06d}.json.gz'
                    with (self.directory/leaf).open('xb') as raw:
                        with gzip.GzipFile(fileobj=raw,mode='wb',mtime=0) as compressed:
                            compressed.write(payload)
                        raw.flush();os.fsync(raw.fileno())
                    self.current_solution_id=leaf;self.solution_number+=1
                    self.last_solution_fingerprint=fingerprint
                row.update(solution_id=self.current_solution_id, values_sha256=fingerprint)
        if extra: row.update(extra)
        self.jsonl.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');self.jsonl.flush()
        self.csv.writerow({k:row.get(k) for k in self.columns});self.csvfile.flush()
        if snapshot or event in ('phase_end','trace_closed','interrupted','error'):
            os.fsync(self.jsonl.fileno());os.fsync(self.csvfile.fileno())
        self.last_event=now
        return row

    def phase_start(self,index,name,refinement=0):
        self.priority=index+1;self.objective_name=name;self.phase_started=False
        self.refinement=refinement
        self.current_solution_id=None;self.last_solution_fingerprint=None
        self.write('phase_start',extra={'effective_limits':unrestricted_stopping(self.model)})
        self.phase_started=True

    def eventexec(self,event):
        try:
            kind=event.getType()
            if kind & SCIP_EVENTTYPE.BESTSOLFOUND:
                self.write('incumbent',self.model.getBestSol(),snapshot=True)
            elif kind & SCIP_EVENTTYPE.DUALBOUNDIMPROVED:
                self.write('dual_bound',self.model.getBestSol())
            elif time.perf_counter()-self.last_event>=self.heartbeat_s:
                self.write('progress',self.model.getBestSol())
        except Exception as exc:
            # Losing the requested evidence is a failure, never silent success.
            self.error=repr(exc);self.model.interruptSolve()

    def phase_end(self,accepted):
        row=self.write('phase_end',self.model.getBestSol(),snapshot=self.model.getBestSol() is not None,
                       acceptance='accepted' if accepted else 'rejected_or_unverified',
                       extra={'solver_status':str(self.model.getStatus()),'trace_error':self.error})
        self.solver_offset+=float(self.model.getSolvingTime());self.phase_started=False
        if self.error: raise RuntimeError('Solve trace failed: '+self.error)
        return row

    def close(self,outcome):
        if not self.jsonl.closed:
            try: self.write('trace_closed',extra={'outcome':outcome,'trace_error':self.error})
            finally: self.jsonl.close();self.csvfile.close()
