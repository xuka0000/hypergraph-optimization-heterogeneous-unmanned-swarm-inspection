"""Optimality accounting for one shared inspection model.

Lexicographic passes change the objective and retain proven objective levels;
they never fix routes, assignments, trajectories, or resource schedules.
No finite time limit is imposed here. Memory/user interrupts are unfinished.
"""
from __future__ import annotations
import hashlib
import json
import time
from pathlib import Path


REQUIRED_CLOSURES = (
    'full_task_and_resource_domain', 'full_vehicle_road_domain',
    'continuous_ackermann_motion', 'full_source_weather_horizon',
    'continuous_flight_and_attitude', 'continuous_image_quality',
    'continuous_obstacle_and_visibility', 'continuous_pairwise_airspace',
    'energy_integration_error_bound', 'executable_recovery_witness',
    'charge_preemption_and_soc_curve', 'battery_and_resource_conservation',
    'lexicographic_objective', 'independent_evaluator_parity',
)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require_complete_model(root, record_path):
    """Enforce the user's model-first instruction, not a new approval gate."""
    root=Path(root); path=root/record_path
    if not path.exists():
        raise RuntimeError('Full-model implementation receipt is absent; construct/repair before optimization')
    record=json.loads(path.read_text(encoding='utf-8'))
    unresolved=[]
    required=list(REQUIRED_CLOSURES)
    if record.get('rolling_scope_extension'):
        required.extend(['full_regional_task_domain','causal_forecast_and_nonanticipativity',
                         'execution_state_and_takeover'])
    for key in required:
        row=record.get('closures',{}).get(key,{})
        if row.get('status')!='verified' or not row.get('evidence'):
            unresolved.append(key); continue
        for item in row['evidence']:
            p=root/item['path']
            if not p.is_file() or digest(p)!=item['sha256']:
                unresolved.append(key+':stale_evidence');break
    for item in record.get('implementation_files',[]):
        p=root/item['path']
        if not p.is_file() or digest(p)!=item['sha256']:
            unresolved.append('stale_implementation:'+item['path'])
    if not record.get('implementation_files'):
        unresolved.append('unbound_implementation')
    if unresolved:
        raise RuntimeError('Full model is not ready; no incomplete optimality run: '+', '.join(unresolved))
    return record


def solve_lexicographic(model, objectives, emit, feasible_acceptance=None, trace=None,
                        refine=None):
    """Run all objective priorities with relative and absolute gap zero.

SCIP's ``optimal`` is a numerical certificate within its feasibility and
optimality tolerances. It is not exact-real proof of a semi-infinite problem.
Each objective is a (name, SCIP expression, numerical tie tolerance) tuple.
An independently rejected incumbent cannot close a priority or the task.
An acceptance callback may return a report with ``accepted`` and ``witnesses``.
The optional refine callback adds only necessary constraints to this same
model and returns their count. No physical decision is fixed. An inconclusive
check without a new constraint returns unfinished, never accepted. Refinement
has no arbitrary iteration cap; duplicate witnesses must return zero.
"""
    from Agent.Publication.engineering_solve_trace import unrestricted_stopping
    effective_limits=unrestricted_stopping(model)
    rows=[]; rounds=[]; begin=time.perf_counter()
    for index,(name,objective,tie_tolerance) in enumerate(objectives):
        if index:model.freeTransform()
        model.setObjective(objective,'minimize')
        refinement=0
        while True:
            if trace:trace.phase_start(index,name,refinement=refinement)
            started=time.perf_counter();model.optimize()
            sol=model.getBestSol();status=str(model.getStatus())
            row=dict(priority=index+1,name=name,status=status,refinement=refinement,
                     wall_seconds=time.perf_counter()-started,
                     solver_seconds=float(model.getSolvingTime()),
                     nodes=int(model.getNNodes()),solutions=int(model.getNSols()),
                     primal_bound=float(model.getPrimalbound()),
                     dual_bound=float(model.getDualbound()),gap=float(model.getGap()),
                     tie_tolerance=float(tie_tolerance),
                     numerical_optimal=status=='optimal',accepted=False)
            report={}
            if sol is not None:
                row['objective_value']=float(model.getSolVal(sol,objective))
                check_started=time.perf_counter()
                response=feasible_acceptance(sol) if feasible_acceptance else False
                report=response if isinstance(response,dict) else {'accepted':bool(response)}
                row['accepted']=report.get('accepted') is True
                row['acceptance_seconds']=time.perf_counter()-check_started
                row['acceptance_report']=report
            if trace:trace.phase_end(row['accepted'])
            row['effective_limits']=effective_limits
            rounds.append(row)
            if len(rows)==index:rows.append(row)
            else:rows[index]=row
            emit(dict(priorities=rows,refinement_history=rounds,complete=False,
                      total_wall_seconds=time.perf_counter()-begin))
            if status=='optimal' and row['accepted']:break
            if status!='optimal' or refine is None or not report.get('witnesses'):
                return dict(priorities=rows,refinement_history=rounds,complete=False,
                            reason='interrupted_or_unverified',total_wall_seconds=time.perf_counter()-begin)
            # All solution values and the rejection report have already been
            # exported. Free only the SCIP transformation, not original vars.
            del sol
            model.freeTransform()
            added=int(refine(report))
            row['new_witness_constraints']=added
            if trace:trace.write('refinement',extra={'refinement':refinement,
                                                    'new_witness_constraints':added})
            if added<=0:
                result=dict(priorities=rows,refinement_history=rounds,complete=False,
                            reason='unresolved_without_new_constraint',total_wall_seconds=time.perf_counter()-begin)
                emit(result);return result
            refinement+=1
        if index+1<len(objectives):
            bound=row['objective_value']+float(tie_tolerance)
            model.freeTransform()
            model.addCons(objective<=bound,name=f'lex_priority_{index+1}_bound')
    result=dict(priorities=rows,refinement_history=rounds,complete=True,total_wall_seconds=time.perf_counter()-begin,
                certificate='numerical global optimum of the admitted transcription at declared lex tolerances')
    emit(result);return result
