"""Same frozen SCIP MILP, no time limit; stop on its first positive incumbent.

The callback observes SCIP decisions and requests termination. It supplies no
solution, cuts, variable ordering, warm start, or additional search heuristic.
"""
from pathlib import Path
import argparse, datetime, gc, json, os, subprocess, sys, time, traceback
from pyscipopt import Model, SCIP_EVENTTYPE, quicksum
from run_hypergraph_scip_demo import ROOT, BRIDGE, OUT as BASE, load, save, sha, evaluate

OUT = BASE / 'SCIP首次有效决策'
PREFIX = 'hypergraph_scip_first_decision'


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def solve(data, event_path=None, verbose=False):
    t0 = time.perf_counter()
    model = Model(data['scenario_id'])
    if not verbose:
        model.hideOutput()
    # Leave limits/time at its unbounded default. Memory is a resource safeguard.
    model.setIntParam('parallel/maxnthreads', 1)
    model.setIntParam('randomization/randomseedshift', 0)
    model.setRealParam('limits/memory', 6144.)
    assert model.getParam('limits/time') == 1e20
    jobs = data['jobs']
    xs = [model.addVar(name=j['id'], vtype='B') for j in jobs]
    incident = {r['id']: [] for r in data['resources']}
    for i, j in enumerate(jobs):
        for r in j['resources']:
            incident[r].append(i)
    for r in data['resources']:
        if incident[r['id']]:
            model.addCons(quicksum(xs[i] for i in incident[r['id']]) <= r['capacity'], name=r['id'])
    model.setObjective(quicksum(j['value'] * x for j, x in zip(jobs, xs)), 'maximize')
    first = {}
    events = []

    def on_incumbent(m, event):
        elapsed = time.perf_counter() - t1
        sol = m.getBestSol()
        value = float(m.getSolObjVal(sol))
        entry = {'objective': value, 'solver_wall_seconds': elapsed,
                 'solver_internal_seconds': m.getSolvingTime(), 'observed_at': now()}
        events.append(entry)
        if value > 0 and not first:
            first.update(entry)
            # Persist the event before requesting safe solver termination.
            if event_path:
                save(event_path, {'first_positive': first, 'events': events,
                                  'termination_requested': True})
            m.interruptSolve()

    model.attachEventHandlerCallback(on_incumbent, [SCIP_EVENTTYPE.BESTSOLFOUND])
    t1 = time.perf_counter()
    model.optimize()
    t2 = time.perf_counter()
    sol = model.getBestSol()
    assert sol is not None, 'SCIP returned without any feasible incumbent'
    selected = [j['id'] for j, x in zip(jobs, xs) if model.getSolVal(sol, x) > .5]
    value = model.getSolObjVal(sol)
    dual_raw = model.getDualbound()
    gap_raw = model.getGap()
    result = {'method': 'SCIP_MILP_first_positive', 'scenario_id': data['scenario_id'],
              'selected': selected, 'selected_count': len(selected), 'reported_objective': value,
              'status': str(model.getStatus()), 'optimal': str(model.getStatus()) == 'optimal',
              'dual_bound_raw': dual_raw, 'dual_bound': None if model.isInfinity(abs(dual_raw)) else dual_raw,
              'scip_gap_raw': gap_raw, 'scip_gap': None if model.isInfinity(abs(gap_raw)) else gap_raw,
              'primal_bound': model.getPrimalbound(), 'nodes_total': model.getNTotalNodes(),
              'solution_count': model.getNSols(), 'first_positive_event': first or None,
              'events': events, 'solver_seconds': t2-t1, 'build_seconds': t1-t0,
              'solver_internal_seconds': model.getSolvingTime(),
              'time_limit_seconds': None, 'time_limit_parameter': model.getParam('limits/time'),
              'memory_guard_MiB': model.getParam('limits/memory'), 'warm_start': False,
              'original_variables': len(xs), 'original_nonempty_constraints': sum(bool(v) for v in incident.values())}
    t3 = time.perf_counter()
    result.update(total_seconds=t3-t0, output_seconds=t3-t2,
                  first_positive_including_build_seconds=(t1-t0+first['solver_wall_seconds']) if first else None)
    model.freeProb()
    assert value > 0 and selected and first, 'Required positive decision was not obtained'
    return result


def regression():
    OUT.mkdir(exist_ok=True)
    fixture = {'scenario_id': 'callback_regression', 'resources': [{'id': 'r', 'capacity': 1}],
               'jobs': [{'id': 'a', 'resources': ['r'], 'value': 3},
                        {'id': 'b', 'resources': ['r'], 'value': 5}]}
    r = solve(fixture)
    v = evaluate(fixture, r['selected'])
    assert v['feasible'] and v['objective'] in (3, 5)
    assert v['objective'] == r['reported_objective']
    assert r['first_positive_event']['objective'] > 0 and r['time_limit_seconds'] is None
    save(OUT/'runner_regression.json', {'observed_at': now(), 'passed': True,
        'purpose': 'Bounded callback/termination adapter regression only; not a scientific replicate',
        'fixture': fixture, 'result': r, 'validation': v,
        'source': 'https://pyscipopt.readthedocs.io/en/v6.2.1/tutorials/eventhandler.html',
        'runner_sha256': sha(Path(__file__))})
    print(json.dumps({'regression_passed': True, 'objective': v['objective'], 'status': r['status']}), flush=True)


def run():
    sys.path.insert(0, str(BRIDGE))
    from check_research_stage import require_transition
    ident = load(OUT/'experiment_identity.json')
    assert ident['status'] == 'prepared_authorized' and ident['pid'] is None
    assert not (OUT/'results.json').exists()
    assert load(OUT/'runner_regression.json')['runner_sha256'] == sha(Path(__file__))
    manifest = load(OUT/'input_manifest.json')
    for row in manifest['instances']:
        assert sha(ROOT/row['path']) == row['sha256']
    receipt = require_transition(ROOT, f'research_quality/{PREFIX}_review_nodes.json', PREFIX)
    save(OUT/'transition_receipt.json', receipt)
    p = subprocess.run([sys.executable, '-X', 'utf8', '-B', str(BRIDGE/'check_experiment_preflight.py'),
                        '--root', str(ROOT), '--record', str(ROOT/f'research_quality/{PREFIX}_preflight.json'),
                        '--next-step', 'public_baselines'], capture_output=True, text=True, encoding='utf-8', timeout=30)
    (OUT/'preflight_receipt.json').write_text(p.stdout, encoding='utf-8')
    assert p.returncode == 0 and json.loads(p.stdout)['preflight_allowed'], p.stdout+p.stderr
    # Exclusive claim prevents accidental duplicate launch of this experiment.
    with (OUT/'run_claim.json').open('x', encoding='utf-8') as f:
        json.dump({'pid': os.getpid(), 'started_at': now()}, f)
    (OUT/'raw').mkdir(exist_ok=True)
    ident.update(status='running', pid=os.getpid(), started_at=now())
    save(OUT/'experiment_identity.json', ident)
    rows = []
    try:
        for source in manifest['instances']:
            ident.update(current_instance=source['scenario_id'], last_progress_at=now())
            save(OUT/'experiment_identity.json', ident)
            print('START '+source['scenario_id'], flush=True)
            data = load(ROOT/source['path'])
            r = solve(data, OUT/'raw'/(source['scenario_id']+'_first_positive_event.json'), verbose=True)
            r.update(observed_at=now(), input_sha256=source['sha256'])
            v = evaluate(data, r['selected'])
            r['validation'] = v
            assert v['feasible'] and v['objective'] == r['reported_objective'] and v['objective'] > 0
            save(OUT/'raw'/(source['scenario_id']+'_SCIP.json'), r)
            old = BASE/'论文原始实例_匹配2025_2026'/'raw'/(source['scenario_id']+'_HeiHGM_Streaming.json')
            hg = load(old)
            rows.append({'scenario_id': source['scenario_id'], 'n': len(data['jobs']),
                         'scip_objective': v['objective'], 'scip_selected_count': len(r['selected']),
                         'scip_first_positive_solver_seconds': r['first_positive_event']['solver_wall_seconds'],
                         'scip_total_seconds': r['total_seconds'], 'scip_build_seconds': r['build_seconds'],
                         'scip_status': r['status'], 'scip_proved_optimal': r['optimal'],
                         'scip_upper_bound': r['dual_bound'], 'scip_gap': r['scip_gap'],
                         'hypergraph_objective': hg['validation']['objective'],
                         'hypergraph_total_seconds': hg['total_seconds'], 'hypergraph_selected_count': len(hg['selected']),
                         'hypergraph_result_reused': True, 'hypergraph_evidence_sha256': sha(old),
                         'all_feasible': True, 'input_sha256': source['sha256']})
            save(OUT/'partial_results.json', rows)
            print(json.dumps(rows[-1], ensure_ascii=False), flush=True)
            ident.update(completed_instances=len(rows), last_progress_at=now())
            save(OUT/'experiment_identity.json', ident)
            del data, r, v, hg
            gc.collect()
        save(OUT/'results.json', {'experiment_id': PREFIX, 'observed_at': now(), 'rows': rows,
                                 'runner_sha256': sha(Path(__file__)), 'protocol_sha256': sha(OUT/'protocol.json'),
                                 'note': 'First-positive-incumbent stopping, not optimization to proven optimum; old native results retained without rerun.'})
        ident.update(status='complete', pid=None, completed_at=now())
    except BaseException as e:
        save(OUT/'failure.json', {'error': repr(e), 'traceback': traceback.format_exc(), 'observed_at': now()})
        ident.update(status='failed', pid=None, last_progress_at=now())
        raise
    finally:
        save(OUT/'experiment_identity.json', ident)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['regression', 'run'])
    args = parser.parse_args()
    regression() if args.mode == 'regression' else run()
