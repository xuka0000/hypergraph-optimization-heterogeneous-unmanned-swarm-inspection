"""Recompute the complete fixed batch from original inputs; never run solvers."""
from hypergraph_stress_common import *


def verify_delivery():
    manifest = load(OUT/'input_manifest.json')
    protocol = load(OUT/'protocol.json')
    report = load(OUT/'results.json')
    assert all(sha(ROOT/x['path']) == x['sha256'] for x in protocol['runtime_code'])
    assert ref(OUT/'input_manifest.json') == protocol['input_manifest']
    evidence, finishes, accounting_finishes = [], [], []
    counts, statuses = Counter(), Counter()
    for row in manifest['instances']:
        path = ROOT/row['input']['path']
        assert sha(path) == row['input']['sha256']
        data = load(path)
        assert verify(data) == row['metrics']
        rr = next(r for r in report['rows'] if r['scenario_id'] == row['scenario_id'])
        for method in METHODS:
            path = OUT/'raw'/f"{row['scenario_id']}__{method}.json"
            raw = load(path)
            value = evaluate(data, raw['selected'])
            assert value['feasible'] and value['objective'] == raw['validation']['objective'], (path, 'decision')
            assert raw['input'] == row['input']
            assert raw['runner_sha256'] == protocol['runtime_code'][1]['sha256']
            assert raw['native_source_unchanged'] is True
            # Native internal algorithm time is nested inside native_call_seconds.
            if method == 'SCIP':
                components = ['build_seconds', 'solver_seconds', 'output_seconds']
            elif method == 'Streaming':
                components = ['build_seconds', 'native_call_seconds', 'output_seconds']
            else:
                components = ['python_conversion_seconds', 'native_call_seconds', 'output_seconds']
            assert math.isclose(raw['total_seconds'], sum(raw[k] for k in components), abs_tol=1e-7), (path, 'timing')
            assert raw['total_seconds'] >= 0
            assert rr['methods'][method]['objective'] == value['objective']
            assert rr['methods'][method]['total_seconds'] == raw['total_seconds']
            account_path = OUT/'logs'/f"{row['scenario_id']}__{method}_accounting.json"
            account = load(account_path)
            assert account['exit_code'] == 0 and account['engineering_stop'] is None
            assert account['peak_sampled_rss_bytes'] < 2560*1024**2
            if method == 'SCIP':
                assert raw['status'] in ['gaplimit', 'optimal'] and raw['gap_target_met']
                assert raw['gap'] <= .05+1e-8 and raw['time_limit_seconds'] is None
                assert math.isclose(value['objective'], raw['objective_reported'], abs_tol=1e-6)
                assert raw['upper_bound']+1e-6 >= value['objective']
                statuses[raw['status']] += 1
            else:
                comp = rr['comparisons'][method]
                scip_value = rr['methods']['SCIP']['objective']
                assert comp['objective_difference'] == value['objective']-scip_value
                assert comp['quality95_and_speed2x'] == (
                    value['objective'] >= .95*scip_value and
                    rr['methods']['SCIP']['total_seconds'] >= 2*raw['total_seconds'])
                assert value['objective'] <= rr['methods']['SCIP']['upper_bound']+1e-6
            evidence.append(dict(scenario_id=row['scenario_id'], method=method,
                objective=value['objective'], total_seconds=raw['total_seconds'], feasible=True,
                gap=raw.get('gap'), gap_target_met=raw.get('gap_target_met'),
                evidence=ref(path), input=row['input'], accounting=ref(account_path)))
            counts[method] += 1
            finishes.append(raw['observed_at'])
            accounting_finishes.append(account['observed_at'])
    assert len(evidence) == 236
    assert all(g['valid_completed_pairs'] == 3 and g['complete'] for g in report['controlled_groups'])
    result = dict(observed_at=now(),
        scope='All 59 fixed cases and 236 raw outputs; original-input feasibility and objective recomputation; no solver rerun',
        passed=True, verified_runs=len(evidence), planned_runs=236, method_counts=dict(counts),
        scip_status_counts=dict(statuses), scip_gap05_complete=59, evidence_rows=evidence,
        frozen_runtime_hashes_unchanged=True, last_solver_result_at=max(finishes),
        last_process_accounting_at=max(accounting_finishes),
        report=ref(OUT/'results.json'), text_report=ref(OUT/'压力测试报告.txt'),
        verifier=ref(Path(__file__).resolve()))
    save(OUT/'delivery_validation.json', result)
    return result


if __name__ == '__main__':
    result = verify_delivery()
    print({k: v for k, v in result.items() if k not in ['evidence_rows', 'report', 'text_report', 'verifier']})
