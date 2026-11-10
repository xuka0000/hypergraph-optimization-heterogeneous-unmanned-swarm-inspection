"""Read-only, version-bound research milestone checks; never grants user identity.

The original owner must read the actual user decision. This module checks the
recorded decision against its source and materials, not scientific validity or
the identity of a person who controls the source file.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

KINDS = {'design', 'benchmark', 'formal_protocol', 'formal_results', 'section', 'final'}
STATUSES = {'draft', 'submitted', 'changes_requested', 'approved', 'superseded'}


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, ensure_ascii=False, sort_keys=True,
                                     separators=(',', ':')).encode('utf-8')).hexdigest()


def material_binding(node):
    return fingerprint({k: v for k, v in node.items() if k != 'review'})


def scope_binding(node):
    return fingerprint({k: node.get(k) for k in
                        ('id', 'kind', 'scope_version', 'protocol_version', 'scope', 'depends_on')})


def resolve(root, value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('missing evidence path')
    path = Path(value)
    return path if path.is_absolute() else root / path


def read_bound_file(root, item):
    if not isinstance(item, dict):
        raise ValueError('evidence must be a path/hash object')
    path = resolve(root, item.get('path'))
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    if digest.hexdigest() != item.get('sha256'):
        raise ValueError('missing or stale SHA256: ' + str(path))
    return path


def verify_user_source(root, source):
    """Check the exact user message/annotation, without inferring approval."""
    if not isinstance(source, dict):
        raise ValueError('approval requires an original user source')
    kind = source.get('kind')
    quote = source.get('quote')
    if not isinstance(quote, str) or len(quote.strip()) < 2:
        raise ValueError('approval requires an exact, substantive user quotation')
    if quote.strip().rstrip('。.!！').lower() in {'继续', '好的', '可以', 'ok', 'continue', '1', 'yes'}:
        raise ValueError('ambiguous continuation is not a milestone approval')
    path = resolve(root, source.get('path'))
    if kind == 'codex_user_message':
        line = source.get('line')
        if type(line) is not int or line < 1:
            raise ValueError('user source requires a 1-based JSONL line')
        # Bind only the selected immutable line; the live session may append.
        with path.open('rb') as handle:
            raw = next((b.rstrip(b'\r\n') for n, b in enumerate(handle, 1) if n == line), None)
        if raw is None or hashlib.sha256(raw).hexdigest() != source.get('sha256'):
            raise ValueError('user message line missing or changed')
        obj = json.loads(raw)
        payload = obj.get('payload', {})
        if obj.get('type') == 'event_msg' and payload.get('type') == 'user_message':
            text = payload.get('message', '')
        elif obj.get('type') == 'response_item' and payload.get('type') == 'message' and payload.get('role') == 'user':
            text = '\n'.join(c.get('text', '') for c in payload.get('content', []) if isinstance(c, dict))
        else:
            raise ValueError('source is not an original user message')
    elif kind == 'pdf_annotation':
        read_bound_file(root, source)
        import fitz
        with fitz.open(path) as doc:
            page = source.get('page')
            if type(page) is not int or not 1 <= page <= len(doc):
                raise ValueError('annotation requires a 1-based PDF page')
            page_object = doc[page - 1]
            annotation = next((a for a in page_object.annots() or []
                               if a.xref == source.get('xref')), None)
            if annotation is None:
                raise ValueError('user annotation missing')
            text = annotation.info.get('content', '')
    else:
        raise ValueError('supported user sources: codex_user_message or pdf_annotation')
    if quote not in text:
        raise ValueError('quotation not present in original user source')
    # Semantics remain the original owner's responsibility, never a keyword vote.


def validate_comparison(node, errors):
    if node.get('kind') not in {'benchmark', 'formal_results'}:
        return
    comparison = node.get('comparison', {})
    scope = node.get('scope', {})
    methods = scope.get('baseline_set', [])
    if not isinstance(methods, list) or not methods or any(not isinstance(m, str) or not m for m in methods) or len(set(methods)) != len(methods):
        errors.append('scope requires the full, unique selected baseline_set')
        return
    results = comparison.get('baselines', [])
    if not isinstance(results, list) or any(not isinstance(r, dict) for r in results):
        errors.append('invalid baseline result records')
        return
    names = [r.get('method') for r in results]
    if len(set(names)) != len(names) or set(names) != set(methods):
        errors.append('all selected public algorithms must be present exactly once')
    artifact_ids = {a.get('id') for a in node.get('artifacts', []) if isinstance(a, dict)}
    if comparison.get('own_method_evidence_artifact') not in artifact_ids:
        errors.append('comparison lacks a bound result for the proposed method')
    for row in results:
        if row.get('status') != 'complete' or row.get('faithful_implementation') is not True:
            errors.append('public baseline incomplete or fidelity unresolved: ' + str(row.get('method')))
        if row.get('evidence_artifact') not in artifact_ids:
            errors.append('baseline lacks a bound result artifact: ' + str(row.get('method')))
    if not scope.get('core_target') or comparison.get('core_target_met') is not True:
        errors.append('agreed core target not met; improve or obtain a scope revision')
    if node.get('kind') == 'benchmark' and scope.get('data_role') not in {'development', 'validation'}:
        errors.append('iterative benchmark must use development/validation data')
    for ablation in comparison.get('ablations', []):
        if not isinstance(ablation, dict) or ablation.get('unit') != 'system_module':
            errors.append('parameter/line changes are not system-module ablations')
            continue
        if not all(ablation.get(k) for k in ('module', 'function', 'inputs', 'outputs', 'intervention')):
            errors.append('ablation requires a functional module boundary')
        if ablation.get('mechanism_evidence_artifact') not in artifact_ids:
            errors.append('ablation requires bound mechanism/call-path evidence')
        if ablation.get('requires_retraining') is True and ablation.get('fair_retraining_completed') is not True:
            errors.append('structural ablation requires its own fair training')


def validate_review_nodes(record, root, transition_to=None):
    try:
        return _validate_review_nodes(record, root, transition_to)
    except (TypeError, ValueError, AttributeError, KeyError) as exc:
        return {'review_errors': ['invalid review record: ' + str(exc)],
                'transition_allowed': False, 'bound_approvals': [],
                'user_identity_authenticated': False, 'scientific_validation': False}


def _validate_review_nodes(record, root, transition_to=None):
    root = Path(root)
    errors, valid_approvals = [], []
    raw_nodes = record.get('review_nodes')
    if not isinstance(raw_nodes, list) or not raw_nodes:
        return {'review_errors': ['review_nodes required; legacy structural validity is not transition approval'],
                'transition_allowed': False, 'bound_approvals': []}
    nodes = {}
    for node in raw_nodes:
        if not isinstance(node, dict) or not isinstance(node.get('id'), str) or not node['id'] or node['id'] in nodes:
            errors.append('invalid or duplicate review node'); continue
        nodes[node['id']] = node
    visiting, visited = set(), set()

    def visit(key):
        if key in visiting:
            errors.append('cyclic node dependency: ' + key); return
        if key in visited:
            return
        visiting.add(key)
        for dep in nodes[key].get('depends_on', []):
            if dep not in nodes:
                errors.append('missing predecessor: ' + str(dep))
            else:
                visit(dep)
        visiting.remove(key); visited.add(key)

    for key, node in nodes.items():
        if node.get('kind') not in KINDS:
            errors.append(key + ': invalid node kind')
        if not isinstance(node.get('depends_on'), list) or any(not isinstance(x, str) for x in node['depends_on']):
            errors.append(key + ': depends_on must be a list of IDs')
            continue
        if not isinstance(node.get('scope'), dict) or not node.get('scope_version') or not node.get('protocol_version'):
            errors.append(key + ': missing versioned scope/protocol')
        if node.get('review', {}).get('status') not in STATUSES:
            errors.append(key + ': invalid review status')
    if errors:
        return {'review_errors': errors, 'transition_allowed': False, 'bound_approvals': []}
    for key in nodes:
        visit(key)
    if errors:
        return {'review_errors': errors, 'transition_allowed': False, 'bound_approvals': []}
    # Validate only the requested dependency closure. Unrelated pending nodes do
    # not block a healthy independent branch; audit mode checks all approvals.
    needed = set()

    def collect(key):
        for dep in nodes[key]['depends_on']:
            if dep not in needed:
                needed.add(dep); collect(dep)

    if transition_to:
        if transition_to not in nodes:
            errors.append('unknown target node: ' + transition_to)
        else:
            collect(transition_to)
            if not needed and nodes[transition_to]['kind'] != 'design':
                errors.append('non-design transition requires reviewed predecessors')
            if nodes[transition_to]['kind'] == 'formal_results' and not any(
                    nodes[d]['kind'] == 'formal_protocol' for d in nodes[transition_to]['depends_on']):
                errors.append('formal results require a reviewed frozen-protocol predecessor')
            if nodes[transition_to]['review']['status'] == 'superseded':
                errors.append('cannot enter a superseded node')
    else:
        needed = {k for k, n in nodes.items() if n['review']['status'] in {'approved', 'submitted'}}
    for key in sorted(needed):
        node = nodes[key]; review = node['review']; local = []
        if review['status'] != 'approved':
            if transition_to:
                errors.append(key + ': explicit user approval pending')
            continue
        artifacts = node.get('artifacts')
        if node.get('blockers'):
            local.append('reviewed node still has unresolved blockers')
        if not isinstance(artifacts, list) or not artifacts:
            local.append('reviewed materials required')
        else:
            ids = [a.get('id') for a in artifacts if isinstance(a, dict)]
            if len(ids) != len(artifacts) or any(not x for x in ids) or len(set(ids)) != len(ids):
                local.append('invalid or duplicate material IDs')
            for artifact in artifacts:
                try:
                    read_bound_file(root, artifact)
                except (ValueError, OSError) as exc:
                    local.append(str(exc))
            required = {
                'formal_protocol': {'frozen_protocol'},
                'formal_results': {'raw_evidence_index', 'primary_results', 'analysis_record'},
                'section': {'section_source', 'section_pdf'},
                'final': {'final_source', 'final_pdf', 'figure_data_chain', 'supplementary_material',
                          'submission_package', 'layout_qa', 'document_binding'},
            }.get(node['kind'], set())
            missing = required - set(ids)
            if missing:
                local.append('missing reviewed delivery materials: ' + ', '.join(sorted(missing)))
        if review.get('material_binding') != material_binding(node):
            local.append('approval binding stale; material or scientific scope changed')
        if review.get('decision') != 'approved' or not review.get('reviewed_at'):
            local.append('explicit dated user decision required')
        annotations = review.get('annotations')
        if not isinstance(annotations, list) or any(not isinstance(a, dict) or a.get('status') not in {'resolved', 'user_accepted'} or not a.get('evidence') for a in annotations):
            local.append('review annotations unresolved or undocumented')
        try:
            verify_user_source(root, review.get('source'))
        except (ValueError, OSError, ImportError, KeyError, TypeError) as exc:
            local.append(str(exc))
        validate_comparison(node, local)
        for dep in node['depends_on']:
            if review.get('predecessor_bindings', {}).get(dep) != material_binding(nodes[dep]):
                local.append('predecessor changed: ' + dep)
        if transition_to:
            for target in [n for n in nodes.values() if key in n['depends_on'] and (n['id'] in needed or n['id'] == transition_to)]:
                if review.get('approved_next', {}).get(target['id']) != scope_binding(target):
                    local.append('next node scope not approved: ' + target['id'])
        if local:
            errors.extend(key + ': ' + e for e in local)
        else:
            valid_approvals.append(key)
    return {'review_errors': errors, 'transition_allowed': bool(transition_to and not errors),
            'bound_approvals': valid_approvals, 'user_identity_authenticated': False,
            'scientific_validation': False}
