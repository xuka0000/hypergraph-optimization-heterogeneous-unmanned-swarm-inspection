"""Exact finite indexed affine relations, materialized by SCIP on demand.

All categories remain available. A binary code replaces a one-hot selector.
The handler checks every integral candidate, including primal heuristics, and
adds globally valid conditional rows at LP enforcement. No incumbent is accepted
on the basis of an unexamined lookup. This is an encoding, not an optimizer.

Reference: PySCIPOpt lazy-constraint tutorial and SCIP CONS fundamental callbacks.
https://pyscipopt.readthedocs.io/en/v6.3.0/tutorials/lazycons.html
https://www.scipopt.org/scip/doc/html/CONS.php
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable
import math

from pyscipopt import Conshdlr, SCIP_RESULT, quicksum


@dataclass
class Selector:
    name: str
    size: int
    bits: list
    value: object

    def mismatch(self, category):
        return quicksum(1-b if category & (1 << k) else b
                        for k, b in enumerate(self.bits))


@dataclass
class AffineRow:
    """lo <= constant + sum(coefficient * variable) <= hi."""
    name: str
    terms: list
    lo: float | None = None
    hi: float | None = None
    constant: float = 0.

    def expression(self):
        return quicksum(c*v for c, v in self.terms) + self.constant

    def bounds(self):
        low = high = self.constant
        for c, v in self.terms:
            a, b = c*v.getLbOriginal(), c*v.getUbOriginal()
            low += min(a, b)
            high += max(a, b)
        if not all(math.isfinite(x) and abs(x) < 1e19 for x in (low, high)):
            raise ValueError(f'Finite valid bounds required for {self.name}')
        return low, high

    def violation(self, value):
        z = self.constant + sum(c*value(v) for c, v in self.terms)
        return max(0., self.lo-z if self.lo is not None else 0.,
                   z-self.hi if self.hi is not None else 0.)


class IndexedRelations(Conshdlr):
    """A registry of (binary selector, optional gate, affine row factory).

    Row factories must be pure and enumerate the complete relation for a
    category. Variable bounds must cover the original feasible domain. Every
    referenced variable is conservatively locked; dual reductions are disabled
    because not-yet-materialized rows are nevertheless part of the problem.
    """
    def __init__(self, model):
        self.relations = []
        self.selectors = []
        self.materialized = set()
        self.cut_rows = 0
        self.check_calls = 0
        self.rejected_candidates = 0
        self.transform_resets = 0
        self.locked = set()
        self.model_ref = model
        model.setBoolParam('misc/allowstrongdualreds', False)
        model.setBoolParam('misc/allowweakdualreds', False)
        model.setBoolParam('presolving/donotaggr', True)
        model.setBoolParam('presolving/donotmultaggr', True)
        model.setIntParam('presolving/maxrounds', 0)
        model.setIntParam('misc/usesymmetry', 0)
        model.includeConshdlr(self, 'indexed_affine',
            'exact binary-indexed affine finite relations',
            enfopriority=-10, chckpriority=-10, needscons=False)

    def lock(self, variables):
        for v in variables:
            key = v.name
            if key not in self.locked:
                self.model_ref.addVarLocks(v, 1, 1)
                self.locked.add(key)

    def enable_primal_presolve(self):
        """Allow equivalence-preserving native primal reductions.

        The registry owns original variables, and SCIP maps those to fixed,
        aggregated or transformed variables in getSolVal and added rows.
        There is no per-constraint mutable data to copy in CONSTRANS
        (needscons=False). Factories remain pure in original problem space.
        Hidden rows still forbid dual reductions: zero objective in the
        visible relaxation does not imply that a variable can be discarded.
        """
        m=self.model_ref
        m.setIntParam('presolving/maxrounds',-1)
        m.setBoolParam('presolving/donotaggr',False)
        m.setBoolParam('presolving/donotmultaggr',False)
        m.setBoolParam('misc/allowstrongdualreds',False)
        m.setBoolParam('misc/allowweakdualreds',False)

    def selector(self, name, size):
        if size < 1:
            raise ValueError('Empty indexed domain')
        m = self.model_ref
        bits = [m.addVar(vtype='B', name=f'{name}_bit{k}')
                for k in range((size-1).bit_length())]
        value = m.addVar(vtype='I', lb=0, ub=size-1, name=name)
        m.addCons(value == quicksum((1 << k)*b for k, b in enumerate(bits)),
                  name=f'{name}_binary_code')
        out = Selector(name, size, bits, value)
        self.selectors.append(out)
        self.lock([value, *bits])
        return out

    def add(self, selector, factory: Callable[[int], list[AffineRow]],
            variables, gate=None, name=None):
        self.lock(variables)
        if gate is not None:
            self.lock([gate])
        self.relations.append((selector, gate, factory,
                               name or f'relation_{len(self.relations)}'))

    def binary_selector(self,name,variable):
        """Alias an existing0/1 decision; no duplicate index/binary variable."""
        if variable.getLbOriginal()<0 or variable.getUbOriginal()>1 or variable.vtype()=='CONTINUOUS':
            raise ValueError('An original binary decision is required')
        selector=Selector(name,2,[variable],variable)
        self.selectors.append(selector);self.lock([variable]);return selector

    def _category(self, selector, value):
        raw = value(selector.value)
        rounded = int(round(raw))
        if abs(raw-rounded) > 1e-6:
            return None
        if not 0 <= rounded < selector.size:
            raise ArithmeticError('Integral selector outside its original domain')
        # An integral index with fractional bits is still a relaxation point.
        if any(abs(value(b)-((rounded >> k) & 1)) > 1e-6
               for k, b in enumerate(selector.bits)):
            return None
        return rounded

    def violations(self, solution=None):
        m = self.model_ref
        value = lambda v: m.getSolVal(solution, v)
        categories = {}
        for rid, (sel, gate, factory, name) in enumerate(self.relations):
            if gate is not None and value(gate) < 1-1e-6:
                continue
            if sel.name not in categories:
                categories[sel.name] = self._category(sel, value)
            category = categories[sel.name]
            if category is None:
                continue
            rows = factory(category)
            if any(row.violation(value) > 1e-6 for row in rows):
                yield rid, category, rows

    def conscheck(self, constraints, solution, check_integrality,
                  check_lp_rows, print_reason, completely, **kwargs):
        self.check_calls += 1
        invalid = next(self.violations(solution), None) is not None
        self.rejected_candidates += int(invalid)
        return {'result': SCIP_RESULT.INFEASIBLE if invalid else SCIP_RESULT.FEASIBLE}

    def _enforce(self):
        added = False
        unresolved = False
        for rid, category, rows in self.violations():
            key = (rid, category)
            if key in self.materialized:
                unresolved = True
                continue
            sel, gate, _, name = self.relations[rid]
            mismatch = sel.mismatch(category)
            if gate is not None:
                mismatch += 1-gate
            for k, row in enumerate(rows):
                expr = row.expression()
                low, high = row.bounds()
                if row.lo is not None:
                    self.model_ref.addCons(expr + max(0., row.lo-low)*mismatch >= row.lo,
                        name=f'{name}_c{category}_r{k}_lo', initial=True)
                    self.cut_rows += 1
                if row.hi is not None:
                    self.model_ref.addCons(expr - max(0., high-row.hi)*mismatch <= row.hi,
                        name=f'{name}_c{category}_r{k}_hi', initial=True)
                    self.cut_rows += 1
            self.materialized.add(key)
            added = True
        if added:
            return {'result': SCIP_RESULT.CONSADDED}
        if unresolved:
            # The added linear rows own their numerical tolerances. Never claim
            # an explicitly violated indexed relation feasible.
            return {'result': SCIP_RESULT.INFEASIBLE}
        return {'result': SCIP_RESULT.FEASIBLE}

    def consenfolp(self, constraints, n_useful_conss, sol_infeasible):
        return self._enforce()

    def consenfops(self, constraints, n_useful_conss, sol_infeasible,
                  obj_infeasible):
        return self._enforce()

    def conslock(self, constraint, locktype, nlockspos, nlocksneg):
        # All original variables were locked at registration, including those
        # used by categories whose rows have not yet been instantiated.
        pass

    def consexit(self, constraints):
        # Rows added while solving belong to SCIP's transformed problem.
        # freeTransform (between lexicographic passes) discards those rows.
        # Their categories must be eligible for materialization again.
        self.materialized.clear()
        self.transform_resets += 1

    def statistics(self):
        return dict(selectors=len(self.selectors), relations=len(self.relations),
                    materialized_categories=len(self.materialized),
                    materialized_affine_rows=self.cut_rows,
                    candidate_checks=self.check_calls,
                    rejected_candidates=self.rejected_candidates,
                    transform_resets=self.transform_resets,
                    representation='complete finite indexed constraints, lazily materialized',
                    scalar_hypergraph_incidence_exported=False)
