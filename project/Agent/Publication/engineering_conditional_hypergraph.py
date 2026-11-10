"""Experimental conditional factor-consensus solver, NOT a public baseline.

ADMM message equations follow Derbinsky et al. (2013), equations5--7.
Nonlinear local projections here are inexact Newton projections, not the
paper's Three-Weight Algorithm and not globally solved proximal problems.
All discrete choices remain variables. Conditional affine rows are added with
valid original-bound big M values; changing a selector can deactivate a row.
Only the original algebra/indexed/continuous validator may accept a solution.
"""
from array import array
import math,sys
from pathlib import Path
_runtime=Path(__file__).resolve().parents[2]/'.codex/tools/hypergraph_runtime'
if _runtime.exists():sys.path.insert(0,str(_runtime))
import numpy as np
from numba import njit
from pyscipopt import Model, Expr, Variable


class Tape:
    """Postorder expression tapes and unique scalar incidences, streamed at build."""
    def __init__(self):
        self.op=array('b'); self.left=array('i'); self.right=array('i'); self.par=array('d')
        self.ptr=array('q',[0]); self.pinptr=array('q',[0]); self.ids=array('i')
        self.lo=array('d'); self.hi=array('d'); self.enabled=array('b')
        self.names=[]; self.con_indices={}; self.varnames=[]; self.lb=[]; self.ub=[]; self.integer=[]

    def node(self,op,left=-1,right=-1,par=0.):
        k=len(self.op); self.op.append(op); self.left.append(left); self.right.append(right); self.par.append(par); return k

    def add(self,cons,name):
        local={}
        def var(v):
            i=v.getIndex()
            if i not in local:
                local[i]=len(self.ids); self.ids.append(i)
            return self.node(1,local[i])
        def const(c):return self.node(0,par=float(c))
        def expr(e):
            if isinstance(e,Variable):return var(e)
            if isinstance(e,(int,float,np.number)):return const(e)
            if isinstance(e,Expr):
                terms=[]
                for term,c in e.terms.items():
                    if not c:continue
                    factors=[var(v) for v in term]
                    if not factors:terms.append(const(c));continue
                    q=factors[0]
                    for v in factors[1:]:q=self.node(3,q,v)
                    if c!=1:q=self.node(3,q,const(c))
                    terms.append(q)
                if not terms:return const(0)
                q=terms[0]
                for v in terms[1:]:q=self.node(2,q,v)
                return q
            op=e.getOp()
            if op=='var':return var(e.children[0])
            if op=='const':return const(e.number)
            if op=='sum':
                q=const(e.constant)
                for c,v in zip(e.coefs,e.children):
                    a=expr(v)
                    if c!=1:a=self.node(3,a,const(c))
                    q=self.node(2,q,a)
                return q
            if op=='prod':
                q=const(e.constant)
                for v in e.children:q=self.node(3,q,expr(v))
                return q
            if op=='**':return self.node(4,expr(e.children[0]),par=e.expo)
            if op=='sqrt':return self.node(4,expr(e.children[0]),par=.5)
            ops={'sin':5,'cos':6,'exp':7,'log':8,'abs':9}
            if op in ops:return self.node(ops[op],expr(e.children[0]))
            raise NotImplementedError('Expression operator '+str(op))
        last=expr(cons.expr)
        assert last==len(self.op)-1
        self.ptr.append(len(self.op)); self.pinptr.append(len(self.ids))
        self.lo.append(-np.inf if cons._lhs is None else cons._lhs)
        self.hi.append(np.inf if cons._rhs is None else cons._rhs)
        # 0=removed, 1=affine (exact projection), 2=nonlinear (inexact).
        self.enabled.append(1 if cons.expr.degree()<=1 else 2); self.names.append(name)
        return len(self.names)-1

    def arrays(self):
        # Copies deliberately release resizable array buffers for lazy additions.
        return tuple(np.array(x) for x in (self.op,self.left,self.right,self.par,self.ptr,self.pinptr,self.ids,self.lo,self.hi,self.enabled))

    def counts(self):
        degree=np.bincount(self.ids,minlength=len(self.varnames));rank=np.diff(self.pinptr)
        return dict(n=len(self.varnames),m=len(self.names),N_I=len(self.ids),r=int(max(rank,default=0)),
                    maximum_variable_degree=int(max(degree,default=0)),isolated_variables=int(sum(degree==0)),
                    tape_nodes=len(self.op),scope='currently materialized explicit algebra only; excludes latent selector alternatives and continuous oracle families')


class RecordingModel(Model):
    """SCIP is used for model construction and validation, not message updates."""
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw); self.tape=Tape()
    def addVar(self,*a,**kw):
        v=super().addVar(*a,**kw);t=self.tape
        assert v.getIndex()==len(t.varnames)
        t.varnames.append(v.name);t.lb.append(v.getLbOriginal());t.ub.append(v.getUbOriginal());t.integer.append(v.vtype()!='CONTINUOUS')
        return v
    def addCons(self,cons,*a,**kw):
        c=super().addCons(cons,*a,**kw)
        self.tape.con_indices[c.name]=self.tape.add(cons,c.name)
        return c
    def delCons(self,c):
        i=self.tape.con_indices[c.name]; self.tape.enabled[i]=0
        return super().delCons(c)
    def chgVarLb(self,v,value):
        out=super().chgVarLb(v,value)
        self.tape.lb[v.getIndex()]=v.getLbOriginal();return out
    def chgVarUb(self,v,value):
        out=super().chgVarUb(v,value)
        self.tape.ub[v.getIndex()]=v.getUbOriginal();return out


@njit(cache=True)
def evaluate_row(a,b,p,q,op,left,right,par,pinids,scale,x,val,adj,grad):
    for k in range(a,b):
        o=op[k];l=left[k];r=right[k]
        if o==0:val[k]=par[k]
        elif o==1:val[k]=x[l]*scale[pinids[l]]
        elif o==2:val[k]=val[l]+val[r]
        elif o==3:val[k]=val[l]*val[r]
        elif o==4:
            u=val[l];ex=par[k]
            if (u<0 and ex!=math.floor(ex)) or (u==0 and ex<0):return np.nan
            val[k]=u**ex
        elif o==5:val[k]=math.sin(val[l])
        elif o==6:val[k]=math.cos(val[l])
        elif o==7:val[k]=math.exp(min(709.,val[l])) if val[l]<710 else np.inf
        elif o==8:
            if val[l]<=0:return np.nan
            val[k]=math.log(val[l])
        elif o==9:val[k]=abs(val[l])
        if not math.isfinite(val[k]):return np.nan
        adj[k]=0.
    for j in range(p,q):grad[j]=0.
    adj[b-1]=1.
    for k in range(b-1,a-1,-1):
        o=op[k];l=left[k];r=right[k];d=adj[k]
        if d==0:continue
        if o==1:grad[l]+=d*scale[pinids[l]]
        elif o==2:adj[l]+=d;adj[r]+=d
        elif o==3:adj[l]+=d*val[r];adj[r]+=d*val[l]
        elif o==4:
            if val[l]==0 and par[k]<1:return np.nan
            if par[k]!=0:adj[l]+=d*par[k]*val[l]**(par[k]-1)
        elif o==5:adj[l]+=d*math.cos(val[l])
        elif o==6:adj[l]-=d*math.sin(val[l])
        elif o==7:adj[l]+=d*val[k]
        elif o==8:adj[l]+=d/val[l]
        elif o==9:adj[l]+=d*(1. if val[l]>0 else -1. if val[l]<0 else 0.)
    return val[b-1]


@njit(cache=True)
def project_factors(op,left,right,par,ptr,pinptr,ids,lo,hi,enabled,scale,z,u,x,val,adj,grad,inner):
    invalid=0;unresolved=0
    for j in range(len(ids)):x[j]=z[ids[j]]-u[j]
    for e in range(len(lo)):
        if not enabled[e]:continue
        p=pinptr[e];q=pinptr[e+1]
        for it in range(inner):
            f=evaluate_row(ptr[e],ptr[e+1],p,q,op,left,right,par,ids,scale,x,val,adj,grad)
            if not math.isfinite(f):
                invalid+=1
                # Do not pass NaNs to consensus; failure remains explicit.
                for j in range(p,q):x[j]=z[ids[j]]-u[j]
                break
            d=f-min(max(f,lo[e]),hi[e])
            if abs(d)<=1e-8:break
            den=0.
            for j in range(p,q):den+=grad[j]**2
            if den<1e-28 or not math.isfinite(den):unresolved+=1;break
            step=d/den
            # Local trust region is an inexact-projection choice, not a model bound.
            maximum=0.
            for j in range(p,q):maximum=max(maximum,abs(step*grad[j]))
            damping=1. if enabled[e]==1 else min(1.,.5/max(maximum,1e-30))
            for j in range(p,q):x[j]-=damping*step*grad[j]
    return invalid,unresolved


@njit(cache=True)
def algebra_metrics(op,left,right,par,ptr,pinptr,ids,lo,hi,enabled,scale,z,x,val,adj,grad):
    for j in range(len(ids)):x[j]=z[ids[j]]
    count=0;invalid=0;rawmax=0.;normalized=0.;worst=-1
    for e in range(len(lo)):
        if not enabled[e]:continue
        f=evaluate_row(ptr[e],ptr[e+1],pinptr[e],pinptr[e+1],op,left,right,par,ids,scale,x,val,adj,grad)
        if not math.isfinite(f):invalid+=1;continue
        d=abs(f-min(max(f,lo[e]),hi[e]))
        if d>1e-6:count+=1
        if d>rawmax:rawmax=d;worst=e
        norm=1.
        for j in range(pinptr[e],pinptr[e+1]):norm+=grad[j]**2
        normalized+=d*d/norm
    return count,invalid,rawmax,math.sqrt(normalized),worst


class FactorConsensus:
    def __init__(self,model,rho=.1):
        self.m=model; self.t=model.tape; self.rho=rho;self.iteration=0
        self.z=np.zeros(0);self.u=np.zeros(0);self.refresh()
        self.objective=np.zeros(len(self.z))

    def refresh(self):
        t=self.t;oldz=self.z;oldu=self.u
        self.a=t.arrays();self.ids=self.a[6]
        self.variables=sorted(self.m.getVars(),key=lambda v:v.getIndex())
        # Read final actual original domains; builders can tighten/restore
        # bounds after addVar, including permanently enabled safety gates.
        t.lb=[v.getLbOriginal() for v in self.variables]
        t.ub=[v.getUbOriginal() for v in self.variables]
        self.lb=np.asarray(t.lb);self.ub=np.asarray(t.ub);self.integer=np.asarray(t.integer,bool)
        self.scale=np.minimum(10000.,np.maximum(1.,np.maximum(abs(self.lb),abs(self.ub))))
        self.scale[self.integer]=1.
        self.z=np.clip(np.zeros(len(self.lb)),self.lb,self.ub)/self.scale
        self.z[:len(oldz)]=oldz
        self.u=np.zeros(len(self.ids));self.u[:len(oldu)]=oldu
        self.x=np.zeros(len(self.ids));self.grad=np.zeros(len(self.ids))
        self.val=np.zeros(len(t.op));self.adj=np.zeros(len(t.op))
        # Disabled rows do not participate in variable consensus.
        self.weights=np.repeat((np.asarray(t.enabled)>0).astype(float),np.diff(t.pinptr))
        self.degree=np.bincount(self.ids,weights=self.weights,minlength=len(self.z))

    def set_objective(self,expression):
        self.objective=np.zeros(len(self.z));self.constant=0.
        for term,c in expression.terms.items():
            vv=list(term)
            if not vv:self.constant+=c
            elif len(vv)==1:self.objective[vv[0].getIndex()]+=c*self.scale[vv[0].getIndex()]
            else:raise ValueError('Only linear lexicographic objective supported')

    def sweep(self,inner=4):
        failures=project_factors(*self.a,self.scale,self.z,self.u,self.x,self.val,self.adj,self.grad,inner)
        old=self.z.copy();den=np.maximum(self.degree,1.)
        total=np.bincount(self.ids,weights=(self.x+self.u)*self.weights,minlength=len(self.z))
        new=(total-self.objective/self.rho)/den
        isolated=self.degree==0;new[isolated & (self.objective==0)]=old[isolated & (self.objective==0)]
        physical=np.clip(new*self.scale,self.lb,self.ub)
        physical[self.integer]=np.clip(np.rint(physical[self.integer]),np.ceil(self.lb[self.integer]),np.floor(self.ub[self.integer]))
        self.z=physical/self.scale
        residual=(self.x-self.z[self.ids])*self.weights
        self.u+=residual;self.iteration+=1
        if not np.all(np.isfinite(self.z)) or not np.all(np.isfinite(self.u)):raise FloatingPointError('Nonfinite message')
        return dict(primal_message_residual=float(np.linalg.norm(residual)),dual_message_residual=float(self.rho*np.linalg.norm((self.z-old)*np.sqrt(den))),
                    local_domain_failures=int(failures[0]),local_stationary_violations=int(failures[1]))

    def metrics(self):
        count,invalid,raw,norm,worst=algebra_metrics(*self.a,self.scale,self.z,self.x,self.val,self.adj,self.grad)
        return dict(explicit_violated_rows=int(count),undefined_expression_rows=int(invalid),maximum_raw_violation=float(raw),
                    linearized_scaled_violation_norm=float(norm),worst_row=self.t.names[worst] if worst>=0 else None,
                    candidate_objective_unvalidated=float(self.constant+self.objective@self.z),
                    accepted_objective_vector=None,integrality_max_error=float(np.max(abs(self.physical()[self.integer]-np.rint(self.physical()[self.integer])),initial=0.)))

    def physical(self):return self.z*self.scale

    def solution(self):
        sol=self.m.createOrigSol()
        for v,a in zip(self.variables,self.physical()):self.m.setSolVal(sol,v,float(a))
        return sol


def expand_selected_relations(j,engine,seen,batch=5000):
    """Globally valid implications, not frozen active-row equalities.

    Selector bits AND external gate are part of every new row. Even incoherent
    iterates may request a category; the implication is valid for all values.
    Every declared category remains reachable and factory-defined.
    """
    values=engine.physical();m=j.path.m;added=0;examined=0
    for rid,(sel,gate,factory,name) in enumerate(j.path.rel.relations):
        if gate is not None and values[gate.getIndex()]<.5:continue
        k=int(np.clip(round(values[sel.value.getIndex()]),0,sel.size-1))
        if (rid,k) in seen:continue
        rows=factory(k);mismatch=sel.mismatch(k)
        if gate is not None:mismatch+=1-gate
        for ri,row in enumerate(rows):
            low,high=row.bounds();expression=row.expression()
            if row.lo is not None:
                m.addCons(expression+max(0.,row.lo-low)*mismatch>=row.lo,name=f'hg_{rid}_{k}_{ri}_lo');added+=1
            if row.hi is not None:
                m.addCons(expression-max(0.,high-row.hi)*mismatch<=row.hi,name=f'hg_{rid}_{k}_{ri}_hi');added+=1
        seen.add((rid,k));examined+=1
        if examined>=batch:break
    if added:
        engine.refresh();engine.set_objective(j.path.objective)
    return dict(new_explicit_conditional_rows=added,new_relation_categories=examined,retained_categories=len(seen),all_selector_domains_retained=True)
