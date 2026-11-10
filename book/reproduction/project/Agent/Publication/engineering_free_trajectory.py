"""Local free-control Bernstein transcription with exact AD derivatives.

This is a continuous block of the engineering optimizer, not a global MINLP
bound. Collision tubes are recomputed from the current path. Every exported
candidate still needs independent terrain/weather/pose and resource replay.
"""
from pathlib import Path
import math, sys, time,os,ctypes
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'.codex/runtime/casadi'))
_native_handles=[]

def load_solver():
    import casadi as ca
    if os.name=='nt' and not _native_handles:
        # CasADi's native plugin search does not reliably decode this Chinese
        # workspace path. Preload through the Unicode Windows loader.
        directory=ROOT/'.codex/runtime/casadi/casadi'
        _native_handles.append(os.add_dll_directory(str(directory)))
        _native_handles.append(ctypes.WinDLL(str(directory/'libcasadi_nlpsol_ipopt.dll')))
    return ca


def split_controls(c,u=.5):
    rows=[np.asarray(c,float)]
    # Derivative curves have fewer than eight controls; de Casteljau must
    # use their actual degree, including when cropping near an hour boundary.
    for k in range(len(rows[0])-1):rows.append((1-u)*rows[-1][:-1]+u*rows[-1][1:])
    return np.array([r[0] for r in rows]),np.array([r[-1] for r in rows[::-1]])


def bernstein(c,T,s):
    ca=load_solver()
    result=[]
    for k in range(4):
        n=c.size2()-1
        result.append(sum(c[:,i]*(math.comb(n,i)*s**i*(1-s)**(n-i)) for i in range(n+1)))
        c=n*(c[:,1:]-c[:,:-1])/T
    return result


def optimize_transfer(initial,profile,geometry,terrain,weather,offset_s=0.,trust_m=2.):
    """Optimize all controls and times, with rest endpoints and C3 joins.

    Local collision tubes are a sufficient restriction used by this local
    step, not a replacement feasible domain for the original problem.
    """
    ca=load_solver()
    from Agent.Publication.engineering_joint_execution import state
    a=np.asarray(initial['points'],float);ts=list(initial['durations_s'])
    controls=[np.array([x]*4+[y]*4) for x,y in zip(a,a[1:])]
    if len(controls)==1:
        left,right=split_controls(controls[0],1/3);middle,last=split_controls(right,.5)
        controls=[left,middle,last];ts=[ts[0]/3]*3
    origin=controls[0][0].copy();air=profile['aircraft'];energy=profile['energy']
    clearance=sum(air[k] for k in ['body_radius_m','obstacle_clearance_m','tracking_error_m','map_error_m'])
    op=ca.Opti();C=[];T=[];expressions=[];energy_expr=0.;cursor=offset_s
    nodes,weights=np.polynomial.legendre.leggauss(12)
    corridors=[]
    for k,(init,t0) in enumerate(zip(controls,ts)):
        t=op.variable();T.append(t);op.set_initial(t,t0)
        op.subject_to(op.bounded(max(.2,t0*.08),t,max(2*t0,15.)))
        # Convex capsule around a certified collision-free line. Convex
        # combinations of all controls remain inside this convex capsule.
        x,y=init[0],init[-1];d=y-x;length2=float(d@d)
        margin=geometry.segment_margin(x,y,clearance)
        radius=min(trust_m,max(0.,margin-.0001)) if math.isfinite(margin) else trust_m
        contact=(initial.get('departure_contact',False) and k==0) or (initial.get('arrival_contact',False) and k==len(controls)-1)
        if contact:radius=0.
        zero_tube=radius<1e-9
        if zero_tube and length2>1e-12:
            fraction_controls=op.variable(1,8)
            c=ca.vertcat(*[(x[q]-origin[q])+d[q]*fraction_controls if abs(d[q])>1e-12 else ca.DM.ones(1,8)*(x[q]-origin[q]) for q in range(3)])
            op.set_initial(fraction_controls,((init-x)@d/length2)[None,:])
            op.subject_to(op.bounded(0,fraction_controls,1))
        else:
            c=op.variable(3,8);op.set_initial(c,(init-origin).T)
        C.append(c)
        corridors.append(dict(from_xyz_m=x.tolist(),to_xyz_m=y.tolist(),radius_m=radius,obstacle_margin_m=margin if math.isfinite(margin) else None,contact=contact))
        for j in range(8):
            delta=c[:,j]-(x-origin);frac=ca.dot(delta,d)/max(length2,1e-20)
            if length2>1e-12:
                op.subject_to(op.bounded(0,frac,1))
                orth=delta-frac*d
                if not zero_tube:op.subject_to(ca.sumsqr(orth)<=radius**2)
            else:op.subject_to(c[:,j]==x-origin)
        expr=bernstein(c,t,ca.MX.sym('unused')) if False else None
        # Source-hour and height interpolation are locally parameterized;
        # the accepted plan is recomputed against exact source records.
        for s in np.linspace(0,1,17):
            p,v,acc,jerk=bernstein(c,t,float(s));world=p+origin
            ref=state(init[0],init[-1],t0,float(s))[0]
            ground=float(terrain(*ref[:2]));wind,rho=weather(ref,cursor+s*t0,ground)
            va=v-wind;force=air['mass_kg']*(acc+ca.DM([0,0,9.81]))+.5*rho*air['drag_CdA_m2']*ca.sqrt(ca.sumsqr(va)+1e-12)*va
            op.subject_to(ca.sumsqr(v)<=air['ground_speed_max_mps']**2*.999**2)
            op.subject_to(op.bounded(-air['vertical_speed_max_mps']*.999,v[2],air['vertical_speed_max_mps']*.999))
            op.subject_to(ca.sumsqr(va)<=air['air_speed_max_mps']**2*.999**2)
            op.subject_to(ca.sumsqr(acc)<=air['acceleration_max_mps2']**2*.999**2)
            op.subject_to(ca.sumsqr(jerk)<=air['jerk_max_mps3']**2*.999**2)
            op.subject_to(ca.sumsqr(force)<=air['thrust_max_N']**2*.999**2)
            op.subject_to(force[2]>=math.cos(math.radians(air['tilt_max_deg']))*ca.sqrt(ca.sumsqr(force)))
            need=.61 if contact else clearance
            op.subject_to(world[2]>=ground+need+.001)
        for s,w in zip((nodes+1)/2,weights/2):
            p,v,ac,j=bernstein(c,t,float(s));ref=np.asarray(init[0])+(init[-1]-init[0])*s
            wind,rho=weather(ref,cursor+s*t0,float(terrain(*ref[:2])));va=v-wind
            power=energy['hover_power_W']*math.sqrt(1.225/rho)+.5*air['drag_CdA_m2']*rho*(ca.sumsqr(va)+1e-12)**1.5+air['mass_kg']*9.81*ca.fmax(v[2],0)/energy['propulsion_efficiency']
            energy_expr+=w*t*power/3600
        cursor+=t0
    def equality(expression):
        for z in ca.vertsplit(expression):
            # Drop only structurally constant zero rows, never a rank test at
            # one trial point. Zero-width vertical tubes already eliminate xy.
            fn=ca.Function('row',[op.x],[z]).expand();symbols=ca.SX.sym('symbols',op.nx);sx=fn(symbols);jac=ca.jacobian(sx,symbols)
            if jac.is_zero():
                value=float(fn(np.zeros(op.nx)))
                if abs(value)>1e-8:raise ValueError('Inconsistent fixed trajectory boundary')
            else:op.subject_to(z==0)
    for k in range(len(C)-1):
        left=bernstein(C[k],T[k],1.);right=bernstein(C[k+1],T[k+1],0.)
        for u,v in zip(left,right):equality(u-v)
    left=bernstein(C[0],T[0],0.);right=bernstein(C[-1],T[-1],1.)
    equality(left[0]-(controls[0][0]-origin));equality(right[0]-(controls[-1][-1]-origin))
    for z in left[1:]+right[1:]:equality(z)
    total=sum(T);op.minimize(total)
    op.solver('ipopt',{'expand':True},{'print_level':0,'max_iter':1500,'tol':1e-7,'acceptable_tol':1e-6,'sb':'yes'})
    begin=time.perf_counter()
    try:
        solution=op.solve();ok=True;reason=op.stats()['return_status']
    except RuntimeError:
        ok=False;reason=op.stats().get('return_status','solver_failed');solution=op.debug
    receipt=dict(success=ok,status=reason,seconds=time.perf_counter()-begin,variables=int(op.nx),constraints=int(op.ng),
        objective='transfer_time',global_gap=None,trust_m=trust_m,collision_corridors=corridors,
        weather_linearization='source weather at initializer quadrature and collocation; exact replay required',
        pose_rates_in_nlp=False)
    if not ok:return None,receipt
    result=[];clock=0.
    for c,t in zip(C,T):
        cp=np.asarray(solution.value(c)).T+origin;dt=float(solution.value(t))
        result.append(dict(kind='transfer',start_s=clock,end_s=clock+dt,from_xyz_m=cp[0].tolist(),to_xyz_m=cp[-1].tolist(),control_points_m=cp.tolist(),
            departure_contact=bool(initial.get('departure_contact',False) and not result),arrival_contact=bool(initial.get('arrival_contact',False) and len(result)==len(C)-1)))
        clock+=dt
    receipt.update(initial_duration_s=sum(ts),optimized_duration_s=clock,linearized_energy_Wh=float(solution.value(energy_expr)))
    return result,receipt
