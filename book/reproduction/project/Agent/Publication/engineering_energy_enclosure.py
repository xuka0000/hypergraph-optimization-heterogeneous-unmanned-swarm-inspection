"""Adaptive energy enclosures, distinct from two-quadrature comparisons.

Bernstein range bounds cover every time in a leaf. On a single source DEM
cell, height layer and climb branch, a bounded second derivative sharpens
the midpoint enclosure. Weather hours are split exactly. IEEE operations
are outward rounded with nextafter; source floats define this simulation.
This certifies a supplied curve, not a uniform SCIP objective error bound.
"""
import math
import heapq
import numpy as np
from Agent.Publication.engineering_joint_weather import hourly_table


def down(x):return np.nextafter(np.asarray(x,dtype=float),-np.inf)
def up(x):return np.nextafter(np.asarray(x,dtype=float),np.inf)


class R:
    def __init__(self,lo,hi=None):self.lo=np.asarray(lo,float);self.hi=np.asarray(lo if hi is None else hi,float)
    def __add__(self,b):
        b=cast(b);return R(down(self.lo+b.lo),up(self.hi+b.hi))
    __radd__=__add__
    def __neg__(self):return R(-self.hi,-self.lo)
    def __sub__(self,b):return self+-cast(b)
    def __rsub__(self,b):return cast(b)+-self
    def __mul__(self,b):
        b=cast(b);p=np.array([self.lo*b.lo,self.lo*b.hi,self.hi*b.lo,self.hi*b.hi]);return R(down(p.min(0)),up(p.max(0)))
    __rmul__=__mul__
    def __truediv__(self,b):
        b=cast(b)
        if np.any((b.lo<=0)&(b.hi>=0)):raise ArithmeticError('zero interval denominator')
        return self*R(down(1/b.hi),up(1/b.lo))
    def __getitem__(self,k):return R(self.lo[k],self.hi[k])
    def sum(self):
        # Accumulate outward at each operation, not once after a long sum.
        total=R(0.)
        for k in range(self.lo.size):total=total+R(self.lo.flat[k],self.hi.flat[k])
        return total
    def square(self):return R(np.where((self.lo<=0)&(self.hi>=0),0,down(np.minimum(self.lo**2,self.hi**2))),up(np.maximum(self.lo**2,self.hi**2)))
    def sqrt(self):
        if np.any(self.hi<0):raise ArithmeticError('negative square root interval')
        return R(down(np.sqrt(np.maximum(0,self.lo))),up(np.sqrt(np.maximum(0,self.hi))))
    def positive(self):return R(np.maximum(0,self.lo),np.maximum(0,self.hi))
    def absmax(self):return up(np.maximum(abs(self.lo),abs(self.hi)))


def cast(x):return x if isinstance(x,R) else R(x)
def norm(x):return x.square().sum().sqrt()
def hull(x):return R(x.lo.min(0),x.hi.max(0))
def split(x,s=.5):
    levels=[x]
    for _ in range(len(x.lo)-1):levels.append(levels[-1][:-1]*(1-s)+levels[-1][1:]*s)
    answer=(R(np.array([r.lo[0] for r in levels]),np.array([r.hi[0] for r in levels])),
            R(np.array([r.lo[-1] for r in levels[::-1]]),np.array([r.hi[-1] for r in levels[::-1]])))
    constant=np.all(x.lo==x.lo[0],axis=0)&np.all(x.hi==x.lo[0],axis=0)
    for part in answer:part.lo[:,constant]=x.lo[0,constant];part.hi[:,constant]=x.lo[0,constant]
    return answer
def evaluate(x,s=.5):
    for _ in range(len(x.lo)-1):x=x[:-1]*(1-s)+x[1:]*s
    return x[0]
def derivative(x,T):
    answer=(x[1:]-x[:-1])*(len(x.lo)-1)/T
    constant=np.all(x.lo==x.lo[0],axis=0)&np.all(x.hi==x.lo[0],axis=0)
    answer.lo[:,constant]=0.;answer.hi[:,constant]=0.
    return answer


class EnergyEnclosure:
    def __init__(self,scene,terrain):
        self.scene=scene;self.terrain=terrain;self.cfg=scene['joint_reference_profile']
        self.hours=hourly_table(scene,'2025-07-01T08:00:00+00:00',self.cfg['planning']['horizon_seconds'])
        self.xy=terrain.tri.points[terrain.tri.simplices]
        self.z=terrain.values[:,0][terrain.tri.simplices]
        self.lo=down(self.xy.min(1));self.hi=up(self.xy.max(1));self.cell_cache={}

    def cell(self,k):
        if k not in self.cell_cache:
            xy=self.xy[k];z=self.z[k];a=R(xy[0])-xy[2];b=R(xy[1])-xy[2]
            det=a[0]*b[1]-a[1]*b[0]
            inverse=[(b[1]/det,-b[0]/det),(-a[1]/det,a[0]/det)]
            dz0=R(z[0])-z[2];dz1=R(z[1])-z[2]
            gx=inverse[0][0]*dz0+inverse[1][0]*dz1;gy=inverse[0][1]*dz0+inverse[1][1]*dz1
            self.cell_cache[k]=(inverse,gx,gy)
        return self.cell_cache[k]

    def ground(self,c):
        bound=hull(c);ids=np.flatnonzero(np.all(self.hi>=bound.lo[:2],axis=1)&np.all(self.lo<=bound.hi[:2],axis=1))
        selected=[];ranges=[]
        for k in ids:
            inverse,gx,gy=self.cell(int(k));rel=c[:,:2]-self.xy[k,2]
            bary=[hull(rel[:,0]*row[0]+rel[:,1]*row[1]) for row in inverse]
            bary.append(1-bary[0]-bary[1])
            if any(b.hi<0 or b.lo>1 for b in bary):continue
            z=hull(R(self.z[k,2])+rel[:,0]*gx+rel[:,1]*gy)
            selected.append((int(k),gx,gy));ranges.append(z)
        if not ranges:raise ArithmeticError('curve outside source terrain or unresolved cell')
        return R(min(float(z.lo) for z in ranges),max(float(z.hi) for z in ranges)),selected

    def atmosphere(self,height,hour):
        row=self.hours[hour]
        if any(not r['reference_flight_allowed'] or r['air_density_proxy_kg_m3'] is None for r in row['rows']):raise ArithmeticError('unavailable source weather hour')
        samples=row['rows'];clipped=R(max(0,float(height.lo)),min(100,max(0,float(height.hi))))
        if height.hi>100+1e-6:raise ArithmeticError('above source weather domain')
        values=[];regions=[]
        for lo,hi,index in [(0,30,0),(30,60,1),(60,100,2)]:
            a=max(lo,float(clipped.lo));b=min(hi,float(clipped.hi))
            if a>b:continue
            x=np.array([samples[max(0,index-1)][k] for k in ['wind_east_ms','wind_north_ms','air_density_proxy_kg_m3']])
            y=np.array([samples[index][k] for k in ['wind_east_ms','wind_north_ms','air_density_proxy_kg_m3']])
            slope=(R(y)-x)/(hi-lo) if index else R(np.zeros(3))
            values.append(R(x)+slope*(R(a,b)-lo));regions.append(slope)
        if not values:raise ArithmeticError('no weather-height interval')
        return R(np.min([x.lo for x in values],0),np.max([x.hi for x in values],0)),regions

    def power(self,c,v,hour):
        g,cells=self.ground(c);height=hull(c)[2]-g;atmos,regions=self.atmosphere(height,hour)
        velocity=hull(v);wind=R(np.r_[atmos.lo[:2],0],np.r_[atmos.hi[:2],0]);rho=atmos[2];speed=norm(velocity-wind)
        en=self.cfg['energy'];air=self.cfg['aircraft']
        P=en['hover_power_W']*(R(1.225)/rho).sqrt()+.5*air['drag_CdA_m2']*rho*speed*speed*speed+air['mass_kg']*9.81/en['propulsion_efficiency']*velocity[2].positive()
        return P,cells,regions,rho,speed

    def leaf(self,c,v,a,j,T,hour):
        P,cells,regions,rho,U=self.power(c,v,hour);energy=P*T/3600;method='range'
        fixed_xy=bool(np.all(v.lo[:,:2]==0)&np.all(v.hi[:,:2]==0))
        if (len(cells)==1 or fixed_xy) and len(regions)==1 and (hull(v)[2].lo>=0 or hull(v)[2].hi<=0):
            _,gx,gy=cells[0]
            if fixed_xy:gx=gy=R(0.)
            slope=regions[0];vh=hull(v);ah=hull(a);jh=hull(j)
            hd=vh[2]-gx*vh[0]-gy*vh[1];hdd=ah[2]-gx*ah[0]-gy*ah[1]
            wind_d=R(np.r_[slope.lo[:2],0],np.r_[slope.hi[:2],0])*hd
            wind_dd=R(np.r_[slope.lo[:2],0],np.r_[slope.hi[:2],0])*hdd
            A=R(float(norm(ah-wind_d).hi));J=R(float(norm(jh-wind_dd).hi));U=R(float(U.hi))
            rd=R(float((slope[2]*hd).absmax()));rdd=R(float((slope[2]*hdd).absmax()));rmin=R(float(rho.lo));rmax=R(float(rho.hi))
            en=self.cfg['energy'];air=self.cfg['aircraft']
            second=en['hover_power_W']*R(1.225).sqrt()*(.75*rd*rd/(rmin*rmin*rmin.sqrt())+.5*rdd/(rmin*rmin.sqrt()))
            second=second+.5*air['drag_CdA_m2']*(rdd*U*U*U+6*rd*U*U*A+rmax*(6*U*A*A+3*U*U*J))
            if vh[2].lo>=0:second=second+air['mass_kg']*9.81/en['propulsion_efficiency']*R(float(jh[2].absmax()))
            pc=evaluate(c);vc=evaluate(v);mid,_,_,_,_=self.power(R(pc.lo[None,:],pc.hi[None,:]),R(vc.lo[None,:],vc.hi[None,:]),hour)
            error=second*T*T*T/24/3600;center=mid*T/3600
            energy=R(max(float(energy.lo),float((center-error).lo)),min(float(energy.hi),float((center+error).hi)));method='midpoint_second_derivative'
        return energy,method

    def integrate(self,timeline,launch_s=0.,tolerance_Wh=.01,max_leaves=200000,progress=None):
        leaves={};heap=[];serial=0;counts={'range':0,'midpoint_second_derivative':0}
        def insert(c,v,a,j,T,hour):
            nonlocal serial
            interval,method=self.leaf(c,v,a,j,T,hour);counts[method]+=1
            leaves[serial]=(c,v,a,j,T,hour,interval)
            heapq.heappush(heap,(-float(interval.hi-interval.lo),serial));serial+=1
        for seg in timeline:
            T=float(seg['end_s']-seg['start_s']);c=R(np.asarray(seg['control_points_m'],float));v=derivative(c,T);a=derivative(v,T);j=derivative(a,T)
            start=launch_s+float(seg['start_s']);end=start+T;cut=start;states=[c,v,a,j]
            boundaries=[h*3600. for h in range(1,len(self.hours)) if start<h*3600<end]+[end]
            for boundary in boundaries:
                span=boundary-cut
                if boundary<end:
                    pieces=[split(x,span/(end-cut)) for x in states];portion=[x[0] for x in pieces];states=[x[1] for x in pieces]
                else:portion=states
                insert(*portion,span,min(len(self.hours)-1,int((cut+boundary)/2//3600)));cut=boundary
        def total():
            return R(down(math.fsum(float(x[-1].lo) for x in leaves.values())),up(math.fsum(float(x[-1].hi) for x in leaves.values())))
        bound=total();refinements=0
        while bound.hi-bound.lo>tolerance_Wh and len(leaves)<max_leaves:
            _,key=heapq.heappop(heap);c,v,a,j,T,hour,_=leaves.pop(key)
            if T<1e-9:raise ArithmeticError('interval refinement reached temporal precision floor')
            parts=[split(x) for x in [c,v,a,j]]
            for side in [0,1]:insert(*(x[side] for x in parts),T/2,hour)
            refinements+=1
            if refinements%16==0:
                bound=total()
                if progress and refinements%256==0:progress(dict(leaves=len(leaves),evaluations=serial,lower_Wh=float(bound.lo),upper_Wh=float(bound.hi),width_Wh=float(bound.hi-bound.lo)))
        bound=total()
        return dict(lower_Wh=float(bound.lo),upper_Wh=float(bound.hi),width_Wh=float(bound.hi-bound.lo),
                    tolerance_Wh=tolerance_Wh,converged=bool(bound.hi-bound.lo<=tolerance_Wh),leaves=len(leaves),evaluations=serial,methods=counts,
                    scope='fixed candidate path; not a uniform optimizer or full-domain gap certificate')
