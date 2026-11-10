"""Conservative interval checks for the declared deterministic force model.

IEEE roundoff is padded; this is a numerical engineering enclosure, not a
formally verified directed-rounding interval library or field certification.
"""
import math
import numpy as np
from shapely.geometry import MultiPoint,Polygon

class I:
    def __init__(self,lo,hi=None):self.lo=np.asarray(lo,float);self.hi=np.asarray(lo if hi is None else hi,float)
    def __add__(self,b):
        b=cast(b);return I(self.lo+b.lo-1e-12,self.hi+b.hi+1e-12)
    __radd__=__add__
    def __neg__(self):return I(-self.hi,-self.lo)
    def __sub__(self,b):return self+-cast(b)
    def __rsub__(self,b):return cast(b)+-self
    def __mul__(self,b):
        b=cast(b);q=np.array([self.lo*b.lo,self.lo*b.hi,self.hi*b.lo,self.hi*b.hi]);return I(q.min(axis=0)-1e-12,q.max(axis=0)+1e-12)
    __rmul__=__mul__
    def __truediv__(self,b):
        b=cast(b)
        if np.any((b.lo<=0)&(b.hi>=0)):raise ArithmeticError('interval denominator contains zero')
        return self*I(1/b.hi,1/b.lo)
    def __getitem__(self,k):return I(self.lo[k],self.hi[k])
    def square(self):
        return I(np.where((self.lo<=0)&(self.hi>=0),0,np.minimum(self.lo**2,self.hi**2)),np.maximum(self.lo**2,self.hi**2)+1e-12)
    def sqrt(self):return I(np.sqrt(np.maximum(0,self.lo)),np.sqrt(np.maximum(0,self.hi))+1e-12)
    def sum(self):return I(self.lo.sum()-1e-12,self.hi.sum()+1e-12)
    def absmax(self):return np.maximum(abs(self.lo),abs(self.hi))
    def sin(self):
        lo,hi=float(self.lo),float(self.hi);values=[math.sin(lo),math.sin(hi)]
        for k in range(math.ceil((lo-math.pi/2)/math.pi),math.floor((hi-math.pi/2)/math.pi)+1):values.append((-1.)**k)
        return I(min(values)-1e-12,max(values)+1e-12)
    def cos(self):return (self+math.pi/2).sin()
def cast(x):return x if isinstance(x,I) else I(x)
def vector(xs):return I([float(cast(x).lo) for x in xs],[float(cast(x).hi) for x in xs])
def dot(a,b):return (a*b).sum()
def norm(a):return a.square().sum().sqrt()
def cross(a,b):return vector([a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]])
def hull(c):return I(np.min(c,axis=0),np.max(c,axis=0))

class ForceEnclosure:
    def __init__(self,scene,terrain,weather):
        self.cfg=scene['joint_reference_profile'];self.terrain=terrain;self.weather=weather
        verts=terrain.tri.simplices;xy=terrain.points[verts];vals=terrain.values[:,0];self.lo=xy.min(axis=1);self.hi=xy.max(axis=1)
        self.zlo=vals[verts].min(axis=1);self.zhi=vals[verts].max(axis=1);self.xy=xy;self.zbase=vals[verts[:,2]]
        self.grad=np.einsum('bi,bij->bj',vals[verts[:,:2]]-vals[verts[:,2,None]],terrain.tri.transform[:,:2,:]);self.polygons={}
    def terrain_enclosure(self,c):
        xy=c[:,:2];center=xy.mean(axis=0)
        if np.linalg.norm(xy-center,axis=1).max()<1e-7:
            j=int(self.terrain.tri.find_simplex(center));return I(float(self.terrain(*center))),I(self.grad[j])
        ids=np.flatnonzero(np.all(self.hi>=xy.min(axis=0)-1e-8,axis=1)&np.all(self.lo<=xy.max(axis=0)+1e-8,axis=1))
        shape=MultiPoint(xy).convex_hull;selected=[];elevations=[]
        def vertices(geometry):
            if geometry.is_empty:return []
            if geometry.geom_type=='Polygon':return list(geometry.exterior.coords)
            if hasattr(geometry,'geoms'):return [p for g in geometry.geoms for p in vertices(g)]
            return list(geometry.coords)
        for j in ids:
            if j not in self.polygons:self.polygons[j]=Polygon(self.xy[j])
            intersection=self.polygons[j].intersection(shape)
            if not intersection.is_empty:
                selected.append(j);points=np.asarray(vertices(intersection))
                elevations.extend(self.zbase[j]+(points-self.xy[j,2])@self.grad[j])
        if not selected:raise ArithmeticError('source terrain hull unresolved')
        # The source interpolation is affine on each triangle. Its extrema
        # over the clipped convex hull occur at intersection vertices.
        return I(min(elevations)-1e-8,max(elevations)+1e-8),hull(self.grad[selected])
    def check(self,c,T,yaw_c,absolute_s,origin=None,details=False,state_derivatives=None,yaw_derivative=None):
        origin=np.zeros(3) if origin is None else np.asarray(origin);world=c+origin
        air=self.cfg['aircraft'];p=hull(c)+origin;d=[];cc=np.array(c)
        if state_derivatives is None:
            for k in range(3):cc=(len(cc)-1)*np.diff(cc,axis=0)/T;d.append(hull(cc))
        else:d=[hull(values) for values in state_derivatives]
        v,a,j=d;terrain,grad=self.terrain_enclosure(world);height=p[2]-terrain
        if height.hi>100+1e-6:raise ArithmeticError('weather ceiling')
        ref=world.mean(axis=0);ground=float(self.terrain(*ref[:2]));heights=[float(height.lo),float(height.hi)]+[h for h in [30.,60.] if height.lo<h<height.hi]
        values=[]
        for h in heights:
            ref[2]=ground+min(100.,h);wind,rho=self.weather(ref,absolute_s,ground);values.append([*wind,rho])
        wr=hull(np.array(values));wind=wr[:3];rho=wr[3]
        slopes=[]
        for l,r in [(float(height.lo),min(float(height.hi),30.)),(max(float(height.lo),30.),min(float(height.hi),60.)),(max(float(height.lo),60.),float(height.hi))]:
            if r<=l:continue
            ref[2]=ground+l;w0,r0=self.weather(ref,absolute_s,ground);ref[2]=ground+r;w1,r1=self.weather(ref,absolute_s,ground);slopes.append(np.array([*(w1-w0),r1-r0])/(r-l))
        slope=hull(np.array(slopes)) if slopes else I(np.zeros(4));hdot=v[2]-dot(grad,v[:2]);wdot=slope[:3]*hdot;rhodot=slope[3]*hdot
        va=v-wind;vdot=a-wdot;speed=norm(va);force=air['mass_kg']*(a+np.array([0,0,9.81]))+.5*air['drag_CdA_m2']*rho*speed*va
        if speed.lo>1e-8:
            fdot=air['mass_kg']*j+.5*air['drag_CdA_m2']*(rhodot*speed*va+rho*((dot(va,vdot)/speed)*va+speed*vdot))
        else:
            bound=.5*air['drag_CdA_m2']*(rhodot.absmax()*float(speed.hi)**2+2*float(rho.hi)*float(speed.hi)*float(norm(vdot).hi));fdot=air['mass_kg']*j+I(np.full(3,-bound),np.full(3,bound))
        fn=norm(force);b3=force/fn;b3d=(fdot-b3*dot(b3,fdot))/fn
        yaw=hull(yaw_c[:,None])[0];yd=hull((7*np.diff(yaw_c)/T)[:,None])[0] if yaw_derivative is None else hull(np.asarray(yaw_derivative)[:,None])[0]
        h=vector([yaw.cos(),yaw.sin(),0]);hd=vector([-yaw.sin()*yd,yaw.cos()*yd,0])
        q=cross(b3,h);qd=cross(b3d,h)+cross(b3,hd);qn=norm(q);b2=q/qn;b2d=(qd-b2*dot(b2,qd))/qn
        b1=cross(b2,b3);b1d=cross(b2d,b3)+cross(b2,b3d);omega=vector([-dot(b2,b3d),dot(b1,b3d),dot(b2,b1d)])
        values=dict(airspeed_upper_mps=float(speed.hi),thrust_upper_N=float(fn.hi),tilt_cos_lower=float((force[2]/fn).lo),body_rate_component_upper_deg_s=float(np.rad2deg(omega.absmax()).max()))
        ok=(speed.hi<=air['air_speed_max_mps']+1e-7 and fn.hi<=air['thrust_max_N']+1e-7 and (force[2]/fn).lo>=math.cos(math.radians(air['tilt_max_deg']))-1e-7 and values['body_rate_component_upper_deg_s']<=air['body_rate_max_deg_s']+1e-7)
        if details:return bool(ok),values,dict(position=p,velocity=v,body_columns=[b1,b2,b3],body_column_rates=[b1d,b2d,b3d])
        return bool(ok),values
