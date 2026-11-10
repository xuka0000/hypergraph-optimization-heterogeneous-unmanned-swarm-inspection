"""Adaptive contract5 force/energy enclosures using continuous forecast cells.

IEEE padding follows the existing engineering interval implementation. Results
are numerical enclosures, explicitly not a formally directed-rounding proof.
Unresolved cells are returned as unresolved, never accepted by dense sampling.
"""
import math
import numpy as np
from Agent.Publication.engineering_interval_physics import I,hull,vector,dot,norm,cross,ForceEnclosure
from Agent.Publication.engineering_free_trajectory import split_controls


class RegionalEnclosure(ForceEnclosure):
    def __init__(self,scene,terrain,forecast):
        super().__init__(scene,terrain,None);self.forecast=forecast

    def weather_box(self,time_lo,time_hi,height_lo,height_hi):
        if time_lo < -1e-7 or time_hi > self.forecast.times[-1]+1e-7:
            raise ArithmeticError('forecast time extrapolation')
        if height_lo < -1e-7 or height_hi > 100+1e-7:
            raise ArithmeticError('forecast height extrapolation')
        values=[];dt=[];dz=[];allowed=True
        for h in range(len(self.forecast.times)-1):
            a=max(time_lo,float(self.forecast.times[h]));b=min(time_hi,float(self.forecast.times[h+1]))
            if b<a:continue
            if b>a+1e-9 and not self.forecast.intervals[h]['flight_allowed']:allowed=False
            for k in range(3):
                lo,hi,c0,c1,c2,c3=self.forecast.coefficients(h,k)
                l=max(height_lo,lo);u=min(height_hi,hi)
                if u<l:continue
                for t in [a,b]:
                    for z in [l,u]:
                        x=t-self.forecast.times[h];y=z-lo
                        values.append(c0+c1*x+c2*y+c3*x*y)
                        dt.append(c1+c3*y);dz.append(c2+c3*x)
        if not values:raise ArithmeticError('no enclosing forecast cells')
        # Bilinear extrema and the derivative extrema over each rectangle
        # occur at corners; unions retain both one-sided boundary derivatives.
        return hull(np.array(values)),hull(np.array(dt)),hull(np.array(dz)),allowed

    def check(self,c,T,ac,start,origin=None,details=False):
        if T<=0:raise ValueError('Positive interval duration required')
        origin=np.zeros(3) if origin is None else np.asarray(origin)
        world=np.asarray(c)+origin;air=self.cfg['aircraft'];p=hull(world)
        if np.any(self.terrain.tri.find_simplex(world[:,:2])<0):
            raise ArithmeticError('control hull not enclosed by source DEM')
        deriv=[];cp=np.asarray(c)
        for _ in range(3):cp=(len(cp)-1)*np.diff(cp,axis=0)/T;deriv.append(hull(cp))
        v,a,jerk=deriv;ground,grad=self.terrain_enclosure(world);height=p[2]-ground
        W,Wt,Wz,allowed=self.weather_box(start,start+T,float(height.lo),float(height.hi))
        wind=vector([W[0],W[1],0]);rho=W[2]
        hdot=v[2]-dot(grad,v[:2]);wdot=vector([Wt[0]+Wz[0]*hdot,Wt[1]+Wz[1]*hdot,0])
        rhodot=Wt[2]+Wz[2]*hdot;va=v-wind;vdot=a-wdot;speed=norm(va)
        force=air['mass_kg']*(a+np.array([0,0,9.81]))+.5*air['drag_CdA_m2']*rho*speed*va
        if speed.lo>1e-8:
            fdot=air['mass_kg']*jerk+.5*air['drag_CdA_m2']*(rhodot*speed*va+rho*((dot(va,vdot)/speed)*va+speed*vdot))
        else:
            bound=.5*air['drag_CdA_m2']*(rhodot.absmax()*float(speed.hi)**2+2*float(rho.hi)*float(speed.hi)*float(norm(vdot).hi))
            fdot=air['mass_kg']*jerk+I(np.full(3,-bound),np.full(3,bound))
        fn=norm(force);b3=force/fn;b3d=(fdot-b3*dot(b3,fdot))/fn
        angles=hull(ac);rates=hull(7*np.diff(ac,axis=0)/T);accels=hull(42*np.diff(ac,n=2,axis=0)/T**2)
        yaw=angles[0];yd=rates[0];heading=vector([yaw.cos(),yaw.sin(),0]);hd=vector([-yaw.sin()*yd,yaw.cos()*yd,0])
        q=cross(b3,heading);qd=cross(b3d,heading)+cross(b3,hd);qn=norm(q);b2=q/qn;b2d=(qd-b2*dot(b2,qd))/qn
        b1=cross(b2,b3);b1d=cross(b2d,b3)+cross(b2,b3d)
        omega=vector([-dot(b2,b3d),dot(b1,b3d),dot(b2,b1d)])
        climb=I(max(0.,float(v[2].lo)),max(0.,float(v[2].hi)))
        power=self.cfg['energy']['hover_power_W']*(I(1.225)/rho).sqrt()+.5*air['drag_CdA_m2']*rho*speed*speed*speed+air['mass_kg']*9.81*climb/self.cfg['energy']['propulsion_efficiency']
        lower=[-math.inf,math.radians(-120),math.radians(-320),1.]
        upper=[math.inf,math.radians(30),math.radians(320),119.94/6.83]
        checks=dict(ground_speed=float(norm(v).hi)<=5+1e-7,vertical_speed=float(v[2].absmax())<=1+1e-7,
            acceleration=float(norm(a).hi)<=3+1e-7,jerk=float(norm(jerk).hi)<=5+1e-7,
            airspeed=float(speed.hi)<=12+1e-7,thrust=float(fn.hi)<=137.34+1e-7,
            tilt=float((force[2]/fn).lo)>=math.cos(math.radians(25))-1e-7,
            body_rates=float(omega.absmax().max())<=math.radians(30)+1e-7,
            weather_discrete=allowed,temperature=float(W[3].lo)>=-1e-7 and float(W[3].hi)<=40+1e-7,
            wind=float(norm(wind).hi)<=10+1e-7,
            pose_bounds=bool(np.all(angles.lo>=np.array(lower)-1e-7) and np.all(angles.hi<=np.array(upper)+1e-7)),
            angle_rates=bool(np.all(rates.absmax()[:3]<=math.radians(30)+1e-7)),
            angle_acceleration=bool(np.all(accels.absmax()[:3]<=math.radians(30)+1e-7)),
            zoom_rate=float(rates[3].absmax())*6.83<=10+1e-7)
        body=dict(position=p,velocity=v,body_columns=[b1,b2,b3],body_column_rates=[b1d,b2d,b3d],
                  angles=angles,rates=rates,power=power,ground=ground,height=height)
        return all(checks.values()),checks,body

    def optics(self,c,T,ac,task,body):
        cfg=self.cfg['camera'];angles=body['angles'];rates=body['rates'];gp,gy=angles[1],angles[2]
        gpd,gyd=rates[1],rates[2]
        opt=vector([gp.cos()*gy.cos(),gp.cos()*gy.sin(),gp.sin()]);right=vector([gy.sin(),-gy.cos(),0]);down=cross(opt,right)
        optd=vector([-gp.sin()*gpd*gy.cos()-gp.cos()*gy.sin()*gyd,
                     -gp.sin()*gpd*gy.sin()+gp.cos()*gy.cos()*gyd,gp.cos()*gpd])
        rightd=vector([gy.cos()*gyd,gy.sin()*gyd,0]);downd=cross(optd,right)+cross(opt,rightd)
        cols=body['body_columns'];dcols=body['body_column_rates']
        rotate=lambda x:cols[0]*x[0]+cols[1]*x[1]+cols[2]*x[2]
        rc=[rotate(x) for x in [right,down,opt]]
        rd=[dcols[0]*x[0]+dcols[1]*x[1]+dcols[2]*x[2]+rotate(xd) for x,xd in zip([right,down,opt],[rightd,downd,optd])]
        target=task['target'];center=np.array([target[k] for k in ['easting_m','northing_m','altitude_m']])
        nominal=np.array([task[k] for k in ['easting_m','northing_m','altitude_m']])
        delta=body['position']-center;distance=norm(delta);incidence=dot(I(target['normal_world']),delta)/distance
        checks=dict(focus=float(distance.lo)>=8-1e-7 and float(distance.hi)<=25+1e-7,
            incidence=float(incidence.lo)>=math.cos(math.radians(30))-1e-7,
            viewpoint=float(np.linalg.norm(np.asarray(c)-nominal,axis=1).max())<=task['observation_requirement']['viewpoint_ball_radius_m']+1e-7)
        fx0=math.hypot(5184,3888)/(2*math.tan(math.radians(66.6/2)));f=fx0*angles[3];fd=fx0*rates[3]
        blur=0.;border=math.inf;gsd=math.inf
        for point in target['surface_samples_utm_m']:
            rel=I(point)-body['position'];vel=-body['velocity'];qc=[dot(col,rel) for col in rc]
            sample_distance=norm(rel);sample_incidence=dot(I(target['normal_world']),-rel)/sample_distance
            checks['focus'] &= float(sample_distance.lo)>=8-1e-7 and float(sample_distance.hi)<=25+1e-7
            checks['incidence'] &= float(sample_incidence.lo)>=math.cos(math.radians(30))-1e-7
            if qc[2].lo<=0:return False,dict(checks,camera_depth=False)
            qcd=[dot(cold,rel)+dot(col,vel) for col,cold in zip(rc,rd)]
            pixel=[f*qc[q]/qc[2] for q in [0,1]]
            border=min(border,float(pixel[0].lo+2560),float(2560-pixel[0].hi),float(pixel[1].lo+1912),float(1912-pixel[1].hi))
            pixel_rates=[]
            for q in [0,1]:
                rate=fd*qc[q]/qc[2]+f*(qcd[q]*qc[2]-qc[q]*qcd[2])/qc[2].square()
                pixel_rates.append(rate)
            blur=max(blur,float(norm(vector(pixel_rates)).hi)*.001)
            J=[]
            for tangent in target['tangent_basis']:
                z=[dot(col,I(tangent)) for col in rc]
                J.append([f*(z[q]*qc[2]-qc[q]*z[2])/qc[2].square() for q in [0,1]])
            aa=(J[0][0].square()+J[0][1].square())*.005**2-1
            bb=(J[1][0].square()+J[1][1].square())*.005**2-1
            ab=(J[0][0]*J[1][0]+J[0][1]*J[1][1])*.005**2
            gsd=min(gsd,float(aa.lo),float(bb.lo),float((aa*bb-ab.square()).lo))
        checks.update(blur=blur<=1+1e-7,border=border>=-1e-7,gsd_psd=gsd>=-1e-7)
        return all(checks.values()),dict(checks,blur_upper_px=blur,border_margin_px=border,gsd_psd_lower=gsd)

    def integrate_and_check(self,c,T,ac,start,task=None,energy_tolerance_Wh=.01,max_depth=20):
        failures=[];leaves=[];stack=[(np.asarray(c),np.asarray(ac),float(T),float(start),0,0.,1.)]
        while stack:
            cp,ap,duration,t,depth,s0,s1=stack.pop()
            detail={};ok=False
            try:
                ok,detail,body=self.check(cp,duration,ap,t)
                if task is not None:
                    optical,details=self.optics(cp,duration,ap,task,body);ok &= optical;detail['optics']=details
                power=body['power'];low=max(0,float(power.lo))*duration/3600;high=float(power.hi)*duration/3600
                precise=high-low<=energy_tolerance_Wh*(s1-s0)
                if ok and precise:
                    leaves.append(dict(start_s=t,end_s=t+duration,lower_Wh=low,upper_Wh=high,depth=depth));continue
            except (ArithmeticError,ValueError,FloatingPointError) as exc:detail={'enclosure_error':str(exc)}
            if depth>=max_depth:
                failures.append(dict(s0=s0,s1=s1,start_s=t,end_s=t+duration,checks=detail,status='failed_or_unresolved'));break
            left,right=split_controls(cp);al,ar=split_controls(ap);sm=(s0+s1)/2
            stack.append((right,ar,duration/2,t+duration/2,depth+1,sm,s1))
            stack.append((left,al,duration/2,t,depth+1,s0,sm))
            # A demonstrated/unresolved branch is enough to reject this
            # candidate. Remaining intervals stay unverified, not auto-passed.
            if failures:break
        return dict(passed=not failures and not stack,failures=failures,leaves=len(leaves),
            lower_Wh=sum(x['lower_Wh'] for x in leaves),upper_Wh=sum(x['upper_Wh'] for x in leaves),
            maximum_depth=max([x['depth'] for x in leaves] or [0]),
            all_intervals_checked=not failures and not stack,formal_directed_rounding_certificate=False,
            geometry_and_pairwise_included=False,method='adaptive polynomial hull + time-height forecast interval enclosure')
