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

    def check(self,c,T,ac,start,origin=None,details=False,state_derivatives=None,pose_derivatives=None):
        if T<=0:raise ValueError('Positive interval duration required')
        origin=np.zeros(3) if origin is None else np.asarray(origin)
        world=np.asarray(c)+origin;air=self.cfg['aircraft'];p=hull(world)
        if np.any(self.terrain.tri.find_simplex(world[:,:2])<0):
            raise ArithmeticError('control hull not enclosed by source DEM')
        deriv=[];cp=np.asarray(c)-np.asarray(c)[0]
        if state_derivatives is None:
            for _ in range(3):cp=(len(cp)-1)*np.diff(cp,axis=0)/T;deriv.append(hull(cp))
        else:deriv=[hull(x) for x in state_derivatives]
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
        angles=hull(ac)
        rates,accels=([hull(7*np.diff(ac,axis=0)/T),hull(42*np.diff(ac,n=2,axis=0)/T**2)]
            if pose_derivatives is None else [hull(x) for x in pose_derivatives])
        # psi is actual unwrap(atan2(R21,R11)), not a heading projected
        # onto the thrust-normal plane. Positive b3.z follows from tilt<=25deg.
        yaw=angles[0];yd=rates[0];cs,sn=yaw.cos(),yaw.sin();cd,sd=-sn*yd,cs*yd
        q=vector([b3[2]*cs,b3[2]*sn,-b3[0]*cs-b3[1]*sn])
        qd=vector([b3d[2]*cs+b3[2]*cd,b3d[2]*sn+b3[2]*sd,
            -b3d[0]*cs-b3[0]*cd-b3d[1]*sn-b3[1]*sd])
        qn=norm(q);b1=q/qn;b1d=(qd-b1*dot(b1,qd))/qn
        b2=cross(b3,b1);b2d=cross(b3d,b1)+cross(b3,b1d)
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
                  angles=angles,rates=rates,power=power,ground=ground,height=height,
                  acceleration=a,jerk=jerk,terrain_gradient=grad,weather=W,weather_t=Wt,weather_z=Wz,
                  air_velocity=va,air_acceleration=vdot,hdot=hdot)
        return all(checks.values()),checks,body

    def energy_interval(self,c,T,derivatives,start,body,origin):
        """Intersect range enclosure with midpoint + bounded second derivative.

        |integral(P)-T*P(mid)| <= sup|P''|*T^3/24. The smooth formula is used
        only in one affine DEM cell/gradient and one bilinear forecast cell,
        without a climb-power kink. Otherwise retain the original range bound.
        No tolerance is loosened. Source: NIST DLMF3.5 / midpoint remainder.
        """
        from Agent.Publication.engineering_regional_geometry import evaluate
        power=body['power'];low=max(0.,float(power.lo))*T/3600;high=float(power.hi)*T/3600
        grad=body['terrain_gradient'];height=body['height'];v=body['velocity'];a=body['acceleration'];j=body['jerk']
        if np.max(grad.hi-grad.lo)>1e-12:return low,high,'range'
        try:
            left=self.forecast.cell(start,max(0.,float(height.lo)))
            right=self.forecast.cell(start+T,min(100.,float(height.hi)))
            if left!=right:return low,high,'range'
            if v[2].lo<0<v[2].hi and float(v[2].absmax())>1e-12:return low,high,'range'
            _,_,_,_,_,crosscoef=self.forecast.coefficients(*left)
            hdd=a[2]-dot(grad,a[:2]);hd=body['hdot']
            Wd=body['weather_t']+body['weather_z']*hd
            Wdd=2*I(crosscoef)*hd+body['weather_z']*hdd
            rho=body['weather'][2];rd=Wd[2];rdd=Wdd[2]
            u=body['air_velocity'];ud=body['air_acceleration'];udd=j-vector([Wdd[0],Wdd[1],0])
            speed=norm(u);speedprime=norm(ud);speedsecond=norm(udd)
            # Cauchy bound avoids division by zero at airspeed=0; ||u||^3
            # has bounded continuous second derivative there.
            drag_bound=(rdd.absmax()*float(speed.hi)**3+6*float(rd.absmax())*float(speed.hi)**2*float(speedprime.hi)
                +3*float(rho.hi)*(2*float(speed.hi)*float(speedprime.hi)**2+float(speed.hi)**2*float(speedsecond.hi)))
            hover=self.cfg['energy']['hover_power_W']*math.sqrt(1.225)
            hover_bound=hover*(.75*float(rd.absmax())**2/float(rho.lo)**2.5+.5*float(rdd.absmax())/float(rho.lo)**1.5)
            climb_bound=(self.cfg['aircraft']['mass_kg']*9.81/self.cfg['energy']['propulsion_efficiency']*float(j[2].absmax())
                if v[2].hi>0 else 0.)
            M=hover_bound+.5*self.cfg['aircraft']['drag_CdA_m2']*float(drag_bound)+climb_bound
            point=evaluate(c,.5)+origin;vel=evaluate(derivatives[0],.5)
            agl=float(point[2]-self.terrain(*point[:2]));w=self.forecast.evaluate(start+T/2,agl)['values']
            rel=vel-np.array([w['wind_east_ms'],w['wind_north_ms'],0.])
            P=hover/math.sqrt(w['density_kg_m3'])+.5*self.cfg['aircraft']['drag_CdA_m2']*w['density_kg_m3']*np.linalg.norm(rel)**3
            P+=self.cfg['aircraft']['mass_kg']*9.81*max(0.,vel[2])/self.cfg['energy']['propulsion_efficiency']
            error=(M*T**3/24+1e-7*T)/3600;mid=P*T/3600
            lower=max(low,mid-error);upper=min(high,mid+error)
            if lower>upper+1e-10:raise ArithmeticError('disjoint energy enclosures')
            return lower,upper,'midpoint_second_derivative'
        except (ValueError,ArithmeticError):return low,high,'range'

    def optics(self,c,T,ac,task,body,exposure=True):
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
            incidence=float(incidence.lo)>=math.cos(math.radians(30))-1e-7)
        radius=task['observation_requirement'].get('viewpoint_ball_radius_m')
        if radius is not None:
            checks['viewpoint']=float(np.linalg.norm(np.asarray(c)-nominal,axis=1).max())<=radius+1e-7
        # The viewing region (distance/incidence) persists over service;
        # image formation/blur is required only during the declared exposure.
        if not exposure:
            for point in target['surface_samples_utm_m']:
                delta=body['position']-I(point);distance=norm(delta)
                incidence=dot(I(target['normal_world']),delta)/distance
                checks['focus'] &= float(distance.lo)>=8-1e-7 and float(distance.hi)<=25+1e-7
                checks['incidence'] &= float(incidence.lo)>=math.cos(math.radians(30))-1e-7
            return all(checks.values()),dict(checks,exposure=False)
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
        # Differentiate once, in local coordinates, then subdivide the
        # derivative polynomials. Differentiating tiny world-coordinate hulls
        # amplifies UTM roundoff by duration^-3 and invents jerk violations.
        origin=np.asarray(c)[0].copy();cp=np.asarray(c)-origin;ap=np.asarray(ac)
        ds=[];work=cp
        for _ in range(3):work=(len(work)-1)*np.diff(work,axis=0)/T;ds.append(work)
        ads=[];work=ap
        for _ in range(2):work=(len(work)-1)*np.diff(work,axis=0)/T;ads.append(work)
        failures=[];leaves=[]
        # Split exactly at exposure boundaries before adaptive subdivision.
        # Derivatives are restricted, never re-differentiated at 1ms scale.
        from Agent.Publication.engineering_regional_geometry import restrict
        if task is not None and T<1.001-1e-9:
            return dict(passed=False,failures=[dict(s0=0.,s1=1.,checks={'exposure_contained':False})],leaves=0,
                lower_Wh=0.,upper_Wh=0.,all_intervals_checked=False)
        cuts=[0.,1/T,1.001/T,1.] if task is not None else [0.,1.]
        cuts=sorted(set(min(1.,x) for x in cuts))
        stack=[(restrict(cp,a,b),restrict(ap,a,b),float(T)*(b-a),float(start)+float(T)*a,0,a,b,
            [restrict(x,a,b) for x in ds],[restrict(x,a,b) for x in ads]) for a,b in reversed(list(zip(cuts,cuts[1:]))) if b>a]
        while stack:
            cp,ap,duration,t,depth,s0,s1,ds,ads=stack.pop()
            detail={};ok=False
            try:
                ok,detail,body=self.check(cp,duration,ap,t,origin=origin,state_derivatives=ds,pose_derivatives=ads)
                if task is not None:
                    exposed=s0*T>=1.-1e-10 and s1*T<=1.001+1e-10
                    optical,details=self.optics(cp+origin,duration,ap,task,body,exposure=exposed)
                    ok &= optical;detail['optics']=details;detail['exposure']=exposed
                low,high,method=self.energy_interval(cp,duration,ds,t,body,origin)
                precise=high-low<=energy_tolerance_Wh*(s1-s0)
                if ok and precise:
                    leaves.append(dict(start_s=t,end_s=t+duration,lower_Wh=low,upper_Wh=high,depth=depth,method=method));continue
            except (ArithmeticError,ValueError,FloatingPointError) as exc:detail={'enclosure_error':str(exc)}
            if depth>=max_depth:
                failures.append(dict(s0=s0,s1=s1,start_s=t,end_s=t+duration,checks=detail,status='failed_or_unresolved'));break
            left,right=split_controls(cp);al,ar=split_controls(ap);sm=(s0+s1)/2
            dd=[split_controls(x) for x in ds];aa=[split_controls(x) for x in ads]
            stack.append((right,ar,duration/2,t+duration/2,depth+1,sm,s1,[x[1] for x in dd],[x[1] for x in aa]))
            stack.append((left,al,duration/2,t,depth+1,s0,sm,[x[0] for x in dd],[x[0] for x in aa]))
            # A demonstrated/unresolved branch is enough to reject this
            # candidate. Remaining intervals stay unverified, not auto-passed.
            if failures:break
        return dict(passed=not failures and not stack,failures=failures,leaves=len(leaves),
            lower_Wh=sum(x['lower_Wh'] for x in leaves),upper_Wh=sum(x['upper_Wh'] for x in leaves),
            maximum_depth=max([x['depth'] for x in leaves] or [0]),
            all_intervals_checked=not failures and not stack,formal_directed_rounding_certificate=False,
            geometry_and_pairwise_included=False,method='subdivided derivative polynomials + range/midpoint second-derivative energy enclosure',
            energy_leaf_methods={name:sum(x['method']==name for x in leaves) for name in ['range','midpoint_second_derivative']})
