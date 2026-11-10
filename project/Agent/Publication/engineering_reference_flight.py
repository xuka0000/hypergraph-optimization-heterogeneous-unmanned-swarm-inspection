"""17-state reference quadrotor, bounded thrust lag and fixed feedback.

Independent implementation of the revision-8 equations. Numerical parameters
are reference assumptions, not identified M300 dynamics. Rotor gyroscopic
moments, aeroelasticity and ground effect are outside this rigid-body model.
"""
from dataclasses import dataclass, asdict
import math
import numpy as np
from scipy.integrate import solve_ivp


@dataclass(frozen=True)
class FlightParameters:
    mass_kg: float=7.0
    gravity_mps2: float=9.81
    arm_radius_m: float=.4475
    inertia_x: float=.18
    inertia_y: float=.18
    inertia_z: float=.30
    drag_area_x_m2: float=.25
    drag_area_y_m2: float=.25
    drag_area_z_m2: float=.35
    thrust_coefficient: float=.000107296875
    torque_coefficient: float=.0000021459375
    rotor_inertia_kg_m2: float=.0002
    motor_efficiency: float=.8
    auxiliary_power_W: float=50.
    thrust_max_per_rotor_N: float=34.335
    thrust_rate_max_Nps: float=100.
    thrust_lag_s: float=.08
    position_gain_Npm: float=10.
    velocity_gain_Nspm: float=10.
    attitude_gain_Nm: float=6.
    angular_velocity_gain_Nms: float=1.8
    pad_friction: float=.6
    foot_radius_m: float=.3
    allowed_position_error_m: float=.5

    def validate(self):
        if any(not math.isfinite(v) or v<=0 for v in asdict(self).values()):
            raise ValueError('Positive finite flight parameters required')
        if self.motor_efficiency>1:raise ValueError('Motor efficiency')
        return self


def skew(v):
    x,y,z=v;return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def rotation(q):
    q=np.asarray(q,dtype=float);norm=np.linalg.norm(q)
    if not math.isfinite(norm) or norm<1e-12:raise ValueError('Invalid quaternion')
    w,x,y,z=q/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def quaternion_rate(q,omega):
    w=q[0];v=q[1:]
    return .5*np.r_[-np.dot(v,omega),w*omega+np.cross(v,omega)]


def allocation(p):
    a=p.arm_radius_m/math.sqrt(2)
    xy=np.array([[a,a],[-a,a],[-a,-a],[a,-a]])
    return np.vstack([np.ones(4),xy[:,1],-xy[:,0],
                      np.array([1.,-1.,1.,-1.])*p.torque_coefficient/p.thrust_coefficient])


def hover_state(position=(0.,0.,10.),p=None):
    p=p or FlightParameters()
    return np.r_[position,np.zeros(3),[1.,0.,0.,0.],np.zeros(3),np.full(4,p.mass_kg*p.gravity_mps2/4)]


def drag_world(y,wind,rho,p):
    R=rotation(y[6:10]);v=R.T@(y[3:6]-np.asarray(wind))
    return R@(-.5*rho*np.array([p.drag_area_x_m2,p.drag_area_y_m2,p.drag_area_z_m2])*np.abs(v)*v)


def plant_derivative(y,command,wind,rho,p,force_disturbance=None,torque_disturbance=None):
    y=np.asarray(y);command=np.asarray(command)
    if y.shape!=(17,) or command.shape!=(4,) or np.any(command<0) or np.any(command>p.thrust_max_per_rotor_N+1e-8):
        raise ValueError('State/command outside reference domain')
    if rho<=0 or not np.all(np.isfinite(y)) or not np.all(np.isfinite(wind)):raise ValueError('Invalid plant input')
    R=rotation(y[6:10]);J=np.array([p.inertia_x,p.inertia_y,p.inertia_z]);w=y[10:13]
    f=np.maximum(y[13:17],0.)
    force=R[:,2]*sum(f)+drag_world(y,wind,rho,p)-np.array([0.,0.,p.mass_kg*p.gravity_mps2])
    torque=allocation(p)[1:]@f-np.cross(w,J*w)
    if force_disturbance is not None:force+=np.asarray(force_disturbance)
    if torque_disturbance is not None:torque+=np.asarray(torque_disturbance)
    fdot=np.clip((command-f)/p.thrust_lag_s,-p.thrust_rate_max_Nps,p.thrust_rate_max_Nps)
    return np.r_[y[3:6],force/p.mass_kg,quaternion_rate(y[6:10],w),torque/J,fdot]


def actuator_power(y,fdot,p):
    f=np.maximum(np.asarray(y)[13:17],0.)
    omega=np.sqrt(f/p.thrust_coefficient)
    # Omega*Omega_dot=fdot/(2*kf), including its continuous f=0 limit.
    shaft=p.torque_coefficient*omega**3+p.rotor_inertia_kg_m2*np.asarray(fdot)/(2*p.thrust_coefficient)
    return float(np.maximum(shaft,0.).sum()/p.motor_efficiency+p.auxiliary_power_W)


def reference_controller(y,position,velocity,acceleration,yaw,wind,rho,p):
    """Frozen geometric PD controller, no desired-rate feedforward.

    Related to Lee et al.'s geometric construction; this is not a faithful
    public baseline or the full tracking law in that paper. Saturation and
    thrust lag invalidate importing its stability theorem without a proof.
    """
    R=rotation(y[6:10])
    force=(-p.position_gain_Npm*(y[:3]-position)-p.velocity_gain_Nspm*(y[3:6]-velocity)
           +p.mass_kg*(acceleration+np.array([0.,0.,p.gravity_mps2]))-drag_world(y,wind,rho,p))
    z=force/np.linalg.norm(force);heading=np.array([math.cos(yaw),math.sin(yaw),0.])
    b2=np.cross(z,heading)
    if np.linalg.norm(b2)<1e-8:raise ValueError('Singular desired heading')
    b2/=np.linalg.norm(b2);Rd=np.column_stack([np.cross(b2,z),b2,z])
    S=.5*(Rd.T@R-R.T@Rd);err=np.array([S[2,1],S[0,2],S[1,0]])
    torque=-p.attitude_gain_Nm*err-p.angular_velocity_gain_Nms*y[10:13]
    torque+=np.cross(y[10:13],np.array([p.inertia_x,p.inertia_y,p.inertia_z])*y[10:13])
    wrench=np.r_[max(0.,np.dot(force,R[:,2])),torque]
    raw=np.linalg.solve(allocation(p),wrench)
    lower=np.maximum(0.,y[13:17]-p.thrust_lag_s*p.thrust_rate_max_Nps)
    upper=np.minimum(p.thrust_max_per_rotor_N,y[13:17]+p.thrust_lag_s*p.thrust_rate_max_Nps)
    return np.clip(raw,lower,upper),bool(np.any(raw<lower) or np.any(raw>upper))


def pad_reaction(y,command,wind,rho,p):
    """Static level-pad contact certificate, with continuous rotor ramp.

    This does not invent an impact law. A landing must reach zero relative
    velocity before entering this mode; high-speed impacts are rejected.
    """
    if np.linalg.norm(y[3:6])>1e-8 or np.linalg.norm(y[10:13])>1e-8 or np.linalg.norm(rotation(y[6:10])-np.eye(3))>1e-8:
        raise ValueError('Contact requires stationary level docking')
    d=plant_derivative(y,command,wind,rho,p)
    F=-p.mass_kg*d[3:6];moment=-allocation(p)[1:]@y[13:17]
    n=F[2]
    valid=n>=-1e-9 and np.linalg.norm(F[:2])<=p.pad_friction*max(n,0)+1e-9 and np.linalg.norm(moment)<=p.foot_radius_m*max(n,0)+1e-9
    return dict(valid=bool(valid),normal_N=float(n),friction_N=float(np.linalg.norm(F[:2])),moment_Nm=float(np.linalg.norm(moment)),fdot=d[13:17])


def simulate_tracking(y0,duration_s,reference,weather,p,*,max_step_s=.02,rtol=1e-8):
    p.validate()
    def rhs(t,z):
        wind,rho=weather(t,z[:3]);pos,vel,acc,yaw=reference(t)
        cmd,_=reference_controller(z,pos,vel,acc,yaw,wind,rho,p)
        return plant_derivative(z,cmd,wind,rho,p)
    sol=solve_ivp(rhs,(0.,duration_s),y0,rtol=rtol,atol=1e-10,max_step=max_step_s,
                  t_eval=np.linspace(0.,duration_s,int(round(duration_s/max_step_s))+1))
    if not sol.success:raise ArithmeticError(sol.message)
    errors=[];powers=[];saturations=0
    for t,y in zip(sol.t,sol.y.T):
        pos,vel,acc,yaw=reference(t);wind,rho=weather(t,y[:3])
        cmd,sat=reference_controller(y,pos,vel,acc,yaw,wind,rho,p)
        d=plant_derivative(y,cmd,wind,rho,p)
        errors.append(float(np.linalg.norm(y[:3]-pos)));powers.append(actuator_power(y,d[13:],p));saturations+=int(sat)
    return dict(t=sol.t,y=sol.y,position_error_m=np.array(errors),power_W=np.array(powers),saturation_samples=saturations)
