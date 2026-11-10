"""Four-foot unilateral pad contact for smooth, impact-free docking.

These are declared reference gear/pad parameters, not measured M300 gear.
Static contact is a special case; body attitude and rotor states need not be
reset at release. Impulsive impacts and vehicle motion during docking are
excluded. Free flight requires zero contact force.
"""
from dataclasses import dataclass,asdict
import math
import numpy as np
from Agent.Publication import engineering_joint_transcription as jt


@dataclass(frozen=True)
class ContactParameters:
    foot_radius_m:float=.3
    foot_vertical_offset_m:float=.6
    friction_coefficient:float=.6
    normal_force_limit_per_foot_N:float=200.
    pad_length_m:float=4.
    pad_width_m:float=1.8
    ground_angular_acceleration_max_radps2:float=2000.
    ground_motor_torque_max_Nm:float=.5

    def validate(self):
        if any(not math.isfinite(x) or x<=0 for x in asdict(self).values()):
            raise ValueError('Positive finite contact parameters required')
        return self


def feet(p):
    a=p.foot_radius_m/math.sqrt(2)
    return [[a,a,-p.foot_vertical_offset_m],[-a,a,-p.foot_vertical_offset_m],
            [-a,-a,-p.foot_vertical_offset_m],[a,-a,-p.foot_vertical_offset_m]]


def foot_kinematics(y,p,ops):
    R=jt.rotation(y[6:10],ops)
    positions=[];velocities=[];arms=[]
    for r in feet(p):
        arm=jt.matvec(R,r);spin=jt.matvec(R,jt.cross(y[10:13],r))
        arms.append(arm)
        positions.append([y[k]+arm[k] for k in range(3)])
        velocities.append([y[3+k]+spin[k] for k in range(3)])
    return positions,velocities,arms


def contact_wrench(y,forces_world,p,ops):
    _,_,arms=foot_kinematics(y,p,ops)
    force=[sum(f[k] for f in forces_world) for k in range(3)]
    moments=[jt.cross(r,f) for r,f in zip(arms,forces_world)]
    moment_world=[sum(m[k] for m in moments) for k in range(3)]
    moment_body=jt.matvec(jt.transpose(jt.rotation(y[6:10],ops)),moment_world)
    return force,moment_body


def residuals(y,forces,p,pad_xyz=(0.,0.,0.),pad_yaw=0.,active=True):
    """Independent physical residuals; zero means satisfied at this instant."""
    p.validate();forces=np.asarray(forces,float)
    if forces.shape!=(4,3) or not np.isfinite(forces).all():raise ValueError('Four finite contact forces')
    pos,vel,_=foot_kinematics(y,p,jt.NumericOps);c,s=math.cos(pad_yaw),math.sin(pad_yaw)
    out=[]
    for k,(x,v,f) in enumerate(zip(pos,vel,forces)):
        gap=x[2]-pad_xyz[2];dx,dy=x[0]-pad_xyz[0],x[1]-pad_xyz[1]
        local=[c*dx+s*dy,-s*dx+c*dy];speed=math.hypot(v[0],v[1]);N=f[2]
        row=dict(foot=k,negative_normal=max(-N,0.),force_upper=max(N-p.normal_force_limit_per_foot_N,0.),
          friction_cone=max(np.linalg.norm(f[:2])-p.friction_coefficient*N,0.),
          inactive_force=0. if active else float(np.linalg.norm(f)),
          penetration=max(-gap,0.) if active else 0.,
          normal_gap=abs(N*gap),normal_velocity=abs(N*v[2]),
          support_x=max(N*(abs(local[0])-p.pad_length_m/2),0.),
          support_y=max(N*(abs(local[1])-p.pad_width_m/2),0.),
          coulomb_x=abs(speed*f[0]+p.friction_coefficient*N*v[0]),
          coulomb_y=abs(speed*f[1]+p.friction_coefficient*N*v[1]))
        out.append(row)
    return out


def add_contact(tr,name,y,active,pad_xyz,pad_yaw=0.,parameters=None):
    """Pad force variables and exact pointwise Coulomb graph.

    'active' is a binary pad-protection mode. Inactive feet carry no force;
    free-flight obstacle/terrain constraints remain the caller's duty. The
    speed-times-force graph permits zero-load feet to move without a spurious
    sticking constraint. No penetration is imposed while inside the pad mode.
    """
    p=(parameters or ContactParameters()).validate()
    if active.vtype()!='BINARY':raise ValueError('Binary pad mode required')
    positions,velocities,arms=foot_kinematics(y,p,jt.ScipOps)
    c,s=jt.scip.cos(pad_yaw),jt.scip.sin(pad_yaw)
    forces=[]
    for k,(x,v) in enumerate(zip(positions,velocities)):
        stem=f'{name}_foot{k}';cap=p.normal_force_limit_per_foot_N
        f=[tr.var(stem+'_Fx',-p.friction_coefficient*cap,p.friction_coefficient*cap),
           tr.var(stem+'_Fy',-p.friction_coefficient*cap,p.friction_coefficient*cap),
           tr.var(stem+'_N',0.,cap)];N=f[2];forces.append(f)
        gap=x[2]-pad_xyz[2];dx,dy=x[0]-pad_xyz[0],x[1]-pad_xyz[1]
        local=[c*dx+s*dy,-s*dx+c*dy]
        speed=jt.scip.sqrt(v[0]*v[0]+v[1]*v[1])
        tr.row(stem+'_mode',N-cap*active,'contact_mode','le')
        tr.row(stem+'_cone',f[0]*f[0]+f[1]*f[1]-(p.friction_coefficient*N)**2,'contact_friction','le')
        tr.row(stem+'_no_penetration',-gap,'contact_geometry','le',activation=active)
        tr.row(stem+'_normal_gap',N*gap,'contact_complementarity')
        tr.row(stem+'_normal_velocity',N*v[2],'smooth_contact')
        tr.row(stem+'_support_x',N*(local[0]**2-(p.pad_length_m/2)**2),'pad_support','le')
        tr.row(stem+'_support_y',N*(local[1]**2-(p.pad_width_m/2)**2),'pad_support','le')
        tr.row(stem+'_Coulomb_x',speed*f[0]+p.friction_coefficient*N*v[0],'contact_friction')
        tr.row(stem+'_Coulomb_y',speed*f[1]+p.friction_coefficient*N*v[1],'contact_friction')
    F,M=contact_wrench(y,forces,p,jt.ScipOps)
    return dict(forces=forces,force_world=F,moment_body=M,positions=positions,velocities=velocities,parameters=asdict(p),
                impact_model=False,complete_takeoff_landing_validation=False)
