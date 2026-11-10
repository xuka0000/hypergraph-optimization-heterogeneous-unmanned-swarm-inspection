"""Implied auxiliary bounds from the complete retained weather/DEM domains."""
import math
import numpy as np


def physical_auxiliary_bounds(cfg, forecast, terrain_index):
    # Bilinear values are convex combinations of each cell's four corners.
    # Both derivatives are affine, so their maxima occur at cell endpoints.
    values=forecast.values
    low=values.min(axis=(0,1));high=values.max(axis=(0,1))
    dt=np.zeros(4);dz=np.zeros(4)
    for h in range(len(forecast.times)-1):
        for k in range(3):
            lo,hi,a,b,c,d=forecast.coefficients(h,k)
            dt=np.maximum(dt,np.maximum(abs(b),abs(b+d*(hi-lo))))
            dz=np.maximum(dz,np.maximum(abs(c),abs(c+d*(forecast.times[h+1]-forecast.times[h]))))
    # Original unconditional component domains, also valid for inactive points.
    va=np.r_[5+np.maximum(abs(low[:2]),abs(high[:2])),5.]
    speed=float(np.linalg.norm(va)+1e-8)
    rho=float(high[2]);air=cfg['aircraft'];mass=air['mass_kg'];drag=.5*air['drag_CdA_m2']
    force_abs=mass*np.array([3.,3.,12.81])+drag*rho*speed*va
    fz_lower=mass*6.81-drag*rho*speed*5
    norm_lower=max(1e-6,float(fz_lower)-1e-7);norm_upper=float(np.linalg.norm(force_abs)+1e-7)
    heading_lower=max(1e-6,norm_lower/norm_upper-1e-8) if fz_lower>0 else 1e-6
    slopes=np.maximum(abs(terrain_index.slope_min),abs(terrain_index.slope_max))
    height_rate=5*(1+float(slopes.sum()))
    field_rate=dt+dz*height_rate
    vad=np.r_[3+field_rate[:2],3.]
    speed_rate=float(np.linalg.norm(vad))
    fd=mass*5+drag*(field_rate[2]*speed*va+rho*speed_rate*va+rho*speed*vad)
    # ||b3||<=1; the orthogonal contraction I-b3*b3^T has norm<=1.
    b3d=float(np.linalg.norm(fd))/norm_lower
    # axis is [b3z*cos(yaw), b3z*sin(yaw), -b3xy.heading].
    # Its derivative norm <=sqrt(2)*(||b3'||+|yaw'|).
    b1d=math.sqrt(2)*(b3d+math.radians(30))/heading_lower
    b2d=b3d+b1d
    pad=lambda x:float(x)*(1+1e-10)+1e-7
    return dict(weather_min=low.tolist(),weather_max=high.tolist(),
                field_time_derivative_abs=dt.tolist(),field_height_derivative_abs=dz.tolist(),
                terrain_gradient_abs=slopes.tolist(),height_rate_abs=height_rate,
                air_velocity_abs=[pad(x) for x in va],airspeed_upper=pad(speed),
                force_abs=[pad(x) for x in force_abs],force_norm_lower=norm_lower,
                force_norm_upper=pad(norm_upper),heading_norm_lower=heading_lower,
                air_velocity_dot_abs=[pad(x) for x in vad],airspeed_dot_abs=pad(speed_rate),
                force_dot_abs=[pad(x) for x in fd],body_z_dot_abs=pad(b3d),
                body_x_dot_abs=pad(b1d),body_y_dot_abs=pad(b2d),
                proof='full indexed bilinear cell domain, full DEM gradient domain, original state component bounds, Cauchy and contraction norms',
                hardware_limits_added=False)
