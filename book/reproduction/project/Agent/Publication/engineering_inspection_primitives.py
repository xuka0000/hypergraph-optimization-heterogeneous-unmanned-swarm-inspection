"""Small independently checkable primitives for the engineering model draft.

This is NOT the joint inspection solver or a flight safety certificate.
Coordinates: camera +z is optical forward; world +z is upward. SI except Wh.
"""
import math
import numpy as np


def camera_metrics(target_world, camera_world, camera_to_world, tangent_world,
                   fx_px, fy_px, width_px, height_px, margin_px=0):
    q = np.asarray(target_world, float)
    p = np.asarray(camera_world, float)
    r = np.asarray(camera_to_world, float)
    tangent = np.asarray(tangent_world, float)
    if r.shape != (3, 3) or tangent.shape != (3, 2):
        raise ValueError('Rotation must be 3x3, surface tangent basis 3x2')
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-8) or np.linalg.det(r) < 0:
        raise ValueError('Invalid camera rotation')
    if not np.allclose(tangent.T @ tangent, np.eye(2), atol=1e-8):
        raise ValueError('Surface tangent basis must be orthonormal')
    if min(fx_px, fy_px, width_px, height_px) <= 0 or margin_px < 0:
        raise ValueError('Invalid camera intrinsics')
    x, y, z = r.T @ (q - p)
    if z <= 0:
        return dict(in_fov=False, pixel=None, worst_gsd_m_per_px=None,
                    reason='target_not_in_front')
    pixel = np.array([fx_px*x/z+width_px/2, fy_px*y/z+height_px/2])
    dproj = np.array([[fx_px/z, 0., -fx_px*x/z**2],
                      [0., fy_px/z, -fy_px*y/z**2]])
    jacobian = dproj @ r.T @ tangent
    sigma = float(np.linalg.svd(jacobian, compute_uv=False)[-1])
    inside = (margin_px <= pixel[0] <= width_px-margin_px and
              margin_px <= pixel[1] <= height_px-margin_px)
    return dict(in_fov=bool(inside), pixel=pixel.tolist(),
                surface_jacobian_px_per_m=jacobian.tolist(),
                worst_gsd_m_per_px=None if sigma <= 1e-12 else 1/sigma,
                reason='degenerate_surface_projection' if sigma <= 1e-12 else None)


def short_exposure_blur_px(point_camera, point_camera_velocity, fx_px, fy_px,
                           exposure_seconds):
    x, y, z = np.asarray(point_camera, float)
    vx, vy, vz = np.asarray(point_camera_velocity, float)
    if z <= 0 or exposure_seconds < 0 or min(fx_px, fy_px) <= 0:
        raise ValueError('Invalid exposure geometry')
    rate = np.array([fx_px*(vx*z-x*vz)/z**2,
                     fy_px*(vy*z-y*vz)/z**2])
    return float(np.max(np.abs(rate))*exposure_seconds)


def bezier_state(control_points, duration_seconds, fraction):
    c = np.asarray(control_points, float)
    if c.ndim != 2 or c.shape[1] != 3 or len(c) < 4:
        raise ValueError('At least four 3D control points required')
    if duration_seconds <= 0 or not 0 <= fraction <= 1:
        raise ValueError('Invalid interval')
    states = []
    for order in range(4):
        n = len(c)-1
        basis = [math.comb(n, k)*fraction**k*(1-fraction)**(n-k)
                 for k in range(n+1)]
        states.append((np.asarray(basis) @ c).tolist())
        c = n*np.diff(c, axis=0)/duration_seconds
    return dict(position_m=states[0], velocity_mps=states[1],
                acceleration_mps2=states[2], jerk_mps3=states[3])


def required_thrust(mass_kg, acceleration_mps2, drag_force_world_n=(0, 0, 0)):
    if mass_kg <= 0:
        raise ValueError('Mass must be positive')
    force = mass_kg*(np.asarray(acceleration_mps2, float)+[0, 0, 9.81])
    force -= np.asarray(drag_force_world_n, float)
    magnitude = float(np.linalg.norm(force))
    if magnitude <= 1e-12:
        raise ValueError('Thrust-axis singularity')
    tilt = math.degrees(math.acos(float(np.clip(force[2]/magnitude, -1, 1))))
    return dict(thrust_n=magnitude, tilt_deg=tilt, axis=(force/magnitude).tolist())


def energy_after_interval(initial_wh, seconds, input_charge_w=0.,
                          efficiency=1., discharge_w=0.):
    if seconds < 0 or input_charge_w < 0 or discharge_w < 0 or not 0 < efficiency <= 1:
        raise ValueError('Invalid energy flow')
    # Do not clip: an overcharge/negative result must remain visible to validator.
    return float(initial_wh+(efficiency*input_charge_w-discharge_w)*seconds/3600)


def interval_peak(intervals):
    """Peak occupancy of half-open intervals; zero duration uses no resource."""
    changes = {}
    for start, end in intervals:
        if not (math.isfinite(start) and math.isfinite(end)) or end < start:
            raise ValueError('Invalid resource interval')
        if end == start:
            continue
        changes[start] = changes.get(start, 0)+1
        changes[end] = changes.get(end, 0)-1
    count = peak = 0
    for t in sorted(changes):
        count += changes[t]
        peak = max(peak, count)
    return peak
