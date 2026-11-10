"""Reference geometric image acceptance; no claim of real defect recognition."""
import math
import numpy as np
from Agent.Publication.engineering_inspection_primitives import camera_metrics


def camera_axes(body_to_world,gimbal_pitch,gimbal_yaw):
    d=np.array([math.cos(gimbal_pitch)*math.cos(gimbal_yaw),math.cos(gimbal_pitch)*math.sin(gimbal_yaw),math.sin(gimbal_pitch)])
    r=np.array([math.sin(gimbal_yaw),-math.cos(gimbal_yaw),0.])
    return np.asarray(body_to_world)@np.column_stack([r,np.cross(d,r),d])


def evaluate_capture(points,normal,tangents,poses,config,visibility_m,ray_visible):
    """Finite-exposure samples with actual camera positions and orientations.

    poses contains time_s,position_m,rotation and focal_length_mm. Occlusion
    callback must return a known boolean for every sampled ray. Unknown is
    rejected. Samples alone cannot certify all continuous exposure instants.
    """
    if len(poses)<2:raise ValueError('Exposure endpoint poses required')
    times=np.array([p['time_s'] for p in poses]);points=np.asarray(points,dtype=float)
    if not np.all(np.diff(times)>0) or not np.all(np.isfinite(points)):raise ValueError('Invalid exposure samples')
    if not math.isfinite(visibility_m) or visibility_m<0:raise ValueError('Visibility required')
    normal=np.asarray(normal,dtype=float)
    if abs(np.linalg.norm(normal)-1)>1e-6:raise ValueError('Unit surface normal required')
    base=math.hypot(config['image_width_px'],config['image_height_px'])/(2*math.tan(math.radians(config['diagonal_fov_deg_at_wide'])/2))
    reasons=[];pixels=[];worst_gsd=0.;near=float('inf');far=0.;max_incidence=0.
    for pose in poses:
        f=pose['focal_length_mm']
        if not config['focal_length_mm_bounds'][0]<=f<=config['focal_length_mm_bounds'][1]:reasons.append('focal_length')
        F=base*f/config['focal_length_mm_bounds'][0];pp=[]
        for q in points:
            distance=np.linalg.norm(np.asarray(pose['position_m'])-q)
            if distance<=0:raise ValueError('Camera on target')
            incidence=math.degrees(math.acos(float(np.clip(np.dot(normal,(np.asarray(pose['position_m'])-q)/distance),-1,1))))
            near=min(near,float(distance));far=max(far,float(distance));max_incidence=max(max_incidence,incidence)
            metrics=camera_metrics(q,pose['position_m'],pose['rotation'],tangents,F,F,config['image_width_px'],config['image_height_px'],config['frame_border_px'])
            if not metrics['in_fov']:reasons.append('FOV')
            gsd=metrics['worst_gsd_m_per_px']
            if gsd is None:reasons.append('degenerate_projection')
            else:worst_gsd=max(worst_gsd,gsd)
            pp.append(metrics['pixel'])
            visible=ray_visible(pose['position_m'],q)
            if visible is not True:reasons.append('occluded_or_unknown')
        pixels.append(pp)
    if near<config['focus_min_m'] or far>config['observation_max_m']:reasons.append('range')
    if max_incidence>config['incidence_angle_max_deg']:reasons.append('incidence')
    if worst_gsd>config['gsd_max_m_per_px']:reasons.append('GSD')
    # Diameter of the sampled pixel trace, using the 2D Euclidean norm.
    blur=None
    if all(v is not None for frame in pixels for v in frame):
        a=np.asarray(pixels);blur=max(float(np.linalg.norm(a[i]-a[j],axis=1).max()) for i in range(len(a)) for j in range(i+1,len(a)))
        if blur>config['blur_max_px']:reasons.append('blur')
    if visibility_m<1000:reasons.append('visibility')
    exposure=float(times[-1]-times[0])
    if not config['exposure_bounds_s'][0]-1e-9<=exposure<=config['exposure_bounds_s'][1]+1e-9:reasons.append('exposure')
    return dict(accepted=not reasons,reasons=sorted(set(reasons)),worst_gsd_m_per_px=worst_gsd,
                blur_px=blur,min_range_m=near,max_range_m=far,max_incidence_deg=max_incidence,
                evidence_kind='simulated_geometric_acceptance_not_real_image_QA',sample_count=len(poses))
