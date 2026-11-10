"""Source-derived nine-view geometry overlay, not an executable flight scenario.

Keep the existing observation IDs and OSM identities. Drawing dimensions are
transferred as a reference template, never asserted as Spanish as-built assets.
Unresolved operational quantities remain null or explicitly design assumptions.
"""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / '总结/07_证据与复现索引/综合报告检索/场景设置文献'
OUT = ROOT / 'Data/Scenarios/geographic_inspection'


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def read(p):
    return json.loads(p.read_text(encoding='utf-8-sig'))


def write(p, value):
    p.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf8')


def source(name, url, locator):
    p = BASE / name
    return dict(path=p.relative_to(ROOT).as_posix(), sha256=sha(p), url=url, locator=locator)


def main():
    ft = 0.3048
    peak = (117 + 3 / 12) * ft
    heights = dict(groundwire=peak - 5 * ft,
                   upper=peak - (5 + 10.5) * ft,
                   middle=peak - (5 + 10.5 + 16.75) * ft,
                   lower=(68 + 3 / 12) * ft)
    arm = 16 * ft
    assert math.isclose(heights['middle'] - heights['lower'], 16.75 * ft)
    sources = {
        'KPLC25': source('kplc_2025_tower_specification.pdf',
                        'https://kplc.co.ke/storage/01JQXMQ0QWCHBZKRQ1MBHAJADM.pdf',
                        'PDF p73 Appendix1A-1, standard-height132kV suspension tower; dimension chain visually checked'),
        'LI23': source('li2023_pdf.pdf', 'https://doi.org/10.3390/rs15030865',
                      'PDF p5-6 section3.2.1/Figure3; p21 Table7; p10 six battery compartments'),
        'GEMINI': source('geoscan_gemini_reference.html',
                         'https://geoscan-gemini.readthedocs.io/ru/latest/reference.html',
                         '20.1MP generation; specifications and endurance footnote'),
    }
    profile = dict(
        schema='nine_view_reference_geometry_1',
        role='engineering_drawing_template_transfer_not_as_built_not_flight_ready',
        user_decision='Keep nine fine-inspection photo tasks; do not replace by four-shot author example.',
        sources=sources,
        drawing=dict(tower_peak_height_m=peak, target_crossarm_levels_m=heights,
                     phase_half_crossarm_m=arm, groundwire_half_crossarm_m=arm,
                     groundwire_width_status='inferred_from_aligned_endpoints_in_drawing_not_separately_dimensioned',
                     target_scope='crossarm-end structural reference, not exact insulator/conductor surface',
                     conversion='international foot=0.3048m',
                     original_dimensions={'peak':"117ft3in", 'peak_to_groundwire':"5ft",
                                          'groundwire_to_upper':"10ft6in", 'upper_to_middle':"16ft9in",
                                          'middle_to_lower':"16ft9in", 'lower_to_ground':"68ft3in",
                                          'phase_half_span':"16ft"}),
        workflow=dict(photo_point_numbers=[1, 2, 3, 4, 5, 8, 9, 10, 11],
                      nonphoto_transfer_point_numbers=[6, 7],
                      transfer_height_above_groundwire_m=8,
                      source='LI23 p5-6',
                      transfer_scope='local cross-tower reference, not regional terrain clearance'),
        camera_distance=dict(center_offset_m=15, component_standoff_m=15-arm,
                             status='retained_design_value_not_source_verified',
                             parameterization='camera = crossarm_endpoint + outward_unit_normal * standoff',
                             top_overview_position='15m along negative line direction, design choice'),
        imaging=dict(source='GEMINI', sensor_width_mm=23.2, sensor_height_mm=15.4,
                     focal_mm=20, width_pixels=5456, height_pixels=3632,
                     formula='frontoparallel pixel_size_m=distance_m*sensor_width_mm/(focal_mm*width_pixels)',
                     component_plane_pixel_size_mm=(15-arm)*23.2/(20*5456)*1000,
                     component_plane_field_width_m=(15-arm)*23.2/20,
                     component_plane_field_height_m=(15-arm)*15.4/20,
                     status='optical_reference_calculation_only_not_defect_detectability_or_flight_compatibility'),
        timing=dict(per_photo_seconds=None, reference_net_tower_seconds=300,
                    source='LI23 Table7', status='aggregate_reference_not_uniform_point_duration',
                    calibration_equation='sum(photo_dwell_i)+within_tower_motion+other_net_overhead=300s only under matching definition/conditions'),
        source_packages=[
            dict(id='Li_mobile_inspection_system', source='LI23', net_tower_minutes=5,
                 reported_sortie_minutes=42, reported_max_distance_m=5000,
                 reported_max_towers_per_sortie=6, battery_change_seconds_upper_bound_exclusive=180,
                 nest_battery_compartments=6,
                 missing=['phase timing', 'loaded energy model', 'charging cycle', 'moving-vehicle swap validity']),
            dict(id='Gemini_optical_reference', source='GEMINI', endurance_upper_minutes=40,
                 endurance_conditions='height120m and wind<=2m/s',
                 max_vertical_speed_mps=5, max_horizontal_speed_mps=15,
                 status='manufacturer_limits_not_working_inspection_speeds_not_Li_hardware')],
        unresolved=['actual Ubeda tower types/heights and crossarm orientations',
                    'insulator/conductor geometry and visibility/occlusion',
                    'energized-line clearance and defect imaging requirements',
                    'per-photo and flight phase timing', 'loaded endurance and charging',
                    'parking permissions and road travel time',
                    'candidate/time discretization error'],
        active_scenario_modified=False, flights_recomputed=False)
    write(OUT/'nine_view_reference_profile.json', profile)
    case_checks = []
    photo_numbers = [1, 2, 3, 4, 5, 8, 9, 10, 11]
    roles = [('overview', 0, 'groundwire')] + [
        (side+'_'+level, sign, level) for side, sign in [('left', 1), ('right', -1)]
        for level in ['groundwire', 'upper', 'middle', 'lower']]
    for n in [60, 100, 150]:
        p = OUT/f'ubeda_osm_{n}.json'
        before = sha(p)
        scene = read(p)
        observations, towers, transfer_points = [], [], []
        residuals = []
        for t in scene['towers']:
            x, y = t['xy_m']; z = t['ground_m']
            dx, dy = t['line_direction']; norm = math.hypot(dx, dy)
            assert norm > 0
            dx, dy = dx/norm, dy/norm
            nx, ny = -dy, dx
            assert len(t['observations']) == 9
            towers.append(dict(id=t['id'], osm_id=t['osm_id'], lon=t['lon'], lat=t['lat'],
                               geometry_status='KPLC_reference_template_not_actual_type_assignment',
                               orientation_status='inherited_line_neighbor_estimate_not_surveyed_crossarm_bearing',
                               ground_status='inherited_terrain_estimate',
                               profile='nine_view_reference_profile.json', observations=t['observations']))
            for obs_id, number, (role, sign, level) in zip(t['observations'], photo_numbers, roles):
                h = heights[level]
                target = [x+sign*arm*nx, y+sign*arm*ny, z+h]
                camera = [x+sign*15*nx, y+sign*15*ny, z+h] if sign else [x-15*dx, y-15*dy, z+h]
                distance = math.dist(camera, target)
                residuals.append(abs(distance-(15-arm if sign else 15)))
                observations.append(dict(id=obs_id, tower=t['id'], osm_id=t['osm_id'],
                    li2023_point_number=number, role=role, xyz_m=camera, target_xyz_m=target,
                    relative_height_m=h, camera_target_distance_m=distance,
                    capture_seconds=None, geometry_source='KPLC25:p73 + LI23:p5-6',
                    camera_distance_status='design_15m_from_center_not_validated_clearance',
                    target_status='crossarm_end_reference_not_individual_defect_location'))
            for number, sign in [(6, 1), (7, -1)]:
                transfer_points.append(dict(tower=t['id'], osm_id=t['osm_id'], li2023_point_number=number,
                    xyz_m=[x+sign*15*nx, y+sign*15*ny, z+heights['groundwire']+8],
                    photo_task=False, source='LI23:p5-6', collision_checked=False))
        assert len(observations) == n*9 and len(transfer_points) == n*2
        assert len({o['id'] for o in observations}) == n*9
        assert {o['id'] for o in observations} == {o['id'] for o in scene['observations']}
        assert max(residuals) < 1e-8
        assert sha(p) == before
        overlay = dict(schema='reference_geometry_overlay_1', parent_scene=p.name,
                       parent_sha256=before, profile='nine_view_reference_profile.json',
                       profile_sha256=sha(OUT/'nine_view_reference_profile.json'),
                       scope='geometry review only; old recipes/flights must not be reused with changed coordinates',
                       coordinate_reference=scene['coordinate_reference'],
                       towers=towers, observations=observations, nonphoto_transfer_points=transfer_points,
                       executable=False, unresolved=profile['unresolved'])
        dest=OUT/f'ubeda_{n}_nine_view_reference.json';write(dest, overlay)
        case_checks.append(dict(towers=n, photo_tasks=len(observations), nonphoto_points=len(transfer_points),
                                file=dest.name, sha256=sha(dest),
                                all_ids_preserved=True, camera_distance_max_error_m=max(residuals),
                                original_scene_unchanged=sha(p)==before))
    receipt=dict(cases=case_checks, template_dimension_chain_passed=True,
                 profile_sha256=sha(OUT/'nine_view_reference_profile.json'),
                 code_sha256=sha(Path(__file__)), all_geometry_checks_passed=True,
                 physical_calibration_complete=False, formal_experiment_ready=False)
    write(OUT/'nine_view_reference_checks.json', receipt)
    lines=['九项精检：公开图纸参数与逐点迁移结果', '',
           '保留每塔9项拍摄任务：正面总览1项，左右两侧各4项；另有2个跨塔过渡点，不算拍摄任务。',
           '任务流程：Li等，Remote Sensing2023，15，865，第5—6页。塔型尺寸：KPLC2025公开招标文件第73页，132kV悬垂塔标准高度图。',
           '数据性质：真实OSM地理位置＋公开工程图纸参考塔型＋明确列出的设计变量。不是Úbeda逐塔实测资产数据。', '',
           '尺寸（m）', f'塔顶：{peak:.4f}；地线横担：{heights["groundwire"]:.4f}；上相：{heights["upper"]:.4f}；中相：{heights["middle"]:.4f}；下相：{heights["lower"]:.4f}。',
           f'相线横担半长：{arm:.4f}。地线横担半长由图中端点对齐推定，未单独标注尺寸；单独保留这一适用限制。',
           '侧面相机目标改为横担端部结构参考点。绝缘子、金具、导线表面位置和遮挡仍需细化，不能把横担端点当作全部缺陷位置。', '',
           '拍摄距离与成像',
           f'暂保留原设计的距塔中心15m，距横担端部为{15-arm:.4f}m。这一距离仍是设计值，不是已证实的带电线路安全间隙。',
           f'按Gemini20.1MP相机公开参数，正对目标平面时约{profile["imaging"]["component_plane_pixel_size_mm"]:.3f}mm/像素，视场约{profile["imaging"]["component_plane_field_width_m"]:.3f}×{profile["imaging"]["component_plane_field_height_m"]:.3f}m。这是光学推算，不能证明微小缺陷可检出。',
           '不把Gemini硬件规格拼接成Li论文设备的实测性能。', '',
           '时间与资源',
           'Li表7报告单塔净精检5min、单架次续航42min、最多6塔、最大5km、自动换电<3min；正文机巢有6个电池仓。它们是同一系统的参考指标，并不保证各上限能同时达到。',
           '每个拍摄点停留时间仍为空：5min包含哪些塔内动作须与轨迹定义对齐，不能直接平均分配，更不能继续把20s写成来源值。',
           '6个电池仓不代表无限满电电池，也不提供充电速率。', '',
           '逐点文件与核验']
    for row in case_checks:
        lines.append(f'{row["towers"]}塔：{row["photo_tasks"]}拍摄任务＋{row["nonphoto_points"]}过渡点；{row["file"]}；ID、归属、尺寸链与坐标距离核验通过。')
    lines += ['', '三组嵌套场景共2790条拍摄记录，但只有150座不同OSM杆塔，不能当作310座独立杆塔。',
              '这些文件是待接入的几何模板。原场景、航段时间和实验结果未覆盖；新几何接入求解前必须重新计算飞行候选，不能沿用旧飞行时间。',
              '仍待补齐：当地塔型/横担朝向、绝缘子及导线细部、成像与带电安全距离、逐阶段时间、载荷续航与充电、停车通行条件及数值离散误差。', '', '来源']
    for key, value in sources.items():lines += [key+' '+value['url'],value['path']+' | '+value['locator']]
    (ROOT/'总结/06_论文初稿/九项精检_几何与参数补齐.txt').write_text('\n'.join(lines)+'\n',encoding='utf8')
    print(json.dumps(receipt,ensure_ascii=False))


if __name__ == '__main__':
    main()
