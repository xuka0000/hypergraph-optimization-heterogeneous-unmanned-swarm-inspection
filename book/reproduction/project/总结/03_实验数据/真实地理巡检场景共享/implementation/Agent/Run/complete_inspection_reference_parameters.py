"""Executable parameter calculations, with transfer/calibration explicitly typed.

This supplies a planning reference, not local asset measurements or flight control.
No old flight catalogue or algorithm result is modified or reused as a new result.
"""
import hashlib
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'Data/Scenarios/geographic_inspection'
BASE=ROOT/'总结/07_证据与复现索引/综合报告检索/场景设置文献'
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def write(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def leg_seconds(distance, speed=1., acceleration=3.):
    """Symmetric rest-to-rest triangular/trapezoidal motion, SI units."""
    if distance < 0 or speed <= 0 or acceleration <= 0:raise ValueError('invalid motion inputs')
    if distance == 0:return 0.
    switch=speed*speed/acceleration
    return 2*math.sqrt(distance/acceleration) if distance <= switch else distance/speed+speed/acceleration


def tower_route(distance_from_center=15., heights=(34.2138,31.0134,25.908,20.8026)):
    """Local x=line direction, y=crossarm direction, z=height above ground.

    Square detour from overview to left side is our geometric construction;
    crossing points6/7 follow Li's8m height increment. Stops at corners are a
    declared conservative kinematic simplification, not measured trajectories.
    """
    d=distance_from_center;h=heights[0]
    route=[dict(number=1,xyz=[-d,0,h],photo=True),
           dict(number='overview_detour',xyz=[-d,d,h],photo=False)]
    route += [dict(number=i,xyz=[0,d,z],photo=True) for i,z in zip([2,3,4,5],heights)]
    route += [dict(number=6,xyz=[0,d,h+8],photo=False),dict(number=7,xyz=[0,-d,h+8],photo=False)]
    route += [dict(number=i,xyz=[0,-d,z],photo=True) for i,z in zip([8,9,10,11],heights)]
    return route


def timing(distance_from_center=15., speed=1., acceleration=3., net_seconds=300., dwell=None):
    route=tower_route(distance_from_center);moving=0.
    for previous,current in zip(route,route[1:]):
        length=math.dist(previous['xyz'],current['xyz'])
        current['incoming_length_m']=length
        current['incoming_seconds']=leg_seconds(length,speed,acceleration)
        moving+=current['incoming_seconds']
    effective=(net_seconds-moving)/9 if dwell is None else dwell
    if effective < 0:raise ValueError('negative residual dwell; source aggregate inconsistent with geometry/motion')
    clock=0.;events=[]
    for point in route:
        clock+=point.get('incoming_seconds',0.)
        if point['photo']:
            clock+=effective;events.append(dict(photo_number=point['number'],completed_seconds=clock,effective_service_seconds=effective))
    return dict(route=route,events=events,moving_seconds=moving,effective_service_seconds=effective,total_seconds=clock,
                meaning='equal residual allocation of all non-translational net overhead; not observed shutter/hover time')


def charging_queue(returns,charge_seconds=3600.,cooldown_seconds=0.):
    """One paired-battery charging channel, FIFO reference scheduling rule."""
    busy=0.;result=[];seen=set()
    for pair_id,returned in sorted(returns,key=lambda row:(row[1],row[0])):
        if pair_id in seen:raise ValueError('duplicate return in this charging batch')
        if returned < 0 or charge_seconds <= 0 or cooldown_seconds < 0:raise ValueError('invalid charging inputs')
        seen.add(pair_id);start=max(busy,returned+cooldown_seconds);busy=start+charge_seconds
        result.append(dict(pair=pair_id,returned_seconds=returned,charge_start_seconds=start,available_seconds=busy))
    return result


def main():
    geom=read(OUT/'nine_view_reference_profile.json')
    sources=dict(geom['sources'])
    for key,name,url,loc in [
        ('M300','dji_m300_manual.pdf','https://dl.djicdn.com/downloads/matrice-300/20230526UM/M300_RTK_User_Manual_EN_v3.4.pdf','p30 charging priority and duration; p32 storage capacity and lab conditions; p35 paired battery replacement; p95 700g-load endurance'),
        ('H20','dji_h20_specs.html','https://enterprise.dji.com/zenmuse-h20-series/specs','H20 mass678±5g; zoom camera5184x3888, diagonal FOV66.6–4deg, telephoto focus>=8m'),
        ('RISS26','riss2026_insulators.pdf','https://arxiv.org/abs/2602.24011','preprint, p6 TablesI/II simulation distances8–10m; p7 TableIII field speeds1m/s, acceleration3m/s2'),
        ('BY26','bychkov2026.pdf','https://doi.org/10.3390/drones10070511','p24 simulation10km/h, launch preparation2min, landing procedure4min; Table5 8h working day'),
        ('WA20','wang2020.pdf','https://doi.org/10.3390/en13020326','p11 Table1, UAV5m/s as a published simulation condition')]:
        p=BASE/name;sources[key]=dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p),url=url,locator=loc)
    sources['LI23']['locator']='p5-6 section3.2.1; p10 six battery compartments; p21 Table7'
    model=timing();d=geom['camera_distance']['component_standoff_m']
    # Diagonal field of view must use image diagonal, not image width.
    diag_px=math.hypot(5184,3888)
    optical=[]
    for dfov in [66.6,4.]:
        diag_m=2*d*math.tan(math.radians(dfov)/2)
        optical.append(dict(diagonal_fov_degrees=dfov,distance_m=d,pixel_size_mm=diag_m/diag_px*1000,
                            field_width_m=diag_m*5184/diag_px,field_height_m=diag_m*3888/diag_px))
    params=[]
    def add(key,value,kind,refs,basis):params.append(dict(key=key,value=value,evidence_kind=kind,sources=refs,basis=basis))
    add('asset_geometry','KPLC132kV standard-height suspension template on OSM geography','transfer',['KPLC25'],'Explicit synthetic asset template, not Spanish as-built type; positions retain OSM identity.')
    add('nine_photo_tasks',[1,2,3,4,5,8,9,10,11],'source',['LI23'],'Photo tasks in Figure3; points6/7 transfer only.')
    add('camera_center_offset_m',15,'design_with_reference_range',['RISS26','H20'],'Retain15m center offset; derived10.1232m crossarm-end distance is close to the8–10m localization study and exceeds telephoto minimum focus8m. Neither establishes energized-line safety.')
    add('local_motion_speed_mps',1,'transfer',['RISS26'],'Horizontal and vertical field limits; moved to a different reference airframe, not measured M300 working speed.')
    add('local_acceleration_mps2',3,'transfer',['RISS26'],'Rest-to-rest motion model uses reported field acceleration limit; stop at each corner is a design simplification.')
    add('net_tower_seconds',300,'transfer',['LI23'],'Reference net task time. Does not include inter-tower/road transfer or prove this tower takes300s in the field.')
    add('effective_nontranslation_seconds_per_photo',model['effective_service_seconds'],'calibrated',['LI23','RISS26','KPLC25'],'(300 minus computed local motion)/9. Equal allocation is not identifiable from source data; includes settling/aiming/capture overhead, not shutter duration.')
    add('inter_tower_reference_speed_mps',5,'transfer',['WA20'],'Published simulation operating speed, not manufacturer maximum.')
    add('airframe_payload','M300 RTK + H20;700g reference load','reference_hardware',['M300','H20'],'H20 nominal678g;700g endurance test is nearby load reference, not exact flight-log calibration.')
    add('hover_endurance_reference_seconds',2580,'source',['M300'],'43min at700g load/sea level, maximum reference; not guaranteed mixed-mission time.')
    add('forward_endurance_reference_seconds',2700,'source',['M300'],'45min at700g load/sea level, maximum reference; climb/wind not independently identified.')
    add('landing_energy_reserve_fraction',0.2,'design',[],'Research operating margin, not a manufacturer-certified safety threshold; inspect0.2/0.3/0.4 separately.')
    add('reference_energy_equation','E=local_and_vertical_seconds/2580+transit_seconds/2700','model',['M300'],'Linear normalized-time surrogate, not calibrated energy or battery SOC dynamics; ascent treated at hover rate is an explicit approximation.')
    add('full_charge_hover_budget_seconds',2064,'derived',['M300'],'2580*(1-0.2), not a measured34.4min task allowance.')
    add('cells_per_aircraft',2,'source',['M300'],'Keep paired TB60 cells; battery inventory uses pairs.')
    add('station_storage_cells',8,'source',['M300'],'BS60 stored TB60 count; excludes batteries installed on aircraft.')
    add('simultaneous_charging_pairs',1,'source',['M300'],'Manual charging logic selects two TB60 cells; eight storage slots do not mean four concurrent charging pairs.')
    add('full_charge_seconds',3600,'source',['M300'],'220–240V laboratory room-temperature reference; not total hot-battery turnaround.')
    add('partial_charge_20_to_90_seconds',1800,'source',['M300'],'Separate regime: does not mean full charge in30min. Not used in full-charge queue.')
    add('battery_cooling_delay_seconds',0,'idealized_condition',['M300'],'Only for already-chargeable batteries at reference thermal condition. Sweep0/900/1800s; no field cooling measurement available.')
    add('fleet_resources',dict(vehicles=2,uavs_per_vehicle=2,total_pairs_per_vehicle=4,stations_per_vehicle=1),'design',['M300'],'Resource scenario, not actual local fleet. Per vehicle2 pairs installed+2 spare pairs; all methods share resources.')
    add('charging_power_supply','220–240V available while vehicle is stationary','design',['M300'],'No charging during road motion is assumed; vehicle power hardware is not specified by M300 manual.')
    add('charge_queue_rule','FIFO','design',['M300'],'Abstract one-pair charger; not an emulation of DJI highest-residual-SOC firmware scheduling.')
    add('launch_preparation_seconds',120,'transfer',['BY26'],'Use published manual-operation simulation; not Li automatic nest timing.')
    add('landing_handling_seconds',240,'transfer',['BY26'],'Published landing procedure reference; airborne descent is counted in path time separately, so accounting boundary needs field calibration.')
    add('additional_automatic_swap_seconds',0,'model_boundary',['BY26','M300'],'Do not add Li<180s automatic swap to manual operations. Battery change/reset must fit within the120+240s handling allowance as a stated modeling assumption.')
    add('road_reference_speed_kmph',10,'transfer',['BY26'],'Published rural-access simulation speed; not actual Úbeda traffic. Recompute graph paths under new edge weights before using.')
    add('working_day_seconds',28800,'transfer',['BY26'],'8h published working-day comparison. Core full-coverage evaluation should also report completion time rather than hide misses behind this horizon.')
    add('task_priority',1,'design',[],'Equal-weight tower count; no fabricated fault-risk weights.')
    add('task_completion','all nine photo services completed','design',['LI23'],'Scheduling completion does not assert actual defect recognition/image acceptance.')
    add('time_representation','continuous seconds','numerical_design',[],'Reference calculations avoid minute-ceiling errors; existing optimizer is not yet converted.')
    sensitivities=[]
    # Keep the base calibrated dwell fixed. Re-fitting every scenario would erase
    # the distance/speed effect the sensitivity calculation is intended to show.
    for standoff in [8.,10.,15.]:
        for speed in [.5,1.,1.5]:
            calc=timing(standoff+geom['drawing']['phase_half_crossarm_m'],speed=speed,dwell=model['effective_service_seconds'])
            sensitivities.append(dict(component_standoff_m=standoff,speed_mps=speed,total_tower_seconds=calc['total_seconds'],
                                      dwell_refitted=False,role='design sensitivity, not measured performance'))
    demo=charging_queue([('pair1',600),('pair2',600),('pair3',1200),('pair4',1200)])
    checks=dict(reference_tower_seconds=model['total_seconds'],photo_count=len(model['events']),
                effective_dwell_nonnegative=model['effective_service_seconds']>=0,
                battery_pair_count=4,battery_cell_count=8,
                four_return_charge_finish_seconds=[x['available_seconds'] for x in demo],
                old_scenarios_replaced=False,old_flight_catalogues_reused=False,formal_experiments_started=False)
    assert math.isclose(model['total_seconds'],300,abs_tol=1e-9)
    assert len(model['events'])==9 and all(x['charge_start_seconds']>=x['returned_seconds'] for x in demo)
    assert all(b['charge_start_seconds']>=a['available_seconds'] for a,b in zip(demo,demo[1:]))
    assert demo[1]['available_seconds']>demo[0]['available_seconds']
    # Known kinematics: distance1m,speed1,a1 gives2s; distance.25 gives1s.
    assert leg_seconds(1,1,1)==2 and leg_seconds(.25,1,1)==1 and leg_seconds(0)==0
    case_rows=[]
    for n in [60,100,150]:
        p=OUT/f'ubeda_{n}_nine_view_reference.json';overlay=read(p)
        records=[dict(observation_id=o['id'],tower=o['tower'],osm_id=o['osm_id'],
                      photo_number=o['li2023_point_number'],effective_service_seconds=model['effective_service_seconds'],
                      evidence_kind='calibrated_equal_residual_not_measured') for o in overlay['observations']]
        dest=OUT/f'ubeda_{n}_nine_view_service_parameters.json'
        routes=[]
        for tower in overlay['towers']:
            obs=[o for o in overlay['observations'] if o['tower']==tower['id']]
            by={o['li2023_point_number']:o for o in obs}
            center=by[1]['target_xyz_m'];height=geom['drawing']['target_crossarm_levels_m']['groundwire']
            ground=center[2]-height
            # Recover local axes from persisted geometry, avoiding a second
            # independent interpretation of line orientation at subset boundaries.
            axis=[(center[i]-by[1]['xyz_m'][i])/15 for i in [0,1]]
            normal=[(by[2]['xyz_m'][i]-center[i])/15 for i in [0,1]]
            route=[]
            for point in model['route']:
                xx,yy,zz=point['xyz']
                world=[center[0]+xx*axis[0]+yy*normal[0],center[1]+xx*axis[1]+yy*normal[1],ground+zz]
                if point['photo']:
                    assert math.dist(world,by[point['number']]['xyz_m'])<1e-7
                route.append(dict(photo=point['photo'],point_number=point['number'],xyz_m=world))
            routes.append(dict(tower=tower['id'],osm_id=tower['osm_id'],path=route,
                               total_seconds=model['total_seconds'],events=model['events'],
                               collision_status='local_reference_construction_not_field_obstacle_checked'))
        write(dest,dict(parent_geometry=p.name,parent_sha256=sha(p),records=records,tower_routes=routes,
                       note='Use with new route costs; never add300s again or reuse old flight durations.'))
        assert len(records)==n*9
        case_rows.append(dict(towers=n,photo_records=len(records),file=dest.name,sha256=sha(dest)))
    result=dict(schema='inspection_operating_reference_1',sources=sources,parameters=params,
                local_tower_timing=model,optical_calculations=optical,sensitivity=sensitivities,
                charger_queue_example=demo,case_rows=case_rows,checks=checks,
                source_status='public-source reference plus explicit transfer/calibration/design; not all measured',
                applicability='nine-photo scheduling reference; no new heterogeneous-hardware claim',
                not_closed=['actual Spanish tower types and detailed insulator/conductor geometry',
                            'energized-line clearance and image defect acceptance criteria',
                            'field phase timing/energy/thermal effects and handling boundary',
                            'parking authorization and driveability',
                            'integration with flight generator, exact road cost recomputation and scheduler resource enforcement'],
                formal_experiment_ready=False,code_sha256=sha(Path(__file__)))
    write(OUT/'inspection_operating_reference.json',result)
    rows=['九项精检：作业参数补充与可复算配置','',
          '本轮完成公开来源补查、运动计时、单塔净时间分配、相机成像计算与有限电池充电排队代码。',
          '配置属于“公开地理数据＋工程图纸模板＋文献迁移及明确设定”的仿真参考，不是当地实测系统。没有把文献未报告的数值伪装成实测。','',
          '1. 单塔时间',
          f'9项精检净参考时间300s；按1m/s、3m/s²的启停运动模型计算移动{model["moving_seconds"]:.3f}s，剩余平均分给每项约{model["effective_service_seconds"]:.3f}s。',
          '这一每项时间包含云台调整、稳定、拍摄等未分辨开销，只是满足总时间约束的一种分配。不能把它称作论文报告的逐点停留时间。',
          '距离/速度敏感性计算固定这个分配值，避免每种条件都重新拟合成300s、掩盖条件影响。','',
          '2. 设备、电池与充电',
          'M300官方手册700g载荷参考：前飞45min、悬停43min；H20名义678g。不是无载55min，也不是实际巡检保证时长。',
          '两块TB60组成一对；BS60可存8块，充电按一对处理。220–240V下全充约60min，20%到90%约30min，二者不能混用。',
          '默认2车、每车2机、4对电池、1个充电站是实验资源设定。原先无限满电备用电池及行车中自动换电假设不进入这份参考配置。',
          '20%能量预留是研究设定；简化能耗公式给出的34.4min悬停预算是推导值。充电默认电池已处于可充温度；冷却0/15/30min单列敏感性。',
          '4对电池分别在10/10/20/20min返还、单通道全充时，依次在70/130/190/250min可用。代码显式计入排队，没有全部同时恢复满电。','',
          '3. 成像核算',
          f'距横担端部{d:.4f}m，按H20的对角视场和5184×3888分辨率计算：']
    for x in optical:rows.append(f'对角视场{x["diagonal_fov_degrees"]}°：视场{x["field_width_m"]:.3f}×{x["field_height_m"]:.3f}m，约{x["pixel_size_mm"]:.3f}mm/像素。')
    rows += ['长焦端能看清小范围，不能一张照片覆盖整个横担；九项是九个拍摄服务位，不等于九张照片足以检查所有缺陷。尚无检出率/遮挡证据，未据像素大小宣称缺陷检测达标。','',
             '4. 地面作业','采用Bychkov公开仿真的路速10km/h、起飞准备2min、降落作业4min作为迁移参考；不另加Li自动机巢换电时间。8h来自其工作日比较。',
             '这些值不是Úbeda实测；道路必须按新速度重新求最短时间。停车与车辆供电均为仿真条件，未声称获得现场许可。','',
             '5. 逐项配置（来源、推导与设定分开）']
    for p in params:rows.append(f'{p["key"]} = {json.dumps(p["value"],ensure_ascii=False)} | {p["evidence_kind"]} | {",".join(p["sources"]) or "研究设计"}\n{p["basis"]}')
    rows += ['', '6. 数据与状态','60/100/150塔分别生成540/900/1350条逐项服务时间参数，均绑定原几何文件哈希。计算与已知算例核验通过。',
             '本轮未启动算法实验。几何/服务参数尚须接入航段生成器；现有调度器仍需真正执行有限电池、充电及作业互斥，不能把配置文件存在当作已经完成接入。',
             '现场仍无法由文献补成实测：当地塔型和导线细部、带电安全间隙、缺陷成像验收、阶段耗时及热/风/载荷能耗、停车条件。','', '来源']
    for k,v in sources.items():rows += [k+' '+v['url'],v['path']+' | '+v['locator']]
    (ROOT/'总结/06_论文初稿/九项精检_作业参数与推导.txt').write_text('\n'.join(rows)+'\n',encoding='utf8')
    print(json.dumps(dict(checks=checks,cases=case_rows,parameter_entries=len(params)),ensure_ascii=False))


if __name__=='__main__':main()
