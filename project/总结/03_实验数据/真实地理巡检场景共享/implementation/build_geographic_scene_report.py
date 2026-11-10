"""Table-only, source-linked data handoff report. No experiments are run."""
from pathlib import Path
import html, json, shutil, sys
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'总结/03_实验数据/真实地理巡检场景共享'
def load(p):return json.loads(p.read_text(encoding='utf8'))
cat=load(OUT/'catalog.json');scenes=[load(OUT/c['file']) for c in cat['scenes']]
SHORT=['作者KML 6塔','作者KML 27塔','Úbeda 60塔','Úbeda 100塔','Úbeda 150塔','Rommerskirchen','Brauweiler','Villevaudé']
pdfmetrics.registerFont(TTFont('Song','C:/Windows/Fonts/simsun.ttc',subfontIndex=0))
normal=ParagraphStyle('body',fontName='Song',fontSize=9.2,leading=12,wordWrap='CJK',spaceAfter=0)
small=ParagraphStyle('small',parent=normal,fontSize=8.5,leading=11)
heading=ParagraphStyle('heading',parent=normal,fontSize=13,leading=18,spaceBefore=6,spaceAfter=8,keepWithNext=True)
title=ParagraphStyle('title',parent=normal,fontSize=20,leading=28,spaceAfter=16)
story=[]
def txt(x):
    if x is None:return '未设置 / 未取得'
    if isinstance(x,float):return f'{x:.7g}'
    if isinstance(x,(list,dict)):return json.dumps(x,ensure_ascii=False,separators=(',',':'))
    return str(x)
def p(x,style=normal):return Paragraph(html.escape(txt(x)).replace('\n','<br/>'),style)
def section(name):story.append(p(name,heading))
def table(headers,rows,widths,compact=False):
    style=small if compact else normal
    data=[[p(x,style) for x in headers]]+[[p(x,style) for x in r] for r in rows]
    t=Table(data,colWidths=[w*mm for w in widths],repeatRows=1,hAlign='LEFT')
    t.setStyle(TableStyle([('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),3),('RIGHTPADDING',(0,0),(-1,-1),3),('TOPPADDING',(0,0),(-1,-1),2.8 if compact else 3),('BOTTOMPADDING',(0,0),(-1,-1),2.8 if compact else 3),('LINEABOVE',(0,0),(-1,0),.8,colors.black),('LINEBELOW',(0,0),(-1,0),.5,colors.black),('LINEBELOW',(0,-1),(-1,-1),.8,colors.black)]))
    story.extend([t,Spacer(1,9)])
def page():story.append(PageBreak())

story.append(p('真实地理巡检场景\n搭建与数据清单',title))
table(['项目','内容'],[
 ('交付范围','8个真实地理来源场景；每个场景1个独立JSON'),
 ('源数据快照','KML/OSM及旧SRTM：2026-11-05/06；新区域Copernicus DEM：2026-11-08'),
 ('报告生成日期',cat.get('updated_at',cat['created_at'])[:10]),
 ('坐标约定','WGS84 / EPSG:4326；longitude为经度，latitude为纬度；JSON保留源精度'),
 ('坐标表精度','经纬度显示8位小数；米制坐标和高程显示3位小数；不代表测量精度'),
 ('参数类别','地图原始值、派生值、文献迁移、建模设定、未知值分别登记'),
 ('完整参数与实体','JSON包含杆塔、线路、道路、候选停车点、基地、地形采样、拍摄点、来源与单位'),
 ('地理样例位置','作者KML样例与Úbeda 60/100/150塔并非同一地理区域；沿用文件标识，按坐标区分'),
 ('验证范围','源文件哈希、坐标、实体关联、计数与参数出处；无新增算法实验'),
 ('适用状态','地理与仿真参考输入；未进行本地实地作业认证'),
], [35,135])
section('表1　场景总表')
table(['场景','塔数','拍摄任务','停车点','地理来源','配置状态'],[
 [name,len(s['towers']),len(s['inspection_tasks']) if s['inspection_tasks'] else '未配置',len(s['stops']) if s['stops'] else '未配置','作者KML [CAB25]' if i<2 else 'OSM','历史模型' if i<2 else '作业参考' if i<5 else '联合参考输入'] for i,(name,s) in enumerate(zip(SHORT,scenes))], [37,14,23,20,35,41])
page();section('表2　地理范围与坐标系')
table(['场景','经度范围（°E）','纬度范围（°N）','投影'],[[name,f"{s['tower_bounds_lon_lat'][0]:8f}\n{s['tower_bounds_lon_lat'][2]:8f}",f"{s['tower_bounds_lon_lat'][1]:8f}\n{s['tower_bounds_lon_lat'][3]:8f}",s['coordinate_reference']['projected']] for name,s in zip(SHORT,scenes)],[40,45,45,40])
section('表3　源数据选取与依赖关系')
table(['对象','规则 / 数值','来源'],[
 ('作者KML样例','29条坐标；5 m内合并为27个塔；6塔子集：T01、T02、T03、T11、T12、T23','AUTHOR_TOWERS；MODEL_GEO'),
 ('Úbeda选塔','3条132 kV线路；223个候选塔；按距变电站中心的UTM距离取前60/100/150塔；同距按OSM ID字符串排序','OSM_UBEDA；MODEL_SELECTION'),
 ('Úbeda基地','OSM way 415865753；闭合多边形去末个重复点后，投影顶点均值','OSM_UBEDA'),
 ('Rommerskirchen查询框','S=50.975，W=6.635，N=51.065，E=6.760','OSM_rommerskirchen_power'),
 ('Brauweiler查询框','S=50.935，W=6.730，N=51.005，E=6.860','OSM_brauweiler_power'),
 ('Villevaudé查询框','S=48.875，W=2.610，N=48.965，E=2.760','OSM_villevaude_power'),
 ('区域线路筛选','power=line；排除line=bay/busbar；保留双端点均在查询框内的相邻线段；端点中去重选power=tower','MODEL_EXPORT'),
 ('区域间重叠','德国两区域共用14个OSM杆塔ID；60⊂100⊂150；6塔⊂27塔','实体ID集合核对'),
 ('计数单位','8个文件、1108条场景内杆塔记录；重复地理实体不作为独立样本累计','catalog.json'),
],[35,99,36])
page();section('表4　场景搭建流程')
table(['步骤','作者样例 / Úbeda规模场景','德国 / 法国区域场景'],[
 ('1 源数据','保留作者仓库提交号、KML、OSM道路、电力线路与SRTM响应；验证SHA256','保留Overpass原查询、时间、URL、电力与道路响应；验证SHA256'),
 ('2 选取对象','作者样例按5 m合并；Úbeda按变电站距离取前N个塔；保留原线路相邻关系','固定查询框；保留双端点在框内的电力相邻线段；从端点选塔'),
 ('3 坐标变换','EPSG:4326 → EPSG:32630；always_xy=True','德国EPSG:32632；法国EPSG:32631；always_xy=True'),
 ('4 道路处理','保留原道路类别、方向、权限与转向过滤；Úbeda保留最大强连通分量；边时间受10 km/h上限约束','权限、方向、障碍及转向过滤；保留最大转向状态强连通分量；10 km/h参考上限'),
 ('5 地面高程','SRTM90m样点；投影后三角线性插值；塔与拍摄点、停车点均检查覆盖','Copernicus GLO90；250 m网格并加塔位补点；投影三角线性插值，禁止外推'),
 ('6 基地与停车点','原基地或变电站中心吸附到实际道路节点；塔邻近点和600 m内路口候选；间距120/350 m','最大变电站质心吸附至真实道路节点；近塔停车候选间距120 m；许可待核验'),
 ('7 拍摄任务','[LI23]每塔9项；原型采用36/30/24/18 m通用塔；规模场景采用[KPLC25]迁移塔型','每塔9项；线路主轴旋转KPLC参考塔型；逐点坐标、朝向、目标面与法向'),
 ('8 时间与资源','分列原型历史设定、规模场景文献迁移参数、归一化能耗与补电敏感性档','2车4机8电池对、8小时；逐对Wh、部分充电、历史天气窗口及功率参考模型'),
 ('9 输出','单场景JSON内嵌参数出处、单位、来源目录、完整坐标和实体关系','单JSON内嵌地图、拍摄任务、天气、完整参考参数、19类变量与16类约束绑定'),
],[25,74,71])
section('表5　地理结构统计')
table(['场景','电力节点','电力线段','道路节点','道路边 / 段','地形样点'],[[name,s['statistics']['power_nodes'],s['statistics']['power_segments'],s['statistics']['road_nodes'],str(s['statistics']['road_edges'])+'有向边' if s['statistics']['road_edges'] else str(s['statistics']['road_segments'])+'几何段',s['statistics']['terrain_samples'] or '未取得'] for name,s in zip(SHORT,scenes)],[37,24,24,28,33,24],True)
page();section('表6　九项拍摄任务的来源与几何')
table(['项目','配置','来源 / 性质'],[
 ('拍摄点编号','1、2、3、4、5、8、9、10、11，共9项','[LI23] §3.2.1、Fig.3；PDF第5-6页'),
 ('非拍摄点','6、7为跨塔过渡；overview_detour为项目增加的转弯点','[LI23]；MODEL_TIMING'),
 ('点1','沿线路负方向15 m；对准塔中心；地线层高','相机位置为参考几何派生值'),
 ('点2-5 / 8-11','两侧分别为地线、上相、中相、下相层','[LI23]任务模式；[KPLC25]高度迁移'),
 ('参考塔型','132 kV标准高度悬垂塔；不作为任何区域杆塔的实测塔型','[KPLC25] Appendix 1A-1，PDF第73页'),
 ('完成判据','同一杆塔的全部9项服务完成','MODEL_OPERATION；不等同于缺陷识别合格'),
 ('初始模型差别','6/27塔保持历史通用塔参数；不套用后续KPLC几何','MODEL_GEO'),
],[35,81,54])
table(['点号','沿线坐标 / m','横担方向 / m','离塔基高度 / m','类型'],[
 ('1','-15','0','34.2138','拍摄'),('overview_detour','-15','15','34.2138','转弯'),
 ('2','0','15','34.2138','拍摄'),('3','0','15','31.0134','拍摄'),('4','0','15','25.9080','拍摄'),('5','0','15','20.8026','拍摄'),
 ('6','0','15','42.2138','过渡'),('7','0','-15','42.2138','过渡'),('8','0','-15','34.2138','拍摄'),('9','0','-15','31.0134','拍摄'),('10','0','-15','25.9080','拍摄'),('11','0','-15','20.8026','拍摄'),
], [35,34,34,39,28])
table(['局部坐标','定义'],[('第一坐标轴','Úbeda按邻塔顺序；三个区域按入射原始电力线段主轴，符号向东/北'),('第二坐标轴','第一坐标轴逆时针旋转90°；正负两侧对应参考路线'),('适用场景','Úbeda 60/100/150及三个区域；世界坐标与经纬度逐点记录在inspection_tasks、tower_routes中')],[35,135])
page();section('表7　Úbeda 60 / 100 / 150塔完整参数')
kind={'source':'原文参数','transfer':'文献迁移','design':'建模设定','model':'模型','numerical':'数值设定','calibrated':'校准分配','derived':'派生值','inferred':'推定','catalogue_design':'目录设定','workflow_transfer':'模式迁移','reference_condition':'参考条件','reference':'参考配置','model_boundary':'核算边界'}
table(['参数','数值','单位','性质','来源'],[[r['label'],r['value'],r['unit'],kind.get(r['evidence_kind'],r['evidence_kind']),', '.join(r['source_ids'])] for r in scenes[2]['parameters']],[42,47,22,24,35],True)
page();section('表7A　三地区联合参考配置')
table(['模块','参数与数值','数据性质'],[
 ('任务','Rommerskirchen 237×9=2133；Brauweiler 185×9=1665；Villevaudé 343×9=3087','逐塔生成；公开模式迁移'),
 ('目标面','0.5×0.5 m；中心与4角；法向指向名义相机；允许视点球半径2 m','项目参考几何'),
 ('障碍','塔柱半径2 m、高35.7378 m；4层导线、参考弧垂5 m、半径0.05 m；植被未建模','工程模板；非现场测绘'),
 ('机队','2车、4机、8对电池；每车1充电通道、1操作位','参考资源设定'),
 ('飞行','质量7 kg；机体半径0.6 m；地速5、空速12、垂速1 m/s；加速度3 m/s²；jerk 5 m/s³；倾角25°','参考包络；待平台标定'),
 ('相机','5184×3888；焦距6.83–119.94 mm；理想针孔；零畸变及安装偏移；云台俯仰−120–30°、偏航±320°','H20公开边界＋内参假设'),
 ('成像要求','距离8–25 m；入射角≤30°；GSD≤0.005 m/px；拖影≤1 px；曝光0.001 s；1帧；稳定1 s','项目阈值，非厂家巡检标准'),
 ('车辆','Ackermann轴距2.5 m；4×1.8×1.8 m；转向35°；加速度1 m/s²；道路走廊6 m；禁止倒车','参考底盘；道路宽度未实测'),
 ('电池','548 Wh/对；初始100%；下限与终态20%；冷却0 s；温区0–40℃；不优化老化','TB60名义容量＋设定'),
 ('充电','20–90%参考1800 s；90–100%尾段1200 s；效率0.9；每车电源1000 W；禁止行车充电','M300公开时长＋参考曲线'),
 ('天气','2025年1月/7月各7日；30/60/100 m层；风/阵风≤10 m/s、0–40℃、前1小时降雨0、日间；密度缺失时禁飞','ERA5历史回放＋项目窗口'),
 ('能耗','参考悬停功率548×3600/2580 W；加空气阻力与正爬升功率；仅机载Wh','显式代理；不是M300高精度预测'),
 ('安全与会合','障碍净距3 m、跟踪/地图误差各0.5 m；同约束返航轨迹＋20%预留；准备120 s、回收240 s','参考限值；许可/电气净距待核验'),
 ('数值','七次Bernstein；每转移最多3段；9点求值；0.01 m/0.01 Wh/0.001 s/0.001 rad容差','离散配置；连续可行性待解后复核'),
 ('决策域','任务序列、航迹、位姿、车辆路线、会合、充电时间/功率；航次槽上界=任务数；无6塔目录上限','19类变量；16类约束'),
 ('状态','3个场景参考输入已绑定；全量轨迹因子图及联合求解器尚未实现；目标值未知','不沿用旧算法成绩作为新结果'),
],[27,104,39],True)
page();section('表8　补电参考档与公式')
table(['档位','SOC节点','累计充电时间 / s','冷却 / s','队列规则'],[[r['id'],r['charge_soc_knots'],[round(x,6) for x in r['charge_time_knots_seconds']],r['cooling_after_recovery_seconds'],r['charging_queue_policy']] for r in scenes[2]['charging_profiles']['sensitivity_profiles']],[31,31,55,20,33],True)
table(['项目','公式 / 状态','依据'],[
 ('当前启用档','Úbeda保留null；三个区域选tail20_cool0参考档，部分充电与顺序由模型决策','MODEL_CHARGE'),
 ('残量充电量','T(1)-T(1-E)，T为上表节点的分段线性插值','MODEL_CHARGE'),
 ('公开参数','20%-90%充电参考时间1800 s；全充参考3600 s，单独保留','[M300] PDF第30页；220-240 V参考条件'),
 ('敏感性部分','90%-100%尾段10/20/30 min；冷却0 s；归一化能耗近似SOC','建模假设；未实测标定'),
 ('运动耗时','d≤v²/a时：2√(d/a)；否则：d/v+v/a','匀加减速模型；各拐点停止'),
 ('飞越高度','max(起点高程,终点高程,沿段地形最大值+43.7378 m)','MODEL_OPERATION；采样步长50 m'),
 ('归一化能耗','倍率×[(局部作业+垂直运动秒数)/2580+水平转场秒数/2700]','MODEL_OPERATION；未标定能耗代理'),
 ('单塔净作业','路线移动119.9002667 s；9项非平移服务各20.01108148 s；合计300 s','[LI23]总量迁移；MODEL_TIMING分配'),
 ('算法接入','候选飞行目录、全停靠点最短路未物化；需按有向边及转向限制生成','JSON提供输入与公式；不附算法结果'),
],[33,88,49])
page();section('表9　作者6 / 27塔历史参数')
table(['参数键','数值','单位','来源'],[[r['key'],r['value'],r['unit'],', '.join(r['source_ids'])] for r in scenes[0]['parameters']],[58,58,24,30],True)
section('表10　未知项与统计口径')
table(['对象','状态 / 数值','出处'],[
 ('实际塔高与塔型','保留原始标签；缺失时actual_height_m、actual_tower_type为null；模板高度单独记录','原始OSM / KML'),
 ('高程基准','保留SRTM/Copernicus原生高程；未做现场高程基准联测','SRTM原响应'),
 ('德国 / 法国作业参数','参考输入已配置；实地标定与完整联合求解未完成','joint_reference_profile / joint_model'),
 ('地理交点','UTM平面内严格线段交点；按1 mm坐标去重；排除已绘制变电站多边形内部','MODEL_EXPORT'),
 ('站外交点：Rommerskirchen','本次23；历史汇总24；历史派生文件/脚本缺失，差异保留','原始OSM重新计算'),
 ('站外交点：Brauweiler / Villevaudé','本次2 / 46；与历史汇总一致','原始OSM重新计算'),
 ('交点含义','几何相交；不表示电气连通、已确认飞行冲突或算法难度','数据字段定义'),
 ('物理验证缺口','停车许可、电气安全净距、杆塔实测几何、图像验收、飞行能耗与补电曲线','待实地数据'),
],[46,88,36])
page();section('表11　数据字段与来源定位')
table(['JSON字段','内容 / 单位','来源'],[
 ('towers','源ID、经纬度、UTM米制坐标、地面高程、原标签、实测参数缺失状态','KML / OSM；各场景DEM插值'),
 ('power_nodes / power_ways / power_segments','电力节点、线路及原始邻接；米制段长','原始地图及源邻接筛选'),
 ('roads','道路节点经纬度；有向边或原始几何段；方向、权限、路面、转向限制','OSM；原过滤器'),
 ('stops / bases','经纬度、xy_m、ground_m；真实道路节点ID；吸附距离','源坐标派生候选点'),
 ('terrain.samples','location.lat / lng；elevation（m）；全部原采样响应','SRTM / Copernicus GLO90；来源分场景登记'),
 ('inspection_tasks','拍摄坐标与目标坐标；高度、朝向、服务时间；与杆塔一一关联','公开模式+显式模板派生'),
 ('tower_routes / transfer_waypoints','有序局部路径、过渡点经纬度和高程、段耗时','参考路线与运动模型'),
 ('parameters','value、unit、evidence_kind、source_ids、status、formula','逐参数来源'),
 ('reference_profiles','applied=true；三个区域逐塔应用模板；历史参考档保留','Úbeda参考配置'),
 ('charging_profiles','残量分段曲线、参考固定全充时间、档位选择状态','M300公开参数+敏感性设定'),
 ('source_catalog','URL、定位页码、取数时间、SHA256、许可、包内源路径、原查询','保存的来源记录'),
 ('field_provenance / construction','字段映射、步骤、输入输出与筛选规则','构建代码'),
],[48,86,36])
section('表12　独立文件与适用状态')
table(['文件','用途'],[[c['file'],'历史地理原型' if i<2 else '规模地理+作业参数参考' if i<5 else '真实地图＋完整声明参考输入；未联合求解/实地标定'] for i,c in enumerate(cat['scenes'])],[100,70])

for k,(name,s) in enumerate(zip(SHORT,scenes),13):
    page();section(f'表{k}　{name}：全部杆塔坐标')
    if s['scene_id'] in {'rommerskirchen','brauweiler','villevaude'}:
        table(['OSM节点ID\n塔ID=OSM_节点ID','经度 / °E','纬度 / °N','E / m','N / m','地面高程 / m'],[[t['source_object_id'],f"{t['longitude']:.8f}",f"{t['latitude']:.8f}",f"{t['easting_m']:.3f}",f"{t['northing_m']:.3f}",f"{t['ground_elevation_m']:.3f}"] for t in s['towers']],[31,28,28,29,30,24],True)
    else:
        table(['塔ID','原始ID','经度 / °E','纬度 / °N','E / m','N / m','地面高程 / m'],[[t['id'],t['source_object_id'],f"{t['longitude']:.8f}",f"{t['latitude']:.8f}",f"{t['easting_m']:.3f}",f"{t['northing_m']:.3f}",'未取得' if t.get('ground_elevation_m') is None else f"{t['ground_elevation_m']:.3f}"] for t in s['towers']],[19,25,26,25,26,28,21],True)
for k,(name,s) in enumerate(zip(SHORT,scenes),21):
    page();section(f'表{k}　{name}：基地与候选停车点')
    table(['基地','经度 / °E','纬度 / °N','对应停车点','吸附距离 / m'],[[b['id'],f"{b['lon']:.8f}",f"{b['lat']:.8f}",b['road_stop'],f"{b['snap_distance_m']:.3f}"] for b in s['bases']],[25,40,40,30,35])
    table(['停车点','OSM节点ID','经度 / °E','纬度 / °N','E / m','N / m','高程 / m'],[[t['id'],t['osm_node_id'],f"{t['lon']:.8f}",f"{t['lat']:.8f}",f"{t['xy_m'][0]:.3f}",f"{t['xy_m'][1]:.3f}",f"{t['ground_m']:.3f}"] for t in s['stops']],[19,25,26,25,26,28,21],True)
page();section('表29　引用论文与技术资料')
refids=['CAB25','LI23','WA20','BY26','RISS26','KPLC25','M300','H20','H20T']
refs={s['id']:s for s in cat['source_catalog']}
for rid in refids:
    r=refs[rid]
    table(['编号','文献 / 原始资料'],[(rid,r['title']),('链接',r['url']),('定位',r['locator'])],[25,145])
section('表30　地理数据与许可')
table(['数据','来源 / 许可','记录位置'],[
 ('作者KML','grvc-robotics-lab/multiUAV_planner；MIT；提交8af42a4c7f6be491f8195ba1327f01fc25ce5b03','AUTHOR_TOWERS、AUTHOR_BASES；原LICENSE随包'),
 ('地图数据','© OpenStreetMap contributors；ODbL 1.0；https://www.openstreetmap.org/copyright','OSM_*；原始OSM XML/JSON、Overpass查询与时间随包'),
 ('地形','原5场景OpenTopoData SRTM90m；新增3场景Copernicus GLO90 2021，经Open-Meteo或公开COG；https://doi.org/10.5270/ESA-c5d3d65','TERRAIN_*；原请求与响应随包；DEM为高程参考，不是实测塔基'),
 ('文献文件','论文和厂商资料以URL、页码和已核对文件哈希引用','不把工程模板或公开参数标成实地测量'),
 ('构建记录','项目原构建脚本与本次导出脚本','implementation/；用于追溯，不是独立算法执行器'),
],[31,91,48])

class NumberedCanvas(canvas.Canvas):
    def __init__(self,*a,**kw):super().__init__(*a,**kw);self.states=[]
    def showPage(self):self.states.append(dict(self.__dict__));self._startPage()
    def save(self):
        total=len(self.states)
        for state in self.states:
            self.__dict__.update(state);self.setFont('Song',8);self.setFillColor(colors.HexColor('#444444'))
            self.drawString(20*mm,13*mm,'真实地理巡检场景 · 数据与来源')
            self.drawRightString(190*mm,13*mm,f'{self._pageNumber} / {total}')
            canvas.Canvas.showPage(self)
        canvas.Canvas.save(self)
pdf=OUT/'真实地理巡检场景_数据与来源报告.pdf'
SimpleDocTemplate(str(pdf),pagesize=A4,rightMargin=20*mm,leftMargin=20*mm,topMargin=18*mm,bottomMargin=22*mm,title='真实地理巡检场景搭建与数据清单',author='paper11',allowSplitting=1).build(story,canvasmaker=NumberedCanvas)
print(pdf)
