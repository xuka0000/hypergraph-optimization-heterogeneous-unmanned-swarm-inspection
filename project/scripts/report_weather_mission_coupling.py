"""Table-first report for the fixed mission/weather interface diagnostic."""
import os,pathlib,re,shutil,subprocess,sys
import fitz
from run_weather_mission_coupling import ROOT,OUT,BASE,TASK,MODELS,read,save,ref,now
LABELS={'calm_15C':'无风15°C','outbound_tail_4ms':'去程顺风4','outbound_head_4ms':'去程逆风4','cross_4ms':'侧风4','outbound_tail_8ms':'去程顺风8','outbound_head_8ms':'去程逆风8','cross_8ms':'侧风8','calm_0C':'无风0°C','calm_35C':'无风35°C','outbound_head_12ms':'去程逆风12','winter_actual_noon':'冬季再分析','summer_actual_noon':'夏季再分析'}

def main():
    v=read(OUT/'delivery_validation.json');assert v['passed_for_current_scope']
    results=read(OUT/'results.json');mission=read(OUT/'mission.json');cases=read(OUT/'cases.json');proto=read(OUT/'protocol.json');analysis=read(OUT/'trajectory_diagnosis.json')
    by={(r['case_id'],r['model']):r for r in results['rows']};diag={r['case_id']:r for r in results['case_diagnostics']}
    energy=[[LABELS[c['id']]]+[f"{by[c['id'],m]['energy_Wh']:.3f}" for m in MODELS] for c in cases]
    deltas=[[LABELS[c['id']]]+[f"{by[c['id'],m]['delta_vs_calm15_Wh']:+.3f}" for m in MODELS] for c in cases]
    conditions=[[LABELS[c['id']],f"{c['temperature_C']:.1f}",f"{c['density_kgm3']:.4f}",f"{sum(x*x for x in c['wind_nwu_ms'])**.5:.2f}"] for c in cases]
    diagnostic=[[LABELS[c['id']],f"{100*diag[c['id']]['any_ood_rows']/diag[c['id']]['predicted_rows']:.1f}",f"{diag[c['id']]['tracking_rms_m']:.3f}",f"{diag[c['id']]['tracking_max_m']:.3f}",f"{by[c['id'],'b-TCN']['reference_margin_Wh']:+.3f}",'通过' if by[c['id'],'b-TCN']['joint_reference_screen'] else '未通过'] for c in cases]
    phase_rows=[[LABELS[r['case_id']],f"{r['transfer_max_m']:.3f}",f"{r['photo_max_m']:.3f}",f"{r['photo_end_max_m']:.3f}",f"{r['energy_min_Wh']:.3f}–{r['energy_max_Wh']:.3f}"] for r in analysis['cases']]
    def plain(h,rows):return '\n'.join([' | '.join(h),' | '.join(['---']*len(h))]+[' | '.join(x) for x in rows])
    text=['天气—真实航段—能耗联算｜2026-11-08','',plain(['设置','固定值'],[
      ['场景/对象','Úbeda60场景T001；OSM4046365257；9个参考拍摄点'],['塔坐标','经度−3.3690616；纬度37.9763934'],['起止路点','P02；经度−3.3652771；纬度37.9765751；参考地面475m'],['航段','P02空中起点→T001九点精检→P02空中终点；转移标高535m'],['时长',f"{mission['duration_s']:.1f}s；塔内{mission['local_inspection_duration_s']:.1f}s（原300s，逐段0.1s取整）；20s稳定过程不计结果"],['速度','转移5m/s；塔内原参考时长；接近/离开塔1m/s'],['模型','原作者4个冻结神经网络+Liu，共60组；不训练、不调参'],['能量参照','TB47D标称99.9Wh×(1−原场景20%预留)=79.92Wh'],['评价边界','空中段；不含起飞、降落、温度衰减；无实际功率真值']]),'',
      plain(['天气','气温°C','密度kg/m³','风速m/s'],conditions),'',
      '条件预测能耗（Wh）',plain(['天气']+MODELS,energy),'',
      '相对各模型无风15°C的差值（Wh）',plain(['天气']+MODELS,deltas),'',
      '输入适用域与跟踪诊断（能量余量及联合筛选使用b-TCN；完整五模型结果见JSON）',plain(['天气','越界行%','跟踪RMS m','最大偏差m','余量Wh','天气+能量参照'],diagnostic),'',
      '样本口径：12种预先固定的条件、1条确定性任务、5模型；没有60次独立实飞或跨种子统计。去程顺/逆风是相对固定去程方向命名，同一空间风在返程作用相反。',
      '控制天气：气压1013.25hPa、RH50%；只改一个因素。真实天气示例来自固定2025-01-01和2025-07-01的12:00UTC，使用38°N/−3.25°E网格60m背景风；全航段均匀，不宣称塔级或逐高度局地风。',
      '必要适配：原动力学与控制系数保持；精确悬停时0/0采用连续零向量极限；密度显式传入原动力学。原训练接口使用真实机体yaw旋转空速分量、带符号roll作为aoa；未用地面航迹角和无符号倾角替代。',
      '原作者归一化固定为训练+验证集min/max；不使用本次任务重拟合；超范围不裁剪。原四模型权重与Liu参数保持。',
      '不确定项：轨迹跟踪偏差不等于拍摄验收；实飞姿态/阵风、机体/相机标定、地形避障、停车条件、起降能耗、电池温度和老化仍未验证。联合参照通过不构成运行可行性证明。',
      '验证：所有60组保存功率独立积分、各阶段能量、原始模型哈希、空速矢量、原动力学状态递推及同一时段核对通过。既有3.51%误差不适用于本次合成状态。',
      '', '事后航段诊断（不新增实验、不设置拍摄达标阈值）',plain(['天气','转移最大偏差m','拍摄时段最大偏差m','九段拍摄结束最大偏差m','五模型能耗范围Wh'],phase_rows),
      '本次按固定时间推进目标；原作者propagate函数则以ghost前视目标和距离条件推进路径。本次仅复用原动力学和控制器，没有复现完整propagate流程。原控制器无速度前馈，地速指令上限与参考转移速度均为5m/s，叠加加速和阻力产生转移滞后。',
      '8m/s侧风相对无风15°C：b-TCN预测−11.87%，b-LSTM预测+0.96%；方向不一致，不能据此断言侧风省电。0°C/35°C及冬夏示例密度越界，温度试验不含电池可用容量变化。',
      '后续依赖：按实际到达与拍摄验收推进任务，记录真实完成时长，再接入巡检调度；位置/相机验收条件需来源依据，不从本次结果倒推达标阈值。',
      '', '来源',
      '[1] Choudhry等，ICRA2021，CVaR-based Flight Energy Risk Assessment for Multirotor UAVs using a Deep Energy Model: https://doi.org/10.1109/ICRA48506.2021.9561658；作者源代码：https://github.com/castacks/cvar-energy-risk-deep-model；commit d613ba37bde8dcdaa5a9e395de4cc4029c24a39d。',
      '[2] Gao等，2021，Weather constraints on global drone flyability：https://doi.org/10.1038/s41598-021-91325-w。',
      '[3] 天气数据：Open-Meteo.com / Copernicus C3S；https://open-meteo.com/en/docs/historical-weather-api；数据许可CCBY4.0：https://open-meteo.com/en/licence。',
      '[4] DJI M100 User Manual，印刷页59～61：https://dl.djicdn.com/downloads/m100/M100_User_Manual_EN.pdf。',
      '[5] 杆塔、道路及参考作业参数：原真实地理场景JSON source_catalog/field_provenance；经纬度OSM，拍摄几何为文献模板转移，300s为原项目参考。',
      '', '文件：mission.json（逐点经纬度/时间/来源）；cases.json（全部天气参数）；protocol.json（模型与配置）；predictions/*（状态和功率）；results.json（全部数值）；delivery_validation.json（独立核验）。']
    (OUT/'天气_航段_能耗联算报告.txt').write_text('\n'.join(text)+'\n',encoding='utf8')
    def esc(x):return str(x).replace('&',r'\&').replace('%',r'\%').replace('_',r'\_')
    def table(headers,rows,spec):return '\n'.join([r'\begin{center}\small\begin{tabular}{'+spec+'}',r'\hline',' & '.join(headers)+r'\\\hline']+[' & '.join(map(esc,r))+r'\\' for r in rows]+[r'\hline\end{tabular}\end{center}'])
    body=r'''\clearpage
\section{天气与真实航段联算：固定条件}
'''
    body+=table(['项目','取值'],[
      ['场景/对象','Úbeda60场景T001；OSM4046365257；9个拍摄点'],['杆塔经纬度','-3.3690616°，37.9763934°'],['起止路点经纬度','P02：-3.3652771°，37.9765751°'],['任务路径','P02空中起点→T001九点精检→P02空中终点'],['转移标高/速度','535m；5m/s；塔旁接近/离开1m/s'],['任务时间',f"{mission['duration_s']:.1f}s；塔内{mission['local_inspection_duration_s']:.1f}s"],['时间口径','原塔内300s，各段0.1s取整；20s稳定过程不计'],['电量参照','标称99.9Wh；沿用20%预留；参照预算79.92Wh']],r'p{.29\linewidth}p{.60\linewidth}')
    body+=r'''\noindent 地理坐标来自原场景OSM记录，拍摄几何为已登记文献模板。
任务为规定的空中段，不含起飞、降落；未验证实际停车、避障和拍摄效果。
以下12条件在求值前固定；每条件运行全部5个原作者模型，无训练或调参。
'''
    body+=table(['天气条件','气温（°C）','密度（kg/m³）','风速（m/s）'],conditions,'lrrr')
    body+=r'''\noindent 控制条件固定气压1013.25hPa、相对湿度50\%；单独改变风或温度。
去程顺/逆风相对P02到T001方向命名，返程作用相反。12m/s为超厂家风速限制的负对照。
冬夏示例固定为2025年1月1日与7月1日12:00UTC，取38°N、-3.25°E网格的60米背景风[2,3]。
背景风、密度全程均匀，尚未做逐塔地形和高度降尺度。
\clearpage
\section{同一任务的条件预测能耗}
\noindent 单位Wh；每列使用同一原作者模型及冻结权重。没有实飞功率真值，不能由表中数值判断哪个模型更准确。
'''
    body+=table(['天气']+MODELS,energy,'lrrrrr')
    body+=r'''\noindent 相对各模型无风15°C条件的能耗差（Wh）：
'''
    body+=table(['天气']+MODELS,deltas,'lrrrrr')
    body+=r'''\noindent 所有模型均积分相同0--533.7秒空中段。12条件×5模型为60组条件预测，
不代表60次独立实飞。温度只通过气体密度及动力学/能耗输入传递，未加入电池冷衰减或热模型。
\clearpage
\section{适用域、路径跟踪与能量参照}
\noindent 越界行指至少一个模型输入超出原训练+验证范围的采样行，占本条件5338行的比例。
跟踪误差为动力学位置相对规定轨迹的位置误差；不是拍摄验收指标。
'''
    body+=table(['天气','越界\%','RMS m','最大m','余量Wh','参照筛选'],diagnostic,'lrrrrl')
    body+=r'''\noindent 余量为79.92Wh减b-TCN预测能量；参照筛选同时检查天气基础条件和该能量预算。
完整五模型余量与筛选结果保存在JSON。标称能量与20\%预留尚未按实际电芯温度、老化或电压截止标定，
并且本段不含起降，因此参照筛选通过不能作为完整航次可行性证明。
\par\noindent 必要接口适配：实际机体yaw用于水平空速旋转，带符号roll对应原实飞预处理中的
\texttt{calc\_aoa(index=0)}；原仿真导出的航迹角和无符号倾角不直接替代这些训练输入。
原控制器精确悬停时的0/0使用连续零向量极限，动力学显式传入本条件密度。
原力学公式、控制增益、权重、Liu参数及训练/验证归一化范围保持不变。
\par\noindent 本轮关闭随机扰动和Dryden阵风，只检验预设平均风机制；未声称复现真实湍流。
全部原始输出保留，越界输入未裁剪，已有3.51\%实飞回放误差不用于本轮合成状态。
\par\noindent 已独立核对：60组功率积分、各阶段能量、原作者文件哈希、空速矢量与机体系变换、
位置和速度递推以及相同评价时段。真实功率误差、完整飞行可行性与超图收益仍未知。
\clearpage
\section{事后诊断：偏差发生在哪些航段}
\noindent 单位米；拍摄结束偏差为九个拍摄时段终点偏差的最大值。
这里仅定位已保存轨迹的问题，没有新增实验或从结果倒推拍摄达标阈值。
'''
    body+=table(['天气','转移最大','拍摄最大','拍摄结束','五模型Wh范围'],phase_rows,'lrrrr')
    body+=r'''\noindent 固定时间目标与实际完成：本次每0.1秒推进目标，保证各条件使用同一规定时间线。
原作者\texttt{propagate}则使用前视目标，并仅在距离条件满足后推进路径索引。
本轮复用原动力学和控制器，没有复现整个原\texttt{propagate}流程。
\par\medskip\noindent 转移滞后的依据：原控制器无速度前馈，水平地速指令上限与规定转移速度均为5m/s。
有限加速、阻力及固定时间目标共同产生滞后。此项诊断针对仿真/任务适配，不代表实机定位性能。
\par\medskip\noindent 能耗变化的分歧：8m/s侧风下，相对无风15°C，b-TCN预测变化为-11.87\%，
b-LSTM为+0.96\%；当前证据不能支持“侧风省电”的结论。0°C、35°C和两个历史天气条件的密度
均超出原训练/验证范围；本轮温度条件没有包含电池可用容量变化。
\par\medskip\noindent 下一项模型依赖：按实际到达、稳定拍摄条件推进任务，重新得到完成时长和对应能量。
位置及相机验收需要来源依据；本轮不自行设置新的科研成功标准。
在该接口补齐及实飞误差核验前，不把固定时长能量作为完成同一巡检任务的电量。
\clearpage
\section{联算数据与来源}
'''
    body+=table(['文件','内容'],[['mission.json','逐点经纬度、标高、时间、作业及原场景来源'],['cases.json','12条件的风矢量、温度、露点、气压、密度及身份'],['protocol.json','固定模型、适配、预算与证据边界'],['predictions/*','每个条件的完整状态轨迹和五模型功率'],['results.json','60组Wh、阶段能量、差值、余量及跟踪诊断'],['delivery_validation.json','独立逐行力学及能量核验']],r'p{.33\linewidth}p{.54\linewidth}')
    body+=r'''\noindent [1] Choudhry等，CVaR-based Flight Energy Risk Assessment for Multirotor UAVs using a Deep Energy Model，ICRA 2021，pp.262--268。
\href{https://doi.org/10.1109/ICRA48506.2021.9561658}{DOI:10.1109/ICRA48506.2021.9561658}。
\href{https://github.com/castacks/cvar-energy-risk-deep-model}{原作者代码与权重}，commit
\texttt{d613ba37bde8dcdaa5a9e395de4cc4029c24a39d}。
采用原动力学、四个预训练网络与Liu函数，来源文件未改动。\par\medskip
\noindent [2] Gao等，Weather constraints on global drone flyability，Scientific Reports11:12092（2021）。
\href{https://doi.org/10.1038/s41598-021-91325-w}{DOI:10.1038/s41598-021-91325-w}，已核对更正版本。\par\medskip
\noindent [3] 天气数据：Open-Meteo.com / Copernicus C3S。
\href{https://open-meteo.com/en/docs/historical-weather-api}{历史天气接口}；
\href{https://open-meteo.com/en/licence}{CC BY 4.0许可}。
原响应及上一步天气推导记录保留，本次取固定时刻、单网格背景作为条件输入。\par\medskip
\noindent [4] DJI M100 User Manual，印刷页59--61，工作条件与TB47D标称电量。
\href{https://dl.djicdn.com/downloads/m100/M100_User_Manual_EN.pdf}{官方手册}。\par\medskip
\noindent [5] 原六场景包中的\texttt{ubeda\_osm\_060.json}，
\texttt{source\_catalog}、\texttt{field\_provenance}和\texttt{parameters}。
保留OSM经纬度、文献转移拍摄模板、参考作业时间及原预留比例，未将参考模板改称实测。\par\medskip
\noindent 运行范围：本地有界模型接口诊断；无新优化算法比较、训练或云费用。
报告表格参照原论文PDF第4页Table I的模型身份与显式单位组织，内容为本次独立计算数据。
'''
    modules=ROOT/'.codex/paper_modules';shared=modules/'weather_mission_coupling.tex';shared.write_text(body,encoding='utf8')
    for name in ['modeling','algorithm','setup','results']:
        p=modules/name/'main.tex';s=p.read_text('utf-8-sig');inc=r'\input{../weather_mission_coupling.tex}'
        if inc not in s:s=s.replace(r'\end{document}',inc+'\n'+r'\end{document}')
        p.write_text(s,encoding='utf8')
    entry=OUT/'天气_航段_能耗联算报告.tex';entry.write_text(r'''\documentclass[UTF8,11pt,fontset=windows]{ctexart}
\usepackage[a4paper,margin=20mm]{geometry}
\usepackage{amsmath,amssymb,array,hyperref}
\hypersetup{hidelinks}\setlength{\parindent}{0pt}\setlength{\tabcolsep}{5pt}
\begin{document}
'''+body.removeprefix('\\clearpage\n')+'\n'+r'\end{document}',encoding='utf8')
    records=[];builder=pathlib.Path('D:/博士研究生/具身智能大论文架构/cross_work_bridge/tools/codex_control_center/build_paper_modules.py')
    cp=subprocess.run([sys.executable,'-X','utf8',str(builder),'--workspace','paper11'],capture_output=True,text=True,encoding='utf8',errors='replace',timeout=30)
    records.append({'shared_builder_exit':cp.returncode,'stderr':cp.stderr[-1000:],'fallback':'Native installed XeLaTeX; inherited sources/styles retained'})
    exe=shutil.which('xelatex');assert exe
    png=OUT/'module_pdf_review';png.mkdir(exist_ok=True)
    jobs=[('report',OUT,entry.name,OUT/'build')]+[(n,modules/n,'main.tex',modules/'build'/n) for n in ['modeling','algorithm','setup','results']]
    for name,work,filename,build in jobs:
        build.mkdir(parents=True,exist_ok=True);env=os.environ.copy();env['TEXINPUTS']=str(work)+';'+str(modules)+';'+env.get('TEXINPUTS','')
        for i in range(2):
            cp=subprocess.run([exe,'-interaction=nonstopmode','-halt-on-error',f'-output-directory={build}',filename],cwd=work,env=env,capture_output=True,text=True,encoding='utf8',errors='replace',timeout=55)
            (build/f'weather_mission_compile_pass{i+1}.txt').write_text(cp.stdout+cp.stderr,encoding='utf8');assert cp.returncode==0,(name,cp.stdout[-2000:])
        src=build/(pathlib.Path(filename).stem+'.pdf');doc=fitz.open(src);texts=[p.get_text() for p in doc]
        first=next(i for i,t in enumerate(texts) if '联算：固定条件' in t and '533.7' in t)
        for i in range(first,len(doc)):doc[i].get_pixmap(matrix=fitz.Matrix(1.3,1.3)).save(png/f'{name}_{i+1}.png')
        pages=len(doc);doc.close();target=work/(pathlib.Path(filename).stem+'.pdf');shutil.copyfile(src,target)
        records.append({'module':name,'pages':pages,'new_pages_1based':list(range(first+1,pages+1)),'pdf':ref(target),'source':ref(work/filename),'shared_content':ref(shared),'visual_review':'pending'})
        print(name,'compiled',pages,'new pages',first+1,'to',pages,flush=True)
    save(OUT/'module_pdf_sync.json',{'observed_at':now(),'status':'compiled_pending_visual_review','records':records})

if __name__=='__main__':main()
