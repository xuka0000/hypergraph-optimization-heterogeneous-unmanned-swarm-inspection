"""Build table-first weather handoff and native PDF modules from verified data."""
import json,os,pathlib,re,shutil,subprocess,sys
import fitz
from prepare_weather_factors import ROOT,OUT,TASK,NAMES,save,ref,now

def read(p):return json.loads(p.read_text('utf8'))
def main():
    result=read(OUT/'results.json');check=read(OUT/'delivery_validation.json');assert check['passed_for_current_scope']
    summaries={(x['scene_id'],x['season'],x['height_agl_m']):x for x in result['summary']}
    mapping=read(OUT/'scene_weather_mapping.json');names={x['scene_id']:x['name'] for x in mapping}
    windows=[];height=[]
    for n in NAMES:
        w=summaries[n,'winter',60];s=summaries[n,'summer',60]
        windows.append([names[n],str(w['tower_entries']),str(w['weather_cells']),str(w['all_scene_cells_base_pass_hours']),str(w['all_scene_cells_gust_pass_hours']),str(s['all_scene_cells_base_pass_hours']),str(s['all_scene_cells_gust_pass_hours'])])
        height.append([names[n]]+[str(summaries[n,season,h]['all_scene_cells_base_pass_hours']) for season in ['winter','summer'] for h in [30,60,100]])
    thresholds=[['气温','−10～40 °C','M100手册印刷页60'],['平均风速','≤10 m/s','M100手册印刷页59'],['雨雪','报告小时降水=0 mm','M100安全指南印刷页12；API精度0.1 mm'],['天气代码','0、1、2、3','本次额外排除天气风险的筛选假设'],['阵风敏感性','10 m阵风≤10 m/s','额外保守筛选；厂家未定义阵风统计口径'],['充电环境','0～40 °C','M100手册印刷页61；非电芯温度']]
    mechanisms=[['水平风','地速减风速得到相对空速','有符号ENU矢量；10/100m之间按对数高度插值'],['气温、露点、气压','改变湿空气密度','复用CMU原作者密度公式；高度修正为近似'],['降水与天气风险','限制候选航次可用时段','全部占用网格、时刻均需通过筛选'],['电池与天气耦合','同一天气状态下累计航次能耗','能耗接入方程已定义；尚未运行巡检求解器'],['局地阵风、垂直风','影响姿态、能耗及飞行安全','尚缺塔级观测或地形/建筑风场；保留null'],['电池冷衰减/发热','改变容量、充电及电压','尚无M100标定；未填经验倍率']]
    sources=[
      ('[1] Gao等（2021）','Weather constraints on global drone flyability. Scientific Reports11:12092；已核对2021更正。','https://doi.org/10.1038/s41598-021-91325-w'),
      ('[1a] Gao等更正（2021）','原城市数据提取存在错误；使用更正版本，不移用城市统计值。','https://doi.org/10.1038/s41598-021-00537-7'),
      ('[2] Choudhry等（2021）','ICRA；沿用原密度公式与已有M100模型输入定义。','https://doi.org/10.1109/ICRA48506.2021.9561658'),
      ('[3] Qin等（2026预印本）','GeoWind2Plan；原作者5kg参考机能耗函数，AppendixE；未运行其风场网络。','https://arxiv.org/abs/2609.36056v1'),
      ('[4] Habib等（2026预印本）','WESPR；本地PDF为v2，2026-08-19；局地几何/CFD候选，仅调研。','https://arxiv.org/abs/2603.09194v2'),
      ('[5] ERA5 / Open-Meteo','小时再分析、0.25°；气象数据：Open-Meteo.com / Copernicus C3S。','https://open-meteo.com/en/docs/historical-weather-api'),
      ('[6] DJI M100手册','印刷页59～61：风速、工作气温和充电温度。','https://dl.djicdn.com/downloads/m100/M100_User_Manual_EN.pdf'),
      ('[7] DJI M100安全指南','印刷页12：雨雪与恶劣天气限制。','https://dl.djicdn.com/downloads/dev/Matrice/en/M100_Disclaimer_and_Safety_Guidelines_en_160304.pdf'),
      ('[8] ECMWF','气象来向与风矢量分量定义。','https://confluence.ecmwf.int/pages/viewpage.action?pageId=242074496'),
      ('[9] Open-Meteo许可','API数据CC BY4.0；标注来源与本地变换。','https://open-meteo.com/en/licence')]
    def plain_table(header,rows):return '\n'.join([' | '.join(header),' | '.join(['---']*len(header))]+[' | '.join(row) for row in rows])
    txt=['天气因素建模报告｜2026-11-08','',plain_table(['输入','取值'],[
      ['场景','Úbeda60/100/150、Rommerskirchen、Brauweiler、Villevaudé'],['样本口径','1075条场景杆塔记录；901个去重OSM对象；5个独立天气网格'],
      ['时间','2025-01-01～01-07、2025-07-01～07-07；各168小时；UTC'],['用途','事前固定的两周开发示例；不代表全年气候或正式独立测试'],
      ['天气分辨率','ERA5 0.25°、1小时；关闭接口地形自动降尺度'],['高度','30/60/100m；相对天气网格地形；非逐塔风场观测'],['样本量','1680网格小时；5040条高度记录'],
      ['机型','M100实验参考；原场景M300参数保持'],['计算','模型与数据核验；无训练、无云费用、无新优化比较']]),'',
      plain_table(['因素','作用','实现/缺项'],mechanisms),'',plain_table(['筛选参数','数值','依据'],thresholds),'',
      '60m场景全域筛选（每列分母168小时）',plain_table(['场景','塔数','网格数','冬季基础h','冬季加阵风h','夏季基础h','夏季加阵风h'],windows),'',
      '高度敏感性：基础筛选小时',plain_table(['场景','冬30m','冬60m','冬100m','夏30m','夏60m','夏100m'],height),'',
      '表注：场景全域要求各关联天气网格同时通过；单次任务只检查其实际占用网格和时段。共享网格导致重复结果。小时筛选不构成实际飞行许可；零降水可能遗漏微量降水。',
      '', '风—能耗数值校验：原作者5kg参考机；1km直线；地速6m/s；166.67s；非M100误差测试',
      plain_table(['风况','空速m/s','能量Wh'],[[x['case'],f"{x['airspeed_ms']:.4f}",f"{x['predicted_Wh']:.4f}"] for x in result['wind_energy_known_cases']]),'',
      plain_table(['核验项','结果'],[['原作者CMU训练+验证密度范围','1.1643185～1.2544031 kg/m³'],['天气表面密度超出该范围','757/1680网格小时（45.06%）'],['高度密度近似不满足未饱和假设','16/5040行，保留null'],['已知算例','17项通过；含4项原作者能耗函数独立计算'],['独立逐行复核','5040行、36组汇总、6场景源文件与映射通过'],['已有3.51%能耗APE','仅原测试航次；不能用于这些天气条件的精度声明']]),'',
      '接入方程：w=(-U sinθ,-U cosθ,w_z)，v_air=v_ground-w；ρ采用CMU原函数；E_k(s)=∫P(v_air,姿态,载荷,ρ)dt；x_k≤a_k(s)；B_next(s)=B_now(s)-E_k(s)+Q_k(s)。',
      '状态定义：k为候选航次，s为共享天气状态；a为全部实际占用网格/时刻的筛选结果，B为电池剩余能量，Q为充入能量。所有能量须用同一单位；若P为W、t为s，转换Wh须除3600。',
      '缺项：s的概率、预报发布时间与误差、w_z、塔级阵风、天气下姿态响应、电芯热状态与容量曲线。天气窗口筛选本身不能证明超图优势。',
      '', '来源']
    txt.extend(' | '.join(x) for x in sources)
    txt+=['','文件：scenes/*.json（逐场景经纬度、网格、阈值、窗口）；weather_records.json（共用逐小时记录）；model.json（方程与边界）；source_manifest.json（URL/SHA256）；delivery_validation.json（独立核验）。',
      '共享使用：保留原“真实地理巡检场景共享”和本“天气因素建模”两个目录的相对位置；JSON path均相对项目根目录。只复制单个overlay JSON不会包含原场景完整几何与共用天气记录。',
      '数据变换：最近网格映射、风分量高度插值、密度推算、M100筛选；原API响应及原场景数据保持。数据许可：https://open-meteo.com/en/licence；原OSM及其他场景数据许可沿用原场景source_catalog/licenses。']
    (OUT/'天气因素建模报告.txt').write_text('\n'.join(txt)+'\n',encoding='utf8')
    def esc(s):return s.replace('&',r'\&').replace('%',r'\%').replace('_',r'\_')
    def tab(headers,rows,spec):
        return '\n'.join([r'\begin{center}\small',r'\begin{tabular}{'+spec+'}',r'\hline',' & '.join(headers)+r'\\\hline']+[' & '.join(map(esc,row))+r'\\' for row in rows]+[r'\hline\end{tabular}\end{center}'])
    body=r'''\clearpage
\section{天气因素：数据与物理接口}
\noindent 数据：ERA5小时再分析，0.25$^\circ$网格；2025年1月1--7日、7月1--7日，UTC。
两周在下载前固定，各168小时，作为开发示例。六场景共1075条杆塔记录、901个去重OSM对象，
对应5个天气网格、1680网格小时及5040条高度记录。关闭接口地形自动降尺度。
'''
    body+=tab(['因素','进入模型的位置'],[[x[0],x[1]] for x in mechanisms],r'p{.28\linewidth}p{.61\linewidth}')
    body+=r'''\noindent ENU坐标下，$\theta$为从正北顺时针计的气象来向，$U$为风速：
\[
 (u,v)=(-U\sin\theta,-U\cos\theta),\qquad
 \boldsymbol v_{\rm air}=\boldsymbol v_{\rm ground}-\boldsymbol w.
\]
\noindent 10--100米间按$\alpha=\log(h/10)/\log 10$对水平风分量插值；
这是发展阶段近似。垂直风不默认填零。表面密度直接沿用CMU原函数：
\[
 e=611\,10^{7.5T_d/(T_d+237.3)},\qquad
 \rho={0.0034848\over T+273.15}(100p-0.3796e).
\]
\noindent $T,T_d$分别为气温和露点（摄氏度），$p$为hPa，$e$为Pa，$\rho$为kg/m$^3$。
高度密度采用静力、固定虚温与0.0065 K/m温度递减近似；出现不满足未饱和假设的情况返回空值。
高度均相对天气网格地形，未转换为逐塔真实地形风场。
'''
    body+=tab(['M100参考筛选','取值'],[[x[0],x[1].replace('≤',r'$\leq$').replace('−','-')] for x in thresholds],r'p{.35\linewidth}p{.53\linewidth}')
    body+=r'''\noindent 温度、风速和充电范围来自DJI手册，雨雪限制来自安全指南[6,7]。
天气代码筛选与10米阵风筛选为本次显式适配；厂家未定义阵风统计口径。
充电气温不等于电芯温度。
\clearpage
\section{六场景天气筛选结果}
\noindent 表中计数为60米背景风条件下通过筛选的小时数；每列分母168。
基础项包含气温、平均风速、报告降水及天气代码。
'''
    body+=tab(['场景','塔数','网格','冬基础','冬阵风','夏基础','夏阵风'],windows,'lrrrrrr')
    body+=r'''\noindent 场景全域统计要求所有关联网格同时通过；单次任务只检查实际经过的网格和占用时段。
德国两个场景共享一个天气网格，三个Úbeda场景共享相同两个网格；相同结果保持原样。
下表为高度敏感性，均为基础筛选小时数。
'''
    body+=tab(['场景','冬30m','冬60m','冬100m','夏30m','夏60m','夏100m'],height,'lrrrrrr')
    body+=r'''\noindent 这两周不能估计全年可飞率。小时平均背景与前一小时降水不构成整段航次的安全保证。
接口降水精度为0.1 mm，报告为零不能排除微量降水；天气代码未充分覆盖雷电、结冰及能见度。
此处窗口未加入昼夜要求，日照字段另行保留。
\begin{center}\small
\begin{tabular}{rrr}\hline
网格纬度（度） & 网格经度（度） & 网格地形高程（m）\\\hline
38.00 & -3.50 & 514\\
38.00 & -3.25 & 682\\
49.00 & 2.50 & 79\\
49.00 & 2.75 & 96\\
51.00 & 6.75 & 67\\\hline
\end{tabular}\end{center}
\noindent 天气数据来源：Open-Meteo.com / Copernicus C3S [5,9]。
逐塔经纬度及网格映射保存在六个场景JSON；网格高程不能代替塔底实测高程。
\clearpage
\section{能耗校验、接入约束与精度边界}
\noindent 下表使用GeoWind2Plan原作者5 kg参考机函数[3]：1 km直线、地速6 m/s、166.67秒。
这是确定性数值校验，不是M100实飞误差测试。
'''
    cn={'calm':'无风','tailwind4':'顺风4 m/s','headwind4':'逆风4 m/s','crosswind4':'侧风4 m/s'}
    body+=tab(['风况','相对空速（m/s）','能量（Wh）'],[[cn[x['case']],f"{x['airspeed_ms']:.4f}",f"{x['predicted_Wh']:.4f}"] for x in result['wind_energy_known_cases']],'lrr')
    body+=tab(['核验对象','实际结果'],[
      ['CMU训练+验证密度','1.1643185～1.2544031 kg/m³'],['天气表面密度超范围','757/1680网格小时（45.06%）'],['高度密度空值','16/5040行'],['已知算例','17项通过（含4项能耗独立计算）'],['保存结果独立复核','5040行、36组汇总及6场景映射通过']],r'p{.40\linewidth}p{.47\linewidth}')
    body+=r'''\noindent 超范围标记说明原精度证据不覆盖这些输入，不能据此判断误差大小。
之前b-TCN的3.51\%能耗APE仅对应原测试航次；本轮没有进行新天气下的TCN推理或训练。
CMU原数据的\texttt{wind\_speed}按作者代码作为实测空速使用，不重复扣除环境风。
Liu原实现虽读取密度，当前有效功率计算未使用密度项；未改写作者模型。
\[
 E_k(s)={1\over3600}\int P(\boldsymbol v_{\rm air},\mathrm{attitude},\mathrm{load},\rho)\,dt,
 \quad x_k\le a_k(s),\quad B_{\rm next}(s)=B_{\rm now}(s)-E_k(s)+Q_k(s).
\]
\noindent $k$为候选航次，$s$为共享天气状态；$P$为W、$t$为秒，$E,B,Q$统一为Wh。
$a_k$为实际占用网格和时段的筛选结果；$Q_k$为充入能量。
这些是待接入巡检求解器的接口方程，尚未验证电池容量/温度曲线、天气下姿态响应和返航决策。
天气概率及机会约束风险水平未设定。
\par\noindent 固定天气窗口本身只会筛掉部分候选航次。是否产生高阶决策耦合，仍需检验共享电池、
充电通道与车机会合是否共同改变选择；本轮没有新的超图与SCIP比较。
\clearpage
\section{数据文件与引用来源}
\begin{center}\small
\begin{tabular}{p{.34\linewidth}p{.53\linewidth}}\hline
文件 & 内容\\\hline
\texttt{scenes/*.json} & 六场景经纬度、原始对象ID、天气网格、M100阈值和窗口。\\
\texttt{weather\_records.json} & 共用逐小时、逐高度天气记录；缺项保留为空。\\
\texttt{model.json} & 方程、来源、能耗接入检查及未标定项目。\\
\texttt{source\_manifest.json} & 原始下载URL、版本、SHA256及失败尝试。\\
\texttt{delivery\_validation.json} & 独立逐行核验与样本口径。\\\hline
\end{tabular}\end{center}
\noindent JSON路径相对项目根目录。共享时同时保留“真实地理巡检场景共享”和“天气因素建模”目录；
只复制单个天气JSON不包含完整原场景或共用天气记录。
\par\smallskip
'''
    for label,desc,url in sources:
        body+=r'\noindent '+esc(label)+' '+esc(desc)+r' \href{'+url+r'}{来源链接}。\par\smallskip'+'\n'
    body+=r'''\noindent 数据变换：最近网格映射、风分量高度插值、密度推算与筛选；API数据按CC BY 4.0署名[9]。
原OSM及场景地理数据许可沿用原场景记录。GeoWind2Plan规划代码与已有预测风场公开，
训练风场网络需向作者申请；本轮未发送申请、下载CFD数据或运行风场网络。
'''
    modules=ROOT/'.codex/paper_modules';shared=modules/'weather_factor_modeling.tex';shared.write_text(body,encoding='utf8')
    for name in ['modeling','algorithm','setup','results']:
        p=modules/name/'main.tex';s=p.read_text('utf-8-sig');inc=r'\input{../weather_factor_modeling.tex}'
        if inc not in s:s=s.replace(r'\end{document}',inc+'\n'+r'\end{document}')
        s=re.sub(r'\\newcommand\{\\ReportDate\}\{[^}]+\}',lambda m:r'\newcommand{\ReportDate}{2026年11月8日}',s)
        p.write_text(s,encoding='utf8')
    standalone=OUT/'天气因素建模报告.tex'
    standalone.write_text(r'''\documentclass[UTF8,11pt,fontset=windows]{ctexart}
\usepackage[a4paper,margin=20mm]{geometry}
\usepackage{amsmath,amssymb,array,hyperref}
\hypersetup{hidelinks}
\setlength{\parindent}{0pt}
\setlength{\tabcolsep}{5pt}
\begin{document}
'''+body.removeprefix('\\clearpage\n')+'\n'+r'\end{document}',encoding='utf8')
    records=[];builder=pathlib.Path('D:/博士研究生/具身智能大论文架构/cross_work_bridge/tools/codex_control_center/build_paper_modules.py')
    p=subprocess.run([sys.executable,'-X','utf8',str(builder),'--workspace','paper11'],capture_output=True,text=True,encoding='utf8',errors='replace',timeout=30)
    records.append({'shared_builder_exit':p.returncode,'stderr':p.stderr[-1000:],'fallback':'Native installed XeLaTeX; existing sources/styles preserved'})
    exe=shutil.which('xelatex');assert exe
    pngdir=OUT/'module_pdf_review';pngdir.mkdir(exist_ok=True)
    jobs=[('report',OUT,standalone.name,OUT/'build')]+[(n,modules/n,'main.tex',modules/'build'/n) for n in ['modeling','algorithm','setup','results']]
    for name,work,filename,build in jobs:
        build.mkdir(exist_ok=True,parents=True);env=os.environ.copy();env['TEXINPUTS']=str(work)+';'+str(modules)+';'+env.get('TEXINPUTS','')
        for i in range(2):
            cp=subprocess.run([exe,'-interaction=nonstopmode','-halt-on-error',f'-output-directory={build}',filename],cwd=work,env=env,capture_output=True,text=True,encoding='utf8',errors='replace',timeout=55)
            (build/f'weather_compile_pass{i+1}.txt').write_text(cp.stdout+cp.stderr,encoding='utf8')
            assert cp.returncode==0,(name,cp.stdout[-2500:])
        pdf=build/(pathlib.Path(filename).stem+'.pdf');doc=fitz.open(pdf);texts=[p.get_text() for p in doc]
        assert '757/1680' in ''.join(texts) and '22.1463' in ''.join(texts)
        # Render every new appendix page, excluding prior unchanged content.
        first=next(i for i,t in enumerate(texts) if '数据与物理接口' in t and '1680' in t)
        for i in range(first,len(doc)):doc[i].get_pixmap(matrix=fitz.Matrix(1.3,1.3)).save(pngdir/f'{name}_{i+1}.png')
        count=len(doc);doc.close();target=work/(pathlib.Path(filename).stem+'.pdf');shutil.copyfile(pdf,target)
        records.append({'module':name,'pages':count,'pdf':ref(target),'source':ref(work/filename),'shared_content':ref(shared),'new_pages_1based':list(range(first+1,count+1)),'visual_review':'pending'})
        print(name,'compiled',count,'new pages',first+1,'to',count,flush=True)
    save(OUT/'module_pdf_sync.json',{'observed_at':now(),'status':'compiled_pending_visual_review','records':records})

if __name__=='__main__':main()
