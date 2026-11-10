# Hypergraph Optimization and Heterogeneous Unmanned Swarm Inspection

《超图优化与异构无人集群巡检：数学建模、求解方法与评价》的学习与换机材料。

**[阅读专著 PDF（221 页）](book/monograph.pdf)** · [数学模型与当前代码](project/Agent/Publication/) · [历史源码与复现说明](book/reproduction/README.md) · [当前研究状态](STUDY_STATUS.json)

| 路径 | 内容 |
|---|---|
| `book/monograph.pdf` | 当前教学专著，含异构无人集群建模、方法和评价讨论 |
| `book/main.tex`、`sections/`、`figures/`、`references.tex` | 可继续编辑的 LaTeX、图表及参考文献 |
| `project/Agent/Publication/` | 当前巡检、飞行动力学、电池、天气、成像、联合转录与超图模块 |
| `project/scripts/` | 原实验、验证与分析入口 |
| `book/reproduction/` | 冻结源码、公开原作者算法、许可证、运行版本绑定和四份历史数据包 |
| `data/geographic_scenes.zip` | 六个真实地理配置的完整 JSON、经纬度、道路/地形来源；另保留早期 6/27 塔参考实例 |
| `data/component_results.zip` | 当前模块实验结果、协议及失败/尝试记录 |
| `data/energy_weather_evidence.zip` | 能耗、天气结果，公开 M100 实测数据与许可清晰的来源文件 |
| `CURRENT_SOURCES.json`、`MANIFEST.json` | 当前源码身份及逐文件 SHA256 |

## 换电脑使用

建议把目录放在较短的本地路径，例如 `D:\inspection-study`。阅读 PDF 不需要安装任何 Python 包。

1. 下载本仓库 Release 中的 `inspection-study.zip` 并解压，或克隆仓库。
2. 安装 Python **3.11**，先校验材料（只用标准库，不启动实验）：

```powershell
python -B tools/verify_package.py --deep
```

3. 建立独立环境，安装核心模块依赖：

```powershell
python -m venv .venv
.venv\Scripts\python -m pip install -r requirements-core.txt
.venv\Scripts\python -B tools/check_environment.py
```

Linux/macOS 将 `.venv\Scripts\python` 换成 `.venv/bin/python`。旧实验使用的额外包见 `requirements-extended.txt`；原作者 C++ 程序还需要其源码中的 CMake/编译器环境。这里没有声称全部旧程序已在所有操作系统重跑。

迁移环境使用 `psutil 5.9.8` 的 Python 3.11 二进制包；原电脑记录为 `5.9.0`，该旧版在干净 Windows 环境需要本地 C++ 编译。此项仅用于资源查询，不修改数学模型或公开算法。

4. 恢复一个**新的**可运行目录（目标必须不存在），展开数据并叠加已登记的当前源码：

```powershell
python -B tools/prepare_workspace.py --destination ../inspection-work
```

原始压缩包保留历史版本；恢复目录中的新版文件及覆盖关系记入 `RESTORE_RECEIPT.json`。六个真实地理场景位于恢复目录的 `总结/03_实验数据/真实地理巡检场景共享/scenes/`。原始参数、来源及坐标均在 JSON 中。

需要学习未在仓库再分发的 Rodrigues 原作者源码时，可按保留的版本 URL 下载并核对 SHA256：

```powershell
python -B tools/fetch_external_assets.py --workspace ../inspection-work
```

该命令仅下载到自己的恢复目录，不运行程序；原作者使用条件仍适用。

5. 可先运行不依赖外部文件的共享充电模块验证：

```powershell
.venv\Scripts\python -B ../inspection-work/scripts/validate_inspection_joint_charging.py
```

结果写入恢复目录的 `总结/03_实验数据/工程化联合求解/module_validation/joint_charging/`。其他模块入口与证据见 [学习路线](LEARNING_GUIDE.md)。旧区域求解脚本包含原机器路径、资源保护和审核入口，须逐项配置后运行；不要把打包视为已经完成全区域求解。

## 重新编译正文

安装 XeLaTeX（TeX Live 或 MiKTeX），并使 `xelatex` 在 PATH 中：

```powershell
python -B tools/build_book.py
```

输出为 `book/build/main.pdf`，不覆盖已交付 PDF。当前母版使用 Windows 的宋体、黑体、NSimSun、Times New Roman、Arial、Consolas。其他系统需合法安装对应字体，或自行更改 `preamble.tex` 字体配置；改字体可能改变分页。

## 数据与许可

来源 URL、论文引用、数据/代码哈希、原作者 commit 和许可证随材料保存。详见 [第三方来源](THIRD_PARTY_NOTICES.md)。VPP 对标书、第三方论文和厂商手册 PDF 不随此包再分发；引用与原下载地址保留。未确认再分发许可的两个 Rodrigues GitHub 源码目录列入 `data/external_assets.json`，可按原 source manifest 定位，未冒充已打包。

本仓库是研究进度快照：237 塔完整区域联合求解与执行重规划闭环仍未完成，M300 同机实测标定仍有缺项；公开 M100 数据不替代 M300 实测。历史失败、未验收候选和一个未恢复的旧源码哈希均保留说明。完整材料交付与科学任务完成是两回事。

项目原创文字与代码尚未指定统一开源许可证；公开访问不改变第三方原许可证或自动授予其他许可。
