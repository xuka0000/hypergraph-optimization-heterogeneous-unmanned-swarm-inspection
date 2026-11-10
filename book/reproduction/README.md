# 电子源码与既有实验数据

这份材料保存书中分析所依据的源码、输入、已存解与失败记录。字节核验、原输入上的可行性复算、重新运行优化和现场验证是不同工作；本书整理期间没有重跑原实验。

## 文件及版本

| 文件 | 内容 |
|---|---|
| `project/`、`source_manifest.json` | 冻结的项目代码、公开算法原码、适配接口及通用依赖许可证；逐文件 SHA256 |
| `historical_sources/` | 按内容哈希保存的运行版本覆盖文件，不是第二套活动项目 |
| `latest_regional_source_bindings.json` | 区域运行 `2bf320f8bbe886f8_24660` 的49个绑定文件，全部字节对应 |
| `regional_resource_source_bindings.json` | 前一次区域运行 `c9434b4984e09850_2852` 的49个绑定文件 |
| `conditional_source_bindings.json` | 条件超图运行 `613b3a9f4cdb_44248`：39项中38项恢复到原哈希，1项待恢复 |
| `historical_source_bindings.json` | 更早区域运行的已恢复覆盖文件；不是完整独立环境声明 |
| `existing_combinatorial_data.zip` | 92个组合输入、236个压力结果及相应记录 |
| `initial_existing_results.zip` | 初始实验、后续保留结果和诊断输出；不同运行口径分别保留 |
| `author_existing_results.zip` | 原作者完整流程的有效见证及5次失败记录 |
| `inspection_existing_data.zip` | 已绑定的地图、地形、天气、局部算例、初始化、能量检查与历史代码档案 |
| `archive_inventory.json` | 四个压缩包及每个成员的哈希和大小，共821个成员 |

条件超图旧版本尚缺 `Agent/Publication/engineering_unified_inspection.py` 的原始字节身份，预期 SHA256 为 `f064acd300f5b703011a5b47b918c924121dd9fdfdb15d0d87584eee64e87866`。已恢复文本的单独身份见 `unverified_historical_text.json`；其内容保存在清单所指的 `historical_sources/UNVERIFIED_TEXT_*.py`，实际哈希与预期不同。它可供研究，但不能把当前同名文件或文字相似的版本当作已恢复的原文件。两个较新的区域运行均完整绑定49项；两次构建阶段的四塔暖启动均有数值通过记录；退出另存候选的验收字段仍为null，且没有全237塔覆盖解。当前天气累计量区间对齐存在单独记录的缺口，既有数值通过不代表按修正区间重新验收。

## 先核验，再解压

以下命令在专著根目录运行，只用 Python 标准库，不启动优化器：

```powershell
python -B reproduction/package_tools.py --deep
```

它逐项核对源文件、运行覆盖文件、压缩包及包内成员。报告中的 `errors` 应为空；已知历史缺失另列，不隐藏在成功状态中。

需要独立复现目录时，可使用下列命令。目标必须尚不存在，而且不得位于本专著内部或覆盖其上级目录。示例路径可改为自己的临时目录。

```powershell
python -B reproduction/package_tools.py --extract D:\paper11_reproduction_scratch --version regional-2bf --include-data
```

脚本先检查完整性和解压空间，再创建独立树，应用所选运行的源码覆盖文件，写入 `BOOK_EXTRACTION_RECEIPT.json`。它不执行其中任何入口。四个压缩包按原相对路径展开；同名成员必须内容相同。当前包内数据约619 MiB，另有源码和运行时工作空间。`conditional-613b` 因一个原始哈希未恢复，不能作为“字节完整版本”解压。

## 数学模型与程序

组合实例统一使用 `resources` 和 `jobs`：每个资源给出标识和容量，每个任务给出标识、正权重及资源集合。可直接导出全部二元变量、目标和容量行：

```powershell
python -B reproduction/export_constraints.py --input <解压后的实例.json> --output <尚不存在的模型.lp>
```

另存的 `.mapping.json` 把 LP 行列对应回原资源和任务标识。它保留冗余、空资源行，不调用预处理器或求解器。若本机 SCIP 对中文文件路径读取失败，把待读取 LP 放在纯英文临时路径中；不用改动模型内容。目录中十二任务算例的完整模型、62个可行集合及最优选择已经在书中核对，不能把这一小例的核对替代大例的最优性证明。

巡检问题的通用模型、参数化模型与有限转录分别对应正文第4—6章。`Agent/Publication/engineering_*.py` 包含运动、成像、能量、电池、道路、区域联合模型与验收逻辑；具体运行采用哪份同名代码，以运行绑定清单为准。只展开已显式物化的关联行，不等于已经展开全部隐式选择关系和连续区间验收器。结果中的候选目标向量不等于独立验收通过。

## 重新执行的依赖边界

研究源码保留原始内容，因此其中可能仍有原电脑的绝对路径、源码构建目录、保护阈值和输出目录。迁移时先查看所选入口的 `ROOT`、输入、输出和动态导入；只在独立复现目录适配路径，并记录改动。不得直接运行原电脑工作区的启动器或当前实验入口。

当前记录使用 SCIP 10.0.2 / PySCIPOpt 6.2.1。Python 数值模块涉及 NumPy、SciPy、PyProj、Matplotlib、psutil 等；原生公开算法另需对应 C++ 编译器及随附依赖。电子清单包含静态导入闭包与发现的动态依赖，但没有宣称在一台干净电脑上完成全量环境安装和全部实验复现。原始 Windows 异常、内存保护及未验收状态均保留，不通过替换原算法掩盖失败。

`scripts/write_section.py`、`accept_section.py` 是本书编辑工具，依赖原 Codex/共享检查器环境，不是科学复现入口。读书、检查哈希、导出组合 LP 和编译已写 LaTeX 均无需调用它们。

## 引用和许可

公开算法保留原作者、来源和许可证；通用 Abseil、日志库等完整文件在电子树中，源码卷不重复排印通用库。对标书 VPP 及论文 PDF 用于本地阅读对标，不应随研究源码对外打包再分发。对外分享时按每份第三方材料的许可处理，不能把其算法、文字或图纸标作本项目原创。

场景构造与参数来源的15个保留文件位于电子项目的 `总结/03_实验数据/真实地理巡检场景共享/implementation/`，其中13个Python文件在源码卷排印；它们保留旧场景生成及参数分摊过程，具体区域求解仍以运行绑定覆盖和正文说明为准。20.0110814815秒的服务下界来自参考总时长残差等分，不能解释为实测快门时间。


完整区域数值输入的字段与连接方式见 `../evidence/regional_numeric_input_locators.json`。它逐项绑定冻结场景压缩包成员、当前2133项目标修正、道路及导线字段，不能把场景计数当成完整输入。

`numeric_inputs/regional_terrain.npz` 提供 `world_xyz`（1953×3）、`triangles`（3742×3，零基索引）和原连接索引；数组原始字节哈希已匹配当前区域运行的派生地形。`numeric_inputs/forecast_nodes.json` 提供9个时间节点的全部原字段、单位以及9×3×4连续场节点。由 `../scripts/export_regional_numeric_inputs.py` 从冻结资料确定性导出，无优化器调用。天气累计字段的小时错位仍保留在说明中，没有重验修正天气下的轨迹。
