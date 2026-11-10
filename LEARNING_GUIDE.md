# 学习入口

| 正文部分 | 代码入口（恢复目录内） | 数据/验证 |
|---|---|---|
| 第1章与异构集群示例 | `scripts/verify_heterogeneous_teaching_examples.py` | 用仓库的 `tools/check_environment.py` 重算；A–E 是教学构造例，不是公开基准试验 |
| 第2—3章组合超图 | `scripts/run_hypergraph_scip_demo.py`，`book/reproduction/export_constraints.py` | 四个历史 ZIP、完整 LP 导出；原作者 C++ 代码及许可证随包保存 |
| 地理与道路 | `Agent/Publication/geographic_inspection_scene.py`、`engineering_road_motion.py` | 六个真实地理配置：Úbeda 60/100/150，Rommerskirchen 237，Brauweiler 185，Villevaudé 343 |
| 电池 | `engineering_battery_electrothermal.py` | `inspection_reference_battery_profile.json`，`module_validation/battery/` |
| 连续飞行 | `engineering_reference_flight.py` | 17 状态参考模型；`module_validation/flight/` |
| 天气与信息边界 | `engineering_reference_weather.py`、`engineering_replanning.py` | `module_validation/weather/`；区分合成预报与再分析数据 |
| 成像与服务 | `engineering_image_acceptance.py`、`engineering_service_boundary.py` | `capture_replanning/`、`service_boundary/` |
| 联合转录 | `engineering_joint_transcription.py` | `joint_transcription/`；飞行和 ECM 联合离散约束 |
| 起降与共享资源 | `engineering_platform_contact.py`、`engineering_joint_charging.py` | `platform_contact/`、`takeoff_landing/`、`joint_charging/` |
| 条件超图 | `engineering_conditional_hypergraph.py` | 旧运行绑定见 `book/reproduction/conditional_source_bindings.json`，一个源码哈希未恢复 |

`engineering_*.py` 均位于 `Agent/Publication/`。模块验证保存理论对照、实际数值、容差、源码身份和适用范围。文件名称中的 joint 不表示 237 塔全覆盖方案已完成。

原始区域入口和一些模块验证需要厂商手册的本地文件用于来源哈希。对应官方 URL、预期文件位置及 SHA256 在代码/profile/manifest 中。公共仓库不重新分发第三方手册。学习纯模型、运行充电模块与环境检查无需这些手册。

历史版本恢复（不叠加当前模块）：

```powershell
python -B book/reproduction/package_tools.py --extract ../historical-regional --version regional-2bf --include-data
```

这会保留与当时运行对应的代码；不是当前完整模型求解的替代物。历史脚本迁移前检查 ROOT、输入/输出、动态导入、求解内存和外部编译器。不要自动执行原工作站的云启动或账户工具。
