# 参数辨识与无感控制 v11：功能模块整理

推荐打开 **[reduce_emf_organized.slx](reduce_emf_organized.slx)**。所有算法代码均已嵌入SLX，直接运行不依赖外部`.m`文件；需要与原模型兼容的MATLAB R2024b、Simulink及Specialized Power Systems。

本版基于v10整理层次和接线。顶层从149个模块收拢为21个模块，功能入口如下。

| 顶层子系统 | 包含内容 |
|---|---|
| 反电动势观测与锁相环 | 原反电动势观测器、PLL、Ed/Eq监测、原角度反馈/转速/Id偏移延迟、估计转速单位转换 |
| 电流采样与矢量控制 | 电机测量总线、电流采样、Clarke/Park变换、d/q电流环、逆Park变换、SVPWM及电压监测 |
| 启动与速度控制 | 原I/F启动、转速参考、速度环、iq指令选择、控制角度选择和辨识保持逻辑 |
| 参数辨识与采集 | v10的中文注释、具名变量、独立命名输出，以及原滤波、采样、显示和日志 |
| 运行监测 | 三相电流、转矩、转速、角度及I/F/PLL切换监测；原备用子系统一并保留 |

双击相应子系统即可查看原模块。电源、逆变桥、电机和powergui仍在顶层，物理电气连接的分支结构全部保留。

## 接线整理

- 短的顺向连接保持直接接线；跨模块信号、反馈线和监测分支使用Goto/From。
- 原来只在顶层有效的局部标签，连同对应From一起改为带`CTRL_`前缀的全局标签，避免移入子系统后失效。
- 子系统内部新增的Ed/Eq、采样机械速度等路由使用局部标签；电机测量总线通过`LINK_Motor_measurements`传入电流采样模块。
- Goto/From隐藏通用模块名，只显示信号标签，减少标注拥挤。
- 新增四个子系统均为虚拟子系统。原计算模块、延迟、采样周期、PI参数、PLL算法及参数辨识代码保留。

观测器角度反馈仍为：**原PLL角度 → 原“PLL角度反馈延迟” → 观测器**。Goto/From不引入延迟，也不替代原延迟。PWM脉冲及辨识确认的时序同样沿用v10。

参数辨识代码入口仍为：**参数辨识与采集 → 参数辨识算法 → 参数辨识**。独立输出和31列兼容日志的说明见 [v10代码整理说明](docs/readable_v10.md)。

## 已验证与边界

- 展开Goto/From及所有虚拟层次后，逐路比较了**478个原计算输入**，实际信号来源与v10完全一致。
- 所有原计算参数及现有子系统正文保持一致；七个Stateflow图的算法代码及配置保留，只同步移动后的图路径。
- 五组物理电气连接树及直流负端回路与v10一致；检查了PWM、观测器反馈及辨识采样确认。
- 原速度控制与I/F接线回归通过，速度PI继续调节iq，辨识保持使能继续为0。
- 故障回归能检出人为绕过观测器反馈延迟、PWM误接转速信号及跨模块状态标签未导出。
- 顶层及四个新子系统没有模块重叠或信号线穿过其他模块。物理电气线的拓扑由独立结构检查验证。

当前环境没有MATLAB/Simulink，尚未执行SLX实际编译或电机整机仿真。本次保留算法，不据静态检查宣称整机仿真已经通过。

## 复查与重建

```text
python check_organized_model.py --self-test
python check_control_path.py --model reduce_emf_organized.slx --self-test
python check_speed_control.py --model reduce_emf_organized.slx
python check_organized_layout.py
```

布局检查可加`--output-dir <目录>`生成几何预览图；该预览不是Simulink实际截图。检查记录位于`validation_organized_*.txt`。

```text
python build_organized.py
```

从`tests/fixtures/reduce_emf_readable_v10.slx`重建本版。新文件名为`reduce_emf_organized.slx`；现有`build_readable.py`等命令继续对应v10，旧版文件用于对照。

在MATLAB中可直接打开新SLX运行；如需编译、25秒整机仿真及记录，运行：

```matlab
report = run_model_validation;
```

验证脚本优先选择`reduce_emf_organized.slx`。`param_identification`、`id_raw`、`id_filtered`、`identification_state`和`id_iq_reference`的日志格式沿用v10，`diagnose_identification`可继续使用。

v9算法说明保存在 [docs/algorithm_v9.md](docs/algorithm_v9.md)。
