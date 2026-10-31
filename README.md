# 电机与观测器角度对比、模型整理

当前模型：**[reduce_emf_tidy.slx](reduce_emf_tidy.slx)**。直接下载单独的SLX，代码均已嵌入。环境沿用原模型：MATLAB R2024b、Simulink及Specialized Power Systems。

进入“运行监测”，查看三个保留的示波器：

| 示波器 | 内容 |
|---|---|
| 电机与观测器_角度及误差对比 | 真实/PLL/FOC电角度、两种实际角度差、辨识A/B角度及实验阶段 |
| 参数辨识_电感与磁链 | 电感H、磁链Wb |
| 电机运行_三相电流转矩转速 | 三相电流、转矩、机械转速 |

Scope由21个减少为3个，并清理其多余输入支路。角度差统一为“真实电角度减观测角”，折回[-pi,pi]。真实角度只用于监测，未用于控制或辨识。

辨识角度应与实际用于Park变换的FOC角度误差比较；PLL原始角度差另行显示，便于检查原有一拍延迟。正负注入会引入额外角度偏移，应结合阶段2、6的基准窗口比较。

辨识代码保留用户原变量名、分段数组、switch采样和逐点PSO。删除12个无用缓存、冗余计数及重复回归计算，去掉差矢量幅值门限和适应度初值的人为上限；保留数据边界与非有限输入保护。1Hz差矢量滤波、五路50Hz低通、30粒子、150代等设置保留。代码见[model_cleanup/fcn.m](model_cleanup/fcn.m)。

已修正四路输出的接线：1/2为角度A/B，3/4为电感/磁链；param_identification日志相应记录这四列。原控制算法及电气参数保留。

**已完成函数回放和离线端口/连线检查；云端没有MATLAB/Simulink，尚未实际打开、编译或整机仿真。** 正常记录下，精简前后全部四路输出及150代PSO逐位一致。不能据此保证实际整机精度或原生界面运行结果。

详细说明与验证记录见[model_cleanup/README.md](model_cleanup/README.md)。本次上传原文件备份位于tests/fixtures/reduce_emf_user_cleanup.slx，未覆盖原文件。

此前模型及脚本保留作历史记录；此前代码精简版说明见[docs/simple_code_v14.md](docs/simple_code_v14.md)。
