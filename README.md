# 参数辨识 v8：辨识期间保持速度闭环，平滑切换 Id

推荐模型：**reduce_emf_speedloop.slx**。`reduce_emf_identified.slx` 是内容相同的兼容副本。需要兼容原模型的 MATLAB R2024b、Simulink 和 Specialized Power Systems；算法已嵌入模型，直接运行 SLX 不依赖外部 .m 文件。

本版直接修改用户上传的 `reduce_emf_identified0x2860x29.slx`，保留该文件的其余设置。上传原件备份在 `tests/fixtures/reduce_emf_before_speedloop.slx`，SHA-256 为：

```
2eaeb54a2e7afe4bbc3d2eb53a300a4848084f01a8220a87d022f228a1ba40e9
```

## 修改内容

1. 将“辨识iq保持”内“辨识恒电流区间”的表达式改为 `0*u`。该统一使能同时控制 iq 锁存和速度 PI 输入误差置零：现在所有辨识阶段都保持 iq 直通，速度 PI 持续调节。保留原有模块及记录接口，不再冻结 iq 或暂停速度误差。
2. 在原 PLL 函数内，将 Id 指令偏移的正常切入、退出改成 **10 A/s** 斜坡：1 A 变化需要 **100 ms**。沿用原 100 微秒控制周期，不新增模块、端口或外部连线。I/F 复位或数值错误中止时仍立即清零。
3. 工作点切换阶段 5 延长至 **1 秒**：前 100 ms 为 Id 斜坡，后 900 ms 为速度闭环恢复。六个采样阶段仍各为 0.6 秒，随后执行原来的 4000 次逐样本 RLS。
4. 更新模型内已有说明文字及整机验证脚本：检查 iq 是否持续跟随速度 PI，而不是检查 iq 是否锁存；增加每段末尾 0.2 秒的 we 均值、最小值、最大值统计。

没有恢复先前的质量阻断门限：-6 等仍只是诊断提示，运行码 30 仍表示递推计算完成。原电流环、速度 PI 参数、PLL 的 PI/角度积分、I/F 启动、电气回路、电机/供电参数、滤波、采样周期、求解器及模型时长均保留。新增及修改代码的文字注释为中文。

归档对比确认只有三个内部资源发生变化：保持子系统的表达式/说明、PLL 脚本、采集面板的一段既有说明文字。所有模块位置和外部连线保持上传原件的布局。

## 运行时观察什么

关闭旧模型，打开 `reduce_emf_speedloop.slx` 后运行。重点比较辨识阶段原先的转速上拱、Id 切入退出时的 iq 凹口及转矩瞬态。

三相电流仍会在两个 Id 工作点出现不同包络，这是预期的激励。例如 iq 约 0.9 A 时，Id=0 的幅值约 0.9 A，Id=-1 A 的幅值约 1.35 A。本次修改针对每组内部的速度漂移及切换冲击，不要求两个工作点的三相幅值相同。

“辨识iq保持”中的原 Switch/记忆模块仍保留，但其统一使能为 0，处于直通状态。`id_iq_reference` 三列仍记录 `[速度PI原输出,实际输出iq指令,保持使能]`，现在前两列应一致、第三列应全为 0。

`param_identification` 仍为原来的 27 列；辨识公式及角度补偿方法未修改。速度闭环启用后 iq 会作必要调节，参数精度仍须结合实际采样验证，不能仅凭运行码 30 判定准确。

## 已执行的验证与边界

当前云环境没有 MATLAB/Simulink，**尚未执行实际模型编译、整机仿真或转速波动改善测量**。

已执行：

- 从 SLX 实际端口、Goto/From、Switch、Fcn 和一拍延迟逐拍追踪，在所有实验阶段及多组速度误差下，确认速度 PI 误差不被置零、iq 持续跟随速度 PI；该检查能检出上传原件中的冻结行为。
- GNU Octave 执行实际交付的 PLL：Id 切入、退出各 100 ms，每拍变化不超过 0.001 A，阶段 5 为 10000 拍，第二组采样前 Id 已到达 -1 A，退出无感模式立即复位。
- 实际 PLL 与原采集器联动，采齐六段并完成 4000 次递推；原理想、噪声、反转、缺数据及诊断不阻断等函数回归通过。合成函数测试不等于真实电机仿真。
- 模型归档/XML、SID、端点、输出维度、内嵌代码与源码一致性检查；所有接线、分支和模块位置与上传原件一致。五条电气网络及原电源负端均保留，求解器和多任务配置未更改。
- MATLAB 函数及验证脚本通过语法解析。

完整验证记录在 `validation_*.txt`。

## 在 MATLAB 中验证整机

将模型和脚本放在同一目录后运行：

```matlab
report = run_model_validation;
```

脚本先执行函数回归，再编译并仿真 25 秒。在结果断言前保存 `id_run_capture.mat`，含结果 `r`、阶段 `st`、原始信号 `raw`、低通信号 `filtered` 和 iq 指令记录 `iqReference`。结果报告增加各段速度统计，便于对照旧版；不以接近额定参数作为显示结果的条件。

直接打开 SLX 运行时也可观察原有示波器；无需先运行外部 .m 文件。

## 复查与重建

```text
python check_speedloop_model.py
python check_speed_control.py
python check_control_path.py --self-test
python check_power_connections.py --self-test
```

后两项的历史错误自测需要仓库 Git 历史。独立函数测试可运行 `test_speedloop_sequence; test_identification`。

`python build_speedloop.py` 可使用上传原件备份及 `pll_vbemf.m` 重建两个交付文件，无需 MATLAB。最终整机编译和仿真仍需要 MATLAB/SPS。

## 精度诊断：回放六段真实采样

若电机已正常运行，但电感、磁链或电阻仍有偏差，可在本次仿真结束后运行：

```matlab
report = diagnose_identification;
```

将 `diagnose_identification.m` 放在当前 MATLAB 文件夹或搜索路径中即可。脚本自动读取工作区的 `id_filtered`、`identification_state`、`param_identification`；也支持工作区 `out` 或选择已有采样 MAT 文件。已有 `id_run_capture.mat` 可以使用 `diagnose_identification('id_run_capture.mat')`。

该脚本不加载或修改模型、不更改参数估计，也不清空工作区。它依据各阶段实际采样完成时间和周期平均长度，重建原辨识器真正保存的六个2000点窗口，导出 `identification_diagnosis.mat`。文件只含这六段窗口、最终结果和诊断摘要，窗口数组未压缩约0.48 MB，便于传回分析。

摘要包含：六段电压/电流/转速均值，正负注入段的实际电流及转速差，原角度差分重放、仅速度归一化的角度对比，实际归一化RLS的解析重放、移除初始协方差影响的对照，以及两组角度误差对L/磁链的局部灵敏度。归一化角度对比尚未补偿电流项，只用于诊断，不作为新的估计结果。

当前速度闭环会调节iq，而论文式12.25的直接电压差分依赖正负段dq电流一致。不能仅凭最终L、磁链、角度和Rs几个数确定实际偏差来源；应先用六段记录区分电流/速度变化、采样时段和求解影响，再验证精度修改。诊断中的解析求解只用于核对原RLS，不替代模型的逐样本递推。

`test_diagnose_identification` 使用实际交付的辨识函数生成带噪记录，验证正转、低速周期平均上限和反转三种情况下的窗口及参数重建；误差小于1e-8。当前已执行GNU Octave函数测试及MATLAB语法解析，未执行MATLAB整机仿真。
