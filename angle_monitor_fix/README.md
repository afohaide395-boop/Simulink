# 本次上传模型的角度监测修复

输入：用户上传的 `reduce_emf_tidy.slx`（174435 字节）。
输出：仓库根目录 `reduce_emf_angle_fixed.slx`。

上传模型中发现：

- Goto `MON_delta_ctrl`（SID 14144）直接接在 FOC 角度取模模块（14013）后面。因此第三幅图的“实际FOC角度差”实际是角度，范围为 0～2π。
- 第二幅图使用 Sum（14146）直接计算两个取模角度之差，没有跨周折回，导致周期性尖峰。
- PLL 显示支路有额外 Unit Delay（14147）。无感运行时，这条支路所用的角度与控制支路 Unit Delay（12110）后的角度一致。因此原图约 0.02 rad 的曲线不能当作未经延迟的 PLL 误差后，再额外叠加一拍延迟。
- MON_theta_motor、MON_theta_pll、MON_theta_ctrl 三个局部 Goto 缺失，残留的 From/误差计算支路无法正常解析。

修复后监测分别读取电机真实电角度、PLL 直接输出、Park 实际使用的控制角度。两种角度误差均用 `atan2(sin(theta_true-theta_est), cos(theta_true-theta_est))` 折回到 [-π,π]。第二幅图同时显示 PLL 和 FOC 误差；第三幅图显示实际 FOC 误差与两组辨识结果。

相对于本次上传文件，仅 `simulink/systems/system_13240.xml` 被修改。辨识代码（包括用户设置的 fc=0.01 Hz）、控制支路原有延迟、注入时序、控制器、物理电机和求解器均逐字节保留。电机/参数示波器的用户配置也保留。

辨识结果应分别与基准阶段 2、6 的实际 FOC 误差比较。阶段 3/4、7/8 中实际误差还含注入造成的偏移；阶段 9 的工作点已改变。此次修复只建立正确的比较依据，不能据此宣称已消除辨识偏差。辨识算法在 stage4/8 由正负注入电压差计算角度，PSO 在 stage9 搜索电感、磁链，故调整 PSO 不能直接修复角度偏差。

可复现修复：

```sh
python angle_monitor_fix/repair_monitor.py <本次上传模型路径> reduce_emf_angle_fixed.slx
```

离线校验结果见 `validation.txt`。当前环境没有 MATLAB/Simulink，未执行 R2024b 加载、编译及仿真，不保证运行结果或辨识精度。
