# 参数辨识 v10：命名输出与代码整理

推荐打开 **reduce_emf_readable.slx**。模型中的辨识代码已经修改并嵌入SLX，直接运行无需额外加载`.m`文件。需要与原模型兼容的MATLAB R2024b、Simulink及Specialized Power Systems。

本次只整理代码风格与接口，电机、控制环、PLL/I/F启动、实验阶段、滤波和数值算法均沿用v9。

## 查看代码

打开模型后依次进入：

**参数辨识与采集 → 参数辨识算法 → 参数辨识（MATLAB Function）**。

外部可读源码为 [identify_vbemf.m](identify_vbemf.m)，与模型内代码一致。

- 输入按电气量命名：`u_d`、`u_q`、`i_d`、`i_q`、`omega_e`、`experiment_stage`。
- 每条语句单独一行，使用中文注释，长公式分行。
- 主流程按初始化、阶段切换、采样、角度及损耗标定、参数递推、输出划分。
- 电阻消元、损耗标定、角度补偿及窗口诊断分别放入具名辅助函数。
- 状态使用具名字段，如`estimate.Ls_hat`，不再用`result(3)`等编号表达参数。

## 独立输出

MATLAB Function不再输出统一的`y`向量。每个参数、标志和诊断量均使用独立命名端口，共31个标量及2组角度历史。

| 端口 | 名称 | 含义/单位 |
|---|---|---|
| 1 | Ls_hat | 定子电感/H |
| 2 | psi_f_hat | 永磁体磁链/Wb |
| 3 | Rs_hat | 定子电阻/Ω |
| 4、5 | delta_theta_A、delta_theta_B | 两组位置误差/rad |
| 6、7 | Lpsi_done、Rs_done | 数值计算完成标志 |
| 8 | ident_status | 辨识运行状态码 |
| 12、16、17 | rls_updates、sample_ack、diagnostic_code | 迭代次数、采样确认、诊断提示 |
| 28、29 | Rs_cal、V_inverter_cal | 辅助标定电阻/Ω、等效压降/V |
| 32、33 | delta_theta_A_history、delta_theta_B_history | 各400点角度历史 |

全部端口顺序见 [identification_interface.py](identification_interface.py)。在MATLAB中仅取主要参数时可写：

```matlab
[Ls_hat, psi_f_hat, Rs_hat, delta_theta_A, delta_theta_B] = ...
    identify_vbemf(u_d, u_q, i_d, i_q, omega_e, experiment_stage);
```

函数具有持久状态，需要按原实验阶段逐拍调用。未完成计算的参数仍返回NaN。

## 模型布局与记录兼容

“参数辨识算法”内每个输出通过一条短线连接同名信号标签。原参数示波器与状态显示直接读取对应物理量，采样确认仍经过一拍Ts延迟反馈给PLL。

“记录与监视”仅在日志边界将命名信号拼成原有31列记录，并提供分组诊断显示。`param_identification`、`id_raw`、`id_filtered`、`identification_state`、`id_iq_reference`的格式不变，原`diagnose_identification`仍可使用。

`record_identification.m`只是旧测试的31列兼容适配器，模型中的辨识函数不调用它。`tests/fixtures/identify_vbemf_v9.m`为逐拍对比保留的原函数，不是当前算法入口。

`reduce_emf_accuracy.slx`、`reduce_emf_speedloop.slx`及`reduce_emf_identified.slx`是内容相同的兼容副本。

## 验证

已在GNU Octave执行实际交付函数：正常、含噪声及反转三组连续物理方程数据，共151,239拍。每拍的全部31个标量和两组400点角度历史均与v9逐位一致，覆盖六段采样、恢复段、4000次真实递推、完成保持、异常输入及重新启动。

结构检查核对独立输出、Stateflow/底层端口、显示及日志映射、采样确认和电气连接。布局检查确认辨识面板及两个新子系统无模块重叠、无连线穿过其他模块。

当前环境没有MATLAB/Simulink，尚未执行SLX实际编译或电机整机仿真。`run_model_validation`可在MATLAB中编译并仿真25秒、保存记录；本次不能据函数回放宣称整机精度提高。

```text
python check_readable_model.py
python check_control_path.py --self-test
python check_speed_control.py
python check_power_connections.py --self-test
python check_readable_layout.py
```

生成连续回放数据后，可在MATLAB/Octave执行逐拍对比：

```text
python make_accuracy_fixtures.py
```

```matlab
test_readable_equivalence;
```

其他原有测试已改为通过兼容适配器调用新的命名接口。验证记录见`validation_readable_*.txt`。

`python build_readable.py`从v9快照和当前辨识源码重建四个交付SLX。旧`build_accuracy.py`、`build_speedloop.py`及对应检查命令委托当前工具，避免重建出端口不匹配的旧接口模型。

v9算法的详细推导和原有仿真边界记录保存在 [docs/algorithm_v9.md](docs/algorithm_v9.md)，其中版本接口说明以本页为准。
