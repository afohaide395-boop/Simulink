# 历史版本说明

当前模型见[仓库主页](../README.md)。以下保留此前代码精简版记录。

# 参数辨识代码精简版

当前模型：**[reduce_emf_simple.slx](../reduce_emf_simple.slx)**。直接打开SLX即可，代码已嵌入，不需要安装包或外部M文件。环境要求沿用原模型：MATLAB R2024b、Simulink及Specialized Power Systems。

打开“参数辨识与采集 → 参数辨识算法 → 参数辨识”可查看本版代码，对应源文件为[identify_simple.m](../identify_simple.m)。旧的`identify_vbemf.m`及其他SLX保留供历史版本回归使用。

## 代码整理

代码从610行减为323行，按以下顺序排列：

1. 初始化与阶段切换。
2. 采样：各阶段先等待0.4秒，保存最后0.2秒的数据。
3. 正负注入计算位置误差，再交替读取A/B基准数据，执行4000次逐样本RLS，随之计算电阻。
4. 电感、磁链、电阻、两组角度等分别输出。

采用`ud、uq、id、iq、we、Ls、psi、Rs`等短电气量名称，中文注释，一行一条语句。`phi、K、P、x`分别表示RLS回归向量、增益、协方差及缩放参数。

删除漂移/纹波评分、质量诊断码、条件数显示、两组400点历史向量及相应辅助块。六段存储数据直接按采样顺序写入，删除重复的窗口缓存、窗口重排和旧31列兼容拼接。

保留五路50Hz低通和相同的电周期平均。辅助压降修正单独放在`loss_fit`、`angle_fix`中；它直接参与角度计算，用于处理两组注入时电流/转速不完全相等的影响。这是此前加入的工程修正，**不属于论文主RLS公式**，本次没有将它删除或换成额定参数。`rs_calc`仍由两组q轴方程逐次求电阻，不复制辅助电阻。

本版针对代码风格与辅助逻辑做精简，没有提出新的精度改进。I/F启动、速度PI、电流PI、反电动势观测器、PLL、注入时序、Id斜坡、电机、电气连接及求解器均保持v13的原字节。

## 输出和日志

独立输出由33路缩减为12路：

| 输出 | 含义 |
|---|---|
| Ls、psi、Rs | 电感H、磁链Wb、电阻Ω |
| da、db | 两组位置误差rad |
| doneL、doneR | L/磁链与电阻是否完成数值计算 |
| run | 0等待、1采样、2角度预处理、3递推、30完成 |
| n | 实际RLS次数，完成时4000 |
| ack | 已完成采样的阶段号，经原一拍延迟反馈给PLL |
| R0、V0 | 角度修正使用的辅助电阻Ω与等效压降V |

`param_identification.signals.values`现在按上表顺序记录**12列**：

```text
[Ls psi Rs da db doneL doneR run n ack R0 V0]
```

此前按31列读取的诊断脚本仅适用于旧模型。本版保留原始/滤波后采集记录及状态记录，不需要诊断脚本即可运行。未计算的参数为NaN；负参数不会被夹成正数。完成标志只表示计算完成，不是精度验收。

## 验证与边界

已在GNU Octave用八组连续电机方程记录逐拍对照v13，保留的12路输出逐位一致，覆盖不同电机参数、反转、噪声、速度波动、谐波和逆变器压降，且确实进行4000次逐样本更新。另检查复位、缺数据、非有限输入及整数阶段号。记录见`validation_simple_algorithm.txt`。

离线检查确认：只更改SLX内辨识相关的五个文件，其他内容逐字节不变；Stateflow、MATLAB Function及其底层S-Function端口定义和12列日志顺序一致；删除105个诊断/历史端口与辅助块，224条可见信号线及底层封装端口完整。连线Src仍先于Points，监测路径有效。

**云端没有MATLAB/Simulink，尚未执行实际打开、编译或整机仿真。Octave函数回放与离线结构检查不能替代这些验证，也不保证实机辨识精度。**

复查命令：

```text
python build_simple.py
python check_simple_model.py
python make_accuracy_fixtures.py
```

生成回放数据后，在MATLAB或Octave运行`test_simple_identification`。`tests/fixtures/reduce_emf_load_fixed_v13.slx`和`identify_vbemf_v13.m`为修改前对照，构建器不会覆盖原模型。

之前的加载结构修正说明保存在[docs/load_fixed_v13.md](load_fixed_v13.md)，其模型与历史回归文件继续保留。
