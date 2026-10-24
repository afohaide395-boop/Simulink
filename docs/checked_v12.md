历史记录：v12已被报告在R2024b加载时崩溃。不要使用该文档中的旧下载版本；当前说明见仓库README。

# 参数辨识与无感控制 v12：端口与接线复查

交付文件：**[reduce_emf_checked_package.zip](reduce_emf_checked_package.zip)**。
单独模型为 **[reduce_emf_checked.slx](reduce_emf_checked.slx)**，算法均嵌入SLX。
需要与原模型兼容的MATLAB R2024b、Simulink及Specialized Power Systems。

本版针对“电流采样与矢量控制”“反电动势观测与锁相环”“运行监测”被报告的断线，补上端口完整性及显式线段检查。上一版仅展开信号来源并比较，不能证明Simulink加载后每条线都附着到真实端口，也漏掉了悬空端口与斜线。

## 本次修正

- 为整理图中的Goto/From明确写入输入/输出端口数。
- 八个可见图的300条普通信号线都写入明确折点及线层次，不依靠省略几何信息自动显示。
- 对齐I/F输入常量和参数辨识电压诊断入口，消除离线发现的斜线。
- 原电机负载转矩输入悬空，本版显式接Ground（0），保持原来的零外加负载。原来未使用的8.5Nm阶跃没有接入电机。
- 原停用子系统和未使用阶跃移至“备用模型_已停用”，补齐备用端口终结，不在运行监测图中显示孤立端口。
- PI、PLL、辨识函数、原延迟、采样周期及五组物理电气连接树保留。

顶层仍按启动与速度控制、电流采样与矢量控制、反电动势观测与锁相环、参数辨识与采集、运行监测组织。

## MATLAB内原生重连与编译

**云端没有MATLAB/Simulink。当前完成的是离线检查，尚不能声称已通过真实SLX编译，也不能确认用户所见的所有界面断线都已消失。**

包内的 [repair_organized_wires.m](repair_organized_wires.m) 不根据模块矩形猜测端点。它在MATLAB中读取真实`PortHandles`，重建清单对应的普通信号线、使用实际端口坐标设置水平/竖直折线，再检查端口附着和编译。这是进一步排除加载/显示连接问题的本机步骤，云端未执行此脚本。

将压缩包解压到同一目录，在MATLAB该目录运行：

```matlab
report = repair_organized_wires;
```

只有300条连接、全部受检活动端口、折线和Simulink更新编译都通过，才保存：

- `reduce_emf_checked_native.slx`：由Simulink原生接口重接并编译后的模型。
- `native_wiring_report.mat`：实际本机检查结果。

下载的SLX保留，脚本不覆盖它。脚本不会修改物理电气连接或执行整机仿真。若模型已有未保存修改，脚本会停止，避免丢失用户工作。源码已通过MATLAB语法解析，实际MATLAB运行仍待本机执行。

## 离线检查证据

`python check_checked_model.py --self-test` 验证：

- 八个可见图的300条显式接线，按模块类型/端口元数据逐个检查活动输入和输出，不用“已经出现在线上”的数量反推应有端口数。
- 所检图中没有斜线、重叠模块或信号线穿过其他模块。
- 730个原输入定义的信号来源保留；原悬空输入与新增Ground均按零信号处理，实际电机负载输入明确接零。
- Goto/From唯一解析，SID和子系统关系有效，Stateflow算法及配置保留，物理电气连接树保留。
- 人为删除Park输入线、人为插入斜线时，检查必须失败。

这些结论仅涵盖指定可见图和序列化文件，不能替代真实端口检查或整机仿真。

```text
python build_checked.py
python check_checked_model.py --self-test
python check_checked_layout.py
python check_control_path.py --model reduce_emf_checked.slx --self-test
python check_speed_control.py --model reduce_emf_checked.slx
```

`build_checked.py`从v11快照重建模型及接线清单；`package_checked.py`生成交付包。
验证记录见`validation_checked_*.txt`。前一版说明保存在 [docs/organized_v11.md](docs/organized_v11.md)。

## 算法与仿真

参数辨识仍位于“参数辨识与采集 → 参数辨识算法 → 参数辨识”，保留中文注释及电感、磁链、电阻等独立命名输出。原31列日志兼容。

原生重连和编译通过后，可在完整仓库运行：

```matlab
report = run_model_validation;
```

它优先选择`reduce_emf_checked_native.slx`，执行原有函数回归及25秒整机仿真并保存记录。云端本次没有运行此步骤。
