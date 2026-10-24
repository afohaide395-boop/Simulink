本包包含：
1. reduce_emf_checked.slx：补齐端口声明、显式折点及悬空端口的模型。
2. repair_organized_wires.m：MATLAB/Simulink真实端口重连与编译脚本。
3. organized_wiring_manifest.json：与本模型对应的300条接线清单。

请将所有文件解压到同一目录，关闭有未保存修改的同名模型。
在MATLAB命令窗口切换到该目录并运行：

    report = repair_organized_wires;

脚本使用实际PortHandles重建8个可见图的普通信号线，核对每个输入/输出端口、
线头与线尾、水平/竖直线段，再执行Simulink模型更新编译。
仅在全部检查通过后，才保存reduce_emf_checked_native.slx和native_wiring_report.mat。
随后请运行reduce_emf_checked_native.slx。

物理电气线不会被重建，PI、PLL、辨识算法和采样周期保留。
原电机负载输入未连接，本版显式接零保持原空载条件；未使用的8.5Nm阶跃和
已停用子系统收进“备用模型_已停用”。没有把备用8.5Nm负载接入电机。

云端已完成端口、清单、来源与几何的离线检查，未执行MATLAB/Simulink。
不能将离线通过理解为上述原生重连脚本或整机仿真已经运行通过。
本机脚本只编译，不执行完整电机仿真。
