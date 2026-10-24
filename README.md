# 参数辨识与无感控制：SLX加载结构修正

当前模型：**[reduce_emf_load_fixed.slx](reduce_emf_load_fixed.slx)**。这是单独的Simulink模型文件，不需要解压或安装包。

用户报告v12在MATLAB R2024b Update 8中，模型窗口出现之前发生Access violation。调用栈包含`gp_SigPropStr`、`sseg_source_location`、`slsSegmentConnectionParameter::getPortType`及XML加载器，故障发生在连线加载路径。不要继续使用旧提交中的v12模型或压缩包。

## 查到并修正的问题

v12生成器把连线的`Points`属性写到了`Src`之前。原始模型中含这两个属性的记录均为`Src`在前；连线加载器需要先建立源连接，再处理依赖源端口的几何信息。v12共有305处倒序。崩溃栈与这处加载顺序缺陷吻合，但没有MATLAB运行时复现，不能据此声称已经证明唯一根因。

本版将其恢复为原生文件采用的顺序，例如：

```xml
<Line>
  <P Name="ZOrder">200001</P>
  <P Name="Src">5#out:1</P>
  <P Name="Points">[-12.5, 0]</P>
  <P Name="Dst">7#in:1</P>
</Line>
```

同时修正两个HMI记录和两个图形接口记录路径：模块已经移至“电流采样与矢量控制”，但旧记录仍指向顶层`From58`和`From111`。这些保存路径现与对应SID的实际路径一致。

相对v12，只更改九个图的连线属性顺序和两个监测元数据文件。模块、端口、信号来源、折点数值、PI、PLL、电机参数、采样周期及全部算法代码保留，分组布局继续保留。

`reduce_emf_checked.slx`已同步为同一份修正文件，避免当前分支继续提供有倒序问题的同名模型。旧的固定提交下载链接不会自动更新，必须下载本页的新文件。旧压缩包已从当前分支撤下。

## 验证范围

```text
python build_checked.py
python audit_model_load_paths.py reduce_emf_load_fixed.slx
python test_load_serialization.py
python check_checked_model.py --self-test
python check_checked_layout.py --model reduce_emf_load_fixed.slx
```

已验证：

- 崩溃v12回归样本必须被拒绝：305处Src/Points倒序、2处HMI路径及2处记录路径错误。
- 当前模型这些检查项全部通过；v8及v10原文件作为原生保存顺序的对照通过。
- 修正前后的模块和连接完全相同，连线参数值及算法完全相同。
- 八个可见图的300条信号线没有离线发现的悬空活动端口、斜线或穿模块情况。
- 730个原输入定义的信号来源保留，物理电气连接树保留，控制路径回归通过。

**当前云端没有MATLAB/Simulink，尚未执行实际打开、编译或整机仿真。上述结论是文件结构与语义回归，不能等同于MATLAB实际加载成功。**

验证记录见`validation_load_*.txt`和`validation_checked_*.txt`。崩溃分析只保留必要的非敏感栈符号，不保存用户报告中的会话、进程或授权标识。

## 恢复备份

[reduce_emf_recovery_v8.slx](reduce_emf_recovery_v8.slx)是此前记录为已运行正常的v8备份的逐字节副本，未重写其SLX内部结构。它保留旧布局及旧接口，用于恢复到前一个工作状态。SHA256为：

```text
57908e1e783b14f056bc3386c21f46c1826ef29ce5926cf6a3a757542352bfd7
```

当前云端也未重新运行该恢复文件。它与原备份字节相同，不是新生成的分组模型。

## 历史文件与后续复测

`tests/fixtures/reduce_emf_checked_v12_crash.slx`仅用于离线故障回归，**不要在MATLAB中打开这个已报告崩溃的样本**。

包内原生重连脚本、旧版分组说明和旧版检查记录保留作历史依据。当前交付以本页SLX为准；用户不需要运行安装包。若在本机复测整机，可使用`run_model_validation`，它优先选择本版`reduce_emf_load_fixed.slx`；这一步尚未在云端运行。
