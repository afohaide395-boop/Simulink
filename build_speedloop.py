"""在用户本次上传的SLX上关闭辨识保持，保留原始接线及全部其他设置。"""
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as E

HERE=Path(__file__).resolve().parent
source=HERE/'tests/fixtures/reduce_emf_before_speedloop.slx'
with ZipFile(source) as z:
    entries=z.infolist();data={info.filename:z.read(info.filename) for info in entries}

def prop(el,name,value):
    q=el.find(f"P[@Name='{name}']")
    if q is None:q=E.SubElement(el,'P',{'Name':name})
    q.text=value

# 同一个保持使能控制iq锁存和速度PI输入置零；固定为0同时关闭两处。
key='simulink/systems/system_12400.xml';system=E.fromstring(data[key])
control=system.find("Block[@SID='12230']")
assert control.find("P[@Name='Expr']").text=='(u>=2)*(u<=4)+(u>=6)*(u<=8)'
prop(control,'Expr','0*u')
prop(control,'Description','辨识保持使能固定为0：iq直通速度PI输出，速度PI误差不再置零。')
data[key]=E.tostring(system,encoding='utf-8',xml_declaration=True)

# 在原PLL内实现Id斜坡及1秒恢复段，不新增模块、端口、速率或外部线。
key='simulink/stateflow/chart_65.xml';chart=E.fromstring(data[key])
chart.find(".//P[@Name='script']").text=(HERE/'pll_vbemf.m').read_text().replace('pll_vbemf(','fcn(',1)
data[key]=E.tostring(chart,encoding='utf-8',xml_declaration=True)

# 仅更新已有说明文字，保留采集与RLS代码及其无门限输出逻辑。
key='simulink/systems/system_12300.xml';system=E.fromstring(data[key])
note=system.find("Annotation[@SID='12662']/P[@Name='Name']")
note.text=('辨识期间速度PI持续调节，iq不锁存；Id切入/退出为100毫秒斜坡。\n'
           'Id工作点切换段为1秒，包含斜坡及恢复时间；采样段仍各0.6秒。\n'
           '运行码30为4000次递推完成；质量诊断仍不阻断采样和参数输出。')
data[key]=E.tostring(system,encoding='utf-8',xml_declaration=True)

with ZipFile(HERE/'reduce_emf_identified.slx','w') as z:
    for entry in entries:z.writestr(entry,data[entry.filename])
(HERE/'reduce_emf_speedloop.slx').write_bytes((HERE/'reduce_emf_identified.slx').read_bytes())
print(HERE/'reduce_emf_speedloop.slx')
