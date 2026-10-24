"""基于已运行正常的v8模型更新辨识器及恢复段小激励，保留控制环和整齐接线。"""
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as E

HERE=Path(__file__).resolve().parent

def prop(el,name,value):
    item=el.find(f"P[@Name='{name}']")
    if item is None:item=E.SubElement(el,'P',{'Name':name})
    item.text=str(value)

def block(system,sid,kind,name,position,properties):
    b=E.Element('Block',{'BlockType':kind,'SID':str(sid),'Name':name})
    prop(b,'Position',position);prop(b,'ZOrder',100004)
    for key,value in properties.items():prop(b,key,value)
    index=next((i for i,x in enumerate(system) if x.tag=='Line'),len(system))
    system.insert(index,b)

def line(system,src,dst,points=None):
    el=E.SubElement(system,'Line');prop(el,'Src',src)
    if points:prop(el,'Points',points)
    prop(el,'Dst',dst)

def build():
    with ZipFile(HERE/'tests/fixtures/reduce_emf_speedloop_v8.slx') as z:
        entries=z.infolist();data={info.filename:z.read(info.filename) for info in entries}
    for number,name in [(22,'identify_vbemf'),(65,'pll_vbemf')]:
        key=f'simulink/stateflow/chart_{number}.xml';chart=E.fromstring(data[key])
        chart.find(".//P[@Name='script']").text=(HERE/(name+'.m')).read_text().replace(name+'(','fcn(',1)
        data[key]=E.tostring(chart,encoding='utf-8',xml_declaration=True)
    key='simulink/systems/system_12300.xml';system=E.fromstring(data[key])
    demux=system.find("Block[@SID='12096']")
    demux.find('PortCounts').set('out','12')
    prop(demux,'Outputs','[1 1 1 1 1 1 1 8 1 1 10 4]')
    prop(demux,'Position','[1540, 100, 1545, 820]')
    # 保持原有12路之前的输出端口坐标，仅延长分解器；输入短线补齐30像素。
    for ll in system.findall('Line'):
        if any(q.text=='12096#in:1' for q in ll.findall("P[@Name='Dst']")):
            prop(ll,'Points','[32.5, 0; 0, 120]')
            points=ll.find("P[@Name='Points']");ll.remove(points)
            ll.insert(list(ll).index(ll.find("P[@Name='Dst']")),points)
    block(system,12800,'Goto','数据损耗标定标签','[1600, 900, 1780, 924]',
          {'GotoTag':'PID_data_loss','TagVisibility':'local'})
    line(system,'12096#out:12','12800#in:1','[15, 0; 0, 122]')
    block(system,12801,'From','数据标定R_V_条件数_激励','[100, 1708, 225, 1732]',
          {'GotoTag':'PID_data_loss'})
    block(system,12802,'Display','数据标定R_V_条件数_激励强度','[420, 1680, 900, 1760]',
          {'FontSize':'14'})
    line(system,'12801#out:1','12802#in:1')
    prop(system,'Location','[0, 0, 2400, 1850]')
    prop(system.find("Block[@SID='12088']"),'Description',
         'v9：末0.2秒采样；用连续电压/电流积分数据标定损耗，补偿电流及转速差分后逐样本RLS，不预填额定参数。')
    prop(system.find("Annotation[@SID='12662']"),'Name',
         '速度PI持续调节，Id基准切换100毫秒；恢复段内有峰值0.2A、200毫秒平滑标定激励。\n'
         '每个0.6秒采样段使用最后0.2秒；先标定损耗，再补偿电流/转速差并执行4000次递推。\n'
         '运行码30表示计算完成；不按质量门限中止，不限制估计值正负或范围。')
    prop(system.find("Annotation[@SID='12720']"),'Name',
         '原五组电压诊断保留；最下方四项为数据标定R、等效压降V、条件数和激励强度。\n'
         '标定系数来自采样积分中心变化，不使用额定电阻、电感或磁链填充结果。\n'
         '前27列接口保留，28～31列新增标定诊断；两组角度随电感递推继续修正。\n'
         '条件数大或激励弱只作诊断，不阻止输出；最终精度须用整机结果复测。')
    data[key]=E.tostring(system,encoding='utf-8',xml_declaration=True)
    key='simulink/systems/system_root.xml';root=E.fromstring(data[key])
    prop(root,'SIDHighWatermark',12820)
    prop(root.find("Block[@SID='11765']"),'Description',
         'v9：速度PI不冻结；采样段0.6秒，Id恢复段1秒含0.2A平滑标定激励，正式采样前恢复650毫秒。')
    data[key]=E.tostring(root,encoding='utf-8',xml_declaration=True)
    output=HERE/'reduce_emf_accuracy.slx'
    with ZipFile(output,'w') as z:
        for info in entries:z.writestr(info,data[info.filename])
    for name in ['reduce_emf_identified.slx','reduce_emf_speedloop.slx']:
        (HERE/name).write_bytes(output.read_bytes())
    print(output)

if __name__=='__main__':build()
