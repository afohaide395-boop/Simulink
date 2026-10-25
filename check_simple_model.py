"""核对精简辨识器的接口和连接，确保其余控制与电气部分不受影响。"""
from collections import Counter
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as E

from audit_model_load_paths import audit
from build_simple import BASE, MODEL, PARTS, INPUTS, OUTPUTS
from check_checked_model import check_visible_graphs, port_counts
from check_organized_model import ResolvedGraph
from check_control_path import check as check_control
from check_speed_control import check as check_speed
from model_geometry import Geometry, field, is_orthogonal, crosses_rectangle


def check():
    assert not audit(MODEL)
    with ZipFile(BASE) as old, ZipFile(MODEL) as new:
        assert new.testzip() is None
        assert old.namelist() == new.namelist()
        assert {name for name in old.namelist() if old.read(name) != new.read(name)} == PARTS
        for name in new.namelist():
            if name.endswith(('.xml', '.rels')):
                E.fromstring(new.read(name))
        chart = E.fromstring(new.read('simulink/stateflow/chart_22.xml'))
        script = chart.find(".//P[@Name='script']").text
        source = Path(__file__).with_name('identify_simple.m').read_text()
        assert script == source.replace('identify_simple(', 'fcn(', 1)
        old_script = E.fromstring(old.read('simulink/stateflow/chart_22.xml')).find(".//P[@Name='script']").text
        old_source = Path(__file__).parent.joinpath('tests/fixtures/identify_vbemf_v13.m').read_text()
        assert old_script == old_source.replace('identify_vbemf_v13(', 'fcn(', 1)
        data = chart.findall('Children/data')
        inputs = [item.get('name') for item in data if field(item, 'scope') == 'INPUT_DATA']
        outputs = [item for item in data if field(item, 'scope') == 'OUTPUT_DATA']
        assert inputs == INPUTS
        assert [item.get('name') for item in outputs] == [item[0] for item in OUTPUTS]
        assert all(item.find("props/array/P[@Name='size']").text == '1' for item in outputs)
        assert field(chart, 'sampleTime') == '1e-4'

    graph = ResolvedGraph(MODEL.read_bytes())
    old_graph = ResolvedGraph(BASE.read_bytes())
    assert not (set(graph.blocks) - set(old_graph.blocks)), '不应新增控制或电气模块'
    removed = set(old_graph.blocks) - set(graph.blocks)
    assert len(removed) == 105, len(removed)
    # 只有辨识器、日志和其显示面板删块，其他所有系统完整保留。
    assert all(old_graph.owners[sid] in {'system_12300', 'system_12900',
                                      'system_12910', 'system_12088'} for sid in removed)
    for sid, item in graph.blocks.items():
        if item.get('BlockType') == 'From':
            graph.goto_for(sid)
    _, count = check_visible_graphs(graph)

    # 对不可见的MATLAB Function底层封装同样检查全部端口与正交连线。
    used = Counter(graph.inputs.values())
    wrapper = graph.systems['system_12088']
    geometry = Geometry(wrapper)
    for item in wrapper.findall('Block'):
        sid = item.get('SID')
        ni, no = port_counts(item)
        for port in range(1, ni+1):
            assert f'{sid}#in:{port}' in graph.inputs, (sid, port)
        for port in range(1, no+1):
            assert used[f'{sid}#out:{port}'], (sid, port)
    for wire in wrapper.findall('Line'):
        for source, destination, points in geometry.polylines(wire):
            assert is_orthogonal(points)
            excluded = {source.split('#')[0], destination.split('#')[0]}
            assert not any(crosses_rectangle(a, b, geometry.position(sid))
                           for sid in geometry.blocks if sid not in excluded
                           for a, b in zip(points, points[1:])), (source, destination)

    assert graph.blocks['12088'].find('PortCounts').attrib == {'in': '6', 'out': '12'}
    assert graph.blocks['12088::25'].find('PortCounts').attrib == {'in': '6', 'out': '13'}
    for index, (name, _, old_port) in enumerate(OUTPUTS, 1):
        sid = f'12088::{42+old_port}'
        assert graph.blocks[sid].get('Name') == name
        assert field(graph.blocks[sid], 'Port') == str(index)
        assert graph.inputs[sid+'#in:1'] == f'12088::25#out:{index+1}'
        assert graph.inputs[f'{12929+old_port}#in:1'] == f'12088#out:{index}'
        log_from = next(s for s, b in graph.blocks.items() if graph.owners[s] == 'system_12910'
                        and b.get('BlockType') == 'From' and b.get('Name') == name)
        assert graph.inputs[f'13050#in:{index}'] == log_from+'#out:1'
        assert graph.origin(log_from+'#out:1') == f'12088::25#out:{index+1}'
    assert graph.inputs['12115#in:1'] == '13050#out:1'
    assert field(graph.blocks['13050'], 'Inputs') == '12'
    for ref, system in graph.systems.items():
        ids = {item.get('SID') for item in system.findall('Block')}
        assert len(ids) == len(system.findall('Block'))
        for endpoint in system.findall("Line//P[@Name='Src']") + system.findall("Line//P[@Name='Dst']"):
            assert endpoint.text.split('#')[0] in ids

    check_control(MODEL.read_bytes())
    check_speed(MODEL.read_bytes())
    print('PASS 嵌入脚本与源文件一致；Stateflow、Function、S-Function和日志的六入十二出映射一致')
    print(f'PASS 删除{len(removed)}个诊断/历史端口及辅助块；{count}条可见信号线和底层封装端口完整')
    print('PASS 可见图及底层封装无离线发现的斜线、悬空活动端口或连线穿模块')
    print('PASS 只改辨识的5个内部文件，I/F、PI、PLL、电机、电气连接和求解器等均逐字节保留')
    print('PASS Src先于Points，HMI/记录路径有效；这是离线结构检查，尚未执行MATLAB加载/编译')


if __name__ == '__main__':
    check()
