"""核对v10命名接口、日志兼容及重构范围；不能替代MATLAB编译。"""
from pathlib import Path
from zipfile import ZipFile
import argparse
import xml.etree.ElementTree as E

from check_control_path import SignalGraph
from identification_interface import INPUTS, OUTPUTS, LOG_COLUMNS

HERE = Path(__file__).resolve().parent


def prop(element, name):
    return element.find(f"P[@Name='{name}']").text


def canonical(element):
    return E.canonicalize(E.tostring(element, encoding='unicode'), strip_text=True)


def check(model, reference):
    content = model.read_bytes()
    graph = SignalGraph(content)
    with ZipFile(reference) as old, ZipFile(model) as new:
        assert new.testzip() is None
        added = set(new.namelist()) - set(old.namelist())
        assert added == {
            'simulink/systems/system_12900.xml',
            'simulink/systems/system_12910.xml',
            'simulink/systems/_rels/system_12900.xml.rels',
        }
        assert set(old.namelist()) <= set(new.namelist())
        changed = {name for name in old.namelist() if old.read(name) != new.read(name)}
        assert changed == {
            'simulink/stateflow/chart_22.xml', 'simulink/stateflow/machine.xml',
            'simulink/systems/system_12300.xml', 'simulink/systems/system_12088.xml',
            'simulink/systems/_rels/system_12300.xml.rels', 'simulink/systems/system_root.xml',
        }, changed

        # 根图只提高SID水位，电机、供电及所有控制接线保持逐项一致。
        roots = [E.fromstring(archive.read('simulink/systems/system_root.xml')) for archive in [old, new]]
        for root in roots:
            root.find("P[@Name='SIDHighWatermark']").text = ''
        assert canonical(roots[0]) == canonical(roots[1])
        assert old.read('simulink/stateflow/chart_65.xml') == new.read('simulink/stateflow/chart_65.xml')

        all_sids = set()
        for name in new.namelist():
            if name.endswith(('.xml', '.rels')):
                root = E.fromstring(new.read(name))
            if name.startswith('simulink/systems/system_') and name.endswith('.xml'):
                blocks = {block.get('SID'): block for block in root.findall('Block')}
                assert len(blocks) == len(root.findall('Block'))
                assert not all_sids.intersection(blocks)
                all_sids.update(blocks)
                drivers = set()
                for line in root.findall('Line'):
                    source = line.find("P[@Name='Src']")
                    if source is not None:
                        assert source.text.split('#')[0] in blocks
                    for destination in line.findall(".//P[@Name='Dst']"):
                        assert destination.text.split('#')[0] in blocks
                        assert destination.text not in drivers
                        drivers.add(destination.text)
                # 每个外置子系统都必须有对应的OPC关系。
                refs = [item.get('Ref') for item in root.findall('Block/System')]
                if refs:
                    relations_name = name.replace('/system_', '/_rels/system_') + '.rels'
                    relations = E.fromstring(new.read(relations_name))
                    targets = {item.get('Id'): item.get('Target') for item in relations}
                    assert set(refs) == set(targets)
                    for ref in refs:
                        assert 'simulink/systems/' + targets[ref] in new.namelist()

        chart = E.fromstring(new.read('simulink/stateflow/chart_22.xml'))
        assert chart.find(".//P[@Name='script']").text == (HERE / 'identify_vbemf.m').read_text().replace('identify_vbemf(', 'fcn(', 1)
        data = chart.findall('Children/data')
        assert len({item.get('SSID') for item in data}) == len(data)
        assert [item.get('name') for item in data if prop(item, 'scope') == 'INPUT_DATA'] == INPUTS
        output_data = [item for item in data if prop(item, 'scope') == 'OUTPUT_DATA']
        assert [item.get('name') for item in output_data] == OUTPUTS
        for item in output_data:
            expected_size = '400' if item.get('name').endswith('_history') else '1'
            assert item.find("props/array/P[@Name='size']").text == expected_size
        assert prop(chart, 'ssIdHighWaterMark') == '75'
        path = '参数辨识与采集/参数辨识算法/参数辨识'
        assert prop(chart, 'name') == path
        machine = E.fromstring(new.read('simulink/stateflow/machine.xml'))
        assert prop(machine.find(".//instance[@id='35']"), 'name') == path
        old_machine = E.fromstring(old.read('simulink/stateflow/machine.xml'))
        for root in [old_machine, machine]:
            root.find(".//instance[@id='35']/P[@Name='name']").text = ''
        assert canonical(old_machine) == canonical(machine)
        old_chart = E.fromstring(old.read('simulink/stateflow/chart_22.xml'))
        for root in [old_chart, chart]:
            root.find(".//P[@Name='script']").text = ''
            for name in ['name', 'ssIdHighWaterMark']:
                root.find(f"P[@Name='{name}']").text = ''
            children = root.find('Children')
            input_data = []
            for item in list(children.findall('data')):
                if prop(item, 'scope') == 'OUTPUT_DATA':
                    children.remove(item)
                else:
                    input_data.append(item)
            for item, name in zip(input_data, INPUTS):
                item.set('name', name)
        assert canonical(old_chart) == canonical(chart)

        wrapper = graph.systems['system_12088']
        sfunction = graph.blocks['12088::25']
        assert sfunction.find('PortCounts').attrib == {'in': '6', 'out': '34'}
        assert prop(sfunction, 'PortCounts') == '[6 34]'
        assert graph.blocks['12088'].find('PortCounts').attrib == {'in': '6', 'out': '33'}
        for index, name in enumerate(OUTPUTS, 1):
            port = next(item for item in wrapper.findall('Block') if item.get('BlockType') == 'Outport' and prop(item, 'Port') == str(index))
            assert port.get('Name') == name
            assert graph.inputs[port.get('SID') + '#in:1'] == f'12088::25#out:{index + 1}'
            assert graph.inputs[f'{12929 + index}#in:1'] == f'12088#out:{index}'
            assert prop(graph.blocks[str(12929 + index)], 'GotoTag') == 'PID_' + name
            meta = graph.blocks['12088'].find(f"PortProperties/Port[@Index='{index}']")
            assert prop(meta, 'PropagatedSignals') == name
        for index, name in enumerate(INPUTS, 1):
            sid = str(12900 + index)
            assert graph.blocks[sid].get('Name') == name
            assert graph.inputs[f'12088#in:{index}'] == sid + '#out:1'

        # 辨识支路的From标签唯一可解析，尤其是跨新层级的命名输出。
        for sid, block in graph.blocks.items():
            if block.get('BlockType') == 'From' and prop(block, 'GotoTag').startswith('PID_'):
                graph.goto_for(sid)

        # 逐列追踪日志及原参数示波器，防止输出重排后显示错量。
        for index, name in enumerate(LOG_COLUMNS, 1):
            source = graph.inputs[f'13050#in:{index}'].split('#')[0]
            assert prop(graph.blocks[source], 'GotoTag') == 'PID_' + name
            output_tag = graph.goto_for(source)
            assert graph.inputs[output_tag + '#in:1'] == f'12088#out:{OUTPUTS.index(name) + 1}'
        displays = {'12362': 'Ls_hat', '12363': 'psi_f_hat', '12365': 'Rs_hat',
                    '12360': 'delta_theta_A', '12361': 'delta_theta_B',
                    '12565': 'ident_status', '12569': 'rls_updates', '12343': 'sample_ack'}
        for sid, name in displays.items():
            assert prop(graph.blocks[sid], 'GotoTag') == 'PID_' + name
        assert graph.inputs['12115#in:1'] == '13050#out:1'
        assert prop(graph.blocks['12115'], 'VariableName') == 'param_identification'
        before_workspace = E.fromstring(old.read('simulink/systems/system_12300.xml')).find("Block[@SID='12115']")
        after_workspace = E.fromstring(E.tostring(graph.blocks['12115']))
        for item in [before_workspace, after_workspace]:
            item.find("P[@Name='Position']").text = ''
        assert canonical(before_workspace) == canonical(after_workspace)

        # 五路滤波及原始信号标签不改参数，也不改其接线。
        before_parent = E.fromstring(old.read('simulink/systems/system_12300.xml'))
        for sid in [str(12200 + index) for index in range(5)] + [str(12310 + index) for index in range(6)]:
            assert canonical(before_parent.find(f"Block[@SID='{sid}']")) == canonical(graph.blocks[sid])

    print('PASS v9控制环、PLL/I/F、所有电气连接、滤波及求解器保留；仅重构辨识代码与接口')
    print('PASS 6输入、31标量及2组400点角度历史：源码、Stateflow及底层端口元数据一致')
    print('PASS 独立命名输出、参数显示、31列兼容日志及辨识Goto/From映射正确')
    print('PASS SLX/XML完整、SID唯一、端点有效、驱动唯一、子系统关系完整')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=Path, default=HERE / 'reduce_emf_readable.slx')
    parser.add_argument('--reference', type=Path, default=HERE / 'tests/fixtures/reduce_emf_accuracy_v9.slx')
    args = parser.parse_args()
    check(args.model, args.reference)


if __name__ == '__main__':
    main()
