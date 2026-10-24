"""严格检查活动图的每个端口、显式线段及来源；不将悬空输入当作通过。"""
from collections import Counter
from pathlib import Path
from tempfile import TemporaryDirectory
from zipfile import ZipFile
import argparse
import json
import xml.etree.ElementTree as E

from build_checked import TARGETS
from check_organized_model import ResolvedGraph, parameters, canonical
from check_power_connections import physical_lines, physical_nets
from model_geometry import Geometry, field, is_orthogonal, crosses_rectangle
from audit_model_load_paths import audit

HERE = Path(__file__).resolve().parent


def port_counts(item):
    specified = item.find('PortCounts')
    if specified is not None:
        return int(specified.get('in', '0')), int(specified.get('out', '0'))
    kind = item.get('BlockType')
    fixed = {'From': (0, 1), 'Goto': (1, 0), 'Inport': (0, 1), 'Outport': (1, 0),
             'Ground': (0, 1), 'Constant': (0, 1),
             'Step': (0, 1), 'Gain': (1, 1), 'UnitDelay': (1, 1), 'ZeroOrderHold': (1, 1),
             'DiscreteTransferFcn': (1, 1), 'DataTypeConversion': (1, 1), 'Fcn': (1, 1),
             'Display': (1, 0), 'ToWorkspace': (1, 0), 'Terminator': (1, 0), 'Switch': (3, 1)}
    if kind in fixed:
        return fixed[kind]
    if kind == 'Sum':
        inputs = field(item, 'Inputs', '++').replace('|', '')
        return (int(inputs) if inputs.isdigit() else len(inputs)), 1
    if kind == 'Math':
        return (2 if field(item, 'Operator') == 'mod' else 1), 1
    if kind == 'Mux':
        return int(field(item, 'Inputs')), 1
    if kind == 'Scope':
        return int(field(item, 'NumInputPorts', '1')), 0
    if kind == 'Reference' and item.get('SID') == '1':
        return 0, 0
    raise AssertionError(('端口数不可只按已连线推测', item.get('SID'), kind))


def normal_origin(graph, endpoint):
    source = graph.origin(endpoint)
    if source.startswith('UNCONNECTED:'):
        return 'ZERO'
    sid = source.split('#')[0]
    if graph.blocks[sid].get('BlockType') == 'Ground':
        return 'ZERO'
    return source


def check_visible_graphs(graph, check_rectangles=True):
    outputs_used = Counter(graph.inputs.values())
    connections = []
    line_count = 0
    for ref in TARGETS:
        system = graph.systems['system_' + ref]
        geometry = Geometry(system)
        rectangles = {sid: geometry.position(sid) for sid in geometry.blocks}
        for sid, item in geometry.blocks.items():
            if field(item, 'Commented', 'off') == 'on':
                continue
            inputs, outputs = port_counts(item)
            for port in range(1, inputs + 1):
                endpoint = f'{sid}#in:{port}'
                assert endpoint in graph.inputs, ('活动输入端口悬空', ref, endpoint)
                assert not graph.origin(endpoint).startswith('UNCONNECTED:'), ('上游悬空', endpoint)
            for port in range(1, outputs + 1):
                assert outputs_used[f'{sid}#out:{port}'], ('活动输出端口悬空', ref, sid, port)
        if check_rectangles:
            items = list(rectangles.items())
            for index, (sid, a) in enumerate(items):
                for other, b in items[index + 1:]:
                    assert not (max(a[0], b[0]) < min(a[2], b[2]) and max(a[1], b[1]) < min(a[3], b[3])), ('模块重叠', ref, sid, other)
        sources = set()
        for wire in system.findall('Line'):
            if wire.get('LineType') == 'Connection':
                continue
            assert field(wire, 'Src') and field(wire, 'Dst'), ('缺少线端点', ref)
            assert field(wire, 'Src') not in sources, ('重复线树', ref, field(wire, 'Src'))
            sources.add(field(wire, 'Src'))
            assert wire.find("P[@Name='Points']") is not None, ('缺少显式折点', ref)
            for source, destination, points in geometry.polylines(wire):
                assert is_orthogonal(points), ('斜线', ref, source, destination, points)
                excluded = {source.split('#')[0], destination.split('#')[0]}
                if check_rectangles:
                    for sid, rectangle in rectangles.items():
                        if sid in excluded:
                            continue
                        assert not any(crosses_rectangle(a, b, rectangle) for a, b in zip(points, points[1:])), ('连线穿模块', ref, source, destination, sid)
                source_sid, source_port = source.split('#out:')
                destination_sid, destination_port = destination.split('#in:')
                connections.append([0 if ref == 'root' else int(ref), int(source_sid), int(source_port), int(destination_sid), int(destination_port)])
                line_count += 1
    return connections, line_count


def check(model, reference):
    load_errors = audit(model)
    assert not load_errors, ('加载元数据一致性失败', load_errors[:3])
    before = ResolvedGraph(reference.read_bytes())
    after = ResolvedGraph(model.read_bytes())
    assert set(before.blocks) <= set(after.blocks)
    assert set(after.blocks) - set(before.blocks) == {str(number) for number in range(13400, 13407)}
    compared = 0
    for sid, item in before.blocks.items():
        if item.get('BlockType') not in {'Goto', 'From'}:
            assert parameters(item) == parameters(after.blocks[sid]), ('计算参数变化', sid)
        if item.get('BlockType') == 'From':
            assert normal_origin(before, sid + '#out:1') == normal_origin(after, sid + '#out:1'), ('原标签来源变化', sid)
        for endpoint in [key for key in before.inputs if key.startswith(sid + '#in:')]:
            assert normal_origin(before, endpoint) == normal_origin(after, endpoint), ('原信号来源变化', endpoint)
            compared += 1
    assert after.inputs['10661#in:1'] == '13401#out:1'
    assert after.blocks['13401'].get('BlockType') == 'Ground'
    assert field(after.blocks['13400'], 'Commented') == 'on'
    assert after.blocks['13400'].find('PortCounts').attrib == {'in': '0', 'out': '0'}
    assert after.owners['11364'] == 'system_13400' and after.owners['11363'] == 'system_13400'

    manifest, line_count = check_visible_graphs(after)
    assert manifest == json.loads((HERE / 'organized_wiring_manifest.json').read_text())['connections']
    for sid, item in after.blocks.items():
        if item.get('BlockType') == 'From':
            after.goto_for(sid)
    roots = [graph.systems['system_root'] for graph in [before, after]]
    assert Counter(map(canonical, physical_lines(roots[0]))) == Counter(map(canonical, physical_lines(roots[1])))
    assert physical_nets(roots[0]) == physical_nets(roots[1])

    with ZipFile(reference) as old, ZipFile(model) as new:
        assert new.testzip() is None
        for name in old.namelist():
            if name.startswith('simulink/stateflow/chart_') and name.endswith('.xml'):
                a, b = [E.fromstring(archive.read(name)) for archive in [old, new]]
                for chart in [a, b]:
                    chart.find("P[@Name='name']").text = ''
                assert canonical(a) == canonical(b), ('算法代码或Stateflow配置变化', name)
        ids = set()
        for name in new.namelist():
            if name.endswith(('.xml', '.rels')):
                E.fromstring(new.read(name))
            if name.startswith('simulink/systems/system_') and name.endswith('.xml'):
                root = E.fromstring(new.read(name))
                blocks = root.findall('Block')
                own_ids = {item.get('SID') for item in blocks}
                assert len(own_ids) == len(blocks) and not ids.intersection(own_ids)
                ids.update(own_ids)
                refs = {item.get('Ref') for item in root.findall('Block/System')}
                rel_key = name.replace('/system_', '/_rels/system_') + '.rels'
                if refs or rel_key in new.namelist():
                    relations = E.fromstring(new.read(rel_key))
                    targets = {item.get('Id'): item.get('Target') for item in relations}
                    assert refs == set(targets)
                    for target in targets.values():
                        assert 'simulink/systems/' + target in new.namelist()
                for wire in root.findall('Line'):
                    for endpoint in wire.findall(".//P[@Name='Src']") + wire.findall(".//P[@Name='Dst']"):
                        assert endpoint.text.split('#')[0] in own_ids

    print(f'PASS 活动的8个可见图：{line_count}条显式信号线，所有活动输入/输出端口均已连接')
    print('PASS 所检图中无斜线、模块重叠或信号线穿模块；Goto/From唯一可解析')
    print(f'PASS {compared}个原输入的来源保留；原悬空负载输入显式接零，备用模块隔离停用')
    print('PASS 电气回路、所有算法及Stateflow设置保留；XML/SID/子系统关系有效')
    print('PASS 连线Src先于Points；HMI及记录信号路径与移动后的模块一致')
    print('上述为离线检查；MATLAB脚本另按真实PortHandles重连、检查并编译，云端尚未执行。')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=Path, default=HERE / 'reduce_emf_load_fixed.slx')
    parser.add_argument('--reference', type=Path, default=HERE / 'tests/fixtures/reduce_emf_organized_v11.slx')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    check(args.model, args.reference)
    if args.self_test:
        with ZipFile(args.model) as archive:
            entries = archive.infolist()
            original = {item.filename: archive.read(item.filename) for item in entries}
        key = 'simulink/systems/system_13220.xml'
        with TemporaryDirectory(prefix='checked_wiring_') as folder:
            for mode in ['missing', 'diagonal']:
                root = E.fromstring(original[key])
                wire = next(item for item in root.findall('Line') if field(item, 'Dst') == '103#in:1')
                if mode == 'missing':
                    root.remove(wire)
                else:
                    wire.find("P[@Name='Points']").text = '[10, 4]'
                broken = Path(folder) / f'{mode}.slx'
                with ZipFile(broken, 'w') as archive:
                    for item in entries:
                        archive.writestr(item, E.tostring(root) if item.filename == key else original[item.filename])
                try:
                    check_visible_graphs(ResolvedGraph(broken.read_bytes()))
                except AssertionError as error:
                    assert ('端口悬空' in str(error)) if mode == 'missing' else ('斜线' in str(error))
                    print('PASS 故障回归检出：' + ('Park输入断线' if mode == 'missing' else '人为插入斜线'))
                else:
                    raise AssertionError('未能检出故障：' + mode)


if __name__ == '__main__':
    main()
