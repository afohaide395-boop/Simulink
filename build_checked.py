"""生成分组模型并修正Src/Points加载顺序及监测路径；不改动计算算法。"""
from pathlib import Path
from zipfile import ZipFile
from io import BytesIO
import json
import xml.etree.ElementTree as E

from build_readable import block, line, prop, relationships, blocks_before_lines
from model_geometry import Geometry, field, encode_points, crosses_rectangle

HERE = Path(__file__).resolve().parent
TARGETS = ['root', '13210', '13220', '13230', '13240', '12300', '12900', '12910']


def route(source, destination, geometry):
    start = geometry.endpoint(source)
    end = geometry.endpoint(destination)
    if start[1] == end[1]:
        return [start, ((start[0] + end[0]) / 2, start[1]), end]
    if start[0] == end[0]:
        return [start, (start[0], (start[1] + end[1]) / 2), end]
    for fraction in [0.5, 0.25, 0.75, 0.1, 0.9]:
        middle_x = start[0] + fraction * (end[0] - start[0])
        candidate = [start, (middle_x, start[1]), (middle_x, end[1]), end]
        excluded = {source.split('#')[0], destination.split('#')[0]}
        if not any(crosses_rectangle(a, b, geometry.position(sid))
                   for sid in geometry.blocks if sid not in excluded
                   for a, b in zip(candidate, candidate[1:])):
            return candidate
    raise AssertionError(('无法找到不穿过其他模块的正交路线', source, destination))


def build():
    with ZipFile(HERE / 'tests/fixtures/reduce_emf_organized_v11.slx') as archive:
        entries = archive.infolist()
        data = {item.filename: archive.read(item.filename) for item in entries}

    def read_system(sid):
        return E.fromstring(data[f'simulink/systems/system_{sid}.xml'])

    def save(key, element):
        data[key] = E.tostring(element, encoding='utf-8', xml_declaration=True)

    root = read_system('root')
    monitors = read_system('13240')
    startup = read_system('13210')
    parameter = read_system('12300')

    # A formerly unconnected torque input means zero in Simulink. Make that value explicit.
    # Connecting the unused 8.5-Nm step here would change the operating point.
    block(root, 13401, 'Ground', '负载转矩为零', '[735, 340, 755, 360]', ports={'in': 0, 'out': 1})
    line(root, '13401#out:1', '10661#in:1', '负载转矩_Tm_0')

    # Keep inactive legacy material in a separate, commented, zero-port container.
    backup = E.Element('System')
    prop(backup, 'Location', '[0, 0, 1100, 600]')
    legacy = monitors.find("Block[@SID='11364']")
    monitors.remove(legacy)
    prop(legacy, 'Position', '[420, 80, 660, 240]')
    backup.append(legacy)
    step = root.find("Block[@SID='11363']")
    root.remove(step)
    prop(step, 'Position', '[100, 350, 200, 390]')
    backup.append(step)
    for sid, center, port in [(13402, 120, 1), (13403, 200, 2)]:
        block(backup, sid, 'Ground', f'备用输入{port}', f'[280, {center-10}, 300, {center+10}]', ports={'in': 0, 'out': 1})
        line(backup, f'{sid}#out:1', f'11364#in:{port}')
    for sid, center, port in [(13404, 120, 1), (13405, 200, 2)]:
        block(backup, sid, 'Terminator', f'备用输出{port}', f'[800, {center-10}, 820, {center+10}]', ports={'in': 1, 'out': 0})
        line(backup, f'11364#out:{port}', f'{sid}#in:1')
    block(backup, 13406, 'Terminator', '未启用的备用负载', '[420, 360, 440, 380]', ports={'in': 1, 'out': 0})
    line(backup, '11363#out:1', '13406#in:1')
    container = block(root, 13400, 'SubSystem', '备用模型_已停用', '[1060, 1030, 1380, 1110]',
                      {'Commented': 'on'}, {'in': 0, 'out': 0})
    E.SubElement(container, 'System', {'Ref': 'system_13400'})
    prop(root, 'SIDHighWatermark', 13500)
    prop(root, 'Location', '[0, 0, 1500, 1180]')

    prop(startup.find("Block[@SID='12000']"), 'Position', '[100, 270, 220, 310]')
    prop(parameter.find("Block[@SID='12701']"), 'Position', '[100, 1478, 225, 1502]')
    systems = {'root': root, '13240': monitors, '13210': startup, '12300': parameter, '13400': backup}
    manifest = []
    for ref in TARGETS + ['13400']:
        system = systems.get(ref)
        if system is None:
            system = read_system(ref)
        for item in system.findall('Block'):
            if item.get('BlockType') not in ['From', 'Goto']:
                continue
            counts = item.find('PortCounts')
            if counts is None:
                counts = E.Element('PortCounts')
                item.insert(0, counts)
            counts.attrib = {'in': '1', 'out': '0'} if item.get('BlockType') == 'Goto' else {'in': '0', 'out': '1'}
        geometry = Geometry(system)
        for index, wire in enumerate(system.findall('Line'), 1):
            if wire.get('LineType') == 'Connection':
                continue
            assert not wire.findall('Branch'), ('目标图中的分支需单独保留', ref)
            source = field(wire, 'Src')
            destination = field(wire, 'Dst')
            assert source and destination
            points = route(source, destination, geometry)
            prop(wire, 'ZOrder', 200000 + index)
            prop(wire, 'Points', encode_points(points))
            # The loader needs Src before Points. Matching native order is essential:
            # Name / ZOrder / Src / Points / Dst. Points must never be moved to the front.
            zorder = wire.find("P[@Name='ZOrder']")
            wire.remove(zorder)
            source_item = wire.find("P[@Name='Src']")
            wire.insert(list(wire).index(source_item), zorder)
            points_item = wire.find("P[@Name='Points']")
            wire.remove(points_item)
            wire.insert(list(wire).index(source_item) + 1, points_item)
            if ref != '13400':
                source_sid, source_port = source.split('#out:')
                destination_sid, destination_port = destination.split('#in:')
                manifest.append([0 if ref == 'root' else int(ref), int(source_sid), int(source_port), int(destination_sid), int(destination_port)])
        systems[ref] = system
        save(f'simulink/systems/system_{ref}.xml', blocks_before_lines(system))

    for ref in ['root', '13240', '13400']:
        refs = [item.get('Ref').split('_', 1)[1] for item in systems[ref].findall('Block/System')]
        save(f'simulink/systems/_rels/system_{ref}.xml.rels', relationships(*refs))
    chart = E.fromstring(data['simulink/stateflow/chart_57.xml'])
    prop(chart, 'name', '备用模型_已停用/Subsystem/MATLAB Function1')
    save('simulink/stateflow/chart_57.xml', chart)
    machine = E.fromstring(data['simulink/stateflow/machine.xml'])
    prop(machine.find(".//instance[@id='64']"), 'name', '备用模型_已停用/Subsystem/MATLAB Function1')
    save('simulink/stateflow/machine.xml', machine)
    # Resolve current paths from the actual tree rather than retaining old top-level paths.
    staged = BytesIO()
    with ZipFile(staged, 'w') as archive:
        for item in entries:
            archive.writestr(item, data[item.filename])
        for name in set(data) - {item.filename for item in entries}:
            archive.writestr(name, data[name])
    from check_organized_model import ResolvedGraph
    graph = ResolvedGraph(staged.getvalue())
    diagram = E.fromstring(data['simulink/blockdiagram.xml'])
    old_paths = {}
    for record in diagram.findall(".//Object[@PropName='InstrumentedSignals']//MATStruct"):
        sid = record.find("Field[@Name='SID_']")
        path = record.find("Field[@Name='BlockPath_']")
        if sid is None or path is None:
            continue
        actual = graph.block_path(sid.text).replace('\n', ' ')
        old_paths[path.text] = actual
        path.text = actual
    save('simulink/blockdiagram.xml', diagram)
    interface = json.loads(data['simulink/graphicalInterface.json'])
    for record in interface['GraphicalInterface'].get('TestPointedSignals', []):
        old_path = record['FullBlockPath'].removeprefix('$bdroot/')
        if old_path in old_paths:
            record['FullBlockPath'] = '$bdroot/' + old_paths[old_path]
    data['simulink/graphicalInterface.json'] = json.dumps(interface, ensure_ascii=False, indent=4).encode()
    output = HERE / 'reduce_emf_load_fixed.slx'
    with ZipFile(output, 'w') as archive:
        for item in entries:
            archive.writestr(item, data.pop(item.filename))
        for name, content in data.items():
            archive.writestr(name, content)
    from audit_model_load_paths import audit
    assert not audit(output), '加载顺序或监测路径一致性检查失败'
    (HERE / 'reduce_emf_checked.slx').write_bytes(output.read_bytes())
    (HERE / 'organized_wiring_manifest.json').write_text(json.dumps({'format': 1, 'connections': manifest}, indent=2))
    print(output, f'：{len(manifest)}条显式信号线，附MATLAB原生端口重连清单。')


if __name__ == '__main__':
    build()
