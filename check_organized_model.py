"""展开Goto/From与虚拟层次，比较整理前后每个原计算端口的实际信号来源。"""
from collections import Counter
from copy import deepcopy
from pathlib import Path
from zipfile import ZipFile
from tempfile import TemporaryDirectory
import argparse
import json
import xml.etree.ElementTree as E

from check_control_path import SignalGraph
from check_power_connections import physical_lines, physical_nets

HERE = Path(__file__).resolve().parent
ROOT_KEY = 'simulink/systems/system_root.xml'
PLUMBING = {'Goto', 'From', 'SubSystem', 'Inport', 'Outport', 'GotoTagVisibility'}
VISUAL_PROPERTIES = {'Position', 'ZOrder', 'BlockMirror', 'BlockRotation', 'Location'}


def canonical(element):
    return E.canonicalize(E.tostring(element, encoding='unicode'), strip_text=True)


def parameters(element):
    element = deepcopy(element)
    for item in list(element.findall('P')):
        if item.get('Name') in VISUAL_PROPERTIES:
            element.remove(item)
    return canonical(element)


class ResolvedGraph(SignalGraph):
    def block_path(self, sid):
        names = [self.blocks[sid].get('Name')]
        while self.owners[sid] != 'system_root':
            sid = self.parents[self.owners[sid]]
            names.append(self.blocks[sid].get('Name'))
        return '/'.join(reversed(names))

    def goto_for(self, sid):
        item = self.blocks[sid]
        tag = self.prop(item, 'GotoTag')
        candidates = [other for other, block in self.blocks.items()
                      if block.get('BlockType') == 'Goto' and self.prop(block, 'GotoTag') == tag]
        local = [other for other in candidates if self.owners[other] == self.owners[sid]]
        if local:
            assert len(local) == 1, (sid, tag, local)
            return local[0]
        global_candidates = [other for other in candidates if self.prop(self.blocks[other], 'TagVisibility', 'local') == 'global']
        assert len(global_candidates) == 1, (sid, tag, global_candidates)
        return global_candidates[0]

    def origin(self, endpoint, active=None):
        active = set() if active is None else active
        assert endpoint not in active, '标签或层次出现组合自回接：' + endpoint
        active.add(endpoint)
        try:
            if '#in:' in endpoint:
                if endpoint not in self.inputs:
                    return 'UNCONNECTED:' + endpoint
                return self.origin(self.inputs[endpoint], active)
            sid, port = endpoint.split('#out:')
            item = self.blocks[sid]
            kind = item.get('BlockType')
            if kind == 'From':
                return self.origin(self.goto_for(sid) + '#in:1', active)
            if kind == 'Inport':
                parent = self.parents[self.owners[sid]]
                return self.origin(parent + '#in:' + self.prop(item, 'Port', '1'), active)
            if kind == 'SubSystem':
                child = self.systems[item.find('System').get('Ref')]
                outports = [block for block in child.findall('Block')
                            if block.get('BlockType') == 'Outport' and self.prop(block, 'Port', '1') == port]
                assert len(outports) == 1, endpoint
                return self.origin(outports[0].get('SID') + '#in:1', active)
            return endpoint
        finally:
            active.remove(endpoint)


def check(model, reference):
    before = ResolvedGraph(reference.read_bytes())
    after = ResolvedGraph(model.read_bytes())
    assert set(before.blocks) <= set(after.blocks), '原模块被删除'
    new_ids = set(after.blocks) - set(before.blocks)
    assert len(new_ids) == 24, len(new_ids)
    assert Counter(after.blocks[sid].get('BlockType') for sid in new_ids) == Counter({'Goto': 5, 'From': 15, 'SubSystem': 4})

    compared = 0
    for sid, item in before.blocks.items():
        kind = item.get('BlockType')
        if kind not in {'Goto', 'From'}:
            assert parameters(item) == parameters(after.blocks[sid]), ('计算参数变化', sid)
        # Every connected input of every original compute block must retain its leaf source.
        # Comparing all original From outputs also checks signals that are only monitored.
        if kind == 'From':
            assert before.origin(sid + '#out:1') == after.origin(sid + '#out:1'), ('标签解析变化', sid)
        if kind in PLUMBING:
            continue
        input_keys = [key for key in before.inputs if key.startswith(sid + '#in:')]
        for endpoint in input_keys:
            old_source = before.origin(endpoint)
            new_source = after.origin(endpoint)
            assert old_source == new_source, (endpoint, old_source, new_source)
            compared += 1
        assert {key for key in after.inputs if key.startswith(sid + '#in:')} == set(input_keys)

    for sid, item in after.blocks.items():
        if item.get('BlockType') == 'From':
            after.goto_for(sid)
        if sid in new_ids and item.get('BlockType') == 'SubSystem':
            assert item.find('PortCounts').attrib == {'in': '0', 'out': '0'}
            assert after.prop(item, 'TreatAsAtomicUnit', 'off') == 'off'
            assert item.find('System') is not None

    # The delay count, sampling, controller, solver and algorithm remain identical.
    old_root = before.systems['system_root']
    new_root = after.systems['system_root']
    assert Counter(map(canonical, physical_lines(old_root))) == Counter(map(canonical, physical_lines(new_root)))
    assert physical_nets(old_root) == physical_nets(new_root)
    for sid in ['13220', '13230']:
        expected = {'13220': {'88', '103', '11246', '11251', '283', '11256', '11845'},
                    '13230': {'11764', '11765', '12112', '11757', '12087'}}[sid]
        assert all(after.owners[item] == 'system_' + sid for item in expected)

    with ZipFile(reference) as old, ZipFile(model) as new:
        assert new.testzip() is None
        assert set(old.namelist()) <= set(new.namelist())
        expected_added = {f'simulink/systems/system_{sid}.xml' for sid in ['13210', '13220', '13230', '13240']}
        expected_added |= {name.replace('/system_', '/_rels/system_') + '.rels' for name in expected_added.copy()}
        assert set(new.namelist()) - set(old.namelist()) == expected_added
        for name in old.namelist():
            if name in [ROOT_KEY, 'simulink/systems/_rels/system_root.xml.rels',
                        'simulink/stateflow/machine.xml', 'simulink/graphicalInterface.json']:
                continue
            if name.startswith('simulink/stateflow/chart_') and name.endswith('.xml'):
                a, b = [E.fromstring(archive.read(name)) for archive in [old, new]]
                # Only chart paths change. The complete scripts and Stateflow settings must match.
                for root in [a, b]:
                    root.find("P[@Name='name']").text = ''
                assert canonical(a) == canonical(b), name
            else:
                assert old.read(name) == new.read(name), ('原文件内容变化', name)

        old_interface = json.loads(old.read('simulink/graphicalInterface.json'))
        new_interface = json.loads(new.read('simulink/graphicalInterface.json'))
        for reference_item in old_interface['GraphicalInterface']['ExternalFileReferences']:
            reference_item['Path'] = '$bdroot/' + after.block_path(reference_item['SID']).replace('\n', ' ')
        assert old_interface == new_interface

        all_sids = set()
        for name in new.namelist():
            if name.endswith(('.xml', '.rels')):
                root = E.fromstring(new.read(name))
            if not name.startswith('simulink/systems/system_') or not name.endswith('.xml'):
                continue
            blocks = {item.get('SID'): item for item in root.findall('Block')}
            assert len(blocks) == len(root.findall('Block'))
            assert not all_sids.intersection(blocks)
            all_sids.update(blocks)
            sources = set()
            for wire in root.findall('Line'):
                if wire.get('LineType') == 'Connection':
                    continue
                source = wire.find("P[@Name='Src']").text
                assert source not in sources, ('同一输出存在多个独立线树', name, source)
                sources.add(source)
                for endpoint in wire.findall(".//P[@Name='Src']") + wire.findall(".//P[@Name='Dst']"):
                    assert endpoint.text.split('#')[0] in blocks
            refs = [item.get('Ref') for item in root.findall('Block/System')]
            if refs:
                relations = E.fromstring(new.read(name.replace('/system_', '/_rels/system_') + '.rels'))
                targets = {item.get('Id'): item.get('Target') for item in relations}
                assert set(refs) == set(targets)
                for target in targets.values():
                    assert 'simulink/systems/' + target in new.namelist()

        machine = E.fromstring(new.read('simulink/stateflow/machine.xml'))
        for instance in machine.findall('.//instance'):
            chart_id = after.prop(instance, 'chart')
            chart = E.fromstring(new.read('simulink/stateflow/chart_' + chart_id + '.xml'))
            assert after.prop(instance, 'name') == after.prop(chart, 'name')
            path = after.prop(instance, 'name')
            names = []
            system = after.systems['system_root']
            while True:
                # Root names such as I/F启动 contain slash; match the full block name first.
                found = [item for item in system.findall('Block')
                         if path == item.get('Name') or path.startswith(item.get('Name') + '/')]
                assert len(found) == 1, (chart_id, path)
                item = found[0]
                names.append(item.get('Name'))
                if path == item.get('Name'):
                    break
                path = path[len(item.get('Name')) + 1:]
                system = after.systems[item.find('System').get('Ref')]

    print(f'PASS {compared}个原计算输入逐路展开Goto/From及子系统后，信号来源与v10完全一致')
    print('PASS 所有原计算参数、采样周期、延迟、控制器和辨识代码保留；新增层次均为虚拟子系统')
    print('PASS 五组原物理电气连接树及直流负端回路保留；PWM、角度反馈与辨识确认未改时序')
    print('PASS Stateflow路径、SID、模型端点、标签作用范围及子系统关系完整')
    print('当前环境无MATLAB/Simulink；上述检查不替代SLX实际编译和电机整机仿真。')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=Path, default=HERE / 'reduce_emf_organized.slx')
    parser.add_argument('--reference', type=Path, default=HERE / 'tests/fixtures/reduce_emf_readable_v10.slx')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    check(args.model, args.reference)
    if args.self_test:
        mutations = [
            ('绕过观测器角度反馈延迟', 'simulink/systems/system_13230.xml',
             None, '11765#out:1'),
            ('PWM误接到转速信号', ROOT_KEY, "Block[@SID='5']/P[@Name='GotoTag']", 'we'),
            ('跨模块状态标签未导出', 'simulink/systems/system_13230.xml',
             "Block[@SID='12094']/P[@Name='TagVisibility']", 'local'),
        ]
        with ZipFile(args.model) as original:
            entries = original.infolist()
            original_data = {item.filename: original.read(item.filename) for item in entries}
        with TemporaryDirectory(prefix='organized_model_') as folder:
            for index, (name, key, query, value) in enumerate(mutations):
                root = E.fromstring(original_data[key])
                if index == 0:
                    target = next(wire.find("P[@Name='Src']") for wire in root.findall('Line')
                                  if any(item.text == '11764#in:6' for item in wire.findall(".//P[@Name='Dst']")))
                else:
                    target = root.find(query)
                assert target is not None
                target.text = value
                broken = Path(folder) / f'broken_{index}.slx'
                with ZipFile(broken, 'w') as archive:
                    for item in entries:
                        archive.writestr(item, E.tostring(root) if item.filename == key else original_data[item.filename])
                try:
                    check(broken, args.reference)
                except AssertionError:
                    print('PASS 故障回归能检出：' + name)
                else:
                    raise AssertionError('未能检出人为接线错误：' + name)


if __name__ == '__main__':
    main()
