"""回归用户报告崩溃的v12：仅离线检查，不加载到MATLAB。"""
from pathlib import Path
from zipfile import ZipFile
import json
import xml.etree.ElementTree as E

from audit_model_load_paths import audit
from build_checked import TARGETS
from check_organized_model import ResolvedGraph

HERE = Path(__file__).resolve().parent


def main():
    broken = HERE / 'tests/fixtures/reduce_emf_checked_v12_crash.slx'
    fixed = HERE / 'reduce_emf_load_fixed.slx'
    errors = audit(broken)
    assert sum('连线属性顺序错误' in error for error in errors) == 305
    assert sum('HMI SID=' in error for error in errors) == 2
    assert sum('记录信号路径不存在' in error for error in errors) == 2
    assert not audit(fixed)
    for name in ['reduce_emf_recovery_v8.slx', 'tests/fixtures/reduce_emf_speedloop_v8.slx',
                 'tests/fixtures/reduce_emf_readable_v10.slx']:
        assert not audit(HERE / name), name
    assert (HERE / 'reduce_emf_recovery_v8.slx').read_bytes() == (HERE / 'tests/fixtures/reduce_emf_speedloop_v8.slx').read_bytes()

    before = ResolvedGraph(broken.read_bytes())
    after = ResolvedGraph(fixed.read_bytes())
    assert before.inputs == after.inputs
    assert before.owners == after.owners
    assert set(before.blocks) == set(after.blocks)
    for sid in before.blocks:
        assert E.canonicalize(E.tostring(before.blocks[sid], encoding='unicode'), strip_text=True) == E.canonicalize(E.tostring(after.blocks[sid], encoding='unicode'), strip_text=True), sid

    with ZipFile(broken) as old, ZipFile(fixed) as new:
        assert old.namelist() == new.namelist()
        changed = {name for name in old.namelist() if old.read(name) != new.read(name)}
        changed_systems = {f'simulink/systems/system_{ref}.xml' for ref in TARGETS + ['13400']}
        assert changed == changed_systems | {'simulink/blockdiagram.xml', 'simulink/graphicalInterface.json'}, changed
        for name in changed_systems:
            a, b = [E.fromstring(archive.read(name)) for archive in [old, new]]
            # Values, endpoints and relative offsets remain unchanged; only P-tag order changes.
            for root in [a, b]:
                for wire in root.findall('Line'):
                    properties = wire.findall('P')
                    for item in properties:
                        wire.remove(item)
                    for item in sorted(properties, key=lambda item: (item.get('Name'), item.text or '')):
                        wire.insert(len(wire), item)
            assert E.canonicalize(E.tostring(a, encoding='unicode'), strip_text=True) == E.canonicalize(E.tostring(b, encoding='unicode'), strip_text=True), name
        diagrams = [E.fromstring(archive.read('simulink/blockdiagram.xml')) for archive in [old, new]]
        for diagram in diagrams:
            for item in diagram.findall(".//Object[@PropName='InstrumentedSignals']//Field[@Name='BlockPath_']"):
                item.text = ''
        assert E.canonicalize(E.tostring(diagrams[0], encoding='unicode'), strip_text=True) == E.canonicalize(E.tostring(diagrams[1], encoding='unicode'), strip_text=True)
        interfaces = [json.loads(archive.read('simulink/graphicalInterface.json')) for archive in [old, new]]
        for interface in interfaces:
            for item in interface['GraphicalInterface']['TestPointedSignals']:
                item['FullBlockPath'] = ''
        assert interfaces[0] == interfaces[1]

    print('PASS 已报告崩溃的v12被拒绝：305处Points/Src倒序、2处HMI路径及2处记录路径不一致')
    print('PASS 新版上述项目全部通过；原v8、v10的加载顺序及路径对照通过')
    print('PASS 只更改9个图的连线P标签顺序及2个监测元数据文件；块、接线、参数、Stateflow代码完全保留')
    print('PASS 恢复版与v8备份逐字节一致')
    print('MATLAB实际打开、编译、仿真尚未执行；不以离线通过替代运行验证。')


if __name__ == '__main__':
    main()
