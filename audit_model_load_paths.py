"""检查连线加载顺序及HMI/记录路径；通过此检查不等于MATLAB加载成功。"""
from pathlib import Path
from zipfile import ZipFile
import argparse
import json
import xml.etree.ElementTree as E

from check_organized_model import ResolvedGraph


def normalize(path):
    return (path or '').replace('\n', ' ').removeprefix('$bdroot/')


def audit(model):
    graph = ResolvedGraph(model.read_bytes())
    actual_paths = {normalize(graph.block_path(sid)): sid for sid in graph.blocks}
    errors = []
    with ZipFile(model) as archive:
        for name in archive.namelist():
            if not name.startswith('simulink/systems/system_') or not name.endswith('.xml'):
                continue
            system = E.fromstring(archive.read(name))
            for wire in system.findall('.//Line') + system.findall('.//Branch'):
                source = wire.find("P[@Name='Src']")
                points = wire.find("P[@Name='Points']")
                if source is not None and points is not None:
                    children = list(wire)
                    if children.index(points) < children.index(source):
                        errors.append(f'连线属性顺序错误：{name}，{source.text}的Points位于Src之前')
                if wire.tag == 'Line' and source is None:
                    errors.append('线树缺少源端点：' + name)
        root = E.fromstring(archive.read('simulink/blockdiagram.xml'))
        for record in root.findall(".//Object[@PropName='InstrumentedSignals']//MATStruct"):
            sid = record.find("Field[@Name='SID_']")
            path = record.find("Field[@Name='BlockPath_']")
            if sid is None or path is None:
                continue
            if sid.text not in graph.blocks:
                errors.append(f'HMI记录SID不存在：{sid.text}')
                continue
            expected = normalize(graph.block_path(sid.text))
            if normalize(path.text) != expected:
                errors.append(f'HMI SID={sid.text}，保存路径={path.text}，实际路径={expected}')
        interface = json.loads(archive.read('simulink/graphicalInterface.json'))['GraphicalInterface']
        for record in interface.get('TestPointedSignals', []):
            path = normalize(record['FullBlockPath'])
            if path not in actual_paths:
                errors.append('记录信号路径不存在：' + record['FullBlockPath'])
        for record in interface.get('ExternalFileReferences', []):
            sid = record['SID']
            expected = normalize(graph.block_path(sid))
            if normalize(record['Path']) != expected:
                errors.append(f'库引用SID={sid}，保存路径={record["Path"]}，实际路径={expected}')
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('model', type=Path)
    args = parser.parse_args()
    errors = audit(args.model)
    if errors:
        for error in errors[:10]:
            print('FAIL ' + error)
        print(f'共{len(errors)}项；最多显示前10项。')
        raise SystemExit(1)
    print('PASS 连线Src先于Points；HMI/记录信号/库引用路径与当前层次一致。')
    print('此项为离线一致性检查，不能证明MATLAB加载、编译或仿真通过。')


if __name__ == '__main__':
    main()
