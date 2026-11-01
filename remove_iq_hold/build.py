"""删除iq保持及速度PI暂停支路，保留原I/F切换与其余模型。"""
from copy import deepcopy
from pathlib import Path
from zipfile import ZipFile
import argparse
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from build_checked import route
from model_cleanup.build_model import prop, remove_blocks, offsets
from model_geometry import Geometry, field


def build(source, destination):
    with ZipFile(source) as archive:
        entries = archive.infolist()
        data = {entry.filename: archive.read(entry.filename) for entry in entries}

    def read(name):
        return ET.fromstring(data[name])

    def save(name, root):
        data[name] = ET.tostring(root, encoding='utf-8', xml_declaration=True)

    def connect(system, source, destination):
        # Src必须在Points之前，避免R2024b连线解析异常。
        path = route(source, destination, Geometry(system))
        wire = ET.SubElement(system, 'Line')
        prop(wire, 'Src', source)
        prop(wire, 'Points', offsets(path))
        prop(wire, 'Dst', destination)

    startup_name = 'simulink/systems/system_13210.xml'
    startup = read(startup_name)
    remove_blocks(startup, {'12400', '12420', '12550', '12551'})
    connect(startup, '11242#out:1', '12010#in:1')
    save(startup_name, startup)

    speed_name = 'simulink/systems/system_11242.xml'
    speed = read(speed_name)
    remove_blocks(speed, {'12240', '12241', '12242'})
    connect(speed, '9701#out:1', '4007#in:1')
    save(speed_name, speed)

    # 同步SLX包内引用与已删子系统的编辑器恢复记录。
    rel_name = 'simulink/systems/_rels/system_13210.xml.rels'
    rels = read(rel_name)
    for rel in list(rels):
        if rel.get('Id') == 'system_12400':
            rels.remove(rel)
    save(rel_name, rels)
    del data['simulink/systems/system_12400.xml']
    windows_name = 'simulink/windowsInfo.xml'
    windows = read(windows_name)
    changed = False
    for editors in windows.findall(".//Array[@PropName='EditorsInfo']"):
        for editor in list(editors):
            if field(editor, 'LoadSaveID') == '12400':
                editors.remove(editor)
                changed = True
        editors.set('Dimension', '1*'+str(len(editors)))
    if changed:
        save(windows_name, windows)

    with ZipFile(destination, 'w') as archive:
        for entry in entries:
            if entry.filename in data:
                archive.writestr(deepcopy(entry), data[entry.filename])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    build(args.source, args.destination)
