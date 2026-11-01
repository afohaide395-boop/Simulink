"""修复上传模型的角度监测；只替换运行监测XML，不改控制和辨识。"""
from copy import deepcopy
from pathlib import Path
from zipfile import ZipFile
import argparse
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from model_cleanup.build_model import prop

MONITOR = 'simulink/systems/system_13240.xml'


def repair(source, destination):
    # 已有整理版提供完整、独立的三种角度与两种误差接线。
    with ZipFile(ROOT / 'reduce_emf_tidy.slx') as template:
        monitor = ET.fromstring(template.read(MONITOR))
    with ZipFile(source) as archive:
        entries = archive.infolist()
        data = {entry.filename: archive.read(entry.filename) for entry in entries}
    uploaded = ET.fromstring(data[MONITOR])
    # 保留用户保存的电机、参数示波器设置；角度Scope恢复全部通道可见。
    for sid in ['11355', '12097']:
        old = uploaded.find(f"Block[@SID='{sid}']")
        new = monitor.find(f"Block[@SID='{sid}']")
        config = old.find("P[@Name='ScopeSpecificationString']")
        prop(new, 'ScopeSpecificationString', config.text)
    prop(monitor, 'Open', 'off')
    data[MONITOR] = ET.tostring(monitor, encoding='utf-8', xml_declaration=True)
    with ZipFile(destination, 'w') as archive:
        for entry in entries:
            archive.writestr(deepcopy(entry), data[entry.filename])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    repair(args.source, args.destination)
