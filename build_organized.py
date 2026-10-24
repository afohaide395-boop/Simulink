"""把v10现有计算块整理为虚拟子系统；只调整层次、标签和几何布局。"""
from collections import defaultdict
from copy import deepcopy
from pathlib import Path
from zipfile import ZipFile
import ast
import json
import xml.etree.ElementTree as E

from build_readable import block, line, prop, relationships, blocks_before_lines

HERE = Path(__file__).resolve().parent
ROOT_KEY = 'simulink/systems/system_root.xml'
GROUPS = {
    '13210': '启动与速度控制',
    '13220': '电流采样与矢量控制',
    '13230': '反电动势观测与锁相环',
    '13240': '运行监测',
}


def positions():
    """按信号处理顺序分行摆放，坐标只决定显示，不参与计算。"""
    startup = {
        9640: (100, 80, 180, 120), 11362: (280, 80, 360, 120), 2570: (460, 88, 660, 112),
        12000: (100, 260, 220, 300), 12075: (320, 200, 480, 380),
        12030: (650, 218, 860, 242), 12029: (650, 278, 860, 302), 12019: (650, 338, 860, 362),
        2577: (500, 528, 660, 552), 11242: (700, 510, 820, 630),
        10489: (60, 548, 260, 572), 12024: (60, 588, 260, 612), 12013: (60, 628, 260, 652),
        12012: (320, 540, 440, 660), 12550: (900, 558, 1100, 582),
        12551: (60, 818, 260, 842), 12420: (60, 878, 260, 902), 12400: (400, 800, 620, 920),
        12010: (900, 840, 1040, 960), 12023: (680, 888, 860, 912),
        12008: (680, 928, 860, 952), 2546: (1160, 888, 1360, 912),
        12074: (60, 1108, 260, 1132), 12110: (400, 1105, 460, 1135),
        12026: (480, 1148, 660, 1172), 12027: (480, 1188, 660, 1212),
        12009: (700, 1100, 840, 1220), 11849: (1000, 1148, 1220, 1172),
    }
    current = {
        10651: (300, 80, 305, 440), 10660: (500, 80, 505, 260),
        11845: (660, 150, 740, 190), 10656: (820, 158, 1020, 182),
        11846: (500, 270, 580, 310), 10653: (1100, 270, 1180, 310),
        10652: (1100, 430, 1180, 470), 10655: (1260, 278, 1460, 302),
        10657: (1260, 438, 1460, 462), 10654: (500, 330, 580, 370),
        11847: (660, 330, 740, 370), 10659: (820, 338, 1020, 362),
        11848: (500, 390, 580, 430), 10658: (660, 398, 860, 422),
        84: (60, 638, 260, 662), 88: (320, 610, 440, 690),
        10827: (500, 618, 700, 642), 10828: (500, 658, 700, 682),
        103: (1100, 590, 1220, 710), 2616: (860, 678, 1060, 702),
        2651: (1300, 608, 1500, 632), 2620: (1300, 668, 1500, 692),
        9709: (60, 923, 240, 947), 12085: (60, 953, 240, 977),
        12086: (320, 920, 380, 980), 11844: (500, 930, 580, 970),
        2561: (680, 938, 840, 962), 2565: (860, 933, 1060, 957),
        2655: (860, 983, 1060, 1007), 11251: (1100, 920, 1220, 1020),
        2542: (1300, 958, 1500, 982), 2552: (60, 1118, 260, 1142),
        2562: (320, 1118, 520, 1142), 2566: (860, 1113, 1060, 1137),
        2645: (860, 1163, 1060, 1187), 11246: (1100, 1100, 1220, 1200),
        2554: (1300, 1138, 1500, 1162),
        2617: (60, 1358, 260, 1382), 2649: (60, 1418, 260, 1442),
        2656: (60, 1478, 260, 1502), 283: (320, 1340, 480, 1520),
        2037: (600, 1373, 800, 1397), 2038: (600, 1463, 800, 1487),
        2658: (860, 1373, 1060, 1397), 2659: (860, 1463, 1060, 1487),
        11256: (1100, 1340, 1240, 1520), 184: (1320, 1418, 1520, 1442),
        12099: (1100, 1590, 1240, 1670),
    }
    observer = {
        11476: (60, 168, 240, 192), 11478: (60, 228, 240, 252),
        11477: (60, 288, 240, 312), 11479: (60, 348, 240, 372),
        11481: (20, 408, 200, 432), 12108: (260, 400, 340, 440),
        12112: (260, 465, 340, 495), 11764: (400, 150, 620, 510),
        11765: (1100, 150, 1300, 510), 12083: (860, 438, 1060, 462),
        12035: (1420, 183, 1630, 207), 11757: (1420, 270, 1500, 300),
        11756: (1600, 273, 1820, 297), 12087: (1420, 360, 1500, 390),
        12084: (1600, 363, 1820, 387), 12094: (1420, 453, 1630, 477),
        12113: (700, 365, 720, 385), 12114: (700, 455, 720, 475),
        11763: (1100, 620, 1240, 740), 11863: (60, 828, 260, 852),
        11347: (400, 820, 480, 860), 10488: (600, 828, 820, 852),
    }
    monitors = {
        11352: (800, 118, 1000, 142), 11353: (800, 178, 1000, 202),
        11354: (800, 238, 1000, 262), 11355: (1100, 100, 1240, 280),
        12032: (60, 348, 260, 372), 12017: (60, 398, 260, 422),
        12016: (60, 448, 260, 472), 12018: (60, 498, 260, 522),
        12015: (400, 335, 405, 535), 12014: (600, 405, 740, 465),
        12033: (60, 638, 260, 662), 12105: (60, 698, 260, 722),
        12104: (400, 620, 500, 740), 12021: (1100, 620, 1105, 800),
        12034: (800, 698, 1000, 722), 12022: (800, 758, 1000, 782),
        12020: (1300, 670, 1440, 750), 12102: (1100, 880, 1200, 1000),
        12103: (1300, 910, 1440, 970),
        10697: (500, 1118, 700, 1142), 10695: (500, 1178, 700, 1202),
        10698: (800, 1100, 805, 1220), 10694: (500, 1288, 700, 1312),
        10699: (800, 1260, 805, 1420), 10693: (1200, 1070, 1340, 1430),
        10696: (60, 1338, 260, 1362), 11236: (60, 1398, 260, 1422),
        11235: (400, 1320, 500, 1440),
        10684: (60, 1628, 260, 1652), 10881: (60, 1688, 260, 1712),
        10880: (400, 1610, 500, 1730), 11234: (500, 1718, 700, 1742),
        10882: (800, 1640, 900, 1760), 10883: (1200, 1670, 1340, 1730),
        11364: (60, 1870, 300, 1990),
    }
    return {key: {str(sid): list(rect) for sid, rect in value.items()} for key, value in zip(GROUPS, [startup, current, observer, monitors])}


def field(element, name, default=None):
    item = element.find(f"P[@Name='{name}']")
    return item.text if item is not None else default


class Organizer:
    def __init__(self, root):
        self.root = root
        self.systems = {'root': root}
        self.blocks = {item.get('SID'): item for item in root.findall('Block')}
        self.owners = {sid: 'root' for sid in self.blocks}
        self.next_sid = 13300
        self.input_counts = defaultdict(int)
        self.output_counts = defaultdict(int)
        self.edges = defaultdict(list)
        self.names = {}
        self.reused_tags = {}
        self.generated_tags = {}
        for item in root.findall('Line'):
            if item.get('LineType') == 'Connection':
                continue
            source = field(item, 'Src')
            self.names[source] = field(item, 'Name')
            self.edges[source] += [destination.text for destination in item.findall(".//P[@Name='Dst']")]
            sid, port = source.split('#out:')
            self.output_counts[sid] = max(self.output_counts[sid], int(port))
            for destination in self.edges[source]:
                sid, port = destination.split('#in:')
                self.input_counts[sid] = max(self.input_counts[sid], int(port))

    def new_sid(self):
        sid = str(self.next_sid)
        self.next_sid += 1
        return sid

    def endpoint(self, endpoint):
        sid, direction = endpoint.split('#')
        side, index = direction.split(':')
        index = int(index)
        item = self.blocks[sid]
        left, top, right, bottom = ast.literal_eval(field(item, 'Position'))
        counts = item.find('PortCounts')
        count = int(counts.get(side, '0')) if counts is not None else 0
        count = max(count, (self.output_counts if side == 'out' else self.input_counts)[sid], 1)
        mirrored = field(item, 'BlockMirror') == 'on'
        x = right if (side == 'out') != mirrored else left
        return x, top + (index - 0.5) * (bottom - top) / count

    def add(self, owner, kind, name, rect, properties):
        sid = self.new_sid()
        item = block(self.systems[owner], sid, kind, name, str(rect), properties)
        self.blocks[sid] = item
        self.owners[sid] = owner
        return sid

    def wire(self, source, destination):
        owner = self.owners[source.split('#')[0]]
        assert owner == self.owners[destination.split('#')[0]]
        line(self.systems[owner], source, destination, self.names.get(source))

    def tag_name(self, source):
        readable = {
            '10661#out:1': 'Motor_measurements',
            '11846#out:1': 'Sampled_omega_m',
            '11764#out:1': 'Observer_Ed_hat',
            '11764#out:2': 'Observer_Eq_hat',
            '12104#out:1': 'Monitor_theta_m_mod',
        }
        return 'LINK_' + readable.get(source, source.replace('#out:', '_port_'))

    def goto_for(self, source):
        if source in self.reused_tags:
            return self.reused_tags[source]
        if source in self.generated_tags:
            return self.generated_tags[source]
        item = self.blocks[source.split('#')[0]]
        # A From signal is already a named route; reuse that tag rather than nesting tags.
        if item.get('BlockType') == 'From':
            return field(item, 'GotoTag')
        x, y = self.endpoint(source)
        owner = self.owners[item.get('SID')]
        tag = self.tag_name(source)
        visibility = 'global' if any(self.owners[d.split('#')[0]] != owner for d in self.edges[source]) else 'local'
        sid = self.add(owner, 'Goto', tag, [x + 40, y - 12, x + 200, y + 12],
                       {'GotoTag': tag, 'TagVisibility': visibility})
        self.wire(source, sid + '#in:1')
        self.generated_tags[source] = tag
        return tag

    def from_to(self, tag, destination):
        x, y = self.endpoint(destination)
        owner = self.owners[destination.split('#')[0]]
        sid = self.add(owner, 'From', tag, [x - 260, y - 12, x - 40, y + 12], {'GotoTag': tag})
        self.wire(sid + '#out:1', destination)

    def regroup(self):
        layouts = positions()
        assigned = set()
        for owner, coordinates in layouts.items():
            system = E.Element('System')
            prop(system, 'Location', '[0, 0, 2000, 2200]')
            self.systems[owner] = system
            for sid, rect in coordinates.items():
                assert sid not in assigned
                assigned.add(sid)
                item = self.blocks[sid]
                self.root.remove(item)
                system.append(item)
                self.owners[sid] = owner
                prop(item, 'Position', str(rect))
                mirror = item.find("P[@Name='BlockMirror']")
                if mirror is not None:
                    item.remove(mirror)

        expected_remaining = {'1', '3', '5', '7', '6198', '10661', '11363', '12300',
                              '12090', '12089', '12091', '12092', '12093', '12095', '12109', '12101'}
        assert set(self.blocks) - assigned == expected_remaining, set(self.blocks) - assigned
        # Original top-level local tags were visible only among top-level blocks.
        # Export just those original pairs using a unique prefix; inner local tags are untouched.
        local_tags = {}
        for item in self.blocks.values():
            if item.get('BlockType') == 'Goto' and field(item, 'TagVisibility', 'local') == 'local':
                old_tag = field(item, 'GotoTag')
                new_tag = 'CTRL_' + old_tag
                local_tags[old_tag] = new_tag
                prop(item, 'GotoTag', new_tag)
                prop(item, 'TagVisibility', 'global')
        for item in self.blocks.values():
            if item.get('BlockType') == 'From' and field(item, 'GotoTag') in local_tags:
                prop(item, 'GotoTag', local_tags[field(item, 'GotoTag')])

        # Translate the physical plant as one unit, retaining every physical branch unchanged.
        for sid in ['3', '5', '7', '6198', '10661', '11363']:
            item = self.blocks[sid]
            x0, y0, x1, y1 = ast.literal_eval(field(item, 'Position'))
            prop(item, 'Position', str([x0 + 1480, y0 + 330, x1 + 1480, y1 + 330]))
        root_positions = {
            '1': [1120, 480, 1201, 519], '12300': [560, 600, 880, 1020],
            '12090': [60, 623, 290, 647], '12089': [60, 693, 290, 717],
            '12091': [60, 763, 290, 787], '12092': [60, 833, 290, 857],
            '12093': [60, 903, 290, 927], '12095': [60, 973, 290, 997],
            '12109': [340, 900, 460, 930], '12101': [340, 970, 460, 1000],
        }
        for sid, rect in root_positions.items():
            prop(self.blocks[sid], 'Position', str(rect))

        # Rebuild signal lines using the original endpoint pairs; physical lines are retained.
        for item in list(self.root.findall('Line')):
            if item.get('LineType') != 'Connection':
                self.root.remove(item)
        for source, destinations in self.edges.items():
            for destination in destinations:
                item = self.blocks[destination.split('#')[0]]
                if item.get('BlockType') == 'Goto':
                    self.reused_tags[source] = field(item, 'GotoTag')

        # Install original Goto drivers first, then connect or route the other destinations.
        for source, destinations in self.edges.items():
            for destination in destinations:
                item = self.blocks[destination.split('#')[0]]
                if item.get('BlockType') == 'Goto':
                    self.wire(source, destination)
        for source, destinations in self.edges.items():
            active = [d for d in destinations if self.blocks[d.split('#')[0]].get('BlockType') != 'Goto']
            same_owner = lambda d: self.owners[source.split('#')[0]] == self.owners[d.split('#')[0]]
            source_is_from = self.blocks[source.split('#')[0]].get('BlockType') == 'From'
            nearest = None
            if source_is_from and active:
                sx, sy = self.endpoint(source)
                nearest = min(active, key=lambda d: abs(self.endpoint(d)[1] - sy) + abs(self.endpoint(d)[0] - sx))
            for destination in active:
                sx, sy = self.endpoint(source)
                dx, dy = self.endpoint(destination)
                # Short left-to-right connections stay explicit. Fanouts and return paths use tags.
                route = (not same_owner(destination) or source in self.reused_tags
                         or len(active) > 1 or abs(dy - sy) > 10 or dx - sx > 600 or dx < sx)
                if destination == nearest and same_owner(destination) and abs(dy - sy) <= 10:
                    route = False
                if source == '5#out:1' and destination == '7#in:1':
                    # The SPS bridge positions its gate input on its custom icon.
                    # Retain the original plant-side pulse wire instead of guessing that location.
                    route = False
                if route:
                    self.from_to(self.goto_for(source), destination)
                else:
                    self.wire(source, destination)

        for annotation in list(self.root.findall('Annotation')):
            self.root.remove(annotation)
        cards = {
            '13210': [100, 100, 380, 300], '13220': [560, 100, 880, 300],
            '13230': [1060, 100, 1380, 300], '13240': [1060, 700, 1380, 900],
        }
        for sid, name in GROUPS.items():
            item = block(self.root, sid, 'SubSystem', name, str(cards[sid]), ports={'in': 0, 'out': 0})
            E.SubElement(item, 'System', {'Ref': f'system_{sid}'})
            prop(item, 'Description', '仅整理原计算块与信号标签；虚拟子系统，不增加采样、延迟或执行条件。')
        headings = {
            'root': [(60, 25, '无感控制系统：启动、矢量控制、观测与参数辨识'),
                     (500, 345, '逆变器与电机')],
            '13210': [(60, 20, '转速参考'), (60, 155, 'I/F启动与无感切换状态'),
                      (60, 455, '速度闭环'), (60, 745, 'q轴电流指令选择'),
                      (60, 1045, '控制角度选择')],
            '13220': [(60, 20, '电机测量与电流采样'), (60, 535, 'Clarke/Park变换'),
                      (60, 855, 'd/q轴电流环'), (60, 1275, '逆Park变换与SVPWM')],
            '13230': [(60, 65, '反电动势观测器与锁相环'),
                      (60, 555, '角度反馈保留原一拍延迟；Ed/Eq通过同名标签送入PLL'),
                      (60, 775, '估计转速单位转换')],
            '13240': [(60, 20, '三相电流、转矩、转速与切换过程监测')],
        }
        for owner, system in self.systems.items():
            for item in system.findall('Block'):
                if item.get('BlockType') in ['Goto', 'From']:
                    prop(item, 'ShowName', 'off')
            for x, y, text in headings[owner]:
                annotation = E.SubElement(system, 'Annotation', {'SID': self.new_sid()})
                prop(annotation, 'Name', text)
                prop(annotation, 'Position', str([x, y, x + len(text) * 15, y + 24]))
                prop(annotation, 'FontSize', 14)
                prop(annotation, 'ZOrder', -1)
        prop(self.root, 'Location', '[0, 0, 1500, 1120]')
        prop(self.root, 'SIDHighWatermark', self.next_sid + 10)
        for owner, system in self.systems.items():
            blocks_before_lines(system)
        return self.systems


def build():
    with ZipFile(HERE / 'tests/fixtures/reduce_emf_readable_v10.slx') as archive:
        entries = archive.infolist()
        data = {entry.filename: archive.read(entry.filename) for entry in entries}
    original_root = E.fromstring(data[ROOT_KEY])
    organizer = Organizer(original_root)
    systems = organizer.regroup()

    def save(key, element):
        data[key] = E.tostring(element, encoding='utf-8', xml_declaration=True)

    for owner, system in systems.items():
        key = ROOT_KEY if owner == 'root' else f'simulink/systems/system_{owner}.xml'
        save(key, system)
        refs = [item.get('Ref').split('_', 1)[1] for item in system.findall('Block/System')]
        if refs:
            save(key.replace('/system_', '/_rels/system_') + '.rels', relationships(*refs))

    old_paths = {}
    with ZipFile(HERE / 'tests/fixtures/reduce_emf_readable_v10.slx') as archive:
        root = E.fromstring(archive.read(ROOT_KEY))
        for item in root.findall('Block'):
            owner = organizer.owners[item.get('SID')]
            if owner != 'root':
                old_paths[item.get('Name')] = GROUPS[owner] + '/' + item.get('Name')

    def moved_path(path):
        for original, changed in sorted(old_paths.items(), key=lambda item: -len(item[0])):
            if path == original or path.startswith(original + '/'):
                return changed + path[len(original):]
        return path

    machine = E.fromstring(data['simulink/stateflow/machine.xml'])
    for instance in machine.findall('.//instance'):
        name = field(instance, 'name')
        changed = moved_path(name)
        if changed == name:
            continue
        prop(instance, 'name', changed)
        chart_key = 'simulink/stateflow/chart_' + field(instance, 'chart') + '.xml'
        chart = E.fromstring(data[chart_key])
        prop(chart, 'name', moved_path(field(chart, 'name')))
        save(chart_key, chart)
    save('simulink/stateflow/machine.xml', machine)
    interface_key = 'simulink/graphicalInterface.json'
    interface = json.loads(data[interface_key])
    for reference in interface['GraphicalInterface']['ExternalFileReferences']:
        path = reference['Path']
        if path.startswith('$bdroot/'):
            reference['Path'] = '$bdroot/' + moved_path(path[len('$bdroot/'):])
    data[interface_key] = json.dumps(interface, ensure_ascii=False, indent=4).encode()
    output = HERE / 'reduce_emf_organized.slx'
    with ZipFile(output, 'w') as archive:
        for entry in entries:
            archive.writestr(entry, data.pop(entry.filename))
        for name, content in data.items():
            archive.writestr(name, content)
    print(output)


if __name__ == '__main__':
    build()
