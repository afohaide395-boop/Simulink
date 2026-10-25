"""在v13加载修正版上精简辨识；其他模型部件保持原字节。"""
from copy import deepcopy
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as E

from build_readable import prop, block, line, blocks_before_lines
from build_checked import route
from model_geometry import Geometry, field

HERE = Path(__file__).resolve().parent
BASE = HERE / 'tests/fixtures/reduce_emf_load_fixed_v13.slx'
MODEL = HERE / 'reduce_emf_simple.slx'
INPUTS = ['ud', 'uq', 'id', 'iq', 'we', 'stage']
# 保留原物理量的SID，只缩短名字并删除已不使用的输出。
OUTPUTS = [
    ('Ls', 'Ls_hat', 1), ('psi', 'psi_f_hat', 2), ('Rs', 'Rs_hat', 3),
    ('da', 'delta_theta_A', 4), ('db', 'delta_theta_B', 5),
    ('doneL', 'Lpsi_done', 6), ('doneR', 'Rs_done', 7),
    ('run', 'ident_status', 8), ('n', 'rls_updates', 12),
    ('ack', 'sample_ack', 16), ('R0', 'Rs_cal', 28), ('V0', 'V_inverter_cal', 29),
]
PARTS = {
    'simulink/stateflow/chart_22.xml',
    'simulink/systems/system_12300.xml',
    'simulink/systems/system_12900.xml',
    'simulink/systems/system_12910.xml',
    'simulink/systems/system_12088.xml',
}


def output_names(element, offset=0):
    previous = element.find('PortProperties')
    if previous is not None:
        element.remove(previous)
    ports = E.SubElement(element, 'PortProperties')
    for index, (name, _, _) in enumerate(OUTPUTS, 1):
        port = E.SubElement(ports, 'Port', Type='out', Index=str(index+offset))
        prop(port, 'Name' if offset else 'PropagatedSignals', name)


def route_lines(system):
    """保留Src在Points之前；不可再将Points移至Src前面。"""
    geometry = Geometry(system)
    for index, wire in enumerate(system.findall('Line'), 1):
        assert not wire.findall('Branch')
        source = field(wire, 'Src')
        destination = field(wire, 'Dst')
        name = field(wire, 'Name')
        path = route(source, destination, geometry)
        # 底层S-Function端口坐标可能含循环小数，不能截成六位有效数字。
        offsets = [(b[0]-a[0], b[1]-a[1]) for a, b in zip(path, path[1:-1])]
        points = '[' + '; '.join(f'{dx:.15g}, {dy:.15g}' for dx, dy in offsets) + ']'
        wire.clear()
        if name:
            prop(wire, 'Name', name)
        prop(wire, 'ZOrder', 200000+index)
        prop(wire, 'Src', source)
        prop(wire, 'Points', points)
        prop(wire, 'Dst', destination)
    return blocks_before_lines(system)


def build():
    with ZipFile(BASE) as archive:
        entries = archive.infolist()
        data = {item.filename: archive.read(item.filename) for item in entries}

    def system(sid):
        return E.fromstring(data[f'simulink/systems/system_{sid}.xml'])

    def save(key, root):
        assert key in PARTS
        data[key] = E.tostring(root, encoding='utf-8', xml_declaration=True)

    # Stateflow只修改辨识器脚本和六入十二出数据定义。
    chart = E.fromstring(data['simulink/stateflow/chart_22.xml'])
    script = (HERE / 'identify_simple.m').read_text().replace('identify_simple(', 'fcn(', 1)
    chart.find(".//P[@Name='script']").text = script
    children = chart.find('Children')
    inputs = [item for item in children.findall('data') if field(item, 'scope') == 'INPUT_DATA']
    outputs = {item.get('name'): item for item in children.findall('data') if field(item, 'scope') == 'OUTPUT_DATA'}
    for item, name in zip(inputs, INPUTS):
        item.set('name', name)
    for item in outputs.values():
        children.remove(item)
    for name, old_name, _ in OUTPUTS:
        item = outputs[old_name]
        item.set('name', name)
        assert item.find("props/array/P[@Name='size']").text == '1'
        children.append(item)
    save('simulink/stateflow/chart_22.xml', chart)

    # MATLAB Function的底层封装必须同步端口数、端口名及连接。
    wrapper = system('12088')
    keep = {'12088::25', '12088::26', '12088::27'}
    keep.update(item.get('SID') for item in wrapper.findall('Block') if item.get('BlockType') == 'Inport')
    keep.update(f'12088::{42+old_port}' for _, _, old_port in OUTPUTS)
    for item in list(wrapper):
        if item.tag == 'Line' or (item.tag == 'Block' and item.get('SID') not in keep):
            wrapper.remove(item)
    prop(wrapper, 'Location', '[0, 0, 1100, 650]')
    for item, name in zip([b for b in wrapper.findall('Block') if b.get('BlockType') == 'Inport'], INPUTS):
        port = int(field(item, 'Port', '1'))
        center = 40+(port-0.5)*80
        item.set('Name', name)
        prop(item, 'Position', f'[60, {center-9:g}, 80, {center+9:g}]')
        line(wrapper, item.get('SID')+'#out:1', f'12088::25#in:{port}')
    native = wrapper.find("Block[@SID='12088::25']")
    native.find('PortCounts').set('out', '13')
    prop(native, 'PortCounts', '[6 13]')
    # 第一输出是Stateflow隐式激活端口，继续接原Demux和Terminator。
    prop(native, 'Position', '[380, 20, 700, 540]')
    output_names(native, offset=1)
    prop(wrapper.find("Block[@SID='12088::26']"), 'Position', '[780, 30, 785, 50]')
    prop(wrapper.find("Block[@SID='12088::27']"), 'Position', '[900, 30, 920, 50]')
    line(wrapper, '12088::25#out:1', '12088::26#in:1')
    line(wrapper, '12088::26#out:1', '12088::27#in:1')
    for index, (name, _, old_port) in enumerate(OUTPUTS, 1):
        item = wrapper.find(f"Block[@SID='12088::{42+old_port}']")
        item.set('Name', name)
        prop(item, 'Port', index)
        center = 40+index*40
        prop(item, 'Position', f'[900, {center-9}, 920, {center+9}]')
        line(wrapper, f'12088::25#out:{index+1}', item.get('SID')+'#in:1', name)
    save('simulink/systems/system_12088.xml', route_lines(wrapper))

    algorithm = system('12900')
    keep = {'12088'} | {str(12900+i) for i in range(1, 7)}
    keep.update(str(12929+port) for _, _, port in OUTPUTS)
    for item in list(algorithm):
        if item.tag == 'Line' or (item.tag == 'Block' and item.get('SID') not in keep):
            algorithm.remove(item)
    prop(algorithm, 'Location', '[0, 0, 1200, 600]')
    function = algorithm.find("Block[@SID='12088']")
    function.find('PortCounts').set('out', '12')
    prop(function, 'Position', '[380, 40, 700, 520]')
    prop(function, 'Description', '采样存储→正负注入求角度→逐样本RLS→电阻计算；中文注释，12路独立短名称输出。')
    output_names(function)
    for index, name in enumerate(INPUTS, 1):
        item = algorithm.find(f"Block[@SID='{12900+index}']")
        item.set('Name', name)
        center = 40+(index-0.5)*80
        prop(item, 'Position', f'[100, {center-9:g}, 120, {center+9:g}]')
        line(algorithm, item.get('SID')+'#out:1', f'12088#in:{index}', name)
    for index, (name, _, old_port) in enumerate(OUTPUTS, 1):
        item = algorithm.find(f"Block[@SID='{12929+old_port}']")
        item.set('Name', name)
        prop(item, 'GotoTag', 'PID_'+name)
        center = 40+(index-0.5)*40
        prop(item, 'Position', f'[860, {center-12:g}, 1040, {center+12:g}]')
        line(algorithm, f'12088#out:{index}', item.get('SID')+'#in:1', name)
    save('simulink/systems/system_12900.xml', route_lines(algorithm))

    # 日志只保存12个真实输出，删除旧诊断分组，不再生成31列兼容占位。
    old_record = system('12910')
    record = E.Element('System')
    prop(record, 'Location', '[0, 0, 1100, 600]')
    for index, (name, old_name, _) in enumerate(OUTPUTS, 1):
        item = deepcopy(next(b for b in old_record.findall('Block')
                             if b.get('BlockType') == 'From' and b.get('Name') == old_name))
        item.set('Name', name)
        prop(item, 'GotoTag', 'PID_'+name)
        center = 40+(index-0.5)*40
        prop(item, 'Position', f'[60, {center-12:g}, 240, {center+12:g}]')
        record.append(item)
        line(record, item.get('SID')+'#out:1', f'13050#in:{index}')
    block(record, 13050, 'Mux', '结果记录_12列', '[420, 40, 425, 520]',
          {'Inputs': 12}, {'in': 12, 'out': 1})
    logger = deepcopy(old_record.find("Block[@SID='12115']"))
    prop(logger, 'Position', '[700, 260, 960, 300]')
    record.append(logger)
    line(record, '13050#out:1', '12115#in:1')
    save('simulink/systems/system_12910.xml', route_lines(record))

    parent = system('12300')
    removed = {'12368', '12369', '12106', '12801', '12802', '12920',
               '12562', '12367', '12226'} | {str(sid) for sid in range(12701, 12708)}
    for wire in list(parent.findall('Line')):
        endpoints = wire.findall(".//P[@Name='Src']") + wire.findall(".//P[@Name='Dst']")
        if any(item.text.split('#')[0] in removed for item in endpoints):
            parent.remove(wire)
    tags = {'PID_'+old: 'PID_'+new for new, old, _ in OUTPUTS}
    names = {old: new for new, old, _ in OUTPUTS}
    for item in list(parent.findall('Block')):
        if item.get('SID') in removed:
            parent.remove(item)
            continue
        tag = field(item, 'GotoTag')
        if tag in tags:
            prop(item, 'GotoTag', tags[tag])
            item.set('Name', names.get(item.get('Name'), item.get('Name')))
    prop(parent, 'Location', '[0, 0, 2340, 1320]')
    prop(parent.find("Block[@SID='12910']"), 'Description',
         '记录顺序：[Ls psi Rs da db doneL doneR run n ack R0 V0]。')
    parent.find("Block[@SID='12097']").set('Name', '电感与磁链')
    parent.find("Block[@SID='12098']").set('Name', '两组位置误差')
    parent.find("Block[@SID='12118']").set('Name', '完成标志与电阻')
    save('simulink/systems/system_12300.xml', route_lines(parent))

    with ZipFile(MODEL, 'w') as archive:
        for item in entries:
            archive.writestr(item, data[item.filename])
    with ZipFile(BASE) as old, ZipFile(MODEL) as new:
        assert old.namelist() == new.namelist()
        assert {name for name in old.namelist() if old.read(name) != new.read(name)} == PARTS
    from audit_model_load_paths import audit
    assert not audit(MODEL)
    print(f'{MODEL.name}：只改辨识的5个内部文件，12路独立输出；其他部件字节不变。')


if __name__ == '__main__':
    build()
