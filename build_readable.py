"""基于v9快照重构辨识接口和版式，保留电机、控制环、采样与数值算法。"""
from copy import deepcopy
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as E

from identification_interface import INPUTS, OUTPUTS, LOG_COLUMNS

HERE = Path(__file__).resolve().parent
REL_NS = 'http://schemas.openxmlformats.org/package/2006/relationships'
SYSTEM_REL = 'http://schemas.mathworks.com/simulink/2010/relationships/system'


def prop(element, name, value):
    item = element.find(f"P[@Name='{name}']")
    if item is None:
        item = E.SubElement(element, 'P', {'Name': name})
    item.text = str(value)


def block(system, sid, kind, name, position, properties=None, ports=None):
    element = E.Element('Block', {'BlockType': kind, 'SID': str(sid), 'Name': name})
    if ports is not None:
        E.SubElement(element, 'PortCounts', {key: str(value) for key, value in ports.items()})
    prop(element, 'Position', position)
    prop(element, 'ZOrder', 100004)
    for key, value in (properties or {}).items():
        prop(element, key, value)
    index = next((i for i, child in enumerate(system) if child.tag == 'Line'), len(system))
    system.insert(index, element)
    return element


def line(system, source, destination, name=None):
    element = E.SubElement(system, 'Line')
    if name:
        prop(element, 'Name', name)
    prop(element, 'Src', source)
    prop(element, 'Dst', destination)


def system_reference(system, sid, name, position, inputs):
    element = block(system, sid, 'SubSystem', name, position, ports={'in': inputs, 'out': 0})
    E.SubElement(element, 'System', {'Ref': f'system_{sid}'})
    return element


def relationships(*sids):
    root = E.Element('Relationships', {'xmlns': REL_NS})
    for sid in sids:
        E.SubElement(root, 'Relationship', {
            'Id': f'system_{sid}', 'Target': f'system_{sid}.xml', 'Type': SYSTEM_REL})
    return root


def blocks_before_lines(system):
    for element in list(system.findall('Line')):
        system.remove(element)
        system.append(element)
    return system


def output_properties(element, offset=0):
    previous = element.find('PortProperties')
    if previous is not None:
        element.remove(previous)
    ports = E.SubElement(element, 'PortProperties')
    for index, name in enumerate(OUTPUTS, 1):
        port = E.SubElement(ports, 'Port', {'Type': 'out', 'Index': str(index + offset)})
        prop(port, 'Name' if offset else 'PropagatedSignals', name)


def function_wrapper(original):
    """同步MATLAB Function的底层端口封装，包括隐式激活端口。"""
    wrapper = E.Element('System')
    prop(wrapper, 'Location', '[0, 0, 1200, 1480]')
    prop(wrapper, 'SIDHighWatermark', 75)
    inport_ids = [1, 28, 30, 40, 41, 42]
    for index, (name, sid) in enumerate(zip(INPUTS, inport_ids), 1):
        center = 150 + (index - 1) * 220
        block(wrapper, f'12088::{sid}', 'Inport', name,
              f'[60, {center - 9}, 80, {center + 9}]', {'Port': index})
        line(wrapper, f'12088::{sid}#out:1', f'12088::25#in:{index}')

    sfunction = deepcopy(original.find("Block[@SID='12088::25']"))
    sfunction.find('PortCounts').set('out', '34')
    prop(sfunction, 'PortCounts', '[6 34]')
    prop(sfunction, 'Position', '[380, 40, 800, 1400]')
    output_properties(sfunction, offset=1)
    wrapper.append(sfunction)
    demux = deepcopy(original.find("Block[@SID='12088::26']"))
    prop(demux, 'Position', '[850, 50, 855, 70]')
    wrapper.append(demux)
    terminator = deepcopy(original.find("Block[@SID='12088::27']"))
    prop(terminator, 'Position', '[960, 50, 980, 70]')
    wrapper.append(terminator)
    line(wrapper, '12088::25#out:1', '12088::26#in:1')
    line(wrapper, '12088::26#out:1', '12088::27#in:1')
    for index, name in enumerate(OUTPUTS, 1):
        sid = f'12088::{42 + index}'
        center = 60 + index * 40
        block(wrapper, sid, 'Outport', name,
              f'[960, {center - 9}, 980, {center + 9}]', {'Port': index})
        line(wrapper, f'12088::25#out:{index + 1}', f'{sid}#in:1', name)
    return blocks_before_lines(wrapper)


def algorithm_system(function):
    """六路输入和33路独立命名输出，各自以短直线连接。"""
    system = E.Element('System')
    prop(system, 'Location', '[0, 0, 1500, 1480]')
    prop(function, 'Position', '[380, 40, 800, 1360]')
    prop(function, 'Description', 'v10：独立命名输出，中文注释；仅重构代码风格及接口，数值算法与v9一致。')
    function.find('PortCounts').set('out', '33')
    output_properties(function)
    system.append(function)
    for index, name in enumerate(INPUTS, 1):
        center = 150 + (index - 1) * 220
        sid = 12900 + index
        block(system, sid, 'Inport', name, f'[100, {center - 9}, 120, {center + 9}]', {'Port': index})
        line(system, f'{sid}#out:1', f'12088#in:{index}', name)
    for index, name in enumerate(OUTPUTS, 1):
        center = 60 + (index - 1) * 40
        sid = 12929 + index
        block(system, sid, 'Goto', name, f'[960, {center - 12}, 1280, {center + 12}]',
              {'GotoTag': 'PID_' + name, 'TagVisibility': 'global'})
        line(system, f'12088#out:{index}', f'{sid}#in:1', name)
    return blocks_before_lines(system)


def logging_system(workspace_block):
    """只在日志边界拼接旧31列，算法及参数显示不依赖列号。"""
    system = E.Element('System')
    prop(system, 'Location', '[0, 0, 2100, 1400]')
    for index, name in enumerate(LOG_COLUMNS, 1):
        sid = 12999 + index
        center = 60 + (index - 1) * 40
        block(system, sid, 'From', name, f'[60, {center - 12}, 330, {center + 12}]',
              {'GotoTag': 'PID_' + name})
        line(system, f'{sid}#out:1', f'13050#in:{index}')
    block(system, 13050, 'Mux', '兼容31列日志', '[500, 40, 505, 1280]',
          {'Inputs': 31}, {'in': 31, 'out': 1})
    prop(workspace_block, 'Position', '[760, 640, 1020, 680]')
    system.append(workspace_block)
    line(system, '13050#out:1', '12115#in:1')

    groups = [
        ('运行诊断', 'PID_runtime_diagnostics', LOG_COLUMNS[7:15], 60),
        ('电压诊断', 'PID_voltage_diag', LOG_COLUMNS[17:27], 480),
        ('损耗标定诊断', 'PID_data_loss', LOG_COLUMNS[27:31], 1040),
    ]
    next_sid = 13100
    for group_index, (name, tag, columns, top) in enumerate(groups):
        mux_sid = 13150 + group_index
        goto_sid = 13160 + group_index
        for index, column in enumerate(columns, 1):
            center = top + (index - 1) * 40
            block(system, next_sid, 'From', f'{name}_{column}',
                  f'[1120, {center - 12}, 1420, {center + 12}]', {'GotoTag': 'PID_' + column})
            line(system, f'{next_sid}#out:1', f'{mux_sid}#in:{index}')
            next_sid += 1
        bottom = top + (len(columns) - 1) * 40
        center = (top + bottom) // 2
        block(system, mux_sid, 'Mux', name, f'[1540, {top - 20}, 1545, {bottom + 20}]',
              {'Inputs': len(columns)}, {'in': len(columns), 'out': 1})
        block(system, goto_sid, 'Goto', name + '标签', f'[1700, {center - 12}, 1990, {center + 12}]',
              {'GotoTag': tag, 'TagVisibility': 'global'})
        line(system, f'{mux_sid}#out:1', f'{goto_sid}#in:1')
    return blocks_before_lines(system)


def build():
    with ZipFile(HERE / 'tests/fixtures/reduce_emf_accuracy_v9.slx') as archive:
        entries = archive.infolist()
        data = {entry.filename: archive.read(entry.filename) for entry in entries}

    def save(key, element):
        data[key] = E.tostring(element, encoding='utf-8', xml_declaration=True)

    key = 'simulink/stateflow/chart_22.xml'
    chart = E.fromstring(data[key])
    prop(chart, 'name', '参数辨识与采集/参数辨识算法/参数辨识')
    prop(chart, 'ssIdHighWaterMark', 75)
    chart.find(".//P[@Name='script']").text = (HERE / 'identify_vbemf.m').read_text().replace('identify_vbemf(', 'fcn(', 1)
    children = chart.find('Children')
    input_data = [item for item in children.findall('data') if item.find("P[@Name='scope']").text == 'INPUT_DATA']
    output_template = deepcopy(children.find("data[@name='y']"))
    for item, name in zip(input_data, INPUTS):
        item.set('name', name)
    for item in list(children.findall('data')):
        if item.find("P[@Name='scope']").text == 'OUTPUT_DATA':
            children.remove(item)
    for index, name in enumerate(OUTPUTS, 1):
        item = deepcopy(output_template)
        item.set('SSID', str(42 + index))
        item.set('name', name)
        item.find("props/array/P[@Name='size']").text = '400' if name.endswith('_history') else '1'
        children.append(item)
    save(key, chart)

    key = 'simulink/stateflow/machine.xml'
    machine = E.fromstring(data[key])
    instance = machine.find(".//instance[@id='35']")
    prop(instance, 'name', '参数辨识与采集/参数辨识算法/参数辨识')
    save(key, machine)

    key = 'simulink/systems/system_12300.xml'
    parent = E.fromstring(data[key])
    function = parent.find("Block[@SID='12088']")
    workspace = parent.find("Block[@SID='12115']")
    removed = {'12088', '12115', '12096', '12340', '12341', '12342', '12344',
               '12566', '12568', '12570', '12571', '12573', '12700', '12800'}
    removed.update(str(sid) for sid in range(12350, 12358))
    for element in list(parent.findall('Line')):
        source = element.find("P[@Name='Src']").text.split('#')[0]
        destinations = element.findall(".//P[@Name='Dst']")
        if source in removed or any(item.text.split('#')[0] in removed for item in destinations):
            parent.remove(element)
    for sid in removed:
        parent.remove(parent.find(f"Block[@SID='{sid}']"))
    system_reference(parent, 12900, '参数辨识算法', '[900, 60, 1110, 540]', 6)
    record = system_reference(parent, 12910, '记录与监视', '[1300, 100, 1540, 170]', 0)
    prop(record, 'Description', '将命名信号组合为兼容日志及分组诊断；电感、电阻、磁链直接从各自命名输出读取。')
    for index, sid in enumerate([12610, 12611, 12612, 12613, 12614, 12620], 1):
        line(parent, f'{sid}#out:1', f'12900#in:{index}')

    tags = {
        '12360': 'delta_theta_A', '12361': 'delta_theta_B',
        '12362': 'Ls_hat', '12363': 'psi_f_hat', '12364': 'Lpsi_done',
        '12365': 'Rs_hat', '12366': 'Rs_done',
        '12368': 'delta_theta_A_history', '12369': 'delta_theta_B_history',
        '12565': 'ident_status', '12569': 'rls_updates', '12343': 'sample_ack',
        '12367': 'runtime_diagnostics',
    }
    for sid, tag in tags.items():
        element = parent.find(f"Block[@SID='{sid}']")
        prop(element, 'GotoTag', 'PID_' + tag)
        element.set('Name', tag)
    prop(parent.find("Block[@SID='12343']"), 'Position', '[1600, 598, 1725, 622]')
    prop(parent.find("Block[@SID='12565']"), 'Position', '[100, 1088, 225, 1112]')
    line(parent, '12343#out:1', '12560#in:1')
    block(parent, 12920, 'From', 'diagnostic_code', '[1600, 748, 1725, 772]',
          {'GotoTag': 'PID_diagnostic_code'})
    line(parent, '12920#out:1', '12562#in:1')
    line(parent, '12565#out:1', '12567#in:1')
    line(parent, '12569#out:1', '12572#in:1')
    prop(parent.find("Annotation[@SID='12720']"), 'Name',
         'v10：双击“参数辨识算法”，再双击MATLAB Function查看中文注释的辨识代码。\n'
         '电感Ls_hat、磁链psi_f_hat、电阻Rs_hat、两组角度及诊断量均有独立命名端口。\n'
         '“记录与监视”仅拼接兼容31列日志；采样时序、RLS公式及控制环与v9一致。')
    save(key, parent)
    save('simulink/systems/system_12900.xml', algorithm_system(function))
    save('simulink/systems/system_12910.xml', logging_system(workspace))
    save('simulink/systems/system_12088.xml', function_wrapper(E.fromstring(data['simulink/systems/system_12088.xml'])))
    save('simulink/systems/_rels/system_12300.xml.rels', relationships(12900, 12910))
    save('simulink/systems/_rels/system_12900.xml.rels', relationships(12088))

    key = 'simulink/systems/system_root.xml'
    root = E.fromstring(data[key])
    prop(root, 'SIDHighWatermark', 13200)
    save(key, root)

    output = HERE / 'reduce_emf_readable.slx'
    with ZipFile(output, 'w') as archive:
        for entry in entries:
            archive.writestr(entry, data.pop(entry.filename))
        for name, content in data.items():
            archive.writestr(name, content)
    for name in ['reduce_emf_accuracy.slx', 'reduce_emf_identified.slx', 'reduce_emf_speedloop.slx']:
        (HERE / name).write_bytes(output.read_bytes())
    print(output)


if __name__ == '__main__':
    build()
