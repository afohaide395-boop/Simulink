"""整理本次上传的模型：角度对比、三个Scope、四输出PSO原风格代码。"""
from collections import defaultdict
from copy import deepcopy
from io import BytesIO
from pathlib import Path
from zipfile import ZipFile
import json
import sys
import xml.etree.ElementTree as E

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(REPO))
from build_readable import block, prop as set_prop, line, blocks_before_lines
from build_checked import route
from model_geometry import Geometry, field, crosses_rectangle
from check_organized_model import ResolvedGraph

BASE = REPO / 'tests/fixtures/reduce_emf_user_cleanup.slx'
OUTPUT = REPO / 'reduce_emf_tidy.slx'
VISIBLE = ['root', '13210', '13220', '13230', '13240', '12300', '12900', '12910']
PROTECTED = {'2566', '2645'}  # 原模型已有HMI记录，保留这两个测点。
OUTPUTS = [('dtheta', 'PID_da'), ('dtheta1', 'PID_db'),
           ('Ls_hat', 'PID_Ls'), ('psi_hat', 'PID_psi')]


def prop(element, name, value):
    set_prop(element, name, value)
    item = element.find(f"P[@Name='{name}']")
    barriers = {'System', 'PortProperties', 'InstanceData'} if element.tag == 'Block' else {'Block', 'Line', 'Annotation'}
    if element.tag in ['Block', 'System']:
        first = next((p for p in element if p.tag in barriers), None)
        if first is not None and list(element).index(item) > list(element).index(first):
            element.remove(item)
            element.insert(list(element).index(first), item)


def archive_bytes(entries, data):
    stream = BytesIO()
    with ZipFile(stream, 'w') as archive:
        for entry in entries:
            archive.writestr(entry, data[entry.filename])
    return stream.getvalue()


def prune_line(wire, removed):
    """删除接到已删块的分支；保留电气连接的Src-only分支。"""
    source = field(wire, 'Src')
    if source and source.split('#')[0] in removed:
        return False
    destination = wire.find("P[@Name='Dst']")
    if destination is not None and destination.text.split('#')[0] in removed:
        wire.remove(destination)
    for child in list(wire.findall('Branch')):
        if not prune_line(child, removed):
            wire.remove(child)
    return (wire.find("P[@Name='Dst']") is not None or bool(wire.findall('Branch'))
            or wire.get('LineType') == 'Connection')


def remove_blocks(system, sids):
    for item in list(system.findall('Block')):
        if item.get('SID') in sids:
            system.remove(item)
    for wire in list(system.findall('Line')):
        if wire.get('LineType') == 'Connection':
            continue
        if not prune_line(wire, sids):
            system.remove(wire)


def direct_line(system, source, destination, name=None):
    line(system, source, destination, name)


def offsets(path):
    pairs = [(b[0]-a[0], b[1]-a[1]) for a, b in zip(path, path[1:-1])]
    return '[' + '; '.join(f'{x:.15g}, {y:.15g}' for x, y in pairs) + ']'


def routed_lines(system):
    """每个源只生成一棵线树，始终保持Src在Points前。"""
    edges = defaultdict(list)
    names = {}
    for wire in system.findall('Line'):
        if wire.get('LineType') == 'Connection':
            continue
        source = field(wire, 'Src')
        names[source] = field(wire, 'Name')
        edges[source].extend(p.text for p in wire.findall(".//P[@Name='Dst']"))
    geometry = Geometry(system)
    for wire in list(system.findall('Line')):
        if wire.get('LineType') != 'Connection':
            system.remove(wire)
    for index, (source, destinations) in enumerate(edges.items(), 1):
        wire = E.SubElement(system, 'Line')
        if names[source]:
            prop(wire, 'Name', names[source])
        prop(wire, 'ZOrder', 200000+index)
        prop(wire, 'Src', source)
        if len(destinations) == 1:
            prop(wire, 'Points', offsets(route(source, destinations[0], geometry)))
            prop(wire, 'Dst', destinations[0])
        else:
            start = geometry.endpoint(source)
            mirrored = field(geometry.blocks[source.split('#')[0]], 'BlockMirror') == 'on'
            dx = -20 if mirrored else 20
            split = (start[0]+dx, start[1])
            prop(wire, 'Points', f'[{dx}, 0]')
            class BranchGeometry:
                blocks = geometry.blocks
                position = geometry.position
                def endpoint(self, endpoint):
                    return split if endpoint == source else geometry.endpoint(endpoint)
            for destination in destinations:
                branch = E.SubElement(wire, 'Branch')
                prop(branch, 'Points', offsets(route(source, destination, BranchGeometry())))
                prop(branch, 'Dst', destination)
    return blocks_before_lines(system)


def scope_config(displays, log_name):
    """沿用R2024b原生Scope配置字段，清空旧通道/游标缓存。"""
    colors = '[0.0667 0.4431 0.7451;0.8667 0.3294 0;0.2314 0.6667 0.1961;0.5216 0.0863 0.8196]'
    structs = []
    for index, (title, unit, channels, limits) in enumerate(displays, 1):
        cells = ','.join("'"+name+"'" for name in channels)
        structs.append(
            "struct('MinYLimReal','"+str(limits[0])+"','MaxYLimReal','"+str(limits[1])+"',"
            "'MinYLimMag','0','MaxYLimMag','"+str(max(abs(limits[0]), abs(limits[1])))+"',"
            "'YLabelReal','"+unit+"','LegendVisibility','On','XGrid',true,'YGrid',true,"
            "'PlotMagPhase',false,'AxesColor',[1 1 1],'AxesTickColor',[0 0 0],"
            "'ColorOrder',"+colors+",'Title','"+title+"','LinePropertiesCache',{{}},"
            "'UserDefinedChannelNames',{{"+cells+"}},'NumLines',"+str(len(channels))+","
            "'LineNames',{{"+cells+"}},'ShowContent',true,'Placement',"+str(index)+")")
    return (
        "Simulink.scopes.TimeScopeBlockCfg('CurrentConfiguration', extmgr.ConfigurationSet("
        "extmgr.Configuration('Core','General UI',true,'FigureColor',[1 1 1]),"
        "extmgr.Configuration('Core','Source UI',true),"
        "extmgr.Configuration('Sources','WiredSimulink',true,'DataLoggingVariableName','"+log_name+"'),"
        "extmgr.Configuration('Visuals','Time Domain',true,'SerializedDisplays',{"+','.join(structs)+"},"
        "'DisplayPropertyDefaults',struct('YLabelReal','','AxesColor',[1 1 1],"
        "'AxesTickColor',[0 0 0],'ColorOrder',"+colors+",'Title','%<SignalLabel>'),"
        "'DisplayLayoutDimensions',["+str(len(displays))+" 1]),"
        "extmgr.Configuration('Tools','Plot Navigation',true,'OnceAtStop',true,'PreviousAutoscale','XY'),"
        "extmgr.Configuration('Tools','Measurements',true,'Version','2024b')),"
        "'Version','2024b','Position',[1 49 1400 800])")


def build():
    with ZipFile(BASE) as archive:
        entries = archive.infolist()
        data = {item.filename: archive.read(item.filename) for item in entries}
    before = ResolvedGraph(BASE.read_bytes())
    systems = {ref: deepcopy(root) for ref, root in before.systems.items()}
    old_scopes = {sid for sid, b in before.blocks.items() if b.get('BlockType') == 'Scope'}
    deleted = set()
    new_id = 14000
    def sid():
        nonlocal new_id
        value = new_id
        new_id += 1
        return value
    def add(system, kind, name, rect, properties=None, ports=None, number=None):
        if kind in ['From', 'Goto']:
            properties = {'ShowName': 'off', **(properties or {})}
        return block(system, number if number is not None else sid(), kind, name, rect, properties, ports)
    def save(name, root):
        data[name] = E.tostring(root, encoding='utf-8', xml_declaration=True)
    def sync():
        for ref, system in systems.items():
            key = 'simulink/systems/'+ref+'.xml'
            original = before.systems.get(ref)
            if original is None or E.tostring(system) != E.tostring(original):
                save(key, blocks_before_lines(system))

    for system in systems.values():
        removed = {b.get('SID') for b in system.findall('Block') if b.get('BlockType') == 'Scope'}
        remove_blocks(system, removed)
        deleted.update(removed)

    # 主算法四个端口分别连接正确的全局信号，清理旧端口显示名。
    alg = systems['system_12900']
    for wire in list(alg.findall('Line')):
        if (field(wire, 'Src') or '').startswith('12088#out:'):
            alg.remove(wire)
    for port, (name, tag) in enumerate(OUTPUTS, 1):
        old_sid = {'PID_Ls': '12930', 'PID_psi': '12931'}.get(tag)
        item = alg.find(f"Block[@SID='{old_sid}']") if old_sid else None
        y = 40+(port-.5)*120
        if item is None:
            item = add(alg, 'Goto', name, f'[860, {y-12:g}, 1060, {y+12:g}]',
                       {'GotoTag': tag, 'TagVisibility': 'global'}, {'in': 1, 'out': 0})
        item.set('Name', name)
        prop(item, 'GotoTag', tag)
        prop(item, 'Position', f'[860, {y-12:g}, 1060, {y+12:g}]')
        direct_line(alg, f'12088#out:{port}', item.get('SID')+'#in:1', name)
    function = alg.find("Block[@SID='12088']")
    prop(function, 'Description', '原变量名与switch采样结构；四路输出依次为角度A、角度B、电感、磁链。')
    for index, port in enumerate(function.findall('PortProperties/Port'), 1):
        name = OUTPUTS[index-1][0]
        prop(port, 'Name', name)
        prop(port, 'PropagatedSignals', name)

    # 旧的12列接口与四输出辨识器不符，改为四个真实输出的记录。
    old_log = systems['system_12910']
    deleted.update(b.get('SID') for b in old_log.findall('Block'))
    logger = deepcopy(old_log.find("Block[@SID='12115']"))
    log = E.Element('System')
    prop(log, 'Location', '[0, 0, 1100, 400]')
    for index, (name, tag) in enumerate(OUTPUTS, 1):
        y = 80+(index-1)*60
        item = add(log, 'From', name, f'[80, {y-12}, 300, {y+12}]', {'GotoTag': tag}, {'out': 1})
        direct_line(log, item.get('SID')+'#out:1', f'13050#in:{index}')
    add(log, 'Mux', '四路辨识结果', '[480, 50, 485, 290]', {'Inputs': 4}, {'in': 4, 'out': 1}, 13050)
    prop(logger, 'Position', '[700, 150, 1000, 190]')
    prop(logger, 'Description', '列顺序：[dtheta dtheta1 Ls_hat psi_hat]。')
    log.append(logger)
    direct_line(log, '13050#out:1', '12115#in:1')
    systems['system_12910'] = log

    parent = systems['system_12300']
    remove = {'12365', '12364', '12366', '12569', '12565', '12572', '12567', '12117'}
    remove_blocks(parent, remove)
    deleted.update(remove)
    ack = parent.find("Block[@SID='12343']")
    ack.set('BlockType', 'Constant')
    ack.set('Name', '固定采样时序_确认口为零')
    for p in list(ack):
        if p.tag == 'P' and p.get('Name') not in ['Position', 'ZOrder']:
            ack.remove(p)
    prop(ack, 'Value', 0)
    prop(ack, 'SampleTime', 'Ts')
    prop(ack, 'Description', '四输出算法不产生ack；PLL原有固定阶段时序无需额外确认。')
    prop(ack, 'Position', '[950, 568, 1100, 592]')
    prop(parent.find("Block[@SID='12560']"), 'Position', '[1180, 565, 1290, 595]')
    prop(parent.find("Block[@SID='12561']"), 'Position', '[1370, 568, 1570, 592]')
    prop(parent, 'Location', '[0, 0, 1660, 1250]')
    prop(parent, 'ZoomFactor', 75)

    # 监测只读三种角度：电机真实电角度、PLL输出和实际FOC控制角。
    old_monitor = systems['system_13240']
    deleted.update(b.get('SID') for b in old_monitor.findall('Block'))
    monitor = E.Element('System')
    prop(monitor, 'Location', '[0, 0, 1600, 1000]')
    prop(monitor, 'ZoomFactor', 70)
    def from_tag(tag, name, x, y):
        return add(monitor, 'From', name, f'[{x}, {y-12}, {x+200}, {y+12}]', {'GotoTag': tag}, {'out': 1})
    def goto_tag(source, tag, name, x, y):
        item = add(monitor, 'Goto', name, f'[{x}, {y-12}, {x+220}, {y+12}]',
                   {'GotoTag': tag}, {'in': 1, 'out': 0})
        direct_line(monitor, source, item.get('SID')+'#in:1', name)
    for index, (tag, title, local) in enumerate([
        ('the', '电机真实电角度_rad', 'MON_theta_motor'),
        ('PLL_theta_monitor', 'PLL观测电角度_rad', 'MON_theta_pll'),
        ('thera_feedback', 'FOC控制电角度_rad', 'MON_theta_ctrl')]):
        y = 80+index*80
        source = from_tag(tag, title, 40, y)
        calc = add(monitor, 'Fcn', title+'_取模', f'[300, {y-20}, 470, {y+20}]',
                   {'Expr': 'u-2*pi*floor(u/(2*pi))'}, {'in': 1, 'out': 1})
        direct_line(monitor, source.get('SID')+'#out:1', calc.get('SID')+'#in:1')
        goto_tag(calc.get('SID')+'#out:1', local, title, 600, y)
    for index, (tag, title, local) in enumerate([
        ('MON_theta_pll', 'PLL相对电机的角度差_rad', 'MON_delta_pll'),
        ('MON_theta_ctrl', 'FOC相对电机的角度差_rad', 'MON_delta_ctrl')]):
        y = 370+index*160
        first = from_tag(tag, title+'_观测角', 40, y-20)
        second = from_tag('MON_theta_motor', title+'_真实角', 40, y+20)
        mux = add(monitor, 'Mux', title+'_输入', f'[300, {y-40}, 305, {y+40}]',
                  {'Inputs': 2}, {'in': 2, 'out': 1})
        calc = add(monitor, 'Fcn', title, f'[380, {y-20}, 570, {y+20}]',
                   {'Expr': 'atan2(sin(u[2]-u[1]),cos(u[2]-u[1]))'}, {'in': 1, 'out': 1})
        direct_line(monitor, first.get('SID')+'#out:1', mux.get('SID')+'#in:1')
        direct_line(monitor, second.get('SID')+'#out:1', mux.get('SID')+'#in:2')
        direct_line(monitor, mux.get('SID')+'#out:1', calc.get('SID')+'#in:1')
        goto_tag(calc.get('SID')+'#out:1', local, title, 650, y)

    def scope(number, name, rect, displays, log_name):
        return add(monitor, 'Scope', name, rect,
                   {'ScopeSpecificationString': scope_config(displays, log_name),
                    'NumInputPorts': len(displays), 'Floating': 'off'}, {'in': len(displays)}, number)
    angle_displays = [
        ('电角度对比', 'rad', ['电机真实角度', 'PLL观测角度', 'FOC控制角度'], [0, 6.3]),
        ('实际角度差_真实减观测', 'rad', ['PLL角度差', 'FOC角度差'], [-0.5, 0.5]),
        ('实际角度差与辨识结果', 'rad', ['实际FOC角度差', '辨识角度A', '辨识角度B'], [-0.2, 0.2]),
        ('注入与采样阶段', 'stage', ['stage'], [0, 10])]
    scope(12098, '电机与观测器_角度及误差对比', '[1740, 80, 1960, 720]', angle_displays, 'AngleScopeData')
    prop(monitor.find("Block[@SID='12098']"), 'Description',
         '角度差为真实电角减观测角并折回[-pi,pi]。辨识A/B对应两工作点的未注入基准误差；注入期间实际误差包含附加偏移，宜结合stage2/6及稳态窗口比较。')
    rows = [
        [('MON_theta_motor', '电机真实角度'), ('MON_theta_pll', 'PLL观测角度'), ('MON_theta_ctrl', 'FOC控制角度')],
        [('MON_delta_pll', 'PLL实际角度差'), ('MON_delta_ctrl', 'FOC实际角度差')],
        [('MON_delta_ctrl', '实际FOC角度差'), ('PID_da', '辨识角度A'), ('PID_db', '辨识角度B')]]
    for index, channels in enumerate(rows):
        center = 160+index*160
        top = center-20*len(channels)
        mux = add(monitor, 'Mux', '角度显示'+str(index+1),
                  f'[1480, {top}, 1485, {top+40*len(channels)}]',
                  {'Inputs': len(channels)}, {'in': len(channels), 'out': 1})
        for port, (tag, name) in enumerate(channels, 1):
            source = from_tag(tag, name, 1120, top+(port-.5)*40)
            direct_line(monitor, source.get('SID')+'#out:1', mux.get('SID')+f'#in:{port}', name)
        direct_line(monitor, mux.get('SID')+'#out:1', f'12098#in:{index+1}')
    source = from_tag('CTRL_State', '注入与采样阶段', 1120, 640)
    direct_line(monitor, source.get('SID')+'#out:1', '12098#in:4', 'stage')
    scope(11355, '电机运行_三相电流转矩转速', '[600, 900, 820, 1140]', [
        ('三相电流', 'A', ['ia', 'ib', 'ic'], [-8, 8]),
        ('电磁转矩', 'Nm', ['Te'], [-8, 10]),
        ('机械转速', 'rpm', ['rpm'], [0, 600])], 'MotorScopeData')
    for port, (tag, name) in enumerate([('CTRL_iabc', '三相电流'), ('CTRL_Te', '电磁转矩'), ('rpm_real', '机械转速')], 1):
        source = from_tag(tag, name, 100, 900+(port-.5)*80)
        direct_line(monitor, source.get('SID')+'#out:1', f'11355#in:{port}', name)
    scope(12097, '参数辨识_电感与磁链', '[1740, 900, 1960, 1060]', [
        ('电感辨识', 'H', ['Ls_hat'], [-0.005, 0.015]),
        ('磁链辨识', 'Wb', ['psi_hat'], [-0.05, 0.3])], 'ParameterScopeData')
    for port, (tag, name) in enumerate([('PID_Ls', 'Ls_hat'), ('PID_psi', 'psi_hat')], 1):
        source = from_tag(tag, name, 1300, 900+(port-.5)*80)
        direct_line(monitor, source.get('SID')+'#out:1', f'12097#in:{port}', name)
    systems['system_13240'] = monitor

    # 删除只为旧Scope服务的From/Mux/Demux，保留既有HMI测点。
    while True:
        changed = False
        for ref, system in systems.items():
            if ref in ['system_13240', 'system_12910']:
                continue
            used = {field(w, 'Src').split('#')[0] for w in system.findall('Line') if field(w, 'Src')}
            leaves = {b.get('SID') for b in system.findall('Block')
                      if b.get('BlockType') in ['From', 'Mux', 'Demux'] and b.get('SID') not in used
                      and b.get('SID') not in PROTECTED}
            if leaves:
                remove_blocks(system, leaves)
                deleted.update(leaves)
                changed = True
        if not changed:
            break

    # 保留HMI信号采集；显示支路删除后显式终止输出。
    for number in PROTECTED:
        owner = before.owners[number]
        system = systems[owner]
        if not any(field(w, 'Src') == number+'#out:1' for w in system.findall('Line')):
            geometry = Geometry(system)
            x, y = geometry.endpoint(number+'#out:1')
            term = add(system, 'Terminator', '保留HMI测点_'+number,
                       f'[{x+80:g}, {y-10:g}, {x+100:g}, {y+10:g}]', ports={'in': 1})
            direct_line(system, number+'#out:1', term.get('SID')+'#in:1')

    # 原机无外加负载，悬空Tm等价于零；显式接零避免端口悬空。
    root = systems['system_root']
    if not any(p.text == '10661#in:1' for p in root.findall("Line//P[@Name='Dst']")):
        zero = add(root, 'Ground', '负载转矩零', '[735, 340, 755, 360]', ports={'out': 1})
        direct_line(root, zero.get('SID')+'#out:1', '10661#in:1')
    prop(root, 'SIDHighWatermark', new_id+100)
    prop(root.find("Block[@SID='13240']"), 'Description',
         '三个主要显示：电角度及误差对比、电感磁链、三相电流转矩转速。角度差统一为真实减观测，单位rad。')

    for ref in VISIBLE:
        systems['system_'+ref] = routed_lines(systems['system_'+ref])
    sync()
    chart = E.fromstring(data['simulink/stateflow/chart_85.xml'])
    chart.find(".//P[@Name='script']").text = (HERE/'fcn.m').read_text()
    for item in chart.findall('Children/data'):
        if field(item, 'scope') in ['INPUT_DATA', 'OUTPUT_DATA']:
            item.find("props/array/P[@Name='size']").text = '1'
    save('simulink/stateflow/chart_85.xml', chart)

    # 清理已删除Scope/显示块的窗口恢复记录，不动其他模型和编辑器设置。
    windows = E.fromstring(data['simulink/windowsInfo.xml'])
    removed_windows = 0
    for editors in windows.findall(".//Array[@PropName='EditorsInfo']"):
        for editor in list(editors):
            load_id = field(editor, 'LoadSaveID')
            if load_id in deleted or load_id in old_scopes:
                editors.remove(editor)
                removed_windows += 1
        editors.set('Dimension', '1*'+str(len(editors)))
    if removed_windows:
        save('simulink/windowsInfo.xml', windows)
    OUTPUT.write_bytes(archive_bytes(entries, data))
    after = ResolvedGraph(OUTPUT.read_bytes())
    for number, item in after.blocks.items():
        if item.get('BlockType') == 'From':
            after.goto_for(number)
    from audit_model_load_paths import audit
    assert not audit(OUTPUT), audit(OUTPUT)
    report = {'scope_before': len(old_scopes), 'scope_after': 3,
              'deleted_blocks': sorted(set(before.blocks)-set(after.blocks)),
              'added_blocks': sorted(set(after.blocks)-set(before.blocks)),
              'removed_window_entries': removed_windows,
              'source': BASE.name, 'output': OUTPUT.name}
    (HERE/'manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(f'{OUTPUT.name}：Scope {len(old_scopes)}→3，四路辨识及角度对比连接完成。')


if __name__ == '__main__':
    build()
