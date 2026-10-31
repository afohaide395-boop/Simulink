"""本次模型整理的端口、信号来源、角度定义及控制回路回归。"""
from collections import Counter
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
from zipfile import ZipFile
import argparse
import math
import sys
import xml.etree.ElementTree as E

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from model_cleanup.build_model import BASE, OUTPUT, OUTPUTS, VISIBLE
from audit_model_load_paths import audit
from check_organized_model import ResolvedGraph, canonical
from check_checked_model import port_counts
from check_power_connections import physical_lines, physical_nets
from check_speed_control import check as check_speed
from model_geometry import Geometry, field, is_orthogonal, crosses_rectangle


def zero_origin(graph, endpoint):
    origin = graph.origin(endpoint)
    if origin.startswith('UNCONNECTED:'):
        return 'ZERO'
    if graph.blocks[origin.split('#')[0]].get('BlockType') == 'Ground':
        return 'ZERO'
    return origin


def check(model=OUTPUT):
    assert not audit(model), audit(model)
    before = ResolvedGraph(BASE.read_bytes())
    after = ResolvedGraph(model.read_bytes())
    scopes = {sid for sid, b in after.blocks.items() if b.get('BlockType') == 'Scope'}
    assert scopes == {'12098', '11355', '12097'}
    assert all(after.owners[sid] == 'system_13240' for sid in scopes)
    for sid, item in after.blocks.items():
        if item.get('BlockType') == 'From':
            after.goto_for(sid)

    # 四路接口不得再把角度错接到电感/磁链。
    for index, (name, tag) in enumerate(OUTPUTS, 1):
        goto = next(sid for sid, b in after.blocks.items()
                    if b.get('BlockType') == 'Goto' and field(b, 'GotoTag') == tag)
        assert after.inputs[goto+'#in:1'] == f'12088#out:{index}', ('辨识端口错接', name)
        assert after.origin(goto+'#in:1') == f'12088::25#out:{index+1}'
        assert after.origin(f'13050#in:{index}') == f'12088::25#out:{index+1}'
    assert field(after.blocks['13050'], 'Inputs') == '4'
    assert after.inputs['12115#in:1'] == '13050#out:1'
    assert after.blocks['12343'].get('BlockType') == 'Constant'
    assert field(after.blocks['12343'], 'Value') == '0'
    assert after.origin('12073#in:4') == '12560#out:1'
    assert after.inputs['12560#in:1'] == '12343#out:1'

    # 电机角度必须是thetam*Np，FOC误差必须使用Park实际采用的角度。
    theta_motor = next(s for s, b in after.blocks.items() if b.get('BlockType') == 'Goto'
                       and field(b, 'GotoTag') == 'MON_theta_motor')
    mod_motor = after.inputs[theta_motor+'#in:1'].split('#')[0]
    assert after.origin(mod_motor+'#in:1') == '11847#out:1'
    assert after.origin('11847#in:1') == '10654#out:1'
    assert field(after.blocks['10654'], 'Gain') == 'Np'
    assert after.origin('10654#in:1') == '10651#out:5'
    bus_signals = field(after.blocks['10651'], 'OutputSignals').split(',')
    assert bus_signals[4] == 'Rotor angle thetam (rad)'
    for tag, expected in [('MON_delta_pll', '12073::32#out:2'),
                          ('MON_delta_ctrl', after.origin('103#in:3'))]:
        goto = next(s for s, b in after.blocks.items() if b.get('BlockType') == 'Goto'
                    and field(b, 'GotoTag') == tag)
        calc = after.inputs[goto+'#in:1'].split('#')[0]
        assert field(after.blocks[calc], 'Expr') == 'atan2(sin(u[2]-u[1]),cos(u[2]-u[1]))'
        mux = after.inputs[calc+'#in:1'].split('#')[0]
        mod_est = after.origin(mux+'#in:1').split('#')[0]
        assert after.origin(mod_est+'#in:1') == expected
        assert after.origin(mux+'#in:2') == mod_motor+'#out:1'
    for true, observed, expected in [(2*math.pi-.01, .01, -.02),
                                      (.01, 2*math.pi-.01, .02), (0, -.1, .1)]:
        value = math.atan2(math.sin(true-observed), math.cos(true-observed))
        assert abs(value-expected) < 1e-12
    # 用模型相同的Park定义核对论文式(2)：Ed=-we*psi*sin(delta)。
    # 因而代码辨识的delta应为theta_true-theta_est，不能用相反号。
    for true, observed in [(0.6, 0.65), (6.2, 0.02), (-0.2, -0.3)]:
        for speed in [180, -180]:
            ea = -speed*0.21*math.sin(true)
            eb = speed*0.21*math.cos(true)
            ed = ea*math.cos(observed)+eb*math.sin(observed)
            eq = -ea*math.sin(observed)+eb*math.cos(observed)
            delta = math.atan2(math.sin(true-observed), math.cos(true-observed))
            assert abs(delta-math.atan(-ed/eq)) < 1e-12

    # 八个可见图：检查所有输入、输出及分支线，不只检查已连接的端口。
    used = Counter(after.inputs.values())
    line_count = 0
    for ref in VISIBLE + ['12088']:
        system = after.systems['system_'+ref]
        geometry = Geometry(system)
        rectangles = {sid: geometry.position(sid) for sid in geometry.blocks}
        for sid, item in geometry.blocks.items():
            if field(item, 'Commented') == 'on':
                continue
            ni, no = port_counts(item)
            for n in range(1, ni+1):
                assert f'{sid}#in:{n}' in after.inputs, ('活动输入悬空', ref, sid, n)
            for n in range(1, no+1):
                assert used[f'{sid}#out:{n}'], ('活动输出悬空', ref, sid, n)
        for sid, a in rectangles.items():
            for other, b in rectangles.items():
                if sid >= other or ref not in VISIBLE:
                    continue
                assert not (max(a[0], b[0]) < min(a[2], b[2]) and max(a[1], b[1]) < min(a[3], b[3])), ('模块重叠', ref, sid, other)
        sources = set()
        for wire in system.findall('Line'):
            if wire.get('LineType') == 'Connection':
                continue
            source = field(wire, 'Src')
            assert source and source not in sources
            sources.add(source)
            for src, dst, points in geometry.polylines(wire):
                if ref in VISIBLE:
                    assert is_orthogonal(points), ('斜线', ref, src, dst)
                excluded = {src.split('#')[0], dst.split('#')[0]}
                if ref in VISIBLE:
                    for sid, rect in rectangles.items():
                        if sid in excluded:
                            continue
                        assert not any(crosses_rectangle(a, b, rect) for a, b in zip(points, points[1:])), ('穿模块', ref, src, dst, sid)
                line_count += 1

    # 所有保留的控制/电机输入仍取自原有来源，唯一例外是原来缺失的ack。
    compared = 0
    for endpoint in before.inputs:
        sid = endpoint.split('#')[0]
        if sid not in after.blocks or before.blocks[sid].get('BlockType') == 'Scope':
            continue
        path = before.block_path(sid)
        if path.startswith(('参数辨识与采集/', '运行监测/')):
            continue
        try:
            old_origin = zero_origin(before, endpoint)
        except AssertionError as error:
            assert 'PID_ack' in str(error), (endpoint, error)
            continue
        assert zero_origin(after, endpoint) == old_origin, ('原控制输入来源变化', endpoint)
        compared += 1
    for port in range(1, 7):
        assert after.origin(f'12088#in:{port}') == before.origin(f'12088#in:{port}')
    for sid in ['3', '7', '1', '10661']:
        assert canonical(before.blocks[sid]) == canonical(after.blocks[sid]), ('电气模块参数变化', sid)
    roots = [g.systems['system_root'] for g in [before, after]]
    assert Counter(map(canonical, physical_lines(roots[0]))) == Counter(map(canonical, physical_lines(roots[1])))
    assert physical_nets(roots[0]) == physical_nets(roots[1])

    with ZipFile(BASE) as old, ZipFile(model) as new:
        assert old.namelist() == new.namelist()
        assert new.testzip() is None
        for name in old.namelist():
            if name.startswith('simulink/stateflow/chart_') and name != 'simulink/stateflow/chart_85.xml':
                assert old.read(name) == new.read(name), ('其他算法变化', name)
            if not name.startswith('simulink/systems/') and name not in ['simulink/stateflow/chart_85.xml', 'simulink/windowsInfo.xml']:
                assert old.read(name) == new.read(name), ('其他配置变化', name)
            if name.endswith(('.xml', '.rels')):
                E.fromstring(new.read(name))
        chart = E.fromstring(new.read('simulink/stateflow/chart_85.xml'))
        assert chart.find(".//P[@Name='script']").text == (HERE/'fcn.m').read_text()
        for item in chart.findall('Children/data'):
            if field(item, 'scope') in ['INPUT_DATA', 'OUTPUT_DATA']:
                assert item.find("props/array/P[@Name='size']").text == '1'
    for ref, system in after.systems.items():
        own_ids = {b.get('SID') for b in system.findall('Block')}
        assert len(own_ids) == len(system.findall('Block'))
        for wire in system.findall('Line'):
            for endpoint in wire.findall(".//P[@Name='Src']")+wire.findall(".//P[@Name='Dst']"):
                assert endpoint.text.split('#')[0] in own_ids
    check_speed(model.read_bytes())
    print('PASS 21个Scope收敛为3个，全部From均有来源，四输出和四列日志对应')
    print('PASS 电机机械角乘Np得到电角度；PLL和FOC角度差均为真实减观测，跨2π正确')
    print(f'PASS 8个可见图及函数封装：{line_count}个信号端点连接，未发现活动端口悬空、斜线或模块重叠')
    print(f'PASS {compared}个原控制输入的来源保留；辨识六路输入、电气回路、其他算法、初始化和求解器保留')
    print('PASS Src先于Points；HMI/记录路径一致；辨识接口六入四出均明确为标量')
    print('以上为离线检查，不等同于MATLAB/Simulink实际打开、编译或仿真。')


def self_test():
    with ZipFile(OUTPUT) as archive:
        entries = archive.infolist()
        data = {item.filename: archive.read(item.filename) for item in entries}
    with TemporaryDirectory(prefix='cleanup_check_') as folder:
        for mode in ['Ls', 'missing', 'order']:
            key = 'simulink/systems/system_12900.xml' if mode == 'Ls' else 'simulink/systems/system_13240.xml'
            root = E.fromstring(data[key])
            if mode == 'Ls':
                wire = next(w for w in root.findall('Line') if field(w, 'Dst') == '12930#in:1')
                wire.find("P[@Name='Src']").text = '12088#out:1'
            elif mode == 'missing':
                root.remove(next(w for w in root.findall('Line') if field(w, 'Dst') == '11355#in:1'))
            else:
                wire = root.find('Line')
                points = wire.find("P[@Name='Points']")
                wire.remove(points)
                wire.insert(0, points)
            broken = Path(folder)/(mode+'.slx')
            with ZipFile(broken, 'w') as archive:
                for entry in entries:
                    archive.writestr(entry, E.tostring(root) if entry.filename == key else data[entry.filename])
            try:
                check(broken)
            except AssertionError:
                print('PASS 故障回归检出：'+{'Ls': '角度错接电感', 'missing': '电流Scope输入断线', 'order': 'Points写在Src前'}[mode])
            else:
                raise AssertionError('未检出故障：'+mode)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    check()
    if args.self_test:
        self_test()
