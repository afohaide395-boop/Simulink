"""将物理电气网络与原模型逐项对照，防止整理信号线时误删电源回路。
在仓库运行python check_power_connections.py，无需MATLAB。
此检查仅验证拓扑及接线，不代表电机整机仿真已经通过。
"""
from pathlib import Path
from collections import Counter
import argparse,io,subprocess,zipfile,xml.etree.ElementTree as E

ORIGINAL_COMMIT='68fcd72467b2e6570c869c553a16e191e0690b8d'
BROKEN_COMMIT='67296e83b2295515e113222ebbf37bbe7c94c05c'
KEY='simulink/systems/system_root.xml'

def canonical(el):
    return E.canonicalize(E.tostring(el,encoding='unicode'),strip_text=True)

def root_from_bytes(content):
    with zipfile.ZipFile(io.BytesIO(content)) as z:
        assert z.testzip() is None,'Invalid SLX ZIP'
        return E.fromstring(z.read(KEY))

def physical_lines(root):
    return [l for l in root.findall('Line') if l.get('LineType')=='Connection']

def physical_nets(root):
    nets=[]
    for line in physical_lines(root):
        # 电气线可能在所有端点均使用Src而没有Dst，不能按普通信号线规则删线。
        endpoints={p.text for p in line.iter('P') if p.get('Name') in ['Src','Dst']
                   and p.text and ('#lconn:' in p.text or '#rconn:' in p.text)}
        assert len(endpoints)>=2,'Dangling physical connection'
        nets.append(frozenset(endpoints))
    return set(nets)

def check_circuit(model,reference):
    assert Counter(map(canonical,physical_lines(model)))==Counter(map(canonical,physical_lines(reference))), 'Physical branch differs from original'
    expected={
        frozenset(['7#rconn:2','3#lconn:1','6198#lconn:1']),
        frozenset(['3#rconn:1','7#rconn:1']),
        *[frozenset([f'10661#lconn:{n}',f'7#lconn:{n}']) for n in [1,2,3]]}
    assert physical_nets(model)==expected,'DC return, DC supply or motor phase net missing'
    for sid in ['3','7','6198','10661','1','12075','12008','12000','12010','12009']:
        a=model.find(f"Block[@SID='{sid}']");b=reference.find(f"Block[@SID='{sid}']")
        assert a is not None and b is not None and canonical(a)==canonical(b), f'Power/startup block {sid} changed'
    edges=set()
    for line in model.findall('Line'):
        if line.get('LineType')=='Connection':continue
        src=line.find("P[@Name='Src']")
        if src is not None:
            edges.update((src.text,p.text) for p in line.findall(".//P[@Name='Dst']"))
    assert ('12008#out:1','12010#in:3') in edges,'I/F current reference disconnected'
    assert ('12023#out:1','12010#in:2') in edges,'I/F current selector control disconnected'
    assert ('12010#out:1','2546#in:1') in edges,'Selected iq command disconnected'

def git_model(repo,commit):
    return subprocess.run(['git','-C',str(repo),'show',commit+':reduce_emf_identified.slx'],check=True,capture_output=True).stdout

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--model',type=Path)
    ap.add_argument('--repo',type=Path,default=Path(__file__).resolve().parent)
    ap.add_argument('--reference',type=Path)
    ap.add_argument('--self-test',action='store_true');args=ap.parse_args()
    model=args.model or args.repo/'reduce_emf_identified.slx'
    uploaded=args.repo/'tests/fixtures/reduce_emf_before_speedloop.slx'
    reference_path=args.reference or (uploaded if uploaded.exists() else None)
    reference=root_from_bytes(reference_path.read_bytes() if reference_path else git_model(args.repo,ORIGINAL_COMMIT))
    check_circuit(root_from_bytes(model.read_bytes()),reference)
    print('PASS all 5 original physical connection trees and nets, including Src-only DC-negative/ground branch')
    print('PASS power source, bridge, ground, motor, powergui and I/F startup block parameters preserved')
    print('PASS I/F iq source, selector control and selected-iq command signal connections')
    if args.self_test:
        bad=root_from_bytes(git_model(args.repo,BROKEN_COMMIT))
        try:
            assert physical_nets(bad)==physical_nets(reference),'原DC负端缺失'
        except AssertionError:print('PASS regression: test rejects the previous model with the missing DC return')
        else:raise AssertionError('Test failed to reject the known broken model')
    print('Electrical/mechanical startup has NOT been simulated by this Python check.')

if __name__=='__main__':main()
