"""核对精度版相对已运行正常v8的改动范围及模型接口，避免改坏控制与电气接线。"""
from pathlib import Path
from zipfile import ZipFile
import argparse,xml.etree.ElementTree as E

def canonical(x):return E.canonicalize(E.tostring(x,encoding='unicode'),strip_text=True)

def main():
    folder=Path(__file__).resolve().parent
    ap=argparse.ArgumentParser();ap.add_argument('--model',type=Path,default=folder/'reduce_emf_accuracy.slx')
    ap.add_argument('--reference',type=Path,default=folder/'tests/fixtures/reduce_emf_speedloop_v8.slx')
    args=ap.parse_args()
    with ZipFile(args.reference) as old,ZipFile(args.model) as new:
        assert new.testzip() is None;assert old.namelist()==new.namelist()
        changed={n for n in old.namelist() if old.read(n)!=new.read(n)}
        assert changed=={'simulink/stateflow/chart_22.xml','simulink/stateflow/chart_65.xml',
                         'simulink/systems/system_12300.xml','simulink/systems/system_root.xml'}
        ids=set()
        for name in new.namelist():
            if name.endswith(('.xml','.rels')):E.fromstring(new.read(name))
            if name.startswith('simulink/systems/system_') and name.endswith('.xml'):
                r=E.fromstring(new.read(name));blocks={b.get('SID'):b for b in r.findall('Block')}
                assert len(blocks)==len(r.findall('Block'));assert not ids.intersection(blocks);ids.update(blocks)
                drivers=set()
                for line in r.findall('Line'):
                    src=line.find("P[@Name='Src']")
                    if src is not None:assert src.text.split('#')[0] in blocks
                    for dst in line.findall(".//P[@Name='Dst']"):
                        assert dst.text.split('#')[0] in blocks;assert dst.text not in drivers;drivers.add(dst.text)
        # 根图只更新水位及说明，所有连接、控制器及电气参数均与v8一致。
        key='simulink/systems/system_root.xml';a=E.fromstring(old.read(key));b=E.fromstring(new.read(key))
        for r in [a,b]:
            r.find("P[@Name='SIDHighWatermark']").text=''
            r.find("Block[@SID='11765']/P[@Name='Description']").text=''
        assert canonical(a)==canonical(b)
        # PLL、辨识器的外部接口、Stateflow元数据保持原样。
        for number,name in [(22,'identify_vbemf'),(65,'pll_vbemf')]:
            key=f'simulink/stateflow/chart_{number}.xml';a=E.fromstring(old.read(key));b=E.fromstring(new.read(key))
            sa=a.find(".//P[@Name='script']");sb=b.find(".//P[@Name='script']")
            assert sb.text==(folder/(name+'.m')).read_text().replace(name+'(','fcn(',1)
            if number==65:
                begin='Ed_pll=Ed_hat+injection;';end='theta_z = mod(theta_z+Ts*we,2*pi);'
                assert sa.text[sa.text.index(begin):sa.text.index(end)+len(end)]==sb.text[sb.text.index(begin):sb.text.index(end)+len(end)]
            sa.text='';sb.text='';assert canonical(a)==canonical(b)
        # 采集面板只新增独立显示支路，原结果、采样确认、滤波及监测映射不变。
        key='simulink/systems/system_12300.xml';a=E.fromstring(old.read(key));b=E.fromstring(new.read(key))
        assert b.find("Block[@SID='12096']/P[@Name='Outputs']").text=='[1 1 1 1 1 1 1 8 1 1 10 4]'
        assert b.find("Block[@SID='12096']/PortCounts").get('out')=='12'
        for sid in ['12800','12801','12802']:b.remove(b.find(f"Block[@SID='{sid}']"))
        for line in list(b.findall('Line')):
            src=line.find("P[@Name='Src']")
            if src is not None and src.text in ['12096#out:12','12801#out:1']:b.remove(line)
        for r in [a,b]:
            r.find("P[@Name='Location']").text=''
            dm=r.find("Block[@SID='12096']");dm.find('PortCounts').set('out','11')
            for prop in ['Outputs','Position']:dm.find(f"P[@Name='{prop}']").text=''
            r.find("Block[@SID='12088']/P[@Name='Description']").text=''
            for sid in ['12662','12720']:r.find(f"Annotation[@SID='{sid}']/P[@Name='Name']").text=''
            for line in r.findall('Line'):
                if any(x.text=='12096#in:1' for x in line.findall("P[@Name='Dst']")):
                    points=line.find("P[@Name='Points']")
                    if points is not None:line.remove(points)
        assert canonical(a)==canonical(b)
    print('PASS v8速度/电流调节器、I/F、PLL PI/积分、电机与电源、求解器及任务配置保留')
    print('PASS 仅更新辨识代码、恢复段平滑标定激励、既有说明及独立显示支路；无控制通路改线')
    print('PASS 前27列及确认反馈映射保留，新增28～31标定诊断；31列分解、元数据与内嵌源码一致')
    print('PASS SLX/XML完整、SID唯一、端点及驱动有效')

if __name__=='__main__':main()
