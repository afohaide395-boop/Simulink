"""比较交付模型与用户上传原件，检查改动范围、接线、端口和内嵌算法。"""
from pathlib import Path
from zipfile import ZipFile
import argparse, xml.etree.ElementTree as E

def canonical(el):return E.canonicalize(E.tostring(el,encoding='unicode'),strip_text=True)

def main():
    folder=Path(__file__).resolve().parent
    ap=argparse.ArgumentParser()
    ap.add_argument('--model',type=Path,default=folder/'reduce_emf_speedloop.slx')
    ap.add_argument('--reference',type=Path,default=folder/'tests/fixtures/reduce_emf_before_speedloop.slx')
    args=ap.parse_args()
    with ZipFile(args.reference) as old,ZipFile(args.model) as new:
        assert new.testzip() is None
        assert old.namelist()==new.namelist()
        expected={'simulink/systems/system_12400.xml','simulink/systems/system_12300.xml',
                  'simulink/stateflow/chart_65.xml'}
        changed={n for n in old.namelist() if old.read(n)!=new.read(n)}
        assert changed==expected,changed
        ids=set()
        for name in new.namelist():
            if name.endswith(('.xml','.rels')):E.fromstring(new.read(name))
            if name.startswith('simulink/systems/system_') and name.endswith('.xml'):
                before=E.fromstring(old.read(name));after=E.fromstring(new.read(name))
                blocks={b.get('SID'):b for b in after.findall('Block')}
                assert len(blocks)==len(after.findall('Block'))
                assert not ids.intersection(blocks);ids.update(blocks)
                assert [canonical(x) for x in before.findall('Line')]==[canonical(x) for x in after.findall('Line')]
                for b in before.findall('Block'):
                    assert b.get('SID') in blocks
                    assert b.find("P[@Name='Position']").text==blocks[b.get('SID')].find("P[@Name='Position']").text
                drivers=set()
                for line in after.findall('Line'):
                    src=line.find("P[@Name='Src']")
                    if src is not None:assert src.text.split('#')[0] in blocks
                    for dst in line.findall(".//P[@Name='Dst']"):
                        assert dst.text.split('#')[0] in blocks
                        assert dst.text not in drivers;drivers.add(dst.text)
        # 保持子系统除表达式和说明外全部不变。
        name='simulink/systems/system_12400.xml'
        a=E.fromstring(old.read(name));b=E.fromstring(new.read(name))
        control=b.find("Block[@SID='12230']")
        assert control.find("P[@Name='Expr']").text=='0*u'
        control.find("P[@Name='Expr']").text=a.find("Block[@SID='12230']/P[@Name='Expr']").text
        control.remove(control.find("P[@Name='Description']"))
        assert canonical(a)==canonical(b)
        # 采集面板只改既有说明文字，不改变参数辨识、滤波或输出维度。
        name='simulink/systems/system_12300.xml'
        a=E.fromstring(old.read(name));b=E.fromstring(new.read(name))
        for r in [a,b]:r.find("Annotation[@SID='12662']/P[@Name='Name']").text=''
        assert canonical(a)==canonical(b)
        # PLL接口、PI及角度积分保持原样，只改变Id输出和工作点恢复时间。
        name='simulink/stateflow/chart_65.xml'
        a=E.fromstring(old.read(name));b=E.fromstring(new.read(name))
        ca=a.find(".//P[@Name='script']");cb=b.find(".//P[@Name='script']")
        start='Ed_pll=Ed_hat+injection;';end='theta_z = mod(theta_z+Ts*we,2*pi);'
        assert ca.text[ca.text.index(start):ca.text.index(end)+len(end)]==cb.text[cb.text.index(start):cb.text.index(end)+len(end)]
        assert cb.text==(folder/'pll_vbemf.m').read_text().replace('pll_vbemf(','fcn(',1)
        ca.text='';cb.text='';assert canonical(a)==canonical(b)
        identify=E.fromstring(new.read('simulink/stateflow/chart_22.xml')).find(".//P[@Name='script']").text
        assert identify==(folder/'identify_vbemf.m').read_text().replace('identify_vbemf(','fcn(',1)
    print('PASS 仅改上传原件的3个资源：统一保持使能、Id斜坡/恢复时序、已有说明')
    print('PASS 所有模块位置、外部连线、端口、分支、电气网络和求解器配置与上传原件一致')
    print('PASS 速度PI本体及PLL的PI/角度积分不变；辨识器、滤波、无门限输出及27列结果保留')
    print('PASS SLX归档/XML完整、SID唯一、端点有效且无重复驱动；内嵌代码与交付源码一致')

if __name__=='__main__':main()
