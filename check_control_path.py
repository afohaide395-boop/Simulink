"""直接读取SLX信号图，解析嵌套端口与Goto/From，核对iq指令来源。
用于检出速度PI断接或自回接，不能替代电机或Simulink整机仿真。"""
from pathlib import Path
import argparse,io,subprocess,zipfile,xml.etree.ElementTree as E

class SignalGraph:
    def __init__(self,content):
        self.blocks={};self.owners={};self.systems={};self.parents={};self.inputs={}
        with zipfile.ZipFile(io.BytesIO(content)) as z:
            for n in z.namelist():
                if n.startswith('simulink/systems/system_') and n.endswith('.xml'):
                    sid=Path(n).stem;r=E.fromstring(z.read(n));self.systems[sid]=r
                    for b in r.findall('Block'):
                        self.blocks[b.get('SID')]=b;self.owners[b.get('SID')]=sid
                        if b.find('System') is not None:self.parents[b.find('System').get('Ref')]=b.get('SID')
                    for line in r.findall('Line'):
                        if line.get('LineType')=='Connection':continue
                        src=line.find("P[@Name='Src']")
                        if src is None:continue
                        for d in line.findall(".//P[@Name='Dst']"):
                            assert d.text not in self.inputs,'Multiple drivers'
                            self.inputs[d.text]=src.text
    @staticmethod
    def prop(b,key,default=None):
        q=b.find(f"P[@Name='{key}']");return q.text if q is not None else default
    def goto_for(self,sid):
        b=self.blocks[sid];tag=self.prop(b,'GotoTag');scope=self.owners[sid]
        candidates=[k for k,v in self.blocks.items() if v.get('BlockType')=='Goto' and self.prop(v,'GotoTag')==tag and
                    (self.owners[k]==scope or self.prop(v,'TagVisibility')=='global')]
        assert len(candidates)==1,(sid,tag,candidates);return candidates[0]
    def evaluate(self,endpoint,values,active=None):
        if endpoint in values:return values[endpoint]
        active=set() if active is None else active
        assert endpoint not in active,'Cyclic signal dependency: '+endpoint
        active.add(endpoint)
        try:
            if '#in:' in endpoint:return self.evaluate(self.inputs[endpoint],values,active)
            sid,port=endpoint.split('#out:');port=int(port);b=self.blocks[sid];kind=b.get('BlockType')
            if kind=='From':return self.evaluate(self.goto_for(sid)+'#in:1',values,active)
            if kind=='SubSystem':
                child=b.find('System').get('Ref')
                out=[v for v in self.systems[child].findall('Block') if v.get('BlockType')=='Outport' and int(self.prop(v,'Port','1'))==port]
                assert len(out)==1;return self.evaluate(out[0].get('SID')+'#in:1',values,active)
            if kind=='Inport':return self.evaluate(self.parents[self.owners[sid]]+'#in:'+self.prop(b,'Port','1'),values,active)
            if kind=='Switch':
                control=self.evaluate(sid+'#in:2',values,active)
                take=1 if control>float(self.prop(b,'Threshold')) else 3
                return self.evaluate(sid+'#in:'+str(take),values,active)
            if kind=='Constant':
                val=self.prop(b,'Value');return 7.0 if val=='Iq_if' else float(val)
            raise AssertionError('Unsupported graph operation '+sid+' '+kind)
        finally:active.remove(endpoint)

def check(content):
    graph=SignalGraph(content)
    for mode in [0,1]:
        for hold in [0,1]:
            for speed in [-2.3,1.2,6.4]:
                values={'11242#out:1':speed,'12023#out:1':mode,'12231#out:1':hold,'12234#out:1':3.7}
                got=graph.evaluate('12010#out:1',values)
                expected=7.0 if mode==0 else (3.7 if hold else speed)
                assert abs(got-expected)<1e-12,(mode,hold,speed,got,expected)
    # 沿两层信号标签逐路追踪，确认辨识输入为预期的五路低通信号，避免dq错接。
    for i,name in enumerate(['ud_raw','uq_raw','id_raw','iq_raw','we_raw']):
        chartsrc=graph.inputs[f'12088#in:{i+1}'];sid=chartsrc.split('#')[0]
        assert graph.prop(graph.blocks[sid],'GotoTag')=='PID_'+name+'_f'
        ft=graph.goto_for(sid);assert graph.inputs[ft+'#in:1']==f'{12200+i}#out:1'
        fs=graph.inputs[f'{12200+i}#in:1'].split('#')[0]
        assert graph.prop(graph.blocks[fs],'GotoTag')=='PID_'+name
        rt=graph.goto_for(fs);assert graph.inputs[rt+'#in:1']==f'{12310+i}#out:1'
    assert graph.inputs['12560#in:1']=='12096#out:9'
    assert graph.prop(graph.blocks['12560'],'SampleTime')=='Ts' and graph.prop(graph.blocks['12560'],'InitialCondition','0')=='0'
    assert graph.inputs['12561#in:1']=='12560#out:1'
    assert graph.goto_for('12580')=='12561'
    assert graph.inputs['12073#in:4']=='12580#out:1'
    assert graph.inputs['12073::32#in:4']=='12073::38#out:1'
    print('PASS iq选择器及I/F连接正确；实际保持使能关闭由check_speed_control.py逐拍核对')
    print('PASS 五路低通映射正确；采样完成/数值错误反馈经一拍Ts延迟到PLL输入4，不作质量验收')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--model',type=Path,default=Path(__file__).with_name('reduce_emf_identified.slx'))
    ap.add_argument('--repo',type=Path,default=Path(__file__).resolve().parent);ap.add_argument('--self-test',action='store_true');args=ap.parse_args()
    check(args.model.read_bytes())
    if args.self_test:
        bad=subprocess.run(['git','-C',str(args.repo),'show','d706c27ec067504c0a9a451174eb194ea740ad34:reduce_emf_identified.slx'],capture_output=True,check=True).stdout
        g=SignalGraph(bad)
        try:g.evaluate('12010#out:1',{'11242#out:1':2.2,'12023#out:1':1,'12231#out:1':0,'12234#out:1':3.7})
        except AssertionError as e:
            assert 'Cyclic signal dependency' in str(e);print('PASS regression: previous selected-iq self-feedback is detected as a cyclic dependency')
        else:raise AssertionError('Previous iq wiring bug was not detected')

if __name__=='__main__':main()
