"""沿SLX实际接线逐拍核对：所有实验阶段保持使能为0，iq及速度误差持续直通。"""
from pathlib import Path
import argparse, math
from check_control_path import SignalGraph

class ControlGraph(SignalGraph):
    def evaluate(self, endpoint, values, active=None):
        if endpoint in values:return values[endpoint]
        if '#out:' not in endpoint:return super().evaluate(endpoint,values,active)
        sid=endpoint.split('#')[0];b=self.blocks[sid];kind=b.get('BlockType')
        if kind not in ['Fcn','DataTypeConversion','Gain','Sum']:
            return super().evaluate(endpoint,values,active)
        active=set() if active is None else active
        assert endpoint not in active,'Cyclic signal dependency: '+endpoint
        active.add(endpoint)
        try:
            def inp(n):return self.evaluate(sid+'#in:'+str(n),values,active)
            if kind=='DataTypeConversion':return float(inp(1))
            if kind=='Fcn':
                expression=self.prop(b,'Expr');u=inp(1)
                if expression=='0*u':return 0*u
                if expression=='(u>=2)*(u<=4)+(u>=6)*(u<=8)':
                    return float(2<=u<=4 or 6<=u<=8)
                raise AssertionError('未识别的Fcn表达式：'+expression)
            if kind=='Gain':
                assert self.prop(b,'Gain')=='2*pi/60'
                return inp(1)*2*math.pi/60
            if kind=='Sum':
                signs=self.prop(b,'Inputs','++').replace('|','')
                return sum((1 if sign=='+' else -1)*inp(i+1) for i,sign in enumerate(signs))
        finally:active.remove(endpoint)

def check(content):
    g=ControlGraph(content)
    for stage in [0,2,3,4,5,6,7,8,9,10]:
        for mode in [0,1]:
            for rpm in [480,500,520]:
                flag=0;memory=3.7
                for command in [.8,1.2,-.4]:
                    values={'4007#out:1':command,'12023#out:1':mode,
                            '12400#in:2':stage,'11242#in:1':500,'11242#in:2':rpm,
                            '12231#out:1':flag,'12234#out:1':memory}
                    pi_error=g.evaluate('4007#in:1',values)
                    assert abs(pi_error-(500-rpm)*2*math.pi/60)<1e-12, '速度PI误差被暂停'
                    held=g.evaluate('12400#out:1',values)
                    assert abs(held-command)<1e-12,'iq没有持续跟随速度PI输出'
                    selected=g.evaluate('12010#out:1',values)
                    assert abs(selected-(7 if mode==0 else command))<1e-12
                    flag=g.evaluate('12231#in:1',values)
                    assert flag==0,'辨识保持使能仍在某些阶段激活'
                    memory=g.evaluate('12234#in:1',values)
    print('PASS 所有实验阶段逐拍核对：保持使能0，iq持续跟随速度PI，速度PI误差不置零；I/F仍选择7A')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--model',type=Path,default=Path(__file__).with_name('reduce_emf_speedloop.slx'))
    ap.add_argument('--reference',type=Path,default=Path(__file__).parent/'tests/fixtures/reduce_emf_before_speedloop.slx')
    args=ap.parse_args();check(args.model.read_bytes())
    try:check(args.reference.read_bytes())
    except AssertionError as err:
        assert any(s in str(err) for s in ['暂停','持续跟随','仍在某些阶段']),str(err)
        print('PASS 回归检查能检出用户上传模型中的冻结行为：'+str(err))
    else:raise AssertionError('原模型冻结行为未被检出')

if __name__=='__main__':main()
