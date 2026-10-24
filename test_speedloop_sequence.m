function test_speedloop_sequence
% 执行交付的PLL函数，检查Id切入、退出及恢复时间，非整机仿真。
folder=fileparts(mfilename('fullpath'));addpath(folder);
clear pll_vbemf
Ts=1e-4;W=220;E=46.2;th=0;
last=0;state5=0;entry=zeros(10000,1);exit=zeros(1000,1);ne=0;nx=0;
states=false(11,1);
for n=1:100000
    t=(n-1)*Ts;delta=W*t-th;
    [theta,we,id,state]=pll_vbemf(-E*sin(delta),E*cos(delta),double(t>=1),0);
    th=mod(theta+Ts*we,2*pi);states(double(state)+1)=true;
    assert(abs(id-last)<=10*Ts+1e-12,'Id发生了超过斜率限制的跳变。');
    assert(id>=-1-1e-12 && id<=1e-12);
    if state==5
        state5=state5+1;ne=ne+1;entry(ne)=id;
    elseif state>=6 && state<=8
        assert(abs(id+1)<1e-12,'采样前Id尚未到达-1A。');
    elseif state==9 && nx<1000
        nx=nx+1;exit(nx)=id;
    end
    last=id;
end
assert(all(states([1 3:10])) && ~states(11));
assert(state5==10000 && ne==10000 && nx==1000);
assert(abs(entry(1)+.001)<1e-12 && abs(entry(500)+.5)<1e-12);
assert(all(abs(entry(1000:end)+1)<1e-12),'Id切入应在100毫秒内完成。');
assert(abs(exit(1)+.999)<1e-12 && abs(exit(500)+.5)<1e-12);
assert(abs(exit(1000))<1e-12,'Id退出应在100毫秒内完成。');
fprintf('PASS Id切入/退出均为100ms，斜率不超过10A/s；工作点恢复段1s，采样前已稳定\n');
% I/F复位时仍立即清零，保留原有模式复位语义。
clear pll_vbemf
for n=1:40000
    [~,~,id,state]=pll_vbemf(0,E,1,0);
    if state==5 && id<-.2,break;end
end
assert(state==5 && id<-.2);
[~,~,id,state]=pll_vbemf(0,E,0,0);
assert(state==0 && id==0);
fprintf('PASS 实验中途退出无感模式时，Id偏移立即复位为0\n');
end
