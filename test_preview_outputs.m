function test_preview_outputs
% 验证调试版确实不按物理范围或注入强度门限隐藏实际递推值。
folder=fileparts(mfilename('fullpath'));addpath(folder);
for mode=1:2
    clear identify_vbemf
    L=.00896;psi=.21;R=1.8665;shift=.01;
    if mode==1,L=-.005;R=-2;end
    if mode==2,shift=.00005;end
    states=[2 3 4 6 7 8];w=220;
    for s=1:6
        g=1+(s>3);ids=[0 -1];angles=[.08 -.12];di=ids(g);qi=4;d=angles(g);
        if s==2 || s==5,d=d+shift;elseif s==3 || s==6,d=d-shift;end
        ud=R*di-w*L*qi-w*psi*sin(d);uq=R*qi+w*L*di+w*psi*cos(d);
        for n=1:6000
            y=identify_vbemf(ud,uq,di,qi,w,states(s));
            if y(16)==states(s),break;end
        end
        assert(y(16)==states(s));
    end
    for n=1:4401,y=identify_vbemf(0,0,0,0,w,9);end
    assert(y(8)==30 && y(12)==4000 && all(isfinite(y([3 4 6]))));
    assert(abs(y(3)/L-1)<.01 && abs(y(4)/psi-1)<.01 && abs(y(6)/R-1)<.02);
    if mode==1,assert(y(3)<0 && y(6)<0);end
    if mode==2,assert(y(17)==-7);end
    fprintf('PASS 不隐藏实际估计 mode=%d：L=%g，磁链=%g，Rs=%g，运行码=%g，提示=%g\n', ...
        mode,y(3),y(4),y(6),y(8),y(17));
end
% 电阻消元分母确实为零时保留NaN，不制造一个看起来正常的电阻值。
clear identify_vbemf
for state=[2 3 4 6 7 8]
    for n=1:6000
        y=identify_vbemf(0,0,0,0,0,state);
        if y(16)==state,break;end
    end
end
for n=1:4401,y=identify_vbemf(0,0,0,0,0,9);end
assert(y(8)==30 && y(12)==4000 && y(7)==0 && isnan(y(6)));
fprintf('PASS 仅保留数值除零保护：分母为零的Rs为NaN，其余递推仍完成\n');
end
