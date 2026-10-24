function test_voltage_ripple
% 回归测试：周期平均正常工作，慢漂移及残余纹波只提示、不再拒绝采样。
% 所有输入均为50 Hz低通之后的信号，模拟仍然残留的电频及六倍频纹波。
folder=fileparts(mfilename('fullpath'));addpath(folder);
R=1.8665;L=.00896;psi=.21;states=[2 3 4 6 7 8];
% 按用户报告的量级构造平稳平台：ud为-3～-2 V，uq为45～45.5 V。
% 这是复现误拒绝条件的合成输入，不冒充用户上传的原始采样记录。
clear identify_vbemf
for n=1:30000
    a=210*n*1e-4;
    y=record_identification(-2.5+.5*sin(a),45.25+.25*sin(a+.3),0,4,210,2);
    if y(16)==2 || y(16)<0,break;end
end
assert(y(16)==2 && y(18)>.3 && all(y(22:25)<=1));
fprintf('PASS 用户波动量级：ud=-3～-2 V、uq=45～45.5 V，电压RMS %g/%g -> %g/%g V\n', ...
    y(18),y(19),y(20),y(21));
for w=[210.5 157 50 -210.5]
    clear identify_vbemf
    record_identification(0,0,0,0,w,0);total=0;
    for s=1:6
        g=1+(s>3);ids=[0 -1];deltas=[.08 -.12];d=deltas(g);
        if s==2 || s==5,d=d+.01;elseif s==3 || s==6,d=d-.01;end
        di=ids(g);qi=4;
        base=[R*di-w*L*qi-w*psi*sin(d),R*qi+w*L*di+w*psi*cos(d),di,qi,w];
        for n=1:30000
            total=total+1;a=w*total*1e-4;
            u=base+[2.5*sin(a)+1.2*sin(6*a+.4), ...
                1.4*sin(a+.7)+.8*sin(6*a), ...
                .02*sin(6*a),.03*sin(6*a+.3),.3*sin(6*a-.4)];
            y=record_identification(u(1),u(2),u(3),u(4),u(5),states(s));
            if y(16)==states(s) || y(16)<0,break;end
        end
        assert(y(16)==states(s),'稳定周期纹波被误判为不稳态。');
        assert(max(y(18:19))>1,'测试输入没有达到预期的大电压纹波。');
        assert(all(y(22:25)<=1),'未通过电压稳态门限却被接受。');
        assert(y(26)>=50 && y(26)<=1257);
    end
    diag=y(18:26);
    for n=1:4401,y=record_identification(0,0,0,0,w,9);end
    assert(y(5)==1 && y(7)==1 && y(12)==4000,'周期纹波下递推未完成。');
    % 此处独立叠加大测量谐波不满足完整电压方程，只验证滤波和不中止流程。
    % 对参数精度的验证使用包含相应电流响应的连续物理记录。
    fprintf('PASS 周期纹波 w=%g：L=%g，磁链=%g，Rs=%g；电压RMS %g/%g -> %g/%g V，N=%g\n', ...
        w,y(3),y(4),y(6),diag(1),diag(2),diag(3),diag(4),diag(9));
end
% 周期平均保留慢漂移；调试版仍保存窗口，-3仅出现在诊断提示中。
clear identify_vbemf
for n=1:6000
    y=record_identification(-10+.001*n,52,0,4,220,2);
    if y(16)==2 || y(16)<0,break;end
end
assert(y(8)==1 && y(17)==-3 && y(16)==2 && y(22)>1);
fprintf('PASS 电压慢漂移仅提示-3，采样仍完成；漂移/门限=%g\n',y(22));
% 80 Hz非同步纹波仅记录-13，不再把诊断变成运行错误。
clear identify_vbemf
for n=1:6000
    y=record_identification(-10+5*sin(2*pi*80*n*1e-4),52,0,4,220,2);
    if y(16)==2 || y(16)<0,break;end
end
assert(y(8)==1 && y(17)==-13 && y(16)==2 && all(y(22:23)<=1) && y(24)>1);
fprintf('PASS 非同步剩余纹波仅提示-13，采样仍完成；纹波/门限=%g\n',y(24));
end
