function test_diagnose_identification
% 用实际辨识函数产生记录，验证诊断脚本能重建真正使用的窗口和RLS结果。
folder=fileparts(mfilename('fullpath'));addpath(folder);rng(52);
for w=[220,50,-220]
    clear identify_vbemf
    stages=[2 3 4 6 7 8];per=6000;total=6*per+4401;Ts=1e-4;
    F=zeros(total,5);S=zeros(total,1);Y=zeros(total,27);t=(0:total-1)'*Ts;
    at=0;R=1.8665;L=.00896;psi=.21;
    for k=1:6
        g=1+(k>3);ids=[0 -1];angles=[.08 -.12];di=ids(g);qi=4;d=angles(g);
        if k==2 || k==5,d=d+.01;elseif k==3 || k==6,d=d-.01;end
        u0=[R*di-w*L*qi-w*psi*sin(d),R*qi+w*L*di+w*psi*cos(d),di,qi,w];
        for n=1:per
            at=at+1;u=u0+[.01*randn,.01*randn,.001*randn,.001*randn,.02*randn];
            F(at,:)=u;S(at)=stages(k);
            y=identify_vbemf(u(1),u(2),u(3),u(4),u(5),stages(k));Y(at,:)=y.';
        end
    end
    for n=1:4401
        at=at+1;F(at,5)=w;S(at)=9;
        y=identify_vbemf(0,0,0,0,w,9);Y(at,:)=y.';
    end
    assert(y(12)==4000);
    data=struct('id_filtered',record(t,F),'identification_state',record(t,S), ...
        'param_identification',record(t,Y));
    file=[tempname '.mat'];report=diagnose_identification(data,file);
    saved=load(file);delete(file);
    assert(isequal(size(saved.windows),[2000 5 6]));
    assert(max(abs(report.RLS_replay_difference))<1e-8,'重建记录与实际RLS数值不一致。');
    assert(max(abs(report.raw_angle_replay-y(1:2)))<1e-9,'重建位置误差与实际代码不一致。');
    assert(all(isfinite(report.angle_sensitivity_L_psi_per_rad(:))));
    assert(report.RLS_iterations==4000);
    fprintf('PASS 实际函数记录回放 w=%g，N=%g，最大参数重建差=%g\n', ...
        w,report.capture_timing(1,4),max(abs(report.RLS_replay_difference)));
    % 同样接受整机验证脚本保存的字段名，无需用户改名。
    if w==220
        alternate=struct('filtered',data.id_filtered,'st',data.identification_state,'r',data.param_identification);
        file=[tempname '.mat'];again=diagnose_identification(alternate,file);delete(file);
        assert(max(abs(again.RLS_replay_difference))<1e-8);
    end
end
end

function out=record(t,v)
out=struct('time',t,'signals',struct('values',v));
end
