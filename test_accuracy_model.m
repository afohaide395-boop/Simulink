function test_accuracy_model(names)
% 回放连续电压方程生成的记录，执行交付的实际辨识器；非整机仿真。
folder=fileparts(mfilename('fullpath'));addpath(folder);
if nargin<1
    names={'nominal','inverter','different_motor','reverse','noise','speed_ripple','harmonics','positive_angles'};
end
for k=1:numel(names)
    clear identify_vbemf
    f=load(fullfile(folder,[names{k} '_fixture.mat']));
    for n=1:size(f.samples,1)
        u=f.samples(n,:);y=record_identification(u(1),u(2),u(3),u(4),u(5),f.states(n));
    end
    history=zeros(4401,4);
    for n=1:4401
        y=record_identification(0,0,0,0,200,9);
        history(n,:)=[y(3),y(4),y(6),y(12)];
    end
    truth=f.truth(:);err=[y(3);y(4);y(6)]./truth(1:3)-1;
    fprintf('%s：L=%g，磁链=%g，Rs=%g；相对误差=%g/%g/%g%%；标定R/V=%g/%g，条件数=%g\n', ...
        names{k},y(3),y(4),y(6),100*err(1),100*err(2),100*err(3),y(28),y(29),y(30));
    assert(y(8)==30 && y(12)==4000 && y(5)==1 && y(7)==1);
    assert(all(diff(history(402:end,4))==1));
    assert(norm(history(402,1:2)-history(end,1:2))>1e-5,'未执行实际参数递推。');
    assert(max(abs(err(1:2)))<.05,'L或磁链误差超出此连续方程测试的5%容限。');
    assert(abs(err(3))<.15,'Rs误差超出此测试的15%容限。');
end
fprintf('PASS %d组连续物理方程记录的实际函数辨识及4000次更新\n',numel(names));
end
