function report = diagnose_identification(inputFile,outputFile)
% 回放当前辨识所用的六段数据，仅诊断和导出，不修改模型或估计值。
% 无参数调用时读取工作区；也支持先前导出的id_data.mat或id_run_capture.mat。
if nargin<1 || isempty(inputFile)
    if evalin('base','exist(''id_filtered'',''var'') && exist(''identification_state'',''var'') && exist(''param_identification'',''var'')')
        data=struct('id_filtered',evalin('base','id_filtered'), ...
            'identification_state',evalin('base','identification_state'), ...
            'param_identification',evalin('base','param_identification'));
    elseif evalin('base','exist(''out'',''var'')')
        out=evalin('base','out');
        data=struct('id_filtered',out.get('id_filtered'), ...
            'identification_state',out.get('identification_state'), ...
            'param_identification',out.get('param_identification'));
    else
        [file,folder]=uigetfile('*.mat','选择辨识采样记录');
        assert(~isequal(file,0),'未选择采样记录。');
        data=load(fullfile(folder,file));
    end
elseif isstruct(inputFile)
    data=inputFile;
else
    data=load(inputFile);
end
if nargin<2 || isempty(outputFile)
    outputFile=fullfile(pwd,'identification_diagnosis.mat');
end
if isfield(data,'id_filtered')
    filtered=data.id_filtered;stage=data.identification_state;result=data.param_identification;
elseif isfield(data,'filtered') && isfield(data,'st') && isfield(data,'r')
    filtered=data.filtered;stage=data.st;result=data.r;
elseif isfield(data,'out')
    filtered=data.out.get('id_filtered');stage=data.out.get('identification_state');result=data.out.get('param_identification');
else
    error('记录中缺少五路低通数据、实验阶段或辨识结果。');
end
[tf,F]=unpack(filtered,5);[ts,S]=unpack(stage,1);[tr,Y]=unpack(result,27);
Ts=median(diff(tf));assert(isfinite(Ts) && Ts>0,'采样时间无效。');
assert(abs(Ts-1e-4)<1e-8,'此回放脚本对应100微秒采样的v7/v8模型。');
phases=[2 3 4 6 7 8];windows=zeros(2000,5,6);means=zeros(6,5);timing=zeros(6,4);
for k=1:6
    s=phases(k);first=find(S(:,1)==s,1,'first');
    assert(~isempty(first),['缺少采样阶段 ' num2str(s)]);
    accepted=find(Y(:,16)==s & Y(:,27)==s,1,'first');
    assert(~isempty(accepted),['阶段尚未保存完整窗口：' num2str(s)]);
    N=round(Y(accepted,26));assert(N>=1 && N<=1257,'周期平均长度无效。');
    tolerance=Ts*1e-3;
    ix=tf>=ts(first)-tolerance & tf<=tr(accepted)+tolerance;
    phaseData=F(ix,:);assert(size(phaseData,1)>=2000+N,'记录长度不足以重建周期平均。');
    % 模型在第1000拍重置平均器；保存窗口之前历史已填满。
    % 因而在该阶段整体作同长度因果平均，末2000点与模型的存储窗口一致。
    averaged=filter(ones(N,1)/N,1,phaseData);
    W=averaged(end-1999:end,:);windows(:,:,k)=W;means(k,:)=mean(W,1);
    timing(k,:)=[s,ts(first),tr(accepted),N];
end
finalVector=Y(end,:);angles=finalVector(1:2);scaleParameters=[.01;.2];
H=zeros(4000,2);z=zeros(4000,1);Amean=zeros(2,2);bmean=zeros(2,1);
rawAngles=zeros(2,1);normalizedAngles=zeros(2,1);pairChanges=zeros(2,5);
for g=1:2
    base=1+3*(g-1);W=windows(:,:,base);positive=windows(:,:,base+1);negative=windows(:,:,base+2);
    id=W(:,3);iq=W(:,4);we=W(:,5);d=angles(g);
    lambda=we.*(id.^2+iq.^2);eta=we.*(id*cos(d)+iq*sin(d));v=W(:,2).*id-W(:,1).*iq;
    raw=[lambda*.01,eta*.2];normalizer=max(sqrt(sum(raw.^2,2)),1e-12);
    rows=g:2:4000;H(rows,:)=raw./normalizer;z(rows)=v./normalizer;
    Amean(g,:)=[mean(lambda),mean(eta)];bmean(g)=mean(v);
    difference=mean(positive-negative,1);pairChanges(g,:)=difference;
    rawAngles(g)=wrap_half(atan2(difference(2),difference(1)));
    assert(all(abs(positive(:,5))>1e-12) && all(abs(negative(:,5))>1e-12),'转速为零，不能做归一化角度对比。');
    du=mean(positive(:,1:2)./positive(:,5)-negative(:,1:2)./negative(:,5),1);
    normalizedAngles(g)=wrap_half(atan2(du(2),du(1)));
end
% 该解析结果只回放原RLS：初始参数为0、P0=10I、遗忘因子1。
% 不用它替换模型输出，用于确认数据重建及初始协方差是否造成偏差。
ridgeTheta=(H.'*H+.1*eye(2))\(H.'*z);ridgeResult=scaleParameters.*ridgeTheta;
singularValues=svd(H);fullRank=singularValues(2)>max(size(H))*eps(singularValues(1));
if fullRank
    leastSquaresResult=scaleParameters.*(H\z);
else
    leastSquaresResult=[NaN;NaN];
end
% 仅计算当前点附近角度误差的参数灵敏度，不把额定值送入估计器。
sensitivity=NaN(2,2);
if rank(Amean)==2
    for g=1:2
        W=windows(:,:,1+3*(g-1));d=angles(g);
        derivative=mean(W(:,5).*(-W(:,3)*sin(d)+W(:,4)*cos(d)));
        rhs=zeros(2,1);rhs(g)=-finalVector(4)*derivative;
        sensitivity(:,g)=Amean\rhs;
    end
end
report=struct('stage_columns',{{'阶段','ud/V','uq/V','id/A','iq/A','we/(rad/s)'}}, ...
    'stage_means',[phases(:),means],'capture_timing',timing, ...
    'pair_difference_columns',{{'Delta_ud','Delta_uq','Delta_id','Delta_iq','Delta_we'}}, ...
    'positive_minus_negative',pairChanges,'final_L_psi',finalVector(3:4).', ...
    'final_Rs',finalVector(6),'final_angles',angles(:),'raw_angle_replay',rawAngles, ...
    'speed_normalized_angles_only',normalizedAngles,'RLS_ridge_replay',ridgeResult, ...
    'unregularized_least_squares',leastSquaresResult,'singular_values',singularValues, ...
    'angle_sensitivity_L_psi_per_rad',sensitivity,'RLS_replay_difference',ridgeResult-finalVector(3:4).', ...
    'status',finalVector(8),'diagnostic',finalVector(17),'RLS_iterations',finalVector(12));
save(outputFile,'report','windows','finalVector','Ts','-v7');
disp('六段实际存储窗口均值：[阶段 ud uq id iq we]');disp(report.stage_means);
disp('各组正注入减负注入：[Delta_ud Delta_uq Delta_id Delta_iq Delta_we]');disp(pairChanges);
disp('原模型[L,磁链]及RLS解析回放，两行应接近：');disp([report.final_L_psi.';ridgeResult.']);
disp('原始差分角度/仅速度归一化角度（后者尚未补偿电流项）：');disp([rawAngles,normalizedAngles]);
fprintf('诊断文件已保存：%s\n',outputFile);
end

function [t,v]=unpack(record,width)
% 同时支持模型原有Structure With Time以及timeseries记录。
if isa(record,'timeseries')
    t=record.Time(:);v=squeeze(record.Data);
else
    t=record.time(:);v=squeeze(record.signals.values);
end
if size(v,1)~=numel(t) && size(v,2)==numel(t),v=v.';end
assert(size(v,1)==numel(t) && size(v,2)==width,'记录维度不匹配。');
end

function d=wrap_half(d)
% 与原辨识器相同的atan比值分支。
if d>pi/2,d=d-pi;elseif d< -pi/2,d=d+pi;end
end
