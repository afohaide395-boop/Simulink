function report = run_model_validation
% 整机仿真验证，需要兼容原模型的MATLAB R2024b及SPS电力系统库。
folder=fileparts(mfilename('fullpath')); addpath(folder);
test_identification;
test_pair_matching;
test_voltage_ripple;
test_preview_outputs;
test_speedloop_sequence;
file=fullfile(folder,'reduce_emf_accuracy.slx');
if ~isfile(file),file=fullfile(folder,'reduce_emf_speedloop.slx');end
if ~isfile(file),file=fullfile(folder,'reduce_emf_identified.slx');end
[~,model]=fileparts(file);
load_system(file); set_param(model,'SimulationCommand','update');
out=sim(model,'StopTime','25','ReturnWorkspaceOutputs','on');
r=out.get('param_identification'); st=out.get('identification_state');
raw=out.get('id_raw'); filtered=out.get('id_filtered'); iqReference=out.get('id_iq_reference');
y=squeeze(r.signals.values);
if size(y,1)~=numel(r.time) && size(y,2)==numel(r.time),y=y.';end
assert(size(y,2)==31,'新版结果向量宽度应为31列。');
% 在收敛断言之前保存记录，失败时仍可追查电压纹波、漂移及状态。
save(fullfile(folder,'id_run_capture.mat'),'r','st','raw','filtered','iqReference');
report=struct('Ls_H',y(end,3),'psi_Wb',y(end,4),'Rs_ohm',y(end,6), ...
    'computed_Lpsi',y(end,5),'computed_Rs',y(end,7), ...
    'deltaA_rad',y(end,1),'deltaB_rad',y(end,2),'status',y(end,8), ...
    'condition',y(end,9),'max_voltage_drift',y(end,10), ...
    'max_current_mismatch',y(end,11),'RLS_iterations',y(end,12),'wait_reason',y(end,17), ...
    'voltage_rms_before_cycle_mean',y(end,18:19), ...
    'voltage_rms_after_cycle_mean',y(end,20:21), ...
    'voltage_drift_limit_ratio',y(end,22:23), ...
    'voltage_ripple_limit_ratio',y(end,24:25),'cycle_samples',y(end,26), ...
    'last_sampling_stage',y(end,27),'data_calibrated_R',y(end,28), ...
    'data_calibrated_voltage_error',y(end,29),'calibration_condition',y(end,30), ...
    'calibration_excitation',y(end,31));
disp(report);
figure('Name','虚拟反电动势辨识v9：电流及转速差分补偿后的实际参数曲线');
subplot(4,1,1); plot(st.time,st.signals.values); ylabel('state'); grid on;
subplot(4,1,2); plot(r.time,y(:,3)*1000); ylabel('Ls (mH)'); grid on;
subplot(4,1,3); plot(r.time,y(:,4)); ylabel('psi (Wb)'); grid on;
subplot(4,1,4); plot(r.time,y(:,12)); ylabel('RLS iteration'); xlabel('s'); grid on;
figure('Name','ud/uq: raw versus prefiltered');
for k=1:2
    subplot(2,1,k); plot(raw.time,raw.signals.values(:,k), ...
        filtered.time,filtered.signals.values(:,k));
    legend('raw (recorded at Ts)','50 Hz LPF at Ts'); grid on;
end
figure('Name','电压稳态判据：纹波与慢漂移分开诊断');
subplot(3,1,1); plot(r.time,y(:,18:21)); ylabel('纹波RMS/V');
legend('ud周期平均前','uq周期平均前','ud周期平均后','uq周期平均后'); grid on;
subplot(3,1,2); plot(r.time,y(:,22:23)); hold on; yline(1,'--');
ylabel('漂移/门限'); legend('ud','uq'); grid on;
subplot(3,1,3); plot(r.time,y(:,24:25)); hold on; yline(1,'--');
ylabel('剩余纹波/门限'); legend('ud','uq'); xlabel('时间/s'); grid on;
assert(any(st.signals.values(:)==9),'Sequence did not reach state 9.');
assert(report.computed_Lpsi==1, 'L/磁链数值未完成，请查看采样记录。');
assert(report.RLS_iterations==4000, 'Expected 4000 actual RLS updates.');
for state=[2 3 4 6 7 8]
    ts=st.time(st.signals.values(:)==state);
    assert(~isempty(ts),'Missing acquisition state.');
    ix=iqReference.time>=ts(1)+0.01 & iqReference.time<=ts(end)-0.01;
    hh=iqReference.signals.values(ix,:);
    assert(all(abs(hh(:,3))<1e-12), '辨识期间保持使能应始终为0。');
    assert(max(abs(hh(:,2)-hh(:,1)))<1e-8, ...
        'iq指令没有持续跟随速度PI输出。');
end
% 给出每段后0.2秒的速度变化，便于与旧版的转速上拱作对比；只记录不设门限。
report.stage_speed_stats=zeros(6,4);
stages=[2 3 4 6 7 8];
for k=1:numel(stages)
    times=st.time(st.signals.values(:)==stages(k));
    ix=filtered.time>=times(end)-.2 & filtered.time<=times(end);
    w=filtered.signals.values(ix,5);
    report.stage_speed_stats(k,:)=[stages(k),mean(w),min(w),max(w)];
end
disp('各段后0.2秒的速度统计：[阶段，we均值，we最小，we最大]，单位rad/s');
disp(report.stage_speed_stats);
% 误差只列入报告，不再以接近额定值作为保留结果或完成仿真的条件。
report.reference_error_fraction=[report.Ls_H/.00896-1,report.psi_Wb/.21-1,report.Rs_ohm/1.8665-1];
disp(report.reference_error_fraction);
fprintf('整机递推已完成，实际参数及误差已记录，不进行精度门限验收。\n');
end
