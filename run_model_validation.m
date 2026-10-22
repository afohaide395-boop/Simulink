function report = run_model_validation
% Full Simulink run. This requires MATLAB R2024b-compatible SPS libraries.
folder=fileparts(mfilename('fullpath')); addpath(folder);
test_identification;
file=fullfile(folder,'reduce_emf_identified.slx'); [~,model]=fileparts(file);
load_system(file); set_param(model,'SimulationCommand','update');
out=sim(model,'StopTime','10','ReturnWorkspaceOutputs','on');
r=out.get('param_identification'); st=out.get('identification_state');
raw=out.get('id_raw'); filtered=out.get('id_filtered'); hold=out.get('id_iq_reference');
y=squeeze(r.signals.values);
if size(y,2)~=15 && size(y,1)==15, y=y.'; end
assert(size(y,2)==15,'Unexpected result width.');
% Save evidence BEFORE any assertion about convergence; failed runs are useful.
save(fullfile(folder,'id_run_capture.mat'),'r','st','raw','filtered','hold');
report=struct('Ls_H',y(end,3),'psi_Wb',y(end,4),'Rs_ohm',y(end,6), ...
    'valid_Lpsi',y(end,5),'valid_Rs',y(end,7), ...
    'deltaA_rad',y(end,1),'deltaB_rad',y(end,2),'status',y(end,8), ...
    'condition',y(end,9),'max_voltage_drift',y(end,10), ...
    'max_current_mismatch',y(end,11),'RLS_iterations',y(end,12));
disp(report);
figure('Name','VBEMF v2: actual RLS and filtered acquisition');
subplot(4,1,1); plot(st.time,st.signals.values); ylabel('state'); grid on;
subplot(4,1,2); plot(r.time,y(:,3)*1000); ylabel('Ls (mH)'); grid on;
subplot(4,1,3); plot(r.time,y(:,4)); ylabel('psi (Wb)'); grid on;
subplot(4,1,4); plot(r.time,y(:,12)); ylabel('RLS iteration'); xlabel('s'); grid on;
figure('Name','ud/uq: raw versus prefiltered');
for k=1:2
    subplot(2,1,k); plot(raw.time,raw.signals.values(:,k), ...
        filtered.time,filtered.signals.values(:,k));
    legend('raw (recorded at Ts)','50 Hz before sampling'); grid on;
end
assert(any(st.signals.values(:)==9),'Sequence did not reach state 9.');
assert(report.valid_Lpsi==1, 'L/psi rejected: inspect status and id_run_capture.mat.');
assert(report.RLS_iterations==4000, 'Expected 4000 actual RLS updates.');
for state=[2 3 4 6 7 8]
    ts=st.time(st.signals.values(:)==state);
    assert(~isempty(ts),'Missing acquisition state.');
    ix=hold.time>=ts(1)+0.01 & hold.time<=ts(end)-0.01;
    hh=hold.signals.values(ix,:);
    assert(all(hh(:,3)>.5) && max(hh(:,2))-min(hh(:,2))<1e-8, ...
        'Iq reference was not held constant during the acquisition triplet.');
end
% Compare against the ORIGINAL supplied model parameters; not estimator priors.
assert(abs(report.Ls_H/.00896-1)<.10,'L error exceeds 10%%.');
assert(abs(report.psi_Wb/.21-1)<.10,'Flux error exceeds 10%%.');
assert(report.valid_Rs==1,'Rs is not reliably observable; L/psi may still be valid.');
assert(abs(report.Rs_ohm/1.8665-1)<.30,'Rs error exceeds 30%%.');
fprintf('PASS full-model run, genuine RLS updates, current-reference hold and parameter checks.\n');
end
