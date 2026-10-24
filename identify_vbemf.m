function [ ...
    Ls_hat, psi_f_hat, Rs_hat, ...
    delta_theta_A, delta_theta_B, Lpsi_done, ...
    Rs_done, ident_status, regression_condition, ...
    ud_uq_drift, id_iq_deviation, rls_updates, ...
    rls_innovation, Rs_denominator_ratio, stage_samples, ...
    sample_ack, diagnostic_code, ud_rms_before, ...
    uq_rms_before, ud_rms_after, uq_rms_after, ...
    ud_drift_ratio, uq_drift_ratio, ud_ripple_ratio, ...
    uq_ripple_ratio, cycle_samples, last_sample_stage, ...
    Rs_cal, V_inverter_cal, calibration_condition, ...
    calibration_strength, delta_theta_A_history, delta_theta_B_history ...
    ] = identify_vbemf(u_d, u_q, i_d, i_q, omega_e, experiment_stage)
    %#codegen
    % 论文式12.25～12.28：两组位置误差与逐样本递推最小二乘辨识。
    % 主要输出：Ls_hat/H，psi_f_hat/Wb，Rs_hat/欧姆，两组delta_theta/rad。
    % scaled_Ls_psi保存[Ls/0.01; psi_f/0.2]，缩放仅用于改善回归数值尺度。
    % 输入为五路同周期、同系数的50 Hz低通信号，采样周期均为100微秒。
    % 在辨识支路内对五路信号使用相同的一电周期滑动平均，抑制周期纹波。
    % 调试版：窗口填满即保存；稳态、工作点、角度及条件数仅提示，不阻断。
    % Lpsi_done、Rs_done表示数值计算完成，精度仍需结合电机仿真核对。
    % 用采样积分的中心变化标定R和逆变器等效压降，不使用额定参数填充结果。
    % 电感、电阻、磁链、两组角度及诊断量分别输出，日志列号由记录模块管理。
    % 采样窗口与实验状态。
    persistent dq_samples dq_stage_mean dq_window raw_ud_uq_window
    persistent stage_sample_count previous_stage stage_ready
    persistent window_index window_fill identification_finished estimate

    % 电周期平均器。
    persistent cycle_dq_buffer cycle_dq_sum cycle_index cycle_count cycle_length

    % 位置误差和电感/磁链递推状态。
    persistent delta_theta delta_theta_A_trace delta_theta_B_trace
    persistent angle_iteration angle_preprocessing_done ud_uq_difference_sum
    persistent scaled_Ls_psi rls_covariance rls_iteration data_checked

    % 静止坐标系积分与电阻/逆变器压降标定。
    persistent alpha_beta_integrals previous_alpha_beta_values previous_omega_e
    persistent theta_e_integrated has_previous_sample integral_window integral_samples
    persistent harmonic_normal_matrix harmonic_rhs dq_difference_moments
    persistent Rs_V_inverter_calibration

    % 一、初始化固定尺寸的状态与采样缓存。
    if isempty(stage_sample_count)
        dq_samples = zeros(2000, 5, 6);
        stage_sample_count = 0;
        previous_stage = -1;
        stage_ready = false(6, 1);
        estimate = initial_estimates();
        delta_theta = zeros(2, 1);
        delta_theta_A_trace = zeros(400, 1);
        delta_theta_B_trace = zeros(400, 1);
        identification_finished = false;
        angle_iteration = 0;
        angle_preprocessing_done = false;
        ud_uq_difference_sum = zeros(2, 2);
        dq_stage_mean = zeros(6, 5);
        scaled_Ls_psi = zeros(2, 1);
        rls_covariance = 10*eye(2);
        rls_iteration = 0;
        data_checked = false;
        dq_window = zeros(2000, 5);
        window_index = 0;
        window_fill = 0;
        cycle_dq_buffer = zeros(1257, 5);
        cycle_dq_sum = zeros(1, 5);
        cycle_index = 0;
        cycle_count = 0;
        cycle_length = 1;
        raw_ud_uq_window = zeros(2000, 2);
        alpha_beta_integrals = zeros(1, 6);
        previous_alpha_beta_values = zeros(1, 6);
        previous_omega_e = 0;
        theta_e_integrated = 0;
        has_previous_sample = false;
        integral_window = zeros(2000, 8);
        integral_samples = zeros(2000, 8, 2);
        harmonic_normal_matrix = zeros(3, 3, 2);
        harmonic_rhs = zeros(3, 6, 2);
        dq_difference_moments = zeros(2, 8);
        Rs_V_inverter_calibration = zeros(2, 1);
    end
    % 二、检查输入并处理实验阶段切换。
    % dq数据的列顺序统一为：[u_d, u_q, i_d, i_q, omega_e]。
    dq_sample = [u_d u_q i_d i_q omega_e];
    if ~all(isfinite(dq_sample)) || ~isfinite(experiment_stage)
        stage_ready(:) = false;
        estimate = initial_estimates();
        delta_theta = zeros(2, 1);
        estimate.ident_status = -1;
        estimate.diagnostic_code = -1;
        if experiment_stage>=2 && experiment_stage<=8
            estimate.sample_ack = -double(experiment_stage);
        end
        identification_finished = true;
    else
        if experiment_stage~=previous_stage
            stage_sample_count = 0;
            previous_stage = experiment_stage;
            window_index = 0;
            window_fill = 0;
            estimate.sample_ack = 0;
            cycle_dq_buffer(:) = 0;
            cycle_dq_sum(:) = 0;
            cycle_index = 0;
            cycle_count = 0;
            cycle_length = 1;
            if experiment_stage==0 || experiment_stage==2
                stage_ready(:) = false;
                estimate = initial_estimates();
                delta_theta = zeros(2, 1);
                delta_theta_A_trace(:) = 0;
                delta_theta_B_trace(:) = 0;
                identification_finished = false;
                angle_iteration = 0;
                angle_preprocessing_done = false;
                ud_uq_difference_sum(:) = 0;
                scaled_Ls_psi(:) = 0;
                rls_covariance = 10*eye(2);
                rls_iteration = 0;
                data_checked = false;
                alpha_beta_integrals(:) = 0;
                previous_alpha_beta_values(:) = 0;
                previous_omega_e = 0;
                theta_e_integrated = 0;
                has_previous_sample = false;
                integral_window(:) = 0;
                integral_samples(:) = 0;
                harmonic_normal_matrix(:) = 0;
                harmonic_rhs(:) = 0;
                dq_difference_moments(:) = 0;
                Rs_V_inverter_calibration(:) = 0;
            end
        end
        % 三、按阶段采样：两组工作点各保留基准、正注入、负注入窗口。
        sample_stage_index = 0;
        if experiment_stage==2
            sample_stage_index = 1;
        elseif experiment_stage==3
            sample_stage_index = 2;
        elseif experiment_stage==4
            sample_stage_index = 3;
        elseif experiment_stage==6
            sample_stage_index = 4;
        elseif experiment_stage==7
            sample_stage_index = 5;
        elseif experiment_stage==8
            sample_stage_index = 6;
        end
        % 连续积分跨越所有实验阶段，包括Id切换恢复段。整体旋转初相位不影响标定。
        % 原五路低通已经匹配；这里使用低通后的信号，避免重置周期平均器破坏积分连续性。
        integral_features = zeros(1, 8);
        if experiment_stage>=2 && experiment_stage<=8
            if has_previous_sample
                theta_e_integrated = mod(theta_e_integrated+0.5e-4*(previous_omega_e+omega_e), 2*pi);
            end
            cos_theta_e = cos(theta_e_integrated);
            sin_theta_e = sin(theta_e_integrated);
            voltage_current_alpha_beta = [ ...
                u_d*cos_theta_e-u_q*sin_theta_e, ...
                u_d*sin_theta_e+u_q*cos_theta_e, ...
                i_d*cos_theta_e-i_q*sin_theta_e, ...
                i_d*sin_theta_e+i_q*cos_theta_e];
            current_amplitude = max(hypot(i_d, i_q), 1e-12);
            alpha_beta_values = [voltage_current_alpha_beta, (4/pi)*voltage_current_alpha_beta(3:4)/current_amplitude];
            if has_previous_sample
                alpha_beta_integrals = alpha_beta_integrals+0.5e-4*(previous_alpha_beta_values+alpha_beta_values);
            end
            previous_alpha_beta_values = alpha_beta_values;
            previous_omega_e = omega_e;
            has_previous_sample = true;
            integral_features = [alpha_beta_integrals, cos_theta_e, sin_theta_e];
        end
        if sample_stage_index>0 && ~stage_ready(sample_stage_index) && ~identification_finished
            estimate.ident_status = 1;
            stage_sample_count = stage_sample_count+1;
            estimate.stage_samples = stage_sample_count;
            estimate.last_sample_stage = double(experiment_stage);
            % 阶段开始0.1秒后，用滤波后的电角速度确定一电周期长度并锁存。
            % N约等于2π/(|omega_e|*Ts)。只对辨识数据处理，不改变控制环及任务周期。
            % 每次切换阶段均清空平均器，防止混入上一个注入状态的数据。
            if stage_sample_count==1000
                cycle_length = min(1257, max(50, round(2*pi/(max(abs(omega_e), 50)*1e-4))));
                cycle_dq_buffer(:) = 0;
                cycle_dq_sum(:) = 0;
                cycle_index = 0;
                cycle_count = 0;
            end
            dq_cycle_mean = dq_sample;
            if stage_sample_count>=1000
                cycle_index = mod(cycle_index, cycle_length)+1;
                cycle_dq_sum = cycle_dq_sum+dq_sample-cycle_dq_buffer(cycle_index, :);
                cycle_dq_buffer(cycle_index, :) = dq_sample;
                cycle_count = min(cycle_length, cycle_count+1);
                dq_cycle_mean = cycle_dq_sum/cycle_count;
            end
            estimate.cycle_samples = cycle_length;
            % 等待0.4秒后保存最后0.2秒窗口；与原每段6000拍的时序匹配。
            % 调试版不再等待窗口通过验收；窗口填满后即存储，PLL按固定时长推进。
            if stage_sample_count>4000
                window_index = mod(window_index, 2000)+1;
                dq_window(window_index, :) = dq_cycle_mean;
                raw_ud_uq_window(window_index, :) = dq_sample(1:2);
                integral_window(window_index, :) = integral_features;
                window_fill = min(2000, window_fill+1);
            end
            if window_fill==2000 && mod(stage_sample_count, 200)==0
                dq_window_ordered = zeros(2000, 5);
                for sample_index = 1:2000
                    ordered_index = mod(window_index+sample_index-1, 2000)+1;
                    dq_window_ordered(sample_index, :) = dq_window(ordered_index, :);
                    if sample_stage_index==1 || sample_stage_index==4
                        integral_samples(sample_index, :, 1+(sample_stage_index==4)) = integral_window(ordered_index, :);
                    end
                end
                [dq_mean, diagnostic_reason, drift, voltage_diagnostics] = diagnose_dq_window(dq_window_ordered);
                estimate.ud_uq_drift = drift;
                raw_voltage_mean = sum(raw_ud_uq_window, 1)/2000;
                raw_voltage_rms = zeros(2, 1);
                for quantity_index = 1:2
                    raw_voltage_rms(quantity_index) = ...
                        sqrt(sum((raw_ud_uq_window(:, quantity_index)-raw_voltage_mean(quantity_index)).^2)/2000);
                end
                estimate.ud_rms_before = raw_voltage_rms(1);
                estimate.uq_rms_before = raw_voltage_rms(2);
                estimate.ud_rms_after = voltage_diagnostics(1);
                estimate.uq_rms_after = voltage_diagnostics(2);
                estimate.ud_drift_ratio = voltage_diagnostics(3);
                estimate.uq_drift_ratio = voltage_diagnostics(4);
                estimate.ud_ripple_ratio = voltage_diagnostics(5);
                estimate.uq_ripple_ratio = voltage_diagnostics(6);
                if sample_stage_index~=1 && sample_stage_index~=4
                    baseline_stage = 1+3*(sample_stage_index>3);
                    current_deviation = norm(dq_mean(3:4)-dq_stage_mean(baseline_stage, 3:4));
                    estimate.id_iq_deviation = max(estimate.id_iq_deviation, current_deviation);
                    if current_deviation>max(0.02, 0.005*norm(dq_stage_mean(baseline_stage, 3:4))) || ...
                            abs(dq_mean(5)-dq_stage_mean(baseline_stage, 5))>max(1, 0.005*abs(dq_stage_mean(baseline_stage, 5)))
                        if diagnostic_reason==0
                            diagnostic_reason = -6;
                        end
                    end
                end
                % 正负窗口差异仅记录提示，按用户要求先输出实际数据的辨识曲线。
                if sample_stage_index==3 || sample_stage_index==6
                    positive_stage = sample_stage_index-1;
                    if norm(dq_mean(3:4)-dq_stage_mean(positive_stage, 3:4))>max(0.003, 0.0005*norm(dq_stage_mean(positive_stage, 3:4))) || ...
                            abs(dq_mean(5)-dq_stage_mean(positive_stage, 5))>max(0.02, 0.0001*abs(dq_stage_mean(positive_stage, 5)))
                        if diagnostic_reason==0
                            diagnostic_reason = -12;
                        end
                    end
                end
                if estimate.diagnostic_code==0 && diagnostic_reason~=0
                    estimate.diagnostic_code = diagnostic_reason;
                end
                dq_samples(:, :, sample_stage_index) = dq_window_ordered;
                dq_stage_mean(sample_stage_index, :) = dq_mean;
                stage_ready(sample_stage_index) = true;
                estimate.sample_ack = double(experiment_stage);
            end
        end
        if experiment_stage==10 && ~identification_finished
            estimate.ident_status = -10;
            estimate.diagnostic_code = -10;
            identification_finished = true;
        end
        % 四、先递推角度，再由积分量标定电阻及逆变器等效压降。
        if experiment_stage==9 && ~identification_finished
            if ~all(stage_ready)
                estimate.ident_status = -2;
                estimate.diagnostic_code = -2;
                identification_finished = true;
            elseif ~data_checked
                data_checked = true;
                estimate.ident_status = 2;
            elseif ~angle_preprocessing_done
                % 位置误差更新400次，每次消耗五个尚未处理的存储样本。
                angle_iteration = angle_iteration+1;
                for group_index = 1:2
                    positive_stage = 2+3*(group_index-1);
                    negative_stage = positive_stage+1;
                    for sample_index = (angle_iteration-1)*5+1:angle_iteration*5
                        ud_uq_difference_sum(group_index, 1) = ...
                            ud_uq_difference_sum(group_index, 1) ...
                            + dq_samples(sample_index, 1, positive_stage) ...
                            - dq_samples(sample_index, 1, negative_stage);
                        ud_uq_difference_sum(group_index, 2) = ...
                            ud_uq_difference_sum(group_index, 2) ...
                            + dq_samples(sample_index, 2, positive_stage) ...
                            - dq_samples(sample_index, 2, negative_stage);
                        dq_positive = dq_samples(sample_index, :, positive_stage);
                        dq_negative = dq_samples(sample_index, :, negative_stage);
                        omega_e_positive = avoid_zero_speed(dq_positive(5));
                        omega_e_negative = avoid_zero_speed(dq_negative(5));
                        inverter_dq_positive = (4/pi)*dq_positive(3:4)/max(hypot(dq_positive(3), dq_positive(4)), 1e-12);
                        inverter_dq_negative = (4/pi)*dq_negative(3:4)/max(hypot(dq_negative(3), dq_negative(4)), 1e-12);
                        dq_difference_moments(group_index, :) = dq_difference_moments(group_index, :) + [ ...
                            dq_positive(1:2)/omega_e_positive-dq_negative(1:2)/omega_e_negative, ...
                            dq_positive(3:4)/omega_e_positive-dq_negative(3:4)/omega_e_negative, ...
                            dq_positive(3:4)-dq_negative(3:4), ...
                            inverter_dq_positive/omega_e_positive-inverter_dq_negative/omega_e_negative];
                        harmonic_basis = [1;integral_samples(sample_index, 7, group_index);integral_samples(sample_index, 8, group_index)];
                        harmonic_normal_matrix(:, :, group_index) = ...
                            harmonic_normal_matrix(:, :, group_index)+harmonic_basis*harmonic_basis';
                        harmonic_rhs(:, :, group_index) = harmonic_rhs(:, :, group_index) ...
                            + harmonic_basis*integral_samples(sample_index, 1:6, group_index);
                    end
                    delta_theta_group = atan2(ud_uq_difference_sum(group_index, 2), ud_uq_difference_sum(group_index, 1));
                    if delta_theta_group>pi/2
                        delta_theta_group = delta_theta_group-pi;
                    elseif delta_theta_group< -pi/2
                        delta_theta_group = delta_theta_group+pi;
                    end
                    delta_theta(group_index) = delta_theta_group;
                    if group_index==1
                        delta_theta_A_trace(angle_iteration) = delta_theta_group;
                    else
                        delta_theta_B_trace(angle_iteration) = delta_theta_group;
                    end
                end
                if angle_iteration==400
                    angle_preprocessing_done = true;
                    % 在两个基准平台拟合积分量的常值、余弦、正弦分量。
                    % 磁链及相电流的稳态旋转分量零均值，中心差满足DeltaU = R*DeltaI+V*DeltaD。
                    [Rs_V_inverter_calibration, ...
                        estimate.calibration_condition, estimate.calibration_strength] = ...
                        calibrate_voltage_losses(harmonic_normal_matrix, harmonic_rhs);
                    estimate.Rs_cal = Rs_V_inverter_calibration(1);
                    estimate.V_inverter_cal = Rs_V_inverter_calibration(2);
                    for group_index = 1:2
                        delta_theta(group_index) = corrected_angle(dq_difference_moments(group_index, :), Rs_V_inverter_calibration, 0);
                    end
                    for group_index = 1:2
                        if norm(ud_uq_difference_sum(group_index, :))/2000<0.2 || abs(delta_theta(group_index))>0.6
                            if estimate.diagnostic_code==0
                                estimate.diagnostic_code = -7;
                            end
                        end
                    end
                    % 条件数仅用于诊断，不在此处预先求解参数，也不阻止递推。
                    parameter_regression_matrix = zeros(2, 2);
                    for group_index = 1:2
                        baseline_stage = 1+3*(group_index-1);
                        i_d_sample = dq_stage_mean(baseline_stage, 3);
                        i_q_sample = dq_stage_mean(baseline_stage, 4);
                        omega_e_sample = dq_stage_mean(baseline_stage, 5);
                        regression_row = [0.01*omega_e_sample*(i_d_sample^2+i_q_sample^2), ...
                            0.2*omega_e_sample*(i_d_sample*cos(delta_theta(group_index))+i_q_sample*sin(delta_theta(group_index)))];
                        parameter_regression_matrix(group_index, :) = regression_row/max(norm(regression_row), 1e-12);
                    end
                    regression_determinant = ...
                        parameter_regression_matrix(1, 1)*parameter_regression_matrix(2, 2)-parameter_regression_matrix(1, 2)*parameter_regression_matrix(2, 1);
                    regression_energy = sum(sum(parameter_regression_matrix.^2));
                    regression_condition_number = ...
                        (regression_energy+sqrt(max(0, regression_energy*regression_energy-4*regression_determinant*regression_determinant)))/(2*max(abs(regression_determinant), 1e-12));
                    estimate.regression_condition = regression_condition_number;
                    if regression_condition_number>100 || abs(regression_determinant)<1e-6
                        if estimate.diagnostic_code==0
                            estimate.diagnostic_code = -8;
                        end
                    end
                end
            else
                % 五、每拍读取一个基准样本，更新电感、磁链及电阻。
                % 按A1、B1、A2、B2……A2000、B2000顺序读取两组基准数据。
                % 每次调用只使用一个观测量，执行一次实际递推，不反复重放同一批数据。
                % 正负注入数据用于估计位置误差，基准数据用于L和磁链辨识。
                rls_iteration = rls_iteration+1;
                % 相同的转速、电流才可直接相减；用数据标定的损耗项和当前L递推值补偿。
                for group_index = 1:2
                    delta_theta(group_index) = corrected_angle( ...
                        dq_difference_moments(group_index, :), ...
                        Rs_V_inverter_calibration, 0.01*scaled_Ls_psi(1));
                end
                group_index = 1+mod(rls_iteration-1, 2);
                sample_index = floor((rls_iteration-1)/2)+1;
                baseline_stage = 1+3*(group_index-1);
                i_d_sample = dq_samples(sample_index, 3, baseline_stage);
                i_q_sample = dq_samples(sample_index, 4, baseline_stage);
                omega_e_sample = dq_samples(sample_index, 5, baseline_stage);
                regression_vector = [0.01*omega_e_sample*(i_d_sample^2+i_q_sample^2); ...
                    0.2*omega_e_sample*(i_d_sample*cos(delta_theta(group_index))+i_q_sample*sin(delta_theta(group_index)))];
                regression_scale = max(norm(regression_vector), 1e-12);
                normalized_regressor = regression_vector/regression_scale;
                normalized_voltage = ( ...
                    dq_samples(sample_index, 2, baseline_stage)*i_d_sample ...
                    - dq_samples(sample_index, 1, baseline_stage)*i_q_sample)/regression_scale;
                rls_gain = rls_covariance*normalized_regressor/(1+normalized_regressor'*rls_covariance*normalized_regressor);
                voltage_innovation = normalized_voltage-normalized_regressor'*scaled_Ls_psi;
                scaled_Ls_psi = scaled_Ls_psi+rls_gain*voltage_innovation;
                % 用Joseph形式更新协方差，测量方差及遗忘因子均为1。
                % 滑动平均后的相邻样本存在相关性，协方差不解释为统计置信区间。
                covariance_transition = eye(2)-rls_gain*normalized_regressor';
                rls_covariance = covariance_transition*rls_covariance*covariance_transition'+rls_gain*rls_gain';
                rls_covariance = 0.5*(rls_covariance+rls_covariance');
                estimate.Ls_hat = 0.01*scaled_Ls_psi(1);
                estimate.psi_f_hat = 0.2*scaled_Ls_psi(2);
                estimate.ident_status = 3;
                estimate.rls_updates = rls_iteration;
                estimate.rls_innovation = voltage_innovation;
                if mod(rls_iteration, 10)==0
                    delta_theta_A_trace(rls_iteration/10) = delta_theta(1);
                    delta_theta_B_trace(rls_iteration/10) = delta_theta(2);
                end
                % 每次L和磁链递推后同步计算Rs，不因物理范围或相对门限隐藏结果。
                [estimate.Rs_hat, estimate.Rs_denominator_ratio] = ...
                    estimate_stator_resistance(dq_stage_mean, delta_theta, ...
                        estimate.Ls_hat, estimate.psi_f_hat);
                if rls_iteration==4000
                    identification_finished = true;
                    estimate.Lpsi_done = double(all(isfinite(scaled_Ls_psi)));
                    estimate.Rs_done = double(isfinite(estimate.Rs_hat));
                    estimate.ident_status = 30;
                    if estimate.Rs_done==0 && estimate.diagnostic_code==0
                        estimate.diagnostic_code = -11;
                    end
                end
            end
        end

    end

    % 六、按物理量分别输出。
    Ls_hat = estimate.Ls_hat;
    psi_f_hat = estimate.psi_f_hat;
    Rs_hat = estimate.Rs_hat;
    delta_theta_A = delta_theta(1);
    delta_theta_B = delta_theta(2);
    Lpsi_done = estimate.Lpsi_done;
    Rs_done = estimate.Rs_done;
    ident_status = estimate.ident_status;
    regression_condition = estimate.regression_condition;
    ud_uq_drift = estimate.ud_uq_drift;
    id_iq_deviation = estimate.id_iq_deviation;
    rls_updates = estimate.rls_updates;
    rls_innovation = estimate.rls_innovation;
    Rs_denominator_ratio = estimate.Rs_denominator_ratio;
    stage_samples = estimate.stage_samples;
    sample_ack = estimate.sample_ack;
    diagnostic_code = estimate.diagnostic_code;
    ud_rms_before = estimate.ud_rms_before;
    uq_rms_before = estimate.uq_rms_before;
    ud_rms_after = estimate.ud_rms_after;
    uq_rms_after = estimate.uq_rms_after;
    ud_drift_ratio = estimate.ud_drift_ratio;
    uq_drift_ratio = estimate.uq_drift_ratio;
    ud_ripple_ratio = estimate.ud_ripple_ratio;
    uq_ripple_ratio = estimate.uq_ripple_ratio;
    cycle_samples = estimate.cycle_samples;
    last_sample_stage = estimate.last_sample_stage;
    Rs_cal = estimate.Rs_cal;
    V_inverter_cal = estimate.V_inverter_cal;
    calibration_condition = estimate.calibration_condition;
    calibration_strength = estimate.calibration_strength;
    delta_theta_A_history = delta_theta_A_trace;
    delta_theta_B_history = delta_theta_B_trace;
end

function omega_e_sample = avoid_zero_speed(omega_e_sample)
    % 仅防止归一化除零，不据此拒绝采样或限制估计值。
    if abs(omega_e_sample)<1e-9
        if omega_e_sample<0
            omega_e_sample = -1e-9;
        else
            omega_e_sample = 1e-9;
        end
    end
end

function delta_theta = corrected_angle(dq_moment, Rs_V_cal, Ls_estimate)
    % 补偿电阻压降、逆变器压降及电流差，得到正负注入的磁链差向量。
    flux_difference_d = dq_moment(1)-Rs_V_cal(1)*dq_moment(3)+Ls_estimate*dq_moment(6)-Rs_V_cal(2)*dq_moment(7);
    flux_difference_q = dq_moment(2)-Rs_V_cal(1)*dq_moment(4)-Ls_estimate*dq_moment(5)-Rs_V_cal(2)*dq_moment(8);
    delta_theta = atan2(flux_difference_q, flux_difference_d);
    if delta_theta>pi/2
        delta_theta = delta_theta-pi;
    elseif delta_theta< -pi/2
        delta_theta = delta_theta+pi;
    end
end

function [dq_mean, diagnostic_reason, max_voltage_drift, voltage_diagnostics] = diagnose_dq_window(dq_window_ordered)
    % 仅计算诊断提示，不拒绝窗口、不超时中止、不抑制参数输出。
    % 原始纹波先经相同的电周期平均；慢漂移不会被此单位直流增益滤波消除。
    % 诊断六列依次为：u_d/u_q剩余标准差、漂移门限比、纹波门限比。
    dq_mean = sum(dq_window_ordered, 1)/2000;
    diagnostic_reason = 0;
    max_voltage_drift = 0;
    voltage_diagnostics = zeros(6, 1);
    for quantity_index = 1:5
        standard_deviation = sqrt(sum((dq_window_ordered(:, quantity_index)-dq_mean(quantity_index)).^2)/2000);
        drift = abs(sum(dq_window_ordered(1:1000, quantity_index))/1000-sum(dq_window_ordered(1001:2000, quantity_index))/1000);
        if quantity_index<=2
            ripple_threshold = max(0.10, 0.005*abs(dq_mean(quantity_index)));
            drift_threshold = max(0.05, 0.002*norm(dq_mean(1:2)));
            max_voltage_drift = max(max_voltage_drift, drift);
            quantity_diagnostic_code = -13;
            voltage_diagnostics(quantity_index) = standard_deviation;
            voltage_diagnostics(2+quantity_index) = drift/drift_threshold;
            voltage_diagnostics(4+quantity_index) = standard_deviation/ripple_threshold;
            % -3只表示均值仍在慢漂移；-13表示周期平均后仍有过大纹波。
            if drift>drift_threshold && (diagnostic_reason==0 || diagnostic_reason==-13)
                diagnostic_reason = -3;
            end
        elseif quantity_index<=4
            ripple_threshold = max(0.01, 0.005*norm(dq_mean(3:4)));
            drift_threshold = max(0.015, 0.005*norm(dq_mean(3:4)));
            quantity_diagnostic_code = -4;
        else
            ripple_threshold = max(0.3, 0.002*abs(dq_mean(quantity_index)));
            drift_threshold = max(0.5, 0.003*abs(dq_mean(quantity_index)));
            quantity_diagnostic_code = -5;
        end
        if (standard_deviation>ripple_threshold || drift>drift_threshold) && diagnostic_reason==0
            diagnostic_reason = quantity_diagnostic_code;
        end
    end
    if abs(dq_mean(5))<50 && diagnostic_reason==0
        diagnostic_reason = -5;
    end
end


function estimate = initial_estimates()
    % NaN表示尚未计算，不用额定参数代替辨识结果。
    estimate.Ls_hat = NaN;
    estimate.psi_f_hat = NaN;
    estimate.Lpsi_done = 0;
    estimate.Rs_hat = NaN;
    estimate.Rs_done = 0;
    estimate.ident_status = 0;
    estimate.regression_condition = 0;
    estimate.ud_uq_drift = 0;
    estimate.id_iq_deviation = 0;
    estimate.rls_updates = 0;
    estimate.rls_innovation = 0;
    estimate.Rs_denominator_ratio = 0;
    estimate.stage_samples = 0;
    estimate.sample_ack = 0;
    estimate.diagnostic_code = 0;
    estimate.ud_rms_before = 0;
    estimate.uq_rms_before = 0;
    estimate.ud_rms_after = 0;
    estimate.uq_rms_after = 0;
    estimate.ud_drift_ratio = 0;
    estimate.uq_drift_ratio = 0;
    estimate.ud_ripple_ratio = 0;
    estimate.uq_ripple_ratio = 0;
    estimate.cycle_samples = 0;
    estimate.last_sample_stage = 0;
    estimate.Rs_cal = NaN;
    estimate.V_inverter_cal = NaN;
    estimate.calibration_condition = NaN;
    estimate.calibration_strength = NaN;
end

function [Rs_V_inverter_calibration, calibration_condition, calibration_strength] = ...
    calibrate_voltage_losses(harmonic_normal_matrix, harmonic_rhs)
    % 用两个基准平台的积分中心差，标定电阻及逆变器等效压降。
    integral_centers = zeros(2, 6);
    for group_index = 1:2
        harmonic_coefficients = (harmonic_normal_matrix(:, :, group_index)+1e-10*eye(3))\harmonic_rhs(:, :, group_index);
        integral_centers(group_index, :) = harmonic_coefficients(1, :);
    end
    integral_center_difference = integral_centers(2, :)-integral_centers(1, :);
    loss_regression_matrix = [integral_center_difference(3:4)', integral_center_difference(5:6)'];
    voltage_integral_difference = integral_center_difference(1:2)';
    calibration_strength = sqrt(sum(sum(loss_regression_matrix.^2)));
    loss_matrix_norm = max(calibration_strength, 1e-12);
    loss_regression_normalized = loss_regression_matrix/loss_matrix_norm;
    voltage_integral_difference = voltage_integral_difference/loss_matrix_norm;
    % 只用数值正则化处理退化数据，不设置参数范围或中止门限。
    Rs_V_inverter_calibration = ...
        (loss_regression_normalized'*loss_regression_normalized+1e-10*eye(2))\(loss_regression_normalized'*voltage_integral_difference);
    loss_matrix_determinant = ...
        loss_regression_normalized(1, 1)*loss_regression_normalized(2, 2)-loss_regression_normalized(1, 2)*loss_regression_normalized(2, 1);
    loss_matrix_energy = sum(sum(loss_regression_normalized.^2));
    if abs(loss_matrix_determinant)>0
        calibration_condition = ...
            (loss_matrix_energy+sqrt(max(0, loss_matrix_energy*loss_matrix_energy-4*loss_matrix_determinant*loss_matrix_determinant)))/(2*abs(loss_matrix_determinant));
    else
        calibration_condition = Inf;
    end
end

function [Rs_estimate, Rs_denominator_ratio] = ...
    estimate_stator_resistance(dq_stage_mean, delta_theta, Ls_estimate, psi_f_estimate)
    % 用两组q轴稳态电压方程消去等效压降，随当前电感和磁链更新电阻。
    inverter_q_gain = zeros(2, 1);
    uq_residual = zeros(2, 1);
    for group_index = 1:2
        baseline_stage = 1+3*(group_index-1);
        i_d_mean = dq_stage_mean(baseline_stage, 3);
        i_q_mean = dq_stage_mean(baseline_stage, 4);
        omega_e_mean = dq_stage_mean(baseline_stage, 5);
        inverter_q_gain(group_index) = (4/pi)*i_q_mean/max(hypot(i_d_mean, i_q_mean), 1e-8);
        uq_residual(group_index) = ...
            dq_stage_mean(baseline_stage, 2)-omega_e_mean*Ls_estimate*i_d_mean-omega_e_mean*psi_f_estimate*cos(delta_theta(group_index));
    end
    Rs_denominator = inverter_q_gain(2)*dq_stage_mean(1, 4)-inverter_q_gain(1)*dq_stage_mean(4, 4);
    Rs_denominator_scale = abs(inverter_q_gain(2)*dq_stage_mean(1, 4))+abs(inverter_q_gain(1)*dq_stage_mean(4, 4));
    Rs_denominator_ratio = abs(Rs_denominator)/max(Rs_denominator_scale, 1e-12);
    % 仅保留浮点除零保护；不可求时保留NaN，不用额定值代替结果。
    Rs_division_valid = abs(Rs_denominator)>64*eps(max(Rs_denominator_scale, 1));
    if Rs_division_valid
        Rs_estimate = (inverter_q_gain(2)*uq_residual(1)-inverter_q_gain(1)*uq_residual(2))/Rs_denominator;
    else
        Rs_estimate = NaN;
    end
end
