function [theta,we,id_offset,vfl_state] = pll_vbemf(Ed_hat,Eq_hat,IF_sensorless_mode,sample_ack)
%#codegen
%==================================================
% 文档式(12.7)、图12.2：反电动势相位误差 -> PI -> 角度积分。
% Ed_hat、eb端口现在分别接Ed_hat、Eq_hat，单位V，不再接磁链。
% 虚拟反电动势注入与六段采样；Id切入退出用100毫秒斜坡。
% 保留原有I/F切换输入及四路输出，外围控制连接保持完整。
%==================================================
persistent theta_z speed_i state tick stable amp id_ramp
if isempty(theta_z)
    theta_z = 0;
    speed_i = 0;
    state=0; tick=0; stable=0; amp=0; id_ramp=0;
end
if ~isfinite(Ed_hat) || ~isfinite(Eq_hat) || ~isfinite(IF_sensorless_mode)
    Ed_hat=0; Eq_hat=0; IF_sensorless_mode=0;
end
Ts = 1e-4;                    % 原控制周期，s
wn = 50;                      % PLL带宽初值，rad/s；沿用原代码数值
kp = 2*0.707*wn;              % PI比例增益
ki = wn*wn;                   % PI积分增益
we_max = 1256;                % 原电角速度限幅，rad/s
E_min = 1e-3;                 % 反电动势不足时不更新相位误差，V

% 状态输出与采集函数共用同一控制周期；1表示已切换至无感控制。
% 虚拟电压仅加到PLL使用的Ed，不直接叠加到电机电压命令。
id_offset=0; injection=0;
if IF_sensorless_mode<0.5 || ~isfinite(Ed_hat+Eq_hat)
    state=0; tick=0; stable=0;
end
if state>0
    tick=tick+1;
    % 调试版每个采样阶段固定0.6秒；不等待稳态、工作点或条件数验收。
    % 确认口只保留非有限输入时的数值错误中止，不接受质量门限中止。
    if state>=2 && state<=8 && sample_ack<0
        state=10; tick=0;
    % 工作点切换共1秒，包含100毫秒Id斜坡和900毫秒恢复时间。
    elseif state==5 && tick>10000
        state=6; tick=1;
        amp=min(0.6,max(0.2,0.01*abs(Eq_hat)));
    elseif state>=2 && state<=8 && state~=5 && tick>6000
        state=state+1; tick=1;
    end
    if state>=5 && state<=8, id_offset=-1.0; end
    if state==3 || state==7, injection=amp;
    elseif state==4 || state==8, injection=-amp;
    end
end
% 正常实验时Id偏移按10 A/s变化，沿用100微秒控制周期。
% I/F复位或数值错误中止时立即清零，不延迟原有复位动作。
if IF_sensorless_mode<0.5 || state==10
    id_ramp=0;
else
    id_step=10*Ts;
    id_ramp=id_ramp+min(max(id_offset-id_ramp,-id_step),id_step);
end
id_offset=id_ramp;
vfl_state=int32(state);
Ed_pll=Ed_hat+injection;

% 文档式(12.7)，严格保留atan比值的正负转速语义。
% 分母正则化只用于避免Eq=0数值除零，不改为磁链q轴误差。
if Ed_hat*Ed_hat+Eq_hat*Eq_hat>E_min*E_min
    eq_safe = Eq_hat;
    if abs(eq_safe)<1e-6
        if eq_safe>=0
            eq_safe = 1e-6;
        else
            eq_safe = -1e-6;
        end
    end
    phase_error = atan(-Ed_pll/eq_safe);
else
    phase_error = 0;
end

% PI输出电角速度；限幅饱和时采用条件积分抑制积分累积。
we_unsat = speed_i+kp*phase_error;
we = min(max(we_unsat,-we_max),we_max);
if (we_unsat<=we_max && we_unsat>=-we_max) || ...
   (we_unsat>we_max && phase_error<0) || ...
   (we_unsat< -we_max && phase_error>0)
    speed_i = speed_i+ki*Ts*phase_error;
end
speed_i = min(max(speed_i,-we_max),we_max);

% 输出本周期角度，再以同周期速度推进下一拍角度。
% 外部一拍角度反馈和一拍速度反馈，避免代数环并保持采样对齐。
theta = theta_z;
theta_z = mod(theta_z+Ts*we,2*pi);
% 原有I/F切换至无感0.5秒后开始实验，不再增加相位、速度趋势验收。
% 此处触发的状态转换从下一拍输出，保持注入与状态标记一致。
if state==0 && IF_sensorless_mode>=0.5
    stable=stable+1;
    if stable>=5000
        state=2; tick=0; stable=0;
        amp=min(0.6,max(0.2,0.01*abs(Eq_hat)));
    end
elseif state==0
    stable=0;
end
% 状态9计算结果并将Id偏移斜坡恢复为0；状态10立即中止并清零。
end
