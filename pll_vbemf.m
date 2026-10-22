function [theta,we,id_offset,vfl_state] = pll_vbemf(Ed_hat,Eq_hat,IF_sensorless_mode)
%#codegen
%==================================================
% 文档式(12.7)、图12.2：反电动势相位误差 -> PI -> 角度积分。
% Ed_hat、eb端口现在分别接Ed_hat、Eq_hat，单位V，不再接磁链。
% 加入文献12.4.1的虚拟反电动势注入与六段采样状态机。
% 保留原有I/F切换输入及四路输出，外围控制连接保持完整。
%==================================================
persistent theta_z speed_i state tick stable speed_f amp
if isempty(theta_z)
    theta_z = 0;
    speed_i = 0;
    state=0; tick=0; stable=0; speed_f=0; amp=0;
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

% State output and sample timing are shared with identify_vbemf.m.
% Virtual voltage is added ONLY to Ed entering the PLL, never to plant voltage.
% Existing I/F mode uses 1 for sensorless control.
id_offset=0; injection=0;
if IF_sensorless_mode<0.5 || abs(Eq_hat)<0.5 || ~isfinite(Ed_hat+Eq_hat)
    state=0; tick=0; stable=0;
end
if state>0
    tick=tick+1;
    if tick>7000 && state<9
        state=state+1; tick=1;
        if state==6, amp=min(3,max(0.2,0.07*abs(Eq_hat))); end
    end
    if state>=5 && state<=8, id_offset=-1.0; end
    if state==3 || state==7, injection=amp;
    elseif state==4 || state==8, injection=-amp;
    end
end
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
% Arm only after sensorless operation has remained settled for 0.5 s. Each acquisition stage then lasts 0.7 s.
% The state transition becomes visible next tick, keeping state and injection aligned.
speed_f=speed_f+0.002*(we-speed_f);
if state==0 && IF_sensorless_mode>=0.5 && abs(we)>100 && ...
        abs(we-speed_f)<2 && abs(phase_error)<0.03 && abs(Eq_hat)>2
    stable=stable+1;
    if stable>=5000
        state=2; tick=0; stable=0;
        amp=min(3,max(0.2,0.07*abs(Eq_hat)));
    end
elseif state==0
    stable=0;
end
% State 9 latches completion until mode drops out; restoration of Id*=0 is immediate.
end