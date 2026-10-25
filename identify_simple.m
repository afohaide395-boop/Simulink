function [Ls, psi, Rs, da, db, doneL, doneR, run, n, ack, R0, V0] = ...
    identify_simple(ud, uq, id, iq, we, stage)
%#codegen
% ud、uq：V；id、iq：A；we：电角速度rad/s；采样周期Ts=100us。
% Ls：H；psi：Wb；Rs：欧姆；da、db：两组位置误差rad。
% stage=2/3/4为A组基准/正注入/负注入，6/7/8为B组，9为计算。
% run=0等待，1采样，2计算角度，3递推，30完成；负数表示数据缺失。
% n为实际递推次数，ack为采样完成的阶段号；未计算的参数为NaN。
stage = double(stage);

% 六段数据均按[ud uq id iq we]存储，每段取最后2000拍。
persistent dq_data dq_mean ready stage_z tick stopped
% 一电周期滑动平均。
persistent dq_buf dq_sum pos count N
% 跨阶段连续积分，用于修正角度计算中的电阻和逆变器压降。
persistent theta we_z ab_z int_ab int_data has_prev
% 正负注入差：电压/转速、电流/转速、电流、压降系数/转速。
persistent dU dU_w dI_w dI dV_w H F pre
% 辨识结果及RLS状态。x=[Ls/0.01; psi/0.2]仅做数值尺度归一化。
persistent L flux R delta done status ack_z loss x P it

%% 1. 初始化与阶段切换
if isempty(stage_z)
    stage_z = -1;
end
if isempty(dq_data) || ((stage == 0 || stage == 2) && stage ~= stage_z)
    dq_data = zeros(2000, 5, 6);
    dq_mean = zeros(6, 5);
    ready = false(6, 1);
    stopped = false;
    tick = 0;
    dq_buf = zeros(1257, 5);
    dq_sum = zeros(1, 5);
    pos = 0;
    count = 0;
    N = 1;
    theta = 0;
    we_z = 0;
    ab_z = zeros(1, 6);
    int_ab = zeros(1, 6);
    int_data = zeros(2000, 8, 2);
    has_prev = false;
    dU = zeros(2, 2);
    dU_w = zeros(2, 2);
    dI_w = zeros(2, 2);
    dI = zeros(2, 2);
    dV_w = zeros(2, 2);
    H = zeros(3, 3, 2);
    F = zeros(3, 6, 2);
    pre = 0;
    L = NaN;
    flux = NaN;
    R = NaN;
    delta = zeros(2, 1);
    done = zeros(2, 1);
    status = 0;
    ack_z = 0;
    loss = [NaN; NaN];
    x = zeros(2, 1);
    P = 10*eye(2);
    it = 0;
end

dq = [ud uq id iq we];
valid = all(isfinite(dq)) && isfinite(stage);
if valid && stage ~= stage_z
    stage_z = stage;
    tick = 0;
    ack_z = 0;
    dq_buf(:) = 0;
    dq_sum(:) = 0;
    pos = 0;
    count = 0;
    N = 1;
end

if ~valid
    % 非有限输入才中止；不按纹波大小或参数正负拒绝结果。
    L = NaN;
    flux = NaN;
    R = NaN;
    delta(:) = 0;
    done(:) = 0;
    loss(:) = NaN;
    ready(:) = false;
    it = 0;
    status = -1;
    ack_z = 0;
    if stage >= 2 && stage <= 8
        ack_z = -double(stage);
    end
    stopped = true;
end

%% 2. 采样：先等待0.4秒，再保存0.2秒
if ~stopped && stage >= 2 && stage <= 8
    % 积分使用外部50Hz低通后的dq量，阶段5也连续积分。
    % 它只用于压降修正，不改变控制环角度或转速。
    if has_prev
        theta = mod(theta + 0.5e-4*(we_z + we), 2*pi);
    end
    c = cos(theta);
    s = sin(theta);
    ab = [ud*c-uq*s, ud*s+uq*c, id*c-iq*s, id*s+iq*c];
    D = (4/pi)*ab(3:4)/max(hypot(id, iq), 1e-12);
    ab_now = [ab D];
    if has_prev
        int_ab = int_ab + 0.5e-4*(ab_z + ab_now);
    end
    ab_z = ab_now;
    we_z = we;
    has_prev = true;

    % A组对应存储段1~3，B组对应存储段4~6，阶段5留给恢复。
    seg = 0;
    if stage >= 2 && stage <= 4
        seg = double(stage) - 1;
    elseif stage >= 6 && stage <= 8
        seg = double(stage) - 2;
    end

    if seg > 0 && ~ready(seg)
        status = 1;
        tick = tick + 1;
        % 0.1秒时锁定周期点数；五路量使用完全相同的平均器。
        if tick == 1000
            N = min(1257, max(50, round(2*pi/(max(abs(we), 50)*1e-4))));
            dq_buf(:) = 0;
            dq_sum(:) = 0;
            pos = 0;
            count = 0;
        end
        dq_avg = dq;
        if tick >= 1000
            pos = mod(pos, N) + 1;
            dq_sum = dq_sum + dq - dq_buf(pos, :);
            dq_buf(pos, :) = dq;
            count = min(N, count + 1);
            dq_avg = dq_sum/count;
        end
        if tick > 4000 && tick <= 6000
            k = tick - 4000;
            dq_data(k, :, seg) = dq_avg;
            if seg == 1 || seg == 4
                g = 1 + (seg == 4);
                int_data(k, :, g) = [int_ab c s];
            end
        end
        if tick == 6000
            dq_mean(seg, :) = sum(dq_data(:, :, seg), 1)/2000;
            ready(seg) = true;
            ack_z = double(stage);
        end
    end
end

%% 3. 正负注入求角度，两组数据交替做参数递推
if ~stopped && stage == 9
    if ~all(ready)
        status = -2;
        stopped = true;
    elseif status == 1
        status = 2;
    elseif pre < 400
        % 每拍处理5个新样本；400拍处理完每组2000个注入样本。
        pre = pre + 1;
        for g = 1:2
            seg = 2 + 3*(g-1);
            for k = (pre-1)*5+1:pre*5
                dq_p = dq_data(k, :, seg);
                dq_m = dq_data(k, :, seg+1);
                wp = speed_safe(dq_p(5));
                wm = speed_safe(dq_m(5));
                Dp = (4/pi)*dq_p(3:4)/max(hypot(dq_p(3), dq_p(4)), 1e-12);
                Dm = (4/pi)*dq_m(3:4)/max(hypot(dq_m(3), dq_m(4)), 1e-12);
                dU(g, :) = dU(g, :) + dq_p(1:2) - dq_m(1:2);
                dU_w(g, :) = dU_w(g, :) + (dq_p(1:2)/wp - dq_m(1:2)/wm);
                dI_w(g, :) = dI_w(g, :) + (dq_p(3:4)/wp - dq_m(3:4)/wm);
                dI(g, :) = dI(g, :) + (dq_p(3:4) - dq_m(3:4));
                dV_w(g, :) = dV_w(g, :) + (Dp/wp - Dm/wm);
                h = [1; int_data(k, 7, g); int_data(k, 8, g)];
                H(:, :, g) = H(:, :, g) + h*h';
                F(:, :, g) = F(:, :, g) + h*int_data(k, 1:6, g);
            end
            delta(g) = angle_wrap(atan2(dU(g, 2), dU(g, 1)));
        end
        if pre == 400
            loss = loss_fit(H, F);
            for g = 1:2
                delta(g) = angle_fix(dU_w(g, :), dI_w(g, :), ...
                    dI(g, :), dV_w(g, :), loss, 0);
            end
        end
    else
        % A1、B1、A2、B2……每拍读取一个未使用的基准样本。
        it = it + 1;
        for g = 1:2
            delta(g) = angle_fix(dU_w(g, :), dI_w(g, :), ...
                dI(g, :), dV_w(g, :), loss, 0.01*x(1));
        end
        g = 1 + mod(it-1, 2);
        k = floor((it-1)/2) + 1;
        seg = 1 + 3*(g-1);
        ud_k = dq_data(k, 1, seg);
        uq_k = dq_data(k, 2, seg);
        id_k = dq_data(k, 3, seg);
        iq_k = dq_data(k, 4, seg);
        we_k = dq_data(k, 5, seg);

        % 稳态方程消去R：uq*id-ud*iq = we*Ls*(id²+iq²)
        %                                 + we*psi*(id*cos(delta)+iq*sin(delta))。
        phi = [0.01*we_k*(id_k^2+iq_k^2); ...
            0.2*we_k*(id_k*cos(delta(g))+iq_k*sin(delta(g)))];
        scale = max(norm(phi), 1e-12);
        phi = phi/scale;
        u = (uq_k*id_k - ud_k*iq_k)/scale;
        K = P*phi/(1 + phi'*P*phi);
        e = u - phi'*x;
        x = x + K*e;
        % Joseph形式更新协方差，遗忘因子为1。
        A = eye(2) - K*phi';
        P = A*P*A' + K*K';
        P = 0.5*(P + P');
        L = 0.01*x(1);
        flux = 0.2*x(2);
        R = rs_calc(dq_mean, delta, L, flux);
        status = 3;
        if it == 4000
            done(1) = double(all(isfinite(x)));
            done(2) = double(isfinite(R));
            status = 30;
            stopped = true;
        end
    end
end

if ~stopped && stage == 10
    status = -10;
    stopped = true;
end

%% 4. 各电气量独立输出
Ls = L;
psi = flux;
Rs = R;
da = delta(1);
db = delta(2);
doneL = done(1);
doneR = done(2);
run = status;
n = it;
ack = ack_z;
R0 = loss(1);
V0 = loss(2);
end

function a = angle_fix(dU_w, dI_w, dI, dV_w, loss, Ls)
% 正负注入的ud/we、uq/we差分；修正电阻、压降和电流变化的影响。
% R0、V0来自数据标定，Ls来自当前RLS结果，不预填额定参数。
fd = dU_w(1) - loss(1)*dI_w(1) + Ls*dI(2) - loss(2)*dV_w(1);
fq = dU_w(2) - loss(1)*dI_w(2) - Ls*dI(1) - loss(2)*dV_w(2);
a = angle_wrap(atan2(fq, fd));
end

function a = angle_wrap(a)
% 位置误差取论文反正切对应的[-pi/2, pi/2]分支。
if a > pi/2
    a = a - pi;
elseif a < -pi/2
    a = a + pi;
end
end

function loss = loss_fit(H, F)
% 压降修正的辅助计算，不是论文的主参数RLS。
% 将积分量拟合为常值+余弦+正弦，取两个工作点的积分中心差。
% 静止坐标系电压方程给出：DeltaU = R0*DeltaI + V0*DeltaD。
center = zeros(2, 6);
for g = 1:2
    coef = (H(:, :, g) + 1e-10*eye(3))\F(:, :, g);
    center(g, :) = coef(1, :);
end
dInt = center(2, :) - center(1, :);
B = [dInt(3:4)', dInt(5:6)'];
u = dInt(1:2)';
scale = max(sqrt(sum(sum(B.^2))), 1e-12);
B = B/scale;
u = u/scale;
loss = (B'*B + 1e-10*eye(2))\(B'*u);
end

function Rs = rs_calc(dq_mean, delta, Ls, psi)
% 两组q轴方程：uq-we*Ls*id-we*psi*cos(delta) = Rs*iq + V0*Dq。
% 消去共同的V0，每次使用新Ls、psi更新Rs；不直接复制辅助电阻R0。
Dq = zeros(2, 1);
uq_res = zeros(2, 1);
for g = 1:2
    seg = 1 + 3*(g-1);
    id = dq_mean(seg, 3);
    iq = dq_mean(seg, 4);
    we = dq_mean(seg, 5);
    Dq(g) = (4/pi)*iq/max(hypot(id, iq), 1e-8);
    uq_res(g) = dq_mean(seg, 2) - we*Ls*id - we*psi*cos(delta(g));
end
den = Dq(2)*dq_mean(1, 4) - Dq(1)*dq_mean(4, 4);
scale = abs(Dq(2)*dq_mean(1, 4)) + abs(Dq(1)*dq_mean(4, 4));
if abs(den) > 64*eps(max(scale, 1))
    Rs = (Dq(2)*uq_res(1) - Dq(1)*uq_res(2))/den;
else
    Rs = NaN;
end
end

function we = speed_safe(we)
% 只避免浮点除零，不设置转速验收门限。
if abs(we) < 1e-9
    if we < 0
        we = -1e-9;
    else
        we = 1e-9;
    end
end
end
