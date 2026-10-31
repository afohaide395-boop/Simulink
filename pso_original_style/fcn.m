function [dtheta, dtheta1, Ls_hat, psi_hat] = fcn(ud, uq, id, iq, we, stage)
%#codegen
% 数据采集 + 差矢量低通后取角(先滤矢量分量再 atan2) + 回归量(内部) + PSO 辨识
% 输出 4 个量: dtheta, dtheta1(角度误差), Ls_hat, psi_hat(辨识参数)
persistent ud0 uq0 id0 iq0 we0
persistent ud1 uq1 id1 iq1 we1
persistent ud2 uq2 id2 iq2 we2
persistent ud00 uq00 id00 iq00 we00
persistent ud11 uq11 id11 iq11 we11
persistent ud22 uq22 id22 iq22 we22
persistent i k prev_stage
persistent dtheta_f dtheta1_f                  % 滤波后角度
persistent dre_f dim_f dre1_f dim1_f           % 差矢量(实/虚)低通状态
persistent Y1_all A1_all B1_all Y2_all A2_all B2_all   % 两组回归数据(逐点)
persistent pso_done Ls_id psi_id                        % PSO 完成标志与结果
persistent n0 n1 n2 n00 n11 n22                        % 六段实际采样点数
persistent ready1 ready2 pso_iter
persistent X V pbest pbest_val gbest gbest_val

stage = double(stage);
if isempty(i) || (stage ~= prev_stage && (stage == 0 || stage == 2))
    ud0 = zeros(5001,1);uq0 = zeros(5001,1);id0 = zeros(5001,1);iq0 = zeros(5001,1);we0 = zeros(5001,1);
    ud1 = zeros(5001,1);uq1 = zeros(5001,1);id1 = zeros(5001,1);iq1 = zeros(5001,1);we1 = zeros(5001,1);
    ud2 = zeros(5001,1);uq2 = zeros(5001,1);id2 = zeros(5001,1);iq2 = zeros(5001,1);we2 = zeros(5001,1);
    ud00 = zeros(5001,1);uq00 = zeros(5001,1);id00 = zeros(5001,1);iq00 = zeros(5001,1);we00 = zeros(5001,1);
    ud11 = zeros(5001,1);uq11 = zeros(5001,1);id11 = zeros(5001,1);iq11 = zeros(5001,1);we11 = zeros(5001,1);
    ud22 = zeros(5001,1);uq22 = zeros(5001,1);id22 = zeros(5001,1);iq22 = zeros(5001,1);we22 = zeros(5001,1);
    i = 0; k = 0; prev_stage = -1;
    dtheta_f = NaN; dtheta1_f = NaN;
    dre_f = 0; dim_f = 0; dre1_f = 0; dim1_f = 0;
    Y1_all = zeros(5001,1); A1_all = zeros(5001,1); B1_all = zeros(5001,1);
    Y2_all = zeros(5001,1); A2_all = zeros(5001,1); B2_all = zeros(5001,1);
    pso_done = 0; Ls_id = NaN; psi_id = NaN;
    n0 = 0; n1 = 0; n2 = 0; n00 = 0; n11 = 0; n22 = 0;
    ready1 = false; ready2 = false; pso_iter = 0;
    X = zeros(2,30); V = zeros(2,30);
    pbest = zeros(2,30); pbest_val = inf(1,30);
    gbest = zeros(2,1); gbest_val = Inf;
end

if ~all(isfinite([ud uq id iq we stage])) || stage == 10
    dtheta = NaN; dtheta1 = NaN; Ls_hat = NaN; psi_hat = NaN;
    return
end
Ts = 1e-4;

% ---- 一阶低通系数： fc 为截止频率(Hz) ----
fc = 0.01;
tau = 1/(2*pi*fc);
alpha = Ts/(tau + Ts);

% ---- 段切换：复位计数，只清当前组滤波状态，保留已求出的角度 ----
if stage ~= prev_stage
    i = 0; k = 0; prev_stage = stage;
    if stage == 4
        dre_f = 0; dim_f = 0;
    elseif stage == 8
        dre1_f = 0; dim1_f = 0;
    end
end
i = i + 1;
k = 0;

% ---- 采集数据 ----
switch(stage)
    case 2
        if i > 1000
            k = i - 1000;
            if k <= 5000
                ud0(k)=ud; uq0(k)=uq; id0(k)=id; iq0(k)=iq; we0(k)=we;
                n0 = k;
            end
        end
    case 3
        if i > 1000
            k = i - 1000;
            if k <= 5000
                ud1(k)=ud; uq1(k)=uq; id1(k)=id; iq1(k)=iq; we1(k)=we;
                n1 = k;
            end
        end
    case 4
        if i > 1000
            k = i - 1000;
            if k <= 5000
                ud2(k)=ud; uq2(k)=uq; id2(k)=id; iq2(k)=iq; we2(k)=we;
                n2 = k;
            end
        end
    case 5
        % 工作点恢复段，不采样，也不清空第一组角度。
    case 6
        if i > 1000
            k = i - 1000;
            if k <= 5000
                ud00(k)=ud; uq00(k)=uq; id00(k)=id; iq00(k)=iq; we00(k)=we;
                n00 = k;
            end
        end
    case 7
        if i > 1000
            k = i - 1000;
            if k <= 5000
                ud11(k)=ud; uq11(k)=uq; id11(k)=id; iq11(k)=iq; we11(k)=we;
                n11 = k;
            end
        end
    case 8
        if i > 1000
            k = i - 1000;
            if k <= 5000
                ud22(k)=ud; uq22(k)=uq; id22(k)=id; iq22(k)=iq; we22(k)=we;
                n22 = k;
            end
        end
end

% ---- 第一组：差矢量两分量先低通，再取角 + 回归量 y/a/b（stage4 内）----
if stage == 4 && k >= 1 && k <= 5000 && n1 >= k
    % 差矢量分量低通（不取角，避免 +-pi 跳变与解缠）
    dre_f = dre_f + alpha*((uq2(k) - uq1(k)) - dre_f);
    dim_f = dim_f + alpha*((ud2(k) - ud1(k)) - dim_f);
    if abs(dre_f) + abs(dim_f) > 1e-9
        dtheta_f = atan2(dre_f, dim_f);
        if dtheta_f > pi/2
            dtheta_f = dtheta_f - pi;
        elseif dtheta_f < -pi/2
            dtheta_f = dtheta_f + pi;
        end
    end

    if k <= 4999 && n0 == 5000
        diq0 = iq0(k+1) - iq0(k);
        did0 = id0(k+1) - id0(k);
        Is0  = sqrt(id0(k)^2 + iq0(k)^2);
        y_o  = Ts * uq0(k) * id0(k) - Ts * ud0(k) * iq0(k);
        a_o  = diq0*id0(k) - did0*iq0(k) + Ts * we0(k) * Is0^2;
        b_o  = Ts * we0(k)*id0(k)*cos(dtheta_f) + Ts * we0(k)*iq0(k)*sin(dtheta_f);

        Y1_all(k) = y_o;  A1_all(k) = a_o;  B1_all(k) = b_o;   % 逐点累积(不取平均)
    end

    if k == 5000 && n0 == 5000 && n2 == 5000 && isfinite(dtheta_f)
        % 用该组最终角度统一更新b，避免前面样本使用尚未稳定的角度。
        for kk = 1:4999
            B1_all(kk) = Ts*we0(kk)*id0(kk)*cos(dtheta_f) + Ts*we0(kk)*iq0(kk)*sin(dtheta_f);
        end
        ready1 = true;
    end
end

% ---- 第二组：差矢量两分量先低通，再取角 + 回归量 y1/a1/b1（stage8 内）----
if stage == 8 && k >= 1 && k <= 5000 && n11 >= k
    dre1_f = dre1_f + alpha*((uq22(k) - uq11(k)) - dre1_f);
    dim1_f = dim1_f + alpha*((ud22(k) - ud11(k)) - dim1_f);
    if abs(dre1_f) + abs(dim1_f) > 1e-9
        dtheta1_f = atan2(dre1_f, dim1_f);
        if dtheta1_f > pi/2
            dtheta1_f = dtheta1_f - pi;
        elseif dtheta1_f < -pi/2
            dtheta1_f = dtheta1_f + pi;
        end
    end

    if k <= 4999 && n00 == 5000
        diq00 = iq00(k+1) - iq00(k);
        did00 = id00(k+1) - id00(k);
        Is00  = sqrt(id00(k)^2 + iq00(k)^2);
        y1_o  = Ts * uq00(k) * id00(k) - Ts * ud00(k) * iq00(k);
        a1_o  = diq00*id00(k) - did00*iq00(k) + Ts * we00(k) * Is00^2;
        b1_o  = Ts * we00(k)*id00(k)*cos(dtheta1_f) + Ts * we00(k)*iq00(k)*sin(dtheta1_f);

        Y2_all(k) = y1_o;  A2_all(k) = a1_o;  B2_all(k) = b1_o;   % 逐点累积(不取平均)
    end

    if k == 5000 && n00 == 5000 && n22 == 5000 && isfinite(dtheta1_f)
        for kk = 1:4999
            B2_all(kk) = Ts*we00(kk)*id00(kk)*cos(dtheta1_f) + Ts*we00(kk)*iq00(kk)*sin(dtheta1_f);
        end
        ready2 = true;
    end
end

% ================= PSO 辨识 [Ls, psi_m]（式30/31）=================
% 两组采齐后在stage9运行；每次调用更新一代，仍逐点计算残差平方和
% 已取消上下限：粒子自由搜索
if stage == 9 && ready1 && ready2 && pso_done == 0
    nPar  = 30;                        % 粒子数
    nIter = 150;                       % 迭代次数
    Ls_c  = 9e-3;   Ls_s  = 9e-3;      % Ls 初始中心/散布(仅用于初始化)
    ps_c  = 0.1;    ps_s  = 0.1;       % psi_m 初始中心/散布(仅用于初始化)
    ww = 0.7;  c1p = 1.5;  c2p = 1.5;  % 惯性权重 / 认知 / 社会因子

    if pso_iter == 0
        X = zeros(2,nPar);  V = zeros(2,nPar);
        for p = 1:nPar
            X(1,p) = Ls_c + Ls_s*(rand-0.5);    % 自由初始化(无硬边界)
            X(2,p) = ps_c + ps_s*(rand-0.5);
            V(1,p) = 0.1*Ls_s*(rand-0.5);
            V(2,p) = 0.1*ps_s*(rand-0.5);
        end
        pbest = X;  pbest_val = 1e30*ones(1,nPar);
        gbest = X(:,1);  gbest_val = 1e30;
    end

    pso_iter = pso_iter + 1;
    for p = 1:nPar
        Lsp = X(1,p);  psip = X(2,p);
        % 适应度 (式30 的多点形式): 两组残差平方和
        fit = 0;
        for kk = 1:4999
            e1 = Y1_all(kk) - A1_all(kk)*Lsp - B1_all(kk)*psip;
            e2 = Y2_all(kk) - A2_all(kk)*Lsp - B2_all(kk)*psip;
            fit = fit + e1*e1 + e2*e2;
        end
        if fit < pbest_val(p)
            pbest_val(p) = fit;  pbest(:,p) = X(:,p);
            if fit < gbest_val
                gbest_val = fit;  gbest = X(:,p);
            end
        end
    end
    if pso_iter < nIter
        for p = 1:nPar
            r1 = rand;  r2 = rand;
            % 速度与位置更新 (式31a/31b)
            V(:,p) = ww*V(:,p) + c1p*r1*(pbest(:,p)-X(:,p)) + c2p*r2*(gbest-X(:,p));
            X(:,p) = X(:,p) + V(:,p);
            % 已删除 min(max(...)) 边界夹取
        end
    end
    Ls_id = gbest(1);  psi_id = gbest(2);
    if pso_iter == nIter
        pso_done = 1;
    end
end
% ---- 输出 4 个量 ----
dtheta  = dtheta_f;    % 第一组角度误差
dtheta1 = dtheta1_f;   % 第二组角度误差
Ls_hat  = Ls_id;       % 辨识电感
psi_hat = psi_id;      % 辨识磁链
end
