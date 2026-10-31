function [dtheta, dtheta1, Ls_hat, psi_hat] = fcn(ud, uq, id, iq, we, stage)
%#codegen
% 虚拟反电动势辨识：两组角度 + 电感、磁链PSO。
% 输入已经过外部50Hz低通，Ts=100us；各采样段持续6000拍。
% stage=2/3/4：A组基准/正注入/负注入；6/7/8：B组；9：计算。
% 输出顺序固定：角度A(rad)、角度B(rad)、电感(H)、磁链(Wb)。

persistent dq count tick last delta angle_done bad
persistent H h yy k it Ls psi
persistent X V best bestFit gbest gbestFit

Ts = 1e-4;
N = 5000;
stage = double(stage);

%% 1. 新实验复位
if isempty(last)
    last = -1;
end
if isempty(dq) || (stage ~= last && (stage == 0 || stage == 2))
    % dq每行依次保存[ud uq id iq we]，第三维为六个采样段。
    dq = zeros(5000, 5, 6);
    count = zeros(6, 1);
    tick = 0;
    delta = [NaN; NaN];
    angle_done = false(2, 1);
    bad = false;
    H = zeros(2, 2);
    h = zeros(2, 1);
    yy = 0;
    k = 0;
    it = 0;
    Ls = NaN;
    psi = NaN;
    X = zeros(2, 30);
    V = zeros(2, 30);
    best = zeros(2, 30);
    bestFit = inf(1, 30);
    gbest = zeros(2, 1);
    gbestFit = Inf;
end

valid = all(isfinite([ud uq id iq we stage]));
if valid && stage ~= last
    last = stage;
    tick = 0;
end
if ~valid || stage == 10
    bad = true;
    delta(:) = NaN;
    Ls = NaN;
    psi = NaN;
end

%% 2. 六段采样：等待1000拍，保存随后5000拍
seg = 0;
if stage >= 2 && stage <= 4
    seg = stage - 1;
elseif stage >= 6 && stage <= 8
    seg = stage - 2;
end
if ~bad && seg > 0 && count(seg) < N
    % 先处理阶段切换，再计数；每段第6000拍恰好写入第5000点。
    tick = tick + 1;
    if tick > 1000 && tick <= 6000
        j = tick - 1000;
        dq(j, :, seg) = [ud uq id iq we];
        count(seg) = j;
    end
end

%% 3. 正负电压差求角度，两组结果分别保存
for g = 1:2
    sp = 2 + 3*(g-1);
    sm = sp + 1;
    if ~bad && ~angle_done(g) && count(sp) == N && count(sm) == N
        % 先平均差矢量分量，再取角；不对角度取绝对值。
        du = sum(dq(:, 1, sp) - dq(:, 1, sm))/N;
        dv = sum(dq(:, 2, sp) - dq(:, 2, sm))/N;
        if du ~= 0 || dv ~= 0
            a = atan2(dv, du);
            if a > pi/2
                a = a - pi;
            elseif a < -pi/2
                a = a + pi;
            end
            delta(g) = a;
        end
        angle_done(g) = true;
    end
end

%% 4. 数据齐备后构造论文式(27)~(30)
if ~bad && stage == 9 && all(count == N) && all(isfinite(delta))
    if k < N-1
        % 5000个真实采样点只能形成4999个前向差分，不能读取伪造的第5001点。
        % 每拍处理A/B各一个基准点，所有有效样本都参与目标函数。
        k = k + 1;
        for g = 1:2
            s = 1 + 3*(g-1);
            [a, b, u] = regression(dq(k, :, s), dq(k+1, :, s), delta(g), Ts);
            v = [a; b];
            H = H + v*v';
            h = h + v*u;
            yy = yy + u*u;
        end
    elseif it < 150
        %% 5. PSO每拍更新一代，共30个粒子、150代
        if it == 0
            % 仅为粒子初始化的中心和散布，不是结果或搜索上下限。
            x0 = [9e-3; 0.1];
            dx = [9e-3; 0.1];
            for p = 1:30
                X(:, p) = x0 + dx.*(rand(2, 1)-0.5);
                V(:, p) = 0.1*dx.*(rand(2, 1)-0.5);
            end
            best = X;
        end

        for p = 1:30
            fit = cost(X(:, p), H, h, yy);
            if fit < bestFit(p)
                bestFit(p) = fit;
                best(:, p) = X(:, p);
                if fit < gbestFit
                    gbestFit = fit;
                    gbest = X(:, p);
                end
            end
        end

        % 发布已经评价过的全局最优解，不发布尚未评价的新位置。
        Ls = gbest(1);
        psi = gbest(2);
        it = it + 1;
        if it < 150
            for p = 1:30
                r1 = rand(2, 1);
                r2 = rand(2, 1);
                V(:, p) = 0.7*V(:, p) ...
                    + 1.5*r1.*(best(:, p)-X(:, p)) ...
                    + 1.5*r2.*(gbest-X(:, p));
                X(:, p) = X(:, p) + V(:, p);
            end
        end
    end
end

%% 6. 独立输出，完成后保持，进入新实验时复位
dtheta = delta(1);
dtheta1 = delta(2);
Ls_hat = Ls;
psi_hat = psi;
end

function [a, b, u] = regression(dq, dq_next, delta, Ts)
% 论文式(28)：u = a*Ls + b*psi。
ud = dq(1);
uq = dq(2);
id = dq(3);
iq = dq(4);
we = dq(5);
did = dq_next(3) - id;
diq = dq_next(4) - iq;
u = Ts*(uq*id - ud*iq);
a = diq*id - did*iq + Ts*we*(id^2 + iq^2);
b = Ts*we*(id*cos(delta) + iq*sin(delta));
end

function fit = cost(x, H, h, yy)
% 将全部样本的sum((u-a*Ls-b*psi)^2)展开，目标函数不变。
% 只省去每个粒子重复遍历全部样本的开销，仍由PSO搜索参数。
fit = x'*H*x - 2*h'*x + yy;
end
