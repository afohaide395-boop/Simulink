function test_pso_code
% 执行交付的fcn，用满足论文方程的数据验证；不是整机仿真。
addpath(fileparts(mfilename('fullpath')));
cases = [8.96e-3 0.21 1.8665 200 0.03 -0.04 0.5; ...
         6.5e-3 0.17 2.2 180 -0.08 0.05 0.8; ...
         11e-3 0.25 1.4 -190 -0.02 -0.06 0.7; ...
         -4e-3 0.18 1.2 200 0.04 -0.03 0];
for c = 1:size(cases, 1)
    clear fcn
    rng(50+c);
    q = cases(c, :);
    L = q(1);
    psi = q(2);
    R = q(3);
    we = q(4);
    delta = q(5:6);
    drop = q(7);
    for stage = 2:8
        if stage == 5
            [da, db, Lh, ph] = fcn(0, 0, 0, 0, we, stage);
            assert(abs(da-delta(1)) < 1e-10 && isnan(db));
            assert(isnan(Lh) && isnan(ph));
            continue
        end
        g = 1 + (stage >= 6);
        shift = 0;
        if stage == 3 || stage == 7
            shift = 0.02;
        elseif stage == 4 || stage == 8
            shift = -0.02;
        end
        angle = delta(g) + shift;
        for j = 1:6000
            % 随时间缓慢变化的电流，使前向差分确实参与方程。
            t = j*1e-4;
            id = -(g-1) + 0.03*sin(2*pi*3*t);
            iq = sign(we)*(0.9 + 0.02*cos(2*pi*2*t));
            tn = (j+1)*1e-4;
            idn = -(g-1) + 0.03*sin(2*pi*3*tn);
            iqn = sign(we)*(0.9 + 0.02*cos(2*pi*2*tn));
            D = (4/pi)*[id iq]/hypot(id, iq);
            ud = R*id + L*(idn-id)/1e-4 - we*L*iq - we*psi*sin(angle) + drop*D(1);
            uq = R*iq + L*(iqn-iq)/1e-4 + we*L*id + we*psi*cos(angle) + drop*D(2);
            [da, db, Lh, ph] = fcn(ud, uq, id, iq, we, int32(stage));
            assert(isnan(Lh) && isnan(ph), '采样未完成就开始辨识。');
            if stage == 4 && j == 5999
                assert(isnan(da));
            elseif stage == 4 && j == 6000
                assert(abs(da-delta(1)) < 1e-10);
            elseif stage == 8 && j == 5999
                assert(isnan(db));
            elseif stage == 8 && j == 6000
                assert(abs(db-delta(2)) < 1e-10);
            end
        end
    end

    path = zeros(150, 2);
    for j = 1:5149
        [da, db, Lh, ph] = fcn(0, 0, 0, 0, we, 9);
        assert(max(abs([da db]-delta)) < 1e-10, '进入计算阶段后角度丢失。');
        if j <= 4999
            assert(isnan(Lh) && isnan(ph), '有效回归点未处理完就运行PSO。');
        else
            path(j-4999, :) = [Lh ph];
        end
    end
    err = [Lh/L-1 ph/psi-1];
    assert(max(abs(err)) < 0.01, '理想方程记录的误差超出1%。');
    assert(norm(path(1, :)-path(end, :)) > 1e-5, '没有实际推进粒子群。');
    for j = 1:20
        [da2, db2, L2, p2] = fcn(0, 0, 0, 0, we, 9);
        assert(isequal([da db Lh ph], [da2 db2 L2 p2]));
    end
    if L < 0
        assert(Lh < 0, '不应将负参数强制改成正数。');
    end
    fprintf('PASS case%d: delta=%g/%g, L=%g, psi=%g, error=%g/%g%%\n', ...
        c, da, db, Lh, ph, 100*err(1), 100*err(2));
    [da, db, Lh, ph] = fcn(0, 0, 0, 0, we, 2);
    assert(all(isnan([da db Lh ph])), '重新进入阶段2后未复位。');
end

clear fcn
[da, db, Lh, ph] = fcn(0, 0, 0, 0, 200, 9);
assert(all(isnan([da db Lh ph])), '缺少数据时不能给出辨识值。');
[da, db, Lh, ph] = fcn(NaN, 0, 0, 0, 200, 2);
assert(all(isnan([da db Lh ph])));
fprintf('PASS 角度符号/跨段保存、5000点边界、4999点差分、150代更新、保持与重启、缺数据保护\n');

% 检查展开的目标函数与原始逐点残差平方和等价。
Q = randn(30, 2);
u = randn(30, 1);
x = [0.006; 0.21];
direct = sum((u-Q*x).^2);
expanded = x'*(Q'*Q)*x - 2*(Q'*u)'*x + sum(u.^2);
assert(abs(direct-expanded) < 1e-12*max(1, direct));
fprintf('PASS 所有样本残差目标函数的展开等价，不是用均值替代原始回归样本\n');
end
