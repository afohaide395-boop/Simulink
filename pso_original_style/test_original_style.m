function test_original_style
% 执行原风格版实际代码；函数验证，不是整机仿真。
addpath(fileparts(mfilename('fullpath')));
clear fcn
rng(53);
L = 6.5e-3;
psi = 0.17;
R = 2.2;
we = -180;
delta = [-0.08 0.05];

for stage = 2:8
    if stage == 5
        [da, db, Lh, ph] = fcn(0, 0, 0, 0, we, stage);
        assert(abs(da-delta(1)) < 1e-10 && isnan(db));
        assert(isnan(Lh) && isnan(ph));
        fprintf('PASS 第一组角度在阶段5继续保存\n');
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
        t = j*1e-4;
        tn = (j+1)*1e-4;
        id = -(g-1) + 0.03*sin(2*pi*3*t);
        iq = -(0.9 + 0.02*cos(2*pi*2*t));
        idn = -(g-1) + 0.03*sin(2*pi*3*tn);
        iqn = -(0.9 + 0.02*cos(2*pi*2*tn));
        D = (4/pi)*[id iq]/hypot(id, iq);
        ud = R*id + L*(idn-id)/1e-4 - we*L*iq - we*psi*sin(angle) + 0.8*D(1);
        uq = R*iq + L*(iqn-iq)/1e-4 + we*L*id + we*psi*cos(angle) + 0.8*D(2);
        [da, db, Lh, ph] = fcn(ud, uq, id, iq, we, int32(stage));
        assert(isnan(Lh) && isnan(ph));
    end
end
assert(max(abs([da db]-delta)) < 1e-10);
fprintf('PASS 两组有符号角度=%g/%g，六段各6000拍后完成采样\n', da, db);

% 窗口结束后继续停留，不应重放旧数据或让计数器循环覆盖。
for j = 1:5
    [a2, b2] = fcn(100, 200, 3, 4, we, 8);
    assert(isequal([da db], [a2 b2]));
end
fprintf('PASS 采样段延长时角度保持，不重放最后一点\n');

history = zeros(150, 2);
for j = 1:150
    [da, db, Lh, ph] = fcn(0, 0, 0, 0, we, 9);
    assert(max(abs([da db]-delta)) < 1e-10);
    assert(all(isfinite([Lh ph])));
    history(j, :) = [Lh ph];
    if mod(j, 30) == 0
        fprintf('PASS PSO已运行%d代\n', j);
    end
end
err = [Lh/L-1 ph/psi-1];
assert(max(abs(err)) < 0.01);
assert(norm(history(1, :)-history(end, :)) > 1e-5);
fprintf('PASS L=%g，psi=%g，理想方程记录误差=%g/%g%%\n', Lh, ph, 100*err(1), 100*err(2));
for j = 1:3
    [a2, b2, L2, p2] = fcn(0, 0, 0, 0, we, 9);
    assert(isequal([da db Lh ph], [a2 b2 L2 p2]));
end
fprintf('PASS 150代后结果保持\n');
[da, db, Lh, ph] = fcn(0, 0, 0, 0, we, 2);
assert(all(isnan([da db Lh ph])));
[da, db, Lh, ph] = fcn(0, 0, 0, 0, we, 9);
assert(all(isnan([da db Lh ph])));
fprintf('PASS 重启清除旧数据与粒子群；缺数据不启动PSO\n');
end
