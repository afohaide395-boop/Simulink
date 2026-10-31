function test_cleanup_identification
% 本次上传代码与精简代码逐拍比较；不替代MATLAB整机仿真。
addpath(fileparts(mfilename('fullpath')));
clear fcn fcn_before
rng(63);
L = 6.5e-3;
psi = 0.17;
R = 2.2;
we = -180;
delta = [-0.08 0.05];
for stage = 2:8
    if stage == 5
        compare([0 0 0 0 we], stage);
        continue
    end
    g = 1+(stage >= 6);
    shift = 0;
    if stage == 3 || stage == 7
        shift = 0.02;
    elseif stage == 4 || stage == 8
        shift = -0.02;
    end
    angle = delta(g)+shift;
    for j = 1:6000
        t = j*1e-4;
        tn = (j+1)*1e-4;
        id = -(g-1)+0.03*sin(2*pi*3*t);
        iq = -(0.9+0.02*cos(2*pi*2*t));
        idn = -(g-1)+0.03*sin(2*pi*3*tn);
        iqn = -(0.9+0.02*cos(2*pi*2*tn));
        D = (4/pi)*[id iq]/hypot(id, iq);
        ud = R*id+L*(idn-id)/1e-4-we*L*iq-we*psi*sin(angle)+0.8*D(1);
        uq = R*iq+L*(iqn-iq)/1e-4+we*L*id+we*psi*cos(angle)+0.8*D(2);
        v = compare([ud uq id iq we], int32(stage));
    end
end
assert(max(abs(v(1:2)-delta)) < 1e-10);
fprintf('PASS 六段采样逐拍四输出与上传版一致，1Hz滤波及两组有符号角度保留\n');
history = zeros(150, 2);
for j = 1:150
    state = rng;
    [da, db, Lh, ph] = fcn(0, 0, 0, 0, we, 9);
    after = rng;
    rng(state);
    [a0, b0, L0, p0] = fcn_before(0, 0, 0, 0, we, 9);
    assert(isequaln([da db Lh ph], [a0 b0 L0 p0]));
    state2 = rng;
    assert(isequal(after.State, state2.State));
    history(j, :) = [Lh ph];
    if mod(j, 30) == 0
        fprintf('PASS PSO第%d代逐位一致\n', j);
    end
end
err = [Lh/L-1 ph/psi-1];
assert(max(abs(err)) < 0.01);
assert(norm(history(1, :)-history(end, :)) > 1e-5);
fprintf('PASS 150代实际递推：L=%g，psi=%g，理想方程误差=%g/%g%%\n', Lh, ph, 100*err(1), 100*err(2));
for j = 1:3
    compare([0 0 0 0 we], 9);
end
compare([0 0 0 0 we], 2);
v = compare([0 0 0 0 we], 9);
assert(all(isnan(v(3:4))));
fprintf('PASS 完成保持、重新启动与缺段不启动PSO\n');
end

function v = compare(dq, stage)
[da, db, Lh, ph] = fcn(dq(1), dq(2), dq(3), dq(4), dq(5), stage);
[a0, b0, L0, p0] = fcn_before(dq(1), dq(2), dq(3), dq(4), dq(5), stage);
v = [da db Lh ph];
assert(isequaln(v, [a0 b0 L0 p0]));
end
