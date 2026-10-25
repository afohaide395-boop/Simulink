function test_simple_identification(fixtures, cases)
% 使用同一批物理方程记录逐拍比较新旧辨识，不是整机仿真。
folder = fileparts(mfilename('fullpath'));
addpath(folder);
addpath(fullfile(folder, 'tests', 'fixtures'));
if nargin < 1
    fixtures = folder;
end
if nargin < 2
    cases = {'nominal', 'inverter', 'different_motor', 'reverse', ...
        'noise', 'speed_ripple', 'harmonics', 'positive_angles'};
end
for c = 1:numel(cases)
    clear identify_simple identify_vbemf_v13
    f = load(fullfile(fixtures, [cases{c} '_fixture.mat']));
    for k = 1:size(f.samples, 1)
        compare(f.samples(k, :), f.states(k));
    end
    history = zeros(4405, 4);
    for k = 1:4405
        v = compare([0 0 0 0 200], 9);
        history(k, :) = v([1 2 3 9]);
    end
    assert(v(8) == 30 && v(9) == 4000);
    assert(v(6) == 1 && v(7) == 1);
    assert(all(diff(history(402:4401, 4)) == 1));
    assert(norm(history(402, 1:2) - history(4401, 1:2)) > 1e-5);
    truth = f.truth(:);
    err = v(1:3)'./truth(1:3) - 1;
    assert(max(abs(err(1:2))) < 0.05);
    assert(abs(err(3)) < 0.15);
    fprintf('PASS %s：逐拍12路输出与v13一致，4000次更新；L/psi/R误差=%g/%g/%g%%\n', ...
        cases{c}, 100*err(1), 100*err(2), 100*err(3));
end

% 重启必须清空缓存与递推状态；缺段数据不应输出伪造参数。
compare([NaN 0 0 0 200], 8);
compare([0 45 0 1 200], 8);
v = compare([0 45 0 1 200], 0);
assert(all(isnan(v(1:3))) && v(9) == 0);
v = compare([0 45 0 1 200], 9);
assert(v(8) == -2 && all(isnan(v(1:3))));
clear identify_simple
[L, psi, R, ~, ~, ~, ~, run, ~, ack] = identify_simple(NaN, 0, 0, 0, 200, int32(2));
assert(run == -1 && ack == -2 && all(isnan([L psi R])));
clear identify_simple
[L, psi, R, ~, ~, ~, ~, run, n] = identify_simple(0, 45, 0, 1, 200, int32(10));
assert(run == -10 && n == 0 && all(isnan([L psi R])));
fprintf('PASS 复位、缺数据、非有限输入及int32阶段输入\n');
end

function v = compare(dq, stage)
% 只比较保留的物理量与流程输出，不比较已移除的诊断和历史向量。
[L, psi, R, da, db, doneL, doneR, run, n, ack, R0, V0] = ...
    identify_simple(dq(1), dq(2), dq(3), dq(4), dq(5), stage);
[Lz, psiz, Rz, daz, dbz, doneLz, doneRz, runz, ~, ~, ~, nz, ...
    ~, ~, ~, ackz, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, R0z, V0z] = ...
    identify_vbemf_v13(dq(1), dq(2), dq(3), dq(4), dq(5), stage);
v = [L psi R da db doneL doneR run n ack R0 V0];
z = [Lz psiz Rz daz dbz doneLz doneRz runz nz ackz R0z V0z];
assert(isequaln(v, z), '简化后保留的输出与原算法不同。');
end
