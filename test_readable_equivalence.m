function test_readable_equivalence(fixture_folder)
% 同一输入逐拍运行重构前后两版函数，比较全部31个标量及两组角度历史。
    folder = fileparts(mfilename('fullpath'));
    addpath(folder);
    addpath(fullfile(folder, 'tests', 'fixtures'));
    if nargin < 1
        fixture_folder = folder;
    end

    cases = {'nominal', 'noise', 'reverse'};
    sample_total = 0;
    for case_index = 1:numel(cases)
        clear identify_vbemf identify_vbemf_v9
        fixture = load(fullfile(fixture_folder, [cases{case_index} '_fixture.mat']));

        % 包含初始化、全部六个采样阶段、Id恢复段及4000次真实递推。
        compare_sample([0, 45, 0, 1, 200], 0);
        for sample_index = 1:size(fixture.samples, 1)
            compare_sample(fixture.samples(sample_index, :), fixture.states(sample_index));
        end
        for sample_index = 1:4405
            compare_sample([0, 0, 0, 0, 200], 9);
        end

        % 保留完成输出、非有限输入、无效确认和重新启动行为也必须一致。
        compare_sample([NaN, 0, 0, 0, 200], 8);
        compare_sample([0, 45, 0, 1, 200], 8);
        compare_sample([0, 45, 0, 1, 200], 0);
        compare_sample([0, 45, 0, 1, 200], 2);
        compare_sample([0, 45, 0, 1, 200], 9);
        compare_sample([0, 45, 0, 1, 200], 10);
        compare_sample([0, 45, 0, 1, 200], NaN);
        sample_total = sample_total + size(fixture.samples, 1) + 4413;
        fprintf('PASS %s：每拍全部输出与v9逐位一致（含NaN及角度历史）。\n', cases{case_index});
    end
    fprintf('PASS 共%d拍，覆盖命名输出映射、4000次递推、异常和复位。\n', sample_total);
end

function compare_sample(dq_sample, experiment_stage)
    [new_record, new_delta_A, new_delta_B] = record_identification( ...
        dq_sample(1), dq_sample(2), dq_sample(3), dq_sample(4), dq_sample(5), experiment_stage);
    [old_record, old_delta_A, old_delta_B] = identify_vbemf_v9( ...
        dq_sample(1), dq_sample(2), dq_sample(3), dq_sample(4), dq_sample(5), experiment_stage);
    assert(isequaln(new_record, old_record), '标量输出与重构前不一致。');
    assert(isequaln(new_delta_A, old_delta_A), 'A组角度历史与重构前不一致。');
    assert(isequaln(new_delta_B, old_delta_B), 'B组角度历史与重构前不一致。');
end
