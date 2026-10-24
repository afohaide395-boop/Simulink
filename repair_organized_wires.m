function report = repair_organized_wires(model_file)
% 在MATLAB/Simulink内按真实端口重建接线，再检查端口、斜线并编译。
% 原SLX保留；只有全部检查通过才保存为reduce_emf_checked_native.slx。
% 不修改电机参数、PI、PLL、辨识代码或物理电气连线。
    folder = fileparts(mfilename('fullpath'));
    if nargin < 1
        model_file = fullfile(folder, 'reduce_emf_checked.slx');
    end
    assert(isfile(model_file), '找不到待检查的SLX文件。');
    manifest_file = fullfile(folder, 'organized_wiring_manifest.json');
    assert(isfile(manifest_file), '请将接线清单JSON与本脚本放在同一目录。');

    manifest = jsondecode(fileread(manifest_file));
    assert(manifest.format == 1, '接线清单版本不匹配。');
    connections = manifest.connections;
    if iscell(connections)
        for row_index = 1:numel(connections)
            connections{row_index} = reshape(double(connections{row_index}), 1, []);
        end
        connections = vertcat(connections{:});
    end
    if size(connections, 2) ~= 5 && size(connections, 1) == 5
        connections = connections.';
    end
    assert(size(connections, 2) == 5, '接线清单必须有五列。');
    [model_folder, model_name] = fileparts(model_file);
    if isempty(model_folder)
        model_folder = pwd;
    end
    if bdIsLoaded(model_name)
        assert(strcmp(get_param(model_name, 'SimulationStatus'), 'stopped'), ...
            '请先停止当前模型的仿真，再重连接线。');
        assert(strcmp(get_param(model_name, 'Dirty'), 'off'), ...
            '当前模型有未保存修改，请先保存或关闭该模型，再运行本脚本。');
        close_system(model_name, 0);
    end
    load_system(model_file);
    model = get_param(model_name, 'Handle');

    % 先解析全部真实端口。若模块不存在或端口不足，立即停止，避免只重建一半。
    port_table = zeros(size(connections, 1), 2);
    owners = unique(connections(:, 1), 'stable');
    owner_handles = zeros(numel(owners), 1);
    for owner_index = 1:numel(owners)
        owner_handles(owner_index) = block_handle(model_name, model, owners(owner_index));
    end
    for wire_index = 1:size(connections, 1)
        record = connections(wire_index, :);
        owner = block_handle(model_name, model, record(1));
        source_block = block_handle(model_name, model, record(2));
        destination_block = block_handle(model_name, model, record(4));
        assert(strcmp(get_param(source_block, 'Parent'), getfullname(owner)), ...
            '源模块所属子系统与清单不一致。');
        assert(strcmp(get_param(destination_block, 'Parent'), getfullname(owner)), ...
            '目标模块所属子系统与清单不一致。');
        source_ports = get_param(source_block, 'PortHandles');
        destination_ports = get_param(destination_block, 'PortHandles');
        assert(numel(source_ports.Outport) >= record(3), ...
            '源模块的实际输出端口数不足。');
        assert(numel(destination_ports.Inport) >= record(5), ...
            '目标模块的实际输入端口数不足。');
        port_table(wire_index, :) = [ ...
            source_ports.Outport(record(3)), destination_ports.Inport(record(5))];
    end

    % 三个问题模块及其余整理图均由Simulink原生add_line重连。
    % 顶层仅删除普通信号线，LConn/RConn物理电气线保持原样。
    for owner_index = 1:numel(owners)
        owner = owner_handles(owner_index);
        old_lines = find_system(owner, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
        for line_index = 1:numel(old_lines)
            wire = old_lines(line_index);
            if owners(owner_index) == 0 && ~is_signal_line(wire)
                continue
            end
            delete_line(wire);
        end
    end
    for wire_index = 1:size(connections, 1)
        owner = block_handle(model_name, model, connections(wire_index, 1));
        source_port = port_table(wire_index, 1);
        destination_port = port_table(wire_index, 2);
        wire = add_line(owner, source_port, destination_port, 'autorouting', 'on');

        % 使用实际端口坐标，避免根据模块矩形猜测端点。
        source_xy = double(get_param(source_port, 'Position'));
        destination_xy = double(get_param(destination_port, 'Position'));
        if source_xy(1) == destination_xy(1) || source_xy(2) == destination_xy(2)
            points = [source_xy; destination_xy];
        else
            middle_x = round((source_xy(1) + destination_xy(1)) / 2);
            points = [source_xy; middle_x, source_xy(2); ...
                middle_x, destination_xy(2); destination_xy];
        end
        set_param(wire, 'Points', points);
    end

    verify_connections(port_table);
    verify_ports(owner_handles);
    verify_line_counts(connections, owners, owner_handles);
    set_param(model, 'SimulationCommand', 'update');
    verify_connections(port_table);
    verify_ports(owner_handles);
    verify_line_counts(connections, owners, owner_handles);

    report = struct();
    report.connection_count = size(connections, 1);
    report.system_count = numel(owners);
    report.ports_connected = true;
    report.orthogonal_lines = true;
    report.compilation_passed = true;
    report.full_motor_simulation_run = false;
    report.model_file = fullfile(model_folder, 'reduce_emf_checked_native.slx');
    save_system(model, report.model_file);
    save(fullfile(model_folder, 'native_wiring_report.mat'), 'report');
    fprintf('已通过：%d条接线、%d个图的端口与正交检查，以及Simulink编译。\n', ...
        report.connection_count, report.system_count);
    fprintf('原生重连后的模型：%s\n', report.model_file);
    fprintf('本脚本未执行电机整机仿真。\n');
end


function verify_line_counts(connections, owner_ids, owners)
    for owner_index = 1:numel(owners)
        if owner_ids(owner_index) == 0
            % 顶层另有物理线和物理分支，不能按普通信号线的数量处理。
            continue
        end
        actual_lines = find_system(owners(owner_index), 'FindAll', 'on', ...
            'SearchDepth', 1, 'Type', 'line');
        expected_count = sum(connections(:, 1) == owner_ids(owner_index));
        assert(numel(actual_lines) == expected_count, ...
            '子系统内存在清单之外的线段，可能是悬空残线。');
    end
end


function handle = block_handle(model_name, model, sid)
    if sid == 0
        handle = model;
    else
        handle = Simulink.ID.getHandle(sprintf('%s:%d', model_name, sid));
    end
end


function yes = is_signal_line(wire)
    % 物理连接端口为LConn/RConn，不能按普通信号线删除。
    yes = false;
    try
        source_port = get_param(wire, 'SrcPortHandle');
        if isscalar(source_port) && source_port > 0
            yes = strcmpi(get_param(source_port, 'PortType'), 'outport');
        end
    catch
        % 某些物理连接不提供SrcPortHandle，保留它们。
    end
end


function verify_connections(port_table)
    for wire_index = 1:size(port_table, 1)
        source_port = port_table(wire_index, 1);
        destination_port = port_table(wire_index, 2);
        wire = get_param(destination_port, 'Line');
        assert(isscalar(wire) && wire > 0, '存在未接上的目标输入端口。');
        assert(get_param(wire, 'SrcPortHandle') == source_port, ...
            '目标输入的实际来源与清单不一致。');
        destinations = get_param(wire, 'DstPortHandle');
        assert(any(destinations == destination_port), '线端未附着在真实目标端口上。');
        points = double(get_param(wire, 'Points'));
        differences = diff(points, 1, 1);
        assert(all(abs(differences(:, 1)) < 1e-8 | abs(differences(:, 2)) < 1e-8), ...
            '存在非水平/竖直的线段。');
        source_xy = double(get_param(source_port, 'Position'));
        destination_xy = double(get_param(destination_port, 'Position'));
        assert(all(abs(points(1, :) - source_xy) < 1e-8), '线头与源端口坐标不一致。');
        assert(all(abs(points(end, :) - destination_xy) < 1e-8), '线尾与目标端口坐标不一致。');
    end
end


function verify_ports(owners)
    for owner_index = 1:numel(owners)
        blocks = find_system(owners(owner_index), 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'block');
        for block_index = 1:numel(blocks)
            block = blocks(block_index);
            if strcmp(get_param(block, 'Commented'), 'on')
                continue
            end
            ports = get_param(block, 'PortHandles');
            signal_ports = [ports.Inport(:); ports.Outport(:)];
            for port_index = 1:numel(signal_ports)
                assert(get_param(signal_ports(port_index), 'Line') > 0, ...
                    ['活动模块存在悬空端口：' getfullname(block)]);
            end
        end
    end
end
