function parse_and_plot_trc(filename)
    % 대상 ID
    target_ids = ["0003", "0004", "0005", "0006", "00002901", "00002902"];

    % 결과 저장용
    time_003 = []; data_003 = [];
    time_004 = []; data_004 = [];
    time_005 = []; data_005 = [];
    time_006 = []; data_006 = [];
    time_2901 = []; data_2901 = [];
    time_2902 = []; data_2902 = [];

    % 파일 열기
    fid = fopen(filename, 'r');
    while ~feof(fid)
        line = fgetl(fid);
        if contains(line, 'Rx')
            tokens = regexp(line, '\)\s+([\d.]+)\s+Rx\s+([0-9A-Fa-f]{4,8})\s+8\s+([0-9A-Fa-f\s]+)', 'tokens');
            if ~isempty(tokens)
                t = str2double(tokens{1}{1});
                id_hex = upper(tokens{1}{2});
                data_str = strtrim(tokens{1}{3});
                data_bytes = uint8(sscanf(data_str, '%2x')');

                % Unpack 데이터
                if id_hex == "0003"
                    decoded = unpack_var4(data_bytes);
                    time_003(end+1, 1) = t;
                    data_003(end+1, :) = decoded;

                elseif id_hex == "0004"
                    decoded = unpack_var4(data_bytes);
                    time_004(end+1,1) = t;
                    data_004(end+1, :) = decoded;

                elseif id_hex == "0005"
                    decoded = unpack_var4(data_bytes);
                    time_005(end+1,1) = t;
                    data_005(end+1, :) = decoded;

                elseif id_hex == "0006"
                    decoded = unpack_var4(data_bytes);
                    time_006(end+1,1) = t;
                    data_006(end+1, :) = decoded;
                    
                elseif id_hex == "00002901"
                    decoded = unpack_motor(data_bytes);
                    time_2901(end+1,1) = t;
                    data_2901(end+1, :) = decoded;
                elseif id_hex == "00002902"
                    decoded = unpack_motor(data_bytes);
                    time_2902(end+1,1) = t;
                    data_2902(end+1, :) = decoded;

                end
            end
        end
    end

    % 1. control_flag가 1인 구간의 time 추출
    flag_idx = find(data_005(:,3) == 1);
    active_time = time_005(flag_idx);

    % 2. 각 ID별로 해당 구간에 포함되는 데이터만 남김
    is_in = @(t) ismember(t, active_time);

    idx_003 = is_in(time_003);
    time_003 = time_003(idx_003);
    data_003 = data_003(idx_003, :);

    idx_004 = is_in(time_004);
    time_004 = time_004(idx_004);
    data_004 = data_004(idx_004, :);

    idx_005 = is_in(time_005);
    time_005 = time_005(idx_005);
    data_005 = data_005(idx_005, :);

    idx_006 = is_in(time_006);
    time_006 = time_006(idx_006);
    data_006 = data_006(idx_006, :);

    idx_2901 = is_in(time_2901);
    time_2901 = time_2901(idx_2901);
    data_2901 = data_2901(idx_2901, :);

    idx_2902 = is_in(time_2902);
    time_2902 = time_2902(idx_2902);
    data_2902 = data_2902(idx_2902, :);


    fclose(fid);
    % save('parsed_trc_data.mat', ...
    %     'time_003','data_003', ...
    %     'time_004','data_004', ...
    %     'time_005','data_005', ...
    %     'time_006','data_006', ...
    %     'time_2901','data_2901', ...
    %     'time_2902','data_2902');


    
    % % Plot
    % figure(1);

    % % ID 0003 - q1, q2, r1, r2
    % subplot(3,1,1);
    % hold on;
    % if ~isempty(time_003)
    %     plot(time_003, data_003(:,1), 'g-', 'LineWidth', 1); % q1
    %     plot(time_003, data_003(:,2), 'b-', 'LineWidth', 1); % q2
    %     plot(time_003, data_003(:,3), 'g--', 'LineWidth',1); % r1
    %     plot(time_003, data_003(:,4), 'b--', 'LineWidth',1); % r2
    % end
    % hold off;
    % title('ID 0003 - q1, q2, r1, r2');
    % legend('q1','q2','r1','r2');

    % % ID 0004 - qdot1, qdot2, rdot1, rdot2
    % subplot(3,1,2);
    % hold on;
    % if ~isempty(time_004)
    %     plot(time_004, data_004(:,1), 'g-', 'LineWidth', 1); % qdot1
    %     plot(time_004, data_004(:,2), 'b-', 'LineWidth', 1); % qdot2
    %     plot(time_004, data_004(:,3), 'g--', 'LineWidth', 1); % rdot1
    %     plot(time_004, data_004(:,4), 'b--', 'LineWidth', 1); % rdot2
    % end
    % hold off;
    % title('ID 0004 - qdot1, qdot2, rdot1, rdot2');
    % legend('qdot1','qdot2','rdot1','rdot2');

    % % ID 0005 - u1, u2, ctrl_flag, ctrl_time
    % subplot(3,1,3);
    % hold on;
    % if ~isempty(time_005)
    %     plot(time_005, data_005(:,1), 'g-', 'LineWidth', 1); % u1
    %     plot(time_005, data_005(:,2), 'b-', 'LineWidth', 1); % u2
    %     plot(time_005, data_005(:,3), 'r--', 'LineWidth', 1); % ctrl_flag
    %     plot(time_005, data_005(:,4), 'm--', 'LineWidth', 1); % ctrl_time
    % end
    % hold off;
    % title('ID 0005 - u1, u2, ctrl_flag, ctrl_time');
    % legend('u1','u2','ctrl\_flag','ctrl\_time');

    % % comparison of qdot1
    % figure(2);
    % hold on;
    % plot(time_006, data_006(:,1), 'g-', 'LineWidth', 1); % qdot1
    % plot(time_006, data_006(:,3), 'g--', 'LineWidth', 1); % filter_qdot1
    % plot(time_2901, data_2901(:,2), 'r-', 'LineWidth', 1); % motor_qdot1
    % hold off;
    % title('qdot1, filter_qdot1 and motor_qdot2');
    % legend('qdot1', 'filter\_qdot1', 'motor\_qdot1');

    % % comparison of qdot2
    % figure(3);
    % hold on;
    % plot(time_006, data_006(:,2), 'g-', 'LineWidth', 1); % qdot2
    % plot(time_006, data_006(:,4), 'g--', 'LineWidth', 1); % filter_qdot2
    % plot(time_2902, data_2902(:,2), 'r-', 'LineWidth', 1); % motor_qdot2
    % hold off;
    % title('qdot2, filter_qdot2 and motor_qdot2');
    % legend('qdot2', 'filter\_qdot2', 'motor\_qdot2');

end

function vals = unpack_var4(data)
    if length(data) ~= 8
        error("Invalid data length");
    end
    vals = zeros(1,4);
    % for i = 1:4
    %     raw = typecast(uint8([data(2*i-1), data(2*i)]), 'int16');    % Little Endian 
    %     vals(i) = double(raw) / 1000.0;
    % end
    for i = 1:4
        high = data(2*i - 1);
        low = data(2*i);
        val = bitor(bitshift(int16(high), 8), int16(low));  % Big endian 조립
        vals(i) = double(val) / 1000.0;
    end
end

function vals = unpack_motor(data)
    if length(data) ~= 8
        error("Invalid data length");
    end
    vals = zeros(1,3);
    pos_int = bitor(bitshift(int16(data(1)),8), int16(data(2)));
    spd_int = bitor(bitshift(int16(data(3)),8), int16(data(4)));
    cur_int = bitor(bitshift(int16(data(5)),8), int16(data(6)));
    
    vals(1) = double(pos_int) * 0.1 * pi / 180.0;                 % rad
    vals(2) = double(spd_int) * 10 / (9 * 21) * 2 * pi / 60;      % rad/s
    vals(3) = double(cur_int) * 1 / 0.75 * 0.01;    % Nm
end
