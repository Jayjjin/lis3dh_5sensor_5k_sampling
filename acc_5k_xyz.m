%% ========================================================================
%  사용자 설정 영역
% =========================================================================
comPort      = '/dev/cu.usbmodem142101'; % TODO: Arduino가 연결된 COM 포트 번호를 입력하세요
baudRate     = 2000000;    % Arduino 코드와 동일한 통신 속도
duration     = 10;         % 측정 시간 (초)
samplingRate = 5000;       % 목표 샘플링 속도 5000Hz
clearvars -except comPort baudRate duration samplingRate audioDeviceName;
% close all; clc;
fprintf('스크립트를 시작합니다...\n');
chunkSize  = 100; % Arduino의 TRANSMIT_CHUNK_SIZE
numSensors = 2;
numAxes    = 3; % X, Y, Z
% timestamp(4) + (x,y,z)(2*3) * 5_sensors = 4 + 30 = 34 bytes
bytesPerSample = 4 + (numSensors * numAxes * 2); 
frameBytes = chunkSize * bytesPerSample;

% --- 데이터 저장을 위한 공간 미리 할당 ---
totalSamples = ceil(duration * samplingRate) + chunkSize * 2;
sensorData = struct();
sensorData.timestamps = nan(totalSamples, 1);
% 3D Matrix: (sample_index, axis_XYZ, sensor_index)
sensorData.axes = nan(totalSamples, numAxes, numSensors); 
currentIndex = 1;
try
    fprintf('시리얼 포트(%s)에 연결 중...\n', comPort);
    % 기존 연결이 있다면 모두 삭제
    delete(instrfind({'Port'}, {comPort}));
    
    device = serialport(comPort, baudRate);
    device.InputBufferSize = frameBytes * 10; 
    configureTerminator(device, "LF");
    
    fprintf('Arduino 부팅 대기 중 (5초)...\n');
    pause(2);
    
    fprintf('Arduino에 시작 신호 전송...\n');
    write(device, 's', "char");
    
    flush(device);
    fprintf('연결 및 동기화 성공.\n');
catch ME
    fprintf('시리얼 포트 연결 또는 동기화 실패: %s\n', ME.message); return;
end

fig = uifigure('Name', 'Sensor Data Acquisition', 'NumberTitle', 'off');
t = tiledlayout(numSensors, numAxes, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = gobjects(numSensors, numAxes);
for i = 1:numSensors
    for j = 1:numAxes
        ax(i,j) = nexttile;
        axis_char = char('X' + j - 1);
        title(sprintf('Sensor %d - %c Axis', i, axis_char));
    end
end
linkaxes(ax(:),'x');

startTime = tic; 
lastPrintTime = tic;  % 시간 출력용
fprintf('데이터 수신을 시작합니다... (총 %d초)\n', duration);

flush(device);
while toc(startTime) < duration
    while device.NumBytesAvailable >= frameBytes
        rawBytes = read(device, frameBytes, "uint8");
        rawMatrix = reshape(uint8(rawBytes), bytesPerSample, chunkSize).';
        
        startIdx = currentIndex; endIdx = startIdx + chunkSize - 1;
        if endIdx > totalSamples, break; end
        
        sensorData.timestamps(startIdx:endIdx) = typecast(reshape(rawMatrix(:,1:4).',[],1), 'uint32');
        all_axes_data = typecast(reshape(rawMatrix(:,5:end).',[],1), 'int16');
        sensorData.axes(startIdx:endIdx, :, :) = permute(reshape(all_axes_data, numAxes, numSensors, chunkSize), [3 1 2]);

        currentIndex = endIdx + 1;
    end
    
    % 1초마다 경과 시간 출력
    if toc(lastPrintTime) > 1.0
        elapsedTime = toc(startTime);
        fprintf('경과 시간: %.1f초 / %d초\n', elapsedTime, duration);
        lastPrintTime = tic;
    end
end

fprintf('데이터 수집 완료. 후처리를 시작합니다...\n');

last_idx = find(~isnan(sensorData.timestamps), 1, 'last');
if isempty(last_idx)
    fprintf('수신된 센서 데이터가 없습니다.\n');
else
    ts = sensorData.timestamps(1:last_idx); 
    axes_data = sensorData.axes(1:last_idx, :, :);
    
    timestamps_sec = (double(ts) - double(ts(1))) / 1e6;
    
    actual_fs = numel(ts) / timestamps_sec(end);
    fprintf('최종 수신된 샘플 개수: %d\n', numel(ts));
    fprintf('실제 평균 샘플링 속도: %.2f Hz\n', actual_fs);
    
    fprintf('데이터 시각화 중...\n');
    for i = 1:numSensors
        for j = 1:numAxes
            plot(ax(i,j), axes_data(:, j, i));
        end
    end
    
end

clear device;
fprintf('모든 작업이 완료되었습니다.\n');

%% ========================================================================
%  필터링 및 최종 시각화 - X, Y, Z 모든 축 데이터 추출
% =========================================================================
last_idx = find(~isnan(sensorData.timestamps), 1, 'last');
if isempty(last_idx)
    fprintf('수신된 센서 데이터가 없습니다.\n');
else
    ts = sensorData.timestamps(1:last_idx); 
    timestamps_sec = (double(ts) - double(ts(1))) / 1e6;
    
    actual_fs = numel(ts) / timestamps_sec(end);
    fprintf('\n=== 데이터 수집 정보 ===\n');
    fprintf('최종 수신된 샘플 개수: %d\n', numel(ts));
    fprintf('실제 평균 샘플링 속도: %.2f Hz\n', actual_fs);
    
    % ===== 3D 배열에서 모든 축 데이터 추출 =====
    % sensorData.axes 구조: (samples, axes[X=1,Y=2,Z=3], sensors)
    axis_names = {'X', 'Y', 'Z'};
    
    % 데이터 추출 및 구조화
    raw_data = struct();
    for sensor_idx = 1:numSensors
        for axis_idx = 1:numAxes
            field_name = sprintf('S%d_%s', sensor_idx, axis_names{axis_idx});
            data = squeeze(sensorData.axes(1:last_idx, axis_idx, sensor_idx));
            
            % 열 벡터 확인
            if size(data, 1) == 1
                data = data(:);
            end
            
            raw_data.(field_name) = data;
        end
    end
    
    fprintf('\n추출된 데이터 필드:\n');
    disp(fieldnames(raw_data));
    
    % --- 저역 통과 필터링 ---
    fprintf('\n신호에 저역 통과 필터를 적용하여 노이즈를 제거합니다...\n');
    
    fs = actual_fs;
    fc = 2000;  % Cutoff Frequency (Hz)
    
    % 4차 버터워스 저역 통과 필터 설계
    [b, a] = butter(4, fc/(fs/2), 'low');
    
    % 필터링된 데이터 저장
    filtered_data = struct();
    field_list = fieldnames(raw_data);
    
    for i = 1:length(field_list)
        field_name = field_list{i};
        filtered_data.(field_name) = filtfilt(b, a, double(raw_data.(field_name)));
    end
    
    % --- 시각화 ---
    fprintf('데이터 시각화 중...\n');
    t_acc = (0:length(ts)-1) / fs;
    
    % 각 센서별로 X, Y, Z를 서브플롯으로 표시
    for sensor_idx = 1:numSensors
        figure('Name', sprintf('Sensor %d - All Axes', sensor_idx), 'Position', [100, 100, 1200, 800]);
        
        for axis_idx = 1:numAxes
            field_name = sprintf('S%d_%s', sensor_idx, axis_names{axis_idx});
            
            subplot(3, 1, axis_idx);
            plot(t_acc, raw_data.(field_name), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
            hold on;
            plot(t_acc, filtered_data.(field_name), 'b-', 'LineWidth', 1.5);
            hold off;
            
            legend('Raw Signal', 'Filtered Signal', 'Location', 'northeast');
            title(sprintf('Sensor %d - %s Axis', sensor_idx, axis_names{axis_idx}));
            xlabel('Time (s)');
            ylabel('Amplitude');
%             xlim([4 4.05])
            grid on;
        end
    end
    
    % --- 오디오 파일로 저장 ---
    % fprintf('\n오디오 파일로 저장 중...\n');
    % 
    % for sensor_idx = 1:numSensors
    %     for axis_idx = 1:numAxes
    %         field_name = sprintf('S%d_%s', sensor_idx, axis_names{axis_idx});
    % 
    %         % 정규화
    %         data_normalized = normalize(filtered_data.(field_name), "range", [-1 1]);
    % 
    %         % 파일명 생성
    %         filename = sprintf('acc_sensor%d_%s.wav', sensor_idx, lower(axis_names{axis_idx}));
    % 
    %         % 오디오 저장
    %         audiowrite(filename, data_normalized, round(actual_fs));
    %         fprintf('  저장 완료: %s\n', filename);
    %     end
    % end
    % 
    % fprintf('\n모든 데이터 처리가 완료되었습니다.\n');
    % fprintf('총 %d개의 오디오 파일이 생성되었습니다.\n', numSensors * numAxes);
end

%% ========================================================================
%  선택적: CSV 파일로도 저장
% =========================================================================
fprintf('\nCSV 파일로도 저장하시겠습니까? (주석 제거 후 실행)\n');

% CSV 저장 코드 (필요시 주석 해제)
csv_table = table();
csv_table.Time_sec = timestamps_sec;

for sensor_idx = 1:numSensors
    for axis_idx = 1:numAxes
        field_name = sprintf('S%d_%s', sensor_idx, axis_names{axis_idx});
        col_name = sprintf('Sensor%d_%s_Raw', sensor_idx, axis_names{axis_idx});
        csv_table.(col_name) = raw_data.(field_name);

        col_name_filt = sprintf('Sensor%d_%s_Filtered', sensor_idx, axis_names{axis_idx});
        csv_table.(col_name_filt) = filtered_data.(field_name);
    end
end

writetable(csv_table, 'sensor_data_all_axes_afer.csv');
fprintf('CSV 파일 저장 완료: sensor_data_all_axes.csv\n');
