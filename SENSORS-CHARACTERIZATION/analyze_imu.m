clc; clear; close all;
%% Load file
% Fix scaling of logs Copy_of_log4 decoded with old DBC
accel_scale_fix = 0.00006103515625 / 0.000061035;
gyro_scale_fix  = 0.00763358778625954 / 0.0609756;
fprintf("Accel fix: %.15f\n", accel_scale_fix);
fprintf("Gyro fix:  %.15f\n", gyro_scale_fix);

% parquet_file = 'data/SHORT.PARQUET';
parquet_file = 'data/Copy_of_log4.PARQUET';
info = parquetinfo(parquet_file);
% Actually load file
T = parquetread(parquet_file);
for i = 1:length(T.Properties.VariableDescriptions)
    disp(T.Properties.VariableDescriptions{i});
end

T.can_signals_ACCELEROMETER_AX = T.can_signals_ACCELEROMETER_AX * accel_scale_fix;
T.can_signals_ACCELEROMETER_AY = T.can_signals_ACCELEROMETER_AY * accel_scale_fix;
T.can_signals_ACCELEROMETER_AZ = T.can_signals_ACCELEROMETER_AZ * accel_scale_fix;
T.can_signals_GYROSCOPE_GX = T.can_signals_GYROSCOPE_GX * gyro_scale_fix;
T.can_signals_GYROSCOPE_GY = T.can_signals_GYROSCOPE_GY * gyro_scale_fix;
T.can_signals_GYROSCOPE_GZ = T.can_signals_GYROSCOPE_GZ * gyro_scale_fix;

if 1
    T.time = T.timestamp_s - T.timestamp_s(1);
    gidx = ~isnan(T.can_signals_GYROSCOPE_GX);
    aidx = ~isnan(T.can_signals_ACCELEROMETER_AX);
    figure
    subplot(2,1,1); hold on;
    plot(T.time(gidx), T.can_signals_GYROSCOPE_GX(gidx));
    plot(T.time(gidx), T.can_signals_GYROSCOPE_GY(gidx));
    plot(T.time(gidx), T.can_signals_GYROSCOPE_GZ(gidx));

    subplot(2,1,2); hold on;
    plot(T.time(aidx), T.can_signals_ACCELEROMETER_AX(aidx));
    plot(T.time(aidx), T.can_signals_ACCELEROMETER_AY(aidx));
    plot(T.time(aidx), T.can_signals_ACCELEROMETER_AZ(aidx));
end

%% Extract the gyro and accel sensor data

gyro_data = sns_cut_fragment(T.time, T.can_signals_GYROSCOPE_GX, 50, 5350);
gyro_data = [gyro_data; sns_cut_fragment(T.time, T.can_signals_GYROSCOPE_GY, 50, 5350)];
gyro_data = [gyro_data; sns_cut_fragment(T.time, T.can_signals_GYROSCOPE_GZ, 50, 5350)];

% Plot biasu gyroscope
fff = figure('Name','sensor_gyro_output','Units','centimeters','Position',[2 2 16 8]);
    i = 2;
    every=50;
    plot(gyro_data(i).t(1:every:end), gyro_data(i).x(1:every:end), '.', "MarkerSize",1); hold on; grid on;
    plot(gyro_data(i).t, movmean(gyro_data(i).x, 6000), "LineWidth",1.5); hold on; grid on;
    xlabel('Time [s]','FontName','Times New Roman','FontSize',9)
    % ylabel('Angular rate [deg/s]','FontName','Times New Roman')
    ylabel('Angular rate [{\circ}/s]','FontName','Times New Roman','FontSize',9)
    legend('Measurement samples', '60 s moving average', 'Location','best','FontName','Times New Roman','FontSize',9);
    grid on;
    set(gca,'FontName','Times New Roman');
    set(gca,'FontSize',9);
    xlim([0 5400])

accel_data = sns_cut_fragment(T.time, T.can_signals_ACCELEROMETER_AX, 50, 5350);
accel_data = [accel_data; sns_cut_fragment(T.time, T.can_signals_ACCELEROMETER_AY, 50, 5350)];
accel_data = [accel_data; sns_cut_fragment(T.time, T.can_signals_ACCELEROMETER_AZ, 50, 5350)];

% return

%% Analyze
% q_step_gyro = 0.0609756; % +/-2000 deg/s per 16 bits OLD
q_step_gyro = 0.00763358778625954; % +/- 250 deg/s per 16 bits
% % % 16.4 LSB/DPS
% % % 1/16.4
gyro = [];
for i = 1:length(gyro_data)
    gyro(i).params = sns_analyze_imu(gyro_data(i).t, detrend(gyro_data(i).x), q_step_gyro);
    disp(gyro(i).params)
end


q_step_accel = 0.00006103515625; % +/-2 G per 16 bits should be but two lower bits are zeroed... so effectively quantization step is 4 times bigger 
% 2 / 2^15;
% 16384 LSB/g
accel = [];
for i = 1:length(accel_data)
    accel(i).params = sns_analyze_imu(accel_data(i).t, (accel_data(i).x), q_step_accel);
    disp(accel(i).params)
end

% for i = 1:length(gyro_data)
%     x = detrend(gyro_data(i).x) - mean(detrend(gyro_data(i).x));
%     acf = xcorr(x, 10, 'coeff');
% 
%     figure
%     lags = -10:10;
%     stem(lags, acf);
%     grid on;
%     title('Autocorrelation of gyro noise');
% end
% 
% for i = 1:length(accel_data)
%     x = (accel_data(i).x) - mean((accel_data(i).x));
%     acf = xcorr(x, 10, 'coeff');
% 
%     figure
%     lags = -10:10;
%     stem(lags, acf);
%     grid on;
%     title('Autocorrelation of gyro noise');
% end

%% MAGISTERA WYRKESY

% Plot histogramu accelerometer
f = figure('Name','sensor_acc_histogram','Units','centimeters','Position',[2 2 8 6]); grid on; hold on;
    i=1;
    value = accel_data(i).x;
    idx = 4700 < accel_data(i).t & accel_data(i).t < 5300;
    value = value(idx);

    q_step = q_step_accel*4; % There is an issue with quantization right now so has to be 4x...
    % But thats not tragically bad, the number of bins is enormous anyway
    noise = (value - mean(value));
    edges = (max(value) - min(value))/q_step;
    edges = (-edges:edges+1)*q_step - q_step/2; 
    histogram(noise, edges, 'Normalization','pdf', 'FaceAlpha',0.5, 'FaceColor',[1 0 0]);
    % histogram(noise, "BinWidth", q_step_accel);

    i=2;
    value = accel_data(i).x;
    noise = (value - mean(value));
    histogram(noise, edges, 'Normalization','pdf', 'FaceAlpha',0.5, 'FaceColor',[0 1 0]);

    i=3;
    value = accel_data(i).x;
    noise = (value - mean(value));
    histogram(noise, edges, 'Normalization','pdf', 'FaceAlpha',0.5, 'FaceColor',[0 0 1]);

    legend('X','Y','Z','FontName','Times New Roman','FontSize',9);
    xlim([-0.012 0.012]);

xlabel('Deviation from mean acceleration [g]','FontName','Times New Roman','FontSize',9)
ylabel('Probability density','FontName','Times New Roman','FontSize',9)
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);

%% Plot histogramu gyroscope
bias = [];

ff = figure('Name','sensor_gyro_histogram','Units','centimeters','Position',[2 2 8 6]); grid on; hold on;
    i = 1;
    value = gyro_data(i).x;
    % value = detrend(value);
    idx = 4700 <= gyro_data(i).t & gyro_data(i).t <= 5300;
    value = value(idx);
    bias(i) = mean(value);
    % value = detrend(value);

    q_step = q_step_gyro;
    noise = (value - mean(value));
    edges = (max(value) - min(value))/q_step;
    edges = (-edges:edges+1)*q_step - q_step/2; 
    histogram(noise, edges, 'Normalization','pdf', 'FaceAlpha',0.5, 'FaceColor',[1 0 0]);
    % histogram(noise, "BinWidth", q_step_gyro, 'Normalization','pdf', 'FaceAlpha',0.4, 'FaceColor',[1 0 0]);
    % histogram(noise, "BinWidth", q_step_gyro);
    fprintf("bias: %f, ", bias(i));
    fprintf("gyroscope std over shorter period: %f, detrended std: %f\n", std(value), std(detrend(value)))

    i = 2;
    value = gyro_data(i).x;
    value = value(idx);
    bias(i) = mean(value);
    value = detrend(value);
    noise = (value - mean(value));
    histogram(noise, edges, 'Normalization','pdf', 'FaceAlpha',0.5, 'FaceColor',[0 1 0]);
    % histogram(noise, "BinWidth", q_step_gyro, 'Normalization','pdf', 'FaceAlpha',0.4, 'FaceColor',[0 1 0]);
    fprintf("bias: %f, ", bias(i));
    fprintf("gyroscope std over shorter period: %f, detrended std: %f\n", std(value), std(detrend(value)))

    i = 3;
    value = gyro_data(i).x;
    value = value(idx);
    bias(i) = mean(value);
    value = detrend(value);
    noise = (value - mean(value));
    histogram(noise, edges, 'Normalization','pdf', 'FaceAlpha',0.5, 'FaceColor',[0 0 1]);
    % histogram(noise, "BinWidth", q_step_gyro, 'Normalization','pdf', 'FaceAlpha',0.4, 'FaceColor',[0 0 1]);
    fprintf("bias: %f, ", bias(i));
    fprintf("gyroscope std over shorter period: %f, detrended std: %f\n", std(value), std(detrend(value)))

    legend('X','Y','Z','FontName','Times New Roman','FontSize',9);
    xlim([-0.22 0.22]);

xlabel('Deviation from mean angular rate [{\circ}/s]','FontName','Times New Roman','FontSize',9)
ylabel('Probability density','FontName','Times New Roman','FontSize',9)
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);

%% quantization check

unique_acc = unique(accel_data(i).x);
unique_acc_du = diff(unique_acc);
fprintf("minimal accelerometer q_step: %.12f\n", min(unique_acc_du));
unique_gyro = unique(gyro_data(i).x);
unique_gyro_du = diff(unique_gyro);
fprintf("minimal gyrscope q_step: %.12f\n", min(unique_gyro_du));
% 
% %% MISCONFIGURED ACCEL!!! CHECK 
% parquet_file = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_04_26_divonnes\logs_parquet\log0.parquet";
% parquet_file = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_01_25_zakrzowek\rpi_logs\log0.parquet";
% T = parquetread(parquet_file);
% unique_acc = unique(T.inputs_ACCELEROMETER_Y);
% unique_acc_du = diff(unique_acc);
% min(unique_acc_du)
% % okay this is kinda bad, go and see if low bits of the rawData in the
% % microcontroller are always 0 and then see if configuration is written to
% % the MPU... 

%%
exportgraphics(f, 'sensor_acc_histogram.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none', ...
    'Units', 'centimeters', ...
    'Width', 8, ...
    'Height', 6);
exportgraphics(ff, 'sensor_gyro_histogram.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none', ...
    'Units', 'centimeters', ...
    'Width', 8, ...
    'Height', 6);
exportgraphics(fff, 'sensor_gyro_output.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none', ...
    'Units', 'centimeters', ...
    'Width', 16, ...
    'Height', 8);

%% Save

accelerometer_noise_parameters.quantization_step = q_step_accel * 4; %% TODO FIXME 
accelerometer_noise_parameters.noise_sigma = [
    accel(1).params.noise_sigma;
    accel(2).params.noise_sigma;
    accel(3).params.noise_sigma
];

gyroscope_noise_parameters.quantization_step = q_step_gyro;
gyroscope_noise_parameters.noise_sigma = [
    gyro(1).params.noise_sigma;
    gyro(2).params.noise_sigma;
    gyro(3).params.noise_sigma
];
gyroscope_noise_parameters.bias = [
    bias(1);
    bias(2);
    bias(3)
];

save imu_noise_parameters.mat gyroscope_noise_parameters accelerometer_noise_parameters

%% final values
% return
% clc
disp('gyro sigmas, biases and accel sigmas:')
disp(gyroscope_noise_parameters.noise_sigma)
disp(gyroscope_noise_parameters.bias)
disp(accelerometer_noise_parameters.noise_sigma)

%% Simulate 
return
clc; clear;

load imu_noise_parameters.mat

t = 0:0.1:60*30;
accel = [zeros(size(t)); zeros(size(t)); ones(size(t))];
gyro = [zeros(size(t)); zeros(size(t)); zeros(size(t))];

accel_simulated = apply_sensor_noise(accel(1,:), accelerometer_noise_parameters);
disp(sns_analyze_imu(t, accel_simulated(1,:), accelerometer_noise_parameters.quantization_step))

gyro_simulated = apply_sensor_noise(gyro(1,:), gyroscope_noise_parameters);
disp(sns_analyze_imu(t, gyro_simulated(1,:), gyroscope_noise_parameters.quantization_step))
