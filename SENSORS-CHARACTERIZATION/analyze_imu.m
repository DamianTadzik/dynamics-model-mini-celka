clc; clear; close all;
%% Load file
% parquet_file = 'data/SHORT.PARQUET';
parquet_file = 'data/LONG.PARQUET';
info = parquetinfo(parquet_file);
% Actually load file
T = parquetread(parquet_file);
for i = 1:length(T.Properties.VariableDescriptions)
    disp(T.Properties.VariableDescriptions{i});
end

if 1
    figure
    subplot(2,1,1); hold on;
    plot(T.seconds_since_start, T.GYROSCOPE_GX);
    plot(T.seconds_since_start, T.GYROSCOPE_GY);
    plot(T.seconds_since_start, T.GYROSCOPE_GZ);

    subplot(2,1,2); hold on;
    plot(T.seconds_since_start, T.ACCELEROMETER_AX);
    plot(T.seconds_since_start, T.ACCELEROMETER_AY);
    plot(T.seconds_since_start, T.ACCELEROMETER_AZ);
end

%% Extract the gyro and accel sensor data

gyro_data = sns_cut_fragment(T.seconds_since_start, T.GYROSCOPE_GX, 50, 4750);
gyro_data = [gyro_data; sns_cut_fragment(T.seconds_since_start, T.GYROSCOPE_GY, 50, 4750)];
gyro_data = [gyro_data; sns_cut_fragment(T.seconds_since_start, T.GYROSCOPE_GZ, 50, 4750)];

% Plot biasu gyroscope
fff = figure('Name','sensor_gyro_output','Units','inches','Position',[2 2 9 3]);
    i = 2;
    every=100;
    plot(gyro_data(i).t(1:every:end), gyro_data(i).x(1:every:end), '.', "MarkerSize",1); hold on; grid on;
    plot(gyro_data(i).t, movmean(gyro_data(i).x, 6000), "LineWidth",1.5); hold on; grid on;
    xlabel('Time [s]','FontName','Times New Roman')
    % ylabel('Angular rate [deg/s]','FontName','Times New Roman')
    ylabel('Angular rate [$^\circ$/s]','FontName','Times New Roman', 'Interpreter','latex')
    legend('Measurement samples', '60 s moving average', 'Location','best','FontName','Times New Roman');
    set(gca,'FontName','Times New Roman');
    xlim([0 4800])

accel_data = sns_cut_fragment(T.seconds_since_start, T.ACCELEROMETER_AX, 50, 4750);
accel_data = [accel_data; sns_cut_fragment(T.seconds_since_start, T.ACCELEROMETER_AY, 50, 4750)];
accel_data = [accel_data; sns_cut_fragment(T.seconds_since_start, T.ACCELEROMETER_AZ, 50, 4750)];



%% Analyze
q_step_gyro = 0.0609756; % +/-2000 deg/s per 16 bits 
% 16.4 LSB/DPS
% 1/16.4
gyro = [];
for i = 1:length(gyro_data)
    gyro(i).params = sns_analyze_imu(gyro_data(i).t, detrend(gyro_data(i).x), q_step_gyro);
    disp(gyro(i).params)
end


q_step_accel = 0.000061035; % +/-2 G per 16 bits
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
f = figure('Name','sensor_acc_histogram','Units','inches','Position',[2 2 6 4]); grid on; hold on;
    i=1;
    value = accel_data(i).x;
    % idx = 4200 < accel_data(i).t || accel_data(i).t < 4700;
    % value = value(idx);

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

    legend('X axis','Y axis','Z axis');
    xlim([-0.012 0.012]);

xlabel('Deviation from the measured acceleration mean [g]','FontName','Times New Roman')
ylabel('Probability density','FontName','Times New Roman')
set(gca,'FontName','Times New Roman');


%% Plot histogramu gyroscope
bias = [];

ff = figure('Name','sensor_gyro_histogram','Units','inches','Position',[2 2 6 4]); grid on; hold on;
    i = 1;
    value = gyro_data(i).x;
    % value = detrend(value);
    idx = 4000 <= gyro_data(i).t & gyro_data(i).t <= 4400;
    value = value(idx);
    bias(i) = mean(value);
    value = detrend(value);

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

    legend('X axis','Y axis','Z axis');
    xlim([-0.22 0.22]);

xlabel('Deviation from gyroscope measurement mean [$^\circ$/s]','FontName','Times New Roman', 'Interpreter','latex')
ylabel('Probability density','FontName','Times New Roman')
set(gca,'FontName','Times New Roman');


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
    'BackgroundColor', 'none');
exportgraphics(ff, 'sensor_gyro_histogram.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none');
exportgraphics(fff, 'sensor_gyro_output.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none');

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
clc
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
