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
%% Extract the gyro and accel sensor data

gyro_data = sns_cut_fragment(T.seconds_since_start, T.GYROSCOPE_GX, 50, 4750);
gyro_data = [gyro_data; sns_cut_fragment(T.seconds_since_start, T.GYROSCOPE_GY, 50, 4750)];
gyro_data = [gyro_data; sns_cut_fragment(T.seconds_since_start, T.GYROSCOPE_GZ, 50, 4750)];

accel_data = sns_cut_fragment(T.seconds_since_start, T.ACCELEROMETER_AX, 50, 4750);
accel_data = [accel_data; sns_cut_fragment(T.seconds_since_start, T.ACCELEROMETER_AY, 50, 4750)];
accel_data = [accel_data; sns_cut_fragment(T.seconds_since_start, T.ACCELEROMETER_AZ, 50, 4750)];


%% Analyze
q_step = 0.0609756; % +/-2000 deg/s per 16 bits 
% 16.4 LSB/DPS
% 1/16.4
gyro = [];
for i = 1:length(gyro_data)
    gyro(i).params = sns_analyze_imu(gyro_data(i).t, detrend(gyro_data(i).x), q_step);
    disp(gyro(i).params)
end


q_step = 0.000061035; % +/-2 G per 16 bits
% 2 / 2^15;
% 16384 LSB/g
accel = [];
for i = 1:length(accel_data)
    accel(i).params = sns_analyze_imu(accel_data(i).t, detrend(accel_data(i).x), q_step);
    disp(accel(i).params)
end


for i = 1:length(gyro_data)
    x = detrend(gyro_data(i).x) - mean(detrend(gyro_data(i).x));
    acf = xcorr(x, 10, 'coeff');
    
    figure
    lags = -10:10;
    stem(lags, acf);
    grid on;
    title('Autocorrelation of gyro noise');
end

for i = 1:length(accel_data)
    x = (accel_data(i).x) - mean((accel_data(i).x));
    acf = xcorr(x, 10, 'coeff');
    
    figure
    lags = -10:10;
    stem(lags, acf);
    grid on;
    title('Autocorrelation of gyro noise');
end
%% Save

accelerometer_noise_parameters.quantization_step = 0.000061035;
accelerometer_noise_parameters.noise_sigma = [
    accel(1).params.noise_sigma;
    accel(2).params.noise_sigma;
    accel(3).params.noise_sigma
];

gyroscope_noise_parameters.quantization_step = 0.0609756;
gyroscope_noise_parameters.noise_sigma = [
    gyro(1).params.noise_sigma;
    gyro(2).params.noise_sigma;
    gyro(3).params.noise_sigma
];

save imu_noise_parameters.mat gyroscope_noise_parameters accelerometer_noise_parameters


%% Simulate 
clc; clear;

load imu_noise_parameters.mat

t = 0:0.1:60*30;
accel = [zeros(size(t)); zeros(size(t)); ones(size(t))];
gyro = [zeros(size(t)); zeros(size(t)); zeros(size(t))];

accel_simulated = apply_sensor_noise(accel(1,:), accelerometer_noise_parameters);
disp(sns_analyze_imu(t, accel_simulated(1,:), accelerometer_noise_parameters.quantization_step))

gyro_simulated = apply_sensor_noise(gyro(1,:), gyroscope_noise_parameters);
disp(sns_analyze_imu(t, gyro_simulated(1,:), gyroscope_noise_parameters.quantization_step))
