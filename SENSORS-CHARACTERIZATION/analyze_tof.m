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

%% Extract the TOF sensor data

sensor_data = sns_cut_fragment(T.seconds_since_start, T.DISTANCE_FORE_FEEDBACK_RANGE_MM_L, 50, 4750);
sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_FORE_FEEDBACK_RANGE_MM_R, 50, 4750)];
sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_ACHTER_FEEDBACK_RANGE_MM_L, 50, 4750)];
sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_ACHTER_FEEDBACK_RANGE_MM_R, 50, 4750)];

%% Analyze the sensor data

q_step = 2; % 2 mm step
sensor = [];
for i = 1:length(sensor_data)
    sensor(i).params = sns_analyze_tof(sensor_data(i).t, sensor_data(i).x, q_step);
end

% params_FL = sns_analyze_tof(t_FL, x_FL, q_step)
% params_FR = sns_analyze_tof(t_FR, x_FR, q_step)
% params_AL = sns_analyze_tof(t_AL, x_AL, q_step)
% params_AR = sns_analyze_tof(t_AR, x_AR, q_step)


% Plot noise_sigma vs x_mode
x_modes = [];
noise_sigmas = [];
for i = 1:length(sensor)
    x_modes(i) = sensor(i).params.x_mode;
    noise_sigmas(i) = sensor(i).params.noise_sigma;
end
figure;
plot(x_modes, noise_sigmas, 'o');
xlabel('x\_mode');
ylabel('Noise \sigma');
title('Noise \sigma vs x\_mode');
grid on;

%% Checking the noise at different ranges

T = parquetread('data\TOF_DISTANCES.PARQUET');
for i = 1:length(T.Properties.VariableDescriptions)
    disp(T.Properties.VariableDescriptions{i});
end

sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_FORE_FEEDBACK_RANGE_MM_L, 4, 64)];
sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_FORE_FEEDBACK_RANGE_MM_L, 80, 140)];
sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_FORE_FEEDBACK_RANGE_MM_L, 532, 592)];

sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_FORE_FEEDBACK_RANGE_MM_R, 4, 64)];
sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_FORE_FEEDBACK_RANGE_MM_R, 80, 140)];
sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_FORE_FEEDBACK_RANGE_MM_R, 455, 515)];

sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_ACHTER_FEEDBACK_RANGE_MM_L, 158, 218)];
sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_ACHTER_FEEDBACK_RANGE_MM_L, 224, 284)];
sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_ACHTER_FEEDBACK_RANGE_MM_L, 375, 435)];

sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_ACHTER_FEEDBACK_RANGE_MM_R, 158, 218)];
sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_ACHTER_FEEDBACK_RANGE_MM_R, 224, 284)];
sensor_data = [sensor_data; sns_cut_fragment(T.seconds_since_start, T.DISTANCE_ACHTER_FEEDBACK_RANGE_MM_R, 300, 360)];

q_step = 2; % 2 mm step
for i = 1:length(sensor_data)
    sensor(i).params = sns_analyze_tof(sensor_data(i).t, sensor_data(i).x, q_step);
end

% Plot noise_sigma vs x_mode
x_modes = [];
noise_sigmas = [];
for i = 1:length(sensor)
    x_modes = [x_modes; sensor(i).params.x_mode];
    noise_sigmas = [noise_sigmas; sensor(i).params.noise_sigma];
end

% Pearson correlation
R = corr(x_modes, noise_sigmas)

coeffs = polyfit(x_modes, noise_sigmas, 1);
a = coeffs(1);
b = coeffs(2);

%%
fig = figure('Name', 'noise_vs_distance','Units','inches','Position',[2 2 6 4]);
plot(x_modes, noise_sigmas, 'o'); hold on;
plot(x_modes, a*x_modes + b, '--'); hold off;
% xlabel('x\_mode', 'FontName', 'Times New Roman'); 
% ylabel('Noise \sigma', 'FontName', 'Times New Roman'); 
% title('Noise \sigma vs x\_mode');
grid on;
set(gca,'FontName','Times New Roman');

xlabel('Measured distance mode [mm]', 'FontName', 'Times New Roman')
ylabel('Noise standard deviation \sigma [mm]', 'FontName', 'Times New Roman')
legend('Measurements', 'Linear regression', 'Location', 'northwest', 'FontName', 'Times New Roman')

exportgraphics(fig, 'sensor_TOF_noise_vs_distance.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none');

%% Save the calculated coefficents to be used in a noise generator

tof_noise_parameters.quantization_step = 2;
tof_noise_parameters.noise_sigma_a = a;
tof_noise_parameters.noise_sigma_b = b;
% save tof_noise_parameters.mat tof_noise_parameters

%% Simulate the signal and analyze
% clc; clear; 

load tof_noise_parameters.mat

t = 0:0.1:60;
for k = 50:50:150

    % Ideal input signal is never quantized, it is some real value signal
    x_ideal = k * ones(size(t));

    % Apply noise to ideal signal
    x_simulated = apply_sensor_noise(x_ideal, tof_noise_parameters);

    params_simulated = sns_analyze_tof(t, x_simulated, tof_noise_parameters.quantization_step)
end

%% non normality chek


for i = 1:length(sensor_data)
% for i = 15
    figure
    qqplot(sensor_data(i).x)
end

%% MAGISTERKA WYKRESY

i = 15; % figura 15 jako reprezentant 
% i = 9; % figura 9 do QQ plota mega benger
% i = 7; % najlepszy kompromis

time = sensor_data(i).t - sensor_data(i).t(1);
value = sensor_data(i).x;

% Plot przebiegu z sensora
f = figure('Name','sensor_output','Units','inches','Position',[2 2 6 4]);
plot(time, value, '-o')
xlabel('Time [s]','FontName','Times New Roman')
ylabel('Measured distance [mm]','FontName','Times New Roman')
set(gca,'FontName','Times New Roman');

exportgraphics(f, 'sensor_output.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none');

% Plot histogramu
ff = figure('Name','sensor_TOF_histogram','Units','inches','Position',[2 2 6 4]);
    q_step = 2;
    noise = (value - mode(value));
    edges = (max(value) - min(value))/q_step;
    edges = (-edges:edges+1)*q_step - q_step/2; 
    histogram(noise, edges); grid on;

xlabel('Deviation from modal distance [mm]','FontName','Times New Roman')
ylabel('Number of samples','FontName','Times New Roman')

set(gca,'FontName','Times New Roman');
exportgraphics(ff, 'sensor_TOF_histogram.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none');

% % Q-Q plot?
% fff = figure(Name='qq_plot');
% set(gca,'FontName','Times New Roman');
% qqplot(value)

% porownanie jest wyzej w skrypcie