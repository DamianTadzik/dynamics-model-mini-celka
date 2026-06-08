clc; clear;

%% Load and stitch? files from RPI log[0 1 2 3 4 5 6].parquet
base = '../../logs-mini-celka/logs_storage/2026_01_25_zakrzowek/rpi_logs/';
% files = ["log0.parquet","log1.parquet","log2.parquet","log3.parquet"];
files = ["log0.parquet"];

T_RPI = table();

for f = files
    T_RPI = [T_RPI; parquetread(fullfile(base, f))];   %#ok<AGROW>
end

% for i = 1:length(T_GFR.Properties.VariableDescriptions)
%     disp(T_GFR.Properties.VariableDescriptions{i});
% end
disp("RPI LOG FIELDS")
disp(T_RPI.Properties.VariableNames')

%% Load and stitch files from GFR [001 002 003 004].parquet
base = '../../logs-mini-celka/logs_storage/2026_01_25_zakrzowek/GFR_logs/';
files = ["001.parquet","002.parquet","003.parquet","004.parquet"];

T_GFR = table();

for f = files
    T_GFR = [T_GFR; parquetread(fullfile(base, f))];   %#ok<AGROW>
end

% for i = 1:length(T_GFR.Properties.VariableDescriptions)
%     disp(T_GFR.Properties.VariableDescriptions{i});
% end
disp("GFR LOG FIELDS")
disp(T_GFR.Properties.VariableNames')

%% Plots
t_s__gfr = T_GFR.time_s;
t_s__rpi = T_RPI.timestamp;

% figure(1);
% subplot(2, 1, 1);
% plot(t_s__gfr)
% subplot(2, 1, 2);
% plot(diff(t_s__gfr))
% 
% 

figure(2)
subplot(2, 1, 1);
plot(t_s__gfr, T_GFR.accel_x_mps2); hold on; % x to right
plot(t_s__gfr, T_GFR.accel_y_mps2); % y to front
plot(t_s__gfr, T_GFR.accel_z_mps2); % z to up

subplot(2, 1, 2)
plot(t_s__rpi, T_RPI.inputs_ACCELEROMETER_X); hold on;
plot(t_s__rpi, T_RPI.inputs_ACCELEROMETER_Y);
plot(t_s__rpi, T_RPI.inputs_ACCELEROMETER_Z);

