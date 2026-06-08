clc; clear;
%% Load file
parquet_file = '../../logs-mini-celka/logs_storage/2026_01_25_zakrzowek/rpi_logs/log0.parquet';
info = parquetinfo(parquet_file);
% Actually load file
T = parquetread(parquet_file);
for i = 1:length(T.Properties.VariableDescriptions)
    disp(T.Properties.VariableDescriptions{i});
end

%% Extract
% x_hat/z_m
% x_hat/phi_rad
% x_hat/theta_rad

% Time (epoch to relative)
t = T.timestamp;
t = t - t(1);           % Simulink-safe time
assert(all(diff(t) > 0))

% Extract states
z     =  T.x_hat_z_m;
phi   =  T.x_hat_phi_rad;
theta =  T.x_hat_theta_rad;

% Remove NaNs
valid = all(~isnan([z phi theta]), 2);
t = t(valid);
z = z(valid);
phi = phi(valid);
theta = theta(valid);

%% Timeseries for Simulink
z_ts = timeseries(z, t);
phi_ts = timeseries(phi, t);
theta_ts = timeseries(theta, t);

z_ts.TimeInfo.Units = 'seconds';
phi_ts.TimeInfo.Units = 'seconds';
theta_ts.TimeInfo.Units = 'seconds';


%% Time of simulation setting
t(1)
t(end)