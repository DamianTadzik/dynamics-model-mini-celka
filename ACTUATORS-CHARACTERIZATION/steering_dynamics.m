clc; clear; close all;
%% Load file
parquet_file_125 = 'data/LOG000.PARQUET';
info = parquetinfo(parquet_file_125);
% Actually load file
T125 = parquetread(parquet_file_125);
for i = 1:length(T125.Properties.VariableDescriptions)
    disp(T125.Properties.VariableDescriptions{i});
end



[tRADIO_CONTROL_MODE_SWITCH, RADIO_CONTROL_MODE_SWITCH] = remove_nans(T125.seconds_since_start, T125.RADIO_CONTROL_MODE_SWITCH);

[tACTUATOR_STEERING_FEEDBACK_SETPOINT_US, ACTUATOR_STEERING_FEEDBACK_SETPOINT_US] = remove_nans(T125.seconds_since_start, T125.ACTUATOR_STEERING_FEEDBACK_SETPOINT_US);
[tACTUATOR_STEERING_FEEDBACK_POSITION_RAW, ACTUATOR_STEERING_FEEDBACK_POSITION_RAW] = remove_nans(T125.seconds_since_start, T125.ACTUATOR_STEERING_FEEDBACK_POSITION_RAW);

%% Plot the signals
tiledlayout("vertical")
nexttile
plot(tRADIO_CONTROL_MODE_SWITCH, RADIO_CONTROL_MODE_SWITCH)

nexttile
yyaxis left
plot(tACTUATOR_STEERING_FEEDBACK_SETPOINT_US, ACTUATOR_STEERING_FEEDBACK_SETPOINT_US, '-o')
yyaxis right
plot(tACTUATOR_STEERING_FEEDBACK_POSITION_RAW, ACTUATOR_STEERING_FEEDBACK_POSITION_RAW, '-o')

t = tACTUATOR_STEERING_FEEDBACK_POSITION_RAW;
u = ACTUATOR_STEERING_FEEDBACK_SETPOINT_US;
y = ACTUATOR_STEERING_FEEDBACK_POSITION_RAW;

%% Cut the fragments of data to identyfi

t_start = 51;
t_end = 53;


idx = (tACTUATOR_STEERING_FEEDBACK_POSITION_RAW > t_start) & ...
      (tACTUATOR_STEERING_FEEDBACK_POSITION_RAW < t_end);

t = tACTUATOR_STEERING_FEEDBACK_POSITION_RAW(idx);
u = ACTUATOR_STEERING_FEEDBACK_SETPOINT_US(idx);
y = ACTUATOR_STEERING_FEEDBACK_POSITION_RAW(idx);

ts =  mean(diff(t))
data = iddata(y, u, ts);   % build dataset, Ts from time vector

% figure
% plot(data); grid on;

% first-order system: 1 pole, 0 zeros
sys1 = tfest(data, 1, 0);

figure
compare(data, sys1); grid on; hold on;
plot(data); hold off;


%%
return
10
%% Multiple experiments

t_start_array = [48, 50, 53, 55, 57, 59, 61, 63, 65, 67, 70, 72];
t_end_array   = [51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 73, 75];
N = numel(t_start_array);
models = cell(N,1);

for i = 1:N
    models{i} = identify_first_order( ...
        t, u, y, ...
        t_start_array(i), ...
        t_end_array(i));
end

K = zeros(N,1);
T = zeros(N,1);
L = zeros(N,1);

K_ci = zeros(N,2);
T_ci = zeros(N,2);
L_ci = zeros(N,2);

for i = 1:N
    p = getpvec(models{i});          % parameters
    covp = getcov(models{i});        % covariance matrix of parameters

    % always present
    K(i) = p(1);
    T(i) = p(2);

    sigma = sqrt(diag(covp));        % std dev of each parameter

    K_ci(i,:) = p(1) + 1.96 * [-sigma(1), sigma(1)];
    T_ci(i,:) = p(2) + 1.96 * [-sigma(2), sigma(2)];

    % delay optional
    if numel(p) == 3
        L(i) = p(3);
        L_ci(i,:) = p(3) + 1.96 * [-sigma(3), sigma(3)];
    else
        L(i) = 0;
        L_ci(i,:) = [0 0];
    end
end
% --- Average values ---
K_avg = mean(K);
T_avg = mean(T);
L_avg = mean(L);

% --- Variance ---
K_var = var(K, 1);   % population variance
T_var = var(T, 1);
L_var = var(L, 1);

% --- CI of the mean (95%) ---
alpha = 0.05;
z = 1.96;     % for large-sample normal approx

K_mean_CI = K_avg + z * [-1 1] * sqrt(K_var / N);
T_mean_CI = T_avg + z * [-1 1] * sqrt(T_var / N);
L_mean_CI = L_avg + z * [-1 1] * sqrt(L_var / N);


function sys = identify_first_order(t, u, y, t_start, t_end)

    idx = (t > t_start) & (t < t_end);

    t_w = t(idx);
    u_w = u(idx);
    y_w = y(idx);

    Ts = mean(diff(t_w));
    data = iddata(y_w, u_w, Ts);

    sys = procest(data, 'P1D');   % [K, T, L]

    % visualize
    figure;
    compare(data, sys);
    title(sprintf('Identification window: %.2f–%.2f s', t_start, t_end));
end