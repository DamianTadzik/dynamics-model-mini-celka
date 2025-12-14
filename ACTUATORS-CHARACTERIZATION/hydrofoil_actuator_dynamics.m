clc; clear; close all;
%% Load file
parquet_file_125 = 'data/FAST_LOG000.PARQUET';
info = parquetinfo(parquet_file_125);
% Actually load file
T125 = parquetread(parquet_file_125);
for i = 1:length(T125.Properties.VariableDescriptions)
    disp(T125.Properties.VariableDescriptions{i});
end

% Load the signals
[tRADIO_CONTROL_MODE_SWITCH, RADIO_CONTROL_MODE_SWITCH] = remove_nans(T125.seconds_since_start, T125.RADIO_CONTROL_MODE_SWITCH);

[tACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US, ACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US] = remove_nans(T125.seconds_since_start, T125.ACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US);
[tACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW, ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW] = remove_nans(T125.seconds_since_start, T125.ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW);

tiledlayout("vertical")
nexttile
plot(tRADIO_CONTROL_MODE_SWITCH, RADIO_CONTROL_MODE_SWITCH)

nexttile
yyaxis left
plot(tACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US, ACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US, '-o')
yyaxis right
plot(tACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW, ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW, '-o')


%% Multiple experiments

t = tACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US;
u = ACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US;
y = ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW;

t_start_array = [36 37 38 38.8 39.6 40.6 41.5 42.4 43.2 44.2 45 46 46.8 55 56.5 58 59.5 61 62.5 64 65 67 67.6 68.45];
t_end_array   = t_start_array+1;

%% continuus P1D
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

    u0 = mean(u_w(1:4));
    y0 = mean(y_w(1:4));

    u_w = u_w - u0;
    y_w = y_w - y0;

    Ts = mean(diff(t_w));
    data = iddata(y_w, u_w, Ts);

    sys = procest(data, 'P1D');   % [K, T, L]

    % visualize
    figure;
    compare(data, sys);
    title(sprintf('Identification window: %.2f–%.2f s', t_start, t_end));
end

fprintf('\n===== CONTINUOUS-TIME IDENTIFICATION (P1D) =====\n');
fprintf('Number of experiments: %d\n\n', N);

fprintf('T per experiment [s]:\n');
fprintf('  %8.4f\n', T);

fprintf('\nAverage T: %.4f s\n', T_avg);
fprintf('95%% CI of mean T: [%.4f, %.4f] s\n', ...
        T_mean_CI(1), T_mean_CI(2));
fprintf('Variance of T: %.6f s^2\n', T_var);

% return

%%  Simulate the average P1D model and compare to real on one plot

t;
u;
y;
t_start = 60;
t_end = 110;

% --- extract window ---
idx = (t >= t_start) & (t <= t_end);
t_w = t(idx);
u_w = u(idx);
y_w = y(idx);

Ts = mean(diff(t_w));

% --- offset removal (pre-step) ---
Npre = min(10, floor(numel(t_w)/4));
u0 = mean(u_w(1:Npre));
y0 = mean(y_w(1:Npre));

u_w = u_w - u0;
y_w = y_w - y0;

% --- build average P1D model ---
sys_avg = tf(K_avg, [T_avg 1], 'InputDelay', L_avg);
% sys_avg = tf(K_avg, [T_avg 1]); % 'InputDelay', L_avg);

% --- simulate ---
t_sim = (0:numel(u_w)-1).' * Ts;
y_sim = lsim(sys_avg, u_w, t_sim);

% y_sim = lsim(sys_avg, u_w, t_w);

% --- plot ---
figure;
plot(t_w, y_w, 'k', 'LineWidth', 1.2); hold on;
plot(t_w, y_sim, 'r--', 'LineWidth', 1.5);
grid on;

xlabel('Time [s]');
ylabel('Output (offset removed)');
title('Average P1D model vs real data');
legend('Measured', 'Simulated (avg P1D)');


%% Save the actuator identification data as .mat file 
save("hydrofoil_actuator_dynamics.mat", "L_avg", "T_avg");
return

%% NOTE TO MYSELF, the L could be decreased, by half if the servo task would run faster, it introduces 10ms of delay probbably



%% Rate-limited + P1 (nonlinear) identification
N = numel(t_start_array);

K_rl    = zeros(N,1);
T_rl    = zeros(N,1);
vmax_rl = zeros(N,1);

for i = 1:N
    [K_rl(i), T_rl(i), vmax_rl(i)] = identify_rate_limit_p1( ...
        t, u, y, ...
        t_start_array(i), ...
        t_end_array(i));
end

% remove NaNs (failed windows)
ok = isfinite(T_rl) & isfinite(K_rl) & isfinite(vmax_rl);
K_rl = K_rl(ok); T_rl = T_rl(ok); vmax_rl = vmax_rl(ok);
Nok = numel(T_rl);

% stats
z = 1.96;

T_rl_avg = mean(T_rl);
T_rl_var = var(T_rl, 1);
T_rl_CI  = T_rl_avg + z * [-1 1] * sqrt(T_rl_var / Nok);

vmax_avg = mean(vmax_rl);


function [K_est, T_est, vmax] = identify_rate_limit_p1(t,u,y,t_start,t_end)

end

fprintf('\n===== RATE-LIMIT + P1 IDENTIFICATION =====\n');
fprintf('Used experiments: %d / %d\n\n', Nok, N);

fprintf('T per experiment [s]:\n');
fprintf('  %8.6f\n', T_rl);

fprintf('\nAverage T: %.6f s\n', T_rl_avg);
fprintf('95%% CI of mean T: [%.6f, %.6f] s\n', T_rl_CI(1), T_rl_CI(2));
fprintf('Variance of T: %.9f s^2\n', T_rl_var);

fprintf('\nRate limit vmax per experiment [units/s]:\n');
fprintf('  %8.3f\n', vmax_rl);
fprintf('\nAverage vmax: %.3f units/s\n', vmax_avg);
return
%% Discrete (low-sampling robust)

N = numel(t_start_array);

K_d  = zeros(N,1);
T_d  = zeros(N,1);
L_d  = zeros(N,1);

a_d  = zeros(N,1);
b_d  = zeros(N,1);
nk_d = zeros(N,1);

for i = 1:N
    [K_d(i), T_d(i), L_d(i), a_d(i), b_d(i), nk_d(i)] = ...
        identify_fo_discrete_step( ...
            t, u, y, ...
            t_start_array(i), ...
            t_end_array(i));
end

% --- Average values ---
K_d_avg = mean(K_d);
T_d_avg = mean(T_d);
L_d_avg = mean(L_d);

% --- Variance ---
K_d_var = var(K_d, 1);
T_d_var = var(T_d, 1);
L_d_var = var(L_d, 1);

% --- CI of the mean (95%) ---
z = 1.96;

K_d_mean_CI = K_d_avg + z * [-1 1] * sqrt(K_d_var / N);
T_d_mean_CI = T_d_avg + z * [-1 1] * sqrt(T_d_var / N);
L_d_mean_CI = L_d_avg + z * [-1 1] * sqrt(L_d_var / N);


function [K,T,L,a,b,nk] = identify_fo_discrete_step(t,u,y,t_start,t_end)

idx = (t >= t_start) & (t <= t_end);
t_w = t(idx); u_w = u(idx); y_w = y(idx);
Ts  = mean(diff(t_w));

% ---- baseline + step size (robust averages) ----
Npre = max(3, round(0.2/ Ts));              % ~0.2s pre, adjust if needed
Npre = min(Npre, floor(numel(y_w)/4));

y0 = mean(y_w(1:Npre));
u0 = mean(u_w(1:Npre));

% crude steady state from last chunk
Npost = max(3, round(0.2/ Ts));
Npost = min(Npost, floor(numel(y_w)/4));

yss = mean(y_w(end-Npost+1:end));
uss = mean(u_w(end-Npost+1:end));

du = uss - u0;
if abs(du) < 1e-9
    error("No step in u detected in this window.");
end

% ---- estimate integer delay nk (samples) ----
noise = std(y_w(1:Npre) - y0);
th = max(3*noise, 1.0);   % '1.0' = minimum threshold in raw units; tune

k0 = find(abs(y_w - y0) > th, 1, 'first');
if isempty(k0), nk = 0; else, nk = max(0, k0-1); end
L = nk * Ts;

% ---- build regression y[k] = a y[k-1] + b u[k-1-nk] ----
k = (2+nk):numel(y_w);
Y  = y_w(k);
Phi = [y_w(k-1), u_w(k-1-nk)];

theta = Phi \ Y;
a = theta(1); b = theta(2);

% sanity clamp (optional)
a = min(max(a, 1e-6), 0.999999);

T = -Ts / log(a);
K = b / (1 - a);
end