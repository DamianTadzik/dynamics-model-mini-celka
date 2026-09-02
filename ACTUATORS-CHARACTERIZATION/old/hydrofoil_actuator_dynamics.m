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

[tRADIO, uRADIO] = remove_nans(T125.seconds_since_start, T125.RADIO_CONTROL_FRONT_PITCH);
[tRPI, uRPI] =  remove_nans(T125.seconds_since_start, T125.AUTO_CONTROL_FRONT_LEFT_SETPOINT);

fprintf('Timing of ACTUATOR_LEFT_FOIL_FB mean=%.4f std=%.4f [min max]=[%.4f %.4f]\n', ...
    mean(diff(tACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US)), std(diff(tACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US)), min(diff(tACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US)), max(diff(tACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US)));
fprintf('Timing of RADIO_CONTROL mean=%.4f std=%.4f [min max]=[%.4f %.4f]\n', ...
    mean(diff(tRADIO)), std(diff(tRADIO)), min(diff(tRADIO)), max(diff(tRADIO)));
fprintf('Timing of AUTO_CONTROL mean=%.4f std=%.4f [min max]=[%.4f %.4f]\n', ...
    mean(diff(tRPI)), std(diff(tRPI)), min(diff(tRPI)), max(diff(tRPI)));

tiledlayout("vertical")
nexttile
yyaxis left
plot(tRADIO_CONTROL_MODE_SWITCH, RADIO_CONTROL_MODE_SWITCH)
yyaxis right
plot(tRADIO, uRADIO)

nexttile
yyaxis left
plot(tACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US, ACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US, '-o')
yyaxis right
plot(tACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW, ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW, '-o')


%% Multiple experiments

t = tACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US;
u = ACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US;
y = ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW;

t_start_array = [36.2 37.04 37.95 38.8 39.8 40.6 41.5 42.4 43.35 44.15 45 46 46.8 55.2 56.55 58.2 59.6 61.1 62.5 64 65.3 67.3 67.9 68.65];
t_start_array([6, 15, 16]) = [];
% t_end_array   = t_start_array+1;
t_end_array   = t_start_array+.6;

%% continuus P1D
N = numel(t_start_array);
models = cell(N,1);
Lt = zeros(N,1);
models_p2d = cell(N,1);

for i = 1:N
    models{i} = identify_first_order( ...
        t, u, y, ...
        t_start_array(i), ...
        t_end_array(i), ...
        tRADIO, uRADIO, ...
        tRPI, uRPI);
    Lt(i) = estimate_step_delay( ...
        tRPI, uRPI, ...
        t, u, ...
        t_start_array(i), ...
        t_end_array(i));

    models_p2d{i} = identify_second_order( ...
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
Lt_avg = mean(Lt, "omitnan");

% --- Variance ---
K_var = var(K, 1);   % population variance
T_var = var(T, 1);
L_var = var(L, 1);
Lt_var = var(Lt, 1, "omitnan");
valid_Lt = Lt(~isnan(Lt));
N_Lt = numel(valid_Lt);

% Median
T_med = median(T, 1);
L_med = median(L, 1);
Lt_med = median(Lt, 1, "omitnan");

% --- CI of the mean (95%) ---
alpha = 0.05;
z = 1.96;     % for large-sample normal approx

K_mean_CI = K_avg + z * [-1 1] * sqrt(K_var / N);
T_mean_CI = T_avg + z * [-1 1] * sqrt(T_var / N);
L_mean_CI = L_avg + z * [-1 1] * sqrt(L_var / N);
Lt_mean_CI = Lt_avg + 1.96 * [-1 1] * sqrt(Lt_var / N_Lt);


function sys = identify_first_order(t, u, y, t_start, t_end, tRAD, uRAD, tRPI, uRPI)

    idx = (t > t_start) & (t < t_end);

    t_w = t(idx);
    u_w = u(idx);
    y_w = y(idx);

    tRAD_w = tRAD((tRAD>t_start) & (tRAD<t_end));
    uRAD_w = uRAD((tRAD>t_start) & (tRAD<t_end));

    tRPI_w = tRPI((tRPI>t_start) & (tRPI<t_end));
    uRPI_w = uRPI((tRPI>t_start) & (tRPI<t_end));
    uRPI_w = uRPI_w - mean(uRPI_w(1:4));

    u0 = mean(u_w(1:4));
    y0 = mean(y_w(1:4));

    u_w = u_w - u0;
    y_w = y_w - y0;

    Ts = mean(diff(t_w));
    data = iddata(y_w, u_w, Ts, ...
        'OutputName', 'y(t)');

    sys = procest(data, 'P1D');   % [K, T, L]

    % visualize
    figure;
    yyaxis left
    % compare(data, sys); hold on; grid on;
    plot(t_w, y_w, '-', 'DisplayName', 'Output response (y(t))'); hold on; grid on;
    y_sim = lsim(sys, data.InputData, data.SamplingInstants);
    plot(t_w(1) - Ts + data.SamplingInstants, y_sim, '--k', 'DisplayName', 'Simulated P1D response');

    yyaxis right    
    % plot(t_w-t_w(1), u_w,'DisplayName', 'u'); hold off;
    u_norm = (u_w - min(u_w)) ./ ...
         (max(u_w) - min(u_w));
    uRAD_norm = (uRAD_w - min(uRAD_w)) ./ ...
            (max(uRAD_w) - min(uRAD_w));
    uRPI_norm = (uRPI_w - min(uRPI_w)) ./ ...
            (max(uRPI_w) - min(uRPI_w));

    stairs(t_w, u_norm, '--ob', 'DisplayName', 'Control input (u(t))');
    % lol = 0.1 * (max(data.InputData) - min(data.InputData));
    % ylim([min(data.InputData)-lol, max(data.InputData)+lol])
    % stairs(tRAD_w, uRAD_norm, '--og', 'DisplayName', 'Radio data (uR(t))');
    stairs(tRPI_w, uRPI_norm, '--om', 'DisplayName', 'RPI control (u_{RPi}(t))');
    ylim([-.2 1.2])

    title(sprintf('Identification window: %.2f–%.2f s', t_start, t_end));
    legend('Location', 'best');
end
function delay = estimate_step_delay(t_in, u_in, t_out, u_out, t_start, t_end)

    idx_in  = (t_in  >= t_start) & (t_in  <= t_end);
    idx_out = (t_out >= t_start) & (t_out <= t_end);

    ti = t_in(idx_in);
    ui = u_in(idx_in);

    to = t_out(idx_out);
    uo = u_out(idx_out);

    if numel(ti) < 3 || numel(to) < 3
        delay = NaN;
        warning("Not enough samples in window %.2f–%.2f s", t_start, t_end);
        return;
    end

    % Detect strongest input step
    dui = diff(ui);
    [dui_max, k_in] = max(abs(dui));

    if dui_max < eps
        delay = NaN;
        warning("No input step detected in window %.2f–%.2f s", t_start, t_end);
        return;
    end

    % Step occurs at the timestamp of the new sample
    t_step_in = ti(k_in + 1);
    du_sign = sign(dui(k_in));

    % Look only for output steps AFTER input step
    idx_after = find(to >= t_step_in);

    if isempty(idx_after) || numel(idx_after) < 2
        delay = NaN;
        warning("No output samples after input step in window %.2f–%.2f s", t_start, t_end);
        return;
    end

    to_after = to(idx_after);
    uo_after = uo(idx_after);

    duo = diff(uo_after);

    % Prefer output step with the same direction
    if du_sign > 0
        candidate_idx = find(duo > 0);
    else
        candidate_idx = find(duo < 0);
    end

    if isempty(candidate_idx)
        delay = NaN;
        warning("No matching output step after input step in window %.2f–%.2f s", t_start, t_end);
        return;
    end

    % Take the first matching output step after input step
    k_out_local = candidate_idx(1);
    t_step_out = to_after(k_out_local + 1);

    delay = t_step_out - t_step_in;

    if delay < 0
        error("Internal logic error: negative delay after causal filtering.");
    end
end
% function delay = estimate_step_delay(t_in, u_in, t_out, u_out, t_start, t_end)
% 
%     idx_in  = (t_in  > t_start) & (t_in  < t_end);
%     idx_out = (t_out > t_start) & (t_out < t_end);
% 
%     ti = t_in(idx_in);
%     ui = u_in(idx_in);
% 
%     to = t_out(idx_out);
%     uo = u_out(idx_out);
% 
%     if numel(ti) < 2 || numel(to) < 2
%         delay = NaN;
%         warning("Not enough samples in window %.2f–%.2f s", t_start, t_end);
%         return;
%     end
% 
%     % Remove local offset
%     ui0 = mean(ui(1:min(4,end)));
%     uo0 = mean(uo(1:min(4,end)));
% 
%     ui = ui - ui0;
%     uo = uo - uo0;
% 
%     % Estimate step amplitude and direction from input
%     du_in = ui(end) - ui(1);
%     du_out = uo(end) - uo(1);
% 
%     if abs(du_in) < eps || abs(du_out) < eps
%         delay = NaN;
%         warning("No clear step in window %.2f–%.2f s", t_start, t_end);
%         return;
%     end
% 
%     sgn = sign(du_in);
% 
%     % 50% crossing threshold
%     ui_thr = ui(1) + 0.5 * du_in;
%     uo_thr = uo(1) + 0.5 * du_out;
% 
%     if sgn > 0
%         k_in  = find(ui >= ui_thr, 1, 'first');
%         k_out = find(uo >= uo_thr, 1, 'first');
%     else
%         k_in  = find(ui <= ui_thr, 1, 'first');
%         k_out = find(uo <= uo_thr, 1, 'first');
%     end
% 
%     if isempty(k_in) || isempty(k_out)
%         delay = NaN;
%         warning("Step crossing not found in window %.2f–%.2f s", t_start, t_end);
%         return;
%     end
% 
%     delay = to(k_out) - ti(k_in);
% 
%     % Optional sanity check
%     if delay < 0
%         warning("Negative delay detected in window %.2f–%.2f s", t_start, t_end);
%     end
% end

fprintf('\n===== CONTINUOUS-TIME IDENTIFICATION (P1D) =====\n');
fprintf('Number of experiments: %d\n\n', N);

fprintf('T per experiment [s]:\n');
fprintf('  %8.4f\n', T);

fprintf('\nAverage T: %.4f s\n', T_avg);
fprintf('Median T: %.4f s\n', T_med);
fprintf('95%% CI of mean T: [%.4f, %.4f] s\n', ...
        T_mean_CI(1), T_mean_CI(2));
fprintf('Variance of T: %.6f s^2\n', T_var);

fprintf('\nAverage L: %.4f s\n', L_avg);
fprintf('Median L: %.4f s\n', L_med);
fprintf('95%% CI of mean L: [%.4f, %.4f] s\n', ...
        L_mean_CI(1), L_mean_CI(2));
fprintf('Variance of L: %.6f s^2\n', L_var);

fprintf('\nAverage Lt: %.4f s\n', Lt_avg);
fprintf('Median Lt: %.4f s\n', Lt_med);
fprintf('95%% CI of mean Lt: [%.4f, %.4f] s\n', ...
        Lt_mean_CI(1), Lt_mean_CI(2));
fprintf('Variance of Lt: %.6f s^2\n', Lt_var);

LLttotal = L + Lt;
TLtotal = T + L; 
TLLttotal = T + L + Lt;

fprintf('\nAverage Ltotal = L + Lt: %.4f s\n', mean(LLttotal, "omitnan"));
fprintf('Average TLtotal = T + L: %.4f s\n', mean(TLtotal, "omitnan"));
fprintf('Average TLLttotal = T + L + Lt: %.4f s\n', mean(TLLttotal, "omitnan"));

fprintf('\nMedian Ltotal = L + Lt: %.4f s\n', median(LLttotal, "omitnan"));
fprintf('Median TLtotal = T + L: %.4f s\n', median(TLtotal, "omitnan"));
fprintf('Median TLLttotal = T + L + Lt: %.4f s\n', median(TLLttotal, "omitnan"));

% return
results = table((1:N)', t_start_array', T, L, Lt, ...
    'VariableNames', {'i','t_start','T','L','Lt'});
disp(results)

%% Histograms or different charts lol

figure;
hold on; grid on;

jitter_T = 0.06 * randn(size(T));
jitter_L = 0.06 * randn(size(L));
jitter_Lt = 0.06 * randn(size(Lt));

jitter_LLttotal = 0.06 * randn(size(LLttotal));
jitter_TLtotal = 0.06 * randn(size(TLtotal));
jitter_TLLttotal = 0.06 * randn(size(TLLttotal));


plot(1 + jitter_T, T, 'o', 'MarkerSize', 6, 'LineWidth', 1.1);
plot(2 + jitter_L, L, 'o', 'MarkerSize', 6, 'LineWidth', 1.1);
plot(3 + jitter_Lt, Lt, 'o', 'MarkerSize', 6, 'LineWidth', 1.1);

plot(4 + jitter_LLttotal, LLttotal, 'o', 'MarkerSize', 6, 'LineWidth', 1.1);
plot(5 + jitter_TLtotal, TLtotal, 'o', 'MarkerSize', 6, 'LineWidth', 1.1);
plot(6 + jitter_TLLttotal, TLLttotal, 'o', 'MarkerSize', 6, 'LineWidth', 1.1);

errorbar(1, T_avg, ...
    T_avg - T_mean_CI(1), T_mean_CI(2) - T_avg, ...
    'k', 'LineWidth', 1.5, 'CapSize', 12);
errorbar(2, L_avg, ...
    L_avg - L_mean_CI(1), L_mean_CI(2) - L_avg, ...
    'k', 'LineWidth', 1.5, 'CapSize', 12);
errorbar(3, Lt_avg, ...
    Lt_avg - Lt_mean_CI(1), Lt_mean_CI(2) - Lt_avg, ...
    'k', 'LineWidth', 1.5, 'CapSize', 12);

% errorbar(4, Lt_avg, ...
%     Lt_avg - Lt_mean_CI(1), Lt_mean_CI(2) - Lt_avg, ...
%     'k', 'LineWidth', 1.5, 'CapSize', 12);
% errorbar(5, Lt_avg, ...
%     Lt_avg - Lt_mean_CI(1), Lt_mean_CI(2) - Lt_avg, ...
%     'k', 'LineWidth', 1.5, 'CapSize', 12);
% errorbar(6, Lt_avg, ...
%     Lt_avg - Lt_mean_CI(1), Lt_mean_CI(2) - Lt_avg, ...
%     'k', 'LineWidth', 1.5, 'CapSize', 12);

xlim([0.5 6.5])
xticks([1 2 3 4 5 6])
xticklabels({'T', 'L', 'L_t', 'LLttotal', 'TLtotal', 'TLLttotal'})
ylabel('Parameter value [s]')
title('Identified actuator model parameters')
legend('Individual estimates', 'Mean with 95% CI', 'Location', 'best')

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

%% P2D analysis
K2  = zeros(N,1);
T21 = zeros(N,1);
T22 = zeros(N,1);
L2  = zeros(N,1);

for i = 1:N
    p2 = getpvec(models_p2d{i});

    K2(i)  = p2(1);
    T21(i) = p2(2);
    T22(i) = p2(3);

    if numel(p2) >= 4
        L2(i) = p2(4);
    else
        L2(i) = 0;
    end
end

results_p2d = table((1:N)', t_start_array', K2, T21, T22, L2, Lt, ...
    'VariableNames', {'i','t_start','K2','T21','T22','L2','Lt'});
disp(results_p2d)


%% Save the actuator identification data as .mat file 
save("hydrofoil_actuator_dynamics.mat", "L_avg", "T_avg");
return


%% COMPARE P1D and P2D

i = 1;

idx = (t > t_start_array(i)) & (t < t_end_array(i));
t_w = t(idx);
u_w = u(idx);
y_w = y(idx);

u_w = u_w - mean(u_w(1:4));
y_w = y_w - mean(y_w(1:4));

Ts = mean(diff(t_w));
data = iddata(y_w, u_w, Ts);

figure;
compare(data, models{i}, models_p2d{i});
legend('Measured', 'P1D', 'P2D');
grid on;

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


function sys = identify_second_order(t, u, y, t_start, t_end)

    idx = (t > t_start) & (t < t_end);

    t_w = t(idx);
    u_w = u(idx);
    y_w = y(idx);

    u0 = mean(u_w(1:4));
    y0 = mean(y_w(1:4));

    u_w = u_w - u0;
    y_w = y_w - y0;

    Ts = mean(diff(t_w));

    data = iddata(y_w, u_w, Ts, ...
        'InputName', 'u(t)', ...
        'OutputName', 'y(t)');

    sys = procest(data, 'P2D');   % [K, Tp1, Tp2, L]

end