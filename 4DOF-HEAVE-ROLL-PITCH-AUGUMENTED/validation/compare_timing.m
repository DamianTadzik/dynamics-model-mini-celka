clear
%% Load Python runtime log 2026_10_17_home 
parquetFile = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_10_28_divonnes\logs_parquet\log1.parquet"; % MPC / LQ
xlims = [327 350]; % MPC
% xlims = [110 187]; % ale byl wlaczony tylko tutaj

% parquetFile = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_09_25_divonnes\logs_parquet\log2.parquet"
P = parquetread(parquetFile);
disp("Parquet columns:");
disp(P.Properties.VariableNames.');

%% Extract control loop timing data
%% Extract control loop timing data
t_ns    = P.("control_cycle_timestamp_monotonic_ns");
exec_ns = P.("control_cycle_execution_ns");
late_ns = P.("control_cycle_lateness_ns");

idx = ~isnan(t_ns) & ~isnan(exec_ns) & ~isnan(late_ns);

% Time relative to beginning of entire log
t_all = (double(t_ns) - double(t_ns(find(idx, 1, 'first')))) / 1e9;

% Apply time window
idx = idx & t_all >= xlims(1) & t_all <= xlims(2);

t_ns    = double(t_ns(idx));
exec_ms = double(exec_ns(idx)) / 1e6;
late_ms = double(late_ns(idx)) / 1e6;

% Actual sampling interval
t = t_all(idx);
dt_ms = diff(t_ns) / 1e6;
t_dt = t(2:end);

Ts_ms = 10;

%% Find largest timing deviations

[dt_sorted, idx_sorted] = sort(dt_ms, 'descend');

fprintf('\n--- 10 largest sampling intervals ---\n');
fprintf('Time [s]    Interval [ms]\n');

for i = 1:min(10, numel(dt_sorted))
    fprintf('%8.3f    %10.4f\n', ...
        t_dt(idx_sorted(i)), dt_sorted(i));
end

fprintf('\nExecution time > 10 ms: %d\n', ...
    sum(exec_ms > Ts_ms));

fprintf('Estimated missed periods: %d\n', ...
    sum(max(0, round(dt_ms / Ts_ms) - 1)));


%% PIL real-time execution performance

% % Control loop timing data
% t_ns = P.("control_cycle_timestamp_monotonic_ns");
% exec_ns = P.("control_cycle_execution_ns");
% late_ns = P.("control_cycle_lateness_ns");
% 
% idx = ~isnan(t_ns) & ~isnan(exec_ns) & ~isnan(late_ns);
% 
% t_ns = double(t_ns(idx));
% exec_ms = double(exec_ns(idx)) / 1e6;
% late_ms = double(late_ns(idx)) / 1e6;
% 
% % Actual sampling interval
% t = (t_ns - t_ns(1)) / 1e9;
% dt_ms = diff(t_ns) / 1e6;
% t_dt = t(2:end);
% 
% Ts_ms = 10;

% Statistics
fprintf('\n--- Control loop timing statistics ---\n');
fprintf('Mean sampling interval: %.4f ms\n', mean(dt_ms));
fprintf('Std sampling interval:  %.4f ms\n', std(dt_ms));
fprintf('99th percentile:        %.4f ms\n', prctile(dt_ms,99));
fprintf('Max sampling interval:  %.4f ms\n', max(dt_ms));
fprintf('Intervals > 10 ms:      %.2f %%\n', 100*mean(dt_ms > Ts_ms));
fprintf('Mean execution time:    %.4f ms\n', mean(exec_ms));
fprintf('Std execution time:     %.4f ms\n', std(exec_ms));
fprintf('Max execution time:     %.4f ms\n', max(exec_ms));
fprintf('Mean lateness:          %.4f ms\n', mean(late_ms));
fprintf('Std lateness:           %.4f ms\n', std(late_ms));
fprintf('Max lateness:           %.4f ms\n', max(late_ms));

% Figure 1 - Sampling interval over time
fff1 = figure('Name','timing', ...
    'Units','centimeters','Position',[2 2 8 4.5]);

hold on; grid on;
plot(t_dt, dt_ms, 'LineWidth', 0.8);
yline(Ts_ms, '--', 'Nominal 10 ms', 'LineWidth', 1);

xlabel('Time [s]', ...
    'FontName','Times New Roman','FontSize',9);
ylabel('Sampling interval [ms]', ...
    'FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman','FontSize',9);
% xlim([0 t_dt(end)]);
xlim(xlims);

% Figure 2 - Sampling interval histogram
fff2 = figure('Name','timing_hist', ...
    'Units','centimeters','Position',[2 2 8 4.5]);

hold on; grid on;
histogram(dt_ms, 'BinWidth', 0.001, ...
    'Normalization','probability');
% xline(Ts_ms, '--', 'Nominal 10 ms', 'LineWidth', 1);

xlabel('Sampling interval [ms]', ...
    'FontName','Times New Roman','FontSize',9);
ylabel('Probability', ...
    'FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman','FontSize',9);
xlim([9.978 10.022])

% Export
% exportgraphics(fff1, 'validation/PIL_timing.pdf', ...
%     'ContentType','vector', 'BackgroundColor','none', ...
%     'Units','centimeters', 'Width',8, 'Height',4.5);
% exportgraphics(fff2, 'validation/PIL_timing_hist.pdf', ...
%     'ContentType','vector', 'BackgroundColor','none', ...
%     'Units','centimeters', 'Width',8, 'Height',4.5);

%% Control loop completion timing

% Completion time relative to scheduled cycle start
finish_ms = late_ms + exec_ms;

% Interval between consecutive cycle completions
finish_dt_ms = dt_ms + diff(exec_ms);

fprintf('\n--- Control loop completion statistics ---\n');
fprintf('Mean finish offset:   %.4f ms\n', mean(finish_ms));
fprintf('Std finish offset:    %.4f ms\n', std(finish_ms));
fprintf('Min finish offset:    %.4f ms\n', min(finish_ms));
fprintf('Max finish offset:    %.4f ms\n', max(finish_ms)); % 6.6806
fprintf('99th percentile:      %.4f ms\n', prctile(finish_ms,99));

fprintf('\n--- Completion-to-completion intervals ---\n');
fprintf('Mean interval:        %.4f ms\n', mean(finish_dt_ms));
fprintf('Std interval:         %.4f ms\n', std(finish_dt_ms));

fprintf('Max completion interval: %.4f ms\n', max(finish_dt_ms));
fprintf('99th completion interval: %.4f ms\n', prctile(finish_dt_ms,99));

%% Histogram
x = figure('Name','MPC_completion_hist', ...
    'Units','centimeters','Position',[2 2 8 7]);
% figure;
histogram(finish_ms, 'BinWidth', 0.2);
xlim([0 10])
grid on;
% xline(Ts_ms, '--', '10 ms deadline');
% xlabel('MPC completion time relative to scheduled tick [ms]','FontName','Times New Roman','FontSize',9);
xlabel('Completion time [ms]', 'FontName','Times New Roman', 'FontSize',9);
ylabel('Count [-]','FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman','FontSize',9);

% Adjust axes width
ax = gca;
ax.Units = 'centimeters';
pos = ax.Position;
pos(3) = pos(3) - 0.6;  % Reduce width
ax.Position = pos;

exportgraphics(ax, 'validation/eh/MPC_completion_hist.pdf', ...
    'ContentType','vector', 'BackgroundColor','none', ...
    'Units','centimeters', 'Width',8, 'Height',7, ...
    'Padding', 'figure');

%% Normality test - Jarque-Bera
[h, p] = jbtest(dt_ms, 0.05);

fprintf('Jarque-Bera test: p = %.4g\n', p);
fprintf('Normal distribution: %s\n', string(~h));

% Visual verification
figure;
qqplot(dt_ms);
grid on;
title('Sampling interval - Normal Q-Q plot');

%% Verify completion interval calculations

finish_t_ms = (t_ns - t_ns(1))/1e6 + exec_ms;

finish_dt_direct_ms = diff(finish_t_ms);
finish_dt_formula_ms = dt_ms + diff(exec_ms);

fprintf('Max calculation difference: %.8f ms\n', ...
    max(abs(finish_dt_direct_ms - finish_dt_formula_ms)));

% Correlation between start interval jitter and execution time changes
fprintf('Correlation (start jitter vs execution change): %.4f\n', ...
    corr(dt_ms - Ts_ms, diff(exec_ms)));

% Correlation between start lateness and execution time
fprintf('Correlation (lateness vs execution): %.4f\n', ...
    corr(late_ms, exec_ms));