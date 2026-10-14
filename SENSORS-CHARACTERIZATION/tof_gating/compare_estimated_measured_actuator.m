clc; clear;
%%
parquetFile = "log2.parquet";
P = parquetread(parquetFile);
disp("Parquet columns:");
disp(P.Properties.VariableNames.');

%% Actuator estimator comparison

% Hardware feedback from CAN
t_hw = P.can_timestamp_s;
raw_FL = P.("can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW");
raw_FR = P.("can_signals_ACTUATOR_RIGHT_FOIL_FEEDBACK_POSITION_RAW");
raw_R  = P.("can_signals_ACTUATOR_REAR_FOIL_FEEDBACK_POSITION_RAW");
% RAW -> [-6, +12] deg
delta_FL_hw = -6 + (2182 - raw_FL) .* ((6+9.74) / (2182 - 1670));
delta_FR_hw = -3.699 + (raw_FR - 1976) .* ((3.699+12) / (2516 - 1976));
delta_R_hw  = -6 + (2153 - raw_R) .* (18 / (2153 - 1606));
% Keep only actual CAN feedback samples
idx_FL = ~isnan(t_hw) & ~isnan(raw_FL);
idx_FR = ~isnan(t_hw) & ~isnan(raw_FR);
idx_R  = ~isnan(t_hw) & ~isnan(raw_R);
t0 = min(P.timestamp_s(~isnan(P.("control_cycle_estimated_state_delta_FL_deg"))));
t_FL_hw = t_hw(idx_FL) - t0;
t_FR_hw = t_hw(idx_FR) - t0;
t_R_hw  = t_hw(idx_R)  - t0;
delta_FL_hw = delta_FL_hw(idx_FL);
delta_FR_hw = delta_FR_hw(idx_FR);
delta_R_hw  = delta_R_hw(idx_R);


% Python actuator estimator from Parquet
t_act_python = P.timestamp_s;
delta_FL_python = P.("control_cycle_estimated_state_delta_FL_deg");
delta_FR_python = P.("control_cycle_estimated_state_delta_FR_deg");
delta_R_python  = P.("control_cycle_estimated_state_delta_R_deg");
% Keep only valid actuator-estimator samples
idx = ...
    ~isnan(t_act_python) & ...
    ~isnan(delta_FL_python) & ...
    ~isnan(delta_FR_python) & ...
    ~isnan(delta_R_python);
t_act_python = t_act_python(idx);
delta_FL_python = delta_FL_python(idx);
delta_FR_python = delta_FR_python(idx);
delta_R_python  = delta_R_python(idx);
t_act_python = t_act_python - t_act_python(1);
t_act_end = min(t_R_hw(end), t_act_python(end));

% Plot estimated actuator states
figure;
sgtitle("Actuator estimator comparison");
ax = [];

ax(end+1) = subplot(3,1,1);
hold on;
grid on;
plot(t_FL_hw, delta_FL_hw, 'LineWidth', 1.5);
plot(t_act_python, delta_FL_python, '--', 'LineWidth', 1.2);
ylabel("\delta_{FL} [deg]");
title("Front-left estimated actuator state");
legend("MATLAB", "Python", 'Location', 'best');
xlim([0 t_act_end]);

ax(end+1) = subplot(3,1,2);
hold on;
grid on;
plot(t_FR_hw, delta_FR_hw, 'LineWidth', 1.5);
plot(t_act_python, delta_FR_python, '--', 'LineWidth', 1.2);
ylabel("\delta_{FR} [deg]");
title("Front-right estimated actuator state");
xlim([0 t_act_end]);

ax(end+1) = subplot(3,1,3);
hold on;
grid on;
plot(t_R_hw, delta_R_hw, 'LineWidth', 1.5);
plot(t_act_python, delta_R_python, '--', 'LineWidth', 1.2);
xlabel("Time [s]");
ylabel("\delta_R [deg]");
title("Rear estimated actuator state");
xlim([0 t_act_end]);

linkaxes(ax, 'x');