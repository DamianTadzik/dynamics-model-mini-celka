%% Files
% parquetFile = "validation/log0.parquet";
parquetFile = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_10_17_home\logs_parquet\log1.parquet"
parquetFile = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_10_27_home\logs_parquet\log5.parquet"
matFile     = "validation/observer_controller_simulation_60s.mat";
matFile     = "validation/observer_controller_simulation_60s_MPC.mat";

%% Load Python runtime log
P = parquetread(parquetFile);
disp("Parquet columns:");
disp(P.Properties.VariableNames.');

%% Load MATLAB reference
D = load(matFile);
r = D.r;

%% MATLAB reference
t_mat = r.time(:);
t_mat = t_mat - t_mat(1);
% MATLAB observer outputs
v_mat     = r.velocity_mps(:);
z_mat     = r.xhat(:,1);
zdot_mat  = r.xhat(:,2);
phi_mat   = r.xhat(:,3);
theta_mat = r.xhat(:,4);
psi_mat   = r.xhat(:,5);
p_mat     = r.xhat(:,6);
q_mat     = r.xhat(:,7);
r_mat     = r.xhat(:,8);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Python observer from Parquet
t_python = P.timestamp_s;
v_python     = P.("control_cycle_estimated_state_velocity_mps");
z_python     = P.("control_cycle_estimated_state_z_m");
zdot_python  = P.("control_cycle_estimated_state_z_dot_mps");
phi_python   = P.("control_cycle_estimated_state_phi_rad");
theta_python = P.("control_cycle_estimated_state_theta_rad");
psi_python   = P.("control_cycle_estimated_state_psi_rad");
p_python     = P.("control_cycle_estimated_state_p_radps");
q_python     = P.("control_cycle_estimated_state_q_radps");
r_python     = P.("control_cycle_estimated_state_r_radps");

% Keep only valid observer samples
idx = ...
    ~isnan(t_python) & ...
    ~isnan(v_python) & ...
    ~isnan(z_python) & ...
    ~isnan(zdot_python) & ...
    ~isnan(phi_python) & ...
    ~isnan(theta_python) & ...
    ~isnan(psi_python) & ...
    ~isnan(p_python) & ...
    ~isnan(q_python) & ...
    ~isnan(r_python);

t_python     = t_python(idx);
v_python     = v_python(idx);
z_python     = z_python(idx);
zdot_python  = zdot_python(idx);
phi_python   = phi_python(idx);
theta_python = theta_python(idx);
psi_python   = psi_python(idx);
p_python     = p_python(idx);
q_python     = q_python(idx);
r_python     = r_python(idx);

t_python = t_python - t_python(1);
t_end = min(t_mat(end), t_python(end));


%% Figure 1 - Forward velocity
figure;
title("Forward velocity estimator comparison");
hold on;
grid on;
plot(t_mat, v_mat, 'LineWidth', 1.5);
plot(t_python, v_python, '--', 'LineWidth', 1.2);
xlabel("Time [s]");
ylabel("Velocity [m/s]");
legend("MATLAB", "Python", 'Location', 'best');
xlim([0 t_end]);


%% Figure 2 - Heave and heave velocity
figure;
sgtitle("Heave estimator comparison");
ax = [];

ax(end+1) = subplot(2,1,1);
hold on;
grid on;
plot(t_mat, z_mat, 'LineWidth', 1.5);
plot(t_python, z_python, '--', 'LineWidth', 1.2);
ylabel("z [m]");
title("Heave position");
legend("MATLAB", "Python", 'Location', 'best');
xlim([0 t_end]);

ax(end+1) = subplot(2,1,2);
hold on;
grid on;
plot(t_mat, zdot_mat, 'LineWidth', 1.5);
plot(t_python, zdot_python, '--', 'LineWidth', 1.2);
xlabel("Time [s]");
ylabel("dz/dt [m/s]");
title("Heave velocity");
xlim([0 t_end]);

linkaxes(ax, 'x');

%% Figure 3 - Body angular rates
figure;
sgtitle("Body angular-rate comparison");
ax = [];

ax(end+1) = subplot(3,1,1);
hold on;
grid on;
plot(t_mat, p_mat, 'LineWidth', 1.5);
plot(t_python, p_python, '--', 'LineWidth', 1.2);
ylabel("p [rad/s]");
title("Roll rate");
legend("MATLAB", "Python", 'Location', 'best');
xlim([0 t_end]);

ax(end+1) = subplot(3,1,2);
hold on;
grid on;
plot(t_mat, q_mat, 'LineWidth', 1.5);
plot(t_python, q_python, '--', 'LineWidth', 1.2);
ylabel("q [rad/s]");
title("Pitch rate");
xlim([0 t_end]);

ax(end+1) = subplot(3,1,3);
hold on;
grid on;
plot(t_mat, r_mat, 'LineWidth', 1.5);
plot(t_python, r_python, '--', 'LineWidth', 1.2);
xlabel("Time [s]");
ylabel("r [rad/s]");
title("Yaw rate");
xlim([0 t_end]);

linkaxes(ax, 'x');

%% Figure 4 - Attitude
figure;
sgtitle("Attitude comparison");
ax = [];

ax(end+1) = subplot(3,1,1);
hold on;
grid on;
plot(t_mat, phi_mat, 'LineWidth', 1.5);
plot(t_python, phi_python, '--', 'LineWidth', 1.2);
ylabel("phi [rad]");
title("Roll angle");
legend("MATLAB", "Python", 'Location', 'best');
xlim([0 t_end]);

ax(end+1) = subplot(3,1,2);
hold on;
grid on;
plot(t_mat, theta_mat, 'LineWidth', 1.5);
plot(t_python, theta_python, '--', 'LineWidth', 1.2);
ylabel("theta [rad]");
title("Pitch angle");
xlim([0 t_end]);

ax(end+1) = subplot(3,1,3);
hold on;
grid on;
plot(t_mat, psi_mat, 'LineWidth', 1.5);
plot(t_python, psi_python, '--', 'LineWidth', 1.2);
xlabel("Time [s]");
ylabel("psi [rad]");
title("Yaw angle");
xlim([0 t_end]);

linkaxes(ax, 'x');

%% Compare sample time diff(t)
dt_mat = diff(t_mat);
dt_python = diff(t_python);
figure;
hold on;
grid on;
plot(t_mat(2:end), dt_mat, ...
    'LineWidth', 1.2);
plot(t_python(2:end), dt_python, '--', ...
    'LineWidth', 1.2);
xlabel("Time [s]");
ylabel("\Delta t [s]");
title("Sampling interval comparison");
legend( ...
    "MATLAB", ...
    "Python", ...
    'Location', 'best' ...
);
xlim([0 min(t_mat(end), t_python(end))]);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Actuator estimator comparison

% MATLAB simulation reference
t_act_mat = r.time(:);
delta_mat = r.deltas;

delta_FL_mat = delta_mat(:,1);
delta_FR_mat = delta_mat(:,2);
delta_R_mat  = delta_mat(:,3);

t_act_mat = t_act_mat - t_act_mat(1);

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
t_act_end = min(t_act_mat(end), t_act_python(end));

% Plot estimated actuator states
figure;
sgtitle("Actuator estimator comparison");
ax = [];

ax(end+1) = subplot(3,1,1);
hold on;
grid on;
plot(t_act_mat, delta_FL_mat, 'LineWidth', 1.5);
plot(t_act_python, delta_FL_python, '--', 'LineWidth', 1.2);
ylabel("\delta_{FL} [deg]");
title("Front-left estimated actuator state");
legend("MATLAB", "Python", 'Location', 'best');
xlim([0 t_act_end]);

ax(end+1) = subplot(3,1,2);
hold on;
grid on;
plot(t_act_mat, delta_FR_mat, 'LineWidth', 1.5);
plot(t_act_python, delta_FR_python, '--', 'LineWidth', 1.2);
ylabel("\delta_{FR} [deg]");
title("Front-right estimated actuator state");
xlim([0 t_act_end]);

ax(end+1) = subplot(3,1,3);
hold on;
grid on;
plot(t_act_mat, delta_R_mat, 'LineWidth', 1.5);
plot(t_act_python, delta_R_python, '--', 'LineWidth', 1.2);
xlabel("Time [s]");
ylabel("\delta_R [deg]");
title("Rear estimated actuator state");
xlim([0 t_act_end]);

linkaxes(ax, 'x');

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Actuator estimator - actuator feedback comparison

% Hardware feedback from CAN
t_hw = P.can_timestamp_s;

raw_FL = P.("can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW");
raw_FR = P.("can_signals_ACTUATOR_RIGHT_FOIL_FEEDBACK_POSITION_RAW");
raw_R  = P.("can_signals_ACTUATOR_REAR_FOIL_FEEDBACK_POSITION_RAW");

% RAW -> [-6, +12] deg
hehe = 10;
delta_FL_hw = -6 + (2182+hehe - raw_FL) .* ((6+9.74) / (2182+hehe - 1670));
delta_FR_hw = -3.699 + (raw_FR - 1976+hehe) .* ((3.699+12) / (2516 - 1976+hehe));
delta_R_hw  = -6 + (2212 - raw_R) .* (18 / (2212 - 1770));

% Keep only actual CAN feedback samples
idx_FL = ~isnan(t_hw) & ~isnan(raw_FL);
idx_FR = ~isnan(t_hw) & ~isnan(raw_FR);
idx_R  = ~isnan(t_hw) & ~isnan(raw_R);

% Align hardware time with the beginning of the Python control log
t0 = min(P.timestamp_s(~isnan(P.("control_cycle_estimated_state_delta_FL_deg"))));

t_FL_hw = t_hw(idx_FL) - t0;
t_FR_hw = t_hw(idx_FR) - t0;
t_R_hw  = t_hw(idx_R)  - t0;

delta_FL_hw = delta_FL_hw(idx_FL);
delta_FR_hw = delta_FR_hw(idx_FR);
delta_R_hw  = delta_R_hw(idx_R);

% Add hardware feedback to the previous actuator-estimator figure
subplot(3,1,1);
plot(t_FL_hw, delta_FL_hw, ':', ...
    'LineWidth', 1.5, ...
    'Color', [0.9290 0.6940 0.1250]);
legend("MATLAB", "Python", "Hardware", 'Location', 'best');

subplot(3,1,2);
plot(t_FR_hw, delta_FR_hw, ':', ...
    'LineWidth', 1.5, ...
    'Color', [0.9290 0.6940 0.1250]);

subplot(3,1,3);
plot(t_R_hw, delta_R_hw, ':', ...
    'LineWidth', 1.5, ...
    'Color', [0.9290 0.6940 0.1250]);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controller comparison

%% MATLAB controller reference
t_ctrl_mat = r.time(:);
u_mat = r.control;

u_FL_mat = u_mat(:,1);
u_FR_mat = u_mat(:,2);
u_R_mat  = u_mat(:,3);

t_ctrl_mat = t_ctrl_mat - t_ctrl_mat(1);

%% Python controller from parquet
t_ctrl_python = P.timestamp_s;

u_FL_python = P.("control_cycle_outputs_front_left_setpoint");
u_FR_python = P.("control_cycle_outputs_front_right_setpoint");
u_R_python  = P.("control_cycle_outputs_rear_setpoint");

% Keep only samples where all controller outputs and timestamp are valid
idx = ...
    ~isnan(t_ctrl_python) & ...
    ~isnan(u_FL_python) & ...
    ~isnan(u_FR_python) & ...
    ~isnan(u_R_python);

t_ctrl_python = t_ctrl_python(idx);

u_FL_python = u_FL_python(idx);
u_FR_python = u_FR_python(idx);
u_R_python  = u_R_python(idx);

t_ctrl_python = t_ctrl_python - t_ctrl_python(1);

%% Plot controller outputs
figure;
sgtitle("Controller output comparison");
ax = [];

ax(end+1) = subplot(3,1,1);
hold on;
grid on;
plot(t_ctrl_mat, u_FL_mat, 'LineWidth', 1.5);
plot(t_ctrl_python, u_FL_python, '--', 'LineWidth', 1.2);
ylabel("FL [deg]");
title("Front-left actuator command");
legend("MATLAB", "Python", 'Location', 'best');
xlim([0 min(t_ctrl_mat(end), t_ctrl_python(end))]);

ax(end+1) = subplot(3,1,2);
hold on;
grid on;
plot(t_ctrl_mat, u_FR_mat, 'LineWidth', 1.5);
plot(t_ctrl_python, u_FR_python, '--', 'LineWidth', 1.2);
ylabel("FR [deg]");
title("Front-right actuator command");
xlim([0 min(t_ctrl_mat(end), t_ctrl_python(end))]);

ax(end+1) = subplot(3,1,3);
hold on;
grid on;
plot(t_ctrl_mat, u_R_mat, 'LineWidth', 1.5);
plot(t_ctrl_python, u_R_python, '--', 'LineWidth', 1.2);
xlabel("Time [s]");
ylabel("Rear [deg]");
title("Rear actuator command");
xlim([0 min(t_ctrl_mat(end), t_ctrl_python(end))]);

linkaxes(ax, 'x');

%% Controller timing comparison
dt_ctrl_mat = diff(t_ctrl_mat);
dt_ctrl_python = diff(t_ctrl_python);

figure;
title("Controller sampling interval comparison");
hold on;
grid on;
plot(t_ctrl_mat(2:end), dt_ctrl_mat, ...
    'LineWidth', 1.2);
plot(t_ctrl_python(2:end), dt_ctrl_python, '--', ...
    'LineWidth', 1.2);
xlabel("Time [s]");
ylabel("\Delta t [s]");
legend( ...
    "MATLAB", ...
    "Python", ...
    'Location', 'best' ...
);
xlim([0 min(t_ctrl_mat(end), t_ctrl_python(end))]);


%% SCOP file to RPI with 
% scp "D:\Dane\workspace\dynamics-model-mini-celka\4DOF-HEAVE-ROLL-PITCH-AUGUMENTED\validation\observer_controller_simulation_60s.mat" "brzanpi@192.168.10.2:/home/brzanpi/ws_minicelka/rpi-controller-mini-celka/src/observers"


%% PIL observer comparison figures

f1 = figure('Name','PIL_observer_xdot','Units','centimeters','Position',[2 2 8 4.5]);
hold on; grid on;
plot(t_mat, v_mat, 'LineWidth', 1.5);
plot(t_python, v_python, '--', 'LineWidth', 1.2);
xlabel('Time [s]','FontName','Times New Roman','FontSize',9);
ylabel('ẋ_W [m/s]','FontName','Times New Roman','FontSize',9);
legend('MATLAB','Python','Location','best','FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman','FontSize',9);
xlim([0 20]);

f2 = figure('Name','PIL_observer_z','Units','centimeters','Position',[2 2 8 4.5]);
hold on; grid on;
plot(t_mat, z_mat, 'LineWidth', 1.5);
plot(t_python, z_python, '--', 'LineWidth', 1.2);
xlabel('Time [s]','FontName','Times New Roman','FontSize',9);
ylabel('z_W [m]','FontName','Times New Roman','FontSize',9);
legend('MATLAB','Python','Location','best','FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman','FontSize',9);
xlim([0 20]);

f3 = figure('Name','PIL_observer_phi','Units','centimeters','Position',[2 2 8 4.5]);
hold on; grid on;
plot(t_mat, phi_mat, 'LineWidth', 1.5);
plot(t_python, phi_python, '--', 'LineWidth', 1.2);
xlabel('Time [s]','FontName','Times New Roman','FontSize',9);
ylabel('\phi [rad]','FontName','Times New Roman','FontSize',9);
legend('MATLAB','Python','Location','best','FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman','FontSize',9);
xlim([0 20]);

f4 = figure('Name','PIL_observer_p','Units','centimeters','Position',[2 2 8 4.5]);
hold on; grid on;
plot(t_mat, p_mat, 'LineWidth', 1.5);
plot(t_python, p_python, '--', 'LineWidth', 1.2);
xlabel('Time [s]','FontName','Times New Roman','FontSize',9);
ylabel('p [rad/s]','FontName','Times New Roman','FontSize',9);
legend('MATLAB','Python','Location','best','FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman','FontSize',9);
xlim([0 20]);

exportgraphics(f1, 'validation/PIL_observer_xdot.pdf', ...
    'ContentType', 'vector', 'BackgroundColor', 'none', ...
    'Units', 'centimeters', 'Width', 8, 'Height', 4.5);
exportgraphics(f2, 'validation/PIL_observer_z.pdf', ...
    'ContentType', 'vector', 'BackgroundColor', 'none', ...
    'Units', 'centimeters', 'Width', 8, 'Height', 4.5);
exportgraphics(f3, 'validation/PIL_observer_phi.pdf', ...
    'ContentType', 'vector', 'BackgroundColor', 'none', ...
    'Units', 'centimeters', 'Width', 8, 'Height', 4.5);
exportgraphics(f4, 'validation/PIL_observer_p.pdf', ...
    'ContentType', 'vector', 'BackgroundColor', 'none', ...
    'Units', 'centimeters', 'Width', 8, 'Height', 4.5);


%% Actuator comparison figures: MATLAB vs Python vs Hardware

ff1 = figure('Name','PIL_controller_actuators_FL','Units','centimeters','Position',[2 2 16 4.5]);
hold on; grid on;
plot(t_act_mat,    delta_FL_mat,    'LineWidth', .8);
plot(t_act_python, delta_FL_python, '--', 'LineWidth', .8);
plot(t_FL_hw,      delta_FL_hw,     ':', 'LineWidth', .4, 'Color', [0.9290 0.6940 0.1250 0.6]);
xlabel('Time [s]','FontName','Times New Roman','FontSize',9);
ylabel('\delta_{FL} [deg]','FontName','Times New Roman','FontSize',9);
legend('MATLAB','Python','Hardware', ...
    'Location','southwest', ...
    'FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);
xlim([20 60]);
ylim([0 5]);

ff2 = figure('Name','PIL_controller_actuators_FR','Units','centimeters','Position',[2 2 16 4.5]);
hold on; grid on;
plot(t_act_mat,    delta_FR_mat,    'LineWidth', .8);
plot(t_act_python, delta_FR_python, '--', 'LineWidth', .8);
plot(t_FR_hw,      delta_FR_hw,     ':', 'LineWidth', .4, 'Color', [0.9290 0.6940 0.1250 0.6]);
xlabel('Time [s]','FontName','Times New Roman','FontSize',9);
ylabel('\delta_{FR} [deg]','FontName','Times New Roman','FontSize',9);
legend('MATLAB','Python','Hardware', ...
    'Location','southwest', ...
    'FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);
xlim([20 60]);
ylim([.8 4.8]);

exportgraphics(ff1, 'validation/PIL_controller_actuators_FL.pdf', ...
    'ContentType', 'vector', 'BackgroundColor', 'none', ...
    'Units', 'centimeters', 'Width', 16, 'Height', 4.5);

exportgraphics(ff2, 'validation/PIL_controller_actuators_FR.pdf', ...
    'ContentType', 'vector', 'BackgroundColor', 'none', ...
    'Units', 'centimeters', 'Width', 16, 'Height', 4.5);



%% PIL real-time execution performance

% Control loop timing data
t_ns = P.("control_cycle_timestamp_monotonic_ns");
exec_ns = P.("control_cycle_execution_ns");
late_ns = P.("control_cycle_lateness_ns");

idx = ~isnan(t_ns) & ~isnan(exec_ns) & ~isnan(late_ns);

t_ns = double(t_ns(idx));
exec_ms = double(exec_ns(idx)) / 1e6;
late_ms = double(late_ns(idx)) / 1e6;

% Actual sampling interval
t = (t_ns - t_ns(1)) / 1e9;
dt_ms = diff(t_ns) / 1e6;
t_dt = t(2:end);

Ts_ms = 10;

% Statistics
fprintf('\n--- Control loop timing statistics ---\n');
fprintf('Mean sampling interval: %.4f ms\n', mean(dt_ms));
fprintf('Std sampling interval:  %.4f ms\n', std(dt_ms));
fprintf('99th percentile:        %.4f ms\n', prctile(dt_ms,99));
fprintf('Max sampling interval:  %.4f ms\n', max(dt_ms));
fprintf('Intervals > 10 ms:      %.2f %%\n', 100*mean(dt_ms > Ts_ms));
fprintf('Mean execution time:    %.4f ms\n', mean(exec_ms));
fprintf('Max execution time:     %.4f ms\n', max(exec_ms));
fprintf('Mean lateness:          %.4f ms\n', mean(late_ms));
fprintf('Max lateness:           %.4f ms\n', max(late_ms));

% Figure 1 - Sampling interval over time
fff1 = figure('Name','PIL_timing', ...
    'Units','centimeters','Position',[2 2 8 6]);

hold on; grid on;
plot(t_dt, dt_ms, 'LineWidth', 0.8);
yline(Ts_ms, '--', 'Nominal 10 ms', 'LineWidth', 1);

xlabel('Time [s]', ...
    'FontName','Times New Roman','FontSize',9);
ylabel('Sampling interval [ms]', ...
    'FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman','FontSize',9);
xlim([0 t_dt(end)]);

% Figure 2 - Sampling interval histogram
fff2 = figure('Name','PIL_timing_hist', ...
    'Units','centimeters','Position',[2 2 8 6]);

hold on; grid on;
histogram(dt_ms, 'BinWidth', 0.001, ...
    'Normalization','probability');
xline(Ts_ms, '--', 'Nominal 10 ms', 'LineWidth', 1);

xlabel('Sampling interval [ms]', ...
    'FontName','Times New Roman','FontSize',9);
ylabel('Probability', ...
    'FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman','FontSize',9);

% Export
exportgraphics(fff1, 'PIL_timing.pdf', ...
    'ContentType','vector', 'BackgroundColor','none', ...
    'Units','centimeters', 'Width',8, 'Height',6);
exportgraphics(fff2, 'PIL_timing_hist.pdf', ...
    'ContentType','vector', 'BackgroundColor','none', ...
    'Units','centimeters', 'Width',8, 'Height',6);
