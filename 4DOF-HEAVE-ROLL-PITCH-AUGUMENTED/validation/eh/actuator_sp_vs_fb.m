clear; clc;
%% Files
parquetFile = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_10_28_divonnes\logs_parquet\log0.parquet" % LQ
xlims = [336 364];

% parquetFile = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_10_28_divonnes\logs_parquet\log1.parquet" % MPC / LQ
% parquetFile = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_10_28_divonnes\logs_parquet\log3.parquet" % MPCi / LQ

% 520 i 490 z log0 to dobre porownanie xd 

% parquetFile = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_10_18_divonnes\logs_parquet\log0.parquet"
% xlims = [485 535]; % oba
% xlims = [485 505]; % gorszy
% xlims = [515 535]; % lepszy

P = parquetread(parquetFile);
disp("Parquet columns:");
disp(P.Properties.VariableNames.');
xlims = [P.timestamp_s(1) P.timestamp_s(end)] - P.timestamp_s(1);

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

t_python = t_python - t_python(1);

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



%% Python actuator estimator from Parquet
delta_FL_python = P.("control_cycle_estimated_state_delta_FL_deg");
delta_FR_python = P.("control_cycle_estimated_state_delta_FR_deg");
delta_R_python  = P.("control_cycle_estimated_state_delta_R_deg");

% Keep only valid actuator-estimator samples
idx = ...
    ~isnan(delta_FL_python) & ...
    ~isnan(delta_FR_python) & ...
    ~isnan(delta_R_python);

delta_FL_python = delta_FL_python(idx);
delta_FR_python = delta_FR_python(idx);
delta_R_python  = delta_R_python(idx);

% Plot estimated actuator states
figure;
sgtitle("Actuator estimator comparison");
ax = [];

ax(end+1) = subplot(2,3,1);
hold on;
grid on;
% plot(t_act_mat, delta_FL_mat, 'LineWidth', 1.5);
plot(t_python, delta_FL_python, '--', 'LineWidth', 1.2);
ylabel("\delta_{FL} [deg]");
title("Front-left estimated actuator state");
% legend("", "Python", 'Location', 'best');
% legend("Python", 'Location', 'best');


ax(end+1) = subplot(2,3,2);
hold on;
grid on;
% plot(t_act_mat, delta_FR_mat, 'LineWidth', 1.5);
plot(t_python, delta_FR_python, '--', 'LineWidth', 1.2);
ylabel("\delta_{FR} [deg]");
title("Front-right estimated actuator state");

ax(end+1) = subplot(2,3,3);
hold on;
grid on;
% plot(t_act_mat, delta_R_mat, 'LineWidth', 1.5);
plot(t_python, delta_R_python, '--', 'LineWidth', 1.2);
xlabel("Time [s]");
ylabel("\delta_R [deg]");
title("Rear estimated actuator state");

linkaxes(ax, 'x');

%% Hardware feedback from CAN
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
% t0 = min(P.timestamp_s(~isnan(P.("control_cycle_estimated_state_delta_FL_deg"))));

t_FL_hw = t_hw(idx_FL) - t_hw(1);
t_FR_hw = t_hw(idx_FR) - t_hw(1);
t_R_hw  = t_hw(idx_R)  - t_hw(1);

delta_FL_hw = delta_FL_hw(idx_FL);
delta_FR_hw = delta_FR_hw(idx_FR);
delta_R_hw  = delta_R_hw(idx_R);

% Add hardware feedback to the previous actuator-estimator figure
subplot(ax(1));
plot(t_FL_hw, delta_FL_hw, ':', ...
    'LineWidth', 1.5, ...
    'Color', [0.9290 0.6940 0.1250]);
legend("Estimated", "Feedback", 'Location', 'best');

subplot(ax(2));
plot(t_FR_hw, delta_FR_hw, ':', ...
    'LineWidth', 1.5, ...
    'Color', [0.9290 0.6940 0.1250]);

subplot(ax(3));
plot(t_R_hw, delta_R_hw, ':', ...
    'LineWidth', 1.5, ...
    'Color', [0.9290 0.6940 0.1250]);

%% put currents on bottom plots

t_hw = P.can_timestamp_s;
current_FL = P.("can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_CURRENT");
current_FR = P.("can_signals_ACTUATOR_RIGHT_FOIL_FEEDBACK_CURRENT");
current_R  = P.("can_signals_ACTUATOR_REAR_FOIL_FEEDBACK_CURRENT");

voltage_FL = P.("can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_VOLTAGE");
voltage_FR = P.("can_signals_ACTUATOR_RIGHT_FOIL_FEEDBACK_VOLTAGE");
voltage_R  = P.("can_signals_ACTUATOR_REAR_FOIL_FEEDBACK_VOLTAGE");

% Keep only actual CAN samples
idx_FL = ~isnan(t_hw) & ~isnan(current_FL);
idx_FR = ~isnan(t_hw) & ~isnan(current_FR);
idx_R  = ~isnan(t_hw) & ~isnan(current_R);

current_FL = current_FL(idx_FL);
current_FR = current_FR(idx_FR);
current_R  = current_R(idx_R);
% voltage is in the same message hence the same timestamp applies
voltage_FL = voltage_FL(idx_FL);
voltage_FR = voltage_FR(idx_FR);
voltage_R  = voltage_R(idx_R);

% Align hardware time with the beginning of the Python control log
% t0 = min(P.timestamp_s(~isnan(P.("control_cycle_estimated_state_delta_FL_deg"))));
t_FL__current_hw = t_hw(idx_FL) - t_hw(1);
t_FR__current_hw = t_hw(idx_FR) - t_hw(1);
t_R__current_hw  = t_hw(idx_R)  - t_hw(1);

ax(end+1) = subplot(2,3,4);
yyaxis left
plot(t_FL__current_hw, current_FL); grid on;
ylabel('Current [A]')
yyaxis right
plot(t_FL__current_hw, voltage_FL)

ax(end+1) = subplot(2,3,5);
yyaxis left
plot(t_FR__current_hw, current_FR); grid on;
yyaxis right
plot(t_FR__current_hw, voltage_FR)

ax(end+1) = subplot(2,3,6);
yyaxis left
plot(t_R__current_hw, current_R); grid on;
yyaxis right
plot(t_R__current_hw, voltage_R)
ylabel('Voltage [V]')

linkaxes(ax, 'x');
xlim(xlims);

%% PLot velocity and control input in separate window

figure;
axvel = [];

% VELOCITY and its ESTIMATE
t_hw = P.timestamp_s;
v_gps = P.("can_signals_GPS_MOTION_GroundSpeed");
idx = ~isnan(t_hw) & ~isnan(v_gps);
v_gps = v_gps(idx);
t_hw = t_hw(idx) - t_hw(1);

axvel(end+1) = subplot(2,3,1);
plot(t_python, v_python); grid on; hold on;
plot(t_hw, v_gps);
legend("Estimated", "raw GPS", 'Location', 'best');


% HEAVE estimate
axvel(end+1) = subplot(2,3,2);
plot(t_python, z_python); grid on; hold on;
% plot(t_hw, v_gps);
legend("Estimated", "raw sns", 'Location', 'best');
ylim([-.25 .05]);
xlim(xlims);

% THROTTLE RPMS
t_hw = P.can_msg_timestamp_s;
thr = P.("can_signals_RADIO_CONTROL_THROTTLE");
rpm_est = P.("can_signals_ODRIVE_GET_SENSORLESS_ESTIMATES_Sensorless_Vel_Estimate");

idx_thr = ~isnan(t_hw) & ~isnan(thr);
idx_rpm_est = ~isnan(t_hw) & ~isnan(rpm_est);

thr = thr(idx_thr);
rpm_est = rpm_est(idx_rpm_est);

t_hw_thr = t_hw(idx_thr) - t_hw(1);
t_hw_rpm = t_hw(idx_rpm_est) - t_hw(1);

axvel(end+1) = subplot(2,3,4);
yyaxis left
plot(t_hw_thr, thr); grid on; hold on;
yyaxis right
plot(t_hw_rpm, rpm_est);


% RAW TOFS
t_hw = P.can_msg_timestamp_s;
distance_FL = P.("can_signals_DISTANCE_FORE_FEEDBACK_RANGE_MM_L");
distance_FR = P.("can_signals_DISTANCE_FORE_FEEDBACK_RANGE_MM_R");
distance_AL = P.("can_signals_DISTANCE_ACHTER_FEEDBACK_RANGE_MM_L");
distance_AR = P.("can_signals_DISTANCE_ACHTER_FEEDBACK_RANGE_MM_R");
idx_FL = ~isnan(distance_FL) & ~isnan(t_hw) & (P.("can_signals_DISTANCE_FORE_FEEDBACK_ERROR_STATUS_L") == 0);
idx_FR = ~isnan(distance_FR) & ~isnan(t_hw) & (P.("can_signals_DISTANCE_FORE_FEEDBACK_ERROR_STATUS_R") == 0);
idx_AL = ~isnan(distance_AL) & ~isnan(t_hw) & (P.("can_signals_DISTANCE_ACHTER_FEEDBACK_ERROR_STATUS_L") == 0);
idx_AR = ~isnan(distance_AR) & ~isnan(t_hw) & (P.("can_signals_DISTANCE_ACHTER_FEEDBACK_ERROR_STATUS_R") == 0);
t_hw = t_hw - t_hw(1);
distance_FL = distance_FL(idx_FL);
distance_FR = distance_FR(idx_FR);
distance_AL = distance_AL(idx_AL);
distance_AR = distance_AR(idx_AR);

axvel(end+1) = subplot(2,3,5);
plot(t_hw(idx_FL), distance_FL, '.'); grid on; hold on;
plot(t_hw(idx_FR), distance_FR, '.');
plot(t_hw(idx_AL), distance_AL, '.');
plot(t_hw(idx_AR), distance_AR, '.');
legend("FL", "FR", "AL", "AR", 'Location', 'best');


%% Roll and pitch on subplot 2 3 3 , 2 3 6 free

axvel(end+1) = subplot(2,3,3);
plot(t_python, rad2deg(phi_python)); grid on; hold on;
plot(t_python, rad2deg(theta_python));

legend("roll phi", "pitch theta", 'Location', 'best');


%% 2 3 6 steering input
t_radio_steering = P.can_msg_timestamp_s;
radio_steering = P.("can_signals_RADIO_CONTROL_STEERING");
idx = ~isnan(radio_steering) & ~isnan(t_radio_steering);
t_radio_steering = t_radio_steering(idx) - t_radio_steering(1);
radio_steering = radio_steering(idx);

axvel(end+1) = subplot(2,3,6);
plot(t_radio_steering, radio_steering);

linkaxes(axvel, 'x');
xlim(xlims);

%%
return
plot_velocity_heave_pitch_roll(t_python, v_python, z_python, rad2deg(theta_python), rad2deg(phi_python), xlims=[485 505], name="log0_lot_stan_gorszy")
plot_velocity_heave_pitch_roll(t_python, v_python, z_python, rad2deg(theta_python), rad2deg(phi_python), xlims=[515 535], name="log0_lot_stan_lepszy")

%%
plot_velocity_heave_pitch_roll(t_python, v_python, z_python, rad2deg(theta_python), rad2deg(phi_python), xlims=xlims)

function plot_velocity_heave_pitch_roll(time_s, velocity_mps, heave_m, pitch_deg, roll_deg, opts)

    arguments
        time_s
        velocity_mps
        heave_m
        pitch_deg
        roll_deg
        opts.name = []
        opts.xlims = []
    end

    Von = 2.4;
    Voff = 1.9;
    active = false(size(velocity_mps));
    enabled = false;
    for i = 1:numel(velocity_mps)
        if velocity_mps(i) >= Von
            enabled = true;
        elseif velocity_mps(i) <= Voff
            enabled = false;
        end
        active(i) = enabled;
    end



    fig = figure('Color', 'w', 'Name','', ...
        'Units','centimeters','Position',[2 2 8 16]);
    tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax(1) = nexttile;
    plot(time_s, velocity_mps, 'LineWidth', 0.5);
    ylabel('Velocity [m/s]', 'FontName', 'Times New Roman', 'FontSize', 9);
    ylim([0 3.3]);
    grid on;

    ax(2) = nexttile;
    plot(time_s, heave_m, 'LineWidth', 0.5);
    hold on;
    ref = nan(size(heave_m));
    ref(active) = -0.1;
    plot(time_s, ref, 'k--', 'LineWidth', 0.5);
    ylabel('Heave [m]', 'FontName', 'Times New Roman', 'FontSize', 9);
    ylim([-0.16 0.06]);
    grid on;

    ax(3) = nexttile;
    plot(time_s, pitch_deg, 'LineWidth', 0.5);
    hold on;
    ref = nan(size(heave_m));
    ref(active) = 0;
    plot(time_s, ref, 'k--', 'LineWidth', 0.5);
    ylabel('Pitch [\circ]', 'FontName', 'Times New Roman', 'FontSize', 9);
    ylim([-5 15]);
    grid on;

    ax(4) = nexttile;
    plot(time_s, roll_deg, 'LineWidth', 0.5);
    hold on;
    plot(time_s, zeros(size(time_s)), 'k--', 'LineWidth', 0.5);
    ylabel('Roll [\circ]', 'FontName', 'Times New Roman', 'FontSize', 9);
    xlabel('Time [s]', 'FontName', 'Times New Roman', 'FontSize', 9);
    ylim([-10 10]);
    grid on;

    % Shared time axis
    linkaxes(ax, 'x');

    % Show time tick labels only on bottom plot
    set(ax(1:3), 'XTickLabel', []);

    % Font for tick labels
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 9);

    tl.Units = 'normalized';
    tl.Position = tl.Position + [+.01 0 -.01 0];
    for k = 1:4
        ax(k).YLabel.Units = 'normalized';
        ax(k).YLabel.Position = [-0.12, 0.5, 0];
    end

    % Optional time limits
    if ~isempty(opts.xlims)
        xlim(ax(4), opts.xlims);
    end

    % Optional save - only if name is provided
    if ~isempty(opts.name) && strlength(string(opts.name)) > 0
        exportgraphics(fig, "validation/eh/" + string(opts.name) + ".pdf", ...
            'ContentType','vector', 'BackgroundColor','none', ...
            'Units','centimeters', 'Width',8, 'Height',16);
    end
end


%% 
plot_estimated_feedback_current( ...
        t_python, delta_R_python, ...
        t_R_hw, delta_R_hw, ...
        t_R__current_hw, current_R, ...
        xlims=[485 535], name="log0_lot_tylne_skrzydlo")

function plot_estimated_feedback_current( ...
        estimate_time_s, estimate_delta_deg, ...
        feedback_time_s, feedback_delta_deg, ...
        current_time_s, current_a,...
        opts)

    arguments
        estimate_time_s
        estimate_delta_deg

        feedback_time_s
        feedback_delta_deg

        current_time_s
        current_a

        opts.name = []
        opts.xlims = []
    end


    % Estimated vs feedback
    f1 = figure('Color', 'w', 'Name','', ...
        'Units','centimeters','Position',[2 2 8 4.5]);

    ax1 = gca;
    plot(estimate_time_s, estimate_delta_deg, '--', 'LineWidth', 1.2);
    hold on;
    plot(feedback_time_s, feedback_delta_deg, ':', ...
        'LineWidth', 1.5, ...
        'Color', [0.9290 0.6940 0.1250]);

    xlabel('Time [s]', 'FontName', 'Times New Roman', 'FontSize', 9);
    ylabel('Angle [\circ]', 'FontName', 'Times New Roman', 'FontSize', 9);
    legend('Estimated', 'Measured', 'Location', 'best');
    grid on;

    % Current
    f2 = figure('Color', 'w', 'Name','', ...
        'Units','centimeters','Position',[2 2 8 4.5]);

    ax2 = gca;
    plot(current_time_s, -1*current_a, 'LineWidth', 0.5);
    grid on;
    ylim([-.2 .8]);

    xlabel('Time [s]', 'FontName', 'Times New Roman', 'FontSize', 9);
    ylabel('Current [A]', 'FontName', 'Times New Roman', 'FontSize', 9);

    % Font for tick labels
    set([ax1 ax2], 'FontName', 'Times New Roman', 'FontSize', 9);

    % Optional time limits
    if ~isempty(opts.xlims)
        xlim(ax1, opts.xlims);
        xlim(ax2, opts.xlims);
    end

    % Optional save - only if name is provided
    if ~isempty(opts.name) && strlength(string(opts.name)) > 0
        exportgraphics(f1, "validation/eh/" + string(opts.name) + "_angle.pdf", ...
            'ContentType','vector','BackgroundColor','none', ...
            'Units','centimeters','Width',8,'Height',4.5);

        exportgraphics(f2, "validation/eh/" + string(opts.name) + "_current.pdf", ...
            'ContentType','vector','BackgroundColor','none', ...
            'Units','centimeters','Width',8,'Height',4.5);
    end
end