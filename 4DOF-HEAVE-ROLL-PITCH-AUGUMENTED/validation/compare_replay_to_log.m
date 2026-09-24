%% Files
parquetFile = "log0.parquet";
matFile     = "observer_controller_simulation.mat";

%% Load Python runtime log
P = parquetread(parquetFile);
disp("Parquet columns:");
disp(P.Properties.VariableNames.');

%% Load MATLAB reference
D = load(matFile);
r = D.r;

%% MATLAB reference
t_mat = r.time(:);
v_mat = r.velocity_mps(:);

t_mat = t_mat - t_mat(1);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Python observer from Parquet
t_python = P.timestamp_s;
v_python = P.("control_cycle_estimated_state_velocity_mps");
idx = ~isnan(v_python) & ~isnan(t_python);
v_python = v_python(idx);
t_python = t_python(idx);

t_python = t_python - t_python(1);

%% Plot
figure;
hold on;
grid on;
plot(t_mat, v_mat, ...
    'LineWidth', 1.5);
plot(t_python, v_python, '--', ...
    'LineWidth', 1.2);
xlabel("Time [s]");
ylabel("Velocity [m/s]");
title("Forward velocity estimator comparison");
legend( ...
    "MATLAB", ...
    "Python", ...
    'Location', 'best' ...
);
xlim([0 min(t_mat(end), t_python(end))]);

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

subplot(3,1,1);
hold on;
grid on;

plot(t_ctrl_mat, u_FL_mat, 'LineWidth', 1.5);
plot(t_ctrl_python, u_FL_python, '--', 'LineWidth', 1.2);

ylabel("FL [deg]");
title("Front-left actuator command");

legend("MATLAB", "Python", 'Location', 'best');

xlim([0 min(t_ctrl_mat(end), t_ctrl_python(end))]);


subplot(3,1,2);
hold on;
grid on;

plot(t_ctrl_mat, u_FR_mat, 'LineWidth', 1.5);
plot(t_ctrl_python, u_FR_python, '--', 'LineWidth', 1.2);

ylabel("FR [deg]");
title("Front-right actuator command");

xlim([0 min(t_ctrl_mat(end), t_ctrl_python(end))]);


subplot(3,1,3);
hold on;
grid on;

plot(t_ctrl_mat, u_R_mat, 'LineWidth', 1.5);
plot(t_ctrl_python, u_R_python, '--', 'LineWidth', 1.2);

xlabel("Time [s]");
ylabel("Rear [deg]");
title("Rear actuator command");

xlim([0 min(t_ctrl_mat(end), t_ctrl_python(end))]);

sgtitle("Controller output comparison");


%% Controller timing comparison

dt_ctrl_mat = diff(t_ctrl_mat);
dt_ctrl_python = diff(t_ctrl_python);

figure;
hold on;
grid on;

plot(t_ctrl_mat(2:end), dt_ctrl_mat, ...
    'LineWidth', 1.2);

plot(t_ctrl_python(2:end), dt_ctrl_python, '--', ...
    'LineWidth', 1.2);

xlabel("Time [s]");
ylabel("\Delta t [s]");
title("Controller sampling interval comparison");

legend( ...
    "MATLAB", ...
    "Python", ...
    'Location', 'best' ...
);

xlim([0 min(t_ctrl_mat(end), t_ctrl_python(end))]);
