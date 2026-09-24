%% Read out measurements, name signals and save to a mat file

% Keep measurements in a struct
s = struct();

% Structure with time
m = out.measurements;

s.time = m.time(:);

% gyro x y z
s.gyro = squeeze(m.signals.values(1:3, :, :)).';

% accel x y z
s.accel = squeeze(m.signals.values(4:6, :, :)).';

% tof
s.tof = squeeze(m.signals.values(7:10, :, :)).';

% tof status / new
s.tof_new = squeeze(m.signals.values(11:14, :, :)).';

% GPS velocity
s.gps = squeeze(m.signals.values(15, :, :));

% GPS new
s.gps_new = squeeze(m.signals.values(16, :, :));

%% Verify dimensions

disp("time:");
disp(size(s.time));

disp("gyro:");
disp(size(s.gyro));

disp("accel:");
disp(size(s.accel));

disp("tof:");
disp(size(s.tof));

disp("tof_new:");
disp(size(s.tof_new));

disp("gps:");
disp(size(s.gps));

disp("gps_new:");
disp(size(s.gps_new));

%% Plot signals for verification against Simulink scopes

% Figure 1 - IMU: gyro + accel
figure;

subplot(2,1,1);
plot(s.time, s.gyro);
grid on;
ylabel("Gyro [deg/s]");
title("Gyroscope");
legend("GX", "GY", "GZ");

subplot(2,1,2);
plot(s.time, s.accel);
grid on;
ylabel("Acceleration [g]");
xlabel("Time [s]");
title("Accelerometer");
legend("AX", "AY", "AZ");

% Figure 2 - ToF measurements + update/status signals
figure;

subplot(2,1,1);
stairs(s.time, s.tof);
grid on;
ylabel("Distance [mm]");
title("ToF measurements");
legend("ToF 1", "ToF 2", "ToF 3", "ToF 4");

subplot(2,1,2);
stairs(s.time, s.tof_new);
grid on;
ylabel("Update / status");
xlabel("Time [s]");
title("ToF update signals");
legend("ToF 1", "ToF 2", "ToF 3", "ToF 4");

% Figure 3 - GPS speed + update signal
figure;

subplot(2,1,1);
stairs(s.time, s.gps);
grid on;
ylabel("Speed [m/s]");
title("GPS speed");

subplot(2,1,2);
stairs(s.time, s.gps_new);
grid on;
ylabel("gps\_new");
xlabel("Time [s]");
title("GPS update signal");

%% Save for python replay
save("observer_replay_inputs.mat", "s", "-v7");

%% Also save expected observer/controller outputs from MATLAB

e = out.estimated;
r = struct();
% Time
r.time = e.time(:);
% Velocity [m/s]
r.velocity_mps = squeeze(e.signals.values(1, :, :));
r.velocity_mps = r.velocity_mps(:);
% x_hat:
% [z z_dot phi theta psi p q r]
r.xhat = squeeze(e.signals.values(2:9, :, :)).';
% Estimated actuator states:
% [delta_FL delta_FR delta_R]
r.deltas = squeeze(e.signals.values(10:12, :, :)).';

% Delay states
d = out.delay_states;
% delay states:
% [FL_d1 FR_d1 R_d1 FL_d2 FR_d2 R_d2 ...]
r.delay_states = squeeze(d.signals.values).';

% Controller output
c = out.control;
% [FL FR R]
r.control = squeeze(c.signals.values).';
% Verify dimensions
disp("Expected output dimensions:");
fprintf("time:         %s\n", mat2str(size(r.time)));
fprintf("velocity:     %s\n", mat2str(size(r.velocity_mps)));
fprintf("xhat:         %s\n", mat2str(size(r.xhat)));
fprintf("deltas:       %s\n", mat2str(size(r.deltas)));
fprintf("delay_states: %s\n", mat2str(size(r.delay_states)));
fprintf("control:      %s\n", mat2str(size(r.control)));
N = numel(r.time);
assert(size(r.velocity_mps,1) == N);
assert(size(r.xhat,1) == N);
assert(size(r.deltas,1) == N);
assert(size(r.delay_states,1) == N);
assert(size(r.control,1) == N);
% Save
save("observer_controller_simulation.mat", "r", "-v7");