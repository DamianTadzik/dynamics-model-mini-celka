clc;
clear;

%% Load linearization grid
load("linearization_grid_zW_xWdot.mat", ...
    "linearizations", ...
    "zWGrid_m", ...
    "xWdotGrid_ms");

%% Select operating point
nominal_zW_m = -0.10;
nominal_velocity_ms = 3.0;

[~, iz] = min(abs(zWGrid_m - nominal_zW_m));
[~, iv] = min(abs(xWdotGrid_ms - nominal_velocity_ms));

nominal_zW_m = zWGrid_m(iz);
nominal_velocity_ms = xWdotGrid_ms(iv);

fprintf("Finding MPC controller at:\n");
fprintf("  zW = %.3f m\n", nominal_zW_m);
fprintf("  V  = %.3f m/s\n\n", nominal_velocity_ms);

%% Get linearization
lin = linearizations(iz, iv);

assert(~isempty(lin.A), "No linearization at selected point.");

%% Create MPC controller
ctrl = find_all_MPC( ...
    lin.A, ...
    lin.B, ...
    lin.x0, ...
    lin.u0);

%% Extract controller and model
mpcobj = ctrl.mpcobj;

Aaug = ctrl.Aaug;
Baug = ctrl.Baug;

x0 = ctrl.x0;
u0 = ctrl.u0;

Ts = ctrl.Ts;

%% Display summary
fprintf("\n=== MPC SUMMARY ===\n");

fprintf("States: %d\n", size(Aaug,1));
fprintf("Inputs: %d\n", size(Baug,2));
fprintf("Sample time: %.3f s\n", Ts);

disp(mpcobj);

%% Save
save("tmp_MPC_controller.mat", ...
    "mpcobj", ...
    "Aaug", ...
    "Baug", ...
    "x0", ...
    "u0", ...
    "nominal_zW_m", ...
    "nominal_velocity_ms");
