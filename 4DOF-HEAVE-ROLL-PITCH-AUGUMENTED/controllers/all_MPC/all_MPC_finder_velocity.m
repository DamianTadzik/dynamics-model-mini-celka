%% Prepare adaptive MPC model grid along forward velocity
% Other operating conditions are kept at nominal heave zW = -0.10 m.
% The grid contains numeric data for boat_controller_parameters().
% One nominal mpcobj is saved separately in the same MAT file for Simulink.

clc;
clear;

%% Load linearization grid
load("linearization_grid_zW_xWdot.mat", ...
    "linearizations", ...
    "zWGrid_m", ...
    "xWdotGrid_ms");

%% Select operating conditions
nominal_zW_m = -0.10;
nominal_velocity_ms = 3.0;

[~, iz] = min(abs(zWGrid_m - nominal_zW_m));
[~, iv] = min(abs(xWdotGrid_ms - nominal_velocity_ms));

nominal_zW_m = zWGrid_m(iz);
nominal_velocity_ms = xWdotGrid_ms(iv);

velocity_grid = xWdotGrid_ms(:);
Nv = numel(velocity_grid);

assert(Nv >= 2 && all(diff(velocity_grid) > 0), ...
    "Velocity grid must contain at least two increasing points.");

fprintf("Finding MPC model grid at zW = %.3f m\n", nominal_zW_m);
fprintf("Nominal controller velocity: %.3f m/s\n", nominal_velocity_ms);
fprintf("Number of velocity points: %d\n\n", Nv);

%% Create nominal MPC controller and determine model dimensions
lin = linearizations(iz, iv);
assert(~isempty(lin.A) && ~isempty(lin.B), ...
    "No linearization at nominal operating point.");

nominal_ctrl = find_all_MPC(lin.A, lin.B, lin.x0, lin.u0);

nx = size(nominal_ctrl.Aaug, 1);
nu = size(nominal_ctrl.Baug, 2);
C = nominal_ctrl.C;
D = nominal_ctrl.D;

Ts = nominal_ctrl.Ts;
Ld = nominal_ctrl.Ld;

assert(nx == 21 && nu == 3, "Unexpected augmented model dimensions.");

%% Preallocate model grids
Aaug_grid = zeros(nx, nx, Nv);
Baug_grid = zeros(nx, nu, Nv);
x0aug_grid = zeros(nx, Nv);
u0_grid = zeros(nu, Nv);

%% Prepare model for each velocity
for j = 1:Nv
    lin = linearizations(iz, j);

    assert(~isempty(lin.A) && ~isempty(lin.B) && ...
           ~isempty(lin.x0) && ~isempty(lin.u0), ...
           "Missing linearization at V = %.3f m/s.", velocity_grid(j));

    if j == iv
        ctrl = nominal_ctrl;
    else
        ctrl = find_all_MPC(lin.A, lin.B, lin.x0, lin.u0);
    end

    assert(isequal(size(ctrl.Aaug), [nx nx]) && ...
           isequal(size(ctrl.Baug), [nx nu]), ...
           "Inconsistent state-space dimensions at V = %.3f m/s.", ...
           velocity_grid(j));
    assert(ctrl.Ts == Ts && ctrl.Ld == Ld, ...
        "Sampling time or delay length differs between grid points.");
    assert(isequal(ctrl.C, C) && isequal(ctrl.D, D), ...
        "Output model differs between grid points.");

    Aaug_grid(:,:,j) = ctrl.Aaug;
    Baug_grid(:,:,j) = ctrl.Baug;

    % First 9 states are physical states; the remaining 12 are
    % queued actuator COMMANDS, whose equilibrium value is u0.
    x0aug_grid(:,j) = [ctrl.x0(:); repmat(ctrl.u0(:), Ld, 1)];
    u0_grid(:,j) = ctrl.u0(:);

    assert(all(isfinite(Aaug_grid(:,:,j)), 'all') && ...
           all(isfinite(Baug_grid(:,:,j)), 'all') && ...
           all(isfinite(x0aug_grid(:,j))) && ...
           all(isfinite(u0_grid(:,j))), ...
           "Non-finite MPC data at V = %.3f m/s.", velocity_grid(j));

    fprintf("V = %.3f m/s ... OK\n", velocity_grid(j));
end

%% Nominal MPC object for Simulink Adaptive MPC Controller
% The helper find_all_MPC() originally creates a deviation-coordinate
% controller. Set native nominal conditions so Simulink can use absolute
% states, references and manipulated variables instead.
mpcobj = nominal_ctrl.mpcobj;
assert(isa(mpcobj, 'mpc'), "Expected an MPC controller object.");

X0 = x0aug_grid(:,iv);
U0 = u0_grid(:,iv);

mpcobj.Model.Nominal = struct( ...
    "X",  X0, ...
    "U",  U0, ...
    "Y",  C*X0, ...
    "DX", zeros(nx,1));

% Physical actuator command limits [deg], not deviation limits.
for i = 1:nu
    mpcobj.MV(i).Min = -6;
    mpcobj.MV(i).Max = 12;
end

Np = mpcobj.PredictionHorizon;
Nc = mpcobj.ControlHorizon;

%% Save numeric grids + ONE nominal MPC object
% Do not put mpcobj into Simulink.Bus.createObject(ctrl_params).
% Only numeric arrays from this MAT file belong in params.controller.MPC.
save("all_MPC_grid_schedule.mat", ...
    "mpcobj", ...
    "Aaug_grid", "Baug_grid", ...
    "x0aug_grid", "u0_grid", "velocity_grid", ...
    "C", "D", "Ts", "Ld", "Np", "Nc", ...
    "nominal_zW_m", "nominal_velocity_ms");

fprintf("\n=== ADAPTIVE MPC GRID SUMMARY ===\n");
fprintf("Velocities: %.3f ... %.3f m/s (%d points)\n", ...
    velocity_grid(1), velocity_grid(end), Nv);
fprintf("Model: %d states, %d inputs, %d outputs\n", nx, nu, size(C,1));
fprintf("Ts = %.3f s, Np = %d, Nc = %d, Ld = %d\n", ...
    Ts, Np, Nc, Ld);
fprintf("Saved: all_MPC_grid_schedule.mat\n");
