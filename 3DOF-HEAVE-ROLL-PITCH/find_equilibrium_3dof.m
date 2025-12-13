clear; clc;

% Load parameters
params = boat_model_parameters_3dof();

% choosen forward speed and thrust
Velcity0 = 3.0;   % [m/s]
Thrust0 = 0;  % [N] (can i estimate that from power drawn by ESC? and speed)
% temporarly 0 

% Initial guess: controllable parameters [alpha_front_left, alpha_front_right, alpha_rear]
v0 = [
    deg2rad(3);
    deg2rad(3); 
    deg2rad(6)  
];

% Bounds for those (important)
lb = [ -6; -6; -6 ];
ub = [ 12; 12; 12 ];

% Flight parameters
zW0    = -0.1;         % [m]
phi0   = deg2rad(0);    % roll
theta0 = deg2rad(0);    % pitch

opts = optimoptions('lsqnonlin','Display','iter');

%% Solve
v = lsqnonlin( ...
    @(vv) trim_residual_3dof(vv,params,zW0,phi0,theta0,Velcity0,Thrust0), ...
    v0, lb, ub, opts);

%% Check the solution
alpha_FL = v(1);
alpha_FR = v(2);
alpha_R  = v(3);

% Build trimmed state
x0 = zeros(8,1);
x0(1) = zW0;
x0(2) = 0;
x0(3) = phi0;
x0(4) = theta0;
x0(5) = 0;
x0(6:8) = 0;

% Build trimmed input
u0 = zeros(5,1);
u0(1) = alpha_FL;
u0(2) = alpha_R;
u0(3) = alpha_FR;
u0(4) = Thrust0;
u0(5) = Velcity0;

[xdot,info] = boat_dynamics_3dof(x0,u0,params);

%% Print info

% fprintf('\n=== Forces check ===\n');
% fprintf('FB_up   = %+8.2f N\n', FB_up);
% fprintf('Fz_foil = %+8.2f N\n', Fz_up);
% fprintf('mg      = %+8.2f N\n', params.m * params.g);

fprintf('\n=== TRIM CHECK ===\n');

fprintf('Heave accel      z_ddot : %+8.4f  m/s^2\n', xdot(2));
fprintf('Roll accel       p_dot  : %+8.4f  rad/s^2\n', xdot(6));
fprintf('Pitch accel      q_dot  : %+8.4f  rad/s^2\n', xdot(7));
fprintf('Yaw accel        r_dot  : %+8.4f  rad/s^2\n', xdot(8));

fprintf('\nState (trimmed):\n');
fprintf('zW      = %+6.3f m\n', x0(1));
fprintf('phi     = %+6.3f deg\n', rad2deg(x0(3)));
fprintf('theta   = %+6.3f deg\n', rad2deg(x0(4)));

fprintf('\nActuators (trimmed):\n');
fprintf('alpha_FL = %+6.3f deg\n', rad2deg(u0(1)));
fprintf('alpha_FR = %+6.3f deg\n', rad2deg(u0(3)));
fprintf('alpha_R  = %+6.3f deg\n', rad2deg(u0(2)));

fprintf('\nSpeed:\n');
fprintf('V = %.2f m/s\n', Velcity0);

fprintf('==================\n');

%% Save the solved parameters for later use or something like that

trim.x0 = x0;
trim.u0 = u0;
trim.V0 = Velcity0;
trim.zW = zW0;
trim.phi = phi0;
trim.theta = theta0;

save('trim_3dof_V3_zm0p1.mat','trim');


%%
function r = trim_residual_3dof(v, params, zW, phi, theta, V0, Thrust0)
    % v = [alpha_FL; alpha_FR; alpha_R]   (radians)
    
    alpha_FL = v(1);
    alpha_FR = v(2);
    alpha_R  = v(3);
    
    % --- Build state (8 states)
    x = zeros(8,1);
    x(1) = zW;        % heave position
    x(2) = 0;         % heave velocity
    x(3) = phi;       % roll
    x(4) = theta;     % pitch
    x(5) = 0;         % yaw
    x(6:8) = 0;       % body rates
    
    % --- Build input
    u = zeros(5,1);
    u(1) = alpha_FL;
    u(2) = alpha_R;
    u(3) = alpha_FR;
    u(4) = Thrust0;
    u(5) = V0;
    
    % --- Evaluate dynamics
    xdot = boat_dynamics_3dof(x, u, params);
    
    % --- Residuals: accelerations must be zero
    r = [
        xdot(2);   % heave acceleration
        xdot(6);   % roll acceleration
        xdot(7);   % pitch acceleration
    ];
end
