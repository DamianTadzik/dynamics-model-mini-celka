% clear; 
clc;

params = boat_model_parameters_3dof();

load('trim_3dof_V3_zm0p1.mat','trim');
x0 = trim.x0;
u0 = trim.u0;
w = [0; 0; 0]; %% No disturbances

% Wrap your dynamics into f(x,u)->xdot
f = @(x,u) boat_dynamics_3dof(x,u,w,params);

% Step sizes (IMPORTANT: radians for angles/actuators)
dx = zeros(size(x0));
dx(1) = 1e-4;          % z [m]
dx(2) = 1e-4;          % z_dot [m/s]
dx(3) = deg2rad(0.01); % phi [rad]
dx(4) = deg2rad(0.01); % theta [rad]
dx(5) = deg2rad(0.01); % psi [rad]
dx(6) = deg2rad(0.05); % p [rad/s]
dx(7) = deg2rad(0.05); % q [rad/s]
dx(8) = deg2rad(0.05); % r [rad/s]
dx(9)  = 0.02; % FL [deg]
dx(10) = 0.02; % FR [deg]
dx(11) = 0.02; % R  [deg]

du = zeros(size(u0));
du(1) = 0.02;   % alpha_FL [deg]
du(2) = 0.02;   % alpha_FR [deg]
du(3) = 0.02;   % alpha_R  [deg]
du(4) = 0.1;    % thrust [N] (unused now but keep nonzero)
du(5) = 0.01;   % speed [m/s] (since you treat it as an input)

[A,B,f0] = linearize_fd(f, x0, u0, dx, du, "central");

fprintf('\n=== LINEARIZATION ===\n');
fprintf('||f0|| = %.3e (should be ~0 at trim)\n', norm(f0));

% Stability of open-loop linearized dynamics
eigA = eig(A);
fprintf('Max real(eig(A)) = %+8.4e\n', max(real(eigA)));

% Optional: show a few dominant poles
[~,idx] = sort(real(eigA),'descend');
disp('Top 6 eigenvalues (by real part):');
disp(eigA(idx(1:min(6,end))).');


%% Try to get the K for LQR

% Select states that are measurable with physical devices like accel
% gyro or states that can be estimated
ix = [1 2 3 4 6 7 9 10 11];    
% 1  zW        heave position (world, NED, +down)
% 2  zWdot     heave velocity
% 3  phi       roll angle
% 4  theta     pitch angle
% 5  psi       yaw angle
% 6  p         roll rate (body)
% 7  q         pitch rate (body)
% 8  r         yaw rate (body)
% 9 10 11   FL FR R   augumented acutators dynamics

% Select what are we controlling and with what order
iu = [1 2 3];
% 1 alpha_FL
% 2 alpha_FR
% 3 alpha_R 

Ar = A(ix,ix);
Br = B(ix,iu);

% check controlability (for theoretical u = kx controller)
Co_r = ctrb(Ar,Br);
fprintf('Controlled-subsystem ctrb rank = %d of %d\n', ...
        rank(Co_r), size(Ar,1));

Cr = [
    1 0 0 0 0 0 0 0 0;   % z (ToF)
    0 0 1 0 0 0 0 0 0;   % phi (assuming we know phi and theta well)
    0 0 0 1 0 0 0 0 0;   % theta (assuming we know phi and theta well)
    0 0 0 0 1 0 0 0 0;   % p (gyro)
    0 0 0 0 0 1 0 0 0;   % q (gyro)
];
Ob_r = obsv(Ar, Cr);
fprintf('Observability rank = %d of %d\n', rank(Ob_r), size(Ar,1));


%% Augumented LQR 
% Penalization for state error
Q = diag([ ...
    12000, ...   1   z       heave
    1, ...     2   zdot    heave velocity
    2000, ...   3   phi     roll angle
    200, ...     4   theta   pitch angle
    200, ...     6   p       roll rate (body)
    800, ...     7   q       pitch rate (body)
    1000, ...    9   FL
    1000, ...    10  FR
    1000 ...     11  R
    ]); 
% Penalization for actuation
R = diag([ ...
    1, ...      FL
    1, ...      FR
    10 ...    R
    ] .*  800);

K = lqr(Ar,Br,Q,R);

% Display the LQR Gain Matrix K:
disp('LQR Gain Matrix K:'); disp(K);
disp('Q:'); disp(Q);
disp('R:'); disp(R);

eig_cl = eig(Ar- Br*K);
disp('Closed-loop max real eig:'); disp(max(real(eig_cl)))
disp('Closed-loop eig:'); disp(eig_cl)

% Save values for continuous controller simulation in simulink
save("tmp_continuous_controller.mat", "K", "u0", "x0", "ix", "iu");

%% Discrete LQR 
Ts_ctrl = 0.01;   % 100 Hz

sysc = ss(Ar, Br, eye(size(Ar,1)), zeros(size(Ar,1), size(Br,2)));
sysd = c2d(sysc, Ts_ctrl, 'zoh');   % ZOH is what digital controller does
Ad = sysd.A;
Bd = sysd.B;

Kd = dlqr(Ad, Bd, Q, R);

eig_cl_d = eig(Ad - Bd*Kd);
fprintf('Discrete closed-loop max |eig| = %.4f (must be in the unit circle)\n', max(abs(eig_cl_d)));
disp('Discrete closed-loop eig:'); disp(eig_cl_d.');
disp('Discrete closed-loop abs(eig):'); disp(abs(eig_cl_d.'));

% Save values for discrete controller simulation in simulink
save("tmp_discrete_controller.mat", "Kd", "Ts_ctrl", "u0", "x0", "ix", "iu");

%%
function [A,B,f0] = linearize_fd(f, x0, u0, dx, du, method)
%LINEARIZE_FD Numerical linearization of xdot = f(x,u) at (x0,u0).
%
% Inputs:
%   f       : function handle, f(x,u) -> xdot (nx1)
%   x0,u0   : trim point
%   dx,du   : step sizes (nx1, nu1) OR scalars
%   method  : "central" (default) or "forward"
%
% Outputs:
%   A,B     : Jacobians at (x0,u0)
%   f0      : f(x0,u0)

    if nargin < 6 || isempty(method), method = "central"; end
    
    x0 = x0(:); u0 = u0(:);
    f0 = f(x0,u0);
    nx = numel(x0);
    nu = numel(u0);
    
    % steps
    if isscalar(dx), dx = dx * ones(nx,1); else, dx = dx(:); end
    if isscalar(du), du = du * ones(nu,1); else, du = du(:); end
    
    A = zeros(nx,nx);
    B = zeros(nx,nu);
    
    switch lower(string(method))
    case "central"
        % A
        for i = 1:nx
            h = dx(i);
            xp = x0; xm = x0;
            xp(i) = xp(i) + h;
            xm(i) = xm(i) - h;
            fp = f(xp,u0);
            fm = f(xm,u0);
            A(:,i) = (fp - fm) / (2*h);
        end
        % B
        for j = 1:nu
            h = du(j);
            up = u0; um = u0;
            up(j) = up(j) + h;
            um(j) = um(j) - h;
            fp = f(x0,up);
            fm = f(x0,um);
            B(:,j) = (fp - fm) / (2*h);
        end
    
    case "forward"
        % A
        for i = 1:nx
            h = dx(i);
            xp = x0; xp(i) = xp(i) + h;
            fp = f(xp,u0);
            A(:,i) = (fp - f0) / h;
        end
        % B
        for j = 1:nu
            h = du(j);
            up = u0; up(j) = up(j) + h;
            fp = f(x0,up);
            B(:,j) = (fp - f0) / h;
        end
    
    otherwise
        error("method must be 'central' or 'forward'");
    end
end
