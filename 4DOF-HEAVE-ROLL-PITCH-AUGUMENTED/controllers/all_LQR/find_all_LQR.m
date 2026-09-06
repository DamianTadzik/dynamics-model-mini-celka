clc; clear;
%% Load A B and reduce to desired control structure even further
load("tmp_linearized_A_B_x0_u0_4dof.mat", "A", "B", "u0", "x0") 
% remember no yaw and yaw rate and thrust in these A B u0 x0 alerady!

% Select states that controller should control
ix = [
        % 1  xW        surge position (world)
        % 2  xWdot     surge velocity
    3;  % 3  zW        heave position (world, NED, +down)
    4;  % 4  zWdot     heave velocity
    5;  % 5  phi       roll angle
    6;  % 6  theta     pitch angle
    7;  % 7  p         roll rate (body)
    8;  % 8  q         pitch rate (body)
    9; 10; 11;  % 9 10 11   FL FR R   augumented acutators dynamics
];    
% Select what are we controlling and with what order
iu = [
    1; % 1 alpha_FL
    2; % 2 alpha_FR
    3; % 3 alpha_R
];

% Reduce matrices to only few states that should be inputs to the LQ
Ar = A(ix,ix);
Br = B(ix,iu);
x0r = x0(ix);

% check controlability (for theoretical u = kx controller)
Co_r = ctrb(Ar,Br);
fprintf('Controlled-subsystem ctrb rank = %d of %d\n', ...
        rank(Co_r), size(Ar,1));

s_r = svd(Co_r);
fprintf('svd: ');
disp(s_r.')

% NOTE check this matrix if it is correct for our system with xW xWdot
% z zdot phi theta p q FL FR R
Cr = [
    % 0 0 0 0 0 0 0 0 0 0;   % xWdot (GPS)
    1 0 0 0 0 0 0 0 0;   % z (ToF)
    0 0 1 0 0 0 0 0 0;   % phi (assuming we know phi and theta well)
    0 0 0 1 0 0 0 0 0;   % theta (assuming we know phi and theta well)
    0 0 0 0 1 0 0 0 0;   % p (gyro)
    0 0 0 0 0 1 0 0 0;   % q (gyro)

    % 1 0 0 0 0 0;   % z (ToF)
    % 0 0 1 0 0 0;   % phi (assuming we know phi and theta well)
    % 0 0 0 1 0 0;   % theta (assuming we know phi and theta well)
    % 0 0 0 0 1 0;   % p (gyro)
    % 0 0 0 0 0 1;   % q (gyro)
];
Ob_r = obsv(Ar, Cr);
fprintf('Observability rank = %d of %d\n', rank(Ob_r), size(Ar,1));

%% Stabilizability check using PBH test
lambda = eig(Ar);
n = size(Ar,1);

fprintf('\nPBH stabilizability check:\n');

for i = 1:length(lambda)
    pbh_rank = rank([lambda(i)*eye(n) - Ar, Br]);

    fprintf('lambda = %+9.4f %+9.4fj, PBH rank = %d/%d', ...
        real(lambda(i)), imag(lambda(i)), pbh_rank, n);

    if real(lambda(i)) >= 0
        if pbh_rank == n
            fprintf('  -> unstable/marginal mode controllable\n');
        else
            fprintf('  -> WARNING: uncontrollable unstable/marginal mode\n');
        end
    else
        fprintf('\n');
    end
end

%% Set the weights and get the K for continouus LQR
% Penalization for state error
Q = diag([ ...
    12000, ...   3   z       heave
    1, ...       4   zdot    heave velocity
    2000, ...    5   phi     roll angle
    200, ...     6   theta   pitch angle
    200, ...     8   p       roll rate (body)
    800, ...     9   q       pitch rate (body)
    1000, ...    11  FL
    1000, ...    12  FR
    1000 ...     13  R
    ]); 
% Penalization for actuation
R = diag([ ...
    1, ...      FL
    1, ...      FR
    10 ...      R
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


%% Augumented discrete LQR

Ld = 4; % number of delay states per actuator
nu = 3; % number of actuators

% ONE DELAY STATE:
% 9x9 gets augumented into (9+3*4)x(9+3*4)
Aaug = [ ...
    Ad,                   Bd;
    zeros(nu,size(Ad,2)), zeros(nu,nu);
];
% 9x3 gets augumented to (9+3*4)x3
Baug = [...
    zeros(size(Bd,1),nu);
    eye(nu);
];
% REST OF THE DELAY STATES
for d = 2:Ld
    Aaug = [ ...
        Aaug,                   Baug;
        zeros(nu,size(Aaug,2)), zeros(nu,nu);
    ];
    Baug = [...
        zeros(size(Baug,1),nu);
        eye(nu);
    ];
end

% x0 also should be augumented.
x0aug = x0r;
for d = 1:Ld
    x0aug = [x0aug; x0aug(end-2); x0aug(end-1); x0aug(end-0)];
end

fprintf('Augmented system size: %d states, %d inputs\n', size(Aaug,1), size(Baug,2));

% No penalty for actuator delay states?
Qaug = blkdiag(Q, zeros(nu*Ld));

Kaug = dlqr(Aaug, Baug, Qaug, R);

%  TESTS
% Closed-loop stability
Acl = Aaug - Baug*Kaug;
eig_cl = eig(Acl);
fprintf('Max |eig(Acl)| = %.6f\n', max(abs(eig_cl)));

% Stabilizability of augmented plant
n = size(Aaug,1);
lambda = eig(Aaug);
fprintf('\nPBH stabilizability check:\n');
for i = 1:length(lambda)
    r = rank([lambda(i)*eye(n) - Aaug,Baug]);
    if abs(lambda(i)) >= 1
        fprintf('lambda = %+9.5f %+9.5fj, rank = %d/%d\n', ...
            real(lambda(i)), imag(lambda(i)), r, n);
    end
end
% Controllability conditioning
Co_aug = ctrb(Aaug,Baug);
s_aug = svd(Co_aug);
fprintf('\nControllability singular values:\n');
disp(s_aug.');
% Closed-loop damping / slowest modes
[~,idx] = sort(abs(eig_cl),'descend');
disp('Dominant closed-loop poles:');
disp(eig_cl(idx(1:min(10,end))).');

% Save 
save("tmp_discrete_augumented_controller.mat", "Kaug", "x0aug", "u0", "ix", "iu");
