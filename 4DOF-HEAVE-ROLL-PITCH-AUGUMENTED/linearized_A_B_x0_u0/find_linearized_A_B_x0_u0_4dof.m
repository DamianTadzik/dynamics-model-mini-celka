function [A, B, x0, u0] = find_linearized_A_B_x0_u0_4dof(x0, u0)

    doSave = (nargin == 0);
    if nargin == 0
        load('tmp_trim_4dof.mat','trim');
        x0 = trim.x0;
        u0 = trim.u0;
    elseif nargin ~= 2
        error('Provide either no arguments or both x0 and u0.');
    end

    params = boat_model_parameters_4dof();
    w = [0; 0; 0]; %% No disturbances
    
    % Wrap dynamics into f(x,u)->xdot
    f = @(x,u) boat_dynamics_4dof(x,u,w,params);
    
    % Step sizes (IMPORTANT: radians for angles/actuators)
    dx = zeros(size(x0));
    dx(1) = 1e-3;          % xW [m] (not important but keep nonzero)
    dx(2) = 1e-4;          % xWdot [m/s]
    dx(3) = 1e-4;          % z [m]
    dx(4) = 1e-4;          % z_dot [m/s]
    dx(5) = deg2rad(0.01); % phi [rad]
    dx(6) = deg2rad(0.01); % theta [rad]
    dx(7) = deg2rad(0.01); % psi [rad]
    dx(8) = deg2rad(0.05); % p [rad/s]
    dx(9) = deg2rad(0.05); % q [rad/s]
    dx(10) = deg2rad(0.05); % r [rad/s]
    dx(11) = 0.02; % FL [deg]
    dx(12) = 0.02; % FR [deg]
    dx(13) = 0.02; % R  [deg]
    
    du = zeros(size(u0));
    du(1) = 0.02;   % alpha_FL [deg]
    du(2) = 0.02;   % alpha_FR [deg]
    du(3) = 0.02;   % alpha_R  [deg]
    du(4) = 0.1;    % thrust [N]
    
    [A,B,f0] = linearize_fd(f, x0, u0, dx, du, "central");

    %% Important to reduce the A B to get rid of two unused yaw states
    % and to get rid of thrust input from B
    ix = [
        1;  % 1  xW
        2;  % 2  xWdot 
        3;  % 3  zW        heave position (world, NED, +down)
        4;  % 4  zWdot     heave velocity
        5;  % 5  phi       roll angle
        6;  % 6  theta     pitch angle
            % 7  psi       yaw angle
        8;  % 8  p         roll rate (body)
        9;  % 9  q         pitch rate (body)
            % 10 r         yaw rate (body)
        11; 12; 13;  % 11 12 13   FL FR R   augumented acutators dynamics
    ];    
    iu = [
        1; % 1 alpha_FL
        2; % 2 alpha_FR
        3; % 3 alpha_R
           % 4 thrust
    ];
    
    A = A(ix,ix);
    B = B(ix,iu);
    x0 = x0(ix);
    u0 = u0(iu);
    
    %% Save linearized model
    if doSave
        save("tmp_linearized_A_B_x0_u0_4dof.mat", ...
            "A", ...
            "B", ...
            "x0", ...
            "u0");
    end
    %% Checks
    fprintf('\n=== LINEARIZATION ===\n');
    fprintf('||f0|| = %.3e (should be ~0 at trim)\n', norm(f0));
    
    % Stability of open-loop linearized dynamics
    eigA = eig(A);
    fprintf('Max real(eig(A)) = %+8.4e\n', max(real(eigA)));
    
    % Optional: show a few dominant poles
    [~,idx] = sort(real(eigA),'descend');
    disp('Eigenvalues (sorted by real part):');
    disp(eigA(idx).');
    
end
