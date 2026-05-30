function [A, B] = linearize_4dof(x0, u0)

    % load('last_trim.mat','trim');
    % x0 = trim.x0;
    % u0 = trim.u0;

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
    du(4) = 0.2;    % thrust [N]
    
    [A,B,f0] = linearize_fd(f, x0, u0, dx, du, "central");
    
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
