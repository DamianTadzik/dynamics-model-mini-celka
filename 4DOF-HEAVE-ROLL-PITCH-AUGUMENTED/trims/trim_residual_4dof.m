function r = trim_residual_4dof(v0, params, zW, phi, theta, Velocity)
    % v = [alpha_FL; alpha_FR; alpha_R]   [deg]
    alpha_FL = v0(1);
    alpha_FR = v0(2);
    alpha_R  = v0(3);
    Thrust0  = v0(4);
    
    % Build state (13 states)
    x = zeros(13,1);
    x(1) = 0;         % surge position
    x(2) = Velocity; % surge velocity
    x(3) = zW;        % heave position
    x(4) = 0;         % heave velocity
    x(5) = phi;       % roll
    x(6) = theta;     % pitch
    x(7) = 0;         % yaw
    x(8:10) = 0;      % r p y body rates
    % actuator equilibrium states
    x(11) = alpha_FL;
    x(12) = alpha_FR;
    x(13) = alpha_R;
    
    % Build input
    u = zeros(4,1);
    u(1) = alpha_FL;
    u(2) = alpha_FR;
    u(3) = alpha_R;
    u(4) = Thrust0;

    % Build disturbance (zeros)
    w = zeros(3,1);
    w(1) = 0; % heave disturbance [N]
    w(2) = 0; % roll disturbance [Nm]
    w(3) = 0; % pitch disturbance [Nm]
    
    % Evaluate dynamics
    xdot = boat_dynamics_4dof(x, u, w, params);
    
    % Residuals: steady constant-speed flight
    r = [
        xdot(2) / 0.02;  % xW_ddot  (surge accel) -> 0 defines required thrust
        xdot(4) / 0.02;  % zW_ddot  (heave accel)
        xdot(8) / 0.002;  % p_dot    (roll accel)
        xdot(9) / 0.002;  % q_dot    (pitch accel)

        xdot(11);  % alpha_FL_dot
        xdot(12);  % alpha_FR_dot
        xdot(13);  % alpha_R_dot
    ];
end
