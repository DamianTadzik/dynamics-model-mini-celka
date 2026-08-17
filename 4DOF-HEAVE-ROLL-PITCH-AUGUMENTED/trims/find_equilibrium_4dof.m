function [trim, info_out] = find_equilibrium_4dof(varargin)

    % defaults 
    cfg.params   = boat_model_parameters_4dof();
    cfg.xWdot    = 2.5;
    cfg.zW       = -0.10;
    cfg.phi      = 0;
    cfg.theta    = 0;
    cfg.v0       = [-5;-5;-5;8];
    cfg.lb       = [-6;-6;-6;0];
    cfg.ub       = [12;12;12;20];
    
    cfg.opts = optimoptions('lsqnonlin', ...
        'Display','iter', ...
        'MaxIterations',400, ...
        'MaxFunctionEvaluations',4000, ...
        'FunctionTolerance',1e-14, ...
        'StepTolerance',1e-14, ...
        'OptimalityTolerance',1e-12, ...
        'FiniteDifferenceType','central', ...
        ... 'FiniteDifferenceStepSize',[0.02;0.02;0.02;0.05], ...
        'FiniteDifferenceStepSize',[0.001; 0.001; 0.002; 0.01], ...
        'Algorithm','trust-region-reflective', ...
        'ScaleProblem','jacobian');

    % ovveride 
    for k = 1:2:numel(varargin)
        assert(isfield(cfg,varargin{k}), 'Unknown option "%s"', varargin{k});
        cfg.(varargin{k}) = varargin{k+1};
    end
        
    %%
    
    % Load parameters
    % params = boat_model_parameters_4dof();
    params = cfg.params;
    
    % Desired forward speed for trim
    Velocity0 = cfg.xWdot;   % [m/s]
    % Other flight parameters
    zW0    = cfg.zW;         % [m]
    phi0   = deg2rad(cfg.phi);    % roll intended is LEVELED so =0
    theta0 = deg2rad(cfg.theta);    % pitch intended is LEVELED so =0
    
    % Initial guess: controllable parameters [alpha_front_left, alpha_front_right, alpha_rear, thrust]
    v0 = cfg.v0;
    % v0 = [
    %     -5;
    %     -5;
    %     -5;
    %     1
    % ];

    % Bounds for those 'actuators'
    % lb = [ -6; -6; -6; 0];
    % ub = [ 12; 12; 12; 10]; % 10 Newtons?? IDK to be tested
    lb = cfg.lb;
    ub = cfg.ub;
    %% Solve
    opts = cfg.opts;
    v = lsqnonlin( ...
        @(vv) trim_residual_4dof(vv,params,zW0,phi0,theta0,Velocity0), ...
        v0, lb, ub, opts);
    
    %% Check the solution
    alpha_FL = v(1);
    alpha_FR = v(2);
    alpha_R  = v(3);
    Thrust   = v(4);
    
    % Build trimmed state
    x0 = zeros(13,1);
    x0(1) = 0;         % surge
    x0(2) = Velocity0;  % surge rate
    x0(3) = zW0; % heave
    x0(4) = 0;   % heave rate
    x0(5) = phi0; % roll
    x0(6) = theta0; % pitch
    x0(7) = 0; % yaw
    x0(8:10) = 0; % angular rates
    x0(11) = alpha_FL;
    x0(12) = alpha_FR;
    x0(13) = alpha_R;
    
    % Build trimmed input
    u0 = zeros(4,1);
    u0(1) = alpha_FL;
    u0(2) = alpha_FR;
    u0(3) = alpha_R;
    u0(4) = Thrust;
    
    % Build zero disturbance
    w0 = zeros(3,1);
    
    % Evaluate dynamics
    [xdot, info] = boat_dynamics_4dof(x0,u0,w0,params);
    
    %% Print info
    fprintf('\n=== TRIM CHECK (4DOF) ===\n');
    fprintf('Surge accel      x_ddot : %+8.4f  m/s^2\n', xdot(2));
    fprintf('Heave accel      z_ddot : %+8.4f  m/s^2\n', xdot(4));
    fprintf('Roll accel       p_dot  : %+8.4f  rad/s^2\n', xdot(8));
    fprintf('Pitch accel      q_dot  : %+8.4f  rad/s^2\n', xdot(9));
    fprintf('Yaw accel        r_dot  : %+8.4f  rad/s^2\n', xdot(10));
    
    fprintf('\nState (trimmed):\n');
    fprintf('xWdot   = %+6.3f m/s\n', x0(2));
    fprintf('zW      = %+6.3f m\n',   x0(3));
    fprintf('phi     = %+6.3f deg\n', rad2deg(x0(5)));
    fprintf('theta   = %+6.3f deg\n', rad2deg(x0(6)));
    
    fprintf('\nActuators (trimmed):\n');
    fprintf('alpha_FL = %+6.3f deg\n', u0(1));
    fprintf('alpha_FR = %+6.3f deg\n', u0(2));
    fprintf('alpha_R  = %+6.3f deg\n', u0(3));
    
    fprintf('\nThrust:\n');
    fprintf('T = %.3f N\n', Thrust);
    fprintf('========================\n');
    
    %% Save the solved parameters for later use or something like that
    
    % Pack outputs
    trim.x0 = x0;
    trim.u0 = u0;
    trim.v  = v;
    trim.xdot = xdot;

    info_out = info;
        
    save('last_trim.mat','trim');
end
