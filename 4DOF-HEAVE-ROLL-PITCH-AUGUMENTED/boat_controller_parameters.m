function params = boat_controller_parameters()
    %% Controller + observer + actuator model parameters and constants

    params.Ts = 0.01; % 100 Hz 

    params.g = 9.81; % [m/s^2]   gravity 
    
    %% Distance sensors placement in a B frame
    params.tof.pos_FL_B = [ +225; -182; -37 ] / 1000;  % [m]
    params.tof.pos_FR_B = [ +225; +182; -37 ] / 1000;  % [m]
    params.tof.pos_AL_B = [ -616; -182; -37 ] / 1000;  % [m]
    params.tof.pos_AR_B = [ -616; +182; -37 ] / 1000;  % [m]

    %% Observer KF parameters
    % Heave KF
    % Tof noise? [z?]
    params.observer.heave_KF.R = 4.864859165974720e-05;
    params.observer.heave_KF.R_i = ones([4, 1]) * 4.864859165974720e-03 * 4;
    % comment???  [z, z_dot, a_bias]
    params.observer.heave_KF.Q = diag([ ...
        1e-6, ...   % z
        1e-4, ...   % z_dot
        1e-3  ...   % accel bias
    ]);

    % Velocity KF
    % State: [x_dot, a_bias_x]
    sigma_gps = 0.05; % [m/s]
    params.observer.velocity_KF.R = sigma_gps^2;

    params.observer.velocity_KF.Q = diag([ ...
        1e-6, ...   % x_dot
        1e-7  ...   % accel bias_x
    ]);

    %% Observer Mahony filter parameters
    params.observer.attitude.Kp = 1.2;
    params.observer.attitude.Ki = 0.01;

    params.observer.attitude.acc_norm_tolerance = 0.0625; % [g]

    %% Actuator model parameters
    data = load("..\ACTUATORS-CHARACTERIZATION\hydrofoil_actuator.mat");

    % Continouus-time parameters
    params.actuator_model.T = data.T_opt;
    params.actuator_model.L = data.L_opt;

    % Discrete-time parameters, G(s) = 1 / (T*s + 1), ZOH discretization: 
    params.actuator_model.Td = exp(-params.Ts / params.actuator_model.T); 
    % Delay discretized to number of samples % Rounded up to next 0.01
    params.actuator_model.Ld = ceil(params.actuator_model.L / params.Ts);
    
    params.actuator_model.alpha_min = -6.0;
    params.actuator_model.alpha_max = 12.0;

    %% LQ gain scheduling parameters
    data = load("all_LQ_grid_schedule.mat");
    params.controller.LQ.velocity_grid = data.xWdotGrid_ms;

    % LQR
    params.controller.LQ.Kaug_grid = data.Kaug_grid;

    % LQI
    params.controller.LQ.K_aug_lqi_grid = data.K_aug_lqi_grid;

    % Scheduled trim point
    params.controller.LQ.x0aug_grid = data.x0aug_grid;
    params.controller.LQ.u0_grid    = data.u0_grid;
    
end