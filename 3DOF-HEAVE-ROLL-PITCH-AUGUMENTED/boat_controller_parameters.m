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
    % Tof noise? [z?]
    params.observer.heave_KF.R = 4.864859165974720e-05;
    % comment???  [z, z_dot, a_bias]
    params.observer.heave_KF.Q = diag([ ...
        1e-6, ...   % z
        1e-4, ...   % z_dot
        1e-3  ...   % accel bias
    ]);

    %% Observer Mahony filter parameters
    params.observer.attitude.Kp = 1.2;
    params.observer.attitude.Ki = 0.01;

    params.observer.attitude.acc_norm_min = 0.1;
    params.observer.attitude.acc_norm_max = 1.1;

    %% Actuator model parameters
    data = load("..\ACTUATORS-CHARACTERIZATION\hydrofoil_actuator_dynamics.mat");

    params.actuator_model.T = data.T_avg;
    % params.actuator_model.L = data.L_avg; % TO BE DONE FIXME LATER
    params.actuator_model.alpha_min = -6.0;
    params.actuator_model.alpha_max = 12.0;

    %% Other parameters?  LQR gains for schedulling probablly but LATER tODO
end