function params = boat_model_parameters_3dof() %#codegen
% note: _B means that parameter is in the body frame - relative to COM
%
% World frame W: NED (x forward, y right, z down), water surface at z_W = 0
% Body frame B: NED attached to COM
%
    %% Physical constants
    params.m    = 4.2;           % [kg]      boat's mass
    params.g    = 9.81;          % [m/s^2]   gravity 
    params.rho  = 1000;          % [kg/m^3]  water density 
    
    %% Inertias
    % Ix_B = 52245720.20525  g mm^2
    % Iy_B = 471960971.2696  g mm^2
    % Iz_B = 491601070.08109 g mm^2
    params.Ix_B = 0.05224572020525;  % [kg m^2]  roll inertia
    params.Iy_B = 0.4719609712696;   % [kg m^2]  pitch inertia
    params.Iz_B = 0.49160107008109;  % [kg m^2]  yaw inertia

    %% Buoyancy LUT
    data = load("..\BUOYANCY-CHARACTERIZATION\simple_buoyancy_LUT.mat");

    params.buoyancy.LUT.z = data.LUT_z;
    params.buoyancy.LUT.Fb = data.LUT_Fb;
    params.buoyancy.LUT.V = data.LUT_V;

    %% Hydrofoil placement
    params.hydrofoils.pos_front_left_B = [
        +0.216;     % [m] ahead of COM
        -0.145;     % [m] to the right of COM (negative bcs on the left)
        +0.178      % [m] below COM
    ]; 
    params.hydrofoils.pos_rear_B = [
        -0.379;     % [m] ahead of COM (negative because it is behind)
        +0.0;       % [m] to the right of COM
        +0.215      % [m] below COM
    ]; 
    params.hydrofoils.pos_front_right_B = [
        +0.216;     % [m] ahead of COM
        +0.145;     % [m] to the right of COM
        +0.178      % [m] below COM
    ]; 

    %% Hydrofoil area 
    % S_front = 8080 mm^2
    % S_rear  = 8700 mm^2
    params.hydrofoils.S_front = 8080 / 1e6;    % [m^2]  single foil  area
    params.hydrofoils.S_rear  = 8700 / 1e6;    % [m^2]  single foil area

    %% Hydrofoil actuators dynamics
    data = load("..\ACTUATORS-CHARACTERIZATION\hydrofoil_actuator_dynamics.mat");

    params.hydrofoils.dynamics.T = data.T_avg;
    params.hydrofoils.dynamics.L = data.L_avg;
    params.hydrofoils.dynamics.Ts_act = 0.01; % PWM update rate 100 Hz
    params.hydrofoils.dynamics.alpha_max = 12;
    params.hydrofoils.dynamics.alpha_min = -6; 

    %% Thrust placement
    params.thruster.pos_thruster_B = [
        -0.475;    % [m] ahead of COM (negative because it is behind)
        +0.0;      % [m] to the right of COM
        +0.182     % [m] below COM
    ]; 

    %% Lookup table data for CL(alpha) and CD(alpha)
    data = load("../ACTUATORS-CHARACTERIZATION/eppler874.mat");
    params.hydrofoils.LUT.alpha = data.LUT_alpha;
    params.hydrofoils.LUT.CL    = data.LUT_CL;
    params.hydrofoils.LUT.CD    = data.LUT_CD;

    %% Distance sensors placement in a B frame
    params.tof.pos_FL_B = [ +225; -182; -37 ] / 1000;  % [m]
    params.tof.pos_FR_B = [ +225; +182; -37 ] / 1000;  % [m]
    params.tof.pos_AL_B = [ -616; -182; -37 ] / 1000;  % [m]
    params.tof.pos_AR_B = [ -616; +182; -37 ] / 1000;  % [m]

    %% Gyroscope and accelerometer placed at the COM

    %% Distance sensor noise, quantization and sampling time
    % Identified sensors noise characteristics
    data = load("..\SENSORS-CHARACTERIZATION\tof_noise_parameters.mat");
    params.tof.noise_sigma_a = data.tof_noise_parameters.noise_sigma_a;
    params.tof.noise_sigma_b = data.tof_noise_parameters.noise_sigma_b;
    params.tof.quantization_step = data.tof_noise_parameters.quantization_step;
    params.tof.sample_time = 0.1; % [s]
    params.tof.range_mm_min = 0;   % [mm]
    params.tof.range_mm_max = 255; % [mm]

    %% IMU
    data = load("..\SENSORS-CHARACTERIZATION\imu_noise_parameters.mat");

    %% Gyroscope sensor noise, quantization and sampling time
    params.gyro.noise_sigma = data.gyroscope_noise_parameters.noise_sigma;
    params.gyro.quantization_step = data.gyroscope_noise_parameters.quantization_step;
    params.gyro.sample_time = 0.01; % [s]
    params.gyro.range_dps = 2000; % [dps] = [deg/s]

    %% Accelerometer sensor noise, quantization and sampling time
    params.accel.noise_sigma = data.accelerometer_noise_parameters.noise_sigma;
    params.accel.quantization_step = data.accelerometer_noise_parameters.quantization_step;
    params.accel.sample_time = 0.01; % [s]
    params.accel.range_g = 2; % [g]
    
end