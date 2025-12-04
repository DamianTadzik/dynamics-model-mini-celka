function params = boat_model_parameters_3dof()
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
    % Ix_B = 56894518 g mm^2
    % Iy_B = 1428474943 g mm^2
    params.Ix_B = 0.056894518;      % [kg m^2]  roll inertia
    params.Iy_B = 1.428474943;      % [kg m^2]  pitch inertia

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

    %% Thrust placement
    params.thruster.pos_thruster_B = [
        -0.475;    % [m] ahead of COM (negative because it is behind)
        +0.0;      % [m] to the right of COM
        +0.182     % [m] below COM
    ]; 

    %% Lookup table data for CL(alpha) and CD(alpha)
    data = load("eppler874.mat");
    params.LUT.alpha = data.LUT_alpha;
    params.LUT.CL    = data.LUT_CL;
    params.LUT.CD    = data.LUT_CD;

    %% Distance sensors placement in a B frame
    params.tof.pos_FL_B = [ +225; -182; -37 ] / 1000;  % [m]
    params.tof.pos_FR_B = [ +225; +182; -37 ] / 1000;  % [m]
    params.tof.pos_AL_B = [ -616; -182; -37 ] / 1000;  % [m]
    params.tof.pos_AR_B = [ -616; +182; -37 ] / 1000;  % [m]

    %% Gyroscope and accelerometer placed at the COM

    %% Distance sensor noise, quantization and sampling time
    % TO BE DONE
    params.tof.max_range = 255;  % [mm]
    params.tof.quantization = 3; % [mm] 3 or 2 dont remember
    params.tof.noise = 0; % to be figured out later
    %% Gyroscope sensor noise, quantization and sampling time
    % TO BE DONE
    % params.gyro.
    %% Accelerometer sensor noise, quantization and sampling time
    % TO BE DONE
    % params.accel. 
end