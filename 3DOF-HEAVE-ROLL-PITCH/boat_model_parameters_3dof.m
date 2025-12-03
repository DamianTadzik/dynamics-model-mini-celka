function params = boat_model_parameters_3dof()
    %% Physical constants
    params.g    = 9.81;          % [m/s^2]   gravity 
    params.rho  = 1000;          % [kg/m^3]  water density 
    params.m    = 4.2;           % [kg]      boat's mass

    %% Inertias
    % Ix_B = 56894518 g mm^2
    % Iy_B = 1428474943 g mm^2
    params.Ix_B = 0.056894518;      % [kg m^2]  roll inertia
    params.Iy_B = 1.428474943;      % [kg m^2]  pitch inertia

    %% Hydrofoil placement
    params.r_front_B = 145 / 1e3;   % [m] lateral distance
    params.d_front_B = 216 / 1e3;   % [m] ahead of COM
    params.h_front_B = 178 / 1e3;   % [m] below COM

    params.d_rear_B  = 379 / 1e3;   % [m] behind COM (use +, code handles sign)
    params.h_rear_B  = 215 / 1e3;   % [m] below COM

    %% Hydrofoil area 
    % S_front = 8080 mm^2
    % S_rear  = 8700 mm^2
    params.S_front = 8080 / 1e6;    % [m^2]  single foil  area
    params.S_rear  = 8700 / 1e6;    % [m^2]  single foil area

    %% Thrust placement
    params.d_thrust_B = 475 / 1e3;  % [m] behind COM
    params.h_thrust_B = 182 / 1e3;  % [m] below COM

    %% Lookup table data for CL(alpha) and CD(alpha)
    data = load("eppler874.mat");
    params.LUT_alpha = data.LUT_alpha;
    params.LUT_CL  = data.LUT_CL;
    params.LUT_CD  = data.LUT_CD;

end