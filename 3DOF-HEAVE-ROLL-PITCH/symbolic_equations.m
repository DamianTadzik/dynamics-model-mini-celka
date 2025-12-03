clear; clc;
%% Orientation and definition
% World frame (W):
%   x_W is forward
%   y_W is right
%   z_W is downward  (water surface at z_W = 0)

% Body frame (B):
%   Attached to boat, rotates with φ (roll) and θ (pitch)
%   Origin at COM
%   x_B is forward along the boat
%   y_B is to the right according to the boat
%   z_B is downward in boat frame

%% State in world frame
%   z_W - vertical position in the world frame
%   _BW - Body relative to World
%   heave roll pitch 
syms z_W(t) phi_BW(t) theta_BW(t)  
x = [
    z_W; 
    diff(z_W); 
    phi_BW; 
    diff(phi_BW); 
    theta_BW; 
    diff(theta_BW)
    ];


%% Parameters
% Parameters from CAD relative to B frame
syms g  real % 9.81 m/s^2
syms m  real % 4.2 kg
syms Ix_B real % 56894518 g mm^2   % 0.056894518 kg m^2
syms Iy_B real % 1428474943 g mm^2 % 1.428474943 kg m^2

% Hydrofoil placement distances, relative to B frame
syms r_front_B real % +145 mm  % +0.145 m
syms d_front_B real % +216 mm  % +0.216 m
syms d_rear_B  real % -379 mm  % -0.379 m
syms h_front_B real % +178 mm  % +0.178 m
syms h_rear_B  real % +215 mm  % +0.215 m

%% Simple lift model
% F_lift = 0.5 * rho * V^2 * S * Cl(alpha)
% Parameters
syms S_front real % 8700 mm^2 % 8700/1e6 m^2
syms S_rear  real % 8080 mm^2 % 8080/1e6 m^2
syms rho    real % 1000 kg/m^3

% V(t) is one of inputs, as well as alpha_left(t) etc...
%   Velocity is in the W frame
syms V_W(t) % m/s 
%   Hydrofoil AoA can be actuated +12/-6 degrees relative to the hull, B frame
syms alpha_left_B(t)  % [-6, +12] deg 
syms alpha_rear_B(t)  % [-6, +12] deg
syms alpha_right_B(t) % [-6, +12] deg
%   So the AoA coming into the CL CD is hull pitch theta + alpha

% CL(alpha [deg]) CD(alpha [deg]) is loaded from mat as a callable function
% load eppler874.mat
syms CL(alpha) CD(alpha)

%% Thruster/propeller
% Thrust force currently threated as an input.
%   Maybe it can be related to the V_W(t)
%   Or better it could be coupled within the 4th equation - surge
%   It's direction is horizontal
syms F_thrust_B(t) % N

% Thruster placement in B frame
syms d_thrust_B real % -475 mm % 0.475 m
syms h_thrust_B real % 182 mm  % 0.182 m

%% Equations

% Lift forces in a W frame (vertical)
%   Omit the induced inflow, induced AoA... assume that V_W is horizontal.
FL_left = 0.5 * rho * S_front * V_W^2 * CL(alpha_left_B + theta_BW);
FL_rear = 0.5 * rho * S_rear * V_W^2 * CL(alpha_rear_B + theta_BW);
FL_right = 0.5 * rho * S_front * V_W^2 * CL(alpha_right_B + theta_BW);

% Drag forces by analogy (horizontal)
FD_left = 0.5 * rho * S_front * V_W^2 * CD(alpha_left_B + theta_BW);
FD_rear = 0.5 * rho * S_rear * V_W^2 * CD(alpha_rear_B  + theta_BW);
FD_right = 0.5 * rho * S_front * V_W^2 * CD(alpha_right_B + theta_BW);

% Equations in a W frame
eqns = [
    % Translational motion equation - heave
    m * diff(z_W, 2) == m*g - cos(phi_BW) * (FL_left + FL_rear + FL_right);

    % Rotational motion equation - roll
    Ix_B * diff(phi_BW, 2) == r_front_B * FL_left - r_front_B * FL_right;

    % Rotational motion equation - pitch
    Iy_B * diff(theta_BW, 2) == ...
    ... %   torque from front wing lift
    + (d_front_B * cos(theta_BW) + h_front_B * sin(theta_BW)) * (FL_left + FL_right) ...
    ... %   torque from front wings drag
    - (h_front_B * cos(theta_BW) - d_front_B * sin(theta_BW)) * (FD_left + FD_right) ...
    ... %   torque from rear wing lift 
    - (d_rear_B * cos(theta_BW) + h_rear_B * sin(theta_BW)) * FL_rear ...
    ... %   torque from rear wing drag 
    - (h_rear_B * cos(theta_BW) + d_rear_B * sin(theta_BW)) * FD_rear ...
    ... %   torque from thrust 
    + h_thrust_B * F_thrust_B;

    % Translational motion equation - surge ??? mayyybe todo in 4 DOF model
    % thrust - drag == m a maybe uncomment in the future
]


% eqn helpers = [
%     m*z_W = mg - lift forces * roll factor
%     
%     
%     m*x_W = slightly rotated thrust by theta_BW - all drags along x_W
% ]

% TODO: Actuator dynamics should be accounted
% TODO: Measurement noise should be accounted


%% Distance sensors and accelerometer and gyroscope parameters
% sensors positions in a B frame
% F - fore      L - left
% A - achter    R - right
syms R_FL_B [3 1] real % [+225; -182; -37]/1000 m
syms R_FR_B [3 1] real % [+225; +182; -37]/1000 m
syms R_AL_B [3 1] real % [-616; -182; -37]/1000 m
syms R_AR_B [3 1] real % [-616; +182; -37]/1000 m

% Accelerometer and gyroscope placed nearly perfectly at the COM
% Gyro and accel placement -> (0; 0; 0)_B 
