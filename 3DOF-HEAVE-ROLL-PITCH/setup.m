% This script, creates all necessary bus objects, 
% opens and initializes the simulik model.

clc; clear; bdclose all; close all;
%% Setup
model_name = "boat_dynamics_model";
    
% Obtain model parameters
params = boat_model_parameters_3dof();   % Returns parameters structure
evalin("base", 'Ts_tof = params.tof.sample_time;');
evalin("base", 'Ts_gyro = params.gyro.sample_time;');
evalin("base", 'Ts_accel = params.accel.sample_time;');

% Create the bus object named boat_model_parameters
busInfo = Simulink.Bus.createObject(params);  
% FIXME why does this command creates a lot of Simulink.bus objects?
boat_model_parameters = eval(busInfo.busName);   % busName is auto-generated

% TODO: Create parameter set like sampling time etc... solver... tsim

% Open the Simulink model
load_system(model_name);
open_system(model_name);

%% See if the model builds
set_param(model_name, "SimulationCommand", "update");
