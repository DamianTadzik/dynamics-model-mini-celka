% This script, creates all necessary bus objects, 
% opens and initializes the simulik model.

clc; clear; bdclose all; close all;
%% Setup
model_name = "boat_dynamics_model";
    
% Obtain model parameters
plant_params = boat_model_parameters_3dof();   % Returns plant parameters structure
busInfo_plant = Simulink.Bus.createObject(plant_params);   % Create the bus object
plant_params_bus = eval(busInfo_plant.busName);

ctrl_params = boat_controller_parameters(); % Returns controller parameters structure
busInfo_ctrl = Simulink.Bus.createObject(ctrl_params);   % Create the bus object
ctrl_params_bus = eval(busInfo_ctrl.busName);

% TODO: Create parameter set like sampling time etc... solver... tsim
Ts_sim = 0.001; 

load("trim_3dof_V3_zm0p1.mat")
x0 = trim.x0;
u0 = trim.u0;

% Open the Simulink model
load_system(model_name);
open_system(model_name);
return
%% See if the model builds
set_param(model_name, "SimulationCommand", "update");

%% Helper setup functions
