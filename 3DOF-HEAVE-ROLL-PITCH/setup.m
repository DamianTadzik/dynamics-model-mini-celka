% This script, creates all necessary bus objects, 
% opens and initializes the simulik model.

clc; clear; bdclose all; close all;
%% Setup
model_name = "boat_dynamics_model";

% Obtain model parameters
params = boat_model_parameters_3dof();   % Returns parameters structure

% Create the bus object named boat_model_parameters
busInfo = Simulink.Bus.createObject(params);  
boat_model_parameters = eval(busInfo.busName);   % busName is auto-generated
assignin("base","boat_model_parameters", boat_model_parameters);
assignin("base","params", params);              % also export params itself

% TODO: Create parameter set like sampling time etc... solver... tsim

% Open the Simulink model
load_system(model_name);
open_system(model_name);

% See if the model builds
set_param(model_name, "SimulationCommand", "update");
