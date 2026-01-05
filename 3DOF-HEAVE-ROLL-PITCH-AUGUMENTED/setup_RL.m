
% x_hat is estimated state vector, estimated by observer
% Observation specification
obsInfo = rlNumericSpec([12 1]);
obsInfo.Name = "observations";

% Action spec
actInfo = rlNumericSpec([3 1], ...
    LowerLimit = [-1; -1; -1], ...
    UpperLimit = [ 1;  1;  1]);
actInfo.Name = "actions";

% Create environment
env = rlSimulinkEnv( ...
    "boat_dynamics_model", ...
    "boat_dynamics_model/RL Agent", ...
    obsInfo, actInfo);
env.ResetFcn = @resetFcn;

%% Actor network 
actorNet = [
    featureInputLayer(12,"Normalization","none","Name","obs")
    fullyConnectedLayer(64,"Name","fc1")
    reluLayer
    fullyConnectedLayer(32,"Name","fc2")
    reluLayer
    fullyConnectedLayer(3,"Name","fc3")
    tanhLayer
];

% actorNet = load("pretrained_actor.mat", "actorNet");
% actorNet = actorNet.actorNet;
actor = rlContinuousDeterministicActor( ...
    actorNet, obsInfo, actInfo);

%% Critic network 
statePath = [
    featureInputLayer(12,"Normalization","none","Name","obs")
    fullyConnectedLayer(64,"Name","fc1")
    reluLayer
];

actionPath = [
    featureInputLayer(3,"Normalization","none","Name","act")
];

commonPath = [
    concatenationLayer(1,2,"Name","concat")
    fullyConnectedLayer(64,"Name","fc2")
    reluLayer
    fullyConnectedLayer(1,"Name","Q")
];

criticNet = layerGraph(statePath);
criticNet = addLayers(criticNet, actionPath);
criticNet = addLayers(criticNet, commonPath);
criticNet = connectLayers(criticNet,"relu","concat/in1");
criticNet = connectLayers(criticNet,"act","concat/in2");

critic = rlQValueFunction(criticNet, obsInfo, actInfo);

%% DDPG agent options 
agentOpts = rlDDPGAgentOptions( ...
    SampleTime = 0.01, ...
    TargetSmoothFactor = 1e-3, ...
    DiscountFactor = 0.995, ... 0.995 originally 
    MiniBatchSize = 256, ...
    ExperienceBufferLength = 1e6);

agentOpts.NoiseOptions.Variance = 0.05;
agentOpts.NoiseOptions.VarianceDecayRate = 1e-5;

agentObj = rlDDPGAgent(actor, critic, agentOpts);
    
%% Training

trainOpts = rlTrainingOptions( ...
    MaxEpisodes = 1000, ...
    MaxStepsPerEpisode = 1000, ... % 1000 * 0.01s = 10s
    StopTrainingCriteria = "None");
    
trainingStats = train(agentObj, env, trainOpts);

save("XXXAGENT3.mat", "agentObj", "env", "trainOpts", "trainingStats");
return 
%% Resume training
trainingStats = train(agentObj, env, trainOpts);

%% Reset function
% Attach reset function
env.ResetFcn = @resetFcn;
function in = resetFcn(in)
    persistent trim
    if isempty(trim)
        data = load('trim_3dof_V3_zm0p1.mat','trim');
        trim = data.trim;
    end

    % Base state
    x0 = trim.x0;

    % Small perturbations
    x0(1) = x0(1) + 0.01*randn;            % z [m]
    % x0(2) = 0.05*randn;                    % z_dot [m/s]
    x0(3) = x0(3) + deg2rad(4)*randn;      % phi [rad]
    x0(4) = x0(4) + deg2rad(1)*randn;      % theta [rad]

    % Actuator states
    x0(9:11) = x0(9:11);% + 0.1*randn(3,1);  % [deg]

    % Set initial state
    in = setVariable(in,'x0',x0);

    % Inputs
    u0 = trim.u0;
    u0(1:3) = x0(9:11);                    % consistent actuators
    in = setVariable(in,'u0',u0);


    % %% Save workspace recorded variables as datafiles
    % datasetDir = "dataset";
    % out = evalin('base','out');
    % persistent k
    % if isempty(k)
    %     k=0;
    % end
    % k = k + 1;
    % 
    % X = out.obs.signals.values;   % [N x nsig]
    % U = out.act.signals.values;   % [N x 3]
    % t = out.obs.time;             % [N x 1]
    % save(fullfile(datasetDir,sprintf('data%d.mat',k)),'X', 'U', 't');
end
