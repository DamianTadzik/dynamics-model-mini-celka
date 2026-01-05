files = dir("dataset/data*.mat");

X = [];
U = [];

for k = 1:numel(files)
    d = load(fullfile(files(k).folder, files(k).name));



    X = [X; d.X];   % stack rows
    U = [U; d.U];
end

% X = X.';   % [N x 8]
% U = U.';   % [N x 3]

U = (U-3) / 9; % Scale to -1 1

%%


%%
actorNet = [
    featureInputLayer(12,"Normalization","none","Name","obs")
    fullyConnectedLayer(128,"Name","fc1")
    reluLayer
    fullyConnectedLayer(64,"Name","fc2")
    reluLayer
    fullyConnectedLayer(3,"Name","fc3")
    tanhLayer

    regressionLayer("Name","reg") % TEMPORARY LAYER
];
opts = trainingOptions("adam", ...
    MaxEpochs = 30, ...
    MiniBatchSize = 256, ...
    Shuffle = "every-epoch", ...
    InitialLearnRate = 1e-3, ...
    Plots = "training-progress", ...
    Verbose = true);

actorNet = trainNetwork(X, U, actorNet, opts);

% Strip regression layer
layers = actorNet.Layers;
layers = layers(1:end-1);   % remove regressionLayer
actorNet = layers;          % RL-compatible actor network
% Save pretrained actor
save("pretrained_actor.mat","actorNet");
