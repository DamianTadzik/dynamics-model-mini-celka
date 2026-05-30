clc; clear;

[trim_k, ~] = find_equilibrium_4dof();

thetaGrid_deg = -5:0.05:3;
trims(numel(thetaGrid_deg)) = struct; % preallocate

% % alphaSeeds = [-6 -4 -2 -1 0 1 2 4 6];   % deg, adjust if needed

cost = @(v) sum( ...
    trim_residual_4dof(v, params, zW0, phi0, theta0, Velocity0 ).^2 );


for k = 1:numel(thetaGrid_deg)
    theta_deg = thetaGrid_deg(k);
    % % % TRYING TO FORCE TO GO TO THE GOOD VALLEY 
    % % trim_k.v(1:2) = trim_k.v(1:2) - .5; NOT SOLVING THE PROBLEM 

    % % bestRes  = inf;
    % % bestTrim = [];
    % % bestInfo = [];
    % % 
    % % for a = alphaSeeds
    % %     v_try = trim_k.v;      % continuation
    % %     v_try(1) = a;          % α_FL seed
    % %     v_try(2) = a;          % α_FR seed
    % % 
    % %     [trim_tmp, info_tmp] = find_equilibrium_4dof( ...
    % %         'theta', theta_deg, ...
    % %         'v0',    v_try);
    % % 
    % %     % residual norm (you already store xdot)
    % %     res = norm(trim_tmp.xdot([2 4 8 9]));
    % % 
    % %     if res < bestRes
    % %         bestRes  = res;
    % %         bestTrim = trim_tmp;
    % %         bestInfo = info_tmp;
    % %     end
    % % end
    % % % accept best branch
    % % trim_k = bestTrim;
    % % 
    % % trims(k).theta = theta_deg;
    % % trims(k).trim  = trim_k;
    % % trims(k).info  = bestInfo;
    % % trims(k).res   = bestRes;

    [trim_k,info_k] = find_equilibrium_4dof( ...
        'theta', theta_deg, ...
        'v0',    trim_k.v);

    trims(k).theta = theta_deg;
    trims(k).trim  = trim_k;
    trims(k).info  = info_k;
end
%%
% % clc; clear;
% % 
% % % First trim (warm start)
% % [trim_k, info_k] = find_equilibrium_4dof();
% % 
% % thetaGrid_deg = -5:0.05:3;
% % trims(numel(thetaGrid_deg)) = struct; % preallocate
% % 
% % % Local-success threshold (tune)
% % resThresh = 1e-2;
% % 
% % % PSO settings
% % optsPS = optimoptions('particleswarm', ...
% %     'SwarmSize', 40, ...
% %     'MaxIterations', 60, ...
% %     'Display','on');
% % 
% % for k = 1:numel(thetaGrid_deg)
% %     theta_deg = thetaGrid_deg(k);
% % 
% %     % --- 1) Try local solve first (warm-start) ---
% %     [trim_try, info_try] = find_equilibrium_4dof( ...
% %         'theta', theta_deg, ...
% %         'v0',    trim_k.v);
% % 
% %     res_try = norm(trim_try.xdot([2 4 8 9]), 2);
% % 
% %     if res_try < resThresh
% %         trim_k = trim_try;
% %         info_k = info_try;
% %     else
% %         fprintf('  -> local failed at theta=%.2f deg (res=%.3e), running PSO...\n', ...
% %             theta_deg, res_try);
% % 
% %         % --- 2) Build PSO cost *only from find_equilibrium_4dof inputs* ---
% %         % Use the same defaults as find_equilibrium_4dof, except theta.
% %         params   = boat_model_parameters_4dof();
% %         zW0      = -0.10;         % must match your find_equilibrium defaults (cfg.zW)
% %         phi0     = 0;             % deg
% %         Velocity0= 2.0;           % must match cfg.xWdot
% %         phi0_rad = deg2rad(phi0);
% %         th_rad   = deg2rad(theta_deg);
% % 
% %         lb = [-6;-6;-6;0];
% %         ub = [12;12;12;10];
% % 
% %         cost = @(v) sum( trim_residual_4dof(v, params, zW0, phi0_rad, th_rad, Velocity0 ).^2 );
% % 
% %         % --- 3) Run PSO to get a good basin ---
% %         [v_ps, J_ps] = particleswarm(cost, 4, lb, ub, optsPS);
% % 
% %         % --- 4) Refine with local solve from PSO seed ---
% %         [trim_k, info_k] = find_equilibrium_4dof( ...
% %             'theta', theta_deg, ...
% %             'v0',    v_ps);
% %     end
% % 
% %     trims(k).theta = theta_deg;
% %     trims(k).trim  = trim_k;
% %     trims(k).info  = info_k;
% %     trims(k).res   = norm(trim_k.xdot([2 4 8 9]), 2);
% % end


%% Plots 1D search

% Stack trim results
x0_mat   = cell2mat(arrayfun(@(s) s.trim.x0,   trims, 'UniformOutput', false));
xdot_mat = cell2mat(arrayfun(@(s) s.trim.xdot, trims, 'UniformOutput', false));
info_mat = cell2mat(arrayfun(@(s) s.info(:),   trims, 'UniformOutput', false));
thrust_N = arrayfun(@(s) s.trim.v(4), trims);

% --- Physical vectors (units encoded) ---
xWdot_ms        = x0_mat(2,:);
zW_m            = x0_mat(3,:);

alphaFL_deg     = x0_mat(11,:);
alphaFR_deg     = x0_mat(12,:);
alphaR_deg      = x0_mat(13,:);

xWddot_res_ms2  = xdot_mat(2,:);
zWddot_res_ms2  = xdot_mat(4,:);
pDot_res_rad2   = xdot_mat(8,:);
qDot_res_rad2   = xdot_mat(9,:);

foilZ_FL_m      = info_mat(1,:);
foilZ_FR_m      = info_mat(2,:);
foilZ_R_m       = info_mat(3,:);

% Residual norm
resIdx  = [2 4 8 9]; % surge, heave, roll, pitch accels
resNorm = vecnorm(xdot_mat(resIdx,:),2,1);


% Actual Plots

% Theta vs surge speed
figure;
plot(thetaGrid_deg, xWdot_ms, '-o');
xlabel('\theta [deg]');
ylabel('xWdot [m/s]');
grid on;

% Theta vs heave position
figure;
plot(thetaGrid_deg, zW_m, '-o');
xlabel('\theta [deg]');
ylabel('zW [m]');
grid on;

% Theta vs actuator trims
figure; hold on; grid on;
plot(thetaGrid_deg, alphaFL_deg, '-o');
plot(thetaGrid_deg, alphaFR_deg, '-o');
plot(thetaGrid_deg, alphaR_deg,  '-o');
xlabel('\theta [deg]');
ylabel('\alpha [deg]');
legend('FL','FR','R');

% Theta vs residual components
figure; grid on;
plot(thetaGrid_deg, ...
     [xWddot_res_ms2;
      zWddot_res_ms2;
      pDot_res_rad2;
      qDot_res_rad2].');
xlabel('\theta [deg]');
ylabel('Residual');
legend('xWddot','zWddot','pDot','qDot');

% Convergence plot
figure;
plot(thetaGrid_deg, resNorm, '-o');
xlabel('\theta [deg]');
ylabel('|| trim residual ||');
grid on;

% Foil immersion vs pitch
figure; hold on; grid on;
plot(thetaGrid_deg, foilZ_FL_m, '-o');
plot(thetaGrid_deg, foilZ_FR_m, '-o');
plot(thetaGrid_deg, foilZ_R_m,  '-o');
xlabel('\theta [deg]');
ylabel('Foil vertical position zW [m]');
legend('Front Left','Front Right','Rear');
title('Foil immersion vs pitch trim');

% Thrust vs pitch
figure;
plot(thetaGrid_deg, thrust_N, '-o');
xlabel('\theta [deg]');
ylabel('Thrust [N]');
grid on;
title('Trim thrust vs pitch');


%% Search and plot the grid (pitches, velocities) of trims

clc; clear;

thetaGrid_deg = -5:0.1:3; %ok
xWdotGrid_ms = 1.4:0.2:4; %ok

Nt = numel(thetaGrid_deg); %ok
Nv = numel(xWdotGrid_ms);  %ok

trims(Nt,Nv) = struct;   % preallocate

[trim_k,~] = find_equilibrium_4dof();

for i = 1:Nt
    theta_deg = thetaGrid_deg(i);

    for j = 1:Nv
        xWdot_ms = xWdotGrid_ms(j);

        [trim_k, info_k] = find_equilibrium_4dof( ...
            'theta',  theta_deg, ...
            'xWdot',  xWdot_ms, ...
            'v0',     trim_k.v);   % warm start in actuator space

        trims(i,j).theta = theta_deg;
        trims(i,j).xWdot = xWdot_ms;
        trims(i,j).trim  = trim_k;
        trims(i,j).info  = info_k;
    end
end

%% Plots 2D search

idx = [2 4 8 9]; % surge, heave, roll, pitch accels - residuals

xWdot_ms          = nan(Nt,Nv);   % surge speed
xWddot_res_ms2    = nan(Nt,Nv);   % surge accel residual

zW_m              = nan(Nt,Nv);   % heave position
zWdot_ms          = nan(Nt,Nv);   % heave velocity
zWddot_res_ms2    = nan(Nt,Nv);   % heave accel residual

alphaFL_deg       = nan(Nt,Nv);
alphaFR_deg       = nan(Nt,Nv);
alphaR_deg        = nan(Nt,Nv);

resNorm           = nan(Nt,Nv);

foilZ_FL_m        = nan(Nt,Nv);
foilZ_FR_m        = nan(Nt,Nv);
foilZ_R_m         = nan(Nt,Nv);

thrust_N          = nan(Nt,Nv);   % Thrust [N]

for i = 1:Nt
    for j = 1:Nv
        tr = trims(i,j).trim;
        if isempty(tr), continue; end

        % Kinematics
        xWdot_ms(i,j)       = tr.x0(2);
        zW_m(i,j)           = tr.x0(3);
        zWdot_ms(i,j)       = tr.x0(4);

        % Residual accelerations
        xWddot_res_ms2(i,j) = tr.xdot(2);
        zWddot_res_ms2(i,j) = tr.xdot(4);

        % Actuators
        alphaFL_deg(i,j)    = tr.x0(11);
        alphaFR_deg(i,j)    = tr.x0(12);
        alphaR_deg(i,j)     = tr.x0(13);

        % Residual norm
        resNorm(i,j) = norm(tr.xdot(idx),2);

        % Foil immersion
        info = trims(i,j).info;
        foilZ_FL_m(i,j) = info(1);
        foilZ_FR_m(i,j) = info(2);
        foilZ_R_m(i,j)  = info(3);

        % Thrust calculations
        thrust_N(i,j) = tr.v(4);   % Thrust [N]
    end
end


% Actual Plots

% Surge speed + acceleration residual
figure;
subplot(1,2,1)
imagesc(xWdotGrid_ms,thetaGrid_deg,xWdot_ms);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Trimmed surge speed xWdot [m/s]');
subplot(1,2,2)
imagesc(xWdotGrid_ms,thetaGrid_deg,xWddot_res_ms2);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Surge accel residual xWddot [m/s^2]');

% Heave position, velocity, acceleration residual
figure;
subplot(1,3,1)
imagesc(xWdotGrid_ms,thetaGrid_deg,zW_m);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Heave position zW [m]');
subplot(1,3,2)
imagesc(xWdotGrid_ms,thetaGrid_deg,zWdot_ms);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Heave velocity zWdot [m/s]');
subplot(1,3,3)
imagesc(xWdotGrid_ms,thetaGrid_deg,zWddot_res_ms2);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Heave accel residual zWddot [m/s^2]');

% Actuator trims
figure;
subplot(1,3,1)
imagesc(xWdotGrid_ms,thetaGrid_deg,alphaFL_deg);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('\alpha_{FL} [deg]');
subplot(1,3,2)
imagesc(xWdotGrid_ms,thetaGrid_deg,alphaFR_deg);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('\alpha_{FR} [deg]');
subplot(1,3,3)
imagesc(xWdotGrid_ms,thetaGrid_deg,alphaR_deg);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('\alpha_{R} [deg]');

% Residual norm
figure;
imagesc(xWdotGrid_ms,thetaGrid_deg,resNorm);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('|| trim residual ||');

% Foil immersion
figure;
subplot(1,3,1)
imagesc(xWdotGrid_ms,thetaGrid_deg,foilZ_FL_m);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('FL foil zW [m]');
subplot(1,3,2)
imagesc(xWdotGrid_ms,thetaGrid_deg,foilZ_FR_m);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('FR foil zW [m]');
subplot(1,3,3)
imagesc(xWdotGrid_ms,thetaGrid_deg,foilZ_R_m);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Rear foil zW [m]');

% Thrust 
figure;
contourf(xWdotGrid_ms, thetaGrid_deg, thrust_N, 20, 'LineColor','none');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Trim thrust [N]');
set(gca,'YDir','normal');