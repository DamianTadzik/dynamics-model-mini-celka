%% Search and plot the grid (pitch, velocities) of trims

clc; clear;

thetaGrid_deg  = -5:0.5:5;
xWdotGrid_ms   = 2:0.1:3;

Ntheta = numel(thetaGrid_deg);
Nv     = numel(xWdotGrid_ms);

trims(Ntheta,Nv) = struct;

[trim_k,~] = find_trim_4dof();

for i = 1:Ntheta
    theta_deg = thetaGrid_deg(i);

    for j = 1:Nv
        xWdot_ms = xWdotGrid_ms(j);

        [trim_k, info_k] = find_trim_4dof( ...
            'xWdot',   xWdot_ms, ...
            'theta',   theta_deg, ...
            'v0',      trim_k.v);   % warm start in actuator space

        trims(i,j).theta_deg = theta_deg;
        trims(i,j).xWdot     = xWdot_ms;
        trims(i,j).trim      = trim_k;
        trims(i,j).info      = info_k;
    end
end

save('trim_grid_theta_xWdot.mat', ...
    'trims', ...
    'thetaGrid_deg', ...
    'xWdotGrid_ms');


%% Extract results

idx = [2 4 8 9]; % surge, heave, roll, pitch accel residuals

xWdot_ms          = nan(Ntheta,Nv);
xWddot_res_ms2    = nan(Ntheta,Nv);

zW_m              = nan(Ntheta,Nv);
zWdot_ms           = nan(Ntheta,Nv);
zWddot_res_ms2     = nan(Ntheta,Nv);

theta_deg          = nan(Ntheta,Nv);

alphaFL_deg        = nan(Ntheta,Nv);
alphaFR_deg        = nan(Ntheta,Nv);
alphaR_deg         = nan(Ntheta,Nv);

resNorm            = nan(Ntheta,Nv);

foilZ_FL_m         = nan(Ntheta,Nv);
foilZ_FR_m         = nan(Ntheta,Nv);
foilZ_R_m          = nan(Ntheta,Nv);

thrust_N           = nan(Ntheta,Nv);

for i = 1:Ntheta
    for j = 1:Nv

        tr = trims(i,j).trim;

        if isempty(tr)
            continue;
        end

        % Kinematics
        xWdot_ms(i,j)       = tr.x0(2);
        zW_m(i,j)           = tr.x0(3);
        zWdot_ms(i,j)       = tr.x0(4);
        theta_deg(i,j)      = rad2deg(tr.x0(6));

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

        % Thrust
        thrust_N(i,j) = tr.v(4);
    end
end


%% Plots

% Surge speed + acceleration residual
figure;

subplot(1,2,1)
imagesc(xWdotGrid_ms,thetaGrid_deg,xWdot_ms);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Trimmed surge speed xWdot [m/s]');

subplot(1,2,2)
imagesc(xWdotGrid_ms,thetaGrid_deg,xWddot_res_ms2);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Surge accel residual xWddot [m/s^2]');


% Heave position + velocity
figure;

subplot(1,2,1)
imagesc(xWdotGrid_ms,thetaGrid_deg,zW_m);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Trimmed heave position zW [m]');

subplot(1,2,2)
imagesc(xWdotGrid_ms,thetaGrid_deg,zWdot_ms);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Heave velocity zWdot [m/s]');


% Heave acceleration residual
figure;

imagesc(xWdotGrid_ms,thetaGrid_deg,zWddot_res_ms2);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Heave accel residual zWddot [m/s^2]');


% Trimmed pitch
figure;

imagesc(xWdotGrid_ms,thetaGrid_deg,theta_deg);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Trimmed pitch \theta [deg]');


% Actuator trims
figure;

subplot(1,3,1)
imagesc(xWdotGrid_ms,thetaGrid_deg,alphaFL_deg);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('\alpha_{FL} [deg]');

subplot(1,3,2)
imagesc(xWdotGrid_ms,thetaGrid_deg,alphaFR_deg);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('\alpha_{FR} [deg]');

subplot(1,3,3)
imagesc(xWdotGrid_ms,thetaGrid_deg,alphaR_deg);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('\alpha_R [deg]');


% Residual norm
figure;

imagesc(xWdotGrid_ms,thetaGrid_deg,resNorm);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('|| trim residual ||');


% Foil immersion
figure;

subplot(1,3,1)
imagesc(xWdotGrid_ms,thetaGrid_deg,foilZ_FL_m);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('FL foil zW [m]');

subplot(1,3,2)
imagesc(xWdotGrid_ms,thetaGrid_deg,foilZ_FR_m);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('FR foil zW [m]');

subplot(1,3,3)
imagesc(xWdotGrid_ms,thetaGrid_deg,foilZ_R_m);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Rear foil zW [m]');


% Thrust
figure;

contourf(xWdotGrid_ms, thetaGrid_deg, thrust_N, ...
    20, 'LineColor','none');

colorbar;
xlabel('xWdot [m/s]');
ylabel('\theta [deg]');
title('Trim thrust [N]');
set(gca,'YDir','normal');
