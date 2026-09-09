%% Search and plot the grid (heave, velocities) of trims

clc; clear;

zWGrid_m     = -0.02:-0.01:-0.18;
xWdotGrid_ms = 2:0.1:3;

Nz = numel(zWGrid_m);
Nv = numel(xWdotGrid_ms);

trims(Nz,Nv) = struct;

[trim_k,~] = find_trim_4dof();

for i = 1:Nz
    zW_m = zWGrid_m(i);

    for j = 1:Nv
        xWdot_ms = xWdotGrid_ms(j);

        [trim_k, info_k] = find_trim_4dof( ...
            'zW',      zW_m, ...
            'xWdot',   xWdot_ms, ...
            'theta',   0, ...
            'v0',      trim_k.v);   % warm start in actuator space

        trims(i,j).zW     = zW_m;
        trims(i,j).xWdot  = xWdot_ms;
        trims(i,j).trim   = trim_k;
        trims(i,j).info   = info_k;
    end
end

save('trim_grid_zW_xWdot.mat', ...
    'trims', ...
    'zWGrid_m', ...
    'xWdotGrid_ms');

%% Extract results

idx = [2 4 8 9]; % surge, heave, roll, pitch accel residuals

xWdot_ms          = nan(Nz,Nv);
xWddot_res_ms2    = nan(Nz,Nv);

zW_m              = nan(Nz,Nv);
zWdot_ms           = nan(Nz,Nv);
zWddot_res_ms2     = nan(Nz,Nv);

theta_deg          = nan(Nz,Nv);

alphaFL_deg        = nan(Nz,Nv);
alphaFR_deg        = nan(Nz,Nv);
alphaR_deg         = nan(Nz,Nv);

resNorm            = nan(Nz,Nv);

foilZ_FL_m         = nan(Nz,Nv);
foilZ_FR_m         = nan(Nz,Nv);
foilZ_R_m          = nan(Nz,Nv);

thrust_N           = nan(Nz,Nv);

for i = 1:Nz
    for j = 1:Nv
        tr = trims(i,j).trim;
        if isempty(tr), continue; end

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
imagesc(xWdotGrid_ms,zWGrid_m,xWdot_ms);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('Trimmed surge speed xWdot [m/s]');

subplot(1,2,2)
imagesc(xWdotGrid_ms,zWGrid_m,xWddot_res_ms2);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('Surge accel residual xWddot [m/s^2]');


% Heave velocity + acceleration residual
figure;
subplot(1,2,1)
imagesc(xWdotGrid_ms,zWGrid_m,zWdot_ms);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('Heave velocity zWdot [m/s]');

subplot(1,2,2)
imagesc(xWdotGrid_ms,zWGrid_m,zWddot_res_ms2);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('Heave accel residual zWddot [m/s^2]');


% Pitch
figure;
imagesc(xWdotGrid_ms,zWGrid_m,theta_deg);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('\theta [deg]');


% Actuator trims
figure;

subplot(1,3,1)
imagesc(xWdotGrid_ms,zWGrid_m,alphaFL_deg);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('\alpha_{FL} [deg]');

subplot(1,3,2)
imagesc(xWdotGrid_ms,zWGrid_m,alphaFR_deg);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('\alpha_{FR} [deg]');

subplot(1,3,3)
imagesc(xWdotGrid_ms,zWGrid_m,alphaR_deg);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('\alpha_R [deg]');


% Residual norm
figure;
imagesc(xWdotGrid_ms,zWGrid_m,resNorm);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('|| trim residual ||');


% Foil immersion
figure;

subplot(1,3,1)
imagesc(xWdotGrid_ms,zWGrid_m,foilZ_FL_m);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('FL foil zW [m]');

subplot(1,3,2)
imagesc(xWdotGrid_ms,zWGrid_m,foilZ_FR_m);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('FR foil zW [m]');

subplot(1,3,3)
imagesc(xWdotGrid_ms,zWGrid_m,foilZ_R_m);
set(gca,'YDir','normal');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('Rear foil zW [m]');


% Thrust
figure;
contourf(xWdotGrid_ms, zWGrid_m, thrust_N, 20, 'LineColor','none');
colorbar;
xlabel('xWdot [m/s]');
ylabel('zW [m]');
title('Trim thrust [N]');
set(gca,'YDir','normal');
