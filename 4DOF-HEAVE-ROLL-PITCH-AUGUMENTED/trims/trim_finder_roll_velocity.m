%% Search and plot the grid (rolls, velocities) of trims

clc; clear;

phiGrid_deg = -5:0.5:5; %ok
xWdotGrid_ms = 1.4:0.2:4; %ok

Nt = numel(phiGrid_deg); %ok
Nv = numel(xWdotGrid_ms);  %ok

trims(Nt,Nv) = struct;   % preallocate

[trim_k,~] = find_equilibrium_4dof();

for i = 1:Nt
    phi_deg = phiGrid_deg(i);

    for j = 1:Nv
        xWdot_ms = xWdotGrid_ms(j);

        [trim_k, info_k] = find_equilibrium_4dof( ...
            'phi',    phi_deg, ...
            'xWdot',  xWdot_ms, ...
            'v0',     trim_k.v);   % warm start in actuator space

        trims(i,j).phi = phi_deg;
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
imagesc(xWdotGrid_ms,phiGrid_deg,xWdot_ms);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('Trimmed surge speed xWdot [m/s]');
subplot(1,2,2)
imagesc(xWdotGrid_ms,phiGrid_deg,xWddot_res_ms2);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('Surge accel residual xWddot [m/s^2]');

% Heave position, velocity, acceleration residual
figure;
subplot(1,3,1)
imagesc(xWdotGrid_ms,phiGrid_deg,zW_m);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('Heave position zW [m]');
subplot(1,3,2)
imagesc(xWdotGrid_ms,phiGrid_deg,zWdot_ms);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('Heave velocity zWdot [m/s]');
subplot(1,3,3)
imagesc(xWdotGrid_ms,phiGrid_deg,zWddot_res_ms2);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('Heave accel residual zWddot [m/s^2]');

% Actuator trims
figure;
subplot(1,3,1)
imagesc(xWdotGrid_ms,phiGrid_deg,alphaFL_deg);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('\alpha_{FL} [deg]');
subplot(1,3,2)
imagesc(xWdotGrid_ms,phiGrid_deg,alphaFR_deg);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('\alpha_{FR} [deg]');
subplot(1,3,3)
imagesc(xWdotGrid_ms,phiGrid_deg,alphaR_deg);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('\alpha_{R} [deg]');

% Residual norm
figure;
imagesc(xWdotGrid_ms,phiGrid_deg,resNorm);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('|| trim residual ||');

% Foil immersion
figure;
subplot(1,3,1)
imagesc(xWdotGrid_ms,phiGrid_deg,foilZ_FL_m);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('FL foil zW [m]');
subplot(1,3,2)
imagesc(xWdotGrid_ms,phiGrid_deg,foilZ_FR_m);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('FR foil zW [m]');
subplot(1,3,3)
imagesc(xWdotGrid_ms,phiGrid_deg,foilZ_R_m);
set(gca,'YDir','normal'); colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('Rear foil zW [m]');

% Thrust 
figure;
contourf(xWdotGrid_ms, phiGrid_deg, thrust_N, 20, 'LineColor','none');
colorbar;
xlabel('xWdot [m/s]');
ylabel('\phi [deg]');
title('Trim thrust [N]');
set(gca,'YDir','normal');