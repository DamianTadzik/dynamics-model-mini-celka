clear; clc;

load("hydrofoil_actuator.mat");
T0 = T/1000 % [s]
L0 = L/1000 % [s]

path = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_08_26_home\logs_parquet\";
paths = path + ["log11" "log14" "log16"] + ".parquet";
D = parquetread(paths(2));
t = D.timestamp_s - D.timestamp_s(1);
%% save the T0 L0 i Topt Lopt in SI units
% RUN THIS AT THE END
T_prel = T0
L_prel = L0
T_opt = T_best
L_opt = L_best
save("hydrofoil_actuator.mat", "L_prel", "T_prel", "T_opt", "L_opt");
 
%% load the three trajectories
% start times k=1:
% dla front: square 3.5;   triangle 26.5;  sine 50.5
% dla left:  square 75.5;  triangle 98.5;  sine 122.5
% dla rear:  square 147.5; triangle 170.5; sine 194.5
% START k=2
% ymin [1.7 2.5]  ymax [2.7 3.5] 

% t = T.timestamp_s - T.timestamp_s(1);
% u = T.can_signals_AUTO_CONTROL_FRONT_LEFT_SETPOINT;
% y = T.can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW;
% 
% tu = t(~isnan(u) & ~isnan(t)); u = u(~isnan(u) & ~isnan(t)); 
% ty = t(~isnan(y) & ~isnan(t)); y = y(~isnan(y) & ~isnan(t));
% 
% ymin = (y(ty >= 4.4 & ty <= 5.2)); % maps to -6 for left
% ymax = (y(ty >= 5.4 & ty <= 6.2)); % maps to +12 for left
% ymin = mean(ymin);
% ymax = mean(ymax);
% y_deg = -6 + (y - ymin) * (18 / (ymax - ymin));
% 
% figure;
% plot(tu, u); hold on;
% plot(ty, y_deg);
%% Check how T0 L0 is working on the whole set before optimization
% RUN THIS AT THE END XD
figure;
tl = tiledlayout(3,1);
ax = [];
% Simulate the response for all datasets
for n = 1:3
    y_sim = simulate_actuator(validation_data{n}, T0, L0);
    
    rmse = sqrt(mean((validation_data{n}.y-y_sim).^2));
    fit = 100*(1-norm(validation_data{n}.y-y_sim)/norm(validation_data{n}.y-mean(validation_data{n}.y)));

    ax = [ax nexttile];
    plot(validation_data{n}.tu,validation_data{n}.u,'--','DisplayName','Command'); hold on;
    plot(validation_data{n}.ty,validation_data{n}.y,'DisplayName','Measured');
    % plot(validation_data{n}.ty,validation_data{n}.y_filt,'DisplayName','Measured');
    plot(validation_data{n}.ty,y_sim,'DisplayName','Model');
    grid on;
    ylabel('Angle [deg]');
    title(sprintf('%s: RMSE = %.3f deg, Fit = %.1f %%',names{n},rmse,fit));
end
linkaxes(ax,'xy');
xlabel(tl,'Time [s]');
legend('Location','best');
title(tl,sprintf('Common actuator model validation: T = %.2f ms, L = %.2f ms', ...
    T0*1e3,L0*1e3));

%%
u = D.can_signals_AUTO_CONTROL_FRONT_LEFT_SETPOINT;
y = D.can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW;

tu = t(~isnan(u)); u = u(~isnan(u));
ty = t(~isnan(y)); y = y(~isnan(y));

% y_m6  = mean(y(ty >= 4.4 & ty <= 5.2));
% y_p12 = mean(y(ty >= 5.4 & ty <= 6.2));
y_m6  = mean(y(ty >= 1.7 & ty <= 2.5));
y_p12 = mean(y(ty >= 2.7 & ty <= 3.5));
y_deg = -6 + (y-y_m6)*18/(y_p12-y_m6);

% data{1} = cut_trajectory(tu,u,ty,y_deg,26.5,50.0);
% data{1} = cut_trajectory(tu,u,ty,y_deg,36,72);
% data{1} = cut_trajectory(tu,u,ty,y_deg,72,110);
% data{1} = cut_trajectory(tu,u,ty,y_deg,36,110);
data{1} = cut_trajectory(tu,u,ty,y_deg,1,110);
validation_data{1} = cut_trajectory(tu,u,ty,y_deg,1,110);
%%
u = D.can_signals_AUTO_CONTROL_FRONT_RIGHT_SETPOINT;
y = D.can_signals_ACTUATOR_RIGHT_FOIL_FEEDBACK_POSITION_RAW;

tu = t(~isnan(u)); u = u(~isnan(u));
ty = t(~isnan(y)); y = y(~isnan(y));

% y_m6  = mean(y(ty >= 76.5 & ty <= 77.2));   % adjust if needed
% y_p12 = mean(y(ty >= 77.5 & ty <= 78.2));   % adjust if needed
y_m6  = mean(y(ty >= 1.7 & ty <= 2.5));
y_p12 = mean(y(ty >= 2.7 & ty <= 3.5));
y_deg = -6 + (y-y_m6)*18/(y_p12-y_m6);

% data{2} = cut_trajectory(tu,u,ty,y_deg,98.5,122.0);
data{2} = cut_trajectory(tu,u,ty,y_deg,1,110);
validation_data{2} = cut_trajectory(tu,u,ty,y_deg,1,110);
%%
u = D.can_signals_AUTO_CONTROL_REAR_SETPOINT;
y = D.can_signals_ACTUATOR_REAR_FOIL_FEEDBACK_POSITION_RAW;

tu = t(~isnan(u)); u = u(~isnan(u));
ty = t(~isnan(y)); y = y(~isnan(y));

% y_m6  = mean(y(ty >= 148.5 & ty <= 149.2)); % adjust if needed
% y_p12 = mean(y(ty >= 149.5 & ty <= 150.2)); % adjust if needed
y_m6  = mean(y(ty >= 1.7 & ty <= 2.5));
y_p12 = mean(y(ty >= 2.7 & ty <= 3.5));
y_deg = -6 + (y-y_m6)*18/(y_p12-y_m6);

% data{3} = cut_trajectory(tu,u,ty,y_deg,170.5,194.0);
data{3} = cut_trajectory(tu,u,ty,y_deg,1,110);
validation_data{3} = cut_trajectory(tu,u,ty,y_deg,1,110);
%% Fit common T and L to all three triangle responses

T_vec = linspace(max(1e-3,T0-5e-3), T0+10e-3, 100);
L_vec = linspace(max(0,L0-10e-3), L0+16e-3, 100);

J = zeros(numel(T_vec),numel(L_vec));

% resampling
for n = 1:3
    data{n} = resample_trajectory(data{n}, 0.004);
    validation_data{n} = resample_trajectory(validation_data{n}, 0.004);
end
% Filtering
for n = 1:3
    data{n}.y_filt = sgolayfilt(data{n}.y, 2, 21);
end
figure;
plot(data{1}.ty, data{1}.y); hold on;
plot(data{1}.ty, data{1}.y_filt, 'LineWidth', 1.5);
grid on;
legend('Raw','Filtered');


for i = 1:numel(T_vec)
    for j = 1:numel(L_vec)

        err = zeros(1,3);

        for n = 1:3
            y_sim = simulate_actuator(data{n},T_vec(i),L_vec(j));
            err(n) = sqrt(mean((data{n}.y_filt-y_sim).^2));
        end

        J(i,j) = mean(err);     % equal weight for each servo
    end
end

[Jmin,idx] = min(J(:));
[i_best,j_best] = ind2sub(size(J),idx);

T_best = T_vec(i_best);
L_best = L_vec(j_best);

fprintf('Best T = %.3f ms\n',T_best*1e3);
fprintf('Best L = %.3f ms\n',L_best*1e3);
fprintf('Mean RMSE = %.3f deg\n',Jmin);

figure;
imagesc(L_vec*1e3,T_vec*1e3,J);
axis xy; colorbar;
xlabel('L [ms]');
ylabel('T [ms]');
title('Mean RMSE for three actuator validation trajectories');
hold on;
plot(L_best*1e3,T_best*1e3,'rx','MarkerSize',12,'LineWidth',2);

%% plot the fit over the fitted fragment
names = {'LEFT','RIGHT','REAR'};
figure;
tl = tiledlayout(3,1);
ax = [];
for n = 1:3
    y_sim = simulate_actuator(data{n},T_best,L_best);

    rmse = sqrt(mean((data{n}.y-y_sim).^2));
    fit = 100*(1-norm(data{n}.y-y_sim)/norm(data{n}.y-mean(data{n}.y)));

    ax = [ax nexttile];
    plot(data{n}.tu,data{n}.u,'--','DisplayName','Command'); hold on;
    plot(data{n}.ty,data{n}.y,'DisplayName','Measured');
    plot(data{n}.ty,data{n}.y_filt,'DisplayName','filtered');
    plot(data{n}.ty,y_sim,'DisplayName','Model');
    grid on;
    ylabel('Angle [deg]');
    title(sprintf('%s: RMSE = %.3f deg, Fit = %.1f %%',names{n},rmse,fit));
end
linkaxes(ax,'xy');
xlabel(tl,'Time [s]');
legend('Location','best');
title(tl,sprintf('Common actuator model fit: T = %.2f ms, L = %.2f ms', ...
    T_best*1e3,L_best*1e3));

%% Plot the fit over all dataset
figure;
tl = tiledlayout(3,1);
ax = [];
% Simulate the response for all datasets
for n = 1:3
    y_sim = simulate_actuator(validation_data{n}, T_best, L_best);
    
    rmse = sqrt(mean((validation_data{n}.y-y_sim).^2));
    fit = 100*(1-norm(validation_data{n}.y-y_sim)/norm(validation_data{n}.y-mean(validation_data{n}.y)));

    ax = [ax nexttile];
    plot(validation_data{n}.tu,validation_data{n}.u,'--','DisplayName','Command'); hold on;
    plot(validation_data{n}.ty,validation_data{n}.y,'DisplayName','Measured');
    plot(validation_data{n}.ty,y_sim,'DisplayName','Model');
    grid on;
    ylabel('Angle [deg]');
    title(sprintf('%s: RMSE = %.3f deg, Fit = %.1f %%',names{n},rmse,fit));
end
linkaxes(ax,'xy');
xlabel(tl,'Time [s]');
legend('Location','best');
title(tl,sprintf('Common actuator model validation: T = %.2f ms, L = %.2f ms', ...
    T_best*1e3,L_best*1e3));

%%
function delta_dot = actuator(delta, u_delayed, T)
    delta_dot = (u_delayed - delta) / T;
end

function D = cut_trajectory(tu,u,ty,y,t0,t1)
    iu = tu>=t0 & tu<=t1;
    iy = ty>=t0 & ty<=t1;

    D.tu = tu(iu)-t0;
    D.u  = u(iu);
    D.ty = ty(iy)-t0;
    D.y  = y(iy);
end

function y_sim = simulate_actuator(D,T,L)
    u_delayed = interp1(D.tu,D.u,D.ty-L,'previous',D.u(1));

    y_sim = zeros(size(D.ty));
    y_sim(1) = D.y(1);

    for k = 2:numel(D.ty)
        dt = D.ty(k)-D.ty(k-1);
        y_sim(k) = y_sim(k-1) + dt*(u_delayed(k-1)-y_sim(k-1))/T;
    end
end

function D = resample_trajectory(D, Ts)

    t0 = max(D.tu(1), D.ty(1));
    t1 = min(D.tu(end), D.ty(end));

    t = (t0:Ts:t1)';

    % Command is piecewise constant
    u = interp1(D.tu, D.u, t, 'previous', 'extrap');

    % Measurement can be interpolated linearly
    y = interp1(D.ty, D.y, t, 'linear');

    D.t  = t;
    D.tu = t;
    D.ty = t;
    D.u  = u;
    D.y  = y;
end