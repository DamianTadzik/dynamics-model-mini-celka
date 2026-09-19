%% Analyze scheduled augmented discrete LQR and LQI controllers along velocity

clc;
clear;

%% Load controller schedule

load("all_LQ_grid_schedule.mat", ...
    "controllers", ...
    "Kaug_grid", ...
    "K_aug_lqi_grid", ...
    "x0aug_grid", ...
    "u0_grid", ...
    "rho_lqr_grid", ...
    "rho_lqi_grid", ...
    "xWdotGrid_ms", ...
    "nominal_zW_m");

Nv = numel(xWdotGrid_ms);

%% Select nominal velocity

nominalVelocity_ms = 2.5;

[~, iv0] = min(abs(xWdotGrid_ms - nominalVelocity_ms));
nominalVelocity_ms = xWdotGrid_ms(iv0);

K0_lqr = Kaug_grid(:,:,iv0);
K0_lqi = K_aug_lqi_grid(:,:,iv0);

x0aug0 = x0aug_grid(:,iv0);
u00    = u0_grid(:,iv0);

fprintf("Nominal velocity = %.3f m/s\n", nominalVelocity_ms);
fprintf("Nominal zW       = %.3f m\n", nominal_zW_m);

%% Relative variation of K, x0 and u0

deltaK_lqr_rel = nan(Nv,1);
deltaK_lqi_rel = nan(Nv,1);

deltaX0_rel = nan(Nv,1);
deltaU0_rel = nan(Nv,1);

for j = 1:Nv

    if any(isnan(Kaug_grid(:,:,j)), "all")
        continue;
    end

    K_lqr_j = Kaug_grid(:,:,j);
    K_lqi_j = K_aug_lqi_grid(:,:,j);

    x0j = x0aug_grid(:,j);
    u0j = u0_grid(:,j);

    deltaK_lqr_rel(j) = ...
        norm(K_lqr_j - K0_lqr, "fro") / norm(K0_lqr, "fro");

    deltaK_lqi_rel(j) = ...
        norm(K_lqi_j - K0_lqi, "fro") / norm(K0_lqi, "fro");

    deltaX0_rel(j) = ...
        norm(x0j - x0aug0) / max(norm(x0aug0), eps);

    deltaU0_rel(j) = ...
        norm(u0j - u00) / max(norm(u00), eps);

end

%% 1. Relative variation of LQR and LQI gain matrices

figure;
hold on;
grid on;

plot(xWdotGrid_ms, 100*deltaK_lqr_rel, ...
    "o-", "LineWidth", 1.5);

plot(xWdotGrid_ms, 100*deltaK_lqi_rel, ...
    "s-", "LineWidth", 1.5);

xlabel("Forward velocity $\dot{x}_W$ [m/s]", ...
    "Interpreter", "latex");

ylabel("Relative gain variation [\%]", ...
    "Interpreter", "latex");

legend( ...
    "$K_{\mathrm{aug}}$ (LQR)", ...
    "$K_{\mathrm{aug,LQI}}$", ...
    "Interpreter", "latex", ...
    "Location", "best");

title(sprintf( ...
    "Relative variation of scheduled controller gains from nominal velocity %.1f m/s", ...
    nominalVelocity_ms), ...
    "Interpreter", "latex");

%% 2. Closed-loop spectral radius

figure;
hold on;
grid on;

plot(xWdotGrid_ms, rho_lqr_grid, ...
    "o-", "LineWidth", 1.5);

plot(xWdotGrid_ms, rho_lqi_grid, ...
    "s-", "LineWidth", 1.5);

yline(1, "--");

xlabel("Forward velocity $\dot{x}_W$ [m/s]", ...
    "Interpreter", "latex");

ylabel("$\rho(A_{\mathrm{cl}})$", ...
    "Interpreter", "latex");

legend( ...
    "LQR", ...
    "LQI", ...
    "Location", "best");

title( ...
    "Closed-loop spectral radius of scheduled LQR and LQI controllers", ...
    "Interpreter", "latex");

%% 3. Relative variation of trim state and trim input

figure;
hold on;
grid on;

plot(xWdotGrid_ms, 100*deltaX0_rel, ...
    "o-", "LineWidth", 1.5);

plot(xWdotGrid_ms, 100*deltaU0_rel, ...
    "s-", "LineWidth", 1.5);

xlabel("Forward velocity $\dot{x}_W$ [m/s]", ...
    "Interpreter", "latex");

ylabel("Relative variation [\%]", ...
    "Interpreter", "latex");

legend( ...
    "$x_{0,\mathrm{aug}}$", ...
    "$u_0$", ...
    "Interpreter", "latex", ...
    "Location", "best");

title(sprintf( ...
    "Relative variation of trim quantities from nominal velocity %.1f m/s", ...
    nominalVelocity_ms), ...
    "Interpreter", "latex");

%% 4. Trim actuator commands directly

figure;
hold on;
grid on;

plot(xWdotGrid_ms, u0_grid(1,:), ...
    "o-", "LineWidth", 1.5);

plot(xWdotGrid_ms, u0_grid(2,:), ...
    "o-", "LineWidth", 1.5);

plot(xWdotGrid_ms, u0_grid(3,:), ...
    "o-", "LineWidth", 1.5);

xlabel("Forward velocity $\dot{x}_W$ [m/s]", ...
    "Interpreter", "latex");

ylabel("Trim hydrofoil incidence [deg]", ...
    "Interpreter", "latex");

legend( ...
    "$\alpha_{FL,0}$", ...
    "$\alpha_{FR,0}$", ...
    "$\alpha_{R,0}$", ...
    "Interpreter", "latex", ...
    "Location", "best");

title(sprintf( ...
    "Trim hydrofoil commands at $z_W = %.2f$ m", ...
    nominal_zW_m), ...
    "Interpreter", "latex");

%% 5. LQR closed-loop eigenvalues along velocity

nEig_lqr = numel(controllers(iv0).closed_loop_eigenvalues_lqr);

eigAbs_lqr = nan(Nv, nEig_lqr);

for j = 1:Nv

    if isempty(controllers(j).closed_loop_eigenvalues_lqr)
        continue;
    end

    lambda = controllers(j).closed_loop_eigenvalues_lqr;

    [~, idx] = sort(abs(lambda), "descend");
    lambda = lambda(idx);

    eigAbs_lqr(j,:) = abs(lambda).';

end

figure;
hold on;
grid on;

for k = 1:nEig_lqr
    plot(xWdotGrid_ms, eigAbs_lqr(:,k), ...
        "-", "LineWidth", 1.0);
end

yline(1, "--");

xlabel("Forward velocity $\dot{x}_W$ [m/s]", ...
    "Interpreter", "latex");

ylabel("$|\lambda_i|$", ...
    "Interpreter", "latex");

title( ...
    "Scheduled LQR closed-loop pole magnitudes along forward velocity", ...
    "Interpreter", "latex");

%% 6. LQI closed-loop eigenvalues along velocity

nEig_lqi = numel(controllers(iv0).closed_loop_eigenvalues_lqi);

eigAbs_lqi = nan(Nv, nEig_lqi);

for j = 1:Nv

    if isempty(controllers(j).closed_loop_eigenvalues_lqi)
        continue;
    end

    lambda = controllers(j).closed_loop_eigenvalues_lqi;

    [~, idx] = sort(abs(lambda), "descend");
    lambda = lambda(idx);

    eigAbs_lqi(j,:) = abs(lambda).';

end

figure;
hold on;
grid on;

for k = 1:nEig_lqi
    plot(xWdotGrid_ms, eigAbs_lqi(:,k), ...
        "-", "LineWidth", 1.0);
end

yline(1, "--");

xlabel("Forward velocity $\dot{x}_W$ [m/s]", ...
    "Interpreter", "latex");

ylabel("$|\lambda_i|$", ...
    "Interpreter", "latex");

title( ...
    "Scheduled LQI closed-loop pole magnitudes along forward velocity", ...
    "Interpreter", "latex");

%% 7. Dominant LQR closed-loop poles

nDominant_lqr = min(8, nEig_lqr);

figure;
hold on;
grid on;

for k = 1:nDominant_lqr
    plot(xWdotGrid_ms, eigAbs_lqr(:,k), ...
        "o-", "LineWidth", 1.2);
end

yline(1, "--");

xlabel("Forward velocity $\dot{x}_W$ [m/s]", ...
    "Interpreter", "latex");

ylabel("$|\lambda_i|$", ...
    "Interpreter", "latex");

title( ...
    "Dominant scheduled LQR closed-loop pole magnitudes", ...
    "Interpreter", "latex");

legend( ...
    arrayfun(@(k) sprintf("$\\lambda_%d$",k), ...
    1:nDominant_lqr, ...
    "UniformOutput", false), ...
    "Interpreter", "latex", ...
    "Location", "eastoutside");

%% 8. Dominant LQI closed-loop poles

nDominant_lqi = min(8, nEig_lqi);

figure;
hold on;
grid on;

for k = 1:nDominant_lqi
    plot(xWdotGrid_ms, eigAbs_lqi(:,k), ...
        "o-", "LineWidth", 1.2);
end

yline(1, "--");

xlabel("Forward velocity $\dot{x}_W$ [m/s]", ...
    "Interpreter", "latex");

ylabel("$|\lambda_i|$", ...
    "Interpreter", "latex");

title( ...
    "Dominant scheduled LQI closed-loop pole magnitudes", ...
    "Interpreter", "latex");

legend( ...
    arrayfun(@(k) sprintf("$\\lambda_%d$",k), ...
    1:nDominant_lqi, ...
    "UniformOutput", false), ...
    "Interpreter", "latex", ...
    "Location", "eastoutside");

%% 9. Element-wise range of LQR K

Kaug_range = ...
    max(Kaug_grid, [], 3, "omitnan") - ...
    min(Kaug_grid, [], 3, "omitnan");

figure;

imagesc(Kaug_range);
set(gca, "YDir", "normal");
colorbar;

xlabel("Augmented state index");
ylabel("Control input index");

title( ...
    "Absolute range of scheduled LQR gain matrix $K_{\mathrm{aug}}$", ...
    "Interpreter", "latex");

%% 10. Element-wise range of LQI K

K_aug_lqi_range = ...
    max(K_aug_lqi_grid, [], 3, "omitnan") - ...
    min(K_aug_lqi_grid, [], 3, "omitnan");

figure;

imagesc(K_aug_lqi_range);
set(gca, "YDir", "normal");
colorbar;

xlabel("LQI augmented state index");
ylabel("Control input index");

title( ...
    "Absolute range of scheduled LQI gain matrix $K_{\mathrm{aug,LQI}}$", ...
    "Interpreter", "latex");

%% 11. LQR gain coefficients as function of velocity

[nu, nx_lqr, ~] = size(Kaug_grid);

inputNames = {
    "$u_{FL}$"
    "$u_{FR}$"
    "$u_R$"
};

figure;
tiledlayout(nu,1);

for r = 1:nu

    nexttile;
    hold on;
    grid on;

    for c = 1:nx_lqr

        vals = squeeze(Kaug_grid(r,c,:));

        if max(abs(vals), [], "omitnan") < 1e-10
            continue;
        end

        plot(xWdotGrid_ms, vals, ...
            "-", "LineWidth", 1.0);

    end

    ylabel(inputNames{r}, ...
        "Interpreter", "latex");

    title(sprintf( ...
        "LQR gains contributing to %s", ...
        inputNames{r}), ...
        "Interpreter", "latex");

    if r == nu
        xlabel("Forward velocity $\dot{x}_W$ [m/s]", ...
            "Interpreter", "latex");
    end

end

sgtitle( ...
    "Scheduled LQR gain coefficients as functions of forward velocity", ...
    "Interpreter", "latex");

%% 12. LQI gain coefficients as function of velocity

[~, nx_lqi, ~] = size(K_aug_lqi_grid);

figure;
tiledlayout(nu,1);

for r = 1:nu

    nexttile;
    hold on;
    grid on;

    for c = 1:nx_lqi

        vals = squeeze(K_aug_lqi_grid(r,c,:));

        if max(abs(vals), [], "omitnan") < 1e-10
            continue;
        end

        plot(xWdotGrid_ms, vals, ...
            "-", "LineWidth", 1.0);

    end

    ylabel(inputNames{r}, ...
        "Interpreter", "latex");

    title(sprintf( ...
        "LQI gains contributing to %s", ...
        inputNames{r}), ...
        "Interpreter", "latex");

    if r == nu
        xlabel("Forward velocity $\dot{x}_W$ [m/s]", ...
            "Interpreter", "latex");
    end

end

sgtitle( ...
    "Scheduled LQI gain coefficients as functions of forward velocity", ...
    "Interpreter", "latex");

%% 13. Variation of LQR physical-state gains

% First 9 states:
%
% 1 z
% 2 zdot
% 3 phi
% 4 theta
% 5 p
% 6 q
% 7 delta_FL
% 8 delta_FR
% 9 delta_R

Kphys_lqr = Kaug_grid(:,1:9,:);

stateNames = {
    "$z_W$"
    "$\dot{z}_W$"
    "$\phi$"
    "$\theta$"
    "$p$"
    "$q$"
    "$\delta_{FL}$"
    "$\delta_{FR}$"
    "$\delta_R$"
};

figure;
tiledlayout(3,3);

for c = 1:9

    nexttile;
    hold on;
    grid on;

    for r = 1:nu

        vals = squeeze(Kphys_lqr(r,c,:));

        plot(xWdotGrid_ms, vals, ...
            "o-", "LineWidth", 1.0);

    end

    title(stateNames{c}, ...
        "Interpreter", "latex");

    xlabel("$\dot{x}_W$ [m/s]", ...
        "Interpreter", "latex");

    ylabel("Gain");

end

sgtitle( ...
    "Scheduled LQR gains associated with physical states", ...
    "Interpreter", "latex");

%% 14. Variation of LQI physical-state gains

Kphys_lqi = K_aug_lqi_grid(:,1:9,:);

figure;
tiledlayout(3,3);

for c = 1:9

    nexttile;
    hold on;
    grid on;

    for r = 1:nu

        vals = squeeze(Kphys_lqi(r,c,:));

        plot(xWdotGrid_ms, vals, ...
            "o-", "LineWidth", 1.0);

    end

    title(stateNames{c}, ...
        "Interpreter", "latex");

    xlabel("$\dot{x}_W$ [m/s]", ...
        "Interpreter", "latex");

    ylabel("Gain");

end

sgtitle( ...
    "Scheduled LQI gains associated with physical states", ...
    "Interpreter", "latex");

%% 15. LQI integral gains only

% Last three LQI states:
%
% nx_lqi-2 : integral z
% nx_lqi-1 : integral phi
% nx_lqi   : integral theta

K_integral = K_aug_lqi_grid(:,end-2:end,:);

integralNames = {
    "$\int e_z\,dt$"
    "$\int e_\phi\,dt$"
    "$\int e_\theta\,dt$"
};

figure;
tiledlayout(1,3);

for c = 1:3

    nexttile;
    hold on;
    grid on;

    for r = 1:nu

        vals = squeeze(K_integral(r,c,:));

        plot(xWdotGrid_ms, vals, ...
            "o-", "LineWidth", 1.2);

    end

    xlabel("Forward velocity $\dot{x}_W$ [m/s]", ...
        "Interpreter", "latex");

    ylabel("Integral gain");

    title(integralNames{c}, ...
        "Interpreter", "latex");

end

sgtitle( ...
    "Scheduled LQI integral gains along forward velocity", ...
    "Interpreter", "latex");

%% 16. Summary

fprintf("\n=== LQ GAIN SCHEDULE ANALYSIS ===\n");

fprintf("Velocity range: %.3f ... %.3f m/s\n", ...
    min(xWdotGrid_ms), ...
    max(xWdotGrid_ms));

fprintf("\nLQR:\n");

fprintf("Maximum relative K variation: %.2f %%\n", ...
    100*max(deltaK_lqr_rel, [], "omitnan"));

fprintf("Maximum closed-loop spectral radius: %.6f\n", ...
    max(rho_lqr_grid, [], "omitnan"));

fprintf("Minimum closed-loop spectral radius: %.6f\n", ...
    min(rho_lqr_grid, [], "omitnan"));

fprintf("\nLQI:\n");

fprintf("Maximum relative K variation: %.2f %%\n", ...
    100*max(deltaK_lqi_rel, [], "omitnan"));

fprintf("Maximum closed-loop spectral radius: %.6f\n", ...
    max(rho_lqi_grid, [], "omitnan"));

fprintf("Minimum closed-loop spectral radius: %.6f\n", ...
    min(rho_lqi_grid, [], "omitnan"));

fprintf("\nTrim:\n");

fprintf("Maximum relative x0 variation: %.2f %%\n", ...
    100*max(deltaX0_rel, [], "omitnan"));

fprintf("Maximum relative u0 variation: %.2f %%\n", ...
    100*max(deltaU0_rel, [], "omitnan"));

fprintf("=================================\n");