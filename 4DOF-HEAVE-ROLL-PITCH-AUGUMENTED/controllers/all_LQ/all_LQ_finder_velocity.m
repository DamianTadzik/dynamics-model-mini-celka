%% Find augmented discrete LQR/LQI controllers along forward velocity
% Other operating conditions are kept at their nominal values.

clc;
clear;

%% Load linearization grid
load("linearization_grid_zW_xWdot.mat", ...
    "linearizations", ...
    "zWGrid_m", ...
    "xWdotGrid_ms");

%% Select nominal heave
nominal_zW_m = -0.10;

[~, iz] = min(abs(zWGrid_m - nominal_zW_m));
nominal_zW_m = zWGrid_m(iz);

Nv = numel(xWdotGrid_ms);

fprintf("Finding LQ gain schedule at zW = %.3f m\n", nominal_zW_m);
fprintf("Number of velocity points: %d\n\n", Nv);

%% Preallocate
controllers = repmat(struct(), Nv, 1);

%% Find controller at every velocity
for j = 1:Nv

    lin = linearizations(iz,j);

    if ~isfield(lin, "A") || isempty(lin.A)
        warning("No linearization for velocity %.3f m/s.", ...
            xWdotGrid_ms(j));
        continue;
    end

    fprintf("Velocity %.3f m/s ... ", xWdotGrid_ms(j));

    ctrl = find_all_LQ( ...
        lin.A, ...
        lin.B, ...
        lin.x0, ...
        lin.u0);

    controllers(j).velocity_ms = xWdotGrid_ms(j);

    %% Common trim data
    controllers(j).x0aug = ctrl.x0aug;
    controllers(j).u0    = ctrl.u0;

    %% LQR
    controllers(j).Kaug = ctrl.Kaug;
    controllers(j).Aaug = ctrl.Aaug;
    controllers(j).Baug = ctrl.Baug;

    Acl_lqr = ctrl.Aaug - ctrl.Baug * ctrl.Kaug;

    controllers(j).closed_loop_eigenvalues_lqr = eig(Acl_lqr);
    controllers(j).spectral_radius_lqr = ...
        max(abs(controllers(j).closed_loop_eigenvalues_lqr));

    %% LQI
    controllers(j).K_aug_lqi = ctrl.K_aug_lqi;
    controllers(j).A_aug_lqi = ctrl.A_aug_lqi;
    controllers(j).B_aug_lqi = ctrl.B_aug_lqi;
    
    Acl_lqi = ctrl.A_aug_lqi - ctrl.B_aug_lqi * ctrl.K_aug_lqi;
    
    controllers(j).closed_loop_eigenvalues_lqi = eig(Acl_lqi);
    controllers(j).spectral_radius_lqi = ...
        max(abs(controllers(j).closed_loop_eigenvalues_lqi));

    fprintf("rho LQR = %.6f, rho LQI = %.6f\n", ...
        controllers(j).spectral_radius_lqr, ...
        controllers(j).spectral_radius_lqi);

end

%% Convert most important quantities to arrays

nx_aug = size(controllers(1).Kaug, 2);
nx_lqi = size(controllers(1).K_aug_lqi, 2);
nu     = size(controllers(1).Kaug, 1);

Kaug_grid = nan(nu, nx_aug, Nv);
K_aug_lqi_grid = nan(nu, nx_lqi, Nv);

x0aug_grid = nan(nx_aug, Nv);
u0_grid    = nan(nu, Nv);

rho_lqr_grid = nan(Nv,1);
rho_lqi_grid = nan(Nv,1);

for j = 1:Nv
    if isempty(controllers(j).Kaug)
        continue;
    end

    Kaug_grid(:,:,j) = controllers(j).Kaug;
    K_aug_lqi_grid(:,:,j) = controllers(j).K_aug_lqi;

    x0aug_grid(:,j) = controllers(j).x0aug;
    u0_grid(:,j)    = controllers(j).u0;

    rho_lqr_grid(j) = controllers(j).spectral_radius_lqr;
    rho_lqi_grid(j) = controllers(j).spectral_radius_lqi;
end

%% Check

fprintf("\n=== GAIN SCHEDULE SUMMARY ===\n");

fprintf("Velocity range: %.3f ... %.3f m/s\n", ...
    min(xWdotGrid_ms), max(xWdotGrid_ms));

fprintf("Maximum LQR closed-loop spectral radius: %.6f\n", ...
    max(rho_lqr_grid, [], "omitnan"));

fprintf("Maximum LQI closed-loop spectral radius: %.6f\n", ...
    max(rho_lqi_grid, [], "omitnan"));

if any(rho_lqr_grid >= 1)
    warning("At least one scheduled LQR controller is unstable.");
else
    fprintf("All scheduled LQR controllers are stable.\n");
end

if any(rho_lqi_grid >= 1)
    warning("At least one scheduled LQI controller is unstable.");
else
    fprintf("All scheduled LQI controllers are stable.\n");
end

%% Save

save("all_LQ_grid_schedule.mat", ...
    "controllers", ...
    "Kaug_grid", ...
    "K_aug_lqi_grid", ...
    "x0aug_grid", ...
    "u0_grid", ...
    "rho_lqr_grid", ...
    "rho_lqi_grid", ...
    "xWdotGrid_ms", ...
    "nominal_zW_m");