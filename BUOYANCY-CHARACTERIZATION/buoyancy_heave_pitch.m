clear; clc;
%% Load the 2D buoyancy results (heave x pitch) from the CSV file
data = readtable('buoyancy_results_heave_pitch.csv');

%% Point cloud (raw samples) - click a point to see its row index in the datatip
figure;
scatter3(rad2deg(data.pitch_rad), data.heave_com_m, data.F_b_N, 20, data.F_b_N, 'filled');
xlabel('Pitch [deg]');
ylabel('Heave (COM) [m]');
zlabel('F_b [N]');
title('Buoyancy force vs heave & pitch - raw samples (click points to get index)');
grid on; colorbar;

dcm = datacursormode(gcf);
set(dcm, 'Enable', 'on', 'UpdateFcn', @(~, event) { ...
    sprintf('idx = %d', event.DataIndex), ...
    sprintf('pitch = %.2f deg', rad2deg(data.pitch_rad(event.DataIndex))), ...
    sprintf('heave = %.4f m', data.heave_com_m(event.DataIndex)), ...
    sprintf('F_b = %.2f N', data.F_b_N(event.DataIndex)) ...
});
zlims = zlim();

%% Make a plane fb(heave, pitch) = a*heave + b and get rid of all points that are underneath that plane
% plane goes through (heave=0, Fb=60) and (heave=-0.05, Fb=0), constant in pitch.
% For heave >= 0 the plane is capped flat (no longer rises) and never exceeds Fb=80,
% so it stops cutting away good points at higher heave.
a = (60 - 0) / (0 - (-0.05));   % = 1200
b = 60;
FB_CAP = 80;
plane_fun = @(heave) min(a * min(heave, 0) + b, FB_CAP);

plane_val = plane_fun(data.heave_com_m);
below_plane_idx = data.F_b_N < plane_val;
fprintf('%d points below the reference plane\n', nnz(below_plane_idx));

% visualize the plane, almost completely opaque on the same plot
hold on;
[pp, hh] = meshgrid(linspace(min(rad2deg(data.pitch_rad)), max(rad2deg(data.pitch_rad)), 2), ...
                     linspace(min(data.heave_com_m), max(data.heave_com_m), 50));
surf(pp, hh, repmat(plane_fun(hh(:,1)), 1, 2), 'FaceAlpha', 0.1, 'FaceColor', 'red', 'EdgeColor', 'none');
hold off; zlim(zlims);

%% THEN Remove outliers by row index (fill this in after inspecting the plot above)
bad_idx = [1193 1290 2799 1197 3410 1598 3419 2907 1786 2358 2907 2799 ...
    1786 2188 2092 2292 1290 2289 2288 2287 2395 2290 2187 2090 2913 1304 ...
    2012 1809 1713 1715 1611 1612 1613 1306 1303 1198 1201  998  895 ...
    4996 4981 4969 4997 5011 4964 4965 5141 4983 4968 1101 1097 5454 ...
    5438 5469 5470 5248 ...
    5042 1383 2359 3823 3422 2465];   % e.g. [12, 87, 203] 2392

good_idx = true(height(data), 1);
good_idx(bad_idx) = false;
good_idx = good_idx & ~below_plane_idx;
data = data(good_idx, :);

%% Point cloud (cleaned)
figure;
scatter3(rad2deg(data.pitch_rad), data.heave_com_m, data.F_b_N, 20, data.F_b_N, 'filled');
xlabel('Pitch [deg]');
ylabel('Heave (COM) [m]');
zlabel('F_b [N]');
title('Buoyancy force vs heave & pitch - cleaned');
grid on; colorbar;

%% Point cloud - CoB position relative to COM (one cloud per x/y/z component)
figure;
scatter3(rad2deg(data.pitch_rad), data.heave_com_m, data.CoB_x_B_m, 20, data.CoB_x_B_m, 'filled');
xlabel('Pitch [deg]'); ylabel('Heave (COM) [m]'); zlabel('CoB_x_B [m]');
title('CoB_x (relative to COM) vs heave & pitch'); grid on; colorbar;

figure;
scatter3(rad2deg(data.pitch_rad), data.heave_com_m, data.CoB_y_B_m, 20, data.CoB_y_B_m, 'filled');
xlabel('Pitch [deg]'); ylabel('Heave (COM) [m]'); zlabel('CoB_y_B [m]');
title('CoB_y (relative to COM) vs heave & pitch'); grid on; colorbar;

figure;
scatter3(rad2deg(data.pitch_rad), data.heave_com_m, data.CoB_z_B_m, 20, data.CoB_z_B_m, 'filled');
xlabel('Pitch [deg]'); ylabel('Heave (COM) [m]'); zlabel('CoB_z_B [m]');
title('CoB_z (relative to COM) vs heave & pitch'); grid on; colorbar;

%% Surface (gridded, from the cleaned data)
pitch_rad = unique(data.pitch_rad);
heave_m   = sort(unique(data.heave_com_m));

nPitch = numel(pitch_rad);
nHeave = numel(heave_m);

% Build the grid by matching each row to its (heave, pitch) cell instead of a plain
% reshape - cleaning removed some points, so the grid is no longer perfectly rectangular
% (missing cells are left as NaN).
[~, rowIdx] = ismember(data.heave_com_m, heave_m);
[~, colIdx] = ismember(data.pitch_rad, pitch_rad);
Fb = NaN(nHeave, nPitch);
Fb(sub2ind([nHeave, nPitch], rowIdx, colIdx)) = data.F_b_N;

pitch_deg = rad2deg(pitch_rad);

% Fill the NaN holes (points removed as outliers) by interpolating across the
% surrounding grid, with nearest-neighbor as a fallback for edge cases.
[PP, HH] = meshgrid(pitch_deg, heave_m);
validMask = ~isnan(Fb);
Fb_interpolant = scatteredInterpolant(PP(validMask), HH(validMask), Fb(validMask), 'linear', 'nearest');
Fb(~validMask) = Fb_interpolant(PP(~validMask), HH(~validMask));

figure;
surf(pitch_deg, heave_m, Fb);
hold on;
contour3(pitch_deg, heave_m, Fb, 20, 'k', 'LineWidth', 1);
hold off;
xlabel('Pitch [deg]');
ylabel('Heave (COM) [m]');
zlabel('F_b [N]');
title('Buoyancy force vs heave & pitch');
shading interp; grid on; colorbar;


%% AT THIS POINT DATA IS CLEARED NOW IT IS TIME TO EXTRAPOLATE IT A BIT HEAVE WISE

%% Extrapolate the gridded surface in the heave direction to [-0.6, +0.4] m
% At very negative heave (deeply out of water) F_b -> 0; at very positive heave
% (fully submerged) F_b saturates and holds at its last computed value.
HEAVE_MIN_EXT = -0.6;
HEAVE_MAX_EXT = 0.4;
EPS_HEAVE = 0.005;

heave_ext = [HEAVE_MIN_EXT; heave_m(1) - EPS_HEAVE; heave_m; heave_m(end) + EPS_HEAVE; HEAVE_MAX_EXT];
Fb_ext = [zeros(1, nPitch); zeros(1, nPitch); Fb; Fb(end, :); Fb(end, :)];

% Resample onto a UNIFORM heave/pitch grid: heave_ext above is NOT evenly spaced
% (huge jumps at the extrapolated ends, uneven steps from the raw z_mm sweep), but
% smooth_grid() convolves in grid-INDEX space later, so it needs uniform spacing to
% be physically meaningful. griddedInterpolant is fine with non-uniform SOURCE
% breakpoints as long as they're strictly increasing - only the TARGET needs to be uniform.
heave_ext_raw = heave_ext;
pitch_deg_raw = pitch_deg;
N_HEAVE_U = 300;
heave_u = linspace(HEAVE_MIN_EXT, HEAVE_MAX_EXT, N_HEAVE_U)';
pitch_u = linspace(min(pitch_deg_raw), max(pitch_deg_raw), numel(pitch_deg_raw))';

Fb_interp = griddedInterpolant({heave_ext_raw, pitch_deg_raw}, Fb_ext, 'linear');
Fb_ext = Fb_interp({heave_u, pitch_u});
heave_ext = heave_u;
pitch_deg = pitch_u;

figure;
surf(pitch_deg, heave_ext, Fb_ext);
hold on;
contour3(pitch_deg, heave_ext, Fb_ext, 120, 'k', 'LineWidth', 1);
hold off;
xlabel('Pitch [deg]');
ylabel('Heave (COM) [m]');
zlabel('F_b [N]');
title('Buoyancy force vs heave & pitch - extrapolated');
shading interp; grid on; colorbar;

%% We might want to see the extrapolations of the COB position too in order to validate if its okay...
% Build the same kind of grid + NaN-fill + heave extrapolation as Fb, for each CoB
% component. Unlike Fb (which goes to 0 out of water), CoB position is held constant
% at BOTH ends - once fully out of water the buoyancy is ~0 so the position barely
% matters, and once fully submerged the CoB position saturates just like Fb does.
cob_fields = {"CoB_x_B_m", "CoB_y_B_m", "CoB_z_B_m"};
CoB = struct();
CoB_ext = struct();

for i = 1:numel(cob_fields)
    fname = cob_fields{i};
    vals = data.(fname);

    grid_i = NaN(nHeave, nPitch);
    grid_i(sub2ind([nHeave, nPitch], rowIdx, colIdx)) = vals;

    valid_i = ~isnan(grid_i);
    interp_i = scatteredInterpolant(PP(valid_i), HH(valid_i), grid_i(valid_i), 'linear', 'nearest');
    grid_i(~valid_i) = interp_i(PP(~valid_i), HH(~valid_i));

    CoB.(fname) = grid_i;
    CoB_ext_raw = [grid_i(1, :); grid_i(1, :); grid_i; grid_i(end, :); grid_i(end, :)];

    % Same uniform resampling as Fb_ext, using the same raw (non-uniform) breakpoints
    % and the same target uniform grid (heave_ext/pitch_deg, already resampled above).
    CoB_interp = griddedInterpolant({heave_ext_raw, pitch_deg_raw}, CoB_ext_raw, 'linear');
    CoB_ext.(fname) = CoB_interp({heave_ext, pitch_deg});

    figure;
    surf(pitch_deg, heave_ext, CoB_ext.(fname));
    hold on;
    contour3(pitch_deg, heave_ext, CoB_ext.(fname), 60, 'k', 'LineWidth', 1);
    hold off;
    xlabel('Pitch [deg]');
    ylabel('Heave (COM) [m]');
    zlabel(strrep(fname, '_', '\_'));
    title(sprintf('%s vs heave & pitch - extrapolated', strrep(fname, '_', '\_')));
    shading interp; grid on; colorbar;
end

%% Smooth Fb and CoB to get rid of sharp edges/noise before using them as LUTs
SMOOTH_SIGMA = 1;   % in grid-cell units, bump this up for more smoothing

Fb_smooth = smooth_grid(Fb_ext, SMOOTH_SIGMA);
figure;
surf(pitch_deg, heave_ext, Fb_smooth);
hold on;
contour3(pitch_deg, heave_ext, Fb_smooth, 20, 'k', 'LineWidth', 1);
hold off;
xlabel('Pitch [deg]');
ylabel('Heave (COM) [m]');
zlabel('F_b [N]');
title('Buoyancy force vs heave & pitch - smoothed');
shading interp; grid on; colorbar;

CoB_smooth = struct();
for i = 1:numel(cob_fields)
    fname = cob_fields{i};
    CoB_smooth.(fname) = smooth_grid(CoB_ext.(fname), SMOOTH_SIGMA);

    figure;
    surf(pitch_deg, heave_ext, CoB_smooth.(fname));
    hold on;
    contour3(pitch_deg, heave_ext, CoB_smooth.(fname), 20, 'k', 'LineWidth', 1);
    hold off;
    xlabel('Pitch [deg]');
    ylabel('Heave (COM) [m]');
    zlabel(strrep(fname, '_', '\_'));
    title(sprintf('%s vs heave & pitch - smoothed', strrep(fname, '_', '\_')));
    shading interp; grid on; colorbar;
end

%% Show abs(diff) between smoothed and raw (extrapolated) - to tune SMOOTH_SIGMA
Fb_diff = abs(Fb_smooth - Fb_ext);
figure;
surf(pitch_deg, heave_ext, Fb_diff);
xlabel('Pitch [deg]');
ylabel('Heave (COM) [m]');
zlabel('|F_b_{smooth} - F_b_{raw}| [N]');
title(sprintf('F_b smoothing difference (SMOOTH_SIGMA = %.2f)', SMOOTH_SIGMA));
shading interp; grid on; colorbar;
fprintf('Fb smoothing diff: max=%.3f N, mean=%.3f N\n', max(Fb_diff(:)), mean(Fb_diff(:)));

for i = 1:numel(cob_fields)
    fname = cob_fields{i};
    diff_i = abs(CoB_smooth.(fname) - CoB_ext.(fname));

    figure;
    surf(pitch_deg, heave_ext, diff_i);
    xlabel('Pitch [deg]');
    ylabel('Heave (COM) [m]');
    zlabel(sprintf('|%s_{smooth} - %s_{raw}| [m]', strrep(fname, '_', '\_'), strrep(fname, '_', '\_')));
    title(sprintf('%s smoothing difference (SMOOTH_SIGMA = %.2f)', strrep(fname, '_', '\_'), SMOOTH_SIGMA));
    shading interp; grid on; colorbar;
    fprintf('%s smoothing diff: max=%.4f m, mean=%.4f m\n', fname, max(diff_i(:)), mean(diff_i(:)));
end


%% At the end we need to save our extrapolated data in some way..
% we want to have Fb(heave, theta) and COB_position(heave, pitch) LUTs
%
% Saved as plain breakpoint vectors + value matrices (not scatteredInterpolant/
% griddedInterpolant objects) so they are directly usable with interp2() inside
% a %#codegen function like boat_dynamics_4dof.m, matching the existing style of
% the 1D buoyancy LUT (interp1(LUT_z, LUT_Fb, zW, 'pchip')) in that file.
%
% Breakpoints:
%   LUT_heave_m   [m]   NED, positive = COM underwater - same convention as zW
%   LUT_pitch_rad [rad] same convention as theta_BW (saved in RAD, not deg!)
% Value matrices (size numel(LUT_heave_m) x numel(LUT_pitch_rad)):
%   LUT_Fb        [N] buoyancy force magnitude (acts upward, i.e. -z_B/-z_W)
%   LUT_CoB_x_B, LUT_CoB_y_B, LUT_CoB_z_B [m] - CoB position relative to COM, in the _B frame

LUT_heave_m   = heave_ext;
LUT_pitch_rad = deg2rad(pitch_deg);
LUT_Fb        = Fb_smooth;
LUT_CoB_x_B   = CoB_smooth.CoB_x_B_m;
LUT_CoB_y_B   = CoB_smooth.CoB_y_B_m;
LUT_CoB_z_B   = CoB_smooth.CoB_z_B_m;

save('buoyancy_LUT_2d.mat', 'LUT_heave_m', 'LUT_pitch_rad', ...
    'LUT_Fb', 'LUT_CoB_x_B', 'LUT_CoB_y_B', 'LUT_CoB_z_B');
fprintf('[OK] Saved buoyancy_LUT_2d.mat\n');

% Suggested usage (NOT applied here - paste into boat_model_parameters_4dof.m):
%   data = load("..\BUOYANCY-CHARACTERIZATION\buoyancy_LUT_2d.mat");
%   params.buoyancy.LUT2.heave_m   = data.LUT_heave_m;
%   params.buoyancy.LUT2.pitch_rad = data.LUT_pitch_rad;
%   params.buoyancy.LUT2.Fb        = data.LUT_Fb;
%   params.buoyancy.LUT2.CoB_x_B   = data.LUT_CoB_x_B;
%   params.buoyancy.LUT2.CoB_y_B   = data.LUT_CoB_y_B;
%   params.buoyancy.LUT2.CoB_z_B   = data.LUT_CoB_z_B;
%
% Suggested usage (NOT applied here - paste into boat_dynamics_4dof.m, replacing
% the current 1D FB_up/V_submerged block): apply Fb as a force AT the CoB
% (relative to COM), then get the restoring moment the same way tau_FL_B etc.
% are already computed (cross(r_B, F_B)):
%   Fb_up = interp2(params.buoyancy.LUT2.pitch_rad, params.buoyancy.LUT2.heave_m, ...
%                   params.buoyancy.LUT2.Fb, theta_BW, zW, 'linear');
%   r_CoB_B = [ ...
%       interp2(params.buoyancy.LUT2.pitch_rad, params.buoyancy.LUT2.heave_m, params.buoyancy.LUT2.CoB_x_B, theta_BW, zW, 'linear'); ...
%       interp2(params.buoyancy.LUT2.pitch_rad, params.buoyancy.LUT2.heave_m, params.buoyancy.LUT2.CoB_y_B, theta_BW, zW, 'linear'); ...
%       interp2(params.buoyancy.LUT2.pitch_rad, params.buoyancy.LUT2.heave_m, params.buoyancy.LUT2.CoB_z_B, theta_BW, zW, 'linear') ...
%   ];
%   F_B_B   = R_WB * [0; 0; -Fb_up];      % buoyancy acts upward (-z_W), rotated into _B
%   tau_B_B = cross(r_CoB_B, F_B_B);      % restoring roll/pitch moment from CoB offset
%   % then add F_B_B into F_total_B and tau_B_B into tau_total_B alongside the foil terms

function out = smooth_grid(in, sigma)
    % 2D Gaussian smoothing with edge-replicate padding (no toolbox dependency)
    half = ceil(3 * sigma);
    ax = -half:half;
    [X, Y] = meshgrid(ax, ax);
    kernel = exp(-(X.^2 + Y.^2) / (2 * sigma^2));
    kernel = kernel / sum(kernel(:));

    nRows = size(in, 1);
    nCols = size(in, 2);
    rowIdxPad = [ones(1, half), 1:nRows, nRows * ones(1, half)];
    colIdxPad = [ones(1, half), 1:nCols, nCols * ones(1, half)];
    padded = in(rowIdxPad, colIdxPad);

    out = conv2(padded, kernel, 'valid');
end

