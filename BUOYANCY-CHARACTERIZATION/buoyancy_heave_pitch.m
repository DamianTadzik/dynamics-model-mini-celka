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
    2012 1809 1713 1715 1611 1612 1613 1306 1303 1198 1201  998  895];   % e.g. [12, 87, 203] 2392

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

figure;
surf(pitch_deg, heave_ext, Fb_ext);
xlabel('Pitch [deg]');
ylabel('Heave (COM) [m]');
zlabel('F_b [N]');
title('Buoyancy force vs heave & pitch - extrapolated');
shading interp; grid on; colorbar;
 

