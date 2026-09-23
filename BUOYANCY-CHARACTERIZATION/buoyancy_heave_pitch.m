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

%% Remove outliers by row index (fill this in after inspecting the plot above)
bad_idx = [71 82  113 123 144 154 156 225 185 106];   % e.g. [12, 87, 203]

good_idx = true(height(data), 1);
good_idx(bad_idx) = false;
data = data(good_idx, :);

%% Point cloud (cleaned)
figure;
scatter3(rad2deg(data.pitch_rad), data.heave_com_m, data.F_b_N, 20, data.F_b_N, 'filled');
xlabel('Pitch [deg]');
ylabel('Heave (COM) [m]');
zlabel('F_b [N]');
title('Buoyancy force vs heave & pitch - cleaned');
grid on; colorbar;

%% Surface (gridded, from the cleaned data)
pitch_rad = unique(data.pitch_rad);
heave_m   = unique(data.heave_com_m);

nPitch = numel(pitch_rad);
nHeave = numel(heave_m);

% reshape into a grid: rows = heave, cols = pitch (CSV is pitch-outer, heave-inner)
Fb = reshape(data.F_b_N, nHeave, nPitch);

% sort rows so heave is increasing (nicer for surf/plotting)
[heave_m, sortIdx] = sort(heave_m);
Fb = Fb(sortIdx, :);

pitch_deg = rad2deg(pitch_rad);

figure;
surf(pitch_deg, heave_m, Fb);
xlabel('Pitch [deg]');
ylabel('Heave (COM) [m]');
zlabel('F_b [N]');
title('Buoyancy force vs heave & pitch');
shading interp; grid on; colorbar;


