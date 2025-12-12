clear; clc;
%% Load the buoyancy results from the CSV file
data = readtable('buoyancy_results_0_80_05_80_180_5.csv');

% Plot the first column vs the second column
figure;
yyaxis left
plot(data.z_mm, data.volume_m3);
xlabel('Height (mm)');
ylabel('Volume (m^3)');
title('Buoyancy Results: Height vs Volume');
grid on;

% Plot buoyancy force vs height on the right axis
hold on;
yyaxis right
plot(data.z_mm, data.F_b_N);
ylabel('Buoyancy Force (N)');
legend('Volume', 'Buoyancy Force');
grid on;
%% Preprocess
z = (33 - data.z_mm) / 1000;     % convert to meters
Fb = data.F_b_N;          % buoyant force
V = data.volume_m3;

% manually mark bad points (indices) 
bad_idx = [1, 32, 50, 85, 117, 118, 119, 120, 121, 123, 124, 125, ... 
    134, 135, 137, 138, 157, 158, 160, 161];

% apply mask 
good_idx = true(size(Fb));
good_idx(bad_idx) = false;

z = z(good_idx);
Fb = Fb(good_idx);
V = V(good_idx);

%% Extend the z, V and Fb data
z  = [ 1.0;  z(1)+0.005; z; z(end)-0.005;  -1.0 ];
Fb = [ Fb(1); Fb(1); Fb; 0; 0 ];
V  = [ V(1);  V(1); V; 0; 0 ];

% visualize 
figure; 
xlabel('z [m]'); 
title('Cleaned buoyancy data');

yyaxis left;
plot(z, V, 'o-', ...
    (33-data.z_mm)/1000, data.volume_m3, '*'); grid on;
ylabel('V [m^3]');

yyaxis right;
plot(z, Fb, 'o-', ...
    (33-data.z_mm)/1000, data.F_b_N, '*'); grid on;
ylabel('F_b [N]');

diff_z = diff(z);
figure;
plot(diff(z));
assert(all(diff_z < 0))

%% Fit function to data
% % Fit a smooth function 
% % Option 1: polynomial fit
% p = polyfit(z, Fb, 3);     % 3rd-order polynomial
% Fb_fit = polyval(p, z);

% Option 2 (better for nonlinearity): interpolation
V_fun = @(zz) interp1(z, V, zz, 'pchip' );
Fb_fun = @(zz) interp1(z, Fb, zz, 'pchip');

zz = -1:0.001:1;

% Plot 
figure; 
xlabel('Heave z [m]');
title('Buoyancy vs Heave');
grid on;
legend;

yyaxis right; 
hold on;
% plot(z, V, 'o', 'DisplayName', 'data');
plot(zz, V_fun(zz), '-', 'DisplayName', 'interp1');
ylabel('Submerged volume [m^3]');
hold off; legend;

yyaxis left; 
hold on;
% plot(z, Fb, 'o', 'DisplayName', 'data');
plot(zz, Fb_fun(zz), '-', 'DisplayName', 'interp1');
ylabel('Buoyancy force [N]');
hold off; legend;


%% Save the LUT data 

LUT_z = zz;
LUT_Fb =  Fb_fun(LUT_z);
LUT_V =  V_fun(LUT_z);

save simple_buoyancy_LUT.mat LUT_z LUT_Fb LUT_V
return
%% Interpolant creation
% Create griddedInterpolant with input z and output Fb from the data
V_interpolant = griddedInterpolant(z, V, 'pchip', 'nearest');
Fb_interpolant = griddedInterpolant(z, Fb, 'pchip', 'none');

% Save the interpolant to a .mat file for later use
% This file can be loaded in a different script using the command:
% load('Fb_interpolant.mat'); 
% After loading, you can use the variable Fb_interpolant to interpolate buoyancy force values.
% Usage example:
% Fb_value = Fb_interpolant(z_value); % where z_value is in meters
save('V_interpolant.mat', 'V_interpolant');
save('Fb_interpolant.mat', 'Fb_interpolant');

% Compute a dense linspace for z values
z_dense = linspace(min(z), max(z), 1000); 

% Apply the interpolant to the dense z values
Fb_interpolated = Fb_interpolant(z_dense);

% Plot the comparison between original data and interpolated output
figure; hold on; grid on;
plot(z, Fb, 'o', 'DisplayName', 'Original Data');
plot(z_dense, Fb_interpolated, '-', 'DisplayName', 'Interpolated Output');
xlabel('Heave z [m]');
ylabel('Buoyancy force [N]');
legend;
title('Comparison of Original Data and Interpolated Output');

%% 