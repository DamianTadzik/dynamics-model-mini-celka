clear; clc;
%% Load the buoyancy results from the CSV file
% data = readtable('buoyancy_results_0_80_05_80_180_5.csv');
data = readtable('buoyancy_results_heave.csv');

% Plot the first column vs the second column
figure;
yyaxis left
plot(data.heave_com_m, data.volume_m3);
xlabel('Heave (m)');
ylabel('Volume (m^3)');
title('Buoyancy Results: Heave vs Volume');
grid on;

% Plot buoyancy force vs height on the right axis
hold on;
yyaxis right
plot(data.heave_com_m, data.F_b_N);
ylabel('Buoyancy Force (N)');
legend('Volume', 'Buoyancy Force');
grid on;
%% Preprocess
z = data.heave_com_m;
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
    data.heave_com_m, data.volume_m3, '*'); grid on;
ylabel('V [m^3]');

yyaxis right;
plot(z, Fb, 'o-', ...
    data.heave_com_m, data.F_b_N, '*'); grid on;
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

save simple_buoyancy_LUT2.mat LUT_z LUT_Fb LUT_V
return

%% Compare the original with new script...

old = load("D:\Dane\workspace\dynamics-model-mini-celka\BUOYANCY-CHARACTERIZATION\simple_buoyancy_LUT.mat")
new = load("D:\Dane\workspace\dynamics-model-mini-celka\BUOYANCY-CHARACTERIZATION\simple_buoyancy_LUT2.mat")

figure;
plot(old.LUT_z, old.LUT_Fb); grid on; hold on;
plot(new.LUT_z, new.LUT_Fb);
