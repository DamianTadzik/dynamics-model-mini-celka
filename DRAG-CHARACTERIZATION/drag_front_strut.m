
% NACA0043 bottom
% chord 50mm bottom 
% 
% taper 34mm 
% 
% NACA0024 top
% chord 35mm top
% 
% length 120mm constant profile aka thin part 

%% Odzysk danych z onshape
clear all
txt = fileread('onshape_result.txt');
tokens = regexp(txt, ...
    '\[\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s*meter\s*,\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s*meter\s*\]', ...
    'tokens');

data = struct([]);

data(1).thickness = 0.00840105635733219 ;
data(1).chord     = 0.0339478773568956 + 1.05212/1000;
data(1).distance_mm = 0;

for i = 1:numel(tokens)
    data(i+1).thickness = str2double(tokens{i}{1});
    data(i+1).chord     = str2double(tokens{i}{2});
    data(i+1).distance_mm = 2*i - 1;
end
clear txt tokens;
data(end).chord = data(end-1).chord % fix the last bs fragment

%add another known profile
data(end+1).chord = 50/1000;
data(end).thickness = 2*10.75127/1000;
data(end).distance_mm = 40;

% NACA00XX for every section along the profile
for i = 1:numel(data)
    data(i).NACA00 = 100 * data(i).thickness / data(i).chord;
end

% delta dol 0.58048 mm  (end)
% delta gora 1.05212 mm  (1)

const = atan(0.3/0.58048) / data(end).NACA00/100;
const = const + atan(0.3/1.05212) / data(1).NACA00/100;
const = const / 2;

for i = 2:(numel(data)-1)
    delta_mm = 0.3 / (1000 * tan(const * data(i).NACA00/100));
    data(i).delta = delta_mm/10200;
    % i

    data(i).chord_corrected = data(i).chord + data(i).delta;
    data(i).NACA00_corrected = 100 * data(i).thickness / data(i).chord_corrected;
end
data(1).delta = 1.05212/1000;
data(end).delta = 0.58048/1000;

data(1).chord_corrected = data(1).chord;
data(end).chord_corrected = data(end).chord;

data(1).NACA00_corrected = data(1).NACA00;
data(end).NACA00_corrected = data(end).NACA00;

figure

subplot(2,2,1)
plot([data.distance_mm],[data.thickness])

subplot(2,2,2)
plot([data.distance_mm],[data.chord]); hold on;
plot([data.distance_mm],[data.chord_corrected]); hold off

subplot(2,2,3)
plot([data.distance_mm],[data.NACA00]); hold on;
plot([data.distance_mm],[data.NACA00_corrected]); hold off;

subplot(2,2,4)
plot([data.distance_mm],[data.delta]);

%% Print out the (naca, chords) 

for k = 1:numel(data)
    fprintf("(%d, %.4f),\n", round(data(k).NACA00_corrected), data(k).chord_corrected);
end

%% now having _corrected NACA and chord, estimate the quantities for each slice

% Re = Velocity * chord / const 
% gdzie const = 1.0e-6 m^2/s

velocities = 2:0.25:3;
chords = [data.chord_corrected];

Res = zeros(length(velocities), length(chords));
i = 1;
j = 1;
for v = velocities
    for c = chords
        Re = v * c / 1.0e-6;
        Res(i,j) = Re;
        j = j + 1;
    end
    i = i + 1;
    j = 1;
end

figure;
surf(chords, velocities, Res)
hold on

% for k = 1:numel(data)
%     data(k).Re_at_2ms0 = Res(1,k);
%     data(k).Re_at_2ms5 = Res(3,k);
%     data(k).Re_at_3ms0 = Res(5,k);
% end

Re_at_2ms0 = Res(1,:);
Re_at_2ms5 = Res(3,:);
Re_at_3ms0 = Res(5,:);

plot3(chords, 2.0*ones(size(chords)), Re_at_2ms0, ...
      'LineWidth', 3, 'Color', 'r')

plot3(chords, 2.5*ones(size(chords)), Re_at_2ms5, ...
      'LineWidth', 3, 'Color', 'r')

plot3(chords, 3.0*ones(size(chords)), Re_at_3ms0, ...
      'LineWidth', 3, 'Color', 'r')

xlabel('Chord')
ylabel('Velocity [m/s]')
zlabel('Re')


%% XFOIL xddd
xfoil = readtable("xfoil_results.csv");
valid_xfoil = xfoil(xfoil.valid == 1, :);

figure;
plot(valid_xfoil.Re, valid_xfoil.CD, '.')
grid on
xlabel('Re')
ylabel('C_D')

%% Interpolated CD(t/c, Re) surface

F = scatteredInterpolant( ...
    valid_xfoil.thickness_percent, ...
    valid_xfoil.Re, ...
    valid_xfoil.CD, ...
    'natural', ...     % interpolation
    'linear');           % NO extrapolation

t_grid  = linspace(24, 43, 100);
Re_grid = linspace(70000, 200000, 100);

[T, RE] = meshgrid(t_grid, Re_grid);

CD = F(T, RE);
%%
figure;
surf(T, RE, CD);
shading interp;
grid on;

xlabel('Thickness t/c [%]');
ylabel('Re');
zlabel('C_D');
title('Interpolated XFOIL C_D(t/c, Re)');
view(3);

hold on;

scatter3( ...
    valid_xfoil.thickness_percent, ...
    valid_xfoil.Re, ...
    valid_xfoil.CD, ...
    30, 'k', 'filled');

hold off;

%% CD for each pylon strip at nominal speed

V_nominal = 2.7;      % m/s
nu = 1.0e-6;          % m^2/s

for i = 1:numel(data)

    Re = V_nominal * data(i).chord_corrected / nu;

    CD = F( ...
        data(i).NACA00_corrected, ...
        Re);

    data(i).Re_nominal = Re;
    data(i).CD_nominal = CD;
end

figure
subplot(2,1,1)
% hold on;
plot([data.distance_mm], [data.CD_nominal])
subplot(2,1,2)
plot([data.distance_mm], [data.chord_corrected])

%% prepare data for export

export = struct([]);

j = 0;
for k = 1:numel(data)
    k = numel(data) - k + 1
    j = j + 1;
    export(j).CD = data(k).CD_nominal;
    export(j).chord_m = data(k).chord_corrected;
    export(j).distance_m = (2*(j-1)) / 1000;
    export(j).dz = 0.002;
end

k = numel(export)
for j = k:(k+69)
    export(j).CD = export(k).CD; 
    export(j).chord_m = export(k).chord_m;
    export(j).distance_m = ((j-1)*2) / 1000;
    export(j).dz = 0.002;
end

figure
subplot(2,1,1)
% hold on;
plot([export.distance_m], [export.CD])
subplot(2,1,2)
plot([export.distance_m], [export.chord_m])

%%
% save("drag_front_strut.mat", "export")
% clear;
load("drag_front_strut.mat")

%% Tak dla weryfikacji policzmy drag dla pylona w zaleznosci od zanurzenia przy 2.7


z = 0;
vel = 2.7; % m/s

bottom_position_B = [0; 0; 0];

% nargin('strut_dragg')
Fs = [];

for i = 1:200
    z = i/1000;

    [F, tau] = strut_dragg(...
        bottom_position_B, ...
        [export.distance_m], ...
        [export.chord_m], ...
        [export.CD], ...
        0.002, ...
        vel, ...
        z, ...
        eye(3), ...
        eye(3), ...
        1000);

    Fs = [Fs, F];
end

plot(Fs(1,:))

%%
