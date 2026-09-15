clc; clear; close all;

% Load CSV (assumes file is in current folder)
T = readtable('xf-e874-il-100000-n5.csv','FileType','text','Delimiter',',','ReadVariableNames',false);
% If file has header, uncomment next line to read with headers:
% T = readtable('xf-e874-il-100000-n5.csv');

% Convert to numeric matrix (drop non-numeric rows if any)
data_csv = table2array(varfun(@double, T));

% If table2array failed due to mixed types, try converting columns individually
if any(isnan(data_csv(:)))
    numCols = width(T);
    data_csv = zeros(height(T), numCols);
    for i=1:numCols
        col = T{:,i};
        if iscell(col)
            data_csv(:,i) = str2double(col);
        else
            data_csv(:,i) = double(col);
        end
    end
end

% Assign commonly expected columns if present: alpha, CL, CD
if size(data_csv,2) >= 3
    alpha_vals = data_csv(:,1);
    CL_vals = data_csv(:,2);
    CD_vals = data_csv(:,3);
else
    error('CSV does not contain at least three columns for alpha, CL, CD.');
end
set(groot,'DefaultAxesFontName','Times New Roman','DefaultTextFontName','Times New Roman');


% Prepare and plot CL figure
fig_cl = figure('Name','fig_eppler874_cl','Units','centimeters','Position',[1 1 8 5]);
plot(alpha_vals, CL_vals, 'g.', 'MarkerSize', 9); hold on;
% plot(alpha_vals+delta_a_r, CL_vals);
% plot(alpha_vals+delta_a_f, CL_vals); hold off;
xlabel('\alpha [{\circ}]','FontName','Times New Roman','FontSize',9); 
ylabel('C_L [-]','FontName','Times New Roman','FontSize',9);
grid on; grid minor;
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);
% % Prepare for export as PDF
% set(fig_cl,'PaperUnits','inches','PaperPosition',[0 0 6 4],'PaperSize',[6 4]);

% Prepare and plot CD figure
fig_cd = figure('Name','fig_eppler874_cd','Units','centimeters','Position',[2 2 8 5]);
plot(alpha_vals, CD_vals, 'r.', 'MarkerSize', 9);
xlabel(['\alpha [{\circ}]'],'FontName','Times New Roman','FontSize',9); 
ylabel('C_D [-]','FontName','Times New Roman','FontSize',9);
grid on; grid minor;
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);
% % Prepare for export as PDF
% set(fig_cd,'PaperUnits','inches','PaperPosition',[0 0 6 4],'PaperSize',[6 4]);

%temporary commented out
exportgraphics(fig_cl, 'fig_eppler874_cl.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none', ...
    'Units', 'centimeters', ...
    'Width', 8, ...
    'Height', 5);
exportgraphics(fig_cd, 'fig_eppler874_cd.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none', ...
    'Units', 'centimeters', ...
    'Width', 8, ...
    'Height', 5);

%% WLICZENIE ASPECT RATIO do CL CD
a = alpha_vals';
l = CL_vals';
d = CD_vals';
%%
%augumentation of CL CD with custom points to better fit the polynomials 
alpha_vals = [-10 a 11 12 13 14 15]';
CL_vals = [-.55 l 0.655 0.6 0.55 0.51 0.48]';
CD_vals = [.12 d 0.12 0.16 0.20 0.28 0.32]';

ARf = 0.204^2 / 8080e-6;
ARr = 0.220^2 / 8708e-6;

% Induced angle of attack [deg]
delta_a_f = rad2deg( CL_vals ./ (pi*ARf));
delta_a_r = rad2deg( CL_vals ./ (pi*ARr));
% Corrected geometric angle of attack [deg]
alpha_f = alpha_vals + delta_a_f;
alpha_r = alpha_vals + delta_a_r;

% Induced drag coefficient
delta_CD_f = CL_vals.^2 ./ (pi * ARf);
delta_CD_r = CL_vals.^2 ./ (pi * ARr);
% Corrected drag coefficient
CD_f = CD_vals + delta_CD_f;
CD_r = CD_vals + delta_CD_r;

% Plot the corrected values over previous figures 
figure(fig_cl);
hold on;
plot(alpha_f, CL_vals, 'b-', 'LineWidth', 1.2);
plot(alpha_r, CL_vals, 'k--', 'LineWidth', 1.2);
legend('2D profile', 'Front hydrofoil', 'Rear hydrofoil', ...
       'Location', 'best');
hold off;

figure(fig_cd);
hold on;
plot(alpha_vals, CD_f, 'b-', 'LineWidth', 1.2);
plot(alpha_vals, CD_r, 'k--', 'LineWidth', 1.2);
legend('2D profile', 'Front hydrofoil', 'Rear hydrofoil', ...
       'Location', 'best');
hold off;

%% Try to approximate the characteristics with polynomials
ft_CL = fittype( ...
    'a + b*x + c*x^2 + d*x^3 + e*x^5 + f*x^7', ...
    'independent','x', ...
    'coefficients',{'a','b','c','d','e','f'});
ft_CD = fittype( ...
    'a + b*x + c*x^2 + d*x^3 + e*x^4 + f*x^5', ...
    'independent','x', ...
    'coefficients',{'a','b','c','d','e','f'});

fCL_f = fit(alpha_f, CL_vals, ft_CL, 'StartPoint', zeros(1,6));
fCL_r = fit(alpha_r, CL_vals, ft_CL, 'StartPoint', zeros(1,6));

fCD_f = fit(alpha_vals, CD_f, ft_CD, 'StartPoint', zeros(1,6));
fCD_r = fit(alpha_vals, CD_r, ft_CD, 'StartPoint', zeros(1,6));

% Typically when boat is leveled the actuation angle is only in the range
% -6/+12 degrees
alpha_range_standard = -6:0.1:12;
% But the boat is pitching by the phi so the range should be extended to 
% broader range
alpha_range_extended = -6-6:0.1:12+6;
alpha_fit = alpha_range_extended;

figc_cl = figure('Name','hydrofoil_corrected_cl','Units','centimeters','Position',[1 1 8 5]);
% plot(alpha_vals, CL_vals, 'g.', 'MarkerSize', 10); hold on;
plot(a, l, '.', 'MarkerSize', 8, 'Color', [0.5 1 0.5]); hold on;
xlabel('\alpha [{\circ}]','FontName','Times New Roman','FontSize',9); 
ylabel('C_L [-]','FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);
grid on; grid minor;
plot(alpha_fit, fCL_f(alpha_fit), '-', 'LineWidth', 1);
plot(alpha_fit, fCL_r(alpha_fit), '-', 'LineWidth', 1);
legend('XFOIL 2D', ...
       'front foil', ...
       'rear foil', ...
       'Location','best','FontName','Times New Roman','FontSize',9);
% plot(alpha_f, CL_vals, '.')
xlim([-6-4 12+2])

figc_cd = figure('Name','fig_eppler874_cd','Units','centimeters','Position',[7 1 8 5]);
% plot(alpha_vals, CD_vals, 'r.', 'MarkerSize', 10); hold on;
plot(a, d, '.', 'MarkerSize', 8, 'Color', [1 0.5 0.5]); hold on;
xlabel('\alpha [{\circ}]','FontName','Times New Roman','FontSize',9); 
ylabel('C_D [-]','FontName','Times New Roman','FontSize',9);
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);
grid on; grid minor;
plot(alpha_fit, fCD_f(alpha_fit), '-', 'LineWidth', 1);
plot(alpha_fit, fCD_r(alpha_fit), '-', 'LineWidth', 1);
legend('XFOIL 2D', ...
       'front foil', ...
       'rear foil', ...
       'Location','best','FontName','Times New Roman','FontSize',9);
% plot(alpha_vals, CD_f, '.')
xlim([-6-4 12+2])
ylim([0 .2])

%temporary commented out
exportgraphics(figc_cl, 'hydrofoil_corrected_cl.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none', ...
    'Units', 'centimeters', ...
    'Width', 8, ...
    'Height', 5);
exportgraphics(figc_cd, 'hydrofoil_corrected_cd.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none', ...
    'Units', 'centimeters', ...
    'Width', 8, ...
    'Height', 5);
%% SANITY CHECKS
figure;
plot(fCD_f(alpha_fit), fCL_f(alpha_fit), '-', 'LineWidth', 1.1); hold on;
plot(fCD_r(alpha_fit), fCL_r(alpha_fit), '--', 'LineWidth', 1.1);
xlabel('C_D');
ylabel('C_L');
grid on;
set(gca,'FontName','Times New Roman');
legend('Front hydrofoil', 'Rear hydrofoil', ...
       'Location','best');
figure;
plot(alpha_fit, fCL_f(alpha_fit)./fCD_f(alpha_fit), '-', 'LineWidth', 1); hold on;
plot(alpha_fit, fCL_r(alpha_fit)./fCD_r(alpha_fit), '-', 'LineWidth', 1);
xlabel('alpha');
ylabel('C_L/C_D');
grid on;
set(gca,'FontName','Times New Roman');
legend('Front hydrofoil', 'Rear hydrofoil', ...
       'Location','best');

%% SAVE THE LUTS TO THE FILE

LUT_alpha_F = alpha_range_extended(:);
LUT_alpha_R = alpha_range_extended(:);

LUT_CL_F = fCL_f(LUT_alpha_F);
LUT_CL_R = fCL_r(LUT_alpha_R);

LUT_CD_F = fCD_f(LUT_alpha_F);
LUT_CD_R = fCD_r(LUT_alpha_R);

save('eppler874.mat', ...
    'LUT_alpha_F', 'LUT_CL_F', 'LUT_CD_F', ...
    'LUT_alpha_R', 'LUT_CL_R', 'LUT_CD_R');

return
%% OLD STUFF:
%% http://airfoiltools.com/polar/details?polar=xf-e874-il-1000000
% [ alpha , CL , CD , CDp , CM , Top_Xtr , Bot_Xtr ]
data = [
  -9.750  -0.5864   0.09553   0.09396  -0.0030   1.0000   0.0076;
  -9.500  -0.5867   0.09116   0.08960  -0.0049   1.0000   0.0077;
  -9.250  -0.5922   0.08584   0.08430  -0.0074   1.0000   0.0075;
  -5.500  -0.5837   0.01397   0.00910  -0.0046   1.0000   0.0053;
  -5.250  -0.5624   0.01328   0.00836  -0.0033   1.0000   0.0060;
  -5.000  -0.5412   0.01249   0.00748  -0.0019   1.0000   0.0064;
  -4.750  -0.5208   0.01154   0.00642  -0.0002   1.0000   0.0065;
  -4.500  -0.4946   0.01100   0.00581   0.0001   0.9995   0.0071;
  -4.250  -0.4604   0.01027   0.00499  -0.0013   0.9975   0.0074;
  -4.000  -0.4256   0.00942   0.00399  -0.0027   0.9953   0.0072;
  -3.750  -0.3917   0.00882   0.00325  -0.0039   0.9929   0.0071;
  -3.500  -0.3579   0.00839   0.00272  -0.0051   0.9896   0.0073;
  -3.250  -0.3222   0.00809   0.00233  -0.0067   0.9863   0.0079;
  -3.000  -0.2857   0.00769   0.00198  -0.0086   0.9832   0.0281;
  -2.750  -0.2610   0.00688   0.00162  -0.0082   0.9736   0.1374;
  -2.500  -0.2334   0.00649   0.00143  -0.0082   0.9613   0.1891;
  -2.250  -0.1985   0.00617   0.00126  -0.0098   0.9486   0.2322;
  -2.000  -0.1476   0.00583   0.00109  -0.0151   0.9345   0.2867;
  -1.750  -0.0988   0.00535   0.00091  -0.0201   0.8950   0.3998;
  -1.500  -0.0723   0.00531   0.00082  -0.0197   0.8403   0.4511;
  -1.250  -0.0498   0.00529   0.00077  -0.0185   0.7941   0.5008;
  -1.000  -0.0274   0.00523   0.00074  -0.0173   0.7537   0.5562;
  -0.750  -0.0049   0.00514   0.00071  -0.0161   0.7190   0.6174;
  -0.500   0.0170   0.00499   0.00069  -0.0148   0.6888   0.6863;
  -0.250   0.0380   0.00481   0.00068  -0.0132   0.6621   0.7656;
   0.000   0.0592   0.00457   0.00069  -0.0116   0.6381   0.8596;
   0.250   0.1372   0.00464   0.00085  -0.0229   0.6071   0.9433;
   0.500   0.1767   0.00486   0.00098  -0.0253   0.5831   0.9615;
   0.750   0.2065   0.00503   0.00107  -0.0257   0.5633   0.9757;
   1.000   0.2380   0.00521   0.00118  -0.0265   0.5448   0.9843;
   1.250   0.2810   0.00534   0.00125  -0.0299   0.5258   0.9892;
   1.750   0.3610   0.00557   0.00137  -0.0354   0.4877   0.9969;
   2.000   0.4001   0.00571   0.00136  -0.0381   0.4497   0.9997;
   2.250   0.4253   0.00590   0.00140  -0.0376   0.4062   1.0000;
   2.500   0.4491   0.00607   0.00146  -0.0368   0.3720   1.0000;
   2.750   0.4726   0.00630   0.00153  -0.0360   0.3276   1.0000;
   3.000   0.4880   0.00761   0.00191  -0.0340   0.1011   1.0000;
   3.250   0.5081   0.00840   0.00233  -0.0326   0.0094   1.0000;
   3.500   0.5321   0.00863   0.00260  -0.0317   0.0076   1.0000;
   3.750   0.5558   0.00890   0.00293  -0.0307   0.0067   1.0000;
   4.000   0.5767   0.00964   0.00383  -0.0291   0.0051   1.0000;
   4.250   0.5989   0.01014   0.00441  -0.0278   0.0049   1.0000;
   4.500   0.6193   0.01088   0.00526  -0.0261   0.0048   1.0000;
   4.750   0.6377   0.01192   0.00642  -0.0240   0.0048   1.0000;
   5.000   0.6525   0.01388   0.00849  -0.0211   0.0053   1.0000;
   5.250   0.6765   0.01413   0.00878  -0.0202   0.0056   1.0000;
   6.000   0.7238   0.02581   0.02143  -0.0116   0.0096   1.0000;
   6.250   0.7398   0.02782   0.02369  -0.0094   0.0095   1.0000;
   6.500   0.7575   0.02892   0.02500  -0.0074   0.0093   1.0000;
   6.750   0.7818   0.02872   0.02489  -0.0062   0.0084   1.0000;
   7.000   0.7984   0.03046   0.02683  -0.0041   0.0076   1.0000;
   7.250   0.8113   0.03279   0.02937  -0.0018   0.0071   1.0000;
   7.500   0.8221   0.03523   0.03202   0.0005   0.0067   1.0000;
   7.750   0.8307   0.03774   0.03473   0.0028   0.0064   1.0000;
   8.000   0.8371   0.04035   0.03753   0.0051   0.0062   1.0000;
   8.250   0.8417   0.04294   0.04029   0.0073   0.0061   1.0000;
   8.500   0.8429   0.04583   0.04335   0.0096   0.0060   1.0000;
   8.750   0.8411   0.04854   0.04620   0.0117   0.0058   1.0000;
   9.000   0.8355   0.05141   0.04921   0.0139   0.0057   1.0000;
   9.250   0.8237   0.05441   0.05234   0.0166   0.0057   1.0000;
   9.500   0.8047   0.05730   0.05535   0.0194   0.0057   1.0000;
   9.750   0.7873   0.06113   0.05928   0.0186   0.0057   1.0000;
  10.000   0.7669   0.06714   0.06541   0.0141   0.0057   1.0000;
  10.250   0.7437   0.07795   0.07637   0.0051   0.0059   1.0000
];

alpha_vals = data(:,1);
CL_vals    = data(:,2);
CD_vals    = data(:,3);

% Add some points
alpha_vals = [alpha_vals; 15; 20];
CL_vals =    [CL_vals; 0.4; 0.4];
CD_vals =    [CD_vals; 0.3; 0.8];

% figure
% yyaxis left
% plot(alpha_vals, CL_vals, ".k");
% yyaxis right
% plot(alpha_vals, CD_vals, ".r");

%% Fitting the curves to data
ft_CL = fittype(....
    'a + b*x + c*x^3 + d*abs(x^3)*x + e*abs(x^5)*x + f*x^2', ...
    'independent', 'x', 'coefficients', ...
    {'a', 'b', 'c', 'd', 'e', 'f'});
fCL = fit(alpha_vals, CL_vals, ft_CL, 'StartPoint', ...
    [0.006, 0.13, -0.0002, -0.00004, 0, 0]);

ft_CD = fittype(...
    '0.0048 + a*x^2 + b*x + c*x^3 + d*x^4 + e*x^5', ...
    'independent', 'x', 'coefficients', ...
    {'a', 'b', 'c', 'd', 'e'});
fCD = fit(alpha_vals, CD_vals, ft_CD, 'StartPoint', ...
    [0, 0.00008, 0.000012, 0, 0]);


% Typically when boat is leveled the actuation angle is only in the range
% -6/+12 degrees
alpha_range_standard = -6:0.1:12;
% But the boat is pitching by the phi so the range should be extended to 
% broader range
alpha_range_extended = -6-6:0.1:12+6;

%% interp1 LUT
% Calculate the LUT
LUT_alpha = (-6-6:0.1:12+6)';
LUT_CL = fCL(LUT_alpha);
LUT_CD = fCD(LUT_alpha);

% Fitting 
CL_interp = interp1(LUT_alpha, LUT_CL, LUT_alpha, 'linear');
CD_interp = interp1(LUT_alpha, LUT_CD, LUT_alpha, 'linear');

%% Save 
save eppler874.mat LUT_alpha LUT_CL LUT_CD

%% Plot all results
figure
legend

yyaxis left
hold on;
% Plot standard usable range
fill([alpha_range_standard(1) alpha_range_standard(end) alpha_range_standard(end) alpha_range_standard(1)], ...
     [ -1 -1 1 1 ], ...
     [0.9 0.9 1.0], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
% Plot the fitted curve
plot(alpha_range_extended, fCL(alpha_range_extended)); 
% Plot the input data points
plot(alpha_vals, CL_vals, ".k");
% Plot the interp1 results
plot(LUT_alpha, CL_interp, '--', 'DisplayName', 'CL interp1 LUT');
hold off;
grid on;

yyaxis right
hold on;
% Plot standard usable range
fill([alpha_range_standard(1) alpha_range_standard(end) alpha_range_standard(end) alpha_range_standard(1)], ...
     [ 0 0  .2 .2 ], ...
     [0.9 0.9 1.0], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
% Plot the fitted curve
plot(alpha_range_extended, fCD(alpha_range_extended));
% Plot the input data points
plot(alpha_vals, CD_vals, ".r"); 
% Plot the interp1 results
plot(LUT_alpha, CD_interp, '--', 'DisplayName', 'CD interp1 LUT');
hold off;
grid on;
