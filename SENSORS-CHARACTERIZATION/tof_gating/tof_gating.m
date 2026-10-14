parquetFile = "log2.parquet";
P = parquetread(parquetFile);
disp("Parquet columns:");
disp(P.Properties.VariableNames.');
%%

t_python = P.timestamp_s;
FL        = P.("can_signals_DISTANCE_FORE_FEEDBACK_RANGE_MM_L");
FL_status = P.("can_signals_DISTANCE_FORE_FEEDBACK_ERROR_STATUS_L");
FR        = P.("can_signals_DISTANCE_FORE_FEEDBACK_RANGE_MM_R");
FR_status = P.("can_signals_DISTANCE_FORE_FEEDBACK_ERROR_STATUS_R");
idx = ...
    ~isnan(t_python) & ...
    ~isnan(FL) & ...
    ~isnan(FL_status) & ...
    ~isnan(FR) & ...
    ~isnan(FR_status);
t_F = t_python(idx)-t_python(1);
FL        = FL(idx);
FL_status = FL_status(idx);
FR        = FR(idx);
FR_status = FR_status(idx);

AL        = P.("can_signals_DISTANCE_ACHTER_FEEDBACK_RANGE_MM_L");
AL_status = P.("can_signals_DISTANCE_ACHTER_FEEDBACK_ERROR_STATUS_L");
AR        = P.("can_signals_DISTANCE_ACHTER_FEEDBACK_RANGE_MM_R");
AR_status = P.("can_signals_DISTANCE_ACHTER_FEEDBACK_ERROR_STATUS_R");
idx = ...
    ~isnan(t_python) & ...
    ~isnan(AL) & ...
    ~isnan(AL_status) & ...
    ~isnan(AR) & ...
    ~isnan(AR_status);
t_A = t_python(idx)-t_python(1);
AL        = AL(idx);
AL_status = AL_status(idx);
AR        = AR(idx);
AR_status = AR_status(idx);


figure;
ax = [];
ax(end+1) = subplot(2,2,1)
grid on; hold on;
plot(t_F, FL, '.')
status_ok = (FL_status == 0) & (FL > 2) & (FL < 250);
plot(t_F(status_ok), FL(status_ok), 'o');

ax(end+1) = subplot(2,2,2)
grid on; hold on;
plot(t_F, FR, '.')
status_ok = (FR_status == 0) & (FR > 2) & (FR < 250);
plot(t_F(status_ok), FR(status_ok), 'o');


ax(end+1) = subplot(2,2,3)
grid on; hold on;
plot(t_A, AL, '.')
status_ok = (AL_status == 0) & (AL > 14) & (AL < 240);
plot(t_A(status_ok), AL(status_ok), 'o');

ax(end+1) = subplot(2,2,4)
grid on; hold on;
plot(t_A, AR, '.')
status_ok = (AR_status == 0) & (AR > 14) & (AR < 240);
plot(t_A(status_ok), AR(status_ok), 'o');


linkaxes(ax, 'xy')
0