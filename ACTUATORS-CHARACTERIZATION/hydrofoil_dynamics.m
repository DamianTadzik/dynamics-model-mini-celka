clc; clear; close all;
%% Load file
parquet_file_125 = 'data/LOG000.PARQUET';
info = parquetinfo(parquet_file_125);
% Actually load file
T125 = parquetread(parquet_file_125);
for i = 1:length(T125.Properties.VariableDescriptions)
    disp(T125.Properties.VariableDescriptions{i});
end

%% Plot the signals

[tRADIO_CONTROL_MODE_SWITCH, RADIO_CONTROL_MODE_SWITCH] = remove_nans(T125.seconds_since_start, T125.RADIO_CONTROL_MODE_SWITCH);

[tACTUATOR_REAR_FOIL_FEEDBACK_SETPOINT_US, ACTUATOR_REAR_FOIL_FEEDBACK_SETPOINT_US] = remove_nans(T125.seconds_since_start, T125.ACTUATOR_REAR_FOIL_FEEDBACK_SETPOINT_US);
[tACTUATOR_REAR_FOIL_FEEDBACK_POSITION_RAW, ACTUATOR_REAR_FOIL_FEEDBACK_POSITION_RAW] = remove_nans(T125.seconds_since_start, T125.ACTUATOR_REAR_FOIL_FEEDBACK_POSITION_RAW);

tiledlayout("vertical")
nexttile
plot(tRADIO_CONTROL_MODE_SWITCH, RADIO_CONTROL_MODE_SWITCH)

nexttile
yyaxis left
plot(tACTUATOR_REAR_FOIL_FEEDBACK_SETPOINT_US, ACTUATOR_REAR_FOIL_FEEDBACK_SETPOINT_US, '-o')
yyaxis right
plot(tACTUATOR_REAR_FOIL_FEEDBACK_POSITION_RAW, ACTUATOR_REAR_FOIL_FEEDBACK_POSITION_RAW, '-o')