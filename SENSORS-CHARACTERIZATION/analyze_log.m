path = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_04_26_divonnes\logs_parquet\";
parquet_files = ["log0" "log1" "log2" "log3"] + ".parquet";

path = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_08_21_home\logs_parquet\";
parquet_files = ["log0" "log7" "log8"] + ".parquet";
% for k = 1
for k = 1:numel(parquet_files)
    parquet_file = path + parquet_files(k);
    % parquet_file = parquet_files(k);

    T = parquetread(parquet_file);

    timestamp = T.timestamp;
    dtimestamp = diff(timestamp);

    figure("Name", parquet_files(k));
    subplot(2,1,1);
    plot(timestamp, timestamp, '.');
    subplot(2,1,2);
    plot(timestamp, [dtimestamp; 0]);
end
