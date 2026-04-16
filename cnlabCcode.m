% Load data
baseline = readmatrix('baseline.csv');
proposed = readmatrix('proposed.csv');

% Fix size mismatch
min_len = min([size(baseline,1), size(proposed,1)]);
baseline = baseline(1:min_len, :);
proposed = proposed(1:min_len, :);

% NOW extract values (IMPORTANT: after trimming)
time = baseline(:,1);

delay_base = baseline(:,2);
delay_prop = proposed(:,2);

pdr_base = baseline(:,3);
pdr_prop = proposed(:,3);

thru_base = baseline(:,4);
thru_prop = proposed(:,4);

fair_base = baseline(:,5);
fair_prop = proposed(:,5);

%% 📊 1. Delay Plot
figure;
plot(time, delay_base, 'r-o', 'LineWidth', 2); hold on;
plot(time, delay_prop, 'b-s', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Average Delay (s)');
title('Delay Comparison');
legend('Baseline DCF','Proposed QoS');
grid on;

%% 📊 2. Throughput Plot
figure;
plot(time, thru_base, 'r-o', 'LineWidth', 2); hold on;
plot(time, thru_prop, 'b-s', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Throughput (kbps)');
title('Throughput Comparison');
legend('Baseline','Proposed');
grid on;

%% 📊 3. PDR Plot
figure;
plot(time, pdr_base, 'r-o', 'LineWidth', 2); hold on;
plot(time, pdr_prop, 'b-s', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Packet Delivery Ratio');
title('PDR Comparison');
legend('Baseline','Proposed');
grid on;

%% 📊 4. Fairness Plot
figure;
plot(time, fair_base, 'r-o', 'LineWidth', 2); hold on;
plot(time, fair_prop, 'b-s', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Jain Fairness Index');
title('Fairness Comparison');
legend('Baseline','Proposed');
grid on;


