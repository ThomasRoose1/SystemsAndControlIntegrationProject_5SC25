%% Actuator System ID
clear; clc; close all

%% Load experiment data
load("motorA_2A.mat");

%% Plot all data to find which is the one we need
% 1. Extract the shared time vector
t_raw = motorA_2A.X.Data;
Ts = t_raw(2) - t_raw(1);
fs = 1 / Ts;

% 2. Extract the 'Position [m]' data for each actuator based on the layout
posActuator1 = motorA_2A.Y(2).Data;
posActuator2 = motorA_2A.Y(5).Data;
posActuator3 = motorA_2A.Y(8).Data;

% 3. Create a tiled plot to compare them instantly
figure('Name', 'Actuator Movement Comparison', 'Color', 'w');

% Actuator 1
subplot(3, 1, 1);
plot(t_raw, posActuator1, 'LineWidth', 1.5, 'Color', [0 0.4470 0.7410]);
title('Actuator 1 - Position [m]');
xlabel('Time [s]'); ylabel('Position');
grid on;

% Actuator 2
subplot(3, 1, 2);
plot(t_raw, posActuator2, 'LineWidth', 1.5, 'Color', [0.8500 0.3250 0.0980]);
title('Actuator 2 - Position [m]');
xlabel('Time [s]'); ylabel('Position');
grid on;

% Actuator 3
subplot(3, 1, 3);
plot(t_raw, posActuator3, 'LineWidth', 1.5, 'Color', [0.9290 0.6940 0.1250]);
title('Actuator 3 - Position [m]');
xlabel('Time [s]'); ylabel('Position');
grid on;

% Link the x-axes so zooming in on one zooms all of them
linkaxes(findobj(gcf, 'Type', 'axes'), 'x');

%% Select data of actuator A
r_raw = motorA_2A.Y(1).Data;
y_raw = motorA_2A.Y(2).Data;

%% Find start and end index
% Set a small noise margin based on your starting data
noise_margin = 0.1 * max(abs(r_raw)); 

% Find every single point where the signal moves past the noise
all_movement = find(abs(r_raw - r_raw(1)) > noise_margin);

% Snip exactly from the very first movement to the absolute last movement
start_idx = all_movement(1);
end_idx   = all_movement(end);

%% Extract active data and seperate periods
% define length and number of periods
N_total = end_idx - start_idx + 1;
P = 10;
N = floor(N_total / P); % Force N to be an integer

% take only active part
t = t_raw(start_idx:end_idx);
r = r_raw(start_idx:end_idx);
y = y_raw(start_idx:end_idx);

% Force column vectors
r = r(:);
y = y(:);

% Truncate t, r, and y so they match exactly N * P elements
t = t(1:N*P);
r = r(1:N*P);
y = y(1:N*P);

% % make data zero mean
% r = r - mean(r);
% y = y - mean(y);

% reshape so each column is a period
r_periods = reshape(r, N, P);
y_periods = reshape(y, N, P);

% make zero mean
r_periods = r_periods - mean(r_periods, 1);
y_periods = y_periods - mean(y_periods, 1);

% remove transients by not using the first period
transient = 1; % period
P_steady = P - transient;
r_steady = r_periods(:, transient+1:end);
y_steady = y_periods(:, transient+1:end);

%% Take the fft
R_fft = fft(r_steady); 
Y_fft = fft(y_steady); 

%% Compute maximum likelyhood FRF
% Sum across the periods (the 2nd dimension)
R_sum = sum(R_fft, 2);
Y_sum = sum(Y_fft, 2);

% Perform the division to get the FRF estimate
G_ml = Y_sum ./ R_sum;

%% Provide an uncertainty estimate
% Find variance of input and output FFT data
var_R = var(R_fft, 0, 2);
var_Y = var(Y_fft, 0, 2);

% Find mean
R_mean = mean(R_fft, 2);
Y_mean = mean(Y_fft, 2);

% Find covariance 
dR = R_fft - R_mean;
dY = Y_fft - Y_mean;

% Calculate covariance matrix
var_YR = (1 / (P_steady - 1)) * sum(dY .* conj(dR), 2);

% Compute std 
term1 = var_Y ./ (abs(Y_mean).^2);
term2 = var_R ./ (abs(R_mean).^2);
term3 = 2 * real(var_YR ./ (Y_mean .* R_mean));

std_G_ml = (1 / P_steady) * (abs(G_ml).^2) .* (term1 + term2 - term3);

%% Parametric model
% compute the mean
r_arx = mean(r_steady, 2);
y_arx = mean(y_steady, 2);

% 3. Define the clean data as an iddata object
data_id = iddata(y_arx, r_arx, Ts);

% 4. Estimate the ARX model
na = 2;
nb = 1;
nk = 0;
model_arx = arx(data_id, [na nb nk]);

%% Compute Parametric Model Frequency Response
% Frequency vector for plotting (Hz)
f_vec = (0:N-1) * (fs / N);
idx = 1:floor(N/2); % Plot only up to Nyquist
f_plot = f_vec(idx);
% Convert our plotting frequency vector from Hz to rad/s
w_plot = 2 * pi * f_plot;

% Evaluate the complex frequency response of the estimated ARX model
[mag_model, phase_model] = bode(model_arx, w_plot);

% bode() returns 3D arrays [outputs x inputs x frequencies], we squeeze them to vectors
mag_model = squeeze(mag_model);
phase_model = squeeze(phase_model); % Note: bode() returns phase in degrees directly!

% Wrap the parametric phase strictly between -180 and 180 degrees
phase_model_wrapped = wrapTo180(phase_model);

%% Plot FRF and Parametric Model Overlay
fig = figure('Name', 'FRF and Model Comparison', 'Color', 'w');

% Subplot 1: Magnitude Response
ax1 = subplot(2,1,1);
semilogx(f_plot, 20*log10(abs(G_ml(idx))), 'LineWidth', 1.5, 'Color', [0 0.447 0.741]); 
hold on;
% Plot the ARX Model Magnitude overlay
semilogx(f_plot, 20*log10(mag_model), '--', 'LineWidth', 2, 'Color', [0.466 0.674 0.188]);
% Plot standard deviation
semilogx(f_plot, 20*log10(sqrt(abs(std_G_ml(idx)))), 'o', 'MarkerSize', 2, ...
    'MarkerFaceColor', [0.85 0.33 0.1], 'Color', [0.85 0.33 0.1]);
ylabel('$|G|$ [dB]', 'Interpreter', 'latex');
grid on; 
legend({'$G_{ML}$', 'ARX Model', '$\sigma_{G}$'}, 'Interpreter', 'latex', 'Location', 'best');

% Subplot 2: Phase Response
ax2 = subplot(2,1,2);
% Get raw phase in degrees and wrap it for FRF
phase_deg = rad2deg(angle(G_ml(idx)));
phase_wrapped = wrapTo180(phase_deg);

semilogx(f_plot, phase_wrapped, 'LineWidth', 1.5, 'Color', [0 0.447 0.741]); 
hold on;
% Plot the ARX Model Phase overlay
semilogx(f_plot, phase_model_wrapped, '--', 'LineWidth', 2, 'Color', [0.466 0.674 0.188]);
ylabel('$\angle G [^\circ]$', 'Interpreter', 'latex');
xlabel('Frequency [Hz]', 'Interpreter', 'latex');
grid on; 
legend({'$G_{ML}$', 'ARX Model'}, 'Interpreter', 'latex', 'Location', 'best');

% Link axes so zooming in on frequency matches both subplots
linkaxes([ax1, ax2], 'x');