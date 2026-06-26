%% Make 2D top-down view and Angle tracking plots (IEEE style)
clear;
clc;
close all;

%% Load Data
dataset = {'square_ref_10s_LQRFF.mat', ...
           'square_MPC_2.mat'};

%% Define Reference Path (Square)
% Assuming a square centered at (0,0) with side length 0.2m (+/- 0.1m)
L = 0.1; 
x_ref = [-L,  L, L, -L, -L];
y_ref = [-L, -L, L,  L, -L];

%% Initialize Figure 1: Top-down Tracking Performance
figWidth = 3.5;  % IEEE single column width
figHeight1 = 3.0; 

fig1 = figure('Units', 'inches', 'Position', [1, 1, figWidth, figHeight1]);
hold on;
grid on;
box on;

% Plot reference path
h_ref = plot(x_ref, y_ref, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference');

% Formatting settings
colors = {'#0072BD', '#D95319'}; % Blue for LQR, Red for MPC
lineStyles = {'-', '-.'}; 
labels = {'LQR', 'MPC'};

% Pre-allocate handles and data arrays for the second figure
h_plots = gobjects(1, length(dataset));
t_data = cell(1, length(dataset));
alpha_c = cell(1, length(dataset));
beta_c = cell(1, length(dataset));
alpha_m = cell(1, length(dataset));
beta_m = cell(1, length(dataset));

for i = 1:length(dataset)
    % Dynamically extract the struct field name from the file name
    [~, name, ~] = fileparts(dataset{i}); 
    data = load(dataset{i});
    measurements = data.(name).Y;
    
    % Extract data
    alpha_c{i} = measurements(1).Data;
    beta_c{i}  = measurements(2).Data;
    x_cam      = measurements(3).Data;
    y_cam      = measurements(4).Data;
    alpha_m{i} = measurements(5).Data;
    beta_m{i}  = measurements(6).Data;
    
    % Generate time vector manually (Ts = 0.001s)
    N = length(alpha_c{i});
    t_data{i} = (0:N-1)' * 0.001; 
    
    % Plot tracked paths for Figure 1
    figure(fig1);
    h_plots(i) = plot(x_cam, y_cam, ...
        'Color', colors{i}, ...
        'LineStyle', lineStyles{i}, ...
        'LineWidth', 1.2, ...
        'DisplayName', labels{i});
end

% Formatting Figure 1 (IEEE Standards)
figure(fig1);
axis equal; 
padding = L * 1.3; 
xlim([-padding padding]); 
ylim([-padding padding]);

xlabel('\it x \rm [m]', 'FontName', 'Times New Roman', 'FontSize', 9);
ylabel('\it y \rm [m]', 'FontName', 'Times New Roman', 'FontSize', 9);

lgd1 = legend([h_ref, h_plots], 'Location', 'northeast');
set(lgd1, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off');

ax1 = gca;
ax1.FontName = 'Times New Roman';
ax1.FontSize = 9;
ax1.TickDir = 'in';
ax1.XMinorTick = 'on';

%% Initialize Figure 2: Commanded vs Measured Angles
% Taller figure to accommodate two vertically stacked subplots
figHeight2 = 4.0; 
fig2 = figure('Units', 'inches', 'Position', [5, 1, figWidth, figHeight2]);

% --- Subplot 1: Alpha Angle ---
ax2_1 = subplot(2,1,1);
hold on; grid on; box on;

for i = 1:length(dataset)
    % Commanded angle (lighter color, dashed)
    plot(t_data{i}, alpha_c{i}, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, ...
        'HandleVisibility', 'off'); % Hiding from legend to save space
    
    % Measured angle (main color, solid/dash-dot)
    plot(t_data{i}, alpha_m{i}, ...
        'Color', colors{i}, ...
        'LineStyle', lineStyles{i}, ...
        'LineWidth', 1.2, ...
        'DisplayName', sprintf('%s Measured', labels{i}));
end
% Add a dummy plot for the commanded legend entry
plot(NaN, NaN, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'DisplayName', 'Commanded');

ylabel('\alpha [rad]', 'FontName', 'Times New Roman', 'FontSize', 9);
lgd2_1 = legend('Location', 'best');
set(lgd2_1, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off');

% --- Subplot 2: Beta Angle ---
ax2_2 = subplot(2,1,2);
hold on; grid on; box on;

for i = 1:length(dataset)
    % Commanded angle
    plot(t_data{i}, beta_c{i}, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, ...
        'HandleVisibility', 'off');
    
    % Measured angle
    plot(t_data{i}, beta_m{i}, ...
        'Color', colors{i}, ...
        'LineStyle', lineStyles{i}, ...
        'LineWidth', 1.2, ...
        'DisplayName', sprintf('%s Measured', labels{i}));
end

xlabel('Time [s]', 'FontName', 'Times New Roman', 'FontSize', 9);
ylabel('\beta [rad]', 'FontName', 'Times New Roman', 'FontSize', 9);

% Format Axes for Figure 2
axesList = [ax2_1, ax2_2];
for k = 1:2
    axesList(k).FontName = 'Times New Roman';
    axesList(k).FontSize = 9;
    axesList(k).TickDir = 'in';
    axesList(k).XMinorTick = 'on';
    axesList(k).YMinorTick = 'on';
end
set(fig2, 'Color', 'w');

%% Initialize Figure 3: Power Spectral Density (PSD) of Measured Alpha
% Slightly shorter figure height as it is a single plot
figHeight3 = 2.5; 
fig3 = figure('Units', 'inches', 'Position', [9, 1, figWidth, figHeight3]);
hold on; 
grid on; 
box on;

Fs = 1000; % Sampling frequency (1 / 0.001s)

for i = 1:length(dataset)
    % Calculate PSD using Welch's method
    % Default window, overlap, and FFT length are usually sufficient
    [pxx, f] = pwelch(alpha_m{i}, [], [], [], Fs);
    
    % Convert power to Decibels (dB/Hz) for standard PSD visualization
    plot(f, 10*log10(pxx), ...
        'Color', colors{i}, ...
        'LineStyle', lineStyles{i}, ...
        'LineWidth', 1.2, ...
        'DisplayName', labels{i});
end

% Formatting
xlabel('Frequency [Hz]', 'FontName', 'Times New Roman', 'FontSize', 9);
ylabel('PSD [dB/Hz]', 'FontName', 'Times New Roman', 'FontSize', 9);

% Mechanical systems rarely have dynamics above 50 Hz, so we limit the x-axis
% to make the low-frequency peaks clear. Change this if your system is faster.
xlim([0 50]); 

% Legend
lgd3 = legend('Location', 'northeast');
set(lgd3, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off');

% Axis Tick Properties
ax3 = gca;
ax3.FontName = 'Times New Roman';
ax3.FontSize = 9;
ax3.TickDir = 'in';
ax3.XMinorTick = 'on';
ax3.YMinorTick = 'on';

% Set figure background to white
set(fig3, 'Color', 'w');

%% Export high-quality vector graphic
% exportgraphics(fig3, 'Alpha_PSD.pdf', 'ContentType', 'vector');