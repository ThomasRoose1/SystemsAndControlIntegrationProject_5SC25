%% Make 2D top-down view and Angle tracking plots with Time Cropping
clear;
clc;
close all;

%% 1. Configuration & Data Loading
dataset = {'square_LQRff_last.mat', ...
           'square_MPC_z0_OFF.mat', ...
           'square_PID_last.mat', ...
           'OptimalHinf.mat'};
labels  = {'LQR', 'MPC', 'PID', 'Hinf'}; % Add more labels if you add more datasets

% Define manual start and end times [start_time, end_time] for EACH dataset.
% Tip: Run once with [0, inf], look at Figure 1, then update these numbers!
crop_times = [
    6, 19.5;   
    4, 19.5;  
    4, 20;
    2, 20
];
% crop_times = [
%     0, inf;   
%     0, inf;   
%     0, inf;   
%     0, inf
% ];

Ts = 0.001; % Sampling time

%% Define Reference Path (Square)
L = 0.1; 
x_ref = [-L,  L, L, -L, -L];
y_ref = [-L, -L, L,  L, -L];

%% Formatting Settings (Handles ANY number of datasets)
colors = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30'}; 
lineStyles = {'-', '-.', '--', ':'}; 

% Pre-allocate cell arrays for cropped data
num_sets = length(dataset);
t_data  = cell(1, num_sets);
alpha_c = cell(1, num_sets);
beta_c  = cell(1, num_sets);
alpha_m = cell(1, num_sets);
beta_m  = cell(1, num_sets);
x_data  = cell(1, num_sets);
y_data  = cell(1, num_sets);

%% Initialize Figure 1: Preview X and Y over time for Cropping
figPreview = figure('Name', 'Preview: Find Your Crop Times', 'Units', 'inches', 'Position', [1, 5, 6, 4]);
ax_x = subplot(2,1,1); hold on; grid on; box on; ylabel('Raw x [m]', 'FontName', 'Times New Roman');
ax_y = subplot(2,1,2); hold on; grid on; box on; ylabel('Raw y [m]', 'FontName', 'Times New Roman'); xlabel('Time [s]', 'FontName', 'Times New Roman');

%% Load, Preview, and Crop Data Loop
for i = 1:num_sets
    % Handle dynamic style wrapping for infinite datasets
    c_idx = mod(i-1, length(colors)) + 1;
    l_idx = mod(i-1, length(lineStyles)) + 1;
    
    % --- THE FIX: Dynamically read the internal variable name ---
    data = load(dataset{i});
    fields = fieldnames(data);
    internal_var = fields{1}; % Grab the first variable inside the .mat file
    
    measurements = data.(internal_var).Y;
    % ------------------------------------------------------------
    
    % Extract raw data
    raw_ac = measurements(1).Data;
    raw_bc = measurements(2).Data;
    raw_x  = measurements(3).Data;
    raw_y  = measurements(4).Data;
    raw_am = measurements(5).Data;
    raw_bm = measurements(6).Data;
    
    % Generate raw time vector
    N = length(raw_ac);
    t_raw = (0:N-1)' * Ts; 
    
    % --- PLOT PREVIEW ---
    plot(ax_x, t_raw, raw_x, 'Color', colors{c_idx}, 'DisplayName', labels{i});
    plot(ax_y, t_raw, raw_y, 'Color', colors{c_idx}, 'DisplayName', labels{i});
    
    % --- CROP DATA ---
    t_start = crop_times(i, 1);
    t_end   = crop_times(i, 2);
    valid_idx = (t_raw >= t_start) & (t_raw <= t_end);
    
    % Store cropped data and align time so all sets start at t=0
    t_data{i}  = t_raw(valid_idx) - t_start; 
    alpha_c{i} = raw_ac(valid_idx);
    beta_c{i}  = raw_bc(valid_idx);
    x_data{i}  = raw_x(valid_idx);
    y_data{i}  = raw_y(valid_idx);
    alpha_m{i} = raw_am(valid_idx);
    beta_m{i}  = raw_bm(valid_idx);
end
legend(ax_x, 'Location', 'best');

%% Initialize Figure 2: Top-down Tracking Performance (IEEE)
figWidth = 3.5;  
figHeight1 = 3.0; 
fig2 = figure('Units', 'inches', 'Position', [1, 1, figWidth, figHeight1]);
hold on; grid on; box on;

h_ref = plot(x_ref, y_ref, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference');
h_plots = gobjects(1, num_sets);

for i = 1:num_sets
    c_idx = mod(i-1, length(colors)) + 1;
    l_idx = mod(i-1, length(lineStyles)) + 1;
    
    h_plots(i) = plot(x_data{i}, y_data{i}, ...
        'Color', colors{c_idx}, ...
        'LineStyle', lineStyles{l_idx}, ...
        'LineWidth', 1.2, ...
        'DisplayName', labels{i});
end

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
set(fig2, 'Color', 'w');

%% Initialize Figure 3: Commanded vs Measured Angles
figHeight2 = 4.0; 
fig3 = figure('Units', 'inches', 'Position', [5, 1, figWidth, figHeight2]);

% --- Subplot 1: Alpha Angle ---
ax3_1 = subplot(2,1,1);
hold on; grid on; box on;
for i = 1:num_sets
    c_idx = mod(i-1, length(colors)) + 1;
    l_idx = mod(i-1, length(lineStyles)) + 1;
    
    plot(t_data{i}, alpha_c{i}, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'HandleVisibility', 'off'); 
    plot(t_data{i}, alpha_m{i}, 'Color', colors{c_idx}, 'LineStyle', lineStyles{l_idx}, 'LineWidth', 1.2, 'DisplayName', sprintf('%s Measured', labels{i}));
end
plot(NaN, NaN, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'DisplayName', 'Commanded');
ylabel('\alpha [rad]', 'FontName', 'Times New Roman', 'FontSize', 9);
lgd2_1 = legend('Location', 'best');
set(lgd2_1, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off');

% --- Subplot 2: Beta Angle ---
ax3_2 = subplot(2,1,2);
hold on; grid on; box on;
for i = 1:num_sets
    c_idx = mod(i-1, length(colors)) + 1;
    l_idx = mod(i-1, length(lineStyles)) + 1;
    
    plot(t_data{i}, beta_c{i}, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'HandleVisibility', 'off');
    plot(t_data{i}, beta_m{i}, 'Color', colors{c_idx}, 'LineStyle', lineStyles{l_idx}, 'LineWidth', 1.2, 'DisplayName', sprintf('%s Measured', labels{i}));
end
xlabel('Time [s]', 'FontName', 'Times New Roman', 'FontSize', 9);
ylabel('\beta [rad]', 'FontName', 'Times New Roman', 'FontSize', 9);

axesList = [ax3_1, ax3_2];
for k = 1:2
    axesList(k).FontName = 'Times New Roman';
    axesList(k).FontSize = 9;
    axesList(k).TickDir = 'in';
    axesList(k).XMinorTick = 'on';
    axesList(k).YMinorTick = 'on';
end
set(fig3, 'Color', 'w');

%% Initialize Figure 4: Power Spectral Density (PSD)
figHeight3 = 2.5; 
fig4 = figure('Units', 'inches', 'Position', [9, 1, figWidth, figHeight3]);
hold on; grid on; box on;
Fs = 1/Ts; 

for i = 1:num_sets
    c_idx = mod(i-1, length(colors)) + 1;
    l_idx = mod(i-1, length(lineStyles)) + 1;
    
    [pxx, f] = pwelch(alpha_m{i}, [], [], [], Fs);
    plot(f, 10*log10(pxx), 'Color', colors{c_idx}, 'LineStyle', lineStyles{l_idx}, 'LineWidth', 1.2, 'DisplayName', labels{i});
end

xlabel('Frequency [Hz]', 'FontName', 'Times New Roman', 'FontSize', 9);
ylabel('PSD [dB/Hz]', 'FontName', 'Times New Roman', 'FontSize', 9);
xlim([0 50]); 
lgd3 = legend('Location', 'northeast');
set(lgd3, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off');

ax4 = gca;
ax4.FontName = 'Times New Roman';
ax4.FontSize = 9;
ax4.TickDir = 'in';
ax4.XMinorTick = 'on';
ax4.YMinorTick = 'on';
set(fig4, 'Color', 'w');