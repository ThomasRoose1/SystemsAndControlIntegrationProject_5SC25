%% Make a 2D plot of top-down view of the plate with reference and tracked performance IEEE style
clear;
clc;
close all;

%% Load data
dataset = {'square_PIDff.mat', ...
           'square_Hinf.mat', ...
           'square_LQRff.mat', ...
           'square_MPC.mat'};

% Ensure labels array matches the number of datasets
labels = {'PID', 'Hinf', 'LQR', 'MPC'}; 

Ts = 0.001; % Assuming 1ms sample time for time vectors and control effort

%% Extract Reference from the Simulation Data
% Load the first dataset to grab the generated reference
ref_data = load(dataset{1});

% Extract the full state reference and squeeze out empty dimensions
r_raw = squeeze(ref_data.out.simout.r.Data);

% Ensure we are grabbing the right axes regardless of row/column orientation
if size(r_raw, 1) > size(r_raw, 2)
    x_ref = r_raw(:, 1);
    y_ref = r_raw(:, 3);
else
    x_ref = r_raw(1, :);
    y_ref = r_raw(3, :);
end
% Force reference into strict column vectors for math later
x_ref = x_ref(:);
y_ref = y_ref(:);

% Generate reference time vector
t_ref = (0:length(x_ref)-1)' * Ts;

%% Initialize Figure 1: Top-Down Tracking Performance
figWidth = 3.5;  % IEEE single column width
figHeight1 = 3.0; 
fig1 = figure('Units', 'inches', 'Position', [1, 1, figWidth, figHeight1]);
hold on; grid on; box on;

% Plot Spatial Reference
h_ref = plot(x_ref, y_ref, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference');

%% Initialize Figure 2: X and Y Time Tracking
figHeight2 = 4.0; % Taller figure for two stacked subplots
fig2 = figure('Units', 'inches', 'Position', [5, 1, figWidth, figHeight2]);

% --- Subplot 1: X Tracking ---
ax_x = subplot(2,1,1);
hold on; grid on; box on;
plot(t_ref, x_ref, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference');
ylabel('\it x \rm [m]', 'FontName', 'Times New Roman', 'FontSize', 9);

% --- Subplot 2: Y Tracking ---
ax_y = subplot(2,1,2);
hold on; grid on; box on;
plot(t_ref, y_ref, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off'); % Hide from legend
xlabel('Time [s]', 'FontName', 'Times New Roman', 'FontSize', 9);
ylabel('\it y \rm [m]', 'FontName', 'Times New Roman', 'FontSize', 9);

%% Load, Plot, and Analyze Tracked Data
% Define styles for infinite wrapping
colors = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30'}; 
lineStyles = {'-', '-.', '--', ':'}; 

num_sets = length(dataset);
h_plots = gobjects(1, num_sets); 

% Pre-allocate metrics arrays
rms_error   = zeros(1, num_sets);
peak_error  = zeros(1, num_sets);
ctrl_effort = zeros(1, num_sets);

for i = 1:num_sets
    % Dynamic style wrapping
    c_idx = mod(i-1, length(colors)) + 1;
    l_idx = mod(i-1, length(lineStyles)) + 1;
    
    % Load the structural data from the .mat file
    data = load(dataset{i});
    
    % Extract x and y data arrays from the timeseries
    x_data = squeeze(data.out.simout.x.Data); x_data = x_data(:);
    y_data = squeeze(data.out.simout.y.Data); y_data = y_data(:);
    
    % Generate dynamic time vector for this specific dataset
    t_data = (0:length(x_data)-1)' * Ts;
    
    % Extract control angles (fallback to 0 if they don't exist)
    if isfield(data.out.simout, 'alpha_out')
        alpha_data = squeeze(data.out.simout.alpha_out.Data); alpha_data = alpha_data(:);
        beta_data  = squeeze(data.out.simout.beta_out.Data);  beta_data = beta_data(:);
    else
        alpha_data = zeros(size(x_data));
        beta_data = zeros(size(x_data));
    end
    
    % --- CALCULATE PERFORMANCE METRICS ---
    min_len = min(length(x_data), length(x_ref));
    
    curr_x = x_data(1:min_len);
    curr_y = y_data(1:min_len);
    curr_ref_x = x_ref(1:min_len);
    curr_ref_y = y_ref(1:min_len);
    
    % Calculate Euclidean distance error
    err_x = curr_x - curr_ref_x;
    err_y = curr_y - curr_ref_y;
    err_mag = sqrt(err_x.^2 + err_y.^2);
    
    rms_error(i) = sqrt(mean(err_mag.^2));
    peak_error(i) = max(err_mag);
    
    % Control Effort (Discrete Integral of Squared Inputs)
    ctrl_effort(i) = sum(alpha_data(1:min_len).^2 + beta_data(1:min_len).^2) * Ts;
    
    % --- PLOT FIGURE 1 (Top-Down) ---
    figure(fig1);
    h_plots(i) = plot(x_data, y_data, ...
        'Color', colors{c_idx}, ...
        'LineStyle', lineStyles{l_idx}, ...
        'LineWidth', 1.5, ...
        'DisplayName', labels{i});
        
    % --- PLOT FIGURE 2 (Time Series X and Y) ---
    % Plot X
    plot(ax_x, t_data, x_data, ...
        'Color', colors{c_idx}, ...
        'LineStyle', lineStyles{l_idx}, ...
        'LineWidth', 1.2, ...
        'DisplayName', labels{i});
        
    % Plot Y (Hide duplicate legend entries to keep it clean)
    plot(ax_y, t_data, y_data, ...
        'Color', colors{c_idx}, ...
        'LineStyle', lineStyles{l_idx}, ...
        'LineWidth', 1.2, ...
        'HandleVisibility', 'off');
end

%% Formatting Figure 1 (Top-Down)
figure(fig1);
axis equal; 

max_val = max(max(abs(x_ref)), max(abs(y_ref)));
if max_val == 0; max_val = 0.1; end
padding = max_val * 1.5; 
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
ax1.YMinorTick = 'on';
set(fig1, 'Color', 'w');

%% Formatting Figure 2 (Time Series)
% Format both subplots
axesList = [ax_x, ax_y];
for k = 1:2
    axesList(k).FontName = 'Times New Roman';
    axesList(k).FontSize = 9;
    axesList(k).TickDir = 'in';
    axesList(k).XMinorTick = 'on';
    axesList(k).YMinorTick = 'on';
end

% Add legend only to the top subplot to save space
lgd2 = legend(ax_x, 'Location', 'best');
set(lgd2, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off', 'NumColumns', 2);
set(fig2, 'Color', 'w');

%% Export high-quality vector graphics
% exportgraphics(fig1, 'Simulation_TopDown.pdf', 'ContentType', 'vector');
% exportgraphics(fig2, 'Simulation_TimeSeries.pdf', 'ContentType', 'vector');

%% -------------------------------------------------------------------------
%% Print Performance Metrics Table to Command Window
%% -------------------------------------------------------------------------
fprintf('\n=================================================================================\n');
fprintf('%-10s | %-16s | %-16s | %-20s\n', 'Controller', 'RMS Error (m)', 'Peak Error (m)', 'Control Effort (rad^2 s)');
fprintf('=================================================================================\n');
for i = 1:num_sets
    if i > length(labels)
        lbl = sprintf('Data %d', i);
    else
        lbl = labels{i};
    end
    fprintf('%-10s | %-16.5f | %-16.5f | %-20.4f\n', lbl, rms_error(i), peak_error(i), ctrl_effort(i));
end
fprintf('=================================================================================\n\n');

%% Export
exportgraphics(fig1, 'Figures/sim_square_2D.pdf', 'ContentType', 'auto');
exportgraphics(fig2, 'Figures/sim_square_xy.pdf', 'ContentType', 'auto');


