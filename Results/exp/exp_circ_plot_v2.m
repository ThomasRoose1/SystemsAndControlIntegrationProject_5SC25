%% Make 2D top-down view, X/Y time tracking, Angle tracking, and PSD plots for Experimental Data
clear;
clc;
close all;

%% 1. Configuration & Data Loading
dataset = {'square_PID_last.mat', ...
           'OptimalHinf.mat', ...
           'square_LQRff_last.mat', ...
           'square_MPC_z0_OFF.mat',
           };
labels  = {'PID', 'Hinf', 'LQR', 'MPC'}; 
Ts = 0.001; % Sampling time

% --- MANUAL CROP SETTINGS ---
% Define the exact start time (in seconds) for each dataset.
% The order must strictly match the 'dataset' array above.
start_times = [4, 1, 1.4, 2.7]; 

% --- NEW: TIME PLOT RANGE ---
% Define the x-axis limits [start, end] for time-domain plots (Fig 2 & Fig 3).
% Note: Time 0 here represents the exact moment the trajectory begins.
% Set to [] to automatically show the full plot_duration.
time_plot_range = [0, 20]; 
% ---------------------------------

%% 2. Trajectory & Plotting Parameters
Tsquare = 10;       
N_squares_exp = 3;  
R = 0.1;            
Tstandstill = 1.0;   
squares_to_plot = 3; 

% This calculates the total duration to plot. It will be universally applied 
% to all datasets starting from their respective manual 'start_times'.
T_intro = (Tsquare / 4) + Tstandstill; 
T_one_square = 4 * ((Tsquare / 4) + Tstandstill);
plot_duration = T_intro + (squares_to_plot * T_one_square);

%% 3. Generate the True Reference Trajectory
[r_ref, ~] = generate_square_trajectory(R, Tsquare, N_squares_exp, Ts);
ref_x_full = r_ref(1, :)'; 
ref_y_full = r_ref(3, :)'; 
t_ref_full = (0:length(ref_x_full)-1)' * Ts;
valid_ref_idx = t_ref_full <= plot_duration;
ref_x_plot = ref_x_full(valid_ref_idx);
ref_y_plot = ref_y_full(valid_ref_idx);
t_ref_plot = t_ref_full(valid_ref_idx); 

%% 4. Formatting Settings & Metric Pre-allocation
colors = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30'}; 
lineStyles = {'-', '-.', '--', ':'}; 
num_sets = length(dataset);

% Pre-allocate metrics arrays
rms_error   = zeros(1, num_sets);
peak_error  = zeros(1, num_sets);
ctrl_effort = zeros(1, num_sets);

%% Initialize Figure 1: Top-Down Tracking Performance
figWidth = 3.5;  % IEEE single column width
fig1 = figure('Units', 'inches', 'Position', [1, 1, figWidth, 3.0]);
hold on; grid on; box on;
h_ref = plot(ref_x_plot, ref_y_plot, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference');
h_plots = gobjects(1, num_sets);

%% Initialize Figure 2: X and Y Time Tracking
fig2 = figure('Units', 'inches', 'Position', [5, 1, figWidth, 4.0]);
ax2_x = subplot(2,1,1); hold on; grid on; box on;
plot(t_ref_plot, ref_x_plot, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference');
ylabel('\it x \rm [m]', 'FontName', 'Times New Roman', 'FontSize', 9);
ax2_y = subplot(2,1,2); hold on; grid on; box on;
plot(t_ref_plot, ref_y_plot, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off'); 
xlabel('Time [s]', 'FontName', 'Times New Roman', 'FontSize', 9);
ylabel('\it y \rm [m]', 'FontName', 'Times New Roman', 'FontSize', 9);

%% Initialize Figure 3: Commanded vs Measured Angles
fig3 = figure('Units', 'inches', 'Position', [9, 1, figWidth, 4.0]);
ax3_a = subplot(2,1,1); hold on; grid on; box on;
ylabel('\alpha [deg]', 'FontName', 'Times New Roman', 'FontSize', 9);
ax3_b = subplot(2,1,2); hold on; grid on; box on;
xlabel('Time [s]', 'FontName', 'Times New Roman', 'FontSize', 9);
ylabel('\beta [deg]', 'FontName', 'Times New Roman', 'FontSize', 9);

%% Initialize Figure 4: Power Spectral Density (PSD)
fig4 = figure('Units', 'inches', 'Position', [1, 5, figWidth, 2.5]);
hold on; grid on; box on;
xlabel('Frequency [Hz]', 'FontName', 'Times New Roman', 'FontSize', 9);
ylabel('PSD [dB/Hz]', 'FontName', 'Times New Roman', 'FontSize', 9);

%% Load, Manually Crop, Plot, and Analyze Data Loop
for i = 1:num_sets
    c_idx = mod(i-1, length(colors)) + 1;
    l_idx = mod(i-1, length(lineStyles)) + 1;
    
    data = load(dataset{i});
    fields = fieldnames(data);
    internal_var = fields{1}; 
    measurements = data.(internal_var).Y;
    
    raw_ac = measurements(1).Data;
    raw_bc = measurements(2).Data;
    raw_x  = measurements(3).Data;
    raw_y  = measurements(4).Data;
    raw_am = measurements(5).Data;
    raw_bm = measurements(6).Data;
    
    N = length(raw_ac);
    t_raw = (0:N-1)' * Ts; 
    
    % --- MANUAL CROP DATA ---
    t_start = start_times(i);
    t_end   = t_start + plot_duration;
    valid_idx = (t_raw >= t_start) & (t_raw <= t_end);
    
    rad2deg_conv = 180 / pi;
    
    % Shift time so everything starts at 0 to overlay perfectly with the ref
    t_curr = t_raw(valid_idx) - t_start; 
    
    ac_curr = raw_ac(valid_idx) * rad2deg_conv;
    bc_curr = raw_bc(valid_idx) * rad2deg_conv;
    am_curr = raw_am(valid_idx) * rad2deg_conv;
    bm_curr = raw_bm(valid_idx) * rad2deg_conv;
    x_curr  = raw_x(valid_idx);
    y_curr  = raw_y(valid_idx);
    
    % --- PLOT FIGURE 1 (Top-Down) ---
    figure(fig1);
    h_plots(i) = plot(x_curr, y_curr, ...
        'Color', colors{c_idx}, ...
        'LineStyle', lineStyles{l_idx}, ...
        'LineWidth', 1.2, ...
        'DisplayName', labels{i});
        
    % --- PLOT FIGURE 2 (Time Series X and Y) ---
    plot(ax2_x, t_curr, x_curr, 'Color', colors{c_idx}, 'LineStyle', lineStyles{l_idx}, 'LineWidth', 1.2, 'DisplayName', labels{i});
    plot(ax2_y, t_curr, y_curr, 'Color', colors{c_idx}, 'LineStyle', lineStyles{l_idx}, 'LineWidth', 1.2, 'HandleVisibility', 'off');
        
    % --- PLOT FIGURE 3 (Angles) ---
    % Alpha
    plot(ax3_a, t_curr, ac_curr, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'HandleVisibility', 'off'); 
    plot(ax3_a, t_curr, am_curr, 'Color', colors{c_idx}, 'LineStyle', lineStyles{l_idx}, 'LineWidth', 1.2, 'DisplayName', sprintf('%s Measured', labels{i}));
    % Beta
    plot(ax3_b, t_curr, bc_curr, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'HandleVisibility', 'off');
    plot(ax3_b, t_curr, bm_curr, 'Color', colors{c_idx}, 'LineStyle', lineStyles{l_idx}, 'LineWidth', 1.2, 'DisplayName', sprintf('%s Measured', labels{i}));
    
    % --- PLOT FIGURE 4 (PSD) ---
    figure(fig4);
    Fs = 1/Ts; 
    [pxx, f] = pwelch(am_curr, [], [], [], Fs);
    plot(f, 10*log10(pxx), 'Color', colors{c_idx}, 'LineStyle', lineStyles{l_idx}, 'LineWidth', 1.2, 'DisplayName', labels{i});
    
    % --- CALCULATE PERFORMANCE METRICS ---
    min_len = min(length(x_curr), length(ref_x_plot));
    
    cx = x_curr(1:min_len); cx = cx(:);
    cy = y_curr(1:min_len); cy = cy(:);
    curr_ref_x = ref_x_plot(1:min_len); curr_ref_x = curr_ref_x(:);
    curr_ref_y = ref_y_plot(1:min_len); curr_ref_y = curr_ref_y(:);
    
    err_x = cx - curr_ref_x;
    err_y = cy - curr_ref_y;
    err_mag = sqrt(err_x.^2 + err_y.^2);
    
    rms_error(i) = sqrt(mean(err_mag.^2));
    peak_error(i) = max(err_mag);
    ctrl_effort(i) = sum(ac_curr(1:min_len).^2 + bc_curr(1:min_len).^2) * Ts;
end

%% Formatting Figure 1 (Top-Down)
figure(fig1);
axis equal; 
max_val = max(max(abs(ref_x_plot)), max(abs(ref_y_plot)));
if max_val == 0; max_val = 0.1; end
padding = max_val * 1.5; 
xlim([-padding padding]); 
ylim([-padding padding]);
lgd1 = legend([h_ref, h_plots], 'Location', 'northeast');
set(lgd1, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off');
ax1 = gca;
ax1.FontName = 'Times New Roman';
ax1.FontSize = 9;
ax1.TickDir = 'in';
ax1.XMinorTick = 'on';
ax1.YMinorTick = 'on';
set(fig1, 'Color', 'w');

%% Formatting Figure 2 (Time Series X/Y)
axesList = [ax2_x, ax2_y];
for k = 1:2
    axesList(k).FontName = 'Times New Roman';
    axesList(k).FontSize = 9;
    axesList(k).TickDir = 'in';
    axesList(k).XMinorTick = 'on';
    axesList(k).YMinorTick = 'on';
    if ~isempty(time_plot_range)
        xlim(axesList(k), time_plot_range);
    end
end
lgd2 = legend(ax2_x, 'Location', 'best');
set(lgd2, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off', 'NumColumns', 2);
set(fig2, 'Color', 'w');

%% Formatting Figure 3 (Angles)
% Add dummy line for 'Commanded' legend entry in Alpha plot
plot(ax3_a, NaN, NaN, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'DisplayName', 'Commanded');
axesList = [ax3_a, ax3_b];
for k = 1:2
    axesList(k).FontName = 'Times New Roman';
    axesList(k).FontSize = 9;
    axesList(k).TickDir = 'in';
    axesList(k).XMinorTick = 'on';
    axesList(k).YMinorTick = 'on';
    if ~isempty(time_plot_range)
        xlim(axesList(k), time_plot_range);
    end
end
lgd3 = legend(ax3_a, 'Location', 'best');
set(lgd3, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off');
set(fig3, 'Color', 'w');

%% Formatting Figure 4 (PSD)
figure(fig4);
xlim([0 50]); 
lgd4 = legend('Location', 'northeast');
set(lgd4, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off');
ax4 = gca;
ax4.FontName = 'Times New Roman';
ax4.FontSize = 9;
ax4.TickDir = 'in';
ax4.XMinorTick = 'on';
ax4.YMinorTick = 'on';
set(fig4, 'Color', 'w');

%% -------------------------------------------------------------------------
%% Print Performance Metrics Table
%% -------------------------------------------------------------------------
fprintf('\n=================================================================================\n');
fprintf('%-10s | %-16s | %-16s | %-20s\n', 'Controller', 'RMS Error (m)', 'Peak Error (m)', 'Control Effort (deg^2 s)');
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

%% Export high-quality vector graphics
if ~exist('Figures', 'dir')
    mkdir('Figures');
end
exportgraphics(fig1, 'Figures/exp_square_2D.pdf', 'ContentType', 'vector');
exportgraphics(fig2, 'Figures/exp_square_xy.pdf', 'ContentType', 'vector');
exportgraphics(fig3, 'Figures/exp_square_angles.pdf', 'ContentType', 'vector');
exportgraphics(fig4, 'Figures/exp_square_psd.pdf', 'ContentType', 'vector');