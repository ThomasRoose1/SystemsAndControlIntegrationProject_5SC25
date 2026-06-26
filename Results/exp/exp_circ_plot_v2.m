%% Make 2D top-down view and Angle tracking plots with Automatic Reference Cropping
clear;
clc;
close all;

%% 1. Configuration & Data Loading
dataset = {'square_LQRff_last.mat', ...
           'square_MPC_z0_OFF.mat', ...
           'square_PID_last.mat', ...
           'OptimalHinf.mat'};
labels  = {'LQR', 'MPC', 'PID', 'Hinf'}; 

Ts = 0.001; % Sampling time

%% 2. Trajectory & Plotting Parameters
Tsquare = 10;       
N_squares_exp = 3;  
R = 0.1;            
Tstandstill = 1.0;   

squares_to_plot = 3; 

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

%% Formatting Settings & Metric Pre-allocation
colors = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30'}; 
lineStyles = {'-', '-.', '--', ':'}; 

num_sets = length(dataset);
t_data  = cell(1, num_sets);
alpha_c = cell(1, num_sets);
beta_c  = cell(1, num_sets);
alpha_m = cell(1, num_sets);
beta_m  = cell(1, num_sets);
x_data  = cell(1, num_sets);
y_data  = cell(1, num_sets);

% Pre-allocate metrics arrays
rms_error   = zeros(1, num_sets);
peak_error  = zeros(1, num_sets);
ctrl_effort = zeros(1, num_sets);

%% Initialize Figure 1: Preview X and Y over time (Verify Sync)
figPreview = figure('Name', 'Preview: Verify Auto-Sync', 'Units', 'inches', 'Position', [1, 5, 6, 4]);
ax_x = subplot(2,1,1); hold on; grid on; box on; ylabel('Raw x [m]', 'FontName', 'Times New Roman');
ax_y = subplot(2,1,2); hold on; grid on; box on; ylabel('Raw y [m]', 'FontName', 'Times New Roman'); xlabel('Time [s]', 'FontName', 'Times New Roman');

%% Load, Sync, Auto-Crop, and Analyze Data Loop
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
    
    % --- AUTO-SYNC REFERENCE ---
    baseline_ac = mean(raw_ac(1:50));
    baseline_bc = mean(raw_bc(1:50));
    
    dev_ac = abs(raw_ac - baseline_ac);
    dev_bc = abs(raw_bc - baseline_bc);
    
    dev_threshold = 0.005; 
    over_thresh = find(dev_ac > dev_threshold | dev_bc > dev_threshold);

    trigger_idx = 1; 
    debounce_samples = 50; 
    
    if length(over_thresh) > debounce_samples
        for k = 1:(length(over_thresh) - debounce_samples)
            if over_thresh(k + debounce_samples) - over_thresh(k) == debounce_samples
                trigger_idx = over_thresh(k);
                break;
            end
        end
    end
    t_trigger = t_raw(trigger_idx);
    
    % --- PLOT PREVIEW ---
    plot(ax_x, t_raw, raw_x, 'Color', colors{c_idx}, 'DisplayName', labels{i});
    plot(ax_y, t_raw, raw_y, 'Color', colors{c_idx}, 'DisplayName', labels{i});
    
    if i == 1
        t_ref_aligned = t_ref_full + t_trigger;
        plot(ax_x, t_ref_aligned, ref_x_full, 'k--', 'LineWidth', 1.5, 'DisplayName', 'True Ref');
        plot(ax_y, t_ref_aligned, ref_y_full, 'k--', 'LineWidth', 1.5, 'DisplayName', 'True Ref');
    end
    
    % --- AUTO-CROP DATA ---
    t_start = t_trigger;
    t_end   = t_trigger + plot_duration;
    valid_idx = (t_raw >= t_start) & (t_raw <= t_end);
    
    rad2deg_conv = 180 / pi;
    
    t_data{i}  = t_raw(valid_idx) - t_start; 
    alpha_c{i} = raw_ac(valid_idx) * rad2deg_conv;
    beta_c{i}  = raw_bc(valid_idx) * rad2deg_conv;
    x_data{i}  = raw_x(valid_idx);
    y_data{i}  = raw_y(valid_idx);
    alpha_m{i} = raw_am(valid_idx) * rad2deg_conv;
    beta_m{i}  = raw_bm(valid_idx) * rad2deg_conv;
    
    % --- CALCULATE PERFORMANCE METRICS ---
    % 1. Align arrays perfectly (in case of 1 sample mismatch due to rounding)
    min_len = min(length(x_data{i}), length(ref_x_plot));
    
    % Force all data into strict column vectors to prevent NxN grid expansion
    curr_x = x_data{i}(1:min_len); curr_x = curr_x(:);
    curr_y = y_data{i}(1:min_len); curr_y = curr_y(:);
    
    curr_ref_x = ref_x_plot(1:min_len); curr_ref_x = curr_ref_x(:);
    curr_ref_y = ref_y_plot(1:min_len); curr_ref_y = curr_ref_y(:);
    
    % 2. Calculate Euclidean distance error at every time step
    err_x = curr_x - curr_ref_x;
    err_y = curr_y - curr_ref_y;
    err_mag = sqrt(err_x.^2 + err_y.^2);
    
    % 3. Extract Error Metrics
    rms_error(i) = sqrt(mean(err_mag.^2));
    peak_error(i) = max(err_mag);
    
    % 4. Calculate Control Effort (Discrete Integral of Squared Inputs)
    % Also using (:) here just to be completely safe
    ctrl_effort(i) = sum(alpha_c{i}(:).^2 + beta_c{i}(:).^2) * Ts;
end
legend(ax_x, 'Location', 'best');

%% Initialize Figure 2: Top-down Tracking Performance (IEEE)
figWidth = 3.5;  
figHeight1 = 3.0; 
fig2 = figure('Units', 'inches', 'Position', [1, 1, figWidth, figHeight1]);
hold on; grid on; box on;

h_ref = plot(ref_x_plot, ref_y_plot, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference');
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
padding = R * 1.3; 
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

%% Initialize Figure 3: Commanded vs Measured Angles (DEGREES)
figHeight2 = 4.0; 
fig3 = figure('Units', 'inches', 'Position', [5, 1, figWidth, figHeight2]);

ax3_1 = subplot(2,1,1);
hold on; grid on; box on;
for i = 1:num_sets
    c_idx = mod(i-1, length(colors)) + 1;
    l_idx = mod(i-1, length(lineStyles)) + 1;
    
    plot(t_data{i}, alpha_c{i}, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'HandleVisibility', 'off'); 
    plot(t_data{i}, alpha_m{i}, 'Color', colors{c_idx}, 'LineStyle', lineStyles{l_idx}, 'LineWidth', 1.2, 'DisplayName', sprintf('%s Measured', labels{i}));
end
plot(NaN, NaN, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'DisplayName', 'Commanded');
ylabel('\alpha [deg]', 'FontName', 'Times New Roman', 'FontSize', 9);
lgd2_1 = legend('Location', 'best');
set(lgd2_1, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off');

ax3_2 = subplot(2,1,2);
hold on; grid on; box on;
for i = 1:num_sets
    c_idx = mod(i-1, length(colors)) + 1;
    l_idx = mod(i-1, length(lineStyles)) + 1;
    
    plot(t_data{i}, beta_c{i}, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'HandleVisibility', 'off');
    plot(t_data{i}, beta_m{i}, 'Color', colors{c_idx}, 'LineStyle', lineStyles{l_idx}, 'LineWidth', 1.2, 'DisplayName', sprintf('%s Measured', labels{i}));
end
xlabel('Time [s]', 'FontName', 'Times New Roman', 'FontSize', 9);
ylabel('\beta [deg]', 'FontName', 'Times New Roman', 'FontSize', 9);

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
ylabel('PSD [dB/Hz (deg^2)]', 'FontName', 'Times New Roman', 'FontSize', 9);
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

%% -------------------------------------------------------------------------
%% Print Performance Metrics Table
%% -------------------------------------------------------------------------
fprintf('\n=================================================================================\n');
fprintf('%-10s | %-16s | %-16s | %-20s\n', 'Controller', 'RMS Error (m)', 'Peak Error (m)', 'Control Effort (deg^2 s)');
fprintf('=================================================================================\n');
for i = 1:num_sets
    fprintf('%-10s | %-16.5f | %-16.5f | %-20.2f\n', labels{i}, rms_error(i), peak_error(i), ctrl_effort(i));
end
fprintf('=================================================================================\n\n');