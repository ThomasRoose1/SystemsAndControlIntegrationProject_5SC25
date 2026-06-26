%% Make step response plot based on experiment data
clear;
clc;
close all

%% Load experiment data
load('stepA.mat');
setpoint = stepA.Y(3).Data;
pos = stepA.Y(1).Data;
Ts = 0.001;

%% Load models
load('Final_models_controllers/model_motor_A_final.mat');
Pss = G_A;
load('Final_models_controllers\Controller_motor_A_17_06.mat');
Ctf = shapeit_data.C_tf_z;

%% Create Closed-Loop System for Simulation
sys_cl = feedback(Pss * Ctf, 1);

%% Detect Rising Edges (Steps Up) and Falling Edges (Steps Down)
sp_diff = diff(setpoint);
threshold = 0.5 * max(sp_diff);

rising_edges = find(sp_diff > threshold) + 1;
falling_edges = find(sp_diff < -threshold) + 1;

min_gap = 0.5 / Ts; 
valid_steps = [];
last_step = -inf;
for k = 1:length(rising_edges)
    if rising_edges(k) - last_step > min_gap
        valid_steps(end+1) = rising_edges(k); %#ok<AGROW>
        last_step = rising_edges(k);
    end
end

num_steps = min(3, length(valid_steps));
valid_steps = valid_steps(1:num_steps);

%% Initialize IEEE Figure
figWidth = 3.5;
figHeight = 2.5; 
fig = figure('Units', 'inches', 'Position', [1, 1, figWidth, figHeight]);
hold on; grid on; box on;

colors = {'#0072BD', '#D95319', '#EDB120'}; 
h_plots = gobjects(1, num_steps);
t_pre = 0.1; 

%% Pre-allocate arrays for stats table
m_OS = zeros(1, num_steps);
m_Tr = zeros(1, num_steps);
m_Ts = zeros(1, num_steps);
m_ess = zeros(1, num_steps);

%% Extract, Align, Simulate, Plot, and Analyze Data
for i = 1:num_steps
    idx_up = valid_steps(i);
    
    idx_down_candidates = falling_edges(falling_edges > idx_up);
    if ~isempty(idx_down_candidates)
        idx_down = idx_down_candidates(1) - 10; 
    else
        idx_down = idx_up + round(1.5/Ts); 
    end
    
    start_idx = max(1, idx_up - round(t_pre / Ts));
    end_idx = min(length(setpoint), idx_down);
    
    t_seg = (start_idx:end_idx)' * Ts - (idx_up * Ts);
    sp_seg = setpoint(start_idx:end_idx);
    pos_seg = pos(start_idx:end_idx);
    
    % Filter data for t >= 0 to calculate accurate metrics (ignore t_pre)
    valid_time_idx = t_seg >= 0;
    t_eval = t_seg(valid_time_idx);
    pos_eval = pos_seg(valid_time_idx);
    sp_eval = sp_seg(valid_time_idx);
    
    % --- THE FIX: Normalize data so stepinfo understands the true baseline ---
    t_eval_norm = t_eval - t_eval(1); % Force time to start exactly at 0
    
    pos_baseline = pos_eval(1); % Where the hardware actually was when the step hit
    pos_norm = pos_eval - pos_baseline;
    
    sp_baseline = sp_eval(1); % The new target
    sp_norm_amp = sp_baseline - pos_baseline; % The true amplitude of the step (e.g., ~10mm)
    
    % Calculate Experimental Metrics on Normalized Data
    info = stepinfo(pos_norm, t_eval_norm, sp_norm_amp);
    m_OS(i)  = info.Overshoot;
    m_Tr(i)  = info.RiseTime;
    m_Ts(i)  = info.SettlingTime;
    m_ess(i) = abs(sp_eval(end) - pos_eval(end)); % Calculate SS Error using absolute raw data
    
    if i == 1
        h_sp = plot(t_seg, sp_seg, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Setpoint');
        
        t_sim = t_seg - t_seg(1);
        sp_norm_sim = sp_seg - sp_seg(1);
        y_sim_norm = lsim(sys_cl, sp_norm_sim, t_sim);
        y_sim = y_sim_norm + sp_seg(1);
        
        h_sim = plot(t_seg, y_sim, 'Color', '#7E2F8E', 'LineStyle', ':', ...
            'LineWidth', 1.5, 'DisplayName', 'Simulation');
            
        % Calculate Simulation Metrics using the exact same normalization logic
        y_sim_eval = y_sim(valid_time_idx);
        y_sim_norm_eval = y_sim_eval - y_sim_eval(1);
        
        info_sim = stepinfo(y_sim_norm_eval, t_eval_norm, sp_norm_amp);
        sim_OS  = info_sim.Overshoot;
        sim_Tr  = info_sim.RiseTime;
        sim_Ts  = info_sim.SettlingTime;
        sim_ess = abs(sp_eval(end) - y_sim_eval(end));
    else
        plot(t_seg, sp_seg, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    end
    
    h_plots(i) = plot(t_seg, pos_seg, ...
        'Color', colors{i}, ...
        'LineStyle', '-', ...
        'LineWidth', 1.2, ...
        'DisplayName', sprintf('Response %d', i));
end

%% Formatting (IEEE Standard)
xlabel('Time [s]', 'FontName', 'Times New Roman', 'FontSize', 9);
ylabel('Position [m]', 'FontName', 'Times New Roman', 'FontSize', 9);
xlim([-t_pre, max(t_seg)]);

lgd = legend([h_sp, h_sim, h_plots], 'Location', 'southeast');
set(lgd, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off');

ax = gca;
ax.FontName = 'Times New Roman';
ax.FontSize = 9;
ax.TickDir = 'in';
ax.XMinorTick = 'on';
ax.YMinorTick = 'on';
set(fig, 'Color', 'w');

%% Print Performance Metrics Table to Command Window
fprintf('\n---------------------------------------------------------------------------\n');
fprintf('%-15s | %-14s | %-13s | %-13s | %-13s\n', 'Response', 'Overshoot (%)', 'Rise Time (s)', 'Settling (s)', 'SS Error (m)');
fprintf('---------------------------------------------------------------------------\n');

% Print Simulation Row
fprintf('%-15s | %-14.2f | %-13.4f | %-13.4f | %-13.4e\n', 'Simulation', sim_OS, sim_Tr, sim_Ts, sim_ess);

% Print Experimental Rows
for i = 1:num_steps
    fprintf('%-15s | %-14.2f | %-13.4f | %-13.4f | %-13.4e\n', sprintf('Experiment %d', i), m_OS(i), m_Tr(i), m_Ts(i), m_ess(i));
end
fprintf('---------------------------------------------------------------------------\n\n');