%% Make a 2D plot of topdown view of the plate with reference and tracked performance IEEE style
clear;
clc;
close all;

%% Load data
dataset = {'square_LQRff.mat', ...
           'square_MPC.mat'};

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

%% Initialize Figure for IEEE 2-Column Format
% A standard IEEE column width is ~3.5 inches. 
figWidth = 3.5;  
figHeight = 3.0; % Aspect ratio tailored for a top-down view
fig = figure('Units', 'inches', 'Position', [1, 1, figWidth, figHeight]);
hold on;
grid on;
box on;

%% Plot Reference
% Using a black dashed line so it stands out as the target path
h_ref = plot(x_ref, y_ref, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference');

%% Load and Plot Tracked Data
% Define styles for color contrast and B&W print readability
colors = {'#0072BD', '#D95319'}; % MATLAB default blue and red
lineStyles = {'-', '-.'};        % Solid for LQR, Dash-Dot for MPC
labels = {'LQR', 'MPC'};
h_plots = gobjects(1, length(dataset)); 

for i = 1:length(dataset)
    % Load the structural data from the .mat file
    data = load(dataset{i});
    
    % Extract x and y data arrays from the timeseries (squeeze for safety)
    x_data = squeeze(data.out.simout.x.Data);
    y_data = squeeze(data.out.simout.y.Data);
    
    % Plot tracked paths
    h_plots(i) = plot(x_data, y_data, ...
        'Color', colors{i}, ...
        'LineStyle', lineStyles{i}, ...
        'LineWidth', 2, ...
        'DisplayName', labels{i});
end

%% Formatting
axis equal; % Crucial for top-down spatial plots so the plot isn't distorted

% Dynamically set limits based on the reference size + 50% padding
max_val = max(max(abs(x_ref)), max(abs(y_ref)));
if max_val == 0
    max_val = 0.1; % Fallback in case the reference is entirely zero
end
padding = max_val * 1.5; 
xlim([-padding padding]); 
ylim([-padding padding]);

% Axes Labels (IEEE prefers Times New Roman, 9pt for labels)
xlabel('\it x \rm [m]', 'FontName', 'Times New Roman', 'FontSize', 9);
ylabel('\it y \rm [m]', 'FontName', 'Times New Roman', 'FontSize', 9);

% Legend (IEEE prefers 8pt for legends, no box to save space)
lgd = legend([h_ref, h_plots], 'Location', 'northeast');
set(lgd, 'FontName', 'Times New Roman', 'FontSize', 8, 'Box', 'off');

% Axis Tick Properties
ax = gca;
ax.FontName = 'Times New Roman';
ax.FontSize = 9;
ax.TickDir = 'in';
ax.XMinorTick = 'on';
ax.YMinorTick = 'on';

% Set figure background to white 
set(fig, 'Color', 'w');

%% Export high-quality vector graphic
% Uncomment the line below to automatically save a PDF ready for LaTeX inclusion
% exportgraphics(fig, 'Tracking_Performance.pdf', 'ContentType', 'vector');