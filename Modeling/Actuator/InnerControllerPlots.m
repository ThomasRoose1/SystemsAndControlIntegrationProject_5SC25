%% Makes plots for a inner loop controller based on parametric model and shapeit controller
clear; clc;
close all;

%% Load plant and controller
% plant
load('model_motor_A.mat');
P_ss = G_A;
P_tf = tf(P_ss);

% controller
load('Controllers\controller_A.mat');
C_tf = shapeit_data.C_tf_z;

%% Compute Open Loop
L_tf = P_tf * C_tf;

%% IEEE Style Nyquist Plot Configuration
% Set your desired window size / axis limits here: [xmin, xmax, ymin, ymax]
plot_window = [-1.8, 0.5, -1.0, 1.0]; 

% Create figure sized for IEEE single-column width (approx 9 cm wide)
fig = figure('Units', 'centimeters', 'Position', [10, 10, 9, 7.5]); 
hold on;
grid on;

% Extract Nyquist Data
[re, im, w] = nyquist(L_tf);
re = squeeze(re);
im = squeeze(im);

% Plot the positive frequencies (solid blue line)
plot(re, im, 'b-', 'LineWidth', 1.5, 'DisplayName', 'L(j\omega)');

% Plot the negative frequencies (solid blue line, hidden from legend)
plot(re, -im, 'b-', 'LineWidth', 1.5, 'HandleVisibility', 'off');

% Create the 6dB Robustness Margin Circle (Ms = 2, radius = 0.5)
radius = 0.5;
theta = linspace(0, 2*pi, 200);
x_circle = -1 + radius * cos(theta);
y_circle = radius * sin(theta);

% Plot the circle (dashed black line)
plot(x_circle, y_circle, 'k--', 'LineWidth', 1.2, 'DisplayName', '6 dB Margin');

% Mark the Critical Point (-1, 0)
plot(-1, 0, 'k+', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', '-1 Point');

% Formatting to IEEE Standards
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10);
xlabel('Re', 'FontName', 'Times New Roman', 'FontSize', 10);
ylabel('Im', 'FontName', 'Times New Roman', 'FontSize', 10);

% Set axes lines through origin
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
box on; 

% Apply the user-defined window size
axis(plot_window);

hold off;

%% export
% check if folder exist, make if not
folderName = 'Figures';
if ~isfolder(folderName)
    mkdir(folderName);
end
exportgraphics(fig, 'Figures/Nyquist_A.pdf', 'ContentType','vector');
