clear all; close all; clc;

%% Load data
load zsine.mat
load square_PID_last.mat
load square_MPC_z0_OFF.mat
load square_MPC_z0_ON.mat

%% Plate definition
x_plate = [-0.195  0.195  0.195 -0.195 -0.195];
y_plate = [-0.195 -0.195  0.195  0.195 -0.195];

%% Reference Square definition
x_ref = [-0.1  0.1  0.1 -0.1 -0.1];
y_ref = [-0.1 -0.1  0.1  0.1 -0.1];

%% Extract data
t_zsine = zsine.X(1).Data;
z_ref = zsine.Y(14).Data;
z_sat = zsine.Y(15).Data;

t_PID = square_PID_last.X.Data;
x_sqr_PID = square_PID_last.Y(3).Data;
y_sqr_PID = square_PID_last.Y(4).Data;

t_MPC_OFF = square_MPC_z0_OFF.X(1).Data;
z_ref_MPC = square_MPC_z0_OFF.Y(14).Data;
z_sat_MPC_OFF = square_MPC_z0_OFF.Y(15).Data;

t_MPC_ON = square_MPC_z0_ON.X(1).Data;
z_sat_MPC_ON = square_MPC_z0_ON.Y(15).Data;

%% Processing

%% Plot data

figure(1); % Figure 1 is the PID square reference
gcf.Position = [100 100 600 600];

hold on
plot(x_plate, y_plate,'k-','LineWidth',2);
plot(x_ref,y_ref,'k--','LineWidth',1.5);
plot(x_sqr_PID,y_sqr_PID,'b-','LineWidth',1.5);
hold off

axis equal
axis square

xlim([-0.3 0.3]);
ylim([-0.3 0.3]);
xlabel('X [m]');
ylabel('Y [m]');
legend('PLATE','Reference','PID Square')
grid on

figA = figure(2); % Figure 2 is the z sine alone

hold on
plot(t_zsine, z_ref*1000, 'k-','LineWidth',1.5)
plot(t_zsine, z_sat*1000, '-','LineWidth',1.5)
hold off

xlabel('Time [s]')
xlim([0 t_zsine(end)])
ylabel('Z [mm]')
legend('Reference','Z measured')
grid on

format_ieee_figure(figA)
drawnow
exportgraphics(figA,'Z_Sine.pdf','ContentType','vector')

figB = figure(3); % Figure 2 is the z sine alone

hold on
plot(t_MPC_OFF, z_ref_MPC*1000, 'k-','LineWidth',1.5)
plot(t_MPC_OFF, z_sat_MPC_OFF*1000, '-','LineWidth',1.5)
plot(t_MPC_ON, z_sat_MPC_ON*1000, '-','LineWidth',1.5)
hold off

xlabel('Time [s]')
% xlim([0 t_zsine(end)])
ylabel('Z [mm]')
legend('Z_{ref}','Z_{sat} MPC OFF', 'Z_{sat} MPC ON')
grid on

% format_ieee_figure(figB)
% drawnow


%% Functions
function format_ieee_figure(fig)

figure(fig);

%% Set the physical size of the exported figure
fig.Units = 'centimeters';
fig.Position = [2 2 8.2 6.2];     % [left bottom width height]

%% Format axes
ax = gca;

ax.FontName = 'Times New Roman';
ax.FontSize = 10;
ax.LineWidth = 1.0;

% Remove excess white margins
ax.LooseInset = ax.TightInset;

ax.Box = 'on';

%% Labels
ax.XLabel.FontSize = 10;
ax.YLabel.FontSize = 10;

%% Title
ax.Title.FontSize = 15;

%% Legend
lgd = findobj(fig,'Type','Legend');
if ~isempty(lgd)
    lgd.FontSize = 10;
    lgd.FontName = 'Times New Roman';
end

grid on

drawnow

end