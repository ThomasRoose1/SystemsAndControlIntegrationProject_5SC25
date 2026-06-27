clear all; close all; clc;

%% Load data
load zsine.mat
load square_PID_last.mat

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
x_sqr_PID = square_PID_last.Y(14).Data;
y_sqr_PID = square_PID_last.Y(15).Data;

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

figure(2) % Figure 2 is the z sine alone

hold on
plot(t_zsine, z_ref*1000, 'b','LineWidth',1.5)
plot(t_zsine, z_sat*1000, 'r','LineWidth',1.5)
hold off

xlabel('Time [s]')
ylabel('Z [mm]')
legend('Z_{ref}','Z_{sat}')
grid on
