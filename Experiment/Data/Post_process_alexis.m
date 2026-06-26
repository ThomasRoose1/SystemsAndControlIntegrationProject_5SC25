clear all; close all; clc;

%% Load data
load zsine.mat
load square_PID_last.mat

%% Extract data
L = zsine.Description.Measurement.Length;
startT = zsine.Description.Measurement.StartTimestamp;
stopT = zsine.Description.Measurement.StopTimestamp;
z_dataX = zsine.X.Data;
z_dataY = zsine.Y.Data;

dataX = square_PID_last.X.Data;
dataY = square_PID_last.Y.Data;

%% Plot data

figure(1);
plot(z_dataX,z_dataY);
xlabel('Time');
ylabel('Amplitude');

figure(2);
plot(dataX,data);
xlabel('Time');
ylabel('Amplitude');