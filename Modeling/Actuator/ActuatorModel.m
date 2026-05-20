%% Define a model for the actuators
clear; clc; close all;

%% parameters
Km = 11;        % [N/A]  actuator force constant
m_a = 0.118;    % [kg]   mass of actuator piston
m_p = 0.682;    % [kg]   mass of plate
Cv = 16.5;      % [Ns/m] viscous damping constant of actuator

%% Total mass actuated by each actuator is its piston and 1/3 of the plate
m = m_a + 1/3 * m_p;

%% Define transfer function from current I to position x
s = tf('s');
G = (Km/m) / (s*(s + Cv/m));  

%% Plot plant
figure;
bode(G); grid on;
title('Bode plot of actuator plant $\frac{X(s)}{I(s)}$', 'Interpreter','latex')
    