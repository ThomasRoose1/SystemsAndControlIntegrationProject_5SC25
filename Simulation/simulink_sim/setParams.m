%% set parameters for the simulink simulation of the ball and plate sytem
clear; clc; close all;

%% Actuator dynamics 
Km = 11;        % [N/A]  actuator force constant
m_a = 0.118;    % [kg]   mass of actuator piston
m_p = 0.682;    % [kg]   mass of plate
Cv = 16.5;      % [Ns/m] viscous damping constant of actuator

% Total mass actuated by each actuator is its piston and 1/3 of the plate
m = m_a + 1/3 * m_p;

% Define transfer function from current I to position x
s = tf('s');
G_act = (Km/m) / (s*(s + Cv/m));  
[G_act_num, G_act_den] = tfdata(G_act, 'v');

% Actuator constraints
x_max = 0.03; % maximum movement relative to zero position, 3 cm
I_max = 3;    % maximum input current, 3A

% Actuator initial position
% x_init = -0.0285;
x_init = 0;

%% ball dynamics
g = 9.81;

