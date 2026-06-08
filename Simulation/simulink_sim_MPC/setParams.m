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
g = 9.81;                  % Gravity (m/s^2)
km_factor = (5/7) * g;     % Rolling ball constant (~7.0071)

% Continuous Matrices (4 states, 2 inputs, 2 outputs)
Ac = [0 1 0 0;
      0 0 0 0;
      0 0 0 1;
      0 0 0 0];
  
Bc = [0          0;
      0          km_factor;
      0          0;
     -km_factor  0];
  
Cc = [1 0 0 0;
      0 0 1 0];
  
Dc = [0 0;
      0 0];

% dimensions
nx = size(Ac, 1); % Number of states
nu = size(Bc, 2); % Number of inputs

%% State observer 
Ts_fast = 0.001; % 1000 Hz sample time matching the simulation rate

% Discretize continuous matrices for the fast estimation rate
sys_fast = c2d(ss(Ac, Bc, Cc, Dc), Ts_fast, 'zoh');
A_fast = sys_fast.A;
B_fast = sys_fast.B;
C_fast = sys_fast.C;

% Define your fast observer poles (safely inside the unit circle)
observer_poles = [0.90, 0.91, 0.92, 0.93];

% Compute the Observer Gain Matrix L
L = place(A_fast', C_fast', observer_poles)';

%% Load MPC params
Ts_Outer = 0.01;
% Tuning
Qmpc = eye(nx); % State weighting
Rmpc = 200*eye(nu); % Input weighting
Nmpc = 25; % prediction horizon

% state constraints 
max_pos = 0.15;
max_vel = 1;

% Load MPC params
MPC_params = compute_mpc_params(Qmpc,Rmpc,Nmpc, Ts_Outer, max_pos, max_vel);

% Create a Simulink Bus Object automatically from 'params' structure
Simulink.Bus.createObject(MPC_params);

% Rename the generic generated object to a clear type name for model
MPC_params_bus = slBus1; 
clear slBus1;

% Force the elements of the bus to match explicit double-precision types
for i = 1:length(MPC_params_bus.Elements)
    MPC_params_bus.Elements(i).DataType = 'double';
end
