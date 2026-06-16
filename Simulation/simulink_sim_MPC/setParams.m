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

%% Load Built-in MPC Toolbox Parameters
Ts_Outer = 1/50;

% tuning
Qmpc = eye(nx); % state weight (only diag)
Rmpc = eye(nu); % control effort weight (only diag);
Nmpc = 25;   % prediction horizon
Nc_mpc = 3;  % control horizon

% Define the Prediction Plant Model
% To apply your Q = eye(4) weight, the MPC block needs to "see" all 4 states.
% We do this by setting the C matrix to an identity matrix for the MPC object.
sys_mpc_c = ss(Ac, Bc, eye(nx), zeros(nx, nu));
sys_mpc_d = c2d(sys_mpc_c, Ts_Outer, 'zoh');

% Compute terminal cost
[Kmpc, Pmpc, ~] = dlqr(sys_mpc_d.A, sys_mpc_d.B, Qmpc, Rmpc); % Terminal cost

% Create the MPC Object
% Syntax: mpc(plant, Ts, PredictionHorizon, ControlHorizon)
mpcobj = mpc(sys_mpc_d, Ts_Outer, Nmpc, Nc_mpc);

% Set Weights (Translating your Q and R matrices)
mpcobj.Weights.OutputVariables = [Qmpc(1,1) Qmpc(2,2) Qmpc(3,3) Qmpc(4,4)];      % State weights (Q)
mpcobj.Weights.ManipulatedVariables = [Rmpc(1,1) Rmpc(2,2)]; % Input weights (R)
mpcobj.Weights.ManipulatedVariablesRate = [0 0]; % No penalty on slew rate (for now)
% mpcobj.Weights.TerminalState = diag(Pmpc); % doesnt work yet

% Set Physical Constraints
max_pos = 0.15;
max_vel = 1.0;
max_angle = 10 * (pi/180); % 10 degrees in radians

% State constraints (Outputs of sys_mpc_d)
mpcobj.OV(1).Min = -max_pos;  mpcobj.OV(1).Max = max_pos; % x position
mpcobj.OV(2).Min = -max_vel;  mpcobj.OV(2).Max = max_vel; % x velocity
mpcobj.OV(3).Min = -max_pos;  mpcobj.OV(3).Max = max_pos; % y position
mpcobj.OV(4).Min = -max_vel;  mpcobj.OV(4).Max = max_vel; % y velocity

% Input constraints (Manipulated Variables: alpha, beta)
mpcobj.MV(1).Min = -max_angle; mpcobj.MV(1).Max = max_angle; 
mpcobj.MV(2).Min = -max_angle; mpcobj.MV(2).Max = max_angle; 

% turn off internal state observer
setEstimator(mpcobj, 'custom');

% Turn off interal integrators, this forces the MPC block to use exactly 4 states
setoutdist(mpcobj, 'model', tf(zeros(nx, 1)));

disp('MPC Object created successfully!');
