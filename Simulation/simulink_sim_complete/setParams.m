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

% Load motor A
load("model_motor_A_final.mat");

% Load motor B
load("model_motor_B_final.mat");
G_B = G_A;

% Load motor C
load("model_motor_C_final.mat");
G_C = G_A;


% Actuator constraints
x_max = 0.03; % maximum movement relative to zero position, 3 cm
I_max = 3;    % maximum input current, 3A
inner_I_sat = 1;

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

% ball initial conditions
x0 = [0, 0, 0, 0];

%% State observer 
Ts_Inner = 0.001; % 1000 Hz sample time matching the simulation rate

Q_kf = diag([1, 100, 1, 100]);
R_kf = diag([0.1, 0.1]);

%% LQR design
% Discrete-Time Model Conversion
sys_c = ss(Ac, Bc, Cc, Dc);
sys_d = c2d(sys_c, Ts_Inner, 'zoh');
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;

% Tune Q_lqr to penalize position error vs velocity.
% Tune R_lqr to limit plate tilt acceleration/effort.
Q_lqr = diag([100, 10, 100, 10]); % [x, x_dot, y, y_dot]
R_lqr = diag([700, 700]);             % [alpha, beta]

% Compute LQR gain
K_lqr = dlqr(Ad, Bd, Q_lqr, R_lqr);
% K_lqr = -K_lqr;


%% Load Built-in MPC Toolbox Parameters
Ts_Outer = 1/50;

% tuning
Qmpc = diag([100, 10, 100, 10]); % [x, x_dot, y, y_dot]
Rmpc = diag([100, 100]);
dQmpc = diag([200, 200]);
Nmpc = 25;   % prediction horizon
Nc_mpc = 3;  % control horizon

% Define the Prediction Plant Model
sys_mpc_c = ss(Ac, Bc, eye(nx), zeros(nx, nu));
sys_mpc_d = c2d(sys_mpc_c, Ts_Outer, 'zoh');

% Compute terminal cost
[Kmpc, Pmpc, ~] = dlqr(sys_mpc_d.A, sys_mpc_d.B, Qmpc, Rmpc); % Terminal cost

% Create the MPC Object
% Syntax: mpc(plant, Ts, PredictionHorizon, ControlHorizon)
mpcobj = mpc(sys_mpc_d, Ts_Outer, Nmpc, Nc_mpc);

% Set Weights (Translating your Q and R matrices)
mpcobj.Weights.OutputVariables = [Qmpc(1,1) Qmpc(2,2) Qmpc(3,3) Qmpc(4,4)];      % State weights (Q)
% mpcobj.Weights.OutputVariables = [0 0 0 0];      % State weights (Q)
mpcobj.Weights.ManipulatedVariables = [Rmpc(1,1) Rmpc(2,2)]; % Input weights (R)
mpcobj.Weights.ManipulatedVariablesRate = [dQmpc(1,1) dQmpc(2,2)]; 
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

%% Load robust controller
load('optimalK.mat');
A_robust = K.A;
B_robust = K.B;
C_robust = K.C;
D_robust = K.D;

nK_robust = size(A_robust,1);

Kaw_robust = 0.02;

alpha_lim = deg2rad(6.0);
beta_lim  = deg2rad(6.0);

u_min_robust = [-alpha_lim; -beta_lim];
u_max_robust = [ alpha_lim;  beta_lim];

aw_reg = 1e-6;
Baw_robust = C_robust' / (C_robust*C_robust' + aw_reg*eye(2));

%% Ref parameters
R = 0.1;
Tcircle = 5; %s
Ncircles = 2;
enable = 1;
% [r, a] = generate_circular_trajectory(R, Tcircle, Ncircles, Ts_Inner);

Tsquare = 10; % Sample time for square trajectory
N_squares = 3; % Number of squares in the trajectory
[r, a] = generate_square_trajectory(R, Tsquare, N_squares, Ts_Inner);

