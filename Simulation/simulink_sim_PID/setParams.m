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

%% MPC controller discretization
Ts_slow = 0.01;

% Discretize continuous matrices for the fast estimation rate
sys_slow = c2d(ss(Ac, Bc, Cc, Dc), Ts_slow, 'zoh');
A_slow = sys_fast.A;
B_slow = sys_fast.B;
C_slow = sys_fast.C;

% dimensions
nx = size(A_slow,1);
ny = size(C_slow,1);
nu = size(B_slow,2);

%% Load Saved Terminal Design Matrices
% This loads K, P, HT, and hT instantly into the workspace
load('mpc_terminal_design.mat'); 

%% Compile/Define YALMIP MPC Optimizer Object
% State and Input Weights (Must match what you used to compute P offline!)
Q = eye(nx); 
R = eye(nu); 

N = 25; % Prediction Horizon
[Phi, Gamma, Omega, Psi] = mpc_obj(A_slow, B_slow, Q, R, P, N); % Uses loaded 'P'

% Setup Polyhedron Constraints (Output and Input)
max_pos = 0.2; max_angle = deg2rad(10);
Hu = [-1 0; 1 0; 0 -1; 0 1];     hu = repmat(max_angle, 4, 1);
Hy = [-1 0; 1 0; 0 -1; 0 1];     hy = repmat(max_pos, 4, 1);

Hu_bar = kron(eye(N), Hu);       hu_bar = repmat(hu, N, 1);
Hy_bar = kron(eye(N), Hy);       hy_bar = repmat(hy, N, 1);

% Define Optimization Variables
x0 = sdpvar(nx,1);
Xk = sdpvar(nx*N, 1);
Uk = sdpvar(nu*N, 1);
Yk = sdpvar(ny*N, 1);

Obj = Xk'*Omega*Xk + Uk'*Psi*Uk;

Con = [Xk == Phi*x0 + Gamma*Uk];
Con = [Con, Yk == kron(eye(N),C_slow)*Xk];
Con = [Con, Hy_bar*Yk <= hy_bar];
Con = [Con, Hu_bar*Uk <= hu_bar];
x_N = Xk(end-nx+1:end); % Extract the final predicted state step
Con = [Con, HT * x_N <= hT];

% Build the compiled optimizer object
options = sdpsettings('solver','quadprog','verbose',0);
u_current = Uk(1:nu);
MPC_sparse = optimizer(Con, Obj, options, x0, u_current);

disp('setParams initialization complete. Ready for Simulink simulation.');