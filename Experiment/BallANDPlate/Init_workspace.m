%% load into work space before building

fs = 1000;
fn = fs/2;

A = 2; % amplitude of multisine         CHANGE IF DESIRED
tlength = 200; % number of seconds       CHANGE IF DESIRED
N = tlength*fs;
P = 1; % periods

Ts_Outer = 0.02;                      %  must be integer multiple of 0.001
Ts_Inner = 0.001;


uA = multisine(0,fn,fs,N,A);
uA = repmat(uA,[1,P]);  % repeat periods if we want f-domain averaging

uB = multisine(0,fn,fs,N,A);
uB= repmat(uB,[1,P]);

uC = multisine(0,fn,fs,N,A);
uC = repmat(uC,[1,P]);

path = quintic(-0.0289,3,0,1/fs);

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
nx = size(Ac,1);
nu = size(Bc, 2); 
  
%% Saturations
plate_angle_sat = deg2rad(10);
ball_pos_sat = 0.2;
ball_vel_sat = 1;
inner_I_sat = 1; % sat on inner loop integrator windup [A]

%% Observer
Q_kf = diag([1, 100, 1, 100]);
R_kf = diag([0.1, 0.1]);

% R_kf = diag ([1e-6, 1e-6]);
% sigma_acc = 1;
% 
% Q1D = sigma_acc^2 * [Ts_Outer^4/4, Ts_Outer^3/2;
%                     Ts_Outer^3/2, Ts_Outer^2];
%                 
% Q_kf = blkdiag(Q1D,Q1D);

Ts_fast = 0.001; % 1000 Hz sample time matching the simulation rate

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
K_lqr = -K_lqr;

%% Load Built-in MPC Toolbox Parameters
Ts_Outer = 1/50;

% tuning
% Qmpc = eye(nx); % state weight (only diag)
% Rmpc = 100*eye(nu); % control effort weight (only diag);
Qmpc = diag([100, 10, 100, 10]); % [x, x_dot, y, y_dot]
Rmpc = diag([100, 100]);
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

%% load references
R = 0.1;
Tcircle = 5;
N_circles = 10;
[r, a] = generate_circular_trajectory(R, Tcircle, N_circles, Ts_Inner);

%% done
disp('Initialization complete. Ready for Simulink simulation.');

