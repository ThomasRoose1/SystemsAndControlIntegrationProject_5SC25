%% load into work space before building

fs = 1000;
fn = fs/2;

A = 2; % amplitude of multisine         CHANGE IF DESIRED
tlength = 200; % number of seconds       CHANGE IF DESIRED
N = tlength*fs;
P = 1; % periods

Ts_Outer = 0.033;                      %  must be integer multiple of 0.001


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
  
%% Plate angle saturation limit
plate_angle_sat = deg2rad(10);

%% Observer
Q_kf = diag([1, 100, 1, 100]);
R_kf = diag([0.1, 0.1]);

%% State observer 
Ts_fast = 0.001; % 1000 Hz sample time matching the simulation rate

% Discretize continuous matrices for the fast estimation rate
sys_fast = c2d(ss(Ac, Bc, Cc, Dc), Ts_fast, 'zoh');
A_fast = sys_fast.A;
B_fast = sys_fast.B;
C_fast = sys_fast.C;

vel_sat = 10; % safety limit for the velocity to prevent large values when the observer is inaccurate

%% Load robust controller
load('optimalK.mat');
K_robust = c2d(K, Ts_Outer);
A_robust = K_robust.A;
B_robust = K_robust.B;
C_robust = K_robust.C;
D_robust = K_robust.D;

%% LQR design
% Discrete-Time Model Conversion
sys_c = ss(Ac, Bc, Cc, Dc);
sys_d = c2d(sys_c, Ts_Outer, 'zoh');
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

%% Load MPC params
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

%% done
disp('Initialization complete. Ready for Simulink simulation.');

