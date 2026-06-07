%% load into work space before building

fs = 1000;
fn = fs/2;

A = 2; % amplitude of multisine         CHANGE IF DESIRED
tlength = 200; % number of seconds       CHANGE IF DESIRED
N = tlength*fs;
P = 1; % periods

Ts_Outer = 0.1;                      %  CHANGE IF DESIRED


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
  
%% Plate angle saturation limit
plate_angle_sat = deg2rad(10);

%% Observer
Q_kf = diag([1e-4, 1e-2, 1e-4, 1e-2]);
R_kf = diag([1e-5, 1e-5]);

%% State observer 
Ts_fast = 0.001; % 1000 Hz sample time matching the simulation rate

% Discretize continuous matrices for the fast estimation rate
sys_fast = c2d(ss(Ac, Bc, Cc, Dc), Ts_fast, 'zoh');
A_fast = sys_fast.A;
B_fast = sys_fast.B;
C_fast = sys_fast.C;

% % Define your fast observer poles (safely inside the unit circle)
% observer_poles = [0.21, 0.22, 0.23, 0.22];
% 
% % Compute the Observer Gain Matrix L
% L = place(A_fast', C_fast', observer_poles)';
% 
% A_obs = A_fast - L*C_fast;
% B_obs = [B_fast, L];
% C_obs = eye(4);
% D_obs = zeros(4,4);

vel_sat = 10; % safety limit for the velocity to prevent large values when the observer is inaccurate

%% Load robust controller
load('optimalK.mat');
K_robust = c2d(K, Ts_Outer);
A_robust = K_robust.A;
B_robust = K_robust.B;
C_robust = K_robust.C;
D_robust = K_robust.D;

%% 
%% System Parameters
g = 9.81;          % Gravity (m/s^2)
fps = 10;
Ts = 1/fps;         % Sampling time camera

%% Continuous-Time Model
A = [0 1 0 0;
     0 0 0 0;
     0 0 0 1;
     0 0 0 0];
 
B = [0      0;
     5/7*g  0;
     0      0;
     0      5/7*g];
 
C = [1 0 0 0;
     0 0 1 0];
 
D = zeros(2,2);

%% Discrete-Time Model Conversion
sys_c = ss(A, B, C, D);
sys_d = c2d(sys_c, Ts, 'zoh');
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;

%% LQR Controller Design
% Tune Q_lqr to penalize position error vs velocity.
% Tune R_lqr to limit plate tilt acceleration/effort.
Q_lqr = diag([100, 10, 100, 10]); % [x, x_dot, y, y_dot]
R_lqr = diag([2000, 2000]);             % [alpha, beta]

K_lqr = dlqr(Ad, Bd, Q_lqr, R_lqr);
K_lqr = -K_lqr;



%% MPC controller discretization
% 
% % Discretize continuous matrices for the fast estimation rate
% sys_slow = c2d(ss(Ac, Bc, Cc, Dc), Ts_Outer, 'zoh');
% A_slow = sys_fast.A;
% B_slow = sys_fast.B;
% C_slow = sys_fast.C;
% 
% % dimensions
% nx = size(A_slow,1);
% ny = size(C_slow,1);
% nu = size(B_slow,2);
% 
% %% Load Saved Terminal Design Matrices
% % This loads K, P, HT, and hT instantly into the workspace
% load('mpc_terminal_design.mat'); 

% %% Compile/Define YALMIP MPC Optimizer Object
% % State and Input Weights (Must match what you used to compute P offline!)
% Q = eye(nx); 
% R = eye(nu); 
% 
% N = 25; % Prediction Horizon
% [Phi, Gamma, Omega, Psi] = mpc_obj(A_slow, B_slow, Q, R, P, N); % Uses loaded 'P'
% 
% % Setup Polyhedron Constraints (Output and Input)
% max_pos = 0.2; max_angle = deg2rad(10);
% Hu = [-1 0; 1 0; 0 -1; 0 1];     hu = repmat(max_angle, 4, 1);
% Hy = [-1 0; 1 0; 0 -1; 0 1];     hy = repmat(max_pos, 4, 1);
% 
% Hu_bar = kron(eye(N), Hu);       hu_bar = repmat(hu, N, 1);
% Hy_bar = kron(eye(N), Hy);       hy_bar = repmat(hy, N, 1);
% 
% % Define Optimization Variables
% x0 = sdpvar(nx,1);
% Xk = sdpvar(nx*N, 1);
% Uk = sdpvar(nu*N, 1);
% Yk = sdpvar(ny*N, 1);
% 
% Obj = Xk'*Omega*Xk + Uk'*Psi*Uk;
% 
% Con = [Xk == Phi*x0 + Gamma*Uk];
% Con = [Con, Yk == kron(eye(N),C_slow)*Xk];
% Con = [Con, Hy_bar*Yk <= hy_bar];
% Con = [Con, Hu_bar*Uk <= hu_bar];
% x_N = Xk(end-nx+1:end); % Extract the final predicted state step
% Con = [Con, HT * x_N <= hT];
% 
% % Build the compiled optimizer object
% options = sdpsettings('solver','quadprog','verbose',0);
% u_current = Uk(1:nu);
% MPC_sparse = optimizer(Con, Obj, options, x0, u_current);

disp('Initialization complete. Ready for Simulink simulation.');

