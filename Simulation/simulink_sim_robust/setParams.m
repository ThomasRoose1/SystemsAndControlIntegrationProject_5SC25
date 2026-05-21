%% setParams.m
% Parameters for the Simulink simulation of the ball-and-plate system
% Robust-controller version: MPC initialization removed.

clear; clc; close all;

%% ------------------------------------------------------------------------
%  Actuator dynamics
% -------------------------------------------------------------------------

Km = 11;        % [N/A] actuator force constant
m_a = 0.118;    % [kg] actuator piston mass
m_p = 0.682;    % [kg] plate mass
Cv = 16.5;      % [Ns/m] viscous damping constant of actuator

% Total mass actuated by each actuator:
% piston mass + one third of plate mass
m = m_a + (1/3)*m_p;

% Transfer function from actuator current I to actuator position x
s = tf('s');
G_act = (Km/m) / (s*(s + Cv/m));

% Numerator and denominator for Simulink Transfer Fcn block
[G_act_num, G_act_den] = tfdata(G_act, 'v');

% Actuator constraints
x_max = 0.03;   % [m] maximum movement relative to zero position
I_max = 3;      % [A] maximum actuator current

% Actuator initial position
x_init = 0;

%% ------------------------------------------------------------------------
%  Ball dynamics
% -------------------------------------------------------------------------

g = 9.81;                  % [m/s^2] gravity
km_factor = (5/7)*g;       % rolling-ball constant

% Continuous-time ball model used in the simulation/MPC structure
%
% State:
%   x_ball = [x; x_dot; y; y_dot]
%
% Input:
%   u = [alpha; beta]
%
% where alpha and beta are plate-angle commands in radians.
%
% Dynamics:
%   x_ddot =  km_factor * beta
%   y_ddot = -km_factor * alpha

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

Dc = zeros(2,2);

%% ------------------------------------------------------------------------
%  Fast discretization for observer and simulation
% -------------------------------------------------------------------------

Ts_fast = 0.001;   % [s] 1000 Hz simulation/observer rate

sys_fast = c2d(ss(Ac, Bc, Cc, Dc), Ts_fast, 'zoh');

A_fast = sys_fast.A;
B_fast = sys_fast.B;
C_fast = sys_fast.C;
D_fast = sys_fast.D;

%% ------------------------------------------------------------------------
%  State observer
% -------------------------------------------------------------------------

% Observer poles should be faster than the closed-loop plant,
% but not too close to zero, otherwise measurement noise is amplified.
observer_poles = [0.90, 0.91, 0.92, 0.93];

L = place(A_fast', C_fast', observer_poles)';

%% ------------------------------------------------------------------------
%  Slow discretization for outer-loop controller
% -------------------------------------------------------------------------

Ts_slow = 0.01;    % [s] 100 Hz outer-loop rate

sys_slow = c2d(ss(Ac, Bc, Cc, Dc), Ts_slow, 'zoh');

A_slow = sys_slow.A;
B_slow = sys_slow.B;
C_slow = sys_slow.C;
D_slow = sys_slow.D;

% Dimensions
nx = size(A_slow,1);
ny = size(C_slow,1);
nu = size(B_slow,2);

%% ------------------------------------------------------------------------
%  Robust controller initialization
% -------------------------------------------------------------------------
% This section replaces the MPC optimizer initialization.
% It loads the H-infinity / robust controller from optimalK.mat.

load('optimalK.mat','K');

K_robust = K;

% Outer-loop controller sample time
Ts_robust = Ts_slow;

% Discretize controller if continuous-time
if K_robust.Ts == 0
    Kd_robust = c2d(K_robust, Ts_robust, 'tustin');
else
    Kd_robust = K_robust;
    Ts_robust = Kd_robust.Ts;
end

% Extract discrete controller matrices for the MATLAB Function block
[Kr_A, Kr_B, Kr_C, Kr_D] = ssdata(Kd_robust);

% Ball-position reference
robust_ref = [0; 0];     % [x_ref; y_ref] in meters

% Safety limits
robust_angle_max = deg2rad(10);  % [rad] maximum plate angle
robust_acc_max   = 10;           % [rad/s^2] maximum angular acceleration

% Controller output interpretation:
%
% Use 'angle' if K outputs:
%   [alpha; beta]
%
% Use 'angle_accel' if K outputs:
%   [alpha_ddot; beta_ddot]
%
% Your current robust-controller design used the 8-state model:
%   X = [x; x_dot; alpha; alpha_dot; y; y_dot; beta; beta_dot]
% with inputs:
%   u = [alpha_ddot; beta_ddot]
%
% Therefore use 'angle_accel' unless you redesigned K using the same
% 4-state model as the MPC.
robust_output_mode = 'angle_accel';

% Mapping between robust-controller convention and simulation convention.
%
% Robust-controller model:
%   x_ddot = Kg*alpha_R
%   y_ddot = Kg*beta_R
%
% Simulation/MPC model:
%   x_ddot =  Kg*beta_sim
%   y_ddot = -Kg*alpha_sim
%
% Therefore:
%   alpha_sim = -beta_R
%   beta_sim  =  alpha_R
robust_use_8state_mapping = true;

% Reset persistent states in the robust-controller wrapper
clear run_outer_loop_robust;

%% ------------------------------------------------------------------------
%  Diagnostics
% -------------------------------------------------------------------------

disp('setParams initialization complete. Robust controller ready for Simulink.');

disp('Controller size:')
disp(size(K_robust))

disp('Discrete controller matrix sizes:')
disp(['Kr_A: ', mat2str(size(Kr_A))])
disp(['Kr_B: ', mat2str(size(Kr_B))])
disp(['Kr_C: ', mat2str(size(Kr_C))])
disp(['Kr_D: ', mat2str(size(Kr_D))])

disp('Controller output mode:')
disp(robust_output_mode)

disp('8-state convention mapping enabled:')
disp(robust_use_8state_mapping)