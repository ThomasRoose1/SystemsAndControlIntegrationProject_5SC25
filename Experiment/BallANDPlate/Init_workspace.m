%% Init_workspace.m

clear; clc;

fs = 1000;
fn = fs/2;

Amp = 2;
tlength = 200;
N = tlength*fs;
P = 1;

Ts_Outer = 0.1;

uA = multisine(0,fn,fs,N,Amp);
uA = repmat(uA,[1,P]);

uB = multisine(0,fn,fs,N,Amp);
uB = repmat(uB,[1,P]);

uC = multisine(0,fn,fs,N,Amp);
uC = repmat(uC,[1,P]);

path = quintic(-0.0289,3,0,1/fs);

%% Combined velocity observer

g = 9.81;
K_roll = 5/7;
Kg = K_roll*g;

Ts_obs = Ts_Outer;

f_plate = 1.5;
zeta_plate = 0.8;
wn_plate = 2*pi*f_plate;

A_vel = [0  1   0             0                       0  0   0             0;
         0  0   0             0                       0  0   Kg            0;
         0  0   0             1                       0  0   0             0;
         0  0  -wn_plate^2   -2*zeta_plate*wn_plate   0  0   0             0;
         0  0   0             0                       0  1   0             0;
         0  0  -Kg            0                       0  0   0             0;
         0  0   0             0                       0  0   0             1;
         0  0   0             0                       0  0  -wn_plate^2   -2*zeta_plate*wn_plate];

B_vel = [0             0;
         0             0;
         0             0;
         wn_plate^2    0;
         0             0;
         0             0;
         0             0;
         0             wn_plate^2];

C_vel = [1  0  0  0  0  0  0  0;
         0  0  0  0  1  0  0  0;
         0  0  1  0  0  0  0  0;
         0  0  0  0  0  0  1  0];

D_vel = zeros(4,2);

sys_vel = c2d(ss(A_vel, B_vel, C_vel, D_vel), Ts_obs, 'zoh');

A_vel_d = sys_vel.A;
B_vel_d = sys_vel.B;
C_vel_d = sys_vel.C;

velocity_observer_poles = [0.55 0.60 0.65 0.70 0.75 0.80 0.85 0.90];

L_vel = place(A_vel_d', C_vel_d', velocity_observer_poles)';

AvelocityObserver = A_vel_d - L_vel*C_vel_d;
BvelocityObserver = [B_vel_d L_vel];

CvelocityObserver = [0  1  0  0  0  0  0  0;
                     0  0  0  0  0  1  0  0;
                     0  0  0  1  0  0  0  0;
                     0  0  0  0  0  0  0  1];

DvelocityObserver = zeros(4,6);

xvelocity_initial = zeros(8,1);

%% Robust controller

if exist('optimalK.mat','file') ~= 2
    error('optimalK.mat not found.');
end

load('optimalK.mat','K');

K_robust = ss(K);

Ts_robust = Ts_Outer;

if K_robust.Ts == 0
    Kd_robust = c2d(K_robust, Ts_robust, 'tustin');
else
    Kd_robust = K_robust;
    Ts_robust = Kd_robust.Ts;
end

[Kr_A, Kr_B, Kr_C, Kr_D] = ssdata(Kd_robust);

robust_ref = [0; 0];
robust_angle_max = deg2rad(10);

robust_use_8state_mapping = true;

disp('Init_workspace complete.')
disp('Velocity observer output: [x_dot_hat; y_dot_hat; alpha_dot_hat; beta_dot_hat]')
disp('Robust controller size [outputs inputs]:')
disp(size(Kd_robust))