%% Robustcontroller_angleOutput_Hinf_conservative.m
% Moderately reactive H-infinity controller synthesis for the 3-actuator ball-balancing table.
%
% Purpose:
%   Synthesize an outer-loop Hinf controller with:
%
%   Controller inputs:
%       yK = [e_x; ev_x; e_z; ev_z]
%          = [r_x - x; -xdot; r_z - z; -zdot]
%
%   Controller outputs:
%       u = [alpha_cmd; beta_cmd]
%
% Important:
%   alpha_cmd and beta_cmd are TABLE ANGLE COMMANDS [rad],
%   NOT angular accelerations.
%
% The controller is moderately reactive:
%   - small reference amplitude
%   - relaxed velocity penalty
%   - strong angle penalty
%   - low bandwidth
%
% Real implementation:
%   camera -> Kalman filter -> [e_x; ev_x; e_z; ev_z]
%   controller -> [alpha_cmd; beta_cmd]
%   geometry converter -> [y1; y2; y3]
%   with center point (0,0) kept at height y=0 and actuator vertices on a 0.34 m diameter circle
%   actuator position loops

clear; clc; close all;

%% Geometry example: angle-to-actuator-height mapping with y_b = 0
% This part is only a check/example. It is NOT part of the Hinf synthesis.

% Actuator vertices lie on a circle with diameter 34 cm.
actuatorDiameter = 0.34;      % [m]
actuatorRadius   = actuatorDiameter/2;

% Equivalent equilateral-triangle side length for vertices on this circle.
% For an equilateral triangle, circumradius R = a/sqrt(3), so a = sqrt(3)*R.
a = sqrt(3)*actuatorRadius;   % [m]

% Fixed height point: table center, not the ball position.
xb = 0.0;                     % [m]
zb = 0.0;                     % [m]

alpha_example = deg2rad(1.0);
beta_example  = deg2rad(-1.0);

M_geom = [ -xb,                 a/sqrt(3) - zb;
          -a/2 - xb,           -a/(2*sqrt(3)) - zb;
           a/2 - xb,           -a/(2*sqrt(3)) - zb ];

y_example = M_geom * [alpha_example; beta_example];

disp('Geometry used for actuator conversion:')
fprintf('actuator circle diameter = %.3f m\n', actuatorDiameter);
fprintf('actuator circle radius   = %.3f m\n', actuatorRadius);
fprintf('equivalent triangle side = %.6f m\n', a);
fprintf('fixed height point       = (%.3f, %.3f) m\n', xb, zb);
disp('Example actuator heights that keep the table center height y(0,0)=0:')
fprintf('alpha_example = %.4f deg\n', rad2deg(alpha_example));
fprintf('beta_example  = %.4f deg\n', rad2deg(beta_example));
fprintf('y1 = %.6f m\n', y_example(1));
fprintf('y2 = %.6f m\n', y_example(2));
fprintf('y3 = %.6f m\n', y_example(3));

%% Physical parameters

g = 9.81;
Kroll = 5/7;
Kg = Kroll*g;

%% Reduced ball model
% State:
%   X = [x; xdot; z; zdot]
%
% Input:
%   u = [alpha_cmd; beta_cmd]
%
% Dynamics:
%   xddot = Kg*alpha_cmd
%   zddot = Kg*beta_cmd

A = [0 1 0 0;
     0 0 0 0;
     0 0 0 1;
     0 0 0 0];

B = [0  0;
     Kg 0;
     0  0;
     0  Kg];

C = eye(4);
D = zeros(4,2);

sys = ss(A,B,C,D);
sys.StateName  = {'x','xdot','z','zdot'};
sys.InputName  = {'alpha_cmd','beta_cmd'};
sys.OutputName = {'x','xdot','z','zdot'};

disp('Reduced plant model:')
disp('Input  = [alpha_cmd; beta_cmd] [rad]')
disp('Output = [x; xdot; z; zdot]')
sys

%% Basic checks

Co = ctrb(A,B);
Ob = obsv(A,C);

fprintf('Rank controllability matrix = %d / %d\n', rank(Co), size(A,1));
fprintf('Rank observability matrix   = %d / %d\n', rank(Ob), size(A,1));

if rank(Co) == size(A,1)
    disp('Model is controllable')
else
    disp('Model is not controllable')
end

if rank(Ob) == size(A,1)
    disp('Model is observable')
else
    disp('Model is not observable')
end

%% Conservative design assumptions / requirements

% Small reference amplitude.
% Do not ask this controller to track 0.6 m if it is designed for centimeters.
r_max = 30e-3;              % [m] 3 cm typical reference

% Relaxed velocity penalty.
% This prevents the controller from trying to kill initial speed instantly.
v_max = 0.50;               % [m/s] stronger velocity damping than conservative version

% Strict angle command limits.
% These are SOFT limits through Wu, not hard saturation.
alpha_max = deg2rad(2.0);   % [rad] allow more table authority
beta_max  = deg2rad(2.0);   % [rad] allow more table authority

% Equivalent angle disturbance level.
d_alpha_max = deg2rad(0.25); % [rad]
d_beta_max  = deg2rad(0.25); % [rad]

% Slow bandwidth settings.
f_ref   = 0.10;             % [Hz] more reactive reference response
f_sens  = 0.15;             % [Hz] slightly more sensitive to position error
f_input = 0.50;             % [Hz] allow faster angle commands
f_noise = 0.80;             % [Hz]

w_ref   = 2*pi*f_ref;
w_sens  = 2*pi*f_sens;
w_input = 2*pi*f_input;
w_noise = 2*pi*f_noise;

% Robust-control specifications
Ms   = 2.2;                 % moderately tight sensitivity peak
epsS = 2e-2;                % stronger position-error tracking

%% Weighting filters

% 1. Reference shaping filter Wr
Wr_ch = r_max * makeweight(1, [w_ref 1/sqrt(2)], 1e-2);
Wr = blkdiag(Wr_ch, Wr_ch);

% 2. Disturbance shaping filter Wd
Wd_alpha_ch = d_alpha_max * makeweight(1, [2*pi*0.02 1/sqrt(2)], 1e-2);
Wd_beta_ch  = d_beta_max  * makeweight(1, [2*pi*0.02 1/sqrt(2)], 1e-2);
Wd = blkdiag(Wd_alpha_ch, Wd_beta_ch);

% 3. Position sensitivity weight Ws
% Less aggressive than before.
Ws_ch = makeweight(1/epsS, w_sens, 1/Ms);
Ws = blkdiag(Ws_ch, Ws_ch);

% 4. Velocity weight Wv
% Much weaker velocity penalty than before.
% This avoids demanding impossible angles when initial ball speed is nonzero.
Wv_ch = (1/v_max) * makeweight(0.7, [w_sens sqrt(0.7*8)], 8);
Wv = blkdiag(Wv_ch, Wv_ch);

% 5. Complementary sensitivity weight Wt
Wt_ch = makeweight(0.01, [w_noise 1], 10);
Wt = blkdiag(Wt_ch, Wt_ch);

% 6. Angle command weight Wu
% Because controller output is alpha_cmd,beta_cmd, this penalizes ANGLES,
% not accelerations.
Wu_alpha_ch = (1/alpha_max) * makeweight(0.7, [w_input sqrt(0.7*30)], 30);
Wu_beta_ch  = (1/beta_max)  * makeweight(0.7, [w_input sqrt(0.7*30)], 30);
Wu = blkdiag(Wu_alpha_ch, Wu_beta_ch);

%% Weighted generalized plant

G = sys;
G.InputName  = {'uplant_alpha','uplant_beta'};
G.OutputName = {'x','xdot','z','zdot'};

Wr.InputName  = {'wr_x','wr_z'};
Wr.OutputName = {'r_x','r_z'};

Wd.InputName  = {'wd_alpha','wd_beta'};
Wd.OutputName = {'d_alpha','d_beta'};

Ws.InputName  = {'e_x','e_z'};
Ws.OutputName = {'zS_x','zS_z'};

Wv.InputName  = {'ev_x','ev_z'};
Wv.OutputName = {'zV_x','zV_z'};

Wt.InputName  = {'x','z'};
Wt.OutputName = {'zT_x','zT_z'};

Wu.InputName  = {'alpha_cmd','beta_cmd'};
Wu.OutputName = {'zU_alpha','zU_beta'};

%% Summing junctions

J1  = sumblk('e_x = r_x - x');
J2  = sumblk('e_z = r_z - z');

Jv1 = sumblk('ev_x = -xdot');
Jv2 = sumblk('ev_z = -zdot');

% Actual angle reaching the reduced plant is commanded angle plus equivalent disturbance.
J3  = sumblk('uplant_alpha = alpha_cmd + d_alpha');
J4  = sumblk('uplant_beta  = beta_cmd  + d_beta');

%% Generalized plant input/output ordering

% Inputs to Pw:
%   exogenous w = [wr_x; wr_z; wd_alpha; wd_beta]
%   control   u = [alpha_cmd; beta_cmd]
inputvar = {'wr_x','wr_z', ...
            'wd_alpha','wd_beta', ...
            'alpha_cmd','beta_cmd'};

% Outputs from Pw:
%   performance z = [zS_x; zS_z; zV_x; zV_z; zT_x; zT_z; zU_alpha; zU_beta]
%   measurements yK = [e_x; ev_x; e_z; ev_z]
outputvar = {'zS_x','zS_z', ...
             'zV_x','zV_z', ...
             'zT_x','zT_z', ...
             'zU_alpha','zU_beta', ...
             'e_x','ev_x','e_z','ev_z'};

Pw = connect(G, Wr, Wd, Ws, Wv, Wt, Wu, ...
             J1, J2, Jv1, Jv2, J3, J4, ...
             inputvar, outputvar);

%% H-infinity synthesis

nmeas = 4;   % [e_x; ev_x; e_z; ev_z]
ncont = 2;   % [alpha_cmd; beta_cmd]

disp('Weighted generalized plant Pw:')
Pw

[Kopt, CLw, gamma] = hinfsyn(Pw, nmeas, ncont);

disp('Optimal Hinf gamma, conservative angle-output controller:')
disp(gamma)

K = ss(Kopt);
K.InputName  = {'e_x','ev_x','e_z','ev_z'};
K.OutputName = {'alpha_cmd','beta_cmd'};

%% Save controller

save('optimalK_angleOutput_speedFeedback.mat','K','gamma','Pw','CLw','actuatorDiameter','actuatorRadius','a','xb','zb');
save('optimalK_angleOutput_speedFeedback_conservative.mat','K','gamma','Pw','CLw','actuatorDiameter','actuatorRadius','a','xb','zb');
save('optimalK_angleOutput_speedFeedback_moderate.mat','K','gamma','Pw','CLw','actuatorDiameter','actuatorRadius','a','xb','zb');
save('optimalK_angleOutput_speedFeedback_positionSensitive.mat','K','gamma','Pw','CLw','actuatorDiameter','actuatorRadius','a','xb','zb');

disp('Controller K(s), output = [alpha_cmd; beta_cmd]:')
tf(K)

fprintf('\nController summary:\n');
fprintf('Inputs : 4 = [e_x; ev_x; e_z; ev_z]\n');
fprintf('Outputs: 2 = [alpha_cmd; beta_cmd] [rad]\n');
fprintf('States : %d\n', order(K));
fprintf('Saved to:\n');
fprintf('  optimalK_angleOutput_speedFeedback.mat\n');
fprintf('  optimalK_angleOutput_speedFeedback_conservative.mat\n');

fprintf('\nImportant implementation note:\n');
fprintf('The Hinf weights give soft limits only. In real implementation and nonlinear simulation,\n');
fprintf('still saturate alpha_cmd and beta_cmd to physical limits, e.g. +/- 1 deg or +/- 2 deg.\n');
