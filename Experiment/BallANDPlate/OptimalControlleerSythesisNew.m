%% Robustcontroller_angleOutput_Hinf_final_nearIntegratorShift.m
% H-infinity outer-loop controller synthesis for ball-and-plate simulation.
%
% Controller inputs:
%   yK = [e_x; ev_x; e_z; ev_z]
%      = [r_x - x; -xdot; r_z - z; -zdot]
%
% Controller outputs:
%   u = [alpha_cmd; beta_cmd] [rad]
%
% Simulation convention:
%   xddot =  Kg * beta_actual
%   zddot = -Kg * alpha_actual
%
% Important:
%   This script saves K as a DISCRETE controller because your Simulink
%   MATLAB Function block uses K.A, K.B, K.C, K.D directly.
%
% Correction:
%   Only near-integrator poles, e.g. -1e-7, are shifted.
%   Normal slow poles such as -0.16, -0.20, -0.55 are left unchanged.

clear; clc; close all;
format long g;

%% Physical parameters

g = 9.81;
Kroll = 5/7;
Kg = Kroll*g;

Ts_slow = 0.01;     % outer-loop sample time [s]

%% Reduced ball model
% State:
%   X = [x; xdot; z; zdot]
%
% Input:
%   u = [alpha_actual; beta_actual]
%
% Dynamics:
%   xddot =  Kg * beta_actual
%   zddot = -Kg * alpha_actual

A_ball = [0 1 0 0;
          0 0 0 0;
          0 0 0 1;
          0 0 0 0];

B_ball = [0    0;
          0    Kg;
          0    0;
         -Kg   0];

C_ball = eye(4);
D_ball = zeros(4,2);

G_ball = ss(A_ball,B_ball,C_ball,D_ball);
G_ball.StateName  = {'x','xdot','z','zdot'};
G_ball.InputName  = {'alpha_actual','beta_actual'};
G_ball.OutputName = {'x','xdot','z','zdot'};

%% Approximate actuator / inner-loop angle dynamics

s = tf('s');

tau_angle = 0.8;       % [s]

Fact = 1/(tau_angle*s + 1);

G_act_angle = blkdiag(Fact, Fact);
G_act_angle.InputName  = {'uplant_alpha','uplant_beta'};
G_act_angle.OutputName = {'alpha_actual','beta_actual'};

%% Total plant used for synthesis

G = connect(G_act_angle, G_ball, ...
            {'uplant_alpha','uplant_beta'}, ...
            {'x','xdot','z','zdot'});

G = minreal(G);

disp('Synthesis plant:')
G

%% Basic checks

[Acheck,Bcheck,Ccheck,~] = ssdata(ss(G));

Co = ctrb(Acheck,Bcheck);
Ob = obsv(Acheck,Ccheck);

fprintf('Rank controllability matrix = %d / %d\n', rank(Co), size(Acheck,1));
fprintf('Rank observability matrix   = %d / %d\n', rank(Ob), size(Acheck,1));

%% Design assumptions / requirements

r_max = 0.25;                 % [m] 25 cm plate scale
v_max = 0.60;                 % [m/s]

alpha_max = deg2rad(3.0);     % [rad], soft design angle
beta_max  = deg2rad(3.0);     % [rad]

d_alpha_max = deg2rad(0.03);  % [rad]
d_beta_max  = deg2rad(0.03);  % [rad]

f_ref   = 0.1;               % [Hz]
f_sens  = 0.25;               % [Hz]
f_input = 0.10;               % [Hz]
f_noise = 0.80;               % [Hz], only used if useWt = true

w_ref   = 2*pi*f_ref;
w_sens  = 2*pi*f_sens;
w_input = 2*pi*f_input;
w_noise = 2*pi*f_noise;

Ms   = 2.0;
epsS = 0.20;                  % low-frequency sensitivity weight = 5

useWt = false;
useStrokeWeight = false;

r_joint = 0.17;               % [m]
x_soft  = 0.030;              % [m]

%% Weighting filters

% 1. Reference shaping filter Wr
Wr_ch = r_max * makeweight(1, [w_ref 1/sqrt(2)], 0.10);
Wr = blkdiag(Wr_ch, Wr_ch);

% 2. Disturbance shaping filter Wd
Wd_alpha_ch = d_alpha_max * makeweight(1, [2*pi*0.05 1/sqrt(2)], 0.10);
Wd_beta_ch  = d_beta_max  * makeweight(1, [2*pi*0.05 1/sqrt(2)], 0.10);
Wd = blkdiag(Wd_alpha_ch, Wd_beta_ch);

% 3. Position sensitivity weight Ws
Ws_ch = makeweight(1/epsS, [w_sens 1], 1/Ms);
Ws = blkdiag(Ws_ch, Ws_ch);

% 4. Velocity weight Wv
Wv_ch = (1/v_max) * makeweight(0.10, [w_sens 1], 1.1);
Wv = blkdiag(Wv_ch, Wv_ch);

% 5. Complementary sensitivity weight Wt
Wt_ch = makeweight(0.02, [w_noise sqrt(0.02*5)], 5);
Wt = blkdiag(Wt_ch, Wt_ch);

% 6. Angle command weight Wu
Wu_hf = 20;
Wu_lf = 0.4;
Wu_mid = sqrt(Wu_lf*Wu_hf);

Wu_alpha_ch = (1/alpha_max) * makeweight(Wu_lf, [w_input Wu_mid], Wu_hf);
Wu_beta_ch  = (1/beta_max)  * makeweight(Wu_lf, [w_input Wu_mid], Wu_hf);
Wu = blkdiag(Wu_alpha_ch, Wu_beta_ch);

% 7. Approximate actuator-stroke weight
% Only included in Pw if useStrokeWeight = true.
Mstroke_mat = [ 0,                         -r_joint;
               -sqrt(3)/2*r_joint,          0.5*r_joint;
                sqrt(3)/2*r_joint,          0.5*r_joint ];

Gstroke = ss([],[],[],Mstroke_mat);
Gstroke.InputName  = {'alpha_cmd','beta_cmd'};
Gstroke.OutputName = {'p1_rel','p2_rel','p3_rel'};

Wp = ss([],[],[], (1/x_soft)*eye(3));
Wp.InputName  = {'p1_rel','p2_rel','p3_rel'};
Wp.OutputName = {'zP1','zP2','zP3'};

%% Name signals

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

J3  = sumblk('uplant_alpha = alpha_cmd + d_alpha');
J4  = sumblk('uplant_beta  = beta_cmd  + d_beta');

%% Generalized plant

inputvar = {'wr_x','wr_z', ...
            'wd_alpha','wd_beta', ...
            'alpha_cmd','beta_cmd'};

if useWt && useStrokeWeight

    outputvar = {'zS_x','zS_z', ...
                 'zV_x','zV_z', ...
                 'zT_x','zT_z', ...
                 'zU_alpha','zU_beta', ...
                 'zP1','zP2','zP3', ...
                 'e_x','ev_x','e_z','ev_z'};

    Pw = connect(G, Wr, Wd, Ws, Wv, Wt, Wu, Gstroke, Wp, ...
                 J1, J2, Jv1, Jv2, J3, J4, ...
                 inputvar, outputvar);

elseif useWt && ~useStrokeWeight

    outputvar = {'zS_x','zS_z', ...
                 'zV_x','zV_z', ...
                 'zT_x','zT_z', ...
                 'zU_alpha','zU_beta', ...
                 'e_x','ev_x','e_z','ev_z'};

    Pw = connect(G, Wr, Wd, Ws, Wv, Wt, Wu, ...
                 J1, J2, Jv1, Jv2, J3, J4, ...
                 inputvar, outputvar);

elseif ~useWt && useStrokeWeight

    outputvar = {'zS_x','zS_z', ...
                 'zV_x','zV_z', ...
                 'zU_alpha','zU_beta', ...
                 'zP1','zP2','zP3', ...
                 'e_x','ev_x','e_z','ev_z'};

    Pw = connect(G, Wr, Wd, Ws, Wv, Wu, Gstroke, Wp, ...
                 J1, J2, Jv1, Jv2, J3, J4, ...
                 inputvar, outputvar);

else

    outputvar = {'zS_x','zS_z', ...
                 'zV_x','zV_z', ...
                 'zU_alpha','zU_beta', ...
                 'e_x','ev_x','e_z','ev_z'};

    Pw = connect(G, Wr, Wd, Ws, Wv, Wu, ...
                 J1, J2, Jv1, Jv2, J3, J4, ...
                 inputvar, outputvar);

end

%% H-infinity synthesis

nmeas = 4;   % [e_x; ev_x; e_z; ev_z]
ncont = 2;   % [alpha_cmd; beta_cmd]

disp('Weighted generalized plant Pw:')
Pw

opts = hinfsynOptions('Display','on');

[Kc_opt, CLw, gamma] = hinfsyn(Pw, nmeas, ncont, opts);

fprintf('\nHinf gamma = %.6f\n', gamma);

%% Full Hinf controller

Kc_full = minreal(ss(Kc_opt),1e-6);
Kc_full.InputName  = {'e_x','ev_x','e_z','ev_z'};
Kc_full.OutputName = {'alpha_cmd','beta_cmd'};

fprintf('\nFull continuous controller order = %d\n', order(Kc_full));
disp('Full continuous controller poles:')
disp(pole(Kc_full))

if any(real(pole(Kc_full)) >= 0)
    warning('Full continuous Kc is not standalone stable.');
else
    disp('Full continuous Kc is standalone stable.');
end

%% Shift only near-integrator controller poles

targetDecay   = 0.50;   % replacement pole: -0.50
nearZeroLimit = 1e-3;   % only poles between -1e-3 and 0 are shifted

Kc = force_near_integrator_decay(Kc_full, targetDecay, nearZeroLimit);
Kc = minreal(ss(Kc),1e-8);

Kc.InputName  = {'e_x','ev_x','e_z','ev_z'};
Kc.OutputName = {'alpha_cmd','beta_cmd'};

fprintf('\nNear-integrator-shifted continuous controller order = %d\n', order(Kc));
disp('Near-integrator-shifted continuous controller poles:')
disp(pole(Kc))

if any(real(pole(Kc)) >= 0)
    warning('Near-integrator-shifted continuous Kc is not standalone stable.');
else
    disp('Near-integrator-shifted continuous Kc is standalone stable.');
end

%% Discretize final controller for Simulink

K = c2d(Kc, Ts_slow, 'tustin');
K.InputName  = {'e_x','ev_x','e_z','ev_z'};
K.OutputName = {'alpha_cmd','beta_cmd'};

A_robust = K.A;
B_robust = K.B;
C_robust = K.C;
D_robust = K.D;

nK_robust = size(A_robust,1);

fprintf('\nFinal discrete controller order = %d\n', nK_robust);

disp('Final discrete controller poles:')
disp(eig(A_robust))

if any(abs(eig(A_robust)) >= 1)
    warning('Final discrete K is not standalone stable.');
else
    disp('Final discrete K is standalone stable.');
end

%% Prepare plant for discrete closed-loop check

Gd = c2d(ss(G), Ts_slow, 'zoh');
[Ag,Bg,Cg,Dg] = ssdata(Gd);

if norm(Dg,inf) > 1e-12
    warning('Gd has nonzero D matrix. Closed-loop check assumes Dg = 0.');
end

Cy = -eye(4);   % yK = -[x; xdot; z; zdot] for zero reference

Ak = A_robust;
Bk = B_robust;
Ck = C_robust;
Dk = D_robust;

Acl_d = [Ag + Bg*Dk*Cy*Cg,   Bg*Ck;
         Bk*Cy*Cg,           Ak];

eigAcl_d = eig(Acl_d);
pole_abs = sort(abs(eigAcl_d),'descend');

disp('Largest final discrete closed-loop pole magnitudes:')
disp(pole_abs(1:min(10,end)))

maxPole = max(pole_abs);

fprintf('Final max pole magnitude = %.15f\n', maxPole);
fprintf('Final practical stability margin = %.15f\n', 1 - maxPole);

if maxPole < 0.995
    disp('Final discrete ideal closed loop has acceptable practical margin.');
elseif maxPole < 1
    warning('Final loop is technically stable, but still too close to marginal.');
else
    warning('Final discrete ideal closed loop is unstable.');
end

%% Saturation / anti-windup parameters for Simulink

alpha_lim = deg2rad(6.0);
beta_lim  = deg2rad(6.0);

u_min_robust = [-alpha_lim; -beta_lim];
u_max_robust = [ alpha_lim;  beta_lim];

Kaw_robust = 0.15;

aw_reg = 1e-6;
Baw_robust = C_robust' / (C_robust*C_robust' + aw_reg*eye(2));

%% Final controller-alone sanity tests

t = 0:Ts_slow:10;

% Test 1: x = +2 cm
% yK = [-0.02; 0; 0; 0]
yK_test = zeros(length(t),4);
yK_test(:,1) = -0.02;

u_test_x = lsim(K, yK_test, t);

figure('Name','Final Hinf controller response: x = +2 cm');
plot(t, rad2deg(u_test_x(:,1)), t, rad2deg(u_test_x(:,2)));
grid on;
xlabel('Time [s]');
ylabel('Command [deg]');
legend('\alpha cmd','\beta cmd');
title('Final controller output for x = +2 cm');

fprintf('\nFinal sanity test x = +2 cm:\n');
fprintf('Expected: beta should mainly be negative.\n');
fprintf('Final alpha = %.4f deg\n', rad2deg(u_test_x(end,1)));
fprintf('Final beta  = %.4f deg\n', rad2deg(u_test_x(end,2)));
fprintf('Max |u|     = %.4f deg\n', rad2deg(max(abs(u_test_x(:)))));

% Test 2: z = +2 cm
% yK = [0; 0; -0.02; 0]
yK_test = zeros(length(t),4);
yK_test(:,3) = -0.02;

u_test_z = lsim(K, yK_test, t);

figure('Name','Final Hinf controller response: z = +2 cm');
plot(t, rad2deg(u_test_z(:,1)), t, rad2deg(u_test_z(:,2)));
grid on;
xlabel('Time [s]');
ylabel('Command [deg]');
legend('\alpha cmd','\beta cmd');
title('Final controller output for z = +2 cm');

fprintf('\nFinal sanity test z = +2 cm:\n');
fprintf('Expected: alpha should mainly be positive.\n');
fprintf('Final alpha = %.4f deg\n', rad2deg(u_test_z(end,1)));
fprintf('Final beta  = %.4f deg\n', rad2deg(u_test_z(end,2)));
fprintf('Max |u|     = %.4f deg\n', rad2deg(max(abs(u_test_z(:)))));

%% Acceptance warnings

maxUx = rad2deg(max(abs(u_test_x(:))));
maxUz = rad2deg(max(abs(u_test_z(:))));

if maxUx < 0.05 || maxUz < 0.05
    warning('Controller may be too weak: command for 2 cm error is below 0.05 deg.');
end

if maxUx > 5.0 || maxUz > 5.0
    warning('Controller may be too aggressive: command for 2 cm error exceeds 5 deg.');
end

if maxPole > 0.997
    warning('Closed-loop pole margin is poor. Simulink may still drift or saturate.');
elseif maxPole > 0.995
    warning('Closed-loop pole margin is marginal but may be testable.');
else
    disp('Closed-loop pole margin is acceptable for first nonlinear simulation test.');
end

%% Save everything needed by Simulink

save('optimalK.mat', ...
     'K', ...                 % final discrete controller used by Simulink
     'Kc', ...                % near-integrator-shifted continuous controller
     'Kc_full', ...           % original Hinf controller
     'gamma', 'Pw', 'CLw', ...
     'Ts_slow', ...
     'A_robust','B_robust','C_robust','D_robust','nK_robust', ...
     'u_min_robust','u_max_robust', ...
     'Baw_robust','Kaw_robust', ...
     'alpha_lim','beta_lim', ...
     'tau_angle','targetDecay','nearZeroLimit', ...
     'r_max','v_max','alpha_max','beta_max', ...
     'd_alpha_max','d_beta_max', ...
     'f_ref','f_sens','f_input','f_noise','Ms','epsS', ...
     'useWt','useStrokeWeight','x_soft','r_joint', ...
     'Mstroke_mat');

disp('Saved optimalK.mat');
disp('K is discrete and ready for your current Simulink implementation.');

%% Local function

function Kout = force_near_integrator_decay(Kin, targetDecay, nearZeroLimit)
% force_near_integrator_decay
%
% Only replaces almost-integrator poles, e.g. -1e-7, with -targetDecay.
% It does NOT shift normal slow controller poles such as -0.16, -0.20, -0.55.
%
% Inputs:
%   targetDecay   desired replacement pole, e.g. 0.50
%   nearZeroLimit only poles with -nearZeroLimit < real(p) < 0 are shifted
%
% Example:
%   targetDecay   = 0.50;
%   nearZeroLimit = 1e-3;
%
% Then:
%   -3e-7  -> -0.50
%   -0.16  -> unchanged
%   -0.20  -> unchanged
%   -0.55  -> unchanged

    Kin = ss(Kin);
    [A,B,C,D] = ssdata(Kin);

    [V,J] = eig(A);

    if rcond(V) < 1e-10
        warning('force_near_integrator_decay: eigenvector matrix is ill-conditioned.');
    end

    lambda = diag(J);
    lambda_new = lambda;

    tolImag = 1e-10;

    for i = 1:length(lambda)

        lam = lambda(i);

        isStable = real(lam) < 0;
        isNearIntegrator = real(lam) > -nearZeroLimit;

        if isStable && isNearIntegrator

            if abs(imag(lam)) < tolImag
                lambda_new(i) = -targetDecay;
            else
                lambda_new(i) = -targetDecay + 1i*imag(lam);
            end

        end
    end

    Jnew = diag(lambda_new);

    Anew = real(V * Jnew / V);

    Kout = ss(Anew,B,C,D);
end