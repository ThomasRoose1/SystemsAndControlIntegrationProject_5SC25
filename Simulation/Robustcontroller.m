%% State-space model from the given formulation
% State vector:
% X = [x; x_dot; alpha; alpha_dot; z; z_dot; beta; beta_dot]
%
%   x         = ball position in x-direction [m]
%   x_dot     = ball velocity in x-direction [m/s]
%   alpha     = plate angle affecting x-motion [rad]
%   alpha_dot = angular velocity of alpha [rad/s]
%   z        = ball position in z-direction [m]
%   z_dot     = ball velocity in z-direction [m/s]
%   beta      = plate angle affecting y-motion [rad]
%   beta_dot  = angular velocity of beta [rad/s]
% Input vector:
% u = [u_x; u_z]
%
% Output:
% y_out = [x; z]


clear; clc;


%% Parameters

g = 9.81;

% Rolling-ball gain coefficient.
% For a solid sphere, K = 5/7.
% If your model already defines Kg as one parameter, use Kg directly.
K = 5/7;

Kg = K*g;

%% State-space matrices

A = [0  1   0   0   0  0   0   0;
     0  0   Kg  0   0  0   0   0;
     0  0   0   1   0  0   0   0;
     0  0   0   0   0  0   0   0;
     0  0   0   0   0  1   0   0;
     0  0   0   0   0  0   Kg  0;
     0  0   0   0   0  0   0   1;
     0  0   0   0   0  0   0   0];

B = [0  0;
     0  0;
     0  0;
     1  0;
     0  0;
     0  0;
     0  0;
     0  1];

C = [1  0  0  0  0  0  0  0;
     0  0  0  0  1  0  0  0];

D = zeros(2,2);

%% Create state-space system

sys = ss(A, B, C, D);

sys.StateName = {'x','x_dot','alpha','alpha_dot', 'z','z_dot','beta','beta_dot'};

sys.InputName = {'u_x','u_z'};

sys.OutputName = {'x','z'};

disp(sys)


%% Problem 1

% Part 2 - Controlability, observability and minimality of state-space representation

%Controlability
ControlMatrix = ctrb(A,B)
sizeControlabilityMatrix = size(ControlMatrix)
rankControlabilityMatrix = rank(ControlMatrix)

if min(sizeControlabilityMatrix)==rankControlabilityMatrix
    disp('Model is controllable')
else 
    disp('Model is not controllable')
end
%Obseravability 
ObserabilityMatrix = obsv(A,C)
sizeObserabilityMatrix = size(ObserabilityMatrix)
rankObserabilityMatrix = rank(ObserabilityMatrix)


if min(sizeObserabilityMatrix)==rankObserabilityMatrix
    disp('Model is observable')
else 
    disp('Model is not observable')
end

%Minimality of state-space representation

%If the model is both controllable and observable then it is minimal
%realisation
minsys = minreal(sys)

if order(minsys) == order(sys)
    disp('Model is minimal')
else 
    disp('Model is not minimal')

end

%Part 3 - Stability

%Stability

if isstable(sys)
    disp('The system is stable')
else
    disp('The system is unstable')
end

%Part 1 - Construct P

sys1 = sys;

sys1.InputName = "usys1"
sys1.OutputName = "yy";

%Sum at plant input

J1 = sumblk ("e(1)=r(1)-yy(1)")
J2 = sumblk ("e(2)=r(2)-yy(2)")
J3 = sumblk ("usys1=d+u",2)

w = {'r', 'd', 'u'}
z = {'e', 'yy', 'u', 'yy(1)', 'yy(2)'}

P = connect(sys1, J1, J2, J3, w, z)

%Part 2 - Is P proper?

if isproper(P)
    disp ('P is proper')
else
    disp ('P is not proper')
end

%Part 3 - Is P generalized?

% Extracting state matrices from P
AP = P.A
BPu =  P.B(1:8,5:6)
CPy = P.C(7:8, 1:8)

lam = eig(AP);
n = size(AP,1);

is_detectable = true;

for k = 1:length(lam)
    if real(lam(k)) >= 0
        M = [lam(k)*eye(n) - AP; CPy];
        if rank(M) < n
            is_detectable = false;
            fprintf('Not detectable: unobservable unstable mode at lambda = %g + %gi\n', ...
                real(lam(k)), imag(lam(k)));
        end
    end
end

if is_detectable == true; 
    disp('P.A, P.Cy is detectable')
else 
    disp ('Is not')
end


is_stabilizable = true;

for k = 1:length(lam)
    if abs(lam(k)) >= 1
        M = [lam(k)*eye(n) - AP, BPu];
        if rank(M) < n
            is_stabilizable = false;
            fprintf('Not stabilizable: uncontrollable unstable mode at lambda = %g + %gi\n', ...
                real(lam(k)), imag(lam(k)));
        end
    end
end

if is_stabilizable == true; 
    disp('P.A, P.Bu is stabilizable')
else 
    disp ('Is not')
end

%% Design assumptions / requirements

% Reference amplitude
r_max = 40e-2;          % [m] maximum expected ball-position reference

% Input limit
u_max = 5;             % [rad/s^2] maximum angular acceleration command

% Disturbance level
d_max = 0.5;            % [rad/s^2] low-frequency input disturbance level

% Frequency specifications
f_ref   = 20;          % [Hz] reference bandwidth
f_sens  = 0.5;          % [Hz] desired sensitivity bandwidth
f_input = 3.0;          % [Hz] avoid exciting actuators above this
f_noise = 2.0;          % [Hz] complementary sensitivity roll-off

w_ref   = 2*pi*f_ref;
w_sens  = 2*pi*f_sens;
w_input = 2*pi*f_input;
w_noise = 2*pi*f_noise;

% Robust-control specifications
Ms   = 2;               % max sensitivity peak, approximately 6 dB
epsS = 1e-2;            % desired low-frequency sensitivity level

%% 1. Reference shaping filter Wr
% Unit input wr gives a realistic reference r.
% Low-frequency reference amplitude is about r_max.
% High-frequency reference content is strongly reduced.

Wr_ch = r_max * makeweight(1, [w_ref 1/sqrt(2)], 1e-2);

Wr = blkdiag(Wr_ch, Wr_ch);

%% 2. Disturbance shaping filter Wd
% Unit input wd gives a physical disturbance d.
% Disturbance is mostly low-frequency.

Wd_ch = d_max * makeweight(1, [2*pi*0.05 1/sqrt(2)], 1e-2);

Wd = blkdiag(Wd_ch, Wd_ch);



%% 3. Sensitivity weight Ws
% Penalizes tracking error e = r - y.
% Large low-frequency gain enforces small steady-state error.
% Crossover around f_sens sets the desired closed-loop bandwidth.
%
% Desired:
%   |S| < 1/|Ws|

Ws_ch = makeweight(1/epsS, w_sens, 1/Ms);

Ws = blkdiag(Ws_ch, Ws_ch);



%% 4. Complementary sensitivity weight Wt
% Penalizes high-frequency output response.
% Helps avoid amplification of camera noise and unmodelled actuator dynamics.
%
% Desired:
%   |T| < 1/|Wt|

Wt_ch = makeweight(0.01, [w_noise 1], 20);

Wt = blkdiag(Wt_ch, Wt_ch);



%% 5. Input weight Wu
% Penalizes large angular acceleration commands.
% Low-frequency gain is approximately 1/u_max.
% Gain increases after f_input to discourage high-frequency actuation.
%
% Desired:
%   |Wu*u| < 1

Wu_ch = (1/u_max) * makeweight(0.5, [w_input 1], 50);

Wu = blkdiag(Wu_ch, Wu_ch);


%% Weighted generalized plant for ball-and-plate

% Rename plant signals
G = sys;
G.InputName  = {'uplant_x','uplant_z'};
G.OutputName = {'x','z'};

%% Weighting filter names

Wr.InputName  = {'wr_x','wr_z'};
Wr.OutputName = {'r_x','r_z'};

Wd.InputName  = {'wd_x','wd_z'};
Wd.OutputName = {'d_x','d_z'};

Ws.InputName  = {'e_x','e_z'};
Ws.OutputName = {'zS_x','zS_z'};

Wt.InputName  = {'x','z'};
Wt.OutputName = {'zT_x','zT_z'};

Wu.InputName  = {'u_x','u_z'};
Wu.OutputName = {'zU_x','zU_z'};

%% Summing junctions

% Tracking errors
J1 = sumblk('e_x = r_x - x');
J2 = sumblk('e_z = r_z - z');

% Plant input disturbance
J3 = sumblk('uplant_x = u_x + d_x');
J4 = sumblk('uplant_z = u_z + d_z');

%% Generalized plant input/output ordering

% Inputs to Pw:
%   w = [wr_x; wr_z; wd_x; wd_z]
%   u = [u_x; u_z]
inputvar = {'wr_x','wr_z', ...
            'wd_x','wd_z', ...
            'u_x','u_z'};

% Outputs from Pw:
%   z  = [zS_x; zS_z; zT_x; zT_z; zU_x; zU_z]
%   yK = [e_x; e_z]
outputvar = {'zS_x','zS_z', ...
             'zT_x','zT_z', ...
             'zU_x','zU_z', ...
             'e_x','e_z'};

Pw = connect(G, Wr, Wd, Ws, Wt, Wu, ...
             J1, J2, J3, J4, ...
             inputvar, outputvar);

%% H-infinity synthesis dimensions

nmeas = 2;   % controller measurements: [e_x; e_z]
ncont = 2;   % controller outputs:      [u_x; u_z]

disp('Weighted generalized plant Pw:')
Pw

[Kopt, CLw, gamma] = hinfsyn(Pw, nmeas, ncont);

disp('Optimal Hinf gamma:')
disp(gamma)

K = Kopt;
save("C:\Users\stani\Documents\MATLAB\GitHub\SystemsAndControlIntegrationProject_5SC25\Simulation\simulink_sim_robust","K")

tf(Kopt)