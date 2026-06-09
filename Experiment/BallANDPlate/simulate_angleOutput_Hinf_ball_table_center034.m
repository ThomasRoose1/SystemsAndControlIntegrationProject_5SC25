%% simulate_angleOutput_Hinf_ball_table_center034.m
% Simulation module for Robustcontroller_angleOutput_Hinf.m
%
% This simulation checks the H-infinity controller whose:
%
%   Inputs are:
%       yK = [e_x; ev_x; e_z; ev_z]
%          = [r_x - x; -xdot; r_z - z; -zdot]
%
%   Outputs are:
%       u = [alpha_cmd; beta_cmd]
%
% These outputs are desired table angles. They are then converted into
% actuator height commands y1,y2,y3 using the geometric transformation
% that keeps a selected point B at y_b = 0.
%
% Plant used in this simulation:
%       X = [x; xdot; z; zdot]
%       xddot = Kg * alpha_cmd
%       zddot = Kg * beta_cmd
%
% Requirements:
%   - Robust Control Toolbox
%   - Control System Toolbox
%   - Robustcontroller_angleOutput_Hinf.m in the same folder, or
%     optimalK_angleOutput_speedFeedback.mat already generated.

clear; clc; close all;

%% User settings

% Simulation time
Tend = 40;              % [s]
dt   = 0.005;           % [s]
T    = (0:dt:Tend)';

% Reference trajectory mode:
%   "point"  : fixed target point
%   "circle" : circular target trajectory
referenceMode = "circle";

% Fixed point target
rx_value = 0.02;        % [m]
rz_value = 0.01;        % [m]

% Circular target
circleCenterX = 0.00;   % [m]
circleCenterZ = 0.00;   % [m]
circleRadius  = 0.06;   % [m]
circleFreqHz  = 0.04;   % [Hz]
circlePhase   = 0.00;   % [rad]

% Initial ball state:
% Xp = [x; xdot; z; zdot]
x0    = 0.08;           % [m]
xdot0 = 0.00;           % [m/s]
z0    = -0.05;          % [m]
zdot0 = 0.00;           % [m/s]

Xp0 = [x0; xdot0; z0; zdot0];

% Table geometry for alpha,beta -> actuator heights
% New requirement:
%   - actuator vertices lie on a circle with diameter 34 cm
%   - the fixed point that keeps height y=0 is the table center (0,0)
actuatorDiameter = 0.34;          % [m]
actuatorRadius   = actuatorDiameter/2;

% Equivalent equilateral triangle side length.
% For an equilateral triangle, circumradius R = a/sqrt(3).
triSide = sqrt(3)*actuatorRadius; % [m]

% Fixed height point: table center, not the ball position.
heightPointMode = "center";

xb_fixed = 0.0;         % [m]
zb_fixed = 0.0;         % [m]

% Optional saturation for plots only.
% This does NOT feed back into the linear closed-loop simulation unless
% useNonlinearSaturationSimulation=true.
usePlotSaturation = false;
angle_limit = deg2rad(8);      % [rad]

% Optional nonlinear time-domain simulation with angle saturation.
% If false, simulation uses exact linear closed-loop lsim.
% If true, simulation runs explicit Euler integration and saturates angles.
useNonlinearSaturationSimulation = false;

%% Build reference trajectory r = [rx, rz]

switch referenceMode
    case "point"
        r = [rx_value*ones(size(T)), rz_value*ones(size(T))];

    case "circle"
        omegaRef = 2*pi*circleFreqHz;
        r = [circleCenterX + circleRadius*cos(omegaRef*T + circlePhase), ...
             circleCenterZ + circleRadius*sin(omegaRef*T + circlePhase)];

    otherwise
        error('referenceMode must be "point" or "circle".');
end

%% Load or synthesize the angle-output H-infinity controller

thisFolder = fileparts(mfilename('fullpath'));
if isempty(thisFolder)
    thisFolder = pwd;
end

Kfile = fullfile(thisFolder, 'optimalK_angleOutput_speedFeedback.mat');
SynthFile = fullfile(thisFolder, 'Robustcontroller_angleOutput_Hinf_positionSensitive_center034.m');

if exist(Kfile, 'file')
    loadedData = load(Kfile, 'K', 'gamma');
    K = loadedData.K;
    if isfield(loadedData, 'gamma')
        gamma = loadedData.gamma;
        fprintf('Loaded optimalK_angleOutput_speedFeedback.mat, gamma = %.6g\n', gamma);
    else
        fprintf('Loaded optimalK_angleOutput_speedFeedback.mat\n');
    end
elseif exist(SynthFile, 'file')
    fprintf('Controller MAT file not found. Running Robustcontroller_angleOutput_Hinf.m...\n');
    run(SynthFile);
    loadedData = load(Kfile, 'K', 'gamma');
    K = loadedData.K;
    gamma = loadedData.gamma;
    fprintf('Synthesis completed, gamma = %.6g\n', gamma);
else
    error(['Cannot find optimalK_angleOutput_speedFeedback.mat or Robustcontroller_angleOutput_Hinf_positionSensitive_center034.m. ', ...
           'Put this simulation file in the same folder as the synthesis file.']);
end

K = ss(K);
[Ak,Bk,Ck,Dk] = ssdata(K);
nk = size(Ak,1);

fprintf('\nController dimensions:\n');
fprintf('Inputs  = %d\n', size(Dk,2));
fprintf('Outputs = %d\n', size(Dk,1));
fprintf('States  = %d\n', nk);

%% Rebuild the reduced nominal plant

g = 9.81;
Kroll = 5/7;
Kg = Kroll*g;

% Xp = [x; xdot; z; zdot]
A = [0 1 0 0;
     0 0 0 0;
     0 0 0 1;
     0 0 0 0];

B = [0  0;
     Kg 0;
     0  0;
     0  Kg];

np = size(A,1);

%% Build closed-loop system manually
%
% Controller input:
%   yK = [r_x - x; -xdot; r_z - z; -zdot]
%
% yK = Cm*Xp + Dm*r

Cm = [-1  0  0  0;     % e_x  = r_x - x
       0 -1  0  0;     % ev_x = -xdot
       0  0 -1  0;     % e_z  = r_z - z
       0  0  0 -1];    % ev_z = -zdot

Dm = [1 0;
      0 0;
      0 1;
      0 0];

% Controller:
%   Xk_dot = Ak*Xk + Bk*yK
%   u      = Ck*Xk + Dk*yK
%
% Plant:
%   Xp_dot = A*Xp + B*u

Acl = [A + B*Dk*Cm,     B*Ck;
       Bk*Cm,           Ak];

Bcl = [B*Dk*Dm;
       Bk*Dm];

% Outputs:
%   plant states Xp
%   controller outputs [alpha_cmd; beta_cmd]
%   controller inputs yK
C_xp = [eye(np), zeros(np,nk)];
D_xp = zeros(np,2);

C_u = [Dk*Cm, Ck];
D_u = Dk*Dm;

C_yK = [Cm, zeros(4,nk)];
D_yK = Dm;

Ccl = [C_xp;
       C_u;
       C_yK];

Dcl = [D_xp;
       D_u;
       D_yK];

CLsim = ss(Acl, Bcl, Ccl, Dcl);
CLsim.InputName = {'r_x','r_z'};
CLsim.OutputName = {'x','xdot','z','zdot', ...
                    'alpha_cmd','beta_cmd', ...
                    'e_x','ev_x','e_z','ev_z'};

% Initial augmented state
X0 = [Xp0; zeros(nk,1)];

%% Run simulation

if ~useNonlinearSaturationSimulation

    Y = lsim(CLsim, r, T, X0);

    x         = Y(:,1);
    xdot      = Y(:,2);
    z         = Y(:,3);
    zdot      = Y(:,4);
    alpha_cmd = Y(:,5);
    beta_cmd  = Y(:,6);
    e_x       = Y(:,7);
    ev_x      = Y(:,8);
    e_z       = Y(:,9);
    ev_z      = Y(:,10);

else
    % Nonlinear explicit simulation including angle saturation.
    % Useful to check whether the controller demands unrealistic angles.
    x_aug = X0;
    Y = zeros(length(T),10);

    for k = 1:length(T)
        Xp = x_aug(1:np);
        Xk = x_aug(np+1:end);

        yK = Cm*Xp + Dm*r(k,:)';
        u  = Ck*Xk + Dk*yK;

        u_sat = max(min(u, angle_limit), -angle_limit);

        Y(k,:) = [Xp.' u_sat.' yK.'];

        if k < length(T)
            Xp_dot = A*Xp + B*u_sat;
            Xk_dot = Ak*Xk + Bk*yK;
            x_aug = x_aug + dt*[Xp_dot; Xk_dot];
        end
    end

    x         = Y(:,1);
    xdot      = Y(:,2);
    z         = Y(:,3);
    zdot      = Y(:,4);
    alpha_cmd = Y(:,5);
    beta_cmd  = Y(:,6);
    e_x       = Y(:,7);
    ev_x      = Y(:,8);
    e_z       = Y(:,9);
    ev_z      = Y(:,10);
end

if usePlotSaturation && ~useNonlinearSaturationSimulation
    warning('usePlotSaturation=true only saturates plotted angle signals, not the closed-loop dynamics.');
    alpha_cmd = max(min(alpha_cmd, angle_limit), -angle_limit);
    beta_cmd  = max(min(beta_cmd,  angle_limit), -angle_limit);
end

%% Convert alpha_cmd,beta_cmd to actuator heights y1,y2,y3 with y(0,0)=0

yAct = zeros(length(T),3);
yb_check = zeros(length(T),1);

for k = 1:length(T)

    switch heightPointMode
        case "center"
            % New requirement: keep the table center (0,0) at height zero.
            xb = 0.0;
            zb = 0.0;

        case "fixed"
            % Optional fallback for a user-defined fixed point.
            xb = xb_fixed;
            zb = zb_fixed;

        otherwise
            error('heightPointMode must be "center" or "fixed".');
    end

    M_geom = [ -xb,                         triSide/sqrt(3) - zb;
              -triSide/2 - xb,             -triSide/(2*sqrt(3)) - zb;
               triSide/2 - xb,             -triSide/(2*sqrt(3)) - zb ];

    yAct(k,:) = (M_geom * [alpha_cmd(k); beta_cmd(k)]).';

    % Check y_fixed = alpha*x_b + beta*z_b + c = 0,
    % where c = mean(y1,y2,y3) for this symmetric geometry.
    c = mean(yAct(k,:));
    yb_check(k) = alpha_cmd(k)*xb + beta_cmd(k)*zb + c;
end

y1 = yAct(:,1);
y2 = yAct(:,2);
y3 = yAct(:,3);

%% Print final values

fprintf('\nGeometry used for actuator conversion:\n');
fprintf('actuator circle diameter = %.3f m\n', actuatorDiameter);
fprintf('actuator circle radius   = %.3f m\n', actuatorRadius);
fprintf('equivalent triangle side = %.6f m\n', triSide);
fprintf('height fixed point       = (0,0)\n');

fprintf('\nFinal simulated values:\n');
fprintf('x         = %+9.5f m\n', x(end));
fprintf('xdot      = %+9.5f m/s\n', xdot(end));
fprintf('z         = %+9.5f m\n', z(end));
fprintf('zdot      = %+9.5f m/s\n', zdot(end));
fprintf('alpha_cmd = %+9.5f rad = %+8.4f deg\n', alpha_cmd(end), rad2deg(alpha_cmd(end)));
fprintf('beta_cmd  = %+9.5f rad = %+8.4f deg\n', beta_cmd(end), rad2deg(beta_cmd(end)));
fprintf('max |alpha_cmd| = %.5f rad = %.4f deg\n', max(abs(alpha_cmd)), rad2deg(max(abs(alpha_cmd))));
fprintf('max |beta_cmd|  = %.5f rad = %.4f deg\n', max(abs(beta_cmd)),  rad2deg(max(abs(beta_cmd))));
fprintf('max |yb_check|  = %.3e m\n', max(abs(yb_check)));

%% Plots

figure('Name','Ball position tracking');
plot(T, x, 'LineWidth', 1.3); hold on;
plot(T, r(:,1), '--', 'LineWidth', 1.0);
plot(T, z, 'LineWidth', 1.3);
plot(T, r(:,2), '--', 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('Position [m]');
legend('x','r_x','z','r_z','Location','best');
title('Ball position response');

figure('Name','Ball speed');
plot(T, xdot, 'LineWidth', 1.3); hold on;
plot(T, zdot, 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend('x dot','z dot','Location','best');
title('Ball speed');

figure('Name','Controller measurements');
plot(T, e_x, 'LineWidth', 1.3); hold on;
plot(T, ev_x, 'LineWidth', 1.3);
plot(T, e_z, 'LineWidth', 1.3);
plot(T, ev_z, 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Controller input');
legend('e_x','ev_x','e_z','ev_z','Location','best');
title('Controller measurement vector');

figure('Name','Controller angle outputs');
plot(T, rad2deg(alpha_cmd), 'LineWidth', 1.3); hold on;
plot(T, rad2deg(beta_cmd), 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Angle command [deg]');
legend('alpha cmd','beta cmd','Location','best');
title('H-infinity controller outputs: desired table angles');

figure('Name','Actuator heights keeping y_b = 0');
plot(T, y1, 'LineWidth', 1.3); hold on;
plot(T, y2, 'LineWidth', 1.3);
plot(T, y3, 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Actuator height [m]');
legend('y_1','y_2','y_3','Location','best');
title(['Actuator heights, heightPointMode = ', char(heightPointMode)]);

figure('Name','Top view ball trajectory');
plot(x, z, 'LineWidth', 1.5); hold on;
plot(r(:,1), r(:,2), '--', 'LineWidth', 1.0);
plot(x(1), z(1), 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(r(1,1), r(1,2), 's', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(r(end,1), r(end,2), 'x', 'MarkerSize', 10, 'LineWidth', 2.0);
grid on; axis equal;
xlabel('x [m]');
ylabel('z [m]');
legend('ball trajectory','reference trajectory','ball start','reference start','reference end','Location','best');
title(['Top view: ball trajectory, referenceMode = ', char(referenceMode)]);

figure('Name','Height constraint check');
plot(T, yb_check, 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('y fixed [m]');
title('Check of imposed height constraint y(0,0) = 0');

%% Save simulation data

simData.T = T;
simData.r = r;
simData.x = x;
simData.xdot = xdot;
simData.z = z;
simData.zdot = zdot;
simData.alpha_cmd = alpha_cmd;
simData.beta_cmd = beta_cmd;
simData.e_x = e_x;
simData.ev_x = ev_x;
simData.e_z = e_z;
simData.ev_z = ev_z;
simData.y1 = y1;
simData.y2 = y2;
simData.y3 = y3;
simData.yb_check = yb_check;
simData.heightPointMode = heightPointMode;
simData.referenceMode = referenceMode;
simData.triSide = triSide;
simData.actuatorDiameter = actuatorDiameter;
simData.actuatorRadius = actuatorRadius;
simData.useNonlinearSaturationSimulation = useNonlinearSaturationSimulation;
simData.angle_limit = angle_limit;

save(fullfile(thisFolder, 'simulation_angleOutput_Hinf_results.mat'), 'simData');

fprintf('\nSimulation data saved to simulation_angleOutput_Hinf_results.mat\n');
