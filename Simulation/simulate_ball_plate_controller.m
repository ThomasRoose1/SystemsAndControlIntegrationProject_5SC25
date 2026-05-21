clear all

%% simulate_ball_plate_controller.m
% Closed-loop simulation module for the ball-on-actuated-table system.
%
% This script assumes that the synthesis script has produced a controller K:
%
%       e = [e_x; e_z] = [r_x - x; r_z - z]
%       u = [u_x; u_z] = [alpha_ddot; beta_ddot]
%
% The plant state is:
%       Xp = [x; x_dot; alpha; alpha_dot; z; z_dot; beta; beta_dot]
%
% The controller state Xk is appended after Xp:
%       Xcl = [Xp; Xk]
%
% Actuator heights are reconstructed from:
%       [y1; y2; y3] = M * [alpha; beta]
%
% where M depends on the equilateral triangle geometry and the fixed point B.

clear; clc; close all;

%% ------------------------------------------------------------------------
%  1. USER SETTINGS
% -------------------------------------------------------------------------

% Option A: load an already synthesized controller
controllerFile = "optimalK.mat";     % must contain variable K

% Option B: if the file does not exist, run the synthesis script first
synthesisScript = "Robustcontroller.m";

% Geometry
par.a  = 1.0;        % triangle side length [m]
par.xb = 0.1;        % fixed/reference point B coordinate x [m]
par.zb = -0.05;      % fixed/reference point B coordinate z [m]

% Ball model
par.g = 9.81;        % gravity [m/s^2]
par.Kroll = 5/7;     % solid sphere rolling coefficient
par.Kg = par.Kroll * par.g;

% Simulation time
Tend = 30;           % [s]

% Initial plant state:
% [x; x_dot; alpha; alpha_dot; z; z_dot; beta; beta_dot]
Xp0 = [ ...
    0.01;   % initial x position [m]
    0.5;       % initial x velocity [m/s]
    0;       % initial alpha [rad]
    0;       % initial alpha_dot [rad/s]
   0.01;   % initial z position [m]
    0.1;       % initial z velocity [m/s]
    0;       % initial beta [rad]
    0];      % initial beta_dot [rad/s]

% Reference type:
% 'zero'       : balance at origin
% 'step'       : step reference
% 'circle'     : circular trajectory
referenceType = "zero";

% Optional actuator saturation on angular acceleration [rad/s^2]
useSaturation = true;
uMax = 10;

%% ------------------------------------------------------------------------
%  2. LOAD OR GENERATE CONTROLLER
% -------------------------------------------------------------------------

if isfile(controllerFile)
    S = load(controllerFile);
    if ~isfield(S,"K")
        error("The file %s does not contain variable K.", controllerFile);
    end
    K = S.K;
else
    warning("Controller file not found. Running synthesis script: %s", synthesisScript);
    run(synthesisScript);
    if ~exist("K","var")
        error("The synthesis script did not create variable K.");
    end
end

Kss = ss(K);
[Ak,Bk,Ck,Dk] = ssdata(Kss);
nk = order(Kss);

fprintf("Loaded controller with %d states, %d inputs, %d outputs.\n", ...
    nk, size(Bk,2), size(Ck,1));

if size(Bk,2) ~= 2 || size(Ck,1) ~= 2
    error("Expected controller dimensions: 2 inputs [e_x;e_z] and 2 outputs [u_x;u_z].");
end

%% ------------------------------------------------------------------------
%  3. BUILD GEOMETRY TRANSFORMATION
% -------------------------------------------------------------------------

par.M = [ ...
    -par.xb,                  par.a/sqrt(3) - par.zb;
    -par.a/2 - par.xb,       -par.a/(2*sqrt(3)) - par.zb;
     par.a/2 - par.xb,       -par.a/(2*sqrt(3)) - par.zb ];

%% ------------------------------------------------------------------------
%  4. CLOSED-LOOP INITIAL CONDITION
% -------------------------------------------------------------------------

Xk0 = zeros(nk,1);
Xcl0 = [Xp0; Xk0];

ctrl.Ak = Ak;
ctrl.Bk = Bk;
ctrl.Ck = Ck;
ctrl.Dk = Dk;
ctrl.useSaturation = useSaturation;
ctrl.uMax = uMax;
ctrl.referenceType = referenceType;

%% ------------------------------------------------------------------------
%  5. RUN SIMULATION
% -------------------------------------------------------------------------

odefun = @(t,Xcl) closedLoopBallPlateODE(t,Xcl,par,ctrl);

opts = odeset( ...
    "RelTol",1e-7, ...
    "AbsTol",1e-9);

[t,Xcl] = ode45(odefun,[0 Tend],Xcl0,opts);

%% ------------------------------------------------------------------------
%  6. POST-PROCESS SIGNALS
% -------------------------------------------------------------------------

N = numel(t);

x      = Xcl(:,1);
xdot   = Xcl(:,2);
alpha  = Xcl(:,3);
adot   = Xcl(:,4);
z      = Xcl(:,5);
zdot   = Xcl(:,6);
beta   = Xcl(:,7);
bdot   = Xcl(:,8);

r = zeros(N,2);
e = zeros(N,2);
u = zeros(N,2);
yact = zeros(N,3);

for k = 1:N
    Xp = Xcl(k,1:8).';
    Xk = Xcl(k,9:end).';

    rk = referenceSignal(t(k),ctrl.referenceType);
    ek = rk - [Xp(1); Xp(5)];

    uk = Ck*Xk + Dk*ek;
    if useSaturation
        uk = max(min(uk,uMax),-uMax);
    end

    theta = [Xp(3); Xp(7)];
    yk = par.M * theta;

    r(k,:) = rk.';
    e(k,:) = ek.';
    u(k,:) = uk.';
    yact(k,:) = yk.';
end

%% ------------------------------------------------------------------------
%  7. PLOTS
% -------------------------------------------------------------------------

figure("Name","Ball position");
plot(t,x,"LineWidth",1.5); hold on;
plot(t,z,"LineWidth",1.5);
plot(t,r(:,1),"--","LineWidth",1.2);
plot(t,r(:,2),"--","LineWidth",1.2);
grid on;
xlabel("Time [s]");
ylabel("Position [m]");
legend("x","z","r_x","r_z","Location","best");
title("Ball position tracking");

figure("Name","Tracking errors");
plot(t,e(:,1),"LineWidth",1.5); hold on;
plot(t,e(:,2),"LineWidth",1.5);
grid on;
xlabel("Time [s]");
ylabel("Error [m]");
legend("e_x","e_z","Location","best");
title("Tracking errors");

figure("Name","Table angles");
plot(t,alpha*180/pi,"LineWidth",1.5); hold on;
plot(t,beta*180/pi,"LineWidth",1.5);
grid on;
xlabel("Time [s]");
ylabel("Angle [deg]");
legend("\alpha","\beta","Location","best");
title("Table inclination angles");

figure("Name","Controller outputs");
plot(t,u(:,1),"LineWidth",1.5); hold on;
plot(t,u(:,2),"LineWidth",1.5);
grid on;
xlabel("Time [s]");
ylabel("Angular acceleration [rad/s^2]");
legend("u_x = \alpha_{ddot}","u_z = \beta_{ddot}","Location","best");
title("Controller outputs");

figure("Name","Actuator heights");
plot(t,yact(:,1),"LineWidth",1.5); hold on;
plot(t,yact(:,2),"LineWidth",1.5);
plot(t,yact(:,3),"LineWidth",1.5);
grid on;
xlabel("Time [s]");
ylabel("Height [m]");
legend("y_1","y_2","y_3","Location","best");
title("Linear actuator heights");

%% ------------------------------------------------------------------------
%  8. OPTIONAL EXPORT OF SIMULATION DATA
% -------------------------------------------------------------------------

simData.t = t;
simData.Xcl = Xcl;
simData.position = [x z];
simData.reference = r;
simData.error = e;
simData.angles = [alpha beta];
simData.commands = u;
simData.actuatorHeights = yact;

save("ball_plate_simulation_result.mat","simData");

disp("Simulation complete. Results saved to ball_plate_simulation_result.mat");

%% ========================================================================
%  LOCAL FUNCTIONS
% ========================================================================

function dXcl = closedLoopBallPlateODE(t,Xcl,par,ctrl)

    Xp = Xcl(1:8);
    Xk = Xcl(9:end);

    % Measured outputs
    y = [Xp(1); Xp(5)];      % [x; z]

    % Reference and error
    r = referenceSignal(t,ctrl.referenceType);
    e = r - y;

    % Dynamic controller
    u = ctrl.Ck*Xk + ctrl.Dk*e;

    % Optional saturation
    if ctrl.useSaturation
        u = max(min(u,ctrl.uMax),-ctrl.uMax);
    end

    % Plant dynamics
    % Xp = [x; x_dot; alpha; alpha_dot; z; z_dot; beta; beta_dot]
    dXp = zeros(8,1);

    dXp(1) = Xp(2);
    dXp(2) = par.Kg * Xp(3);   % x_ddot = Kg * alpha
    dXp(3) = Xp(4);
    dXp(4) = u(1);             % alpha_ddot command

    dXp(5) = Xp(6);
    dXp(6) = par.Kg * Xp(7);   % z_ddot = Kg * beta
    dXp(7) = Xp(8);
    dXp(8) = u(2);             % beta_ddot command

    % Controller dynamics
    dXk = ctrl.Ak*Xk + ctrl.Bk*e;

    dXcl = [dXp; dXk];
end

function r = referenceSignal(t,referenceType)

    switch string(referenceType)
        case "zero"
            r = [0; 0];

        case "step"
            if t < 1
                r = [0; 0];
            else
                r = [20e-3; -10e-3];
            end

        case "circle"
            R = 20e-3;       % [m]
            f = 0.1;         % [Hz]
            r = R*[cos(2*pi*f*t); sin(2*pi*f*t)];

        otherwise
            error("Unknown reference type: %s", referenceType);
    end
end
