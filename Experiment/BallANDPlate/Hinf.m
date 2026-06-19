function [u_cmd, u_raw, u_box, sat_flag, xK_norm] = Hinf( ...
    e_x, ev_x, e_z, ev_z, ...
    reset, ...
    Arob, Brob, Crob, Drob, ...
    Baw, Kaw, ...
    umin, umax)
%#codegen
%
% Hinf controller with:
%   1) hard angle saturation
%   2) command rate limiting
%   3) anti-windup based on the ACTUAL command sent to the plant
%
% Inputs:
%   e_x, ev_x, e_z, ev_z = [r_x-x; -xdot; r_z-z; -zdot]
%   reset                = 1 when outer loop disabled
%
% Outputs:
%   u_cmd = final command to plant / AngleToPos
%   u_raw = unconstrained Hinf output
%   u_box = amplitude-saturated Hinf output before rate limit

persistent xK u_prev initialized

nK = size(Arob,1);

if isempty(initialized)
    xK = zeros(nK,1);
    u_prev = zeros(2,1);
    initialized = true;
end

u_raw = zeros(2,1);
u_box = zeros(2,1);
u_cmd = zeros(2,1);
sat_flag = false;
xK_norm = 0;

% Reset controller state while outer loop is off
if reset ~= 0
    xK = zeros(nK,1);
    u_prev = zeros(2,1);
    return;
end

%% Inputs to Hinf controller

yK = [e_x; ev_x; e_z; ev_z];

u_min = [umin(1); umin(2)];
u_max = [umax(1); umax(2)];

%% Raw Hinf command

u_raw = Crob*xK + Drob*yK;

%% Hard amplitude saturation

u_box = min(max(u_raw, u_min), u_max);

%% Rate limit the command sent to the plant

Ts_outer = 0.01;              % Hinf sample time [s]
maxRate  = deg2rad(20.0);    % [rad/s]

du_max = maxRate * Ts_outer;

du = u_box - u_prev;

du_limited = min(max(du, [-du_max; -du_max]), [du_max; du_max]);

u_cmd = u_prev + du_limited;

% Final safety saturation
u_cmd = min(max(u_cmd, u_min), u_max);

%% Anti-windup
% Use the actual plant command, not only the amplitude-saturated command.

du_aw = u_cmd - u_raw;

x_nom = Arob*xK + Brob*yK;
x_next = x_nom + Kaw*(Baw*du_aw);

% Numerical guard
if any(~isfinite(x_next)) || norm(x_next) > 1e8
    x_next = zeros(nK,1);
    u_cmd = zeros(2,1);
end

xK = x_next;
u_prev = u_cmd;

sat_flag = any(abs(u_cmd - u_raw) > 1e-10);
xK_norm = norm(xK);
end



nK_robust = size(A_robust,1);

Kaw_robust = 0.15;

alpha_lim = deg2rad(6.0);
beta_lim  = deg2rad(6.0);

u_min_robust = [-alpha_lim; -beta_lim];
u_max_robust = [ alpha_lim;  beta_lim];

aw_reg = 1e-8;
Baw_robust = C_robust' / (C_robust*C_robust' + aw_reg*eye(2));