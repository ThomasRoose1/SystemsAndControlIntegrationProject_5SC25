function u_cmd = run_outer_loop_robust(x_hat)
% Robust outer-loop controller wrapper for Simulink.
%
% Input:
%   x_hat = [x; x_dot; y; y_dot]
%
% Output:
%   u_cmd = [alpha; beta]
%
% alpha and beta are plate-angle commands in radians.

persistent xK theta theta_dot initialized

%% Read controller data from base workspace

A = evalin('base','Kr_A');
B = evalin('base','Kr_B');
C = evalin('base','Kr_C');
D = evalin('base','Kr_D');

Ts = evalin('base','Ts_robust');

r = evalin('base','robust_ref');
angle_max = evalin('base','robust_angle_max');
acc_max = evalin('base','robust_acc_max');

output_mode = evalin('base','robust_output_mode');
use_mapping = evalin('base','robust_use_8state_mapping');

%% Initialize persistent controller states

if isempty(initialized)
    xK = zeros(size(A,1),1);
    theta = zeros(2,1);
    theta_dot = zeros(2,1);
    initialized = true;
end

%% Read ball position from observer state

% Simulation state convention:
% x_hat = [x; x_dot; y; y_dot]
pos = [x_hat(1); x_hat(3)];

% Tracking error
e = r - pos;

%% Build controller measurement vector

% If controller expects 2 inputs:
%   yK = [e_x; e_y]
%
% If controller expects 4 inputs:
%   yK = [e_x; e_y; x; y]

nKinputs = size(B,2);

if nKinputs == 2
    yK = e;
elseif nKinputs == 4
    yK = [e; pos];
else
    error('Unsupported robust controller input dimension. K expects %d inputs.', nKinputs);
end

%% Dynamic controller update

u_raw = C*xK + D*yK;
xK = A*xK + B*yK;

%% Interpret controller output

if strcmp(output_mode,'angle')

    % Controller directly outputs [alpha; beta]
    theta_robust = u_raw;

elseif strcmp(output_mode,'angle_accel')

    % Controller outputs angular accelerations:
    %   u_raw = [alpha_ddot; beta_ddot]

    acc_cmd = min(max(u_raw, -acc_max), acc_max);

    theta_dot = theta_dot + Ts*acc_cmd;
    theta     = theta     + Ts*theta_dot;

    % Saturate angles
    theta_sat = min(max(theta, -angle_max), angle_max);

    % Simple anti-windup: stop integrating if saturated
    for i = 1:2
        if theta_sat(i) ~= theta(i)
            theta_dot(i) = 0;
        end
    end

    theta = theta_sat;
    theta_robust = theta;

else
    error('Unknown robust_output_mode. Use ''angle'' or ''angle_accel''.');
end

%% Map robust-controller convention to simulation convention

if use_mapping
    % Robust-controller model convention:
    %   x_ddot = Kg*alpha_R
    %   y_ddot = Kg*beta_R
    %
    % Simulink/MPC model convention:
    %   x_ddot =  Kg*beta_sim
    %   y_ddot = -Kg*alpha_sim
    %
    % Therefore:
    %   alpha_sim = -beta_R
    %   beta_sim  =  alpha_R

    alpha_sim = -theta_robust(2);
    beta_sim  =  theta_robust(1);

    u_cmd = [alpha_sim; beta_sim];

else
    u_cmd = theta_robust;
end

%% Final safety saturation

u_cmd = min(max(u_cmd, -angle_max), angle_max);

% Ensure correct output shape
u_cmd = reshape(u_cmd,2,1);

end