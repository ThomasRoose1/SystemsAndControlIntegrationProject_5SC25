% function u_cmd = run_outer_loop_robust(x_hat)
% % Robust outer-loop controller wrapper for Simulink.
% %
% % Expected observer state:
% %   x_hat = [x; x_dot; alpha; alpha_dot; y; y_dot; beta; beta_dot]
% %
% % Output:
% %   u_cmd = [alpha_cmd; beta_cmd]
% %
% % alpha_cmd and beta_cmd are plate-angle commands in radians.
% 
% persistent xK initialized
% 
% %% Read controller data from base workspace
% 
% A = evalin('base','Kr_A');
% B = evalin('base','Kr_B');
% C = evalin('base','Kr_C');
% D = evalin('base','Kr_D');
% 
% r = evalin('base','robust_ref');
% angle_max = evalin('base','robust_angle_max');
% use_mapping = evalin('base','robust_use_8state_mapping');
% 
% %% Initialize controller state
% 
% if isempty(initialized)
%     xK = zeros(size(A,1),1);
%     initialized = true;
% end
% 
% %% Read states from 8-state observer
% 
% % State convention:
% % x_hat = [x; x_dot; alpha; alpha_dot; y; y_dot; beta; beta_dot]
% 
% x_pos     = x_hat(1);
% x_vel     = x_hat(2);
% alpha     = x_hat(3);
% alpha_dot = x_hat(4);
% 
% y_pos     = x_hat(5);
% y_vel     = x_hat(6);
% beta      = x_hat(7);
% beta_dot  = x_hat(8);
% 
% pos = [x_pos; y_pos];
% 
% % Tracking error
% e = r - pos;
% 
% %% Build controller measurement vector
% 
% nKinputs = size(B,2);
% 
% if nKinputs == 2
% 
%     % Controller uses only position error:
%     % yK = [e_x; e_y]
%     yK = e;
% 
% elseif nKinputs == 4
% 
%     % Controller uses position error and ball velocity:
%     % yK = [e_x; x_dot; e_y; y_dot]
%     yK = [e(1);
%           x_vel;
%           e(2);
%           y_vel];
% 
% elseif nKinputs == 8
% 
%     % Controller uses full 8-state estimate.
%     % Position states are replaced by tracking errors:
%     % yK = [e_x; x_dot; alpha; alpha_dot; e_y; y_dot; beta; beta_dot]
%     yK = [e(1);
%           x_vel;
%           alpha;
%           alpha_dot;
%           e(2);
%           y_vel;
%           beta;
%           beta_dot];
% 
% else
%     error('Unsupported robust controller input dimension. K expects %d inputs.', nKinputs);
% end
% 
% %% Dynamic robust controller update
% 
% theta_robust = C*xK + D*yK;
% xK = A*xK + B*yK;
% 
% %% Optional convention mapping
% 
% if use_mapping
% 
%     % Robust-controller model convention:
%     %   x_ddot = Kg*alpha_R
%     %   y_ddot = Kg*beta_R
%     %
%     % Simulink/MPC convention:
%     %   x_ddot =  Kg*beta_sim
%     %   y_ddot = -Kg*alpha_sim
%     %
%     % Therefore:
%     %   alpha_sim = -beta_R
%     %   beta_sim  =  alpha_R
% 
%     alpha_sim = -theta_robust(2);
%     beta_sim  =  theta_robust(1);
% 
%     u_cmd = [alpha_sim; beta_sim];
% 
% else
% 
%     % Controller already uses the same alpha/beta convention as Simulink
%     u_cmd = theta_robust;
% 
% end
% 
% %% Final safety saturation
% 
% u_cmd = min(max(u_cmd, -angle_max), angle_max);
% 
% % Ensure correct output shape
% u_cmd = reshape(u_cmd,2,1);
% 
% end

function u_cmd = run_outer_loop_robust(y_meas)
% Robust outer-loop controller wrapper.
%
% Input:
%   y_meas = [x_cam; alpha_meas; y_cam; beta_meas]
%
% Controller input:
%   yK = [e_x; alpha_meas; e_y; beta_meas]
%
% Output:
%   u_cmd = [alpha_cmd; beta_cmd]

persistent xK initialized

A = evalin('base','Kr_A');
B = evalin('base','Kr_B');
C = evalin('base','Kr_C');
D = evalin('base','Kr_D');

r = evalin('base','robust_ref');
angle_max = evalin('base','robust_angle_max');
use_mapping = evalin('base','robust_use_8state_mapping');

if isempty(initialized)
    xK = zeros(size(A,1),1);
    initialized = true;
end

x_cam      = y_meas(1);
alpha_meas = y_meas(2);
y_cam      = y_meas(3);
beta_meas  = y_meas(4);

e_x = r(1) - x_cam;
e_y = r(2) - y_cam;

yK = [e_x;
      alpha_meas;
      e_y;
      beta_meas];

if size(B,2) ~= 4
    error('K expects %d inputs, but this wrapper supplies 4 inputs.', size(B,2));
end

theta_R = C*xK + D*yK;
xK = A*xK + B*yK;

if use_mapping
    alpha_cmd = -theta_R(2);
    beta_cmd  =  theta_R(1);
    u_cmd = [alpha_cmd; beta_cmd];
else
    u_cmd = theta_R;
end

u_cmd = min(max(u_cmd, -angle_max), angle_max);
u_cmd = reshape(u_cmd,2,1);

end