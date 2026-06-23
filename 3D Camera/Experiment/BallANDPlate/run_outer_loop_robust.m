function u_cmd = run_outer_loop_robust(x_hat, reset, Kr_A, Kr_B, Kr_C, Kr_D, robust_ref, robust_angle_max, robust_use_mapping)
% Robust outer-loop controller.
%
% Input:
%   x_hat = [x; x_dot; y; y_dot]
%
% Controller input:
%   yK = [e_x; x_dot; e_y; y_dot]
%
% Output:
%   u_cmd = [alpha_cmd; beta_cmd]

persistent xK initialized

if isempty(initialized) || reset
    xK = zeros(size(Kr_A,1),1);
    initialized = true;
end

x_pos = x_hat(1);
x_vel = x_hat(2);
y_pos = x_hat(3);
y_vel = x_hat(4);

e_x = robust_ref(1) - x_pos;
e_y = robust_ref(2) - y_pos;

yK = [e_x;
      x_vel;
      e_y;
      y_vel];

if size(Kr_B,2) ~= 4
    error('Controller expects %d inputs, but yK has 4 inputs.', size(Kr_B,2));
end

theta_R = Kr_C*xK + Kr_D*yK;
xK = Kr_A*xK + Kr_B*yK;

if robust_use_mapping
    alpha_cmd = -theta_R(2);
    beta_cmd  =  theta_R(1);
    u_cmd = [alpha_cmd; beta_cmd];
else
    u_cmd = theta_R;
end

u_cmd = min(max(u_cmd, -robust_angle_max), robust_angle_max);

u_cmd = reshape(u_cmd,2,1);

end