function [alpha_ref, beta_ref] = PDController(ball_pos, ball_vel, target_pos)
    % ball_pos = [x; y] in meters
    % ball_vel = [vx; vy] in m/s
    % target_pos = [0; 0] (center)

    % 1. Super Weak Gains
    % Kp: Proportional gain (position)
    % Kd: Derivative gain (damping/velocity)
    Kp = 0.01; 
    Kd = 0.5;

    % 2. Calculate Error
    error_pos = target_pos - ball_pos;
    error_vel = [0; 0] - ball_vel; % Target velocity is 0

    % 3. PD Control Law
    % Beta (Pitch) controls X-displacement
    % Alpha (Roll) controls Y-displacement
    % Note: A negative sign is used because a positive tilt creates
    % downhill acceleration in the opposite direction.
    beta_ref  = (Kp * error_pos(1) + Kd * error_vel(1));
    alpha_ref = (Kp * error_pos(2) + Kd * error_vel(2));

    % 4. Safety Limits (Approx 12.6 degrees) [cite: 1579, 1584]
    max_tilt = deg2rad(12); 
    alpha_ref = max(min(alpha_ref, max_tilt), -max_tilt);
    beta_ref  = max(min(beta_ref, max_tilt), -max_tilt);
end