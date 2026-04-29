function [alpha_ref, beta_ref, next_states] = ballPositionControl(ball_pos, target_pos, current_states, dt)
    % ball_pos = [x; y], target_pos = [x_ref; y_ref]
    
    % Controller Matrices (Discrete C2New from report eq 6.1)
    persistent Ad Bd Cd Dd
    if isempty(Ad)
        % Define the transfer function provided in the report
        z = tf('z', 0.1); % 0.1s sampling time w.r.t camera [cite: 1747]
        num = 2.5037 * conv([1 -0.06981], [1 -1.92 0.9213]);
        den = conv([1 -1], conv([1 -0.3774], [1 0.3764 0.03567]));
        sys_d = tf(num, den, 0.1);
        [Ad, Bd, Cd, Dd] = ssdata(sys_d);
    end

    % Calculate Error
    error_x = target_pos(1) - ball_pos(1);
    error_y = target_pos(2) - ball_pos(2);

    % Update States for X-axis (Beta)
    dx_x = Ad * current_states.x_ball + Bd * error_x;
    next_states.x_ball = current_states.x_ball + dx_x * dt;
    beta_ref = Cd * current_states.x_ball + Dd * error_x;

    % Update States for Y-axis (Alpha)
    dx_y = Ad * current_states.y_ball + Bd * error_y;
    next_states.y_ball = current_states.y_ball + dx_y * dt;
    alpha_ref = Cd * current_states.y_ball + Dd * error_y;

    % Safety: Limit angles to approx 12.6 degrees (0.22 rad)
    alpha_ref = max(min(alpha_ref, 0.22), -0.22);
    beta_ref = max(min(beta_ref, 0.22), -0.22);
end