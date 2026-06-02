function x_est = intermittent_kalman(y_meas, u, valid_frame, Qkf, Rkf, Ts_outer)
    
    g = 9.81;
    Ad = [1, Ts_outer, 0,0;
        0, 1, 0, 0;
        0, 0, 1, Ts_outer;
        0, 0, 0, 1];

    Bd = [5/14 * g * Ts_outer^2, 0;
        5/7 * g * Ts_outer, 0;
        0, 5/14 * g * Ts_outer^2;
        0, 5/7 * g * Ts_outer];
    
    Cd = [1 0 0 0;
        0 0 1 0];


    % Persistent variables keep their state between sample steps
    persistent x_hat P
    
    % Initialization on the very first time step
    if isempty(x_hat)
        x_hat = zeros(4,1);
        P = eye(4) * 0.1;
    end
    
    % 1. Prediction Step (Always executes)
    x_pred = Ad * x_hat + Bd * u;
    P_pred = Ad * P * Ad' + Qkf;
    
    % 2. Correction Step (Conditional)
    if valid_frame > 0.5
        % Frame is valid: Run normal Kalman Correction
        S = Cd * P_pred * Cd' + Rkf;
        K = P_pred * Cd' / S; % Equivalent to P_pred * Cd' * inv(S)
        
        x_hat = x_pred + K * (y_meas - Cd * x_pred);
        P = (eye(4) - K * Cd) * P_pred;
    else
        % Frame was dropped: Skip correction, trust physics prediction
        x_hat = x_pred;
        P = P_pred;
    end
    
    % Output the current state estimate
    x_est = x_hat;
end