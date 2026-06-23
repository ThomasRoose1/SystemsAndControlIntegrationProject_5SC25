function path = Quintic_Time(reference_end, end_time, Ts, measured_position)
% QUINTIC_POLYNOMIAL_TRAJECTORY_TIME
% Calculated analytically per step to be compatible with dSPACE/Control Desk.

% Persistent variables to store coefficients and the "internal clock"
persistent coeffs t_elapsed prev_ref_end tf_internal
if isempty(coeffs)
    coeffs = zeros(6, 1); % a0, a1, a2, a3, a4, a5
end
if isempty(t_elapsed)
    t_elapsed = 0;
end
if isempty(prev_ref_end)
    prev_ref_end = reference_end;
end
if isempty(tf_internal)
    tf_internal = 0;
end

% Check if a new trajectory is requested
if reference_end ~= prev_ref_end
    % Reset timer
    t_elapsed = 0;
    tf_internal = end_time;
    
    % Boundary conditions
    q0 = measured_position; 
    qf = reference_end;
    v0 = 0; vf = 0;
    ac0 = 0; acf = 0;
    
    % Solver for t0 = 0
    % We solve the system: M * coeffs = b
    % Using tf_internal instead of end_time to ensure consistency during the run
    T = tf_internal;
    M = [1, 0, 0,  0,    0,    0;
         0, 1, 0,  0,    0,    0;
         0, 0, 2,  0,    0,    0;
         1, T, T^2, T^3,  T^4,  T^5;
         0, 1, 2*T, 3*T^2, 4*T^3, 5*T^4;
         0, 0, 2,   6*T,   12*T^2, 20*T^3];
     
    b = [q0; v0; ac0; qf; vf; acf];
    
    % Calculate coefficients (Matrix is small 6x6, so this is fast)
    coeffs = M \ b;
end

% Calculate current reference based on elapsed time
t = t_elapsed;

if t <= tf_internal
    % Quintic Equation: path = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    path = coeffs(1) + coeffs(2)*t + coeffs(3)*t^2 + coeffs(4)*t^3 + coeffs(5)*t^4 + coeffs(6)*t^5;
    
    % Increment time for the next step
    t_elapsed = t_elapsed + Ts;
else
    % Hold the final position once trajectory is finished
    path = reference_end;
end

% Update memory for next iteration
prev_ref_end = reference_end;

end