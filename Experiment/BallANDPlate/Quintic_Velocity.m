function path = Quintic_Velocity(reference_end, average_velocity, Ts, measured_position)
% QUINTIC_POLYNOMIAL_TRAJECTORY_VELOCITY
% Analytical version for dSPACE / Control Desk compatibility.

persistent coeffs t_elapsed prev_ref_end tf_internal
if isempty(coeffs)
    coeffs = zeros(6, 1);
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

% Trigger new trajectory calculation
if reference_end ~= prev_ref_end
    % 1. Calculate the required duration (tf)
    distance = abs(measured_position - reference_end);
    
    % Safety check: avoid division by zero if velocity is 0 or distance is 0
    if average_velocity > 0 && distance > 0
        tf_internal = distance / average_velocity;
    else
        tf_internal = 0;
    end
    
    % 2. Reset internal timer
    t_elapsed = 0;
    
    % 3. Define Boundary Conditions (t0 is always 0)
    q0 = measured_position; 
    qf = reference_end;
    v0 = 0; vf = 0;
    ac0 = 0; acf = 0;
    
    % 4. Solve for coefficients
    % If tf is essentially 0, we just jump to the end (coeffs are mostly 0)
    if tf_internal > 0
        T = tf_internal;
        M = [1, 0, 0,  0,    0,    0;
             0, 1, 0,  0,    0,    0;
             0, 0, 2,  0,    0,    0;
             1, T, T^2, T^3,  T^4,  T^5;
             0, 1, 2*T, 3*T^2, 4*T^3, 5*T^4;
             0, 0, 2,   6*T,   12*T^2, 20*T^3];
         
        b = [q0; v0; ac0; qf; vf; acf];
        coeffs = M \ b;
    else
        % Immediate jump: p(t) = reference_end
        coeffs = [reference_end; 0; 0; 0; 0; 0];
    end
end

% 5. Calculate the output for the current time step
if t_elapsed <= tf_internal
    t = t_elapsed;
    % Polynomial: a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    path = coeffs(1) + coeffs(2)*t + coeffs(3)*t^2 + ...
           coeffs(4)*t^3 + coeffs(5)*t^4 + coeffs(6)*t^5;
    
    % Increment time
    t_elapsed = t_elapsed + Ts;
else
    % Trajectory finished, hold the end position
    path = reference_end;
end

% Update persistent memory
prev_ref_end = reference_end;

end