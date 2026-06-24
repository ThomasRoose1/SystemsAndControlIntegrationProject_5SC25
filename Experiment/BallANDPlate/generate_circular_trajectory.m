function [r, a] = generate_circular_trajectory(R, Tcircle, N_circles, Ts)
    % Inputs:
    % R         - Radius of the circle
    % Tcircle   - Time it takes to complete one circle
    % N_circles - Number of circles to perform
    % Ts        - Sample time
    
    % Internal parameters
    Tstandstill = 1.0; % Rest time in seconds
    T_in_out = 2.0;    % Time taken to travel from (0,0) to (0,R). Adjust as needed.
    
    % --- Phase 1: Entry from (0,0) to (0,R) ---
    [y_in, ydot_in, yddot_in] = quintic_traj(0, -R, T_in_out, Ts);
    N_in = length(y_in);
    x_in = zeros(1, N_in);
    xdot_in = zeros(1, N_in);
    xddot_in = zeros(1, N_in);
    
    % --- Phase 2: Rest at (0,R) ---
    % Start the time vector from Ts to avoid duplicating the exact boundary point
    t_rest = Ts:Ts:Tstandstill; 
    N_rest = length(t_rest);
    
    x_rest1 = zeros(1, N_rest);
    xdot_rest1 = zeros(1, N_rest);
    xddot_rest1 = zeros(1, N_rest);
    
    y_rest1 = -R * ones(1, N_rest);
    ydot_rest1 = zeros(1, N_rest);
    yddot_rest1 = zeros(1, N_rest);
    
    % --- Phase 3: Circular Motion ---
    % We apply the quintic polynomial to the ANGLE (theta) to ensure the 
    % trajectory starts and ends from rest perfectly smoothly.
    T_circ_total = N_circles * Tcircle;
    theta_start = -pi/2; % Starting angle corresponding to (0, R)
    theta_end = -pi/2 - N_circles * 2 * pi; % Going clockwise
    
    [theta, thetadot, thetaddot] = quintic_traj(theta_start, theta_end, T_circ_total, Ts);
    
    % Strip the first point to prevent duplicate timestamps when concatenating
    theta = theta(2:end);
    thetadot = thetadot(2:end);
    thetaddot = thetaddot(2:end);
    
    % Convert polar kinematics to Cartesian (x, y)
    x_circ = R * cos(theta);
    xdot_circ = -R * sin(theta) .* thetadot;
    xddot_circ = -R * cos(theta) .* (thetadot.^2) - R * sin(theta) .* thetaddot;
    
    y_circ = R * sin(theta);
    ydot_circ = R * cos(theta) .* thetadot;
    yddot_circ = -R * sin(theta) .* (thetadot.^2) + R * cos(theta) .* thetaddot;
    
    % --- Phase 4: Rest at (0,R) ---
    x_rest2 = zeros(1, N_rest);
    xdot_rest2 = zeros(1, N_rest);
    xddot_rest2 = zeros(1, N_rest);
    
    y_rest2 = -R * ones(1, N_rest);
    ydot_rest2 = zeros(1, N_rest);
    yddot_rest2 = zeros(1, N_rest);
    
    % --- Phase 5: Exit from (0,R) back to (0,0) ---
    [y_out, ydot_out, yddot_out] = quintic_traj(-R, 0, T_in_out, Ts);
    
    % Strip the first point
    y_out = y_out(2:end);
    ydot_out = ydot_out(2:end);
    yddot_out = yddot_out(2:end);
    
    N_out = length(y_out);
    x_out = zeros(1, N_out);
    xdot_out = zeros(1, N_out);
    xddot_out = zeros(1, N_out);
    
    % --- Stitch Everything Together ---
    x = [x_in, x_rest1, x_circ, x_rest2, x_out];
    xdot = [xdot_in, xdot_rest1, xdot_circ, xdot_rest2, xdot_out];
    xddot = [xddot_in, xddot_rest1, xddot_circ, xddot_rest2, xddot_out];
    
    y = [y_in, y_rest1, y_circ, y_rest2, y_out];
    ydot = [ydot_in, ydot_rest1, ydot_circ, ydot_rest2, ydot_out];
    yddot = [yddot_in, yddot_rest1, yddot_circ, yddot_rest2, yddot_out];
    
    % Formulate final output vectors
    r = [x; xdot; y; ydot];
    a = [xddot; yddot];

end

% --- Helper Function ---
function [q, qdot, qddot] = quintic_traj(q0, qf, tf, Ts)
    % Upgraded quintic function that also outputs velocity and acceleration
    t0 = 0;
    
    M = [1 t0 t0^2 t0^3 t0^4 t0^5;
         0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
         0 0 2 6*t0 12*t0^2 20*t0^3;
         1 tf tf^2 tf^3 tf^4 tf^5;
         0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
         0 0 2 6*tf 12*tf^2 20*tf^3];
     
    % Boundary conditions: start and end velocities and accelerations are 0
    b = [q0; 0; 0; qf; 0; 0];
    x = M \ b;
    t = t0:Ts:tf;
    
    % Precompute powers of t for cleaner calculations
    t2 = t.^2; t3 = t.^3; t4 = t.^4; t5 = t.^5;
    
    % Position, velocity, and acceleration
    q     = x(1) + x(2)*t + x(3)*t2 + x(4)*t3 + x(5)*t4 + x(6)*t5;
    qdot  = x(2) + 2*x(3)*t + 3*x(4)*t2 + 4*x(5)*t3 + 5*x(6)*t4;
    qddot = 2*x(3) + 6*x(4)*t + 12*x(5)*t2 + 20*x(6)*t3;
end