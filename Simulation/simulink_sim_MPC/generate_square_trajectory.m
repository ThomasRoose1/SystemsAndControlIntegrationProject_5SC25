function [r, a] = generate_square_trajectory(R, Tsquare, N_squares, Ts)
    % Inputs:
    % R         - Used to define the corners (starting corner at R, R)
    % Tsquare   - Time to complete ONE full square (all 4 sides)
    % N_squares - Number of squares to perform
    % Ts        - Sample time
    
    % Internal parameters
    Tstandstill = 1.0;          % Rest time between movements in seconds
    Tside = Tsquare / 4;        % Time allocated per side of the square
    T_in_out = Tside;           % Time to travel from (0,0) to (R,R). Adjust if needed.

    % Define the 4 corners of the square. 
    % Assuming a square centered around (0,0) with corners at +/- R.
    % If you want a different size/offset, just change C2, C3, and C4.
    C1 = [ R,  R];
    C2 = [-R,  R];
    C3 = [-R, -R];
    C4 = [ R, -R];
    corners = [C1; C2; C3; C4];

    % --- Phase 1: Start from (0,0) and move to C1 ---
    [x, xdot, xddot, y, ydot, yddot] = move_pt([0,0], C1, T_in_out, Ts);
    
    % Rest at C1
    [xr, xdr, xddr, yr, ydr, yddr] = rest_pt(C1, Tstandstill, Ts);
    [x, xdot, xddot, y, ydot, yddot] = append_traj(x, xdot, xddot, y, ydot, yddot, xr, xdr, xddr, yr, ydr, yddr);

    % --- Phase 2: Perform N_squares ---
    for i = 1:N_squares
        for j = 1:4
            % Determine current corner and the next corner in the sequence
            p_current = corners(j, :);
            next_idx = mod(j, 4) + 1;
            p_next = corners(next_idx, :);
            
            % Move to the next corner
            [xm, xdm, xddm, ym, ydm, yddm] = move_pt(p_current, p_next, Tside, Ts);
            [x, xdot, xddot, y, ydot, yddot] = append_traj(x, xdot, xddot, y, ydot, yddot, xm, xdm, xddm, ym, ydm, yddm);
            
            % Rest at the new corner
            [xr, xdr, xddr, yr, ydr, yddr] = rest_pt(p_next, Tstandstill, Ts);
            [x, xdot, xddot, y, ydot, yddot] = append_traj(x, xdot, xddot, y, ydot, yddot, xr, xdr, xddr, yr, ydr, yddr);
        end
    end

    % --- Phase 3: Return to origin from C1 ---
    [xm, xdm, xddm, ym, ydm, yddm] = move_pt(C1, [0,0], T_in_out, Ts);
    [x, xdot, xddot, y, ydot, yddot] = append_traj(x, xdot, xddot, y, ydot, yddot, xm, xdm, xddm, ym, ydm, yddm);

    % Formulate final output vectors
    r = [x; xdot; y; ydot];
    a = [xddot; yddot];
end

% --- Helper Functions ---

function [x, xdot, xddot, y, ydot, yddot] = move_pt(p_start, p_end, T, Ts)
    % Generates a straight line between two points using a normalized quintic polynomial
    % This guarantees that X and Y reach their targets at the exact same time.
    [s, sdot, sddot] = quintic_traj(0, 1, T, Ts);
    
    dx = p_end(1) - p_start(1);
    dy = p_end(2) - p_start(2);
    
    x = p_start(1) + s .* dx;
    xdot = sdot .* dx;
    xddot = sddot .* dx;
    
    y = p_start(2) + s .* dy;
    ydot = sdot .* dy;
    yddot = sddot .* dy;
end

function [x, xdot, xddot, y, ydot, yddot] = rest_pt(p, T, Ts)
    % Generates a standstill trajectory at a specific coordinate
    t = 0:Ts:T;
    N = length(t);
    
    x = p(1) * ones(1, N);
    xdot = zeros(1, N);
    xddot = zeros(1, N);
    
    y = p(2) * ones(1, N);
    ydot = zeros(1, N);
    yddot = zeros(1, N);
end

function [x, xdot, xddot, y, ydot, yddot] = append_traj(x, xdot, xddot, y, ydot, yddot, x_new, xdot_new, xddot_new, y_new, ydot_new, yddot_new)
    % Appends new trajectory segments onto the main arrays.
    % It strips the very first index of the incoming array (index 2:end) 
    % to prevent duplicate time-steps at the boundaries.
    x = [x, x_new(2:end)];
    xdot = [xdot, xdot_new(2:end)];
    xddot = [xddot, xddot_new(2:end)];
    y = [y, y_new(2:end)];
    ydot = [ydot, ydot_new(2:end)];
    yddot = [yddot, yddot_new(2:end)];
end

function [q, qdot, qddot] = quintic_traj(q0, qf, tf, Ts)
    % Core quintic polynomial generator with velocity and acceleration
    t0 = 0;
    
    M = [1 t0 t0^2 t0^3 t0^4 t0^5;
         0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
         0 0 2 6*t0 12*t0^2 20*t0^3;
         1 tf tf^2 tf^3 tf^4 tf^5;
         0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
         0 0 2 6*tf 12*tf^2 20*tf^3];
     
    b = [q0; 0; 0; qf; 0; 0];
    coefs = M \ b;
    t = t0:Ts:tf;
    
    t2 = t.^2; t3 = t.^3; t4 = t.^4; t5 = t.^5;
    
    q     = coefs(1) + coefs(2)*t + coefs(3)*t2 + coefs(4)*t3 + coefs(5)*t4 + coefs(6)*t5;
    qdot  = coefs(2) + 2*coefs(3)*t + 3*coefs(4)*t2 + 4*coefs(5)*t3 + 5*coefs(6)*t4;
    qddot = 2*coefs(3) + 6*coefs(4)*t + 12*coefs(5)*t2 + 20*coefs(6)*t3;
end