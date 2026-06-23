function u = MPC_solver_ADMM(x_hat, params, u_prev)
% Embedded MPC Solver using Alternating Direction Method of Multipliers (ADMM).

coder.inline('never');
coder.varsize('u', [2, 1], [0, 0]);

n_inputs = 2;                  
n_vars = size(params.G, 1);     
n_constraints = size(params.A_condensed, 1); 
N_horizon = n_vars / n_inputs;

%% Build Gradient and Constraint Borders Dynamically
f_vec = params.F * x_hat;
b_dynamic = params.b_static - params.W_state * x_hat;

%% Allocate ADMM Variables (Local Stack)
% Warm-start the primal variable with the previous control sequence
x_sol = repmat(u_prev, N_horizon, 1); 
z = zeros(n_constraints, 1);       % Slack variables for constraints
y_dual = zeros(n_constraints, 1);  % Dual multipliers (error integrators)

%% 3. Bounded ADMM Resolution Loop
max_iter = 40; % Fixed real-time execution budget

for iter = 1:max_iter
    % A. Unconstrained Primal Update (Using the pre-computed inverse!)
    % Solves for x while ignoring hard boundaries
    q_vec = params.rho * params.A_trans * (z - y_dual) - f_vec;
    x_sol = params.M_inv * q_vec;
    
    % B. Slack Variable Projection
    % Maps the unconstrained x into the constraint space Ax <= b
    Ax = params.A_condensed * x_sol;
    z = min(b_dynamic, Ax + y_dual); % Element-wise clipping bounds the solution
    
    % C. Dual Multiplier Update
    % Accumulates the error between the unconstrained x and bounded z
    y_dual = y_dual + Ax - z;
end

%% 4. Safety Guard and Tracking Fallback
% Extract current positions from the state estimator
x_pos = x_hat(1);
y_pos = x_hat(3);

% Check if the ball is outside the state constraints
is_outside_square = (abs(x_pos) > params.max_pos) || (abs(y_pos) > params.max_pos);

if is_outside_square
    % --- PRIORITY 1: Physical Boundary Breach ---
    % Hard recovery tilt if ball escapes the safe tracking box
    recovery_angle = 1.5;
    u = zeros(2, 1);
    
    if abs(x_pos) > params.max_pos
        % Pos X error requires Neg Beta (u(2)) to accelerate in -x
        u(2) = -recovery_angle * sign(x_pos) * (pi/180); 
    end
    
    if abs(y_pos) > params.max_pos
        % Pos Y error requires Pos Alpha (u(1)) to accelerate in -y
        u(1) = recovery_angle * sign(y_pos) * (pi/180); 
    end

elseif any(isnan(x_sol)) || any(isinf(x_sol)) || any(isnan(x_hat))
    % --- PRIORITY 2: Mathematical / Sensor Failure ---
    % If the camera glitches while the ball is INSIDE the box, hold the last 
    % known safe actuator command to prevent a dSPACE crash.
    u = u_prev;

else
    % --- PRIORITY 3: Normal Receding Horizon Execution ---
    u = x_sol(1:n_inputs);
end
end