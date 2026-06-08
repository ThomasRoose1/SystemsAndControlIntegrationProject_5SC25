% function u = MPC_solver(x_hat, params)
% % Standalone, real-time Model Predictive Control solver function.
% % Completely decoupled from YALMIP and optimized for dSPACE C-compilation.
% 
% %% Enforce Code Generation and Memory Rules
% % Freeze array boundaries to enforce static allocation on the board cache.
% coder.inline('never');
% coder.varsize('u', [2, 1], [0, 0]);
% 
% %% Extract and Deduce Dimensions Statically
% n_inputs = 2;                  % [alpha; beta]
% n_vars = size(params.G, 1);     % Optimization variables
% n_constraints = size(params.A_condensed, 1); % Total inequality rows
% 
% %% Construct the Objective Linear Gradient Vector (f_vec)
% % Dynamically calculate: f_vec = F * x_hat
% f_vec = params.F * x_hat;
% 
% %% Construct the Constraint Bounding Vector (b_dynamic)
% % Shift the polyhedral limits based on the live state: b_static - W_state * x_hat 
% b_dynamic = params.b_static - params.W_state * x_hat;
% 
% %% Allocate Static Local Working Memory
% x_sol = zeros(n_vars, 1);
% 
% % Fixed settings for strict real-time determinism
% max_iter = 30;                 % Loop cap ensures no task overruns on the CPU clock
% mu = 0.01;                     % Bounded projection gradient step size
% 
% %% Real-Time Primal Gradient Projection Loop
% for iter = 1:max_iter
%     % Compute objective gradient (grad = G * x + f) 
%     grad = params.G * x_sol + f_vec;
% 
%     % Primal update step (Move down the gradient slope)
%     x_sol = x_sol - mu * grad; 
% 
%     % Evaluate polyhedral constraint boundaries (A_condensed * x - b)
%     violations = params.A_condensed * x_sol - b_dynamic;
% 
%     % Active-Set projection sweep
%     for i = 1:n_constraints
%         if violations(i) > 0
%             % Push parameter choices back onto the safe constraint surface
%             x_sol = x_sol - (mu * violations(i)) * params.A_condensed(i, :)';
%         end
%     end
% end
% 
% %% Receding Horizon Selection
% % Pass only the immediate actuator targets to the linear servos
% u = x_sol(1:n_inputs);
% 
% end

%% alternative version using quadprog

function u = MPC_solver(x_hat, params, u_prev)
% Embedded MPC solver utilizing MATLAB's quadprog Active-Set algorithm.
% This formulation is fully compliant with Simulink Coder for real-time dSPACE execution.
%
% Inputs:
%   x_hat  : Current estimated state vector from observer (4 x 1)
%   params : Constant parameter structure containing condensed matrices
%   u_prev : Previous input command

%% Enforce Code Generation and Structural Constraints
% Freeze array boundaries to guarantee static memory allocation.
coder.inline('never');
coder.varsize('u', [2, 1], [0, 0]);

%% Deduce Optimization Horizon Dimensions Statically
n_inputs = 2;                  % [alpha; beta] (pitch and roll commands)
n_vars = size(params.G, 1);     % Total optimization variables (nu * N = 50)
N_horizon = n_vars / n_inputs;

%% Construct Objective and Constraint Matrices Natively
% Calculate the state-dependent linear objective vector: f_vec = F * x_hat
f_vec = params.F * x_hat;

% Compute the dynamic right-hand side polyhedral boundaries: b_static - W_state * x_hat
b_dynamic = params.b_static - params.W_state * x_hat;

%% Define Static Non-Tunable Optimization Options
% Real-time C code compilation requires pre-configuring an explicit active-set target.
% We must also supply a strict MaxIterations cap to secure our 10ms execution budget.
options = optimoptions('quadprog', ...
    'Algorithm', 'active-set', ...
    'MaxIterations', 50, ...
    'Display', 'off');

%% allocate Optimization Output and Initial Guess Workspace
% Pre-allocating zero-vectors forces the compiler to use local CPU stack cache.
x_sol = zeros(n_vars, 1);

% Initialize a static initial guess vector for the active-set algorithm.
% Based on the previous inputs.
X0 = repmat(u_prev, N_horizon, 1);

% Define empty arrays for unutilized QP fields (Equality & lower/upper bounds)
Aeq = [];
beq = [];
lb  = [];
ub  = [];

%% Run Bounded Embedded Quadprog Solver
% Passing X0 explicitly as the 9th argument satisfies the embedded compiler.
[x_sol, ~, exitflag] = quadprog(params.G, f_vec, params.A_condensed, b_dynamic, ...
                                Aeq, beq, lb, ub, X0, options);

%% Diagnostic Fallback and Safety Check
% In an embedded deployment, if an extreme disturbance drives the system to infeasibility (exitflag < 0),
% we must handle the exception gracefully to prevent a task overrun or physical damage.
if exitflag < 0
    u = u_prev;
else
    % Extract exclusively the first optimal input pair [alpha_ref; beta_ref] for the actuators
    u = x_sol(1:n_inputs);
end

end