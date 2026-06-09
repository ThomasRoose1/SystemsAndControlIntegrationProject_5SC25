%% MPC controller for the outer loop of the ball and plate system
% designed to use only functions that are compatible with dSPACE C
% compilation. first part is all setup, seconds part emulated real time
% implementation
clear; clc; close all;
exportswitch = 0;

%% System Parameters & Continuous State-Space Model
g = 9.81;                  % Gravity (m/s^2)
km_factor = (5/7) * g;     % Rolling ball constant (~7.0071)

% Continuous Matrices (4 states, 2 inputs, 2 outputs)
Ac = [0 1 0 0;
      0 0 0 0;
      0 0 0 1;
      0 0 0 0];
  
Bc = [0          0;
      0          km_factor;
      0          0;
     -km_factor  0];
  
Cc = [1 0 0 0;
      0 0 1 0];
  
Dc = [0 0;
      0 0];

%% Discretization slow loop (for MPC solver)
Ts = 0.01; % Sample time (100 Hz matches typical outer loop from literature)
sys_c = ss(Ac, Bc, Cc, Dc);
sys_d = c2d(sys_c, Ts, 'zoh');

% Extract Discrete Matrices
A = sys_d.A;
B = sys_d.B;
C = sys_d.C;
D = sys_d.D;

% dimensions
nx = size(A,1);
ny = size(C,1);
nu = size(B,2);

%% Discretization fast loop (for observer and simulation)
Ts_fast = 0.001; % Sample time (1000 Hz matches simulink model)
sys_d_fast = c2d(sys_c, Ts_fast, 'zoh');

% Extract Discrete Matrices
A_fast = sys_d_fast.A;
B_fast = sys_d_fast.B;
C_fast = sys_d_fast.C;
D_fast = sys_d_fast.D;

%% Tuning
Q = eye(nx); % State weighting
R = eye(nu); % Input weighting
N = 25; % prediction horizon
Ts_slow = 0.01;

% state constraints 
max_pos = 0.15;
max_vel = 1;

%% Load MPC params
MPC_params = compute_mpc_params(Q,R,N, Ts_slow, max_pos, max_vel);

%% State observer 
% Define observer poles, they should be faster than plant's closed-loop poles, so closer to the origin.
% But not too close to the origin as this can cause more noise to come through.
observer_poles = [0.90, 0.91, 0.92, 0.93];

% Compute the Observer Gain Matrix L (Note the transpose matching duality)
L = place(A_fast', C_fast', observer_poles)';

%% Simulation Parameters
T_sim = 5;                 % Total simulation time (seconds)
N_steps = T_sim / Ts_fast;      % Total simulation steps
time = (0:N_steps-1) * Ts_fast; % Time vector

% Allocate arrays for logging results
state_log  = zeros(4, N_steps);         % true state
state_hat_log = zeros(4, N_steps);      % estimated state
input_log  = zeros(2, N_steps);
output_log = zeros(2, N_steps);

% Initial Conditions (e.g., ball starts at x = 10cm, y = -5cm, stationary)
x_state = [0.10; 0; -0.05; 0]; 
x_hat   = [0.10; 0; -0.05; 0];  % estimator starts at 0

% Reference Targets (Where you want the ball to go - the center)
r_target = [0; 0]; 

% Initialize control input for the first iteration
u_k = zeros(nu, 1);
u_prev = zeros(nu, 1);

%% Main Simulation Loop
time_sum = 0;

for k = 1:N_steps
    % --- Measure Outputs ---
    y = C * x_state;
    
    % Log current states and outputs
    state_log(:, k)  = x_state;
    state_hat_log(:, k) = x_hat;
    output_log(:, k) = y;
    
    % --- MPC OPTIMIZATION ALGORITHM ---
    % only comute a new input value at 100Hz, so each 10 samples
    if mod(k-1, 10) == 0
        tic; % start timing

        % Call MPC solver
        u_k = MPC_solver(x_hat, MPC_params, u_prev);
        u_prev = u_k;

        computation_time = toc; % stop timing
        time_sum = time_sum + computation_time;
    end

    input_log(:, k) = u_k;

    % --- Observer State Estimation Update ---
    % x_hat[k+1] = A*x_hat[k] + B*u[k] + L*(y[k] - C*x_hat[k])
    y_hat = C_fast * x_hat;
    x_hat = A_fast * x_hat + B_fast * u_k + L * (y - y_hat);
    
    % --- True Plant Physics Update (State-Space Step) ---
    % x[k+1] = A*x[k] + B*u[k]
    x_state = A_fast * x_state + B_fast * u_k;
end
avg_computation_time = time_sum / (N_steps/10);
fprintf("Average computatin time of optimizer: %.4f s\n", avg_computation_time);

%% Define state constraints for plotting
Hx = [-1 0 0 0;
          1  0 0 0;
          0 -1 0 0;
          0 1  0 0;
          0 0 -1 0;
          0 0 1  0;
          0 0 0 -1;
          0 0 0  1];
hx = [max_pos;
      max_pos;
      max_vel;
      max_vel;
      max_pos;
      max_pos;
      max_vel;
      max_vel];
X_set = Polyhedron(Hx, hx);

% Project position states onto 2D plane
X_projected = X_set.projection([1, 3]);

%% Plotting the Results
figure('Name', "Obeserver performance")
subplot(2,1,1);
plot(time, state_log(2,:), 'g', 'LineWidth', 1.5); hold on;
plot(time, state_hat_log(2,:), 'b--', 'LineWidth', 1.5);
grid on;
legend('True State', 'Estimated State');
xlabel('Time (s)'); ylabel('velocity \dot{x} (m)');
title('Observer Performance for x velocity');

subplot(2,1,2);
plot(time, state_log(3,:), 'g', 'LineWidth', 1.5); hold on;
plot(time, state_hat_log(3,:), 'b--', 'LineWidth', 1.5);
grid on;
legend('True State', 'Estimated State');
xlabel('Time (s)'); ylabel('Position \dot{y} (m/s)');
title('Observer Performance for y velocity');

fig = figure('Name', 'MPC Simulation');
fig.Units = 'inches';
fig.Position = [1, 1, 3.5, 2.2];

subplot(2,1,1);
plot(time, output_log(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(time, output_log(2,:), 'r', 'LineWidth', 1.5);
grid on;
legend('Ball X (m)', 'Ball Y (m)');
xlabel('Time (s)'); ylabel('Position (m)');
% title('Ball Position States');

subplot(2,1,2);
plot(time, rad2deg(input_log(1,:)), 'b--', 'LineWidth', 1.5); hold on;
plot(time, rad2deg(input_log(2,:)), 'r--', 'LineWidth', 1.5);
grid on;
legend('\alpha', '\beta');
xlabel('Time (s)'); ylabel('Input Angles (deg)');
% title('Plate Control Input Commands');

if exportswitch
    exportgraphics(fig, 'Figures/MPC_sim.pdf', 'ContentType','vector');
end

% 2D plot of positions on plate
fig = figure('Name', '2D Position on Plate');
X_projected.plot('color', 'lightblue', 'alpha', 0.5); hold on;
plot(state_log(1,:), state_log(3,:), 'k--', 'MarkerSize', 5, 'LineWidth', 1.5);
grid on;
xlabel('Position X [m]');
ylabel('Position Y [m]');
title('2D Position of the Ball on the Plate');
axis equal;
if exportswitch
    exportgraphics(fig, 'Figures/phasep_plot_sim.pdf', 'ContentType','vector');
end
