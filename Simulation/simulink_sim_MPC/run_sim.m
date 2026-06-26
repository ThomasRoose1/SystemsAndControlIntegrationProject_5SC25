%% Ball and Plate MPC Automation & Plotting Script

%% Initialize Parameters
disp('Loading parameters...');
setParams; % Calls setup script

%% Run the Simulation
sim_time = 20;
disp(['Running Simulink model for ', num2str(sim_time), ' seconds...']);
out = sim('sim_ballAndPlate', 'StopTime', num2str(sim_time)); 
disp('Simulation complete. Generating plots...');

%% 3. Extract Signals
% Extract the root structure from the simulation output
data = out.simout; 

% Extract time vector (all signals share the same time basis)
t = data.x.Time;

% Extract signals by their proper names and convert units
x_pos   = data.x.Data * 100;            % [cm]
y_pos   = data.y.Data * 100;            % [cm]
alpha   = data.alpha_out.Data * (180/pi);   % [deg]
beta    = data.beta_out.Data * (180/pi);    % [deg]

% Squeeze the [2 x 1 x N] 3D array into a [2 x N] matrix (needed for LQR
% data)
u_matrix = squeeze(data.u.Data); 

% Grab row 1 (alpha) and row 2 (beta), then transpose (') to match the 't' column
u_alpha = u_matrix(2, :)' * (180/pi); % Commanded Alpha [deg]
u_beta  = u_matrix(1, :)' * (180/pi); % Commanded Beta [deg]

%% 4. Generate Plots

% --- Plot 1: Ball Position Over Time ---
figure('Name', 'Position Tracking', 'NumberTitle', 'off', 'Position', [100, 100, 800, 500]);
subplot(2,1,1);
plot(t, x_pos, 'b', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 1);
ylabel('X Position [cm]', 'FontWeight', 'bold');
title('Ball Tracking Performance', 'FontSize', 12);
grid on;

subplot(2,1,2);
plot(t, y_pos, 'r', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 1);
xlabel('Time [s]', 'FontWeight', 'bold');
ylabel('Y Position [cm]', 'FontWeight', 'bold');
grid on;

% --- Plot 2: 2D Ball Trajectory (Top-Down View) ---
figure('Name', '2D Trajectory', 'NumberTitle', 'off', 'Position', [950, 100, 600, 500]);
plot(x_pos, y_pos, 'b', 'LineWidth', 1.5);
hold on;
% Mark start and end points
plot(x_pos(1), y_pos(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
% plot(0, 0, 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Target
xlabel('X Position [cm]', 'FontWeight', 'bold');
ylabel('Y Position [cm]', 'FontWeight', 'bold');
title('2D Plate Trajectory', 'FontSize', 12);
legend('Trajectory', 'Start Point', 'Plate Boundary', 'Target Center', 'Location', 'best');
axis equal; % Keeps the square ratio realistic
xlim([-20 20]); ylim([-20 20]);
grid on;

% --- Plot 3: Actuator Effort (Commanded vs Actual) ---
figure('Name', 'Control Effort', 'NumberTitle', 'off', 'Position', [100, 650, 800, 500]);
subplot(2,1,1);
plot(t, u_alpha, 'k--', 'LineWidth', 1.5); hold on;
plot(t, alpha, 'b', 'LineWidth', 1.5);
ylabel('Alpha [deg]', 'FontWeight', 'bold');
title('Plate Angles: Commanded vs Actual', 'FontSize', 12);
legend('Commanded (u_1)', 'Actual (\alpha)', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t, u_beta, 'k--', 'LineWidth', 1.5); hold on;
plot(t, beta, 'r', 'LineWidth', 1.5);
xlabel('Time [s]', 'FontWeight', 'bold');
ylabel('Beta [deg]', 'FontWeight', 'bold');
legend('Commanded (u_2)', 'Actual (\beta)', 'Location', 'best');
grid on;