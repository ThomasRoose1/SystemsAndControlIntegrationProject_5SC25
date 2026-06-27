%% Ball and Plate MPC Automation & Plotting Script
%% Initialize Parameters
disp('Loading parameters...');
setParams; % Calls setup script

%% Run the Simulation
sim_time = 20;
disp(['Running Simulink model for ', num2str(sim_time), ' seconds...']);
out = sim('sim_ballAndPlate_complete', 'StopTime', num2str(sim_time)); 
disp('Simulation complete. Generating plots...');

%% 3. Extract Signals
% Extract the root structure from the simulation output
data = out.simout; 

% Extract time vector (all signals share the same time basis)
t = data.x.Time;

% Extract signals by their proper names and convert units
x_pos   = data.x.Data;            % [m]
y_pos   = data.y.Data;            % [m]
alpha   = data.alpha_out.Data * (180/pi);   % [deg]
beta    = data.beta_out.Data * (180/pi);    % [deg]

% Squeeze the [2 x 1 x N] 3D array into a [2 x N] matrix
u_matrix = squeeze(data.u.Data); 

% Grab row 1 (alpha) and row 2 (beta), then transpose (') to match the 't' column
u_alpha = u_matrix(2, :)' * (180/pi); % Commanded Alpha [deg]
u_beta  = u_matrix(1, :)' * (180/pi); % Commanded Beta [deg]

% Squeeze the [4 x 1 x N] 3D array into a [4 x N] matrix
r = squeeze(data.r.Data); %[x xdot y ydot]

% Extract the X and Y position references (Rows 1 and 3)
r_x = r(1, :)'; 
r_y = r(3, :)'; 

%% 4. Generate Plots
% --- Plot 1: Ball Position Over Time ---
figure('Name', 'Position Tracking', 'NumberTitle', 'off', 'Position', [100, 100, 800, 500]);
subplot(2,1,1);
plot(t, r_x, 'k--', 'LineWidth', 1.5); hold on;
plot(t, x_pos, 'b', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 1); % Changed to dotted so it doesn't overlap the reference
ylabel('X Position [m]', 'FontWeight', 'bold'); % Updated to [m] based on extraction
title('Ball Tracking Performance', 'FontSize', 12);
legend('Reference', 'Actual', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t, r_y, 'k--', 'LineWidth', 1.5); hold on;
plot(t, y_pos, 'r', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 1);
xlabel('Time [s]', 'FontWeight', 'bold');
ylabel('Y Position [m]', 'FontWeight', 'bold');
legend('Reference', 'Actual', 'Location', 'best');
grid on;

% --- Plot 2: 2D Ball Trajectory (Top-Down View) ---
figure('Name', '2D Trajectory', 'NumberTitle', 'off', 'Position', [950, 100, 600, 500]);
plot(r_x, r_y, 'k--', 'LineWidth', 1.5); hold on; % Plot the reference path
plot(x_pos, y_pos, 'b', 'LineWidth', 1.5);        % Plot the actual path

% Mark start points
plot(x_pos(1), y_pos(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
plot(0, 0, 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Target Center

xlabel('X Position [m]', 'FontWeight', 'bold');
ylabel('Y Position [m]', 'FontWeight', 'bold');
title('2D Plate Trajectory', 'FontSize', 12);
legend('Reference Path', 'Actual Trajectory', 'Start Point', 'Target Center', 'Location', 'best');
axis equal; % Keeps the square ratio realistic
xlim([-0.2 0.2]); ylim([-0.2 0.2]); % Adjusted limits for meters instead of cm
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