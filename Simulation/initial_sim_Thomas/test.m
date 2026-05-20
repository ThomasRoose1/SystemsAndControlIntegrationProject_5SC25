%% Setup Simulation Parameters
L_plate = 0.39; % Square plate side length (m) 
r_act = 0.17;   % Actuator joint radius (m) 
T_rest = 0.322; % Resting height (m) [cite: 1032]
r_ball = 0.01905;
ball_pos = [0.05 0];
ball_vel = [0 0];
g = 9.81;
K_accel = 5/7;

% Corner coordinates of the square plate (relative to center)
corners_local = [ 0.5*L_plate,  0.5*L_plate, 0;
                  0.5*L_plate, -0.5*L_plate, 0;
                 -0.5*L_plate, -0.5*L_plate, 0;
                 -0.5*L_plate,  0.5*L_plate, 0]';

% Initialize Plot
fig = figure(1); clf; hold on; grid on; axis equal;
view(3); xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Real-Time Ball & Plate Mechanical Simulation');
zlim([0, 0.5]); xlim([-0.3, 0.3]); ylim([-0.3, 0.3]);

% Handles for animated objects
hPlate = patch('Vertices', zeros(4,3), 'Faces', 1:4, 'FaceColor', 'cyan', 'FaceAlpha', 0.6);
hLegs = plot3(NaN, NaN, NaN, 'LineWidth', 2, 'Color', [0.4 0.4 0.4]); 
hJoints = plot3(NaN, NaN, NaN, 'ro', 'MarkerFaceColor', 'r');
% hCenter = plot3(0, 0, T_rest, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

% Define the ball
[sx, sy, sz] = sphere(20); 
sx = sx * r_ball; sy = sy * r_ball; sz = sz * r_ball;
hBall = surf(sx, sy, sz, 'FaceColor', 'r', 'EdgeColor', 'none');
camlight; lighting gouraud; % Makes the sphere look 3D

%% Animation Loop
t_sim = 50;     % Simulation time (s)
t_prev = 0;
sim_succes = true;
tic;
while toc < t_sim
    % Update clock and dt
    t_now = toc;
    dt = t_now - t_prev;
    t_prev = t_now;
    t = t_now;

    % 1. Simulate Actuator Input (e.g., Sine wave tilting)
    % pos1, 2, 3 are the relative heights (-0.05m to 0.05m)
    amplitude = 0.001; 
    pos1 = amplitude * sin(2*pi*0.5*t);
    pos2 = amplitude * sin(2*pi*0.5*t + 2*pi/3);
    pos3 = amplitude * sin(2*pi*0.5*t + 4*pi/3);
    
    % 2. Calculate Angles using function
    [alpha, beta, psi] = posToAngle(pos1, pos2, pos3);

    % Update the ball acceleration
    ball_accel = [K_accel*g*sin(beta);
             K_accel*g*sin(alpha)];

    % Apply Eulers method to update the velocity and position
    ball_vel = ball_vel + ball_accel * dt;
    ball_pos = ball_pos + ball_vel * dt;
    
    % 3. Create Rotation Matrix (Standard Tait-Bryan: Roll-Pitch-Yaw) 
    % Note: psi is usually ~0 for this hardware
    Rx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
    Ry = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R_total = Rz * Ry * Rx; 

    % Compute position in global frame
    ball_pos_global = R_total * [ball_pos(1); ball_pos(2); r_ball] + [0; 0; z_center];
    
    % 4. Transform Square Corners
    % Current center moves vertically with the average height of motors
    z_center = T_rest + (pos1 + pos2 + pos3)/3; 
    corners_world = (R_total * corners_local) + [0; 0; z_center];
    
    % 5. Transform Actuator Joint Points
    theta = deg2rad([0, 240, 120]);
    A_base = [r_act*cos(theta); r_act*sin(theta); zeros(1,3)];
    A_top = [r_act*cos(theta); r_act*sin(theta); T_rest + [pos1, pos2, pos3]];
    
    % 6. Update Plot Data
    set(hPlate, 'Vertices', corners_world');
    set(hJoints, 'XData', A_top(1,:), 'YData', A_top(2,:), 'ZData', A_top(3,:));
    % set(hCenter, 'ZData', z_center);
    
    % Update Legs (Drawing lines from base Z=0 to joint heights)
    legX = [A_base(1,1) A_top(1,1) NaN A_base(1,2) A_top(1,2) NaN A_base(1,3) A_top(1,3)];
    legY = [A_base(2,1) A_top(2,1) NaN A_base(2,2) A_top(2,2) NaN A_base(2,3) A_top(2,3)];
    legZ = [0 A_top(3,1) NaN 0 A_top(3,2) NaN 0 A_top(3,3)];
    set(hLegs, 'XData', legX, 'YData', legY, 'ZData', legZ);

    % 7. Update ball
    set(hBall, 'XData', sx + ball_pos_global(1), ...
               'YData', sy + ball_pos_global(2), ...
               'ZData', sz + ball_pos_global(3));
    
    drawnow;

    % STOP Condition: Check if ball center leaves the 0.39m square
    if max(abs(ball_pos)) > (L_plate / 2)
        fprintf('Ball fell off at t = %.2f s\n', t);
        sim_succes = false;
        break;
    end
end
if(sim_succes) 
    fprintf("Simulation finished!\n");
end