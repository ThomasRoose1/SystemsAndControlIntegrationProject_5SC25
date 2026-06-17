function r = Square_Reference(L, T_segment, T_home, Ts, enable)
% SQUARE_REFERENCE Generates [x; xdot; y; ydot] for a smooth square trajectory.
% Loops continuously when enabled. Returns smoothly to (0,0) when disabled.
%
% Inputs:
%   L         - Side length of the square
%   T_segment - Time allowed to traverse one side of the square (seconds)
%   T_home    - Time allowed to transition from/to the origin (seconds)
%   Ts        - Sample time (seconds)
%   enable    - Logical flag (1 = active square, 0 = return home/idle)

% Persistent variables for tracking state machine and trajectories
persistent state t_elapsed coeffs_x coeffs_y q0_x q0_y qf_x qf_y T_current prev_enable current_pos

% Initialize persistent variables on the very first execution step
if isempty(state)
    state = 0;        % States: 0=Idle@Origin, 1=To Corner1, 2..5=Square Sides, 6=Return Home
    t_elapsed = 0;
    coeffs_x = zeros(6, 1);
    coeffs_y = zeros(6, 1);
    q0_x = 0; q0_y = 0;
    qf_x = 0; qf_y = 0;
    T_current = 0;
    prev_enable = 0;
    current_pos = [0; 0];
end

% Define the 4 corners of the square (centered around the origin)
c1_x =  L/2;  c1_y =  L/2;   % Top-Right Corner
c2_x = -L/2;  c2_y =  L/2;   % Top-Left Corner
c3_x = -L/2;  c3_y = -L/2;   % Bottom-Left Corner
c4_x =  L/2;  c4_y = -L/2;   % Bottom-Right Corner

calc_coeffs = false;

%% 1. Check for Edge Transitions on the 'enable' Signal
if enable == 1 && prev_enable == 0
    % Rising Edge: Start moving from current position toward Corner 1
    state = 1;
    q0_x = current_pos(1);
    q0_y = current_pos(2);
    qf_x = c1_x;
    qf_y = c1_y;
    T_current = T_home;
    t_elapsed = 0;
    calc_coeffs = true;
    
elseif enable == 0 && prev_enable == 1
    % Falling Edge: Abort tracking and command a smooth path back to (0,0)
    state = 6;
    q0_x = current_pos(1);
    q0_y = current_pos(2);
    qf_x = 0;
    qf_y = 0;
    T_current = T_home;
    t_elapsed = 0;
    calc_coeffs = true;
end

%% 2. Check Segment Completion and Advance State Machine
if state ~= 0 && t_elapsed >= T_current
    t_elapsed = 0; % Reset segment timer
    calc_coeffs = true;
    
    if state == 1        % Reached Corner 1 -> Move to Corner 2
        state = 2;   q0_x = c1_x; q0_y = c1_y;  qf_x = c2_x; qf_y = c2_y;  T_current = T_segment;
    elseif state == 2    % Reached Corner 2 -> Move to Corner 3
        state = 3;   q0_x = c2_x; q0_y = c2_y;  qf_x = c3_x; qf_y = c3_y;  T_current = T_segment;
    elseif state == 3    % Reached Corner 3 -> Move to Corner 4
        state = 4;   q0_x = c3_x; q0_y = c3_y;  qf_x = c4_x; qf_y = c4_y;  T_current = T_segment;
    elseif state == 4    % Reached Corner 4 -> Move back to Corner 1
        state = 5;   q0_x = c4_x; q0_y = c4_y;  qf_x = c1_x; qf_y = c1_y;  T_current = T_segment;
    elseif state == 5    % Reached Corner 1 again -> Seamlessly loop to Corner 2
        state = 2;   q0_x = c1_x; q0_y = c1_y;  qf_x = c2_x; qf_y = c2_y;  T_current = T_segment;
    elseif state == 6    % Reached Origin -> Go to true idle state
        state = 0;   q0_x = 0;    q0_y = 0;     qf_x = 0;    qf_y = 0;     T_current = 0;
        calc_coeffs = false;
    end
end

%% 3. Analytical Quintic Coefficient Generation
if calc_coeffs
    dx = qf_x - q0_x;
    dy = qf_y - q0_y;
    
    if T_current > 0
        % Exact algebraic solution for boundary conditions: v0=vf=a0=af=0
        coeffs_x = [q0_x; 0; 0;  10*dx/(T_current^3); -15*dx/(T_current^4);  6*dx/(T_current^5)];
        coeffs_y = [q0_y; 0; 0;  10*dy/(T_current^3); -15*dy/(T_current^4);  6*dy/(T_current^5)];
    else
        coeffs_x = zeros(6,1); coeffs_x(1) = qf_x;
        coeffs_y = zeros(6,1); coeffs_y(1) = qf_y;
    end
end

%% 4. Trajectory Evaluation
t = t_elapsed;

if state == 0
    % Safe Idle at Origin
    x = 0;     x_dot = 0;
    y = 0;     y_dot = 0;
elseif t <= T_current
    % Evaluate Quintic Equations for Position & Velocity
    x     = coeffs_x(1) + coeffs_x(2)*t + coeffs_x(3)*t^2 +   coeffs_x(4)*t^3 +   coeffs_x(5)*t^4 +   coeffs_x(6)*t^5;
    x_dot = coeffs_x(2) + 2*coeffs_x(3)*t + 3*coeffs_x(4)*t^2 + 4*coeffs_x(5)*t^3 + 5*coeffs_x(6)*t^4;
    
    y     = coeffs_y(1) + coeffs_y(2)*t + coeffs_y(3)*t^2 +   coeffs_y(4)*t^3 +   coeffs_y(5)*t^4 +   coeffs_y(6)*t^5;
    y_dot = coeffs_y(2) + 2*coeffs_y(3)*t + 3*coeffs_y(4)*t^2 + 4*coeffs_y(5)*t^3 + 5*coeffs_y(6)*t^4;
else
    % Latch to target parameters if edge case timing occurs
    x = qf_x;  x_dot = 0;
    y = qf_y;  y_dot = 0;
end

% Package system outputs
r = [x; x_dot; y; y_dot];

%% 5. Memory Management for Next Sample Step
if state ~= 0
    t_elapsed = t_elapsed + Ts;
end

prev_enable = enable;
current_pos = [x; y];

end