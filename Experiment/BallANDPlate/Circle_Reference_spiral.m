function r = Circle_Reference_spiral(R, omega, Ts, reset)
% CIRCLE_REFERENCE Generates [x; xdot; y; ydot] for a spiral-in/spiral-out trajectory.

% Persistent variables to track phase and the dynamic radius
persistent theta current_R

% Initialize persistent variables on the very first run
if isempty(theta)
    theta = 0;
end
if isempty(current_R)
    current_R = 0; % Start at the origin (0,0)
end

% Define how long you want the spiral transition to take (in seconds)
T_ramp = 3.0; 
R_rate = R / T_ramp; % Radial speed (units per second)

% Determine target radius based on the reset flag
if reset
    R_target = 0;  % Spiral inward to origin
else
    R_target = R;  % Spiral outward to full circle
end

% Smoothly update the current radius and compute dR (radial velocity)
if current_R < R_target
    dR = R_rate;
    current_R = current_R + dR * Ts;
    if current_R > R_target
        current_R = R_target;
        dR = 0;
    end
elseif current_R > R_target
    dR = -R_rate;
    current_R = current_R + dR * Ts;
    if current_R < R_target
        current_R = R_target;
        dR = 0;
    end
else
    dR = 0; % Radius is at target, no radial velocity
end

% Compute current positions and velocities
% Note: Velocities now include the dR component for perfect kinematic accuracy!
x     = current_R * cos(theta);
x_dot = dR * cos(theta) - current_R * omega * sin(theta);

y     = current_R * sin(theta);
y_dot = dR * sin(theta) + current_R * omega * cos(theta);

r = [x; x_dot; y; y_dot];

% Integrate the phase for the next time step
next_theta = theta + omega * Ts;

% Wrap theta to [0, 2*pi] to protect numerical precision
theta = mod(next_theta, 2*pi);
end