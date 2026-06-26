function [alpha, beta, psi] = posToAngle(pos1, pos2, pos3)
    % Returns the angles resulting from the given position
    % note: the position should be relative from the actuators midpoint in the global frame

    % Physical Constants
    r = 0.17;                 % Radius to joints (m)
    T = 0.322;                % Resting height (m)
    theta = deg2rad(120);     % angle between bases (rad)

    % Reconstruction of the Plate Unit Vectors 
    % We find where the joints are in the world frame by using the inputs
    % If pos = 0.322, the joints are exactly T_height above the base.
    P1_global = [r; 0; T + pos1];
    P2_global = [r*cos(theta); r*sin(theta); T + pos2];
    P3_global = [r*cos(theta); -r*sin(theta); T + pos3];

    % Find the center of the tilted plate
    % The average of the three joint positions gives the center of the motion plate
    center = (P1_global + P2_global + P3_global) / 3;

    % Vectors from the plate center to the joints
    % This isolates the tilt of the plate surface
    v1 = P1_global - center;
    v2 = P2_global - center;
    v3 = P3_global - center;

    % Compute Unit Vectors
    uX = v1 / norm(v1);             % Plate X-axis
    u_temp = cross(uX, v3);         % Temporary vector to find normal
    uZ = u_temp / norm(u_temp);     % Plate Z-axis (Normal)
    uY = cross(uZ, uX);             % Plate Y-axis

    % Extract Tait-Bryan Angles
    beta = asin(-uX(3));                            
    alpha = asin(uY(3) / sqrt(1 - uX(3)^2));       
    psi = asin(uX(2) / sqrt(1 - uX(3)^2));
end
