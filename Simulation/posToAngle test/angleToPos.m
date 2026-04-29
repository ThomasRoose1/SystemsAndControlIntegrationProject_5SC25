function [pos1, pos2, pos3] = angleToPos(alpha, beta, psi)
    % Returns the actuator positions required to reach the desired angle
    % note: the position is absolute in the global frame

    % Physical Constants
    r = 0.17;           % Joint radius (m)
    T = [0; 0; 0.322];  % Resting height vector (m)
    
    % Joint positions on base and plate (equilateral triangle)
    % These match the definitions in your posToAngle function
    theta = [0, deg2rad(240), deg2rad(120)];
    B = [r*cos(theta); r*sin(theta); zeros(1,3)]; % Base joints (b_i)
    P = [r*cos(theta); r*sin(theta); zeros(1,3)]; % Plate joints (p_i)
    
    % Rotation Matrices (Yaw-Pitch-Roll order)
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    Ry = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
    Rx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
    R_BtoP = Rz * Ry * Rx; 
    
    % Compute absolute position for each motor
    % Motor 1
    l1_vec = T + (R_BtoP * P(:,1)) - B(:,1);
    pos1 = norm(l1_vec);
    
    % Motor 2
    l2_vec = T + (R_BtoP * P(:,2)) - B(:,2);
    pos2 = norm(l2_vec);
    
    % Motor 3
    l3_vec = T + (R_BtoP * P(:,3)) - B(:,3);
    pos3 = norm(l3_vec);
end