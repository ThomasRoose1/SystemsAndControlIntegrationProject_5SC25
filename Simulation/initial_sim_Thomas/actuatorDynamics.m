function [h_new, next_x_plant] = actuatorDynamics(I, x_plant, dt)
    % A, B, C, D matrices for the motor plant [cite: 415, 827]
    persistent Ap Bp Cp
    if isempty(Ap)
        m = 0.417; Km = 11; Cv = 16.5;
        % Manually define A, B, C for a mass-damper system:
        % States: x1 = position, x2 = velocity
        % dx1 = x2
        % dx2 = (Km*I - Cv*x2) / m
        Ap = [0, 1; 0, -Cv/m];
        Bp = [0; Km/m];
        Cp = [1, 0];
    end

    % Physical state update
    dx = Ap * x_plant + Bp * I;
    next_x_plant = x_plant + dx * dt;

    % New physical position
    h_new = Cp * next_x_plant;
end