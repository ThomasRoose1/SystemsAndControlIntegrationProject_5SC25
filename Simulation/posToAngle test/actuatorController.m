function [I, next_x_ctrl] = actuatorController(h_target, h_actual, x_ctrl, dt)
    % A, B, C, D matrices for the 50Hz (300 rad/s) controller
    persistent Ac Bc Cc Dc
    if isempty(Ac)
        B_freq = 300; 
        Kc = 5.752e06; 
        num = Kc * conv([1 100], [1 50]); 
        den = conv([1 0], conv([1 900], [1 1800]));
        [Ac, Bc, Cc, Dc] = tf2ss(num, den);
    end

    error = h_target - h_actual;
    
    % State-space update (Euler integration)
    dx = Ac * x_ctrl + Bc * error;
    next_x_ctrl = x_ctrl + dx * dt;
    
    % Output current
    I = Cc * x_ctrl + Dc * error;
    
    % Hardware Saturation: Real system is limited to [-3, 3] Amps [cite: 348, 1318]
    I = max(min(I, 3), -3);
end