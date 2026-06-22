function [rsample, asample, rpreview] = ref_player(r, a, enable)
%#codegen

Npreview = 25; % number of samples preview for MPC

% Persistent variable to track the current sample index
persistent idx_ref;

% Initialize the index on the very first time step
if isempty(idx_ref)
    idx_ref = 1;
end

% Determine the total number of offline samples available
N_total = size(r, 2);

% Preallocate outputs for code generation stability
rsample = zeros(4, 1);
asample = zeros(2, 1);
rpreview = zeros(4, Npreview);

% --- Trajectory Generation Logic ---
if enable == 1
    % 1. Extract the current sample
    if idx_ref <= N_total
        rsample = r(:, idx_ref);
        asample = a(:, idx_ref);
    else
        % Hold the final sample if the trajectory ends but enable stays 1
        rsample = r(:, N_total);
        asample = a(:, N_total);
    end
    
    % 2. Extract the next Npreview samples for MPC
    for i = 1:Npreview
        preview_idx = idx_ref + i; 
        
        if preview_idx <= N_total
            rpreview(:, i) = r(:, preview_idx);
        else
            % Zero-order hold: Pad with the last available reference state 
            % if the preview window extends past the end of the data array
            rpreview(:, i) = r(:, N_total);
        end
    end
    
    % 3. Advance the index for the next sample time (Ts)
    if idx_ref <= N_total
        idx_ref = idx_ref + 1;
    end

else
    % --- Reset / Idle State (enable == 0) ---
    % Keep index reset to the first sample
    idx_ref = 1; 
    
    % Output the initial conditions while waiting
    rsample = r(:, 1);
    asample = a(:, 1);
    
    % Provide a static look-ahead from the starting point
    for i = 1:Npreview
        if (1 + i) <= N_total
            rpreview(:, i) = r(:, 1 + i);
        else
            rpreview(:, i) = r(:, N_total);
        end
    end
end