function u_cmd = run_outer_loop_mpc(x_estimate)
    % This function executes directly in the MATLAB base workspace
    persistent mpc_ptr;
    
    % Fetch the compiled YALMIP optimizer object from the workspace once
    if isempty(mpc_ptr)
        mpc_ptr = evalin('base', 'MPC_sparse');
    end
    
    % Evaluate the optimization problem using the current state estimate
    sol = mpc_ptr(x_estimate);
    
    % Ensure the solver returned a valid numerical vector. 
    % If it returns a YALMIP error string, hold the plate flat to protect hardware.
    if ischar(sol) || isempty(sol) || any(isnan(sol))
        u_cmd = [0; 0]; 
    else
        u_cmd = sol; 
    end
end