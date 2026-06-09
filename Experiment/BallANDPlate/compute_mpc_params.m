function MPC_params = compute_mpc_params(Q,R,N, Ts, max_pos, max_vel)
    %% System Parameters & Continuous State-Space Model
    g = 9.81;                  % Gravity (m/s^2)
    km_factor = (5/7) * g;     % Rolling ball constant (~7.0071)
    
    % Continuous Matrices (4 states, 2 inputs, 2 outputs)
    Ac = [0 1 0 0;
          0 0 0 0;
          0 0 0 1;
          0 0 0 0];
      
    Bc = [0          0;
          0          km_factor;
          0          0;
         -km_factor  0];
      
    Cc = [1 0 0 0;
          0 0 1 0];
      
    Dc = [0 0;
          0 0];
    
    %% Discretization slow loop (for MPC solver)
    sys_c = ss(Ac, Bc, Cc, Dc);
    sys_d = c2d(sys_c, Ts, 'zoh');
    
    % Extract Discrete Matrices
    A = sys_d.A;
    B = sys_d.B;
    C = sys_d.C;
    D = sys_d.D;
    
    % dimensions
    nx = size(A,1);
    ny = size(C,1);
    nu = size(B,2);
    
    %% Constraints
    % state constraints
    Hx = [-1 0 0 0;
          1  0 0 0;
          0 -1 0 0;
          0 1  0 0;
          0 0 -1 0;
          0 0 1  0;
          0 0 0 -1;
          0 0 0  1];
    hx = [max_pos;
          max_pos;
          max_vel;
          max_vel;
          max_pos;
          max_pos;
          max_vel;
          max_vel];
    X_set = Polyhedron(Hx, hx);
    
    % Project position states onto 2D plane
    X_projected = X_set.projection([1, 3]);
    
    % Input constraints
    max_angle = deg2rad(10); % plate should not tilt beyond 10 degrees;
    Hu = [-1 0; 1 0; 0 -1; 0 1];
    hu = [max_angle; max_angle; max_angle; max_angle];
    U_set = Polyhedron(Hu,hu);
    
    % Output contraints
    Hy = [-1 0; 1 0; 0 -1; 0 1];
    hy = [max_pos; max_pos; max_pos; max_pos];
    Y_set = Polyhedron(Hy,hy);
    
    %% Compute terminal cost    
    [K, P, ~] = dlqr(A, B, Q, R); % Terminal cost
    K = -K;
    
    % Check eigenvalues
    A_cl = A + B*K;
    fprintf('Closed loop eigenvalues: \n');
    abs(eig(A_cl))
    
    %% Compute terminal set
    % Define autonomous LTI closed loop system
    model = LTISystem('A', A_cl);
    
    % Define constraint admissable set
    U_CA_set = Polyhedron(Hu*K, hu); % Input constraint admissable set
    CA_set = U_CA_set & X_set; % constraint admissable set (intersection)
    
    % Find maximal invariant set of CA_set
    Inv_set = model.invariantSet('X', CA_set);
    
    % Extract constraint matrices
    HT = Inv_set.A;
    hT = Inv_set.b;
  
    %% Define MPC params    
    % Compute standard MPC matrices
    [Phi,Gamma,Omega,Psi] = mpc_obj(A,B,Q,R,P,N);
    Theta = kron(eye(N), C);
    
    % Compute matrices for condensed forumlation
    G = full(Psi + Gamma'*Omega*Gamma);
    F = full(Gamma'*Omega*Phi);
    
    % Force perfect numerical symmetry to prevent real-time console warnings
    G = 0.5 * (G + G');
    
    % Expand constraints over horizon N
    Hu_bar = kron(eye(N), Hu);
    hu_bar = repmat(hu, N, 1);
    Hy_bar = kron(eye(N), Hy);
    hy_bar = repmat(hy, N, 1);
    
    % Condense the constraints into a single polyhedron
    % Extract the final prediction block row of Gamma and Phi for terminal
    % constraint
    Gamma_N = Gamma(end-nx+1:end, :); 
    Phi_N = Phi(end-nx+1:end, :);
    
    % Define LHS
    A_condensed = full([ Hy_bar * Theta * Gamma; ...
                    Hu_bar; ...
                    HT * Gamma_N ]);
    
    % Define RHS
    b_static = full([ hy_bar; ...
                 hu_bar; ...
                 hT ]);
    
    W_state  = full([ Hy_bar * Theta * Phi; ...
                 zeros(size(Hu_bar, 1), nx); ...
                 HT * Phi_N ]);
    
    %% Define param struct to pass to MPC function
    MPC_params.G = G;
    MPC_params.F = F;
    MPC_params.A_condensed = A_condensed;
    MPC_params.b_static = b_static;
    MPC_params.W_state = W_state;
end