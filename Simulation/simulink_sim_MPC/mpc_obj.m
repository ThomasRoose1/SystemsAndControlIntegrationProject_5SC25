% 5LMB0 Model predictive control
% Author: Duo Xu
function [Phi,Gamma,Omega,Psi] = mpc_obj(A,B,Q,R,P,N)
Phi = A;
Gamma = kron(eye(N),B);
for k = 1:N-1
    Phi = [Phi;A^(k+1)];
    Gamma = Gamma + kron(diag(ones(1,N-k),-k),A^k*B);
end
Omega = blkdiag(kron(eye(N-1),Q),P);
Psi = kron(eye(N),R);
end