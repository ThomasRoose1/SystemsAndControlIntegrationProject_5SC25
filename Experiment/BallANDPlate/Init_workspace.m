%% load into work space before building

fs = 1000;
fn = fs/2;

A = 2; % amplitude of multisine         CHANGE IF DESIRED
tlength = 200; % number of seconds       CHANGE IF DESIRED
N = tlength*fs;
P = 1; % periods

Ts_Outer = 0.1;                      %  CHANGE IF DESIRED


uA = multisine(0,fn,fs,N,A);
uA = repmat(uA,[1,P]);  % repeat periods if we want f-domain averaging

uB = multisine(0,fn,fs,N,A);
uB= repmat(uB,[1,P]);

uC = multisine(0,fn,fs,N,A);
uC = repmat(uC,[1,P]);



path = quintic(-0.0289,3,0,1/fs);

%% ball dynamics
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

%% State observer 
Ts_fast = 0.001; % 1000 Hz sample time matching the simulation rate

% Discretize continuous matrices for the fast estimation rate
sys_fast = c2d(ss(Ac, Bc, Cc, Dc), Ts_fast, 'zoh');
A_fast = sys_fast.A;
B_fast = sys_fast.B;
C_fast = sys_fast.C;

% Define your fast observer poles (safely inside the unit circle)
observer_poles = [0.90, 0.91, 0.92, 0.93];

% Compute the Observer Gain Matrix L
L = place(A_fast', C_fast', observer_poles)';

A_obs = A_fast - L*C_fast;
B_obs = [B_fast, L];
C_obs = eye(4);
D_obs = zeros(4,4);

