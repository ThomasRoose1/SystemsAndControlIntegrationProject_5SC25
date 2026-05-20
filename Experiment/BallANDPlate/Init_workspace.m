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

