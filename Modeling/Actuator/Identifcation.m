clear, close, clc


dataA = load("motorA_2A.mat");
dataB = load("motorB_2A.mat");
dataC = load("motorC_2A.mat");
YA = dataA.motorA_2A.Y;
ma = YA(1).Data;
mb = YA(4).Data;
mc = YA(7).Data;

ua = YA(3).Data;
ub = YA(6).Data;
uc = YA(9).Data;
% e = load;
ya = YA(2).Data;
yb = YA(5).Data;
yc = YA(8).Data;


tol = 1e-6;
idx = find(abs(ma) > tol);

ma = ma(idx);
ua = ua(idx); %ua = ua - mean(ua);
ya = ya(idx); %ya = ya - mean(ya);

data_id = iddata(ya.',ua.', 0.001);


r  = zeros(1,length(idx)); % reference was 0

ea = r - ya;
ea = -ea;


% sysOE = oe(data_id,[4 4 1]);
sysBJ = bj(data_id,[4 4 4 4 1]);

figure
% resid(data_id, sysOE)
resid(data_id, sysBJ)

%General settings
fs = 1000;
fres = 0.5;
nfft = fs/fres;  %number of frames, taken to avoid noise for example
noverlap = 0.5*nfft;
window = hann(nfft);
window = rectwin(nfft);

% e = -y(:,1);
% u1test = u(:,1);

[P_ud, f_s] = cpsd(ua, ma, window, noverlap, nfft, fs);   % P_ud = S_ud
[P_ed, ~] = cpsd(ea, ma, window, noverlap, nfft, fs);   % P_ed = S_ed


H = P_ed ./ P_ud;


H_mag = mag2db(abs(H));
H_phase = rad2deg(angle(H));

% Gd = load("Plantje_discrete.mat");
% Gd = Gd.Gd;
% g11 = Gd(1,1);
% [mag_gt, phase_gt] = bode(g11, 2*pi*f_s);
% mag_gt = squeeze(mag_gt);
% phase_gt = squeeze(phase_gt);
% mag_gt_db = 20*log10(mag_gt);
% phase_gt_deg = phase_gt;



[mag_bj, phase_bj] = bode(sysBJ, 2*pi*f_s);

mag_bj = squeeze(mag_bj);
mag_bj_db = 20*log10(mag_bj);
phase_bj_deg = squeeze(phase_bj);

figure;

% Magnitude Subplot
subplot(2,1,1);
semilogx(f_s, H_mag, 'b', 'LineWidth', 1.2); hold on;
% semilogx(f_s, mag_gt_db, 'r--', 'LineWidth', 1.2);
semilogx(f_s, mag_bj_db, 'g', 'LineWidth', 1.5); % Added sysBJ (Green)
grid on;
ylabel('Magnitude [dB]');
% legend('Estimated (CPSD)', 'Ground truth (g11)', 'Box-Jenkins (sysBJ)');
legend('Estimated (CPSD)', 'Box-Jenkins (sysBJ)');

% Phase Subplot
subplot(2,1,2);
semilogx(f_s, H_phase, 'b', 'LineWidth', 1.2); hold on;
% semilogx(f_s, phase_gt_deg, 'r--', 'LineWidth', 1.2);
semilogx(f_s, phase_bj_deg, 'g', 'LineWidth', 1.5); % Added sysBJ (Green)
grid on;
ylabel('Phase [deg]');
xlabel('Frequency [Hz]');
% legend('Estimated (CPSD)', 'Ground truth (g11)', 'Box-Jenkins (sysBJ)');
legend('Estimated (CPSD)', 'Box-Jenkins (sysBJ)');