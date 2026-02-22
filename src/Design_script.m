
format long     

clc;

Fs = 1000;  % sampling frequency [Hz]
Fc = 1;     % cut-off frequency [Hz]
N = 2;      %  2nd order filter

Wn = Fc/(Fs/2);     % normalized co-frequncy
%Wn = Fc/Fs        % wiil try for fun

[b,a] = butter(N, Wn, 'high'); 

% very informal way in print a and b
b
a

% compute pole zeros
[z,p,k] = tf2zpk(b,a);
z
p
k

% Magnitude response plot
nfft = 1000000;
[H,f] = freqz(b,a,nfft,Fs);         % f in Hz
mag_db = 20*log10(abs(H));

figure;
semilogx(f, mag_db); grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('High-pass Butterworth: N=2, Fc=1 Hz, Fs=1000 Hz');
xlim([0.05 100]);     

% calculate slop
f1 = 0.01; f2 = 0.1;    
m1 = interp1(f, mag_db, f1);
m2 = interp1(f, mag_db, f2);
slope_db_per_dec = (m2 - m1) / (log10(f2) - log10(f1));
fprintf('decade slope between 0.01 and 0.1 Hz: %f \n', slope_db_per_dec);

% calculate gain at cut off frequency
mag_at_Fc_db = interp1(f, mag_db, Fc);
fprintf('gain at cut off frequency: %f \n', mag_at_Fc_db);

% step response of the digital filter
T = 5;                        % seconds to view
Ns = T*Fs;
y_step = stepz(b,a,Ns);
t_step = (0:Ns-1)/Fs;
figure;
plot(t_step, y_step); grid on;
xlabel('Time (s)'); ylabel('Output');
title('High-pass filter step response (settling shows transient length)');

% pick delay
eps = 0.01*max(abs(y_step));
hold_s = 0.25;             % must stay settled for 0.25 s
K = round(hold_s*Fs);

idx = find(movmax(abs(y_step),[K-1 0]) < eps, 1, 'first');
Tdelay = t_step(idx);

Tdelay
