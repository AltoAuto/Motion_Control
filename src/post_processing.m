%% Lab 4 - Phase 3 Post-Processing
clear; close all; clc;                           % reset workspace/figures for a clean run

Fs = 1000;          % Hz                         % sampling frequency used during data logging (given by lab)
Ts = 1/Fs;          % seconds per sample         % sample period; converts sample index -> real time

fname = '50_10test.txt';                         % input data file: 2 columns [RawCounts, FilteredCounts]

Tdelay =  1.071000000000000;                    % seconds to skip before RMS (filter start-up transient)

% Expect data format: each row -> [raw_count, filtered_count]
data = readmatrix(fname);                        % reads numeric CSV / whitespace-delimited files
data = data(all(isfinite(data),2), :);           % safety: remove any rows with NaN/Inf (headers/blank lines)

raw  = data(:,1);                                % raw encoder counts (often sinusoid + drift/ramp)
filt = data(:,2);                                % filtered counts (high-pass output)
N    = length(raw);                              % number of samples in record (assumes both cols same length)

% create time vector
t = (0:N-1)*Ts;                                  % time vector aligned with sample index n (0...N-1)


% Lab said manual, so I eye ball Filterd, identify when amplitude stops
% decaying -> oscillation about ~ 0 set T_delay to that time
% Meaning: do NOT include early samples in RMS because IIR filter initial conditions
% (and the "turn-on" event) create a transient that biases RMS / amplitude estimates.
idx0 = floor(Tdelay*Fs) + 1;                     % convert delay time (sec) -> starting sample index
idx0 = max(idx0,1);                              % safety: prevent idx0 from being < 1
fseg = filt(idx0:end);                           % segment used for RMS: "steady-state" portion only

% --- 2. Slice the arrays into Transient vs. Steady-State ---
% Transient arrays (from start up to just before idx0)
t_transient    = t(1:idx0-1);
filt_transient = filt(1:idx0-1);
% Steady-state arrays (from idx0 to the end)
t_steady    = t(idx0:end);
filt_steady = filt(idx0:end);

% plot raw vs filterd
figure;
plot(t, raw, 'r', 'DisplayName', 'Raw Output');
hold on;
plot(t_transient, filt_transient, 'b', 'DisplayName', 'Filtered (Transient)'); 
plot(t_steady, filt_steady, 'g', 'DisplayName', 'Discarding Filter Transient'); 
xlabel('Time (s)'); 
ylabel('Counts');
title('Raw vs Filtered Encoder Counts (Transient Isolation)');
legend;


% Remove residual DC bias (prevents RMS inflation)
% Even with a high-pass, a small mean can remain due to finite record length or incomplete settling.
% Subtracting the mean ensures RMS represents oscillation energy, not leftover offset.
fseg0 = fseg - mean(fseg);

Crms = sqrt(mean(fseg0.^2));                     % RMS counts of steady-state filtered signal

% For a zero-mean sinusoid: RMS = Amp/sqrt(2)  ->  Amp = sqrt(2)*RMS
Camp = sqrt(2)*Crms;                             % estimated sinusoid amplitude in counts (peak, not p-p)

% Input amplitude bookkeeping:
% Manual convention: "50_10test" corresponds to 5.0 Hz, 1.0 V amplitude (peak).
Vamp_in = 1.0;                                   % volts amplitude (from filename convention)

% Gain is an AMPLITUDE ratio (counts per volt). For amplitude ratios use 20*log10(.), not 10*log10(.)
Gain    = Camp / Vamp_in;                        % amplitude gain (counts/Volt)
Gain_dB = 20*log10(Gain);                        % gain in dB

% Print results for your post-lab writeup
fprintf('Tdelay = %.3f s\n', Tdelay);
fprintf('Crms   = %.6f counts\n', Crms);
fprintf('Camp   = %.6f counts (amplitude)\n', Camp);
fprintf('Gain   = %.6f counts/Volt\n', Gain);
fprintf('Gain_dB= %.3f dB\n', Gain_dB);
