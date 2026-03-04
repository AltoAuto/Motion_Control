%% Lab 5 - System Identification Batch Processor
clear; close all; clc;

% Define the Sweep Parameters 
test_amps  = [0.5, 1.0];
test_freqs = [2.5, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, ...
              40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0];

Fs = 1000;           % Hz (Sampling Frequency)
Tdelay = 1.071;      % Seconds to skip for filter transient

% Pre-allocate arrays
Mag_dB_05V     = zeros(1, length(test_freqs));
Mag_dB_10V     = zeros(1, length(test_freqs));
Mag_dB_05V_bw  = zeros(1, length(test_freqs));
Mag_dB_10V_bw  = zeros(1, length(test_freqs));

fprintf('Starting Batch Process...\n');

%% Process Every File Automatically
for a_idx = 1:length(test_amps)
    Amp_in = test_amps(a_idx);
    
    for f_idx = 1:length(test_freqs)
        Freq_in = test_freqs(f_idx);
        
        % Construct the filename
        fname = fullfile('Test_run', sprintf('sweep_%.1fV_%.1fHz.txt', Amp_in, Freq_in));
        
        % Safety check
        if ~isfile(fname)
            warning('File %s not found! Skipping...', fname);
            continue;
        end
        
        % Read the data
        % Row 1 [Freq, Amp]. Row 2 to End is [Raw, Filtered]
        raw_data = readmatrix(fname);
        raw  = raw_data(2:end, 1);  % Column 1: raw encoder counts
        filt = raw_data(2:end, 2);  % Column 2: real-time HP filtered counts
        
        % Slice the steady-state segment
        idx0 = floor(Tdelay * Fs) + 1;
        if idx0 > length(filt)
            warning('File %s is too short! Skipping...', fname);
            continue;
        end
        
        % Input peak-to-peak voltage
        In_PP = 2 * Amp_in;

        % ── Series A: Real-time C++ filtered ──────────────────────────────
        fseg  = filt(idx0:end);
        fseg0 = fseg - mean(fseg);
        Crms_A    = sqrt(mean(fseg0.^2));
        Out_PP_A  = 2 * sqrt(2) * Crms_A;
        Gain_dB_A = 20 * log10(Out_PP_A / In_PP);

        % ── Series B: Butterworth HP filter on raw counts ─────────────────
        Fc = Freq_in / 2;                        % cutoff = half the test frequency
        [b, a] = butter(2, Fc/(Fs/2), 'high');
        raw_bw = filter(b, a, raw);

        bseg  = raw_bw(idx0:end);
        bseg0 = bseg - mean(bseg);
        Crms_B    = sqrt(mean(bseg0.^2));
        Out_PP_B  = 2 * sqrt(2) * Crms_B;
        Gain_dB_B = 20 * log10(Out_PP_B / In_PP);

        % ── Store results ─────────────────────────────────────────────────
        if Amp_in == 0.5
            Mag_dB_05V(f_idx)    = Gain_dB_A;
            Mag_dB_05V_bw(f_idx) = Gain_dB_B;
        elseif Amp_in == 1.0
            Mag_dB_10V(f_idx)    = Gain_dB_A;
            Mag_dB_10V_bw(f_idx) = Gain_dB_B;
        end
        
        fprintf('Processed %s -> RT: %.2f dB | BW: %.2f dB\n', fname, Gain_dB_A, Gain_dB_B);
    end
end

%% Generate the Bode Plot
omega_rad_sec = test_freqs * 2 * pi;

figure('Name', 'DC Servomotor System Identification', 'Color', 'w');

semilogx(omega_rad_sec, Mag_dB_05V,    'bo-',  'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', '0.5V – RT Filter (C++)');
hold on;
semilogx(omega_rad_sec, Mag_dB_10V,    'rs-',  'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', '1.0V – RT Filter (C++)');
semilogx(omega_rad_sec, Mag_dB_05V_bw, 'b^--', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', '0.5V – MATLAB BW Filter');
semilogx(omega_rad_sec, Mag_dB_10V_bw, 'rv--', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', '1.0V – MATLAB BW Filter');

grid on;
xlabel('Frequency \omega (rad/sec)', 'FontWeight', 'bold');
ylabel('Magnitude 20*log_{10}(Counts/Voltage) (dB)', 'FontWeight', 'bold');
title('Open Loop Bode Plot of DC Servomotor', 'FontWeight', 'bold');
legend('Location', 'southwest');
xlim([1, 1000]);
