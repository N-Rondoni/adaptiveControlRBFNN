clear all; close all; clc;

%  This code generates figure 3 of the accompanying manuscript. 

%% ========================================================================
%  RBF Controller with the following noise flags/sample ranges zeroed out. 
% 
% Ablation_Mode = 'None', sigma =  0, MEAS_template.enable = false
% 
% Measurement Noise/Bias, Plant Perturbations,
%  and Actuator Jitter — Setpoint Sweep Implementation
%
%  Comparable noise model to the Adaptive PID (GD) code:
%    - Measurement LPF, no bias estimator (let weights learn it) at each T_measure sample
%    - Plant parameter perturbations (same struct as PID)
%    - Actuator gain mismatch, delay, and jitter in EF delivery
%    - Sinusoidal disturbance on T4_ext
%
%  NO band termination — runs to tf for every setpoint/sigma combo.
%  A_basal floor: once T4 stays in band for holdN consecutive amplitude
%  windows, amplitude is floored at the strength when band was entered.
%
%  INTEGRAL ACTION: ki accumulates ONLY at measurement times (synchronized
%  with weight updates) to prevent ramp-within-hold artifacts, but is
%  currently set to 0 (off) because it didn't seem to improve performance.
%
%  Logs: rise time, overshoot, SS Mean Error, SS RMSE, SS NRMSE, termination time.
%
% Takes amplitude at moment of lock, not min_A, or A_streak. Fancier A lock
% not needed in the noiseless case. 
%% ========================================================================


% ---- Save settings ----
DO_SAVE_PDF = false;
output_dir = fullfile(pwd, 'results_rbf_no_noise_no_ki');
if ~exist(output_dir,'dir'), mkdir(output_dir); end

fprintf('Current Folder (pwd): %s\n', pwd);
fprintf('Output directory:      %s\n', output_dir);

%% ==================== SETPOINT SWEEP CONFIGURATION ====================
setpoint_values = [12, 16, 20, 25];
num_setpoints   = numel(setpoint_values);

%% ==================== SIGMA SWEEP CONFIGURATION ====================
% Retained for future use — set to single value for setpoint sweeps
%sigma_values = linspace(0.02, 0.20, 5) .* T4_des_ref;  % example for multi-sigma
sigma_values = 0;
num_sigma    = numel(sigma_values);

%% ==================== ABLATION MODE ======================================
%  Controls which noise sources are active:
%    'full'            — all perturbations + measurement noise (default, matches PID)
%    'none'            — nominal plant, measurement noise only (isolates sensor noise)
%    'half'            — perturbation ranges halved, measurement noise on
%    'full_no_rhythm'  — same as full, no sine curve disturbance.
ABLATION_MODE = 'none';   % <-- change to above settings for ablation runs

%% ==================== BAND template (setpoint-dependent fields set per run) =====
BAND_pct_val = 0.20;   % ±20% band for lock logic


% ---- Build filename tags ----
sp_tag = strjoin(arrayfun(@(v) sprintf('%g', v), setpoint_values, 'UniformOutput', false), '_');
sigma_tag = strjoin(arrayfun(@(v) strrep(sprintf('%.2f', v), '.', 'p'), sigma_values, 'UniformOutput', false), '_');
band_tag = sprintf('bnd%d', round(BAND_pct_val * 100));

% Results storage filename
results_file = fullfile(output_dir, sprintf('RBF_setpoint_sweep_results_%s_%s_sp_%s_sigma_%s.mat', ...
    ABLATION_MODE, band_tag, sp_tag, sigma_tag));

% Check for partial results to resume from
if exist(results_file, 'file')
    fprintf('Found existing results file. Loading...\n');
    load(results_file, 'ALL_RESULTS', 'completed_runs');
    fprintf('Resuming from run %d of %d\n', completed_runs + 1, num_setpoints * num_sigma);
else
    ALL_RESULTS    = struct();
    completed_runs = 0;
end

%% -------------------- Time Unit Conversion --------------------
SEC_PER_MIN = 60;
toUnit   = @(sec)  sec / SEC_PER_MIN;     % seconds -> minutes
fromUnit = @(unit) unit * SEC_PER_MIN;     % minutes -> seconds

%% -------------------- Timing parameters --------------------
dt   = 0.1/12;          % ~0.5 sec in minutes
ti   = 0;
tf   = 4500;            % minutes (~33.3 h)
time_vector = ti:dt:tf;
len  = length(time_vector);

%% -------------------- Time-varying rates --------------------
alpha1_0 = 3.308; epsilon_alpha1 = 0;
alpha1_func = @(t) alpha1_0 + epsilon_alpha1 * t;

gammaext1_0 = 0.01; epsilon_gamma = 0;
gammaext_T4_time = @(t) gammaext1_0 + epsilon_gamma * t;

M0 = 0.15; epsilon_M = 0; t_sM = 0; delta_M = 0; epsilon2_M = 0;
M_time = @(t) (M0 + epsilon_M * t) + ...
              (delta_M + (epsilon2_M - epsilon_M)*(t - t_sM)) .* (t >= t_sM);

% Minute versions
gammaext_T4_time_min = @(t_min) gammaext_T4_time(fromUnit(t_min));
M_time_min           = @(t_min) M_time(fromUnit(t_min));
alpha1_func_min      = @(t_min) alpha1_func(fromUnit(t_min));

%% -------------------- Burst & micro-pulse parameters --------------------
t1 = toUnit(500e-6);  t2 = toUnit(100e-6);
t3 = toUnit(500e-6);  t4 = toUnit(9.5e-4);
Tp = t1 + t2 + t3 + t4;
pulseDuty = (t1 + t3)/Tp;

np = 500;
t5 = np * Tp;          % burst ON (minutes)
t6 = toUnit(0.5);      % gap (minutes)

%% -------------------- Window parameters --------------------
Ts        = 120;        % amplitude hold window (minutes)
r_duty    = 0.60;
T_on      = r_duty * Ts;
T_cycle   = Ts;
T_total   = tf;

T_measure = 20;         % T4 measurement interval (minutes)

%% -------------------- EF to Amplitude mapping --------------------
thresh = 1e-4;
A_min  = 0.00;  A_max = 0.10;  kA = 0.20;

%% -------------------- RBF parameters --------------------
Nc = 40;                              % number of centres
c1d = linspace(0, 1, Nc);
c_template = repmat(c1d, [16, 1]);    % 16-dim centres on [0,1]

beeta = 0.13;
learning_rate_base = 1e-6 * (T_measure / dt);

% Integral action (synchronized with measurement updates)
ki = 0;
e_int_max = 5;                        % hard cap on integrator state

% Saturation limits on RBF output
UL = 100;  LL = 0;

%% ==================== Measurement model (template) ====================
MEAS_template.enable    = false;
MEAS_template.alpha_y   = 0.30;    % measurement LPF (0..1)

%% ==================== Perturbation config (TRUE plant) ==================
PERT_TRUE.enable = true;

PERT_TRUE.range.alpha_scale   = 0.25;
PERT_TRUE.range.K_scale       = 0.25;
PERT_TRUE.range.gamma1_scale  = 0.25;
PERT_TRUE.range.kappa_scale   = 0.20;
PERT_TRUE.range.K2_scale      = 0.20;
PERT_TRUE.range.gammaI_scale  = 0.25;
PERT_TRUE.range.gammaTg_scale = 0.25;
PERT_TRUE.range.gint_scale    = 0.25;
PERT_TRUE.range.gext_scale    = 0.25;
PERT_TRUE.range.M_scale       = 0.25;
PERT_TRUE.range.alpha1_scale  = 0.25;

PERT_TRUE.range.d4_bias  = 0.02;
PERT_TRUE.range.d4_amp   = 0.03;
PERT_TRUE.d4_period_min  = 6*60;       % 6-hour sinusoidal disturbance

PERT_TRUE.range.act_gain   = 0.20;
PERT_TRUE.range.delay_min  = toUnit(10);   % delay in minutes
PERT_TRUE.range.jitter_min = toUnit(5);    % jitter in minutes

%% ==================== Perturbation sampling (respects ABLATION_MODE) =====
% Sample TRUE plant perturbation (same seed = same plant as PID)
rng(1);
switch ABLATION_MODE
    case 'none'
        pert_true = neutral_perturbation(PERT_TRUE.d4_period_min);
        fprintf('*** ABLATION: perturbations OFF (nominal plant) ***\n');
    case 'half'
        half_range = PERT_TRUE.range;
        fnames = fieldnames(half_range);
        for fi = 1:numel(fnames)
            half_range.(fnames{fi}) = half_range.(fnames{fi}) * 0.5;
        end
        pert_true = sample_perturbation(half_range, PERT_TRUE.d4_period_min);
        fprintf('*** ABLATION: perturbations at 50%% scale ***\n');
    case 'full_no_rhythm'
        pert_true = sample_perturbation(PERT_TRUE.range, PERT_TRUE.d4_period_min);
        pert_true.d4_bias = 0;
        pert_true.d4_amp  = 0;
        fprintf('*** ABLATION: full perturbations, biological rhythm OFF ***\n');
    otherwise  % 'full'
        pert_true = sample_perturbation(PERT_TRUE.range, PERT_TRUE.d4_period_min);
        fprintf('*** ABLATION: full perturbations ***\n');
end


%% ==================== MAIN LOOP OVER SETPOINTS x SIGMA ====================
total_runs = num_setpoints * num_sigma;

for sp_idx = 1:num_setpoints
    T4_des = setpoint_values(sp_idx);

    % ---- Setpoint-dependent parameters ----
    W_max = (T4_des/1000 + 0.005);                  % linearly interpolated cap
    MEAS_bias_true = 0;%0.05 * T4_des;                  % 5% systematic sensor bias

    BAND_template_run.enable    = true;
    BAND_template_run.pct       = BAND_pct_val;
    BAND_template_run.width     = BAND_pct_val * T4_des;
    BAND_template_run.holdN     = 2;
    BAND_template_run.entered   = false;
    BAND_template_run.inband_ct = 0;
    BAND_template_run.reported  = false;
    BAND_template_run.min_A     = inf;   % initialize, wanna take the min across holdn windows

    for sigma_idx = 1:num_sigma
        run_number = (sp_idx - 1) * num_sigma + sigma_idx;
        if run_number <= completed_runs
            fprintf('Skipping run %d/%d (already completed)\n', run_number, total_runs);
            continue;
        end

        current_sigma = sigma_values(sigma_idx);
        fprintf('\n========================================\n');
        fprintf('Run %d/%d: T4_des = %g, sigma = %.2f\n', run_number, total_runs, T4_des, current_sigma);
        fprintf('========================================\n');

        % ---- Set up MEAS for this run ----
        MEAS            = MEAS_template;
        MEAS.sigma      = current_sigma;
        MEAS.bias_true  = MEAS_bias_true;

        % ---- Reset measurement state ----
        meas_state.y_filt = NaN;

        % ---- Reset RBF weights & centres ----
        W  = 0.5 * W_max * ones(1, Nc);
        c  = c_template;                      % fresh centres each run
        learning_rate = learning_rate_base;

        % ---- State & log initialisation ----
        y0=[229.437253584811; 0.273804879029534;...
        22.9437253584891; 4.10707318544213;...
        zeros(11,1);25]; % Ksenia's found stable state

        y_log = zeros(16, len);
        y_log(:,1) = y0;

        ref = T4_des * ones(1, len);          % constant setpoint

        new_in = zeros(16, len);
        new_in(1,:) = ref;
        x = 0;                                % initial "measured" T4
        new_in(2,1) = x;

        u       = zeros(1, len);
        e       = ref(1) - x;
        e_int   = 0;

        log_d   = zeros(3, len);
        log_d(:,1) = [x; 0; e];
        Weights = zeros(len, Nc);
        Weights(1,:) = W;

        A_log   = zeros(1, len);
        EF_log  = zeros(1, len);
        T4_measured_log     = zeros(1, len);
        T4_measured_raw_log = zeros(1, len);
        T4_measured_log(1)     = x;
        T4_measured_raw_log(1) = x;

        % ---- Amplitude hold logic ----
        A_hold = 0;
        A_basal = 0;
        BAND = BAND_template_run;
        band_lock_time = NaN;    % will be set when band lock activates
        sample_times_amp  = ti:Ts:(tf + Ts);
        m_amp = 1;
        next_window_time  = sample_times_amp(1);

        % ---- T4 measurement hold logic ----
        T4_hold = 0;
        T4_sample_times    = ti:T_measure:(tf + T_measure);
        m_T4 = 1;
        next_T4_measure_time = T4_sample_times(1);

        % Precompute ED storage for this run
        ED = zeros(len, Nc);

        % ---- Held control output ----
        u_hold = 0;

        %% -------------------- Main simulation loop --------------------
        for i = 1 : len-1

            % ---- Normalise inputs for RBF ----
            input_scale = 150 * ones(16,1);
            norm_input  = new_in(:,i) ./ input_scale;

            for j = 1:Nc
                ED(i,j) = exp(-((norm_input - c(:,j))' * (norm_input - c(:,j))) / (2*beeta^2));
            end

            % ---- RBF control output (held constant between measurements) ----
            u(i) = u_hold;
            EF = u(i);

            % ---- Amplitude hold update at window boundary ----
            current_time = time_vector(i);
            if current_time >= next_window_time
                if EF < thresh
                    A_hold = 0;
                else
                    A_hold = min(max(kA*(EF - thresh), A_min), A_max);
                end

                % ---- Band lock check ----
                if BAND.enable && ~BAND.entered
                    if abs(x - T4_des) <= BAND.width
                        BAND.inband_ct = BAND.inband_ct + 1;
                        if ~BAND.reported && BAND.inband_ct >= 1
                            fprintf('  [%.1f h] First entered ±%.0f%% band (count %d/%d)\n', ...
                                current_time/60, BAND.pct*100, BAND.inband_ct, BAND.holdN);
                            BAND.reported = true;
                        end
                        if BAND.inband_ct >= BAND.holdN 
                            BAND.entered = true;
                            A_basal = A_hold;
                            band_lock_time = current_time;
                            fprintf('  [%.1f h] Band lock ACTIVATED after %d consecutive windows\n', ...
                                current_time/60, BAND.holdN);
                        end
                    else
                        BAND.inband_ct = 0;
                    end
                end

                m_amp = m_amp + 1;
                if m_amp <= length(sample_times_amp)
                    next_window_time = sample_times_amp(m_amp);
                else
                    next_window_time = inf;
                end
            end

            % ---- Enforce A_basal floor once band lock is active ----
            if BAND.entered
                A_hold = max(A_hold, A_basal);
            end

            A_now = A_hold;
            A_log(i) = A_now;

            % ---- Build EF(t) with burst pattern + actuator perturbation ----
            if A_now <= 0
                EF_time_min = @(t_min) 0.0;
            else
                A_applied  = clamp(A_now * (1 + pert_true.act_gain), A_min, A_max);
                delay_min  = max(0, pert_true.delay_min);
                jitter_min = pert_true.jitter_min;

                EF_time_min = @(t_min) ef_avg_burst_in_window_delay( ...
                    t_min, T_on, T_cycle, T_total, t5, t6, ...
                    A_applied, pulseDuty, delay_min, jitter_min);
            end
            EF_log(i) = EF_time_min(time_vector(i));

            % ---- ODE step with PERTURBED plant ----
            [~, y_temp] = ode15s( ...
                @(t,y) eng_time_varying_ode_EF_MINUTES_PERT( ...
                    t, y, EF_time_min, gammaext_T4_time_min, ...
                    M_time_min, alpha1_func_min, pert_true), ...
                [time_vector(i), time_vector(i+1)], y_log(:,i));
            y_new = y_temp(end,:)';
            y_log(:, i+1) = y_new;

            T4_actual = y_new(4);

           % ---- T4 measurement update ----
            current_time_next = time_vector(i+1);
            if current_time_next >= next_T4_measure_time

                if MEAS.enable
                    y_raw = T4_actual + MEAS.bias_true + MEAS.sigma * randn();
                else
                    y_raw = T4_actual;
                end

                [y_used, meas_state] = measurement_filter(y_raw, meas_state, MEAS);
                x = y_used;
                e = ref(i+1) - x;
                T4_hold = x;

                % === Adaptation: FREEZE once band lock is active ===
                if ~BAND.entered
                    % Integral accumulation (ONLY at measurement times)
                    e_int = e_int + e * T_measure;
                    e_int = clamp(e_int, -e_int_max, e_int_max);

                    % RBF weight update
                    m_sq = 1 + (e / max(ref(i+1), 1e-6))^2 + ED(i,:) * ED(i,:)';
                    W = W + learning_rate * e * ED(i,:) / m_sq;
                    W = clamp(W, -W_max, W_max);

                    % Centre adaptation
                    eta_c = 1e-5;
                    for j = 1:Nc
                        phi_j  = ED(i,j);
                        c(:,j) = c(:,j) + eta_c * e * W(j) * phi_j ...
                                 .* (norm_input - c(:,j)) / (beeta^2);
                    end
                end

                % Recompute held control output (still needed for A_hold calc,
                % even if weights are frozen - the frozen weights produce a
                % steady-state output that gets floored by A_basal anyway)
                u_hold = max(min(W * ED(i,:)' + ki*e_int, UL), LL);

                new_in(3:end, i+1) = new_in(2:end-1, i);
                new_in(2, i+1)     = x;
                T4_measured_raw_log(i+1) = y_raw;

                m_T4 = m_T4 + 1;
                if m_T4 <= length(T4_sample_times)
                    next_T4_measure_time = T4_sample_times(m_T4);
                else
                    next_T4_measure_time = inf;
                end
            else
                x = T4_hold;
                e = ref(i+1) - x;
                new_in(2:end, i+1) = new_in(2:end, i);
                T4_measured_raw_log(i+1) = T4_measured_raw_log(i);
            end

            T4_measured_log(i+1) = x;
            log_d(:, i+1) = [x; EF; e];
            Weights(i+1,:) = W;
        end

        % Tails
        u(end)   = u(end-1);
        A_log(end) = A_log(end-1);
        EF_log(end) = EF_log(end-1);
        T4_measured_log(end) = T4_measured_log(end-1);

        %% -------------------- Store results --------------------
        fieldname = sprintf('sp%g_sigma%.2f', T4_des, current_sigma);
        fieldname = strrep(fieldname, '.', 'p');
        fieldname = strrep(fieldname, '-', 'm');

        T4_true = y_log(4,:);

        ALL_RESULTS.(fieldname).T4_des      = T4_des;
        ALL_RESULTS.(fieldname).sigma       = current_sigma;
        ALL_RESULTS.(fieldname).time        = time_vector;
        ALL_RESULTS.(fieldname).T4_log      = T4_true;
        ALL_RESULTS.(fieldname).T4_meas_used = T4_measured_log;
        ALL_RESULTS.(fieldname).T4_meas_raw  = T4_measured_raw_log;
        ALL_RESULTS.(fieldname).u           = u;
        ALL_RESULTS.(fieldname).A_log       = A_log;
        ALL_RESULTS.(fieldname).EF_log      = EF_log;
        ALL_RESULTS.(fieldname).Weights_final = W;
        ALL_RESULTS.(fieldname).A_basal     = A_basal;
        ALL_RESULTS.(fieldname).band_lock_time = band_lock_time;
        ALL_RESULTS.(fieldname).RMSE        = sqrt(mean(log_d(3,:).^2));
        ALL_RESULTS.(fieldname).Weights     = Weights;                      % full Nc x len history
        ALL_RESULTS.(fieldname).centres_final = c;
        ALL_RESULTS.(fieldname).new_in = new_in;

        completed_runs = run_number;
        fprintf('Run %d/%d complete (sp=%g, sigma=%.2f). Saving...\n', ...
            run_number, total_runs, T4_des, current_sigma);
        save(results_file, 'ALL_RESULTS', 'completed_runs', ...
             'setpoint_values', 'sigma_values', 'ABLATION_MODE', ...
             'BAND_pct_val', 'MEAS_template', '-v7.3');
        fprintf('Results saved to: %s\n', results_file);
    end
end

fprintf('\n========================================\n');
fprintf('All %d runs completed!\n', total_runs);
fprintf('========================================\n');

%% ==================== COMPARISON PLOTS ====================
fprintf('\nGenerating comparison plots...\n');

tf_h = tf / 60;

% ---- Color map: one color per setpoint ----
sp_colors = lines(num_setpoints);

%% ---- Figure 1: All T4 trajectories overlaid, color-matched setpoint lines ----
figure('Color','w','Position',[100 100 1200 600]);
hold on;

for sp_i = 1:num_setpoints
    T4_des_i = setpoint_values(sp_i);
    col = sp_colors(sp_i,:);

    % Plot setpoint as dashed line — this goes in the legend
    plot([0 tf_h], [T4_des_i T4_des_i], '--', 'Color', col, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Setpoint = %g', T4_des_i));

    % Plot trajectories — hidden from legend
    for sigma_i = 1:num_sigma
        fn = sprintf('sp%g_sigma%.2f', T4_des_i, sigma_values(sigma_i));
        fn = strrep(fn, '.', 'p');
        fn = strrep(fn, '-', 'm');
        if ~isfield(ALL_RESULTS, fn), continue; end
        R = ALL_RESULTS.(fn);
        t_hr = R.time / 60;

        plot(t_hr, R.T4_log, '-', 'Color', col, 'LineWidth', 1.3, ...
            'HandleVisibility', 'off');
    end
end

xlabel('Time (h)', 'FontSize', 18); ylabel('T_4^{ext}  (a.u.)', 'FontSize', 18);
title('RBF: T_4 Trajectories Across Setpoints', 'FontSize', 20, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 18); grid on;
xlim([0 tf_h]);
set(gca, 'FontSize', 16);

if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_setpoint_sweep_trajectories_sp_%s_sigma_%s', sp_tag, sigma_tag)); end

%% ---- Figure 2: Zoomed settling region (last 15 h) ----
figure('Color','w','Position',[120 120 1200 600]);
hold on;

for sp_i = 1:num_setpoints
    T4_des_i = setpoint_values(sp_i);
    col = sp_colors(sp_i,:);

    plot([0 tf_h], [T4_des_i T4_des_i], '--', 'Color', col, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Setpoint = %g', T4_des_i));

    for sigma_i = 1:num_sigma
        fn = sprintf('sp%g_sigma%.2f', T4_des_i, sigma_values(sigma_i));
        fn = strrep(fn, '.', 'p');
        fn = strrep(fn, '-', 'm');
        if ~isfield(ALL_RESULTS, fn), continue; end
        R = ALL_RESULTS.(fn);

        plot(R.time/60, R.T4_log, '-', 'Color', col, 'LineWidth', 1.3, ...
            'HandleVisibility', 'off');
    end
end

xlabel('Time (h)', 'FontSize', 14); ylabel('T_4^{ext}  (a.u.)', 'FontSize', 14);
title('RBF: Zoomed Settling Region (Last 15 h)', 'FontSize', 16, 'FontWeight', 'bold');
legend('Location', 'eastoutside', 'FontSize', 16); grid on;
xlim([max(0, tf_h - 15), tf_h]);
set(gca, 'FontSize', 16);

if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_setpoint_sweep_settling_sp_%s_sigma_%s', sp_tag, sigma_tag)); end

%% ---- Figure 3: Per-setpoint absolute error subfigures ----
n_cols = min(num_setpoints, 3);
n_rows = ceil(num_setpoints / n_cols);
n_cols = 2;
n_rows = 2;
figure('Color','w','Position',[140 140 400*n_cols 350*n_rows]);
sgtitle('RBF: Tracking Error by Setpoint', 'FontSize', 20, 'FontWeight', 'bold');

for sp_i = 1:num_setpoints
    T4_des_i = setpoint_values(sp_i);
    col = sp_colors(sp_i,:);

    subplot(n_rows, n_cols, sp_i); hold on;
    yline(0, '--k', 'LineWidth', 0.8, 'HandleVisibility', 'off');

    for sigma_i = 1:num_sigma
        fn = sprintf('sp%g_sigma%.2f', T4_des_i, sigma_values(sigma_i));
        fn = strrep(fn, '.', 'p');
        fn = strrep(fn, '-', 'm');
        if ~isfield(ALL_RESULTS, fn), continue; end
        R = ALL_RESULTS.(fn);

        abs_err =  T4_des_i - R.T4_log;

        if num_sigma == 1
            lbl = sprintf('sp = %g', T4_des_i);
        else
            lbl = sprintf('\\sigma = %.1f', sigma_values(sigma_i));
        end
        plot(R.time/60, abs_err, '-', 'Color', col, 'LineWidth', 1.0, ...
            'DisplayName', lbl);
    end

    xlabel('Time (h)', 'FontSize', 18); ylabel('Error  (a.u.)', 'FontSize', 18);
    title(sprintf('Setpoint = %g', T4_des_i), 'FontSize', 20);
    grid on; xlim([0 tf_h]);
    set(gca, 'FontSize', 16);
    if num_sigma > 1, legend('Location','best','FontSize',18); end
end

if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_setpoint_sweep_abs_error_sp_%s_sigma_%s', sp_tag, sigma_tag)); end

%% ---- Figure 4: RBF output & Amplitude by setpoint ----
figure('Color','w','Position',[160 160 1400 500]);
sgtitle('RBF: Control Signals by Setpoint', 'FontSize', 20, 'FontWeight', 'bold');
subplot(1,2,1); hold on;
for sp_i = 1:num_setpoints
    T4_des_i = setpoint_values(sp_i);
    col = sp_colors(sp_i,:);
    for sigma_i = 1:num_sigma
        fn = sprintf('sp%g_sigma%.2f', T4_des_i, sigma_values(sigma_i));
        fn = strrep(fn, '.', 'p'); fn = strrep(fn, '-', 'm');
        if ~isfield(ALL_RESULTS, fn), continue; end
        R = ALL_RESULTS.(fn);
        plot(R.time/60, R.u, '-', 'Color', col, 'LineWidth', 1.0, ...
            'DisplayName', sprintf('Setpoint = %g', T4_des_i));
    end
end
xlabel('Time (h)', 'FontSize', 18); ylabel('u (RBF output)', 'FontSize', 18);
title('RBF Control Output', 'FontSize', 18); legend('Location','southeast','FontSize',18); grid on; xlim([0 tf_h]);
set(gca, 'FontSize', 16);

subplot(1,2,2); hold on;
for sp_i = 1:num_setpoints
    T4_des_i = setpoint_values(sp_i);
    col = sp_colors(sp_i,:);
    for sigma_i = 1:num_sigma
        fn = sprintf('sp%g_sigma%.2f', T4_des_i, sigma_values(sigma_i));
        fn = strrep(fn, '.', 'p'); fn = strrep(fn, '-', 'm');
        if ~isfield(ALL_RESULTS, fn), continue; end
        R = ALL_RESULTS.(fn);
        stairs(R.time/60, R.A_log, '-', 'Color', col, 'LineWidth', 1.0, ...
            'DisplayName', sprintf('Setpoint = %g', T4_des_i));
        if R.A_basal > 0
            yline(R.A_basal, '--', 'Color', col, 'LineWidth', 1.2, ...
                'Label', sprintf('A_{basal}=%.4f (sp=%g)', R.A_basal, T4_des_i), ...
                'FontSize', 12, 'LabelHorizontalAlignment', 'right', ...
                'HandleVisibility', 'off');
        end
    end
end
xlabel('Time (h)', 'FontSize', 18); ylabel('Amplitude A (a.u.)', 'FontSize', 18);
title('Control Amplitude', 'FontSize', 18); legend('Location','southeast','FontSize',18); grid on; xlim([0 tf_h]);
set(gca, 'FontSize', 16);

if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_setpoint_sweep_control_sp_%s_sigma_%s', sp_tag, sigma_tag)); end

%% ==================== PERFORMANCE METRICS ====================
fprintf('\n');
fprintf('============================================================================================\n');
fprintf('  RBF CONTROLLER — PERFORMANCE METRICS (ablation = %s, sigma = %s)\n', ...
    ABLATION_MODE, mat2str(sigma_values));
fprintf('============================================================================================\n');
fprintf('%-8s | %-8s | %10s | %10s | %10s | %10s | %10s | %10s\n', ...
    'T4_des', 'sigma', 'Rise(hr)', 'OS(%)', 'SS_ME', 'SS_RMSE', 'SS_NRMSE%', 'Term(hr)');
fprintf('--------------------------------------------------------------------------------------------\n');

metrics = struct();
metrics.setpoint_values = setpoint_values;
metrics.sigma_values    = sigma_values;
metrics.ablation        = ABLATION_MODE;
metrics.rise_time       = nan(num_setpoints, num_sigma);
metrics.overshoot       = nan(num_setpoints, num_sigma);
metrics.ss_mean_error   = nan(num_setpoints, num_sigma);
metrics.ss_rmse         = nan(num_setpoints, num_sigma);
metrics.ss_nrmse        = nan(num_setpoints, num_sigma);
metrics.term_time       = nan(num_setpoints, num_sigma);

for sp_i = 1:num_setpoints
    T4_des_i = setpoint_values(sp_i);
    for sigma_i = 1:num_sigma
        fn = sprintf('sp%g_sigma%.2f', T4_des_i, sigma_values(sigma_i));
        fn = strrep(fn, '.', 'p'); fn = strrep(fn, '-', 'm');
        if ~isfield(ALL_RESULTS, fn), continue; end
        R = ALL_RESULTS.(fn);

        t_hr    = R.time / 60;
        T4_true = R.T4_log;
        n_pts   = length(T4_true);

        metrics.term_time(sp_i, sigma_i) = max(t_hr);

        % Rise time (TRUE signal: 10% to 90%)
        idx_10 = find(T4_true >= 0.10 * T4_des_i, 1, 'first');
        idx_90 = find(T4_true >= 0.90 * T4_des_i, 1, 'first');
        if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
            metrics.rise_time(sp_i, sigma_i) = t_hr(idx_90) - t_hr(idx_10);
        end

        % Overshoot (TRUE signal)
        idx_first_sp = find(T4_true >= T4_des_i, 1, 'first');
        if ~isempty(idx_first_sp)
            T4_peak = max(T4_true(idx_first_sp:end));
            if T4_peak > T4_des_i
                metrics.overshoot(sp_i, sigma_i) = (T4_peak - T4_des_i) / T4_des_i * 100;
            else
                metrics.overshoot(sp_i, sigma_i) = 0;
            end
        end

        % Steady-state metrics (final 20%)
        ss_idx = round(0.80 * n_pts) : n_pts;
        ss_error = T4_true(ss_idx) - T4_des_i;
        metrics.ss_mean_error(sp_i, sigma_i) = mean(ss_error);
        metrics.ss_rmse(sp_i, sigma_i)       = sqrt(mean(ss_error.^2));
        metrics.ss_nrmse(sp_i, sigma_i)      = metrics.ss_rmse(sp_i, sigma_i) / T4_des_i * 100;

        fprintf('%-8g | %-8.2f | %10.2f | %10.2f | %10.4f | %10.4f | %10.2f | %10.1f\n', ...
            T4_des_i, sigma_values(sigma_i), ...
            metrics.rise_time(sp_i, sigma_i), metrics.overshoot(sp_i, sigma_i), ...
            metrics.ss_mean_error(sp_i, sigma_i), metrics.ss_rmse(sp_i, sigma_i), ...
            metrics.ss_nrmse(sp_i, sigma_i), metrics.term_time(sp_i, sigma_i));
    end
end

fprintf('============================================================================================\n');
fprintf('\nDefinitions:\n');
fprintf('  Rise Time     : Time from T4_true first crossing 10%% to 90%% of setpoint\n');
fprintf('  Overshoot     : (peak T4_true after first crossing setpoint - setpoint) / setpoint x 100\n');
fprintf('  SS Mean Error : mean(T4_true - T4_des) over final 20%% — positive = above setpoint\n');
fprintf('  SS RMSE       : sqrt(mean((T4_true - T4_des)^2)) over final 20%%\n');
fprintf('  SS NRMSE      : SS_RMSE / T4_des x 100 — normalized for cross-setpoint comparison\n');
fprintf('  Term. Time    : Total simulation time (no early termination)\n');

%% ---- Figure 5: Summary bar charts organized by setpoint ----
% For multi-sigma, bars are grouped; for single sigma, simple bars.
figure('Color','w','Position',[200 200 1800 500]);
sgtitle(sprintf('RBF: Summary Statistics by Setpoint'), ...
    'FontSize', 20, 'FontWeight', 'bold');

sp_labels = arrayfun(@(v) sprintf('%g', v), setpoint_values, 'UniformOutput', false);

if num_sigma == 1
    subplot(1,4,1);
    bar(categorical(sp_labels, sp_labels), metrics.rise_time(:,1), 'FaceColor', [0.3 0.6 0.9]);
    ylabel('Rise Time (h)', 'FontSize', 18); title('Rise Time (10%→90%)', 'FontSize', 18);
    xlabel('Setpoint  (a.u.)', 'FontSize', 18); grid on;
    set(gca, 'FontSize', 16);

    subplot(1,4,2);
    bar(categorical(sp_labels, sp_labels), metrics.ss_mean_error(:,1), 'FaceColor', [0.4 0.7 0.7]);
    ylabel('(a.u.)', 'FontSize', 18); title('SS Mean Error (final 20%)', 'FontSize', 18);
    xlabel('Setpoint  (a.u.)', 'FontSize', 18); grid on;
    set(gca, 'FontSize', 16);
    
    subplot(1,4,3);
    bar(categorical(sp_labels, sp_labels), metrics.ss_rmse(:,1), 'FaceColor', [0.7 0.5 0.8]);
    ylabel('(a.u.)', 'FontSize', 18); title('SS RMSE (final 20%)', 'FontSize', 18);
    xlabel('Setpoint  (a.u.)', 'FontSize', 18); grid on;
    set(gca, 'FontSize', 16);
    
    subplot(1,4,4);
    bar(categorical(sp_labels, sp_labels), metrics.ss_nrmse(:,1), 'FaceColor', [0.8 0.6 0.4]);
    ylabel('(%)', 'FontSize', 18); title('SS NRMSE (final 20%)', 'FontSize', 18);
    xlabel('Setpoint  (a.u.)', 'FontSize', 18); grid on;
    set(gca, 'FontSize', 16);
else
    subplot(1,4,1);
    bar(categorical(sp_labels, sp_labels), metrics.rise_time);
    ylabel('Rise Time (h)', 'FontSize', 18); title('Rise Time (10%→90%)', 'FontSize', 18);
    xlabel('Setpoint  (a.u.)', 'FontSize', 18); grid on; set(gca, 'FontSize', 16);
    legend(arrayfun(@(s) sprintf('\\sigma=%.1f',s), sigma_values, 'UniformOutput',false), 'Location','best', 'FontSize', 16);
    
    subplot(1,4,2);
    bar(categorical(sp_labels, sp_labels), metrics.ss_mean_error);
    ylabel('(a.u.)', 'FontSize', 18); title('SS Mean Error (final 20%)', 'FontSize', 18);
    xlabel('Setpoint  (a.u.)', 'FontSize', 18); grid on;
    set(gca, 'FontSize', 16);
    
    subplot(1,4,3);
    bar(categorical(sp_labels, sp_labels), metrics.ss_rmse);
    ylabel('(a.u.)', 'FontSize', 18); title('SS RMSE (final 20%)', 'FontSize', 18);
    xlabel('Setpoint  (a.u.)', 'FontSize', 18); grid on;
    set(gca, 'FontSize', 16);
    
    subplot(1,4,4);
    bar(categorical(sp_labels, sp_labels), metrics.ss_nrmse);
    ylabel('(%)', 'FontSize', 18); title('SS NRMSE (final 20%)', 'FontSize', 18);
    xlabel('Setpoint  (a.u.)', 'FontSize', 18); grid on;
    set(gca, 'FontSize', 16);
end

if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_setpoint_sweep_summary_bars_sp_%s_sigma_%s', sp_tag, sigma_tag)); end

%% ---- Figure 6: RBF Weight Evolution per setpoint ----
n_cols_w = min(num_setpoints, 3);
n_rows_w = ceil(num_setpoints / n_cols_w);
n_cols_w = 2;
n_rows_w = 2;
figure('Color','w','Position',[220 220 450*n_cols_w 350*n_rows_w]);
sgtitle('RBF: Weight Evolution by Setpoint', 'FontSize', 20, 'FontWeight', 'bold');

for sp_i = 1:num_setpoints
    T4_des_i = setpoint_values(sp_i);

    for sigma_i = 1:num_sigma
        fn = sprintf('sp%g_sigma%.2f', T4_des_i, sigma_values(sigma_i));
        fn = strrep(fn, '.', 'p');
        fn = strrep(fn, '-', 'm');
        if ~isfield(ALL_RESULTS, fn), continue; end
        R = ALL_RESULTS.(fn);

        subplot(n_rows_w, n_cols_w, sp_i); hold on;

        t_hr = R.time / 60;
        W_hist = R.Weights;          % len x Nc

        plot(t_hr, W_hist, '-', 'LineWidth', 0.6);

        % Vertical dashed line at band-lock activation
        if isfield(R, 'band_lock_time') && ~isnan(R.band_lock_time)
            xline(R.band_lock_time / 60, '--k', 'LineWidth', 1.4, ...
                'Label', sprintf('Band lock @ %.1f h', R.band_lock_time/60), ...
                'FontSize', 12, 'LabelVerticalAlignment', 'bottom');
        end

        xlabel('Time (h)', 'FontSize', 18);
        ylabel('Weight value', 'FontSize', 18);
        title(sprintf('Setpoint = %g', T4_des_i), 'FontSize', 20);
        grid on; xlim([0 tf_h]);
        set(gca, 'FontSize', 16);
    end
end

if DO_SAVE_PDF
    savePlot(gcf, output_dir, sprintf('RBF_setpoint_sweep_trajectories_sp_%s_sigma_%s', sp_tag, sigma_tag));
end




%% ---- Save metrics ----
metrics_file = fullfile(output_dir, sprintf('RBF_setpoint_sweep_metrics_%s_sp_%s_sigma_%s.mat', ABLATION_MODE, sp_tag, sigma_tag));
save(metrics_file, 'metrics', 'setpoint_values', 'sigma_values', 'ABLATION_MODE');
fprintf('\nMetrics saved to: %s\n', metrics_file);

metrics_txt = fullfile(output_dir, sprintf('RBF_setpoint_sweep_metrics_%s_sp_%s_sigma_%s.txt', ABLATION_MODE, sp_tag, sigma_tag));
fid = fopen(metrics_txt, 'w');
fprintf(fid, 'Performance Metrics: RBF Controller (setpoint sweep)\n');
fprintf(fid, 'Ablation mode: %s\n', ABLATION_MODE);
fprintf(fid, 'Sigma values: %s\n', mat2str(sigma_values));
fprintf(fid, 'Setpoint values: %s\n', mat2str(setpoint_values));
fprintf(fid, 'Band width: +/-%d%%\n', round(BAND_pct_val*100));
fprintf(fid, 'Measurement bias (true): 5%% of setpoint (no estimator — RBF absorbs implicitly)\n');
fprintf(fid, 'Perturbation: same seed (rng(1)) and ranges as PID code\n\n');
fprintf(fid, '%-8s | %-8s | %10s | %10s | %10s | %10s | %10s | %10s\n', ...
    'T4_des', 'sigma', 'Rise(hr)', 'OS(%)', 'SS_ME', 'SS_RMSE', 'SS_NRMSE%', 'Term(hr)');
fprintf(fid, '--------------------------------------------------------------------------------------------\n');
for sp_i = 1:num_setpoints
    for sigma_i = 1:num_sigma
        fprintf(fid, '%-8g | %-8.2f | %10.2f | %10.2f | %10.4f | %10.4f | %10.2f | %10.1f\n', ...
            setpoint_values(sp_i), sigma_values(sigma_i), ...
            metrics.rise_time(sp_i, sigma_i), metrics.overshoot(sp_i, sigma_i), ...
            metrics.ss_mean_error(sp_i, sigma_i), metrics.ss_rmse(sp_i, sigma_i), ...
            metrics.ss_nrmse(sp_i, sigma_i), metrics.term_time(sp_i, sigma_i));
    end
end
fprintf(fid, '--------------------------------------------------------------------------------------------\n');
fclose(fid);
fprintf('Text metrics saved to: %s\n', metrics_txt);
fprintf('\nAll plots generated.\n');
fprintf('Results file: %s\n', results_file);

%% ========================================================================
%                              FUNCTIONS
%% ========================================================================

%% ---- Perturbed plant ODE (minutes) — IDENTICAL to PID version -----------
function dydt_min = eng_time_varying_ode_EF_MINUTES_PERT( ...
         t_min, y, EF_time_min, gammaext_T4_time_min, M_time_min, alpha1_func_min, pert)

    EF_base      = EF_time_min(t_min);
    gammaext_T4b = gammaext_T4_time_min(t_min);
    M_fun_b      = M_time_min(t_min);
    alpha1_b     = alpha1_func_min(t_min);

    gammaext_T4 = gammaext_T4b * (1 + pert.gext_scale);
    M_fun       = M_fun_b * (1 + pert.M_scale);
    alpha1      = alpha1_b * (1 + pert.alpha1_scale);

    x      = y(1:4);
    u_f    = y(5);
    u_hist = y(6:15);
    nkx21  = y(16);

    alpha0  = 20*8.43812;
    K0      = 10*0.1195;
    gamma10 = 0.0296;
    n = 2;

    alpha = alpha0 * (1 + pert.alpha_scale);
    K     = K0     * (1 + pert.K_scale);
    gamma1= gamma10* (1 + pert.gamma1_scale);

    N = 10;  a=0.00002*N;
    U = zeros(N,1);  U(1) = u_f;

    EFv = EF_base;
    du_f    = alpha1 * EFv^n / (K^n + EFv^n) - gamma1*u_f;
    du_hist = (-a*eye(N) + a*diag(ones(1,N-1),-1)) * u_hist + a*U;
    dNKX21  = alpha*u_hist(end) - 0.01*nkx21 + 0.25;

    kappa0 = 1;  K20 = 1000;
    kappa = kappa0 * (1 + pert.kappa_scale);
    K2    = K20    * (1 + pert.K2_scale);
    k_1   = kappa * ((nkx21/K2)^3 / (1 + (nkx21/K2)^3));

    alpha_TG = 1;  alpha_I = 1;
    gamma_I0     = 0.004;
    gamma_Tg0    = 0.04;
    gammaint_T40 = 0.15 + 0.0004;

    gamma_I     = gamma_I0     * (1 + pert.gammaI_scale);
    gamma_Tg    = gamma_Tg0    * (1 + pert.gammaTg_scale);
    gammaint_T4 = gammaint_T40 * (1 + pert.gint_scale);

    dx1 = -k_1*x(3)*x(1) - gamma_I*x(1) + alpha_I;
    dx2 =  k_1*x(3)*x(1) - M_fun*x(2) - gammaint_T4*x(2);
    dx3 =  alpha_TG - k_1*x(3)*x(1) - gamma_Tg*x(3);

    d4 = pert.d4_bias + pert.d4_amp * sin(2*pi*(t_min/pert.d4_period_min + pert.d4_phase));
    dx4 = M_fun*x(2) - gammaext_T4*x(4) - d4;

    dydt_sec = [dx1; dx2; dx3; dx4; du_f; du_hist; dNKX21];
    dydt_min = 60 * dydt_sec;
end

%% ---- EF burst helper with delay + jitter (same as PID) -----------------
function ef = ef_avg_burst_in_window_delay(t, T_on, T_cycle, T_total, t5, t6, A, pulseDuty, delay_min, jitter_min)
    t_eff = t - delay_min + jitter_min;
    if t_eff < 0,       ef = 0; return; end
    if t_eff > T_total,  ef = 0; return; end

    tau_win = mod(t_eff, T_cycle);
    if tau_win >= T_on,  ef = 0; return; end

    Tb = t5 + t6;
    xi = mod(tau_win, Tb);
    in_burst = (xi < t5);
    A_eff = A * pulseDuty;
    ef = A_eff * double(in_burst);
end

%% ---- Measurement LPF only (no bias estimator — RBF absorbs bias) --------
function [y_used, st] = measurement_filter(y_raw, st, MEAS)
    if isnan(st.y_filt), st.y_filt = y_raw; end

    a = MEAS.alpha_y;
    st.y_filt = (1-a)*st.y_filt + a*y_raw;

    y_used = st.y_filt;   % no bhat subtraction
end

%% ---- Perturbation sampling (same as PID) --------------------------------
function pert = sample_perturbation(R, d4_period_min)
    u = @(a) a*(2*rand-1);

    pert.alpha_scale   = u(R.alpha_scale);
    pert.K_scale       = u(R.K_scale);
    pert.gamma1_scale  = u(R.gamma1_scale);
    pert.kappa_scale   = u(R.kappa_scale);
    pert.K2_scale      = u(R.K2_scale);
    pert.gammaI_scale  = u(R.gammaI_scale);
    pert.gammaTg_scale = u(R.gammaTg_scale);
    pert.gint_scale    = u(R.gint_scale);
    pert.gext_scale    = u(R.gext_scale);
    pert.M_scale       = u(R.M_scale);
    pert.alpha1_scale  = u(R.alpha1_scale);

    pert.d4_bias       = u(R.d4_bias);
    pert.d4_amp        = abs(u(R.d4_amp));
    pert.d4_phase      = rand();
    pert.d4_period_min = d4_period_min;

    pert.act_gain      = u(R.act_gain);
    pert.delay_min     = max(0, u(R.delay_min));
    pert.jitter_min    = u(R.jitter_min);

    pert.meas_sigma = 0;
    pert.meas_bias  = 0;
    pert.noise_seed = 0;
end

%% ---- Neutral perturbation (nominal plant, all zeros) --------------------
function pert = neutral_perturbation(d4_period_min)
    pert.alpha_scale   = 0;  pert.K_scale      = 0;  pert.gamma1_scale = 0;
    pert.kappa_scale   = 0;  pert.K2_scale     = 0;  pert.gammaI_scale = 0;
    pert.gammaTg_scale = 0;  pert.gint_scale   = 0;  pert.gext_scale   = 0;
    pert.M_scale       = 0;  pert.alpha1_scale = 0;
    pert.d4_bias = 0;  pert.d4_amp = 0;  pert.d4_phase = 0;
    pert.d4_period_min = d4_period_min;
    pert.act_gain = 0;  pert.delay_min = 0;  pert.jitter_min = 0;
    pert.meas_sigma = 0;  pert.meas_bias = 0;  pert.noise_seed = 0;
end



%% ---- Utilities ---------------------------------------------------------
function v = clamp(x, xmin, xmax)
    v = min(max(x, xmin), xmax);
end

function savePlot(figH, folder, name)
    if ~exist(folder,'dir'), mkdir(folder); end
    ts = char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
    base = sprintf('%s_%s', ts, name);
    pdfPath = fullfile(folder, [base '.pdf']);
    try
        if exist('exportgraphics','file') == 2
            exportgraphics(figH, pdfPath, 'ContentType','vector', 'Resolution',300);
        else
            set(figH, 'PaperPositionMode','auto');
            print(figH, pdfPath, '-dpdf', '-bestfit');
        end
        fprintf('Saved: %s\n', pdfPath);
    catch ME
        warning('savePlot failed (%s): %s', pdfPath, ME.message);
    end
end

