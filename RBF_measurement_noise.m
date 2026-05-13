clear all; close all; clc;

%  This code generates figure 5 of the accompanying paper. 

%% ========================================================================
%  RBF Controller with Measurement Noise/Bias - Sigma Sweep Implementation
%  Plant Perturbations, Actuator Jitter, etc disabled via ablation mode 'none'
%  measurement noise is still enabled via 
%      MEAS_template.enable    = true;
%
%  Comparable noise model to the Adaptive PID (GD) code:
%    - Measurement LPF, no bias estimator however (impacted performance, let weights do their best to learn it) at each T_measure sample
%    - Plant parameter perturbations (same struct as PID)
%    - Actuator gain mismatch, delay, and jitter in EF delivery
%    - Sinusoidal disturbance on T4_ext
%
%  NO band termination — runs to tf for every sigma - but band implements A_basal still
%  A_basal floor: once T4 stays in ±5% band for holdN consecutive amplitude
%  windows, amplitude is floored at what strength was when band was entered, to prevent collapse.
%
%  INTEGRAL ACTION: ki accumulates ONLY at measurement times (synchronized
%  with weight updates) to prevent ramp-within-hold artifacts, but is
%  currently set to 0 (off).

%  Logs: rise time, overshoot, SS Mean Error, SS RMSE, SS NRMSE, termination time.
%% ========================================================================

% ---- Save settings ----
DO_SAVE_PDF = false;
output_dir = fullfile(pwd, 'results_rbf_meas_noise');
if ~exist(output_dir,'dir'), mkdir(output_dir); end

fprintf('Current Folder (pwd): %s\n', pwd);
fprintf('Output directory:      %s\n', output_dir);

%% -------------------- Setpoint --------------------
T4_des = 22;   % RBF setpoint (ref)
sp_tag = sprintf('sp%g', T4_des);


%% ==================== SIGMA SWEEP CONFIGURATION ====================
sigma_values = linspace(0.02, 0.20, 5) * T4_des;
num_sigma    = numel(sigma_values);

%% ==================== ABLATION MODE ======================================
%  Controls which noise sources, impacted by sigma, are active:
%    'full'  — all perturbations + measurement noise (default, matches PID)
%    'none'  — nominal plant, measurement noise only (isolates sensor noise)
%    'half'  — perturbation ranges halved, measurement noise on
% for other noise sources, check out 
ABLATION_MODE = 'none';   % <-- change to 'none' or 'half' for ablation runs


% Results storage filename
results_file = fullfile(output_dir, sprintf('RBF_sigma_sweep_results_%s_%s.mat', sp_tag, ABLATION_MODE));

% Check for partial results to resume from
if exist(results_file, 'file')
    fprintf('Found existing results file. Loading...\n');
    load(results_file, 'ALL_RESULTS', 'completed_runs');
    fprintf('Resuming from run %d of %d\n', completed_runs + 1, num_sigma);
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
tf   = 6000;            % minutes (~33.3 h)
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

%% ==================== ±5% band lock + basal amplitude floor ==============
BAND_template.enable    = true;
BAND_template.pct       = 0.20;                          % ±20%
BAND_template.width     = BAND_template.pct * T4_des;    % absolute band
BAND_template.holdN     = 2;                    % consecutive windows in band before lock
BAND_template.entered   = false;                % latched once entered
BAND_template.inband_ct = 0;                    % consecutive windows in band
BAND_template.reported  = false;                % milestone printed once
A_basal = 0;                                    % set adaptively when band lock activates


%% -------------------- RBF parameters --------------------
Nc = 40;                              % number of centres
c1d = linspace(0, 1, Nc);
c_template = repmat(c1d, [16, 1]);    % 16-dim centres on [0,1]

W_max = (T4_des/1000 + 0.005);       % linearly interpolated cap
beeta = 0.13;
learning_rate_base = 1e-6 * (T_measure / dt);

% Integral action (synchronized with measurement updates)
ki = 1e-4;
e_int_max = 5;                        % hard cap on integrator state

% Saturation limits on RBF output
UL = 100;  LL = 0;

%% ==================== Measurement model (template) ====================
MEAS_template.enable    = true;
MEAS_template.bias_true = 0.05 * T4_des;  % 5% systematic sensor bias
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
    otherwise  % 'full'
        pert_true = sample_perturbation(PERT_TRUE.range, PERT_TRUE.d4_period_min);
        fprintf('*** ABLATION: full perturbations ***\n');
end

%% ==================== Band definition (for METRICS only) ================
BAND_pct   = BAND_template.pct;
BAND_width = BAND_template.width;
GOAL_pct   = 0.30;               % ±30% target band for plots

%% ==================== MAIN LOOP OVER SIGMA VALUES ====================
for sigma_idx = (completed_runs + 1) : num_sigma

    current_sigma = sigma_values(sigma_idx);
    fprintf('\n========================================\n');
    fprintf('Starting run %d/%d: sigma = %.2f\n', sigma_idx, num_sigma, current_sigma);
    fprintf('========================================\n');

    % ---- Set up MEAS for this run ----
    MEAS       = MEAS_template;
    MEAS.sigma = current_sigma;

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
    T4_measured_log     = zeros(1, len);   % held (bias-corrected) measurement
    T4_measured_raw_log = zeros(1, len);   % raw noisy measurement
    T4_measured_log(1)     = x;
    T4_measured_raw_log(1) = x;

    % ---- Amplitude hold logic ----
    A_hold = 0;
    BAND = BAND_template;   % reset band state each run
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

    % ---- Held control output (recomputed only at measurement times) ----
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

            % ---- Band lock check (at each amplitude window) ----
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
                        A_basal = A_hold;   % lock current amplitude as floor
                        fprintf('  [%.1f h] Band lock ACTIVATED after %d consecutive windows\n', ...
                            current_time/60, BAND.holdN);
                    end
                else
                    BAND.inband_ct = 0;  % reset — must be consecutive
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
            % Apply actuator gain mismatch, delay, jitter from pert_true
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
            @(t,y) eng_time_varying_ode_EF_MINUTES_PERT_refined( ...
                t, y, EF_time_min, gammaext_T4_time_min, ...
                M_time_min, alpha1_func_min, pert_true), ...
            [time_vector(i), time_vector(i+1)], y_log(:,i));
        y_new = y_temp(end,:)';
        y_log(:, i+1) = y_new;

        T4_actual = y_new(4);

        % ---- T4 measurement update ----
        current_time_next = time_vector(i+1);
        if current_time_next >= next_T4_measure_time

            % === Apply measurement noise + bias ===
            if MEAS.enable
                y_raw = T4_actual + MEAS.bias_true + MEAS.sigma * randn();
            else
                y_raw = T4_actual;
            end

            % === Measurement LPF (no bias estimator) ===
            [y_used, meas_state] = measurement_filter( ...
                y_raw, meas_state, MEAS);

            % System output uses bias-corrected measurement
            x = y_used;
            e = ref(i+1) - x;

            T4_hold = x;  % hold for inter-measurement steps

            % === Integral accumulation (ONLY at measurement times) ===
            e_int = e_int + e * T_measure;
            e_int = clamp(e_int, -e_int_max, e_int_max);

            % === Recompute held control output ===
            u_hold = max(min(W * ED(i,:)' + ki*e_int, UL), LL);

            % === RBF weight update (only at measurement times) ===
            m_sq = 1 + (e / max(ref(i+1), 1e-6))^2 + ED(i,:) * ED(i,:)';
            W = W + learning_rate * e * ED(i,:) / m_sq;
            W = clamp(W, -W_max, W_max);

            % === Centre adaptation ===
            eta_c = 1e-5;
            for j = 1:Nc
                phi_j  = ED(i,j);
                c(:,j) = c(:,j) + eta_c * e * W(j) * phi_j ...
                         .* (norm_input - c(:,j)) / (beeta^2);
            end

            % === Update input history at measurement times ===
            new_in(3:end, i+1) = new_in(2:end-1, i);
            new_in(2, i+1)     = x;

            % Log raw measurement
            T4_measured_raw_log(i+1) = y_raw;

            m_T4 = m_T4 + 1;
            if m_T4 <= length(T4_sample_times)
                next_T4_measure_time = T4_sample_times(m_T4);
            else
                next_T4_measure_time = inf;
            end
        else
            % No new measurement - hold everything, NO integral update here
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
    fieldname = strrep(sprintf('sigma_%.2f', current_sigma), '.', 'p');

    T4_true = y_log(4,:);

    ALL_RESULTS.(fieldname).sigma       = current_sigma;
    ALL_RESULTS.(fieldname).time        = time_vector;       % minutes
    ALL_RESULTS.(fieldname).T4_log      = T4_true;           % true T4
    ALL_RESULTS.(fieldname).T4_meas_used = T4_measured_log;  % bias-corrected
    ALL_RESULTS.(fieldname).T4_meas_raw  = T4_measured_raw_log;
    ALL_RESULTS.(fieldname).u           = u;
    ALL_RESULTS.(fieldname).A_log       = A_log;
    ALL_RESULTS.(fieldname).EF_log      = EF_log;
    ALL_RESULTS.(fieldname).Weights_final = W;
    ALL_RESULTS.(fieldname).A_basal     = A_basal;
    ALL_RESULTS.(fieldname).RMSE        = sqrt(mean(log_d(3,:).^2));

    completed_runs = sigma_idx;
    fprintf('Run %d/%d complete (sigma=%.2f). Saving...\n', sigma_idx, num_sigma, current_sigma);
    save(results_file, 'ALL_RESULTS', 'completed_runs', 'sigma_values', ...
         'T4_des', 'BAND_pct', 'BAND_width', 'MEAS_template', '-v7.3');
    fprintf('Results saved to: %s\n', results_file);
end

fprintf('\n========================================\n');
fprintf('All %d sigma runs completed!\n', num_sigma);
fprintf('========================================\n');

%% ==================== COMPARISON PLOTS ====================
fprintf('\nGenerating comparison plots...\n');
 
% Recover A_basal from last completed run (needed on reruns when sim loop is skipped)
fnames_res = fieldnames(ALL_RESULTS);
if ~isempty(fnames_res)
    last_run = ALL_RESULTS.(fnames_res{end});
    if isfield(last_run, 'A_basal')
        A_basal = last_run.A_basal;
    end
end
 
colors = lines(num_sigma);
tf_h   = tf / 60;
 
%% ---- Figure 1: Trajectories WITH measurements ----
figure('Color','w','Position',[100 100 1400 900]);
sgtitle(sprintf('RBF: T_4 Trajectories (with measurements), Setpoint = %g (a.u.)', T4_des), ...
    'FontSize', 20, 'FontWeight', 'bold');
 
% Subplot 1: All overlaid
subplot(2,2,1); hold on;
plot([0 tf_h], [T4_des T4_des], '--k', 'LineWidth', 1.5, 'DisplayName', 'Setpoint');
yline(T4_des*(1+BAND_pct), ':k', 'LineWidth', 0.8, 'HandleVisibility', 'off');
yline(T4_des*(1-BAND_pct), ':k', 'LineWidth', 0.8, 'HandleVisibility', 'off');
 
for i = 1:num_sigma
    fn = strrep(sprintf('sigma_%.2f', sigma_values(i)), '.', 'p');
    if ~isfield(ALL_RESULTS, fn), continue; end
    R = ALL_RESULTS.(fn);
    t_hr = R.time / 60;
    plot(t_hr, R.T4_log, '-', 'Color', colors(i,:), 'LineWidth', 1.2, ...
        'DisplayName', sprintf('\\sigma=%.2f', sigma_values(i)));
end
%plot(NaN, NaN, 'ok', 'MarkerSize', 4, 'DisplayName', 'Meas.
%(bias-corrected)'); unused here, no bias correction
xlabel('Time (h)', 'FontSize', 18); ylabel('T_4^{ext} (a.u.)', 'FontSize', 18);
title('All Trajectories', 'FontSize', 20); legend('Location','southeast','FontSize', 16); grid on;
set(gca, 'FontSize', 18);
xlim([0 tf_h]);
 
% Subplots 2-4: individual sigma runs
sigma_to_show = [1, 3, 5];
for j = 1:3
    subplot(2,2,j+1); hold on;
    idx = sigma_to_show(j);
    if idx > num_sigma, continue; end
    fn = strrep(sprintf('sigma_%.2f', sigma_values(idx)), '.', 'p');
    if ~isfield(ALL_RESULTS, fn), continue; end
    R = ALL_RESULTS.(fn);
    t_hr = R.time / 60;
 
    plot([0 tf_h], [T4_des T4_des], '--k', 'LineWidth', 1.5, 'DisplayName', 'Setpoint');
    yline(T4_des*(1+BAND_pct), ':k', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    yline(T4_des*(1-BAND_pct), ':k', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    plot(t_hr, R.T4_log, '-', 'Color', colors(idx,:), 'LineWidth', 1.5, 'DisplayName', 'True T_4');
    stairs(t_hr, R.T4_meas_used, '-', 'Color', [0.85 0.5 0], 'LineWidth', 1.0, 'DisplayName', 'Used meas.');
    plot(t_hr, R.T4_meas_raw, '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 4, 'DisplayName', 'Raw meas.');
 
    xlabel('Time (h)', 'FontSize', 18); ylabel('T_4^{ext} (a.u.)', 'FontSize', 18);
    title(sprintf('\\sigma = %.2f', sigma_values(idx)), 'FontSize', 20);
    legend('Location','southeast', 'FontSize', 16); grid on; xlim([0 tf_h]);
    set(gca, 'FontSize', 18);
end
 
if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_sigma_sweep_with_meas_%s', sp_tag)); end
 
%% ---- Figure 2: True state only ----
figure('Color','w','Position',[150 150 1400 900]);
sgtitle(sprintf('RBF: T_4 Trajectories (true state only), Setpoint = %g (a.u.)', T4_des), ...
    'FontSize', 20, 'FontWeight', 'bold');
 
% All overlaid
subplot(2,2,1); hold on;
plot([0 tf_h], [T4_des T4_des], '--k', 'LineWidth', 1.5, 'DisplayName', 'Setpoint');
yline(T4_des*(1+BAND_pct), ':k', 'LineWidth', 0.8, 'HandleVisibility', 'off');
yline(T4_des*(1-BAND_pct), ':k', 'LineWidth', 0.8, 'HandleVisibility', 'off');
for i = 1:num_sigma
    fn = strrep(sprintf('sigma_%.2f', sigma_values(i)), '.', 'p');
    if ~isfield(ALL_RESULTS, fn), continue; end
    R = ALL_RESULTS.(fn);
    plot(R.time/60, R.T4_log, '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('\\sigma=%.2f', sigma_values(i)));
end
xlabel('Time (h)', 'FontSize', 18); ylabel('T_4^{ext} (a.u.)', 'FontSize', 18);
title('All Trajectories (True State Only)', 'FontSize', 20);
legend('Location','southeast', 'FontSize', 16); grid on;
set(gca, 'FontSize', 18);
xlim([0 tf_h]);
 
% Zoomed settling region
subplot(2,2,2); hold on;
plot([0 tf_h], [T4_des T4_des], '--k', 'LineWidth', 1.5, 'DisplayName', 'Setpoint');
yline(T4_des*(1+BAND_pct), ':k', 'LineWidth', 0.8, 'HandleVisibility', 'off');
yline(T4_des*(1-BAND_pct), ':k', 'LineWidth', 0.8, 'HandleVisibility', 'off');
for i = 1:num_sigma
    fn = strrep(sprintf('sigma_%.2f', sigma_values(i)), '.', 'p');
    if ~isfield(ALL_RESULTS, fn), continue; end
    R = ALL_RESULTS.(fn);
    plot(R.time/60, R.T4_log, '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('\\sigma=%.2f', sigma_values(i)));
end
xlabel('Time (h)', 'FontSize', 18); ylabel('T_4^{ext} (a.u.)', 'FontSize', 18);
title('Zoomed: Settling Region', 'FontSize', 20); legend('Location','southwest', 'FontSize', 16); grid on;
set(gca, 'FontSize', 18);
xlim([max(0, tf_h - 15), tf_h]);
ylim([T4_des - 5, T4_des + 5]);
 
% RBF output comparison
subplot(2,2,3); hold on;
for i = 1:num_sigma
    fn = strrep(sprintf('sigma_%.2f', sigma_values(i)), '.', 'p');
    if ~isfield(ALL_RESULTS, fn), continue; end
    R = ALL_RESULTS.(fn);
    plot(R.time/60, R.u, '-', 'Color', colors(i,:), 'LineWidth', 1.0, ...
        'DisplayName', sprintf('\\sigma=%.2f', sigma_values(i)));
end
xlabel('Time (h)', 'FontSize', 18); ylabel('u (RBF output)', 'FontSize', 18);
title('RBF Control Output', 'FontSize', 20); legend('Location','northeast', 'FontSize', 16); grid on;
set(gca, 'FontSize', 18);
xlim([0 tf_h]);
 
% Amplitude comparison
subplot(2,2,4); hold on;
for i = 1:num_sigma
    fn = strrep(sprintf('sigma_%.2f', sigma_values(i)), '.', 'p');
    if ~isfield(ALL_RESULTS, fn), continue; end
    R = ALL_RESULTS.(fn);
    stairs(R.time/60, R.A_log, '-', 'Color', colors(i,:), 'LineWidth', 1.0, ...
        'DisplayName', sprintf('\\sigma=%.2f', sigma_values(i)));
end
xlabel('Time (h)', 'FontSize', 18); ylabel('Amplitude A', 'FontSize', 18);
title('Control Amplitude', 'FontSize', 20); legend('Location','northeast', 'FontSize', 16); grid on;
set(gca, 'FontSize', 18);
xlim([0 tf_h]);
 
if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_sigma_sweep_true_state_%s', sp_tag)); end
 
%% ---- Figure 3: Tracking error comparison ----
figure('Color','w','Position',[200 200 1000 500]);
sgtitle(sprintf('RBF: Tracking Error, Setpoint = %g (a.u.)', T4_des), ...
    'FontSize', 20, 'FontWeight', 'bold');
hold on;
yline(0, '--k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
for i = 1:num_sigma
    fn = strrep(sprintf('sigma_%.2f', sigma_values(i)), '.', 'p');
    if ~isfield(ALL_RESULTS, fn), continue; end
    R = ALL_RESULTS.(fn);
    plot(R.time/60, R.T4_log - T4_des, '-', 'Color', colors(i,:), 'LineWidth', 1.2, ...
        'DisplayName', sprintf('\\sigma=%.2f', sigma_values(i)));
end
xlabel('Time (h)', 'FontSize', 18); ylabel('T_4^{ext} - T_{4,des} (a.u.)', 'FontSize', 18);
title('True Tracking Error Over Time', 'FontSize', 20); legend('Location','southeast', 'FontSize', 16); grid on;
set(gca, 'FontSize', 18);
xlim([0 tf_h]);
 
if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_sigma_sweep_error_%s', sp_tag)); end
 
%% ==================== PERFORMANCE METRICS ====================
fprintf('\n');
fprintf('============================================================================================\n');
fprintf('                       RBF CONTROLLER — PERFORMANCE METRICS TABLE                          \n');
fprintf('============================================================================================\n');
fprintf('%-8s | %12s | %12s | %12s | %12s | %12s | %12s\n', ...
    'sigma', 'Rise Time', 'Overshoot', 'SS MeanErr', 'SS RMSE', 'SS NRMSE', 'Term. Time');
fprintf('%-8s | %12s | %12s | %12s | %12s | %12s | %12s\n', ...
    '', '(hr)', '(%)', '(ug/dL)', '(ug/dL)', '(%)', '(hr)');
fprintf('--------------------------------------------------------------------------------------------\n');
 
metrics = struct();
metrics.sigma         = sigma_values;
metrics.rise_time     = nan(1, num_sigma);
metrics.overshoot     = nan(1, num_sigma);
metrics.ss_mean_error = nan(1, num_sigma);
metrics.ss_rmse       = nan(1, num_sigma);
metrics.ss_nrmse      = nan(1, num_sigma);
metrics.term_time     = nan(1, num_sigma);
 
for i = 1:num_sigma
    fn = strrep(sprintf('sigma_%.2f', sigma_values(i)), '.', 'p');
    if ~isfield(ALL_RESULTS, fn), continue; end
    R = ALL_RESULTS.(fn);
 
    t_hr    = R.time / 60;
    T4_true = R.T4_log;
    T4_meas = R.T4_meas_used;
    n_pts   = length(T4_true);
 
    % ---- Termination time ----
    metrics.term_time(i) = max(t_hr);
 
    % ---- Rise time (on measured signal: 10% to 90% of setpoint) ----
    %idx_10 = find(T4_meas >= 0.10 * T4_des, 1, 'first');
    %idx_90 = find(T4_meas >= 0.90 * T4_des, 1, 'first');
    %if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
    %    metrics.rise_time(i) = t_hr(idx_90) - t_hr(idx_10);
    %end
 
    % ---- Rise time (on TRUE signal — dense time series) ----
    idx_10 = find(T4_true >= 0.10 * T4_des, 1, 'first');
    idx_90 = find(T4_true >= 0.90 * T4_des, 1, 'first');
    if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
        metrics.rise_time(i) = t_hr(idx_90) - t_hr(idx_10);
    end
 
    % ---- Overshoot (on TRUE signal, after first crossing setpoint) ----
    idx_first_sp = find(T4_true >= T4_des, 1, 'first');
    if ~isempty(idx_first_sp)
        T4_peak = max(T4_true(idx_first_sp:end));
        if T4_peak > T4_des
            metrics.overshoot(i) = (T4_peak - T4_des) / T4_des * 100;
        else
            metrics.overshoot(i) = 0;
        end
    end
 
    % ---- Steady-state metrics (on TRUE signal, final 20%) ----
    ss_idx = round(0.80 * n_pts) : n_pts;
    ss_error = T4_true(ss_idx) - T4_des;
    metrics.ss_mean_error(i) = mean(ss_error);                  % DC offset (sign preserved)
    metrics.ss_rmse(i)       = sqrt(mean(ss_error.^2));          % total error magnitude
    metrics.ss_nrmse(i)      = metrics.ss_rmse(i) / T4_des * 100;  % normalized for cross-setpoint comparison
 
    fprintf('%-8.2f | %12.2f | %12.2f | %12.4f | %12.4f | %12.2f | %12.1f\n', ...
        sigma_values(i), metrics.rise_time(i), metrics.overshoot(i), ...
        metrics.ss_mean_error(i), metrics.ss_rmse(i), metrics.ss_nrmse(i), metrics.term_time(i));
end
 
fprintf('============================================================================================\n');
fprintf('\nDefinitions:\n');
fprintf('  Rise Time     : Time from T4_meas first crossing 10%% of setpoint (%.1f) to\n', 0.10*T4_des);
fprintf('                  first crossing 90%% of setpoint (%.1f)\n', 0.90*T4_des);
fprintf('  Overshoot     : (peak T4_true after first crossing setpoint - setpoint) / setpoint x 100\n');
fprintf('  SS Mean Error : mean(T4_true - T4_des) over final 20%% — positive = above setpoint\n');
fprintf('  SS RMSE       : sqrt(mean((T4_true - T4_des)^2)) over final 20%%\n');
fprintf('  SS NRMSE      : SS_RMSE / T4_des x 100 — normalized for cross-setpoint comparison\n');
fprintf('  Term. Time    : Total simulation time (no early termination)\n');
 
%% ---- Figure 4: Summary statistics bar charts ----
figure('Color','w','Position',[200 200 1800 500]);
sgtitle(sprintf('RBF: Summary Statistics, Setpoint = %g (a.u.)', T4_des), ...
    'FontSize', 20, 'FontWeight', 'bold');
 
subplot(1,4,1);
bar(sigma_values, metrics.rise_time, 'FaceColor', [0.3 0.6 0.9]);
xlabel('\sigma', 'FontSize', 18); ylabel('Rise Time (h)', 'FontSize', 18);
title('Rise Time (10%→90%)', 'FontSize', 20); grid on;
set(gca, 'FontSize', 18);
 
subplot(1,4,2);
bar(sigma_values, metrics.ss_mean_error, 'FaceColor', [0.4 0.7 0.7]);
xlabel('\sigma', 'FontSize', 18); ylabel('(a.u.)', 'FontSize', 18);
title('SS Mean Error (final 20%)', 'FontSize', 20); grid on;
set(gca, 'FontSize', 18);
 
subplot(1,4,3);
bar(sigma_values, metrics.ss_rmse, 'FaceColor', [0.7 0.5 0.8]);
xlabel('\sigma', 'FontSize', 18); ylabel('(a.u.)', 'FontSize', 18);
title('SS RMSE (final 20%)', 'FontSize', 20); grid on;
set(gca, 'FontSize', 18);
 
subplot(1,4,4);
bar(sigma_values, metrics.ss_nrmse, 'FaceColor', [0.8 0.6 0.4]);
xlabel('\sigma', 'FontSize', 18); ylabel('(%)', 'FontSize', 18);
title('SS NRMSE (final 20%)', 'FontSize', 20); grid on;
set(gca, 'FontSize', 18);
 
if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_sigma_sweep_summary_%s', sp_tag)); end
 
% ---- Save metrics ----
metrics_file = fullfile(output_dir, sprintf('RBF_sigma_sweep_metrics_%s.mat', sp_tag));
save(metrics_file, 'metrics', 'sigma_values', 'T4_des');
fprintf('\nMetrics saved to: %s\n', metrics_file);
 
metrics_txt = fullfile(output_dir, sprintf('RBF_sigma_sweep_metrics_%s.txt', sp_tag));
fid = fopen(metrics_txt, 'w');
fprintf(fid, 'Performance Metrics: RBF Controller (sigma sweep)\n');
fprintf(fid, 'Setpoint T4_des = %.1f ug/dL\n', T4_des);
fprintf(fid, '+/-%d%% band width = %.2f ug/dL\n', round(BAND_pct*100), BAND_width);
fprintf(fid, 'Measurement bias (true) = %.2f (%.0f%% of setpoint, no estimator — RBF absorbs implicitly)\n', MEAS_template.bias_true, 100*MEAS_template.bias_true/T4_des);
fprintf(fid, 'Perturbation: same seed (rng(1)) and ranges as PID code\n\n');
fprintf(fid, '%-8s | %12s | %12s | %12s | %12s | %12s | %12s\n', ...
    'sigma', 'Rise Time', 'Overshoot', 'SS MeanErr', 'SS RMSE', 'SS NRMSE', 'Term. Time');
fprintf(fid, '%-8s | %12s | %12s | %12s | %12s | %12s | %12s\n', ...
    '', '(hr)', '(%)', '(ug/dL)', '(ug/dL)', '(%)', '(hr)');
fprintf(fid, '--------------------------------------------------------------------------------------------\n');
for i = 1:num_sigma
    fprintf(fid, '%-8.2f | %12.2f | %12.2f | %12.4f | %12.4f | %12.2f | %12.1f\n', ...
        sigma_values(i), metrics.rise_time(i), metrics.overshoot(i), ...
        metrics.ss_mean_error(i), metrics.ss_rmse(i), metrics.ss_nrmse(i), metrics.term_time(i));
end
fprintf(fid, '--------------------------------------------------------------------------------------------\n');
fclose(fid);
fprintf('Text metrics saved to: %s\n', metrics_txt);
fprintf('\nAll plots generated.\n');
fprintf('Results file: %s\n', results_file);

%% ========================================================================
%                              FUNCTIONS
%% ========================================================================

%% ---- Perturbed plant ODE (minutes)  -----------
function dydt_min = eng_time_varying_ode_EF_MINUTES_PERT_refined( ...
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

%% ---- Neutral perturbation (nominal plant, all zeros, for testing) --------------------
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

%% ---- Utilities ----------------------------------------------------------
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