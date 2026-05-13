clear all; close all; clc;

% This code generates figure 4 of the accompanying manuscript

% Sequential setpoint tracking: warm-start RBF controller.
% Setpoints change mid-run (e.g., 12 -> 20 -> 16).
% Weights carry over across transitions; band lock, A_basal, and e_int
% reset at each transition.  Transitions occur at measurement boundaries.
%
% Adapted from RBF_setpoint_sweep.m with minimal structural changes.

% ---- Save settings ----
DO_SAVE_PDF = false;
output_dir = fullfile(pwd, 'results_rbf_sequential');
if ~exist(output_dir,'dir'), mkdir(output_dir); end

fprintf('Current Folder (pwd): %s\n', pwd);
fprintf('Output directory:      %s\n', output_dir);

%% ==================== SETPOINT SEQUENCE CONFIGURATION ====================
setpoint_values   = [12, 25, 10];        % sequential setpoints
num_setpoints     = numel(setpoint_values);
segment_duration  = 4500;                 % minutes per setpoint
transition_times  = (0:num_setpoints-1) * segment_duration;  % [0 4000 8000]

%% ==================== SIGMA CONFIGURATION ====================
sigma_values = 0;
num_sigma    = numel(sigma_values);

%% ==================== ABLATION MODE ======================================
ABLATION_MODE = 'none';

%% ==================== BAND template ======================================
BAND_pct_val = 0.20;

% ---- Build filename tags ----
sp_tag    = strjoin(arrayfun(@(v) sprintf('%g', v), setpoint_values, 'UniformOutput', false), '_');
sigma_tag = strjoin(arrayfun(@(v) strrep(sprintf('%.2f', v), '.', 'p'), sigma_values, 'UniformOutput', false), '_');
band_tag  = sprintf('bnd%d', round(BAND_pct_val * 100));

results_file = fullfile(output_dir, sprintf('RBF_sequential_results_%s_%s_sp_%s_sigma_%s.mat', ...
    ABLATION_MODE, band_tag, sp_tag, sigma_tag));

% check for a completed run
if exist(results_file, 'file')
    fprintf('Found existing results file. Loading...\n');
    load(results_file, 'RUN');
    fprintf('Loaded previous sequential run. Skipping simulation.\n');
    run_simulation = false;
else
    ALL_RESULTS    = struct();
    run_simulation = true;
end

%% -------------------- Time Unit Conversion --------------------
SEC_PER_MIN = 60;
toUnit   = @(sec)  sec / SEC_PER_MIN;
fromUnit = @(unit) unit * SEC_PER_MIN;

%% -------------------- Timing parameters --------------------
dt   = 0.1/12;
ti   = 0;
tf   = num_setpoints * segment_duration;   % 12000 minutes
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

gammaext_T4_time_min = @(t_min) gammaext_T4_time(fromUnit(t_min));
M_time_min           = @(t_min) M_time(fromUnit(t_min));
alpha1_func_min      = @(t_min) alpha1_func(fromUnit(t_min));

%% -------------------- Burst & micro-pulse parameters --------------------
t1 = toUnit(500e-6);  t2 = toUnit(100e-6);
t3 = toUnit(500e-6);  t4 = toUnit(9.5e-4);
Tp = t1 + t2 + t3 + t4;
pulseDuty = (t1 + t3)/Tp;

np = 500;
t5 = np * Tp;
t6 = toUnit(0.5);

%% -------------------- Window parameters --------------------
Ts        = 120;
r_duty    = 0.60;
T_on      = r_duty * Ts;
T_cycle   = Ts;
T_total   = tf;

T_measure = 20;

%% -------------------- EF to Amplitude mapping --------------------
thresh = 1e-4;
A_min  = 0.00;  A_max = 0.10;  kA = 0.20;

%% -------------------- RBF parameters --------------------
Nc = 40;
c1d = linspace(0, 1, Nc);
c_template = repmat(c1d, [16, 1]);

beeta = 0.13;
learning_rate_base = 1e-6 * (T_measure / dt);

ki = 0;
e_int_max = 5;

UL = 100;  LL = 0;

%% ==================== Measurement model ====================
MEAS_template.enable    = false;
MEAS_template.alpha_y   = 0.30;

%% ==================== Perturbation config ====================
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
PERT_TRUE.d4_period_min  = 6*60;

PERT_TRUE.range.act_gain   = 0.20;
PERT_TRUE.range.delay_min  = toUnit(10);
PERT_TRUE.range.jitter_min = toUnit(5);

%% ==================== Perturbation sampling ====================
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
    otherwise
        pert_true = sample_perturbation(PERT_TRUE.range, PERT_TRUE.d4_period_min);
        fprintf('*** ABLATION: full perturbations ***\n');
end

%% ==================== SINGLE SEQUENTIAL RUN ====================
% (sigma loop retained for structural compatibility)
if run_simulation    
for sigma_idx = 1:num_sigma

    current_sigma = sigma_values(sigma_idx);
    fprintf('\n========================================\n');
    fprintf('Sequential run: setpoints = %s, sigma = %.2f\n', ...
        mat2str(setpoint_values), current_sigma);
    fprintf('Segment duration = %g min (%.1f h each)\n', segment_duration, segment_duration/60);
    fprintf('========================================\n');

    % ---- First setpoint initialisation ----
    seg_idx        = 1;
    current_T4_des = setpoint_values(1);
    W_max = (current_T4_des/1000 + 0.005);
    MEAS_bias_true = 0;

    % ---- MEAS for this run ----
    MEAS            = MEAS_template;
    MEAS.sigma      = current_sigma;
    MEAS.bias_true  = MEAS_bias_true;
    meas_state.y_filt = NaN;

    % ---- RBF weights & centres ----
    W  = 0.5 * W_max * ones(1, Nc);
  
    c  = c_template;
    learning_rate = learning_rate_base;

    % ---- State & log initialisation ----
    y0=[229.437253584811; 0.273804879029534;...
    22.9437253584891; 4.10707318544213;...
    zeros(11,1);25];

    y_log = zeros(16, len);
    y_log(:,1) = y0;

    % ---- Piecewise-constant reference ----
    ref = zeros(1, len);
    for sp_i = 1:num_setpoints
        ref(time_vector >= transition_times(sp_i)) = setpoint_values(sp_i);
    end

    new_in = zeros(16, len);
    new_in(1,:) = ref;
    x = 0;
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

    BAND.enable    = true;
    BAND.pct       = BAND_pct_val;
    BAND.width     = BAND_pct_val * current_T4_des;
    BAND.holdN     = 2;
    BAND.entered   = false;
    BAND.inband_ct = 0;
    BAND.reported  = false;
    BAND.min_A     = inf;

    band_lock_time = NaN;

    sample_times_amp = ti:Ts:(tf + Ts);
    m_amp = 1;
    next_window_time = sample_times_amp(1);

    % ---- T4 measurement hold logic ----
    T4_hold = 0;
    T4_sample_times    = ti:T_measure:(tf + T_measure);
    m_T4 = 1;
    next_T4_measure_time = T4_sample_times(1);

    ED = zeros(len, Nc);
    u_hold = 0;

    % ---- Per-segment tracking ----
    band_lock_times  = nan(1, num_setpoints);
    A_basal_values   = zeros(1, num_setpoints);
    transition_indices = ones(1, num_setpoints);  % sim indices of actual transitions
    transition_indices(1) = 1;

    %% -------------------- Main simulation loop --------------------
    for i = 1 : len-1

        % ---- Normalise inputs for RBF ----
        input_scale = 150 * ones(16,1);
        norm_input  = new_in(:,i) ./ input_scale;

        for j = 1:Nc
            ED(i,j) = exp(-((norm_input - c(:,j))' * (norm_input - c(:,j))) / (2*beeta^2));
        end

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
                if abs(x - current_T4_des) <= BAND.width
                    BAND.inband_ct = BAND.inband_ct + 1;
                    if ~BAND.reported && BAND.inband_ct >= 1
                        fprintf('  [%.1f h] First entered +/-%d%% band of sp=%g (count %d/%d)\n', ...
                            current_time/60, BAND.pct*100, current_T4_des, BAND.inband_ct, BAND.holdN);
                        BAND.reported = true;
                    end
                    if BAND.inband_ct >= BAND.holdN
                        BAND.entered = true;
                        %A_basal = A_hold;               % good for steps up
                        %A_basal = max(A_basal, A_hold); % allows setpoint drops
                        % logic for set ups/downs
                        if A_basal > 0
                            % Downward step with carry-forward: only upgrade
                            A_basal = max(A_basal, A_hold);
                        else
                            % Upward step or first segment: capture directly
                            A_basal = A_hold;
                        end

                        band_lock_time = current_time;
                        fprintf('  [%.1f h] Band lock ACTIVATED for sp=%g after %d windows\n', ...
                            current_time/60, current_T4_des, BAND.holdN);
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
        % Enforce A_basal floor:
        %   - After band lock (any step direction)
        %   - Before band lock (downward steps only: A_basal > 0 from carry-forward;
        %     upward steps have A_basal = 0 so it just returns A_hold)

        if BAND.entered || A_basal > 0
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

        % ---- ODE step ----
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
            T4_hold = x;

            % ============================================================
            %  SETPOINT TRANSITION CHECK (at measurement boundaries)
            % ============================================================
            if seg_idx < num_setpoints && current_time_next >= transition_times(seg_idx + 1)
                % Store outgoing segment stats
                A_basal_values(seg_idx)  = A_basal;
                band_lock_times(seg_idx) = band_lock_time;

                seg_idx = seg_idx + 1;
                current_T4_des = setpoint_values(seg_idx);
                transition_indices(seg_idx) = i + 1;

                fprintf('  [%.1f h] === SETPOINT TRANSITION: T4_des = %g ===\n', ...
                    current_time_next/60, current_T4_des);

                % Reset band lock state
                BAND.entered   = false;
                BAND.inband_ct = 0;
                BAND.reported  = false;
                BAND.width     = BAND_pct_val * current_T4_des;
                %A_basal        = 0;
                
                % logit to scale A_basal if downward, reset to 0 if up. 
                A_basal = A_basal * (current_T4_des / setpoint_values(seg_idx - 1));
                if current_T4_des < setpoint_values(seg_idx - 1)
                    % Downward step: carry forward scaled A_basal as floor
                    A_basal = A_basal * (current_T4_des / setpoint_values(seg_idx - 1));
                else
                    % Upward step: reset to zero, let controller ramp naturally
                    A_basal = 0;
                end
                
                band_lock_time = NaN;

                % Reset integrator
                e_int = 0;

                % Update weight clamp for new setpoint
                W_max = (current_T4_des/1000 + 0.005);
            end
            % ============================================================

            e = ref(i+1) - x;

            % === Adaptation: FREEZE once band lock is active ===
            if ~BAND.entered
                e_int = e_int + e * T_measure;
                e_int = clamp(e_int, -e_int_max, e_int_max);

                m_sq = 1 + (e / max(ref(i+1), 1e-6))^2 + ED(i,:) * ED(i,:)';
                W = W + learning_rate * e * ED(i,:) / m_sq;
                %W = clamp(W, -W_max, W_max);
                W = clamp(W, 0.0, W_max);
                eta_c = 1e-5;
                for j = 1:Nc
                    phi_j  = ED(i,j);
                    c(:,j) = c(:,j) + eta_c * e * W(j) * phi_j ...
                             .* (norm_input - c(:,j)) / (beeta^2);
                end
            end

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

    % Store final segment stats
    A_basal_values(seg_idx)  = A_basal;
    band_lock_times(seg_idx) = band_lock_time;

    %% -------------------- Store results --------------------
    T4_true = y_log(4,:);

    RUN.setpoint_values   = setpoint_values;
    RUN.transition_times  = transition_times;
    RUN.segment_duration  = segment_duration;
    RUN.sigma             = current_sigma;
    RUN.time              = time_vector;
    RUN.T4_log            = T4_true;
    RUN.T4_meas_used      = T4_measured_log;
    RUN.T4_meas_raw       = T4_measured_raw_log;
    RUN.u                 = u;
    RUN.A_log             = A_log;
    RUN.EF_log            = EF_log;
    RUN.Weights_final     = W;
    RUN.Weights           = Weights;
    RUN.centres_final     = c;
    RUN.new_in            = new_in;
    RUN.ref               = ref;
    RUN.band_lock_times   = band_lock_times;
    RUN.A_basal_values    = A_basal_values;
    RUN.transition_indices = transition_indices;

    fprintf('\nRun complete. Saving...\n');
    save(results_file, 'RUN', 'setpoint_values', 'sigma_values', ...
         'ABLATION_MODE', 'BAND_pct_val', 'transition_times', ...
         'segment_duration', '-v7.3');
    fprintf('Results saved to: %s\n', results_file);
end
end
fprintf('\n========================================\n');
fprintf('Sequential run finished.\n');
fprintf('========================================\n');

%% ==================== PLOTS ====================
fprintf('\nGenerating plots...\n');

tf_h = tf / 60;
t_hr = RUN.time / 60;
T4_true = RUN.T4_log;

% Colors
sp_colors = lines(num_setpoints);
transition_color = [0.6 0.0 0.8];   % purple for setpoint transitions

% Transition times in hours (skip the first at t=0)
trans_hr = transition_times(2:end) / 60;

%% ---- Figure 1: T4 trajectory with setpoint steps ----
figure('Color','w','Position',[100 100 1200 600]);
hold on;

% Setpoint reference as staircase
stairs(t_hr, RUN.ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Setpoint');

% T4 trajectory
plot(t_hr, T4_true, '-', 'Color', [0.0 0.45 0.74], 'LineWidth', 1.3, ...
    'DisplayName', 'T_4^{ext}');

% Transition lines
for k = 1:length(trans_hr)
    xline(trans_hr(k), '-', 'Color', transition_color, 'LineWidth', 1.8, ...
        'Label', sprintf('sp \\rightarrow %g', setpoint_values(k+1)), ...
        'FontSize', 16, 'LabelVerticalAlignment', 'bottom', ...
        'HandleVisibility', 'off');
end

% Band lock lines
for k = 1:num_setpoints
    if ~isnan(RUN.band_lock_times(k))
        xline(RUN.band_lock_times(k)/60, '--k', 'LineWidth', 1.2, ...
            'Label', sprintf('Lock (sp=%g)', setpoint_values(k)), ...
            'FontSize', 16, 'LabelVerticalAlignment', 'bottom', ...
            'HandleVisibility', 'off');
    end
end

xlabel('Time (h)', 'FontSize', 18); ylabel('T_4^{ext}  (a.u.)', 'FontSize', 18);
title('RBF Sequential Tracking: T_4 Trajectory', 'FontSize', 22, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 18); grid on;
xlim([0 tf_h]);
set(gca, 'FontSize', 16);

if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_sequential_trajectory_sp_%s', sp_tag)); end

%% ---- Figure 2: Zoomed settling per segment ----
figure('Color','w','Position',[120 120 400*num_setpoints 500]);
sgtitle('RBF Sequential: Settling Detail per Segment', 'FontSize', 20, 'FontWeight', 'bold');

for seg = 1:num_setpoints
    subplot(1, num_setpoints, seg); hold on;

    t_start_h = transition_times(seg) / 60;
    if seg < num_setpoints
        t_end_h = transition_times(seg+1) / 60;
    else
        t_end_h = tf_h;
    end
    sp_val = setpoint_values(seg);
    col = sp_colors(seg,:);

    % Show last 15 h of segment (or full segment if shorter)
    view_start = max(t_start_h, t_end_h - 15);

    yline(sp_val, '--', 'Color', col, 'LineWidth', 1.5);
    yline(sp_val*(1+BAND_pct_val), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8);
    yline(sp_val*(1-BAND_pct_val), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8);

    seg_mask = (t_hr >= view_start) & (t_hr <= t_end_h);
    plot(t_hr(seg_mask), T4_true(seg_mask), '-', 'Color', col, 'LineWidth', 1.3);

    if ~isnan(RUN.band_lock_times(seg))
        bl_h = RUN.band_lock_times(seg)/60;
        if bl_h >= view_start
            xline(bl_h, '--k', 'LineWidth', 1.2, 'Label', 'Lock', 'FontSize', 10);
        end
    end

    xlabel('Time (h)', 'FontSize', 14); ylabel('T_4^{ext}', 'FontSize', 14);
    title(sprintf('Segment %d: sp = %g', seg, sp_val), 'FontSize', 16);
    xlim([view_start, t_end_h]); grid on;
    set(gca, 'FontSize', 14);
end

if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_sequential_settling_sp_%s', sp_tag)); end

%% ---- Figure 3: Tracking error (single panel) ----
figure('Color','w','Position',[140 140 1200 500]);
hold on;
yline(0, '--k', 'LineWidth', 0.8, 'HandleVisibility', 'off');

tracking_error = RUN.ref - T4_true;
plot(t_hr, tracking_error, '-', 'Color', [0.0 0.45 0.74], 'LineWidth', 1.0);

for k = 1:length(trans_hr)
    xline(trans_hr(k), '-', 'Color', transition_color, 'LineWidth', 1.8, ...
        'HandleVisibility', 'off');
end
for k = 1:num_setpoints
    if ~isnan(RUN.band_lock_times(k))
        xline(RUN.band_lock_times(k)/60, '--k', 'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
    end
end

xlabel('Time (h)', 'FontSize', 18); ylabel('Error  (a.u.)', 'FontSize', 18);
title('RBF Sequential: Tracking Error (ref - T_4)', 'FontSize', 20, 'FontWeight', 'bold');
grid on; xlim([0 tf_h]);
set(gca, 'FontSize', 16);

if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_sequential_error_sp_%s', sp_tag)); end

%% ---- Figure 4: Control signals (single run, transition lines) ----
figure('Color','w','Position',[160 160 1400 500]);
sgtitle('RBF Sequential: Control Signals', 'FontSize', 20, 'FontWeight', 'bold');

abasal_colors = [0.85 0.33 0.10;   % red-orange
                 0.47 0.67 0.19;   % green
                 0.64 0.08 0.18];  % dark red

subplot(1,2,1); hold on;
plot(t_hr, RUN.u, '-', 'Color', [0.0 0.45 0.74], 'LineWidth', 1.0);
for k = 1:length(trans_hr)
    xline(trans_hr(k), '-', 'Color', transition_color, 'LineWidth', 1.8, ...
        'HandleVisibility', 'off');
end
for k = 1:num_setpoints
    if ~isnan(RUN.band_lock_times(k))
        xline(RUN.band_lock_times(k)/60, '--k', 'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
    end
end
xlabel('Time (h)', 'FontSize', 18); ylabel('u (RBF output)', 'FontSize', 18);
title('RBF Control Output', 'FontSize', 18); grid on; xlim([0 tf_h]);
set(gca, 'FontSize', 16);

subplot(1,2,2); hold on;
stairs(t_hr, RUN.A_log, '-', 'Color', [0.0 0.45 0.74], 'LineWidth', 1.0, ...
    'HandleVisibility', 'off');
for seg = 1:num_setpoints
    if RUN.A_basal_values(seg) > 0
        % Draw A_basal only during its active segment
        t_s = transition_times(seg)/60;
        if seg < num_setpoints
            t_e = transition_times(seg+1)/60;
        else
            t_e = tf_h;
        end
        plot([t_s t_e], RUN.A_basal_values(seg)*[1 1], '--', ...
            'Color', abasal_colors(seg,:), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('A_{basal} (sp=%g)', setpoint_values(seg)));
    end
end
for k = 1:length(trans_hr)
    xline(trans_hr(k), '-', 'Color', transition_color, 'LineWidth', 1.8, ...
        'HandleVisibility', 'off');
end
for k = 1:num_setpoints
    if ~isnan(RUN.band_lock_times(k))
        xline(RUN.band_lock_times(k)/60, '--k', 'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
    end
end
xlabel('Time (h)', 'FontSize', 18); ylabel('Amplitude A (a.u.)', 'FontSize', 18);
title('Control Amplitude', 'FontSize', 18);
legend('Location','best','FontSize',14); grid on; xlim([0 tf_h]);
set(gca, 'FontSize', 16);

if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_sequential_control_sp_%s', sp_tag)); end

%% ---- Figure 5: Weight evolution (single panel, transition lines) ----
figure('Color','w','Position',[200 200 1200 500]);
hold on;

plot(t_hr, RUN.Weights, '-', 'LineWidth', 0.6);

for k = 1:length(trans_hr)
    xline(trans_hr(k), '-', 'Color', transition_color, 'LineWidth', 1.8, ...
        'Label', sprintf('sp \\rightarrow %g', setpoint_values(k+1)), ...
        'FontSize', 16, 'LabelVerticalAlignment', 'bottom');
end
for k = 1:num_setpoints
    if ~isnan(RUN.band_lock_times(k))
        xline(RUN.band_lock_times(k)/60, '--k', 'LineWidth', 1.2, ...
            'Label', sprintf('Lock (sp=%g)', setpoint_values(k)), ...
            'FontSize', 16, 'LabelVerticalAlignment', 'top');
    end
end

xlabel('Time (h)', 'FontSize', 18); ylabel('Weight value', 'FontSize', 18);
title('RBF Sequential: Weight Evolution', 'FontSize', 20, 'FontWeight', 'bold');
grid on; xlim([0 tf_h]);
set(gca, 'FontSize', 16);

if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_sequential_weights_sp_%s', sp_tag)); end

%% ==================== PER-SEGMENT PERFORMANCE METRICS ====================
fprintf('\n');
fprintf('============================================================================================\n');
fprintf('  RBF SEQUENTIAL — PER-SEGMENT METRICS (ablation = %s)\n', ABLATION_MODE);
fprintf('============================================================================================\n');
fprintf('%-4s | %-8s | %10s | %10s | %10s | %10s | %10s | %10s\n', ...
    'Seg', 'T4_des', 'Resp(hr)', 'OS(%%)', 'SS_ME', 'SS_RMSE', 'SS_NRMSE%%', 'Lock(hr)');
fprintf('--------------------------------------------------------------------------------------------\n');

metrics = struct();
metrics.setpoint_values = setpoint_values;
metrics.response_time   = nan(1, num_setpoints);
metrics.overshoot       = nan(1, num_setpoints);
metrics.ss_mean_error   = nan(1, num_setpoints);
metrics.ss_rmse         = nan(1, num_setpoints);
metrics.ss_nrmse        = nan(1, num_setpoints);
metrics.band_lock_hr    = nan(1, num_setpoints);

for seg = 1:num_setpoints
    sp_val  = setpoint_values(seg);
    t_start = transition_times(seg);
    if seg < num_setpoints
        t_end = transition_times(seg+1);
    else
        t_end = tf;
    end

    seg_mask = (time_vector >= t_start) & (time_vector < t_end);
    T4_seg   = T4_true(seg_mask);
    t_seg    = time_vector(seg_mask);
    n_seg    = length(T4_seg);

    % Response time: time from segment start to first entry into ±band
    band_w = BAND_pct_val * sp_val;
    idx_in_band = find(abs(T4_seg - sp_val) <= band_w, 1, 'first');
    if ~isempty(idx_in_band)
        metrics.response_time(seg) = (t_seg(idx_in_band) - t_start) / 60;  % hours
    end

    % Overshoot: peak deviation past setpoint after first crossing
    if seg == 1 || setpoint_values(seg) > setpoint_values(seg-1)
        % Upward step: overshoot above
        idx_cross = find(T4_seg >= sp_val, 1, 'first');
        if ~isempty(idx_cross)
            T4_peak = max(T4_seg(idx_cross:end));
            metrics.overshoot(seg) = max(0, (T4_peak - sp_val) / sp_val * 100);
        end
    else
        % Downward step: undershoot below
        idx_cross = find(T4_seg <= sp_val, 1, 'first');
        if ~isempty(idx_cross)
            T4_trough = min(T4_seg(idx_cross:end));
            metrics.overshoot(seg) = max(0, (sp_val - T4_trough) / sp_val * 100);
        end
    end

    % Steady-state metrics (final 20% of segment)
    ss_start = round(0.80 * n_seg);
    ss_idx   = ss_start : n_seg;
    ss_error = T4_seg(ss_idx) - sp_val;
    metrics.ss_mean_error(seg) = mean(ss_error);
    metrics.ss_rmse(seg)       = sqrt(mean(ss_error.^2));
    metrics.ss_nrmse(seg)      = metrics.ss_rmse(seg) / sp_val * 100;

    % Band lock time (relative to segment start)
    if ~isnan(RUN.band_lock_times(seg))
        metrics.band_lock_hr(seg) = (RUN.band_lock_times(seg) - t_start) / 60;
    end

    fprintf('%-4d | %-8g | %10.2f | %10.2f | %10.4f | %10.4f | %10.2f | %10.1f\n', ...
        seg, sp_val, ...
        metrics.response_time(seg), metrics.overshoot(seg), ...
        metrics.ss_mean_error(seg), metrics.ss_rmse(seg), ...
        metrics.ss_nrmse(seg), metrics.band_lock_hr(seg));
end

fprintf('============================================================================================\n');
fprintf('\nDefinitions:\n');
fprintf('  Response Time : Time from segment start to first entry into +/-%d%% band (hours)\n', round(BAND_pct_val*100));
fprintf('  Overshoot     : Peak deviation past setpoint after first crossing / setpoint x 100\n');
fprintf('  SS Mean Error : mean(T4_true - T4_des) over final 20%% of segment\n');
fprintf('  SS RMSE       : sqrt(mean((T4_true - T4_des)^2)) over final 20%% of segment\n');
fprintf('  SS NRMSE      : SS_RMSE / T4_des x 100\n');
fprintf('  Lock Time     : Time from segment start to band lock activation (hours)\n');

%% ---- Figure 6: Summary bar charts per segment ----
figure('Color','w','Position',[220 220 1800 500]);
sgtitle('RBF Sequential: Per-Segment Summary', 'FontSize', 20, 'FontWeight', 'bold');

seg_labels = arrayfun(@(k) sprintf('%g (seg %d)', setpoint_values(k), k), ...
    1:num_setpoints, 'UniformOutput', false);

subplot(1,4,1);
bar(categorical(seg_labels, seg_labels), metrics.response_time, 'FaceColor', [0.3 0.6 0.9]);
ylabel('Response Time (h)', 'FontSize', 16); title('Response Time', 'FontSize', 18);
xlabel('Setpoint', 'FontSize', 16); grid on; set(gca, 'FontSize', 14);

subplot(1,4,2);
bar(categorical(seg_labels, seg_labels), metrics.ss_mean_error, 'FaceColor', [0.4 0.7 0.7]);
ylabel('(a.u.)', 'FontSize', 16); title('SS Mean Error', 'FontSize', 18);
xlabel('Setpoint', 'FontSize', 16); grid on; set(gca, 'FontSize', 14);

subplot(1,4,3);
bar(categorical(seg_labels, seg_labels), metrics.ss_rmse, 'FaceColor', [0.7 0.5 0.8]);
ylabel('(a.u.)', 'FontSize', 16); title('SS RMSE', 'FontSize', 18);
xlabel('Setpoint', 'FontSize', 16); grid on; set(gca, 'FontSize', 14);

subplot(1,4,4);
bar(categorical(seg_labels, seg_labels), metrics.ss_nrmse, 'FaceColor', [0.8 0.6 0.4]);
ylabel('(%)', 'FontSize', 16); title('SS NRMSE', 'FontSize', 18);
xlabel('Setpoint', 'FontSize', 16); grid on; set(gca, 'FontSize', 14);

if DO_SAVE_PDF, savePlot(gcf, output_dir, sprintf('RBF_sequential_summary_bars_sp_%s', sp_tag)); end

%% ---- Save metrics ----
metrics_file = fullfile(output_dir, sprintf('RBF_sequential_metrics_%s_sp_%s.mat', ABLATION_MODE, sp_tag));
save(metrics_file, 'metrics', 'setpoint_values', 'transition_times', 'ABLATION_MODE');
fprintf('\nMetrics saved to: %s\n', metrics_file);

metrics_txt = fullfile(output_dir, sprintf('RBF_sequential_metrics_%s_sp_%s.txt', ABLATION_MODE, sp_tag));
fid = fopen(metrics_txt, 'w');
fprintf(fid, 'Performance Metrics: RBF Sequential Controller\n');
fprintf(fid, 'Ablation mode: %s\n', ABLATION_MODE);
fprintf(fid, 'Setpoint sequence: %s\n', mat2str(setpoint_values));
fprintf(fid, 'Segment duration: %g min (%.1f h)\n', segment_duration, segment_duration/60);
fprintf(fid, 'Transition times: %s min\n', mat2str(transition_times));
fprintf(fid, 'Band width: +/-%d%%\n\n', round(BAND_pct_val*100));
fprintf(fid, '%-4s | %-8s | %10s | %10s | %10s | %10s | %10s | %10s\n', ...
    'Seg', 'T4_des', 'Resp(hr)', 'OS(%%)', 'SS_ME', 'SS_RMSE', 'SS_NRMSE%%', 'Lock(hr)');
fprintf(fid, '--------------------------------------------------------------------------------------------\n');
for seg = 1:num_setpoints
    fprintf(fid, '%-4d | %-8g | %10.2f | %10.2f | %10.4f | %10.4f | %10.2f | %10.1f\n', ...
        seg, setpoint_values(seg), ...
        metrics.response_time(seg), metrics.overshoot(seg), ...
        metrics.ss_mean_error(seg), metrics.ss_rmse(seg), ...
        metrics.ss_nrmse(seg), metrics.band_lock_hr(seg));
end
fprintf(fid, '--------------------------------------------------------------------------------------------\n');
fclose(fid);
fprintf('Text metrics saved to: %s\n', metrics_txt);
fprintf('\nAll plots generated.\n');

%% ========================================================================
%                              FUNCTIONS
%% ========================================================================

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

function [y_used, st] = measurement_filter(y_raw, st, MEAS)
    if isnan(st.y_filt), st.y_filt = y_raw; end

    a = MEAS.alpha_y;
    st.y_filt = (1-a)*st.y_filt + a*y_raw;

    y_used = st.y_filt;
end

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