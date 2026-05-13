clear all; close all; clc;

% This code generates figures 7, 8, and the data used in figure 6 
% (The same holds for the repeated figures in the supplementary). 
% Specific seeds (seen in figure 6) are viewed with the file
% MC_seed_viewer.m


%% ========================================================================
%  RBF Controller — Monte Carlo Setpoint Sweep
%
%  Outer loop over RNG seeds (1:num_seeds).  Each seed draws one perturbed plant;
%  all setpoints share that realization (paired design).
%
%  Storage optimisations vs. the single-run script:
%    - Weights logged at measurement times only  (~226 rows, not 540 k)
%    - new_in   logged at measurement times only
%    - meas_times vector stored for indexing
%
%  Outputs:
%    - Per-seed .mat  (full ALL_RESULTS with lean Weights/new_in)
%    - MC_aggregate.mat  (checkpoint after every seed)
%    - Final bar chart: mean ± σ SS_NRMSE by setpoint
%    - CSV of per-seed metrics
%
% USES A_streak and the leaky integrator
%% ========================================================================

%% ==================== MONTE CARLO CONFIGURATION ==========================
num_seeds = 100;
seed_list = 1:num_seeds;

%% ==================== SETPOINT / SIGMA SWEEP ============================
setpoint_values = [12, 16, 20, 25];
num_setpoints   = numel(setpoint_values);

sigma_values = 2.0;
num_sigma    = numel(sigma_values);

%% ==================== ABLATION MODE =====================================
ABLATION_MODE = 'full';

%% ==================== BAND TEMPLATE =====================================
BAND_pct_val = 0.20;

%% ==================== OUTPUT DIRECTORY ==================================
output_dir ='C:/Users/nicho/Documents/MATLAB/T4_March/results_rbf_monte_carlo'; %<- make MC_seed_viewer match this. 
if ~exist(output_dir, 'dir'), mkdir(output_dir); end

fprintf('Monte Carlo RBF Sweep — %d seeds × %d setpoints\n', num_seeds, num_setpoints);
fprintf('Output directory: %s\n\n', output_dir);

%% ==================== AGGREGATE STORAGE =================================
MC_rise_time     = nan(num_seeds, num_setpoints);
MC_overshoot     = nan(num_seeds, num_setpoints);
MC_ss_mean_error = nan(num_seeds, num_setpoints);
MC_ss_rmse       = nan(num_seeds, num_setpoints);
MC_ss_nrmse      = nan(num_seeds, num_setpoints);
MC_band_lock_hr  = nan(num_seeds, num_setpoints);

%% ==================== RESUME LOGIC ======================================
agg_file = fullfile(output_dir, 'MC_aggregate.mat');
completed_seeds = false(1, num_seeds);

for s = 1:num_seeds
    seed_file = fullfile(output_dir, sprintf('RBF_MC_seed%03d.mat', seed_list(s)));
    if exist(seed_file, 'file')
        completed_seeds(s) = true;
    end
end

% Load existing aggregate if present
if exist(agg_file, 'file')
    loaded = load(agg_file);
    if isfield(loaded, 'MC_rise_time'),     MC_rise_time     = loaded.MC_rise_time;     end
    if isfield(loaded, 'MC_overshoot'),     MC_overshoot     = loaded.MC_overshoot;     end
    if isfield(loaded, 'MC_ss_mean_error'), MC_ss_mean_error = loaded.MC_ss_mean_error; end
    if isfield(loaded, 'MC_ss_rmse'),       MC_ss_rmse       = loaded.MC_ss_rmse;       end
    if isfield(loaded, 'MC_ss_nrmse'),      MC_ss_nrmse      = loaded.MC_ss_nrmse;      end
    if isfield(loaded, 'MC_band_lock_hr'),  MC_band_lock_hr  = loaded.MC_band_lock_hr;  end
    fprintf('Loaded aggregate checkpoint. %d/%d seeds already complete.\n', ...
        sum(completed_seeds), num_seeds);
end

% Reload metrics from existing per-seed files that aren't in aggregate
for s = 1:num_seeds
    if completed_seeds(s) && any(isnan(MC_ss_nrmse(s,:)))
        seed_file = fullfile(output_dir, sprintf('RBF_MC_seed%03d.mat', seed_list(s)));
        tmp = load(seed_file, 'seed_metrics');
        if isfield(tmp, 'seed_metrics')
            MC_rise_time(s,:)     = tmp.seed_metrics.rise_time(:,1)';
            MC_overshoot(s,:)     = tmp.seed_metrics.overshoot(:,1)';
            MC_ss_mean_error(s,:) = tmp.seed_metrics.ss_mean_error(:,1)';
            MC_ss_rmse(s,:)       = tmp.seed_metrics.ss_rmse(:,1)';
            MC_ss_nrmse(s,:)      = tmp.seed_metrics.ss_nrmse(:,1)';
            MC_band_lock_hr(s,:)  = tmp.seed_metrics.band_lock_hr(:,1)';
        end
    end
end

%% -------------------- Time Unit Conversion --------------------
SEC_PER_MIN = 60;
toUnit   = @(sec)  sec / SEC_PER_MIN;
fromUnit = @(unit) unit * SEC_PER_MIN;

%% -------------------- Timing parameters --------------------
dt   = 0.1/12;
ti   = 0;
tf   = 8000;
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

%% ==================== Nominal plant parameters ==========================
NOM.alpha   = 20*8.43812;
NOM.K       = 10*0.1195;
NOM.gamma1  = 0.0296;
NOM.kappa   = 1;
NOM.K2      = 1000;
NOM.gammaI  = 0.004;
NOM.gammaTg = 0.04;
NOM.gint    = 0.15 + 0.0004;
NOM.gext    = gammaext1_0;
NOM.M       = M0;
NOM.alpha1  = alpha1_0;

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

ki = 0.01;
e_int_max = 2.25;
lambda_leak = 0.05;
INT_COND.enable     = true;
INT_COND.min_time   = 20 * 60;
INT_COND.frac_thresh = 0.40;

UL = 100;  LL = 0;

%% ==================== Measurement model (template) ======================
MEAS_template.enable    = true;
MEAS_template.alpha_y   = 0.30;

%% ==================== Perturbation config ===============================
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

%% ==================== Expected measurement count ========================
n_meas_expected = ceil((tf - ti) / T_measure) + 1;

%% ========================================================================
%                     MAIN MONTE CARLO LOOP
%% ========================================================================

mc_timer = tic;

for seed_idx = 1:num_seeds
    RNGesus = seed_list(seed_idx);

    % ---- Skip completed seeds ----
    if completed_seeds(seed_idx)
        fprintf('[Seed %3d/%d] Already complete — skipping.\n', seed_idx, num_seeds);
        continue;
    end

    seed_timer = tic;
    fprintf('\n################################################################\n');
    fprintf('  SEED %d/%d  (RNGesus = %d)\n', seed_idx, num_seeds, RNGesus);
    fprintf('################################################################\n');

    % ---- Sample perturbations for this seed ----
    rng(RNGesus);
    switch ABLATION_MODE
        case 'none'
            pert_true = neutral_perturbation(PERT_TRUE.d4_period_min);
        case 'half'
            half_range = PERT_TRUE.range;
            fnames = fieldnames(half_range);
            for fi = 1:numel(fnames)
                half_range.(fnames{fi}) = half_range.(fnames{fi}) * 0.5;
            end
            pert_true = sample_perturbation(half_range, PERT_TRUE.d4_period_min);
        case 'full_no_rhythm'
            pert_true = sample_perturbation(PERT_TRUE.range, PERT_TRUE.d4_period_min);
            pert_true.d4_bias = 0;
            pert_true.d4_amp  = 0;
        case 'half_no_rhythm'
            half_range = PERT_TRUE.range;
            fnames = fieldnames(half_range);
            for fi = 1:numel(fnames)
                half_range.(fnames{fi}) = half_range.(fnames{fi}) * 0.5;
            end
            pert_true = sample_perturbation(half_range, PERT_TRUE.d4_period_min);
            pert_true.d4_bias = 0;
            pert_true.d4_amp  = 0;
        otherwise  % 'full'
            pert_true = sample_perturbation(PERT_TRUE.range, PERT_TRUE.d4_period_min);
    end

    ALL_RESULTS = struct();

    %% ---- Inner loop over setpoints x sigma ----
    for sp_idx = 1:num_setpoints
        T4_des = setpoint_values(sp_idx);

        W_max = (T4_des/1000 + 0.005);
        MEAS_bias_true = 0;

        BAND_template_run.enable    = true;
        BAND_template_run.pct       = BAND_pct_val;
        BAND_template_run.width     = BAND_pct_val * T4_des;
        BAND_template_run.holdN     = 2;
        BAND_template_run.entered   = false;
        BAND_template_run.inband_ct = 0;
        BAND_template_run.reported  = false;
        BAND_template_run.min_A     = inf;
        BAND_template_run.streak_A  = 0;

        for sigma_idx = 1:num_sigma
            current_sigma = sigma_values(sigma_idx);
            run_label = sprintf('sp=%g sig=%.1f', T4_des, current_sigma);
            fprintf('  [Seed %d] %s ...', seed_idx, run_label);
            run_tic = tic;

            % ---- MEAS for this run ----
            MEAS            = MEAS_template;
            MEAS.sigma      = current_sigma;
            MEAS.bias_true  = MEAS_bias_true;
            meas_state.y_filt = NaN;

            % ---- Reset RBF weights & centres ----
            W  = 0.5 * W_max * ones(1, Nc);
            c  = c_template;
            learning_rate = learning_rate_base;

            % ---- State & log initialisation ----
            y0 = [229.437253584811; 0.273804879029534; ...
                  22.9437253584891; 4.10707318544213; ...
                  zeros(11,1); 25];

            y_log = zeros(16, len);
            y_log(:,1) = y0;

            ref = T4_des * ones(1, len);

            new_in = zeros(16, len);
            new_in(1,:) = ref;
            x = 0;
            new_in(2,1) = x;

            u       = zeros(1, len);
            e       = ref(1) - x;
            e_int   = 0;
            int_active = ~INT_COND.enable;

            log_d   = zeros(3, len);
            log_d(:,1) = [x; 0; e];

            % ---- Measurement-cadence logging for Weights & new_in ----
            Weights_meas = zeros(n_meas_expected, Nc);
            new_in_meas  = zeros(16, n_meas_expected);
            meas_times   = zeros(1, n_meas_expected);
            meas_log_idx = 0;

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
            band_lock_time = NaN;
            sample_times_amp  = ti:Ts:(tf + Ts);
            m_amp = 1;
            next_window_time  = sample_times_amp(1);

            % ---- T4 measurement hold logic ----
            T4_hold = 0;
            T4_sample_times    = ti:T_measure:(tf + T_measure);
            m_T4 = 1;
            next_T4_measure_time = T4_sample_times(1);

            ED = zeros(len, Nc);
            u_hold = 0;

            % ---- Center history at measurement times ----
            centers_log = zeros(16, Nc, n_meas_expected);
            c_log_idx = 0;

            %% ---- Simulation loop ----
            for i = 1 : len-1
                input_scale = 150 * ones(16,1);
                norm_input  = new_in(:,i) ./ input_scale;

                for j = 1:Nc
                    ED(i,j) = exp(-((norm_input - c(:,j))' * (norm_input - c(:,j))) / (2*beeta^2));
                end

                u(i) = u_hold;
                EF = u(i);

                current_time = time_vector(i);
                if current_time >= next_window_time
                    if EF < thresh
                        A_hold = 0;
                    else
                        A_hold = min(max(kA*(EF - thresh), A_min), A_max);
                    end

                    if BAND.enable && ~BAND.entered
                        if abs(x - T4_des) <= BAND.width
                            if BAND.inband_ct == 0
                                BAND.streak_A = A_hold;
                            end
                            BAND.inband_ct = BAND.inband_ct + 1;
                            if A_hold > 0
                                BAND.min_A = min(BAND.min_A, A_hold);
                            end
                            if ~BAND.reported && BAND.inband_ct >= 1
                                BAND.reported = true;
                            end
                            if BAND.inband_ct >= BAND.holdN
                                BAND.entered = true;
                                A_basal = BAND.streak_A;
                                band_lock_time = current_time;
                            end
                        else
                            BAND.inband_ct = 0;
                            BAND.min_A = inf;
                            BAND.streak_A = 0;
                            BAND.reported = false;
                        end
                    end

                    m_amp = m_amp + 1;
                    if m_amp <= length(sample_times_amp)
                        next_window_time = sample_times_amp(m_amp);
                    else
                        next_window_time = inf;
                    end
                end

                if BAND.entered
                    A_hold = max(A_hold, A_basal);
                end

                A_now = A_hold;
                A_log(i) = A_now;

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

                [~, y_temp] = ode15s( ...
                    @(t,y) eng_time_varying_ode_EF_MINUTES_PERT( ...
                        t, y, EF_time_min, gammaext_T4_time_min, ...
                        M_time_min, alpha1_func_min, pert_true, NOM), ...
                    [time_vector(i), time_vector(i+1)], y_log(:,i));
                y_new = y_temp(end,:)';
                y_log(:, i+1) = y_new;

                T4_actual = y_new(4);

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

                    if ~BAND.entered
                        if ~int_active && INT_COND.enable
                            if current_time_next > INT_COND.min_time && ...
                               x < INT_COND.frac_thresh * T4_des
                                int_active = true;
                            end
                        end

                        m_sq = 1 + (e / max(ref(i+1), 1e-6))^2 + ED(i,:) * ED(i,:)';
                        W = W + learning_rate * e * ED(i,:) / m_sq;
                        W = clamp(W, -W_max, W_max);

                        eta_c = 1e-5;
                        for j = 1:Nc
                            phi_j  = ED(i,j);
                            c(:,j) = c(:,j) + eta_c * e * W(j) * phi_j ...
                                     .* (norm_input - c(:,j)) / (beeta^2);
                        end
                    end

                    if ~BAND.entered && int_active
                        e_int = (1 - lambda_leak) * e_int + e * T_measure;
                        e_int = clamp(e_int, -e_int_max, e_int_max);
                    end

                    % ---- Log at measurement cadence ----
                    c_log_idx = c_log_idx + 1;
                    centers_log(:,:,c_log_idx) = c;

                    meas_log_idx = meas_log_idx + 1;
                    Weights_meas(meas_log_idx, :) = W;
                    new_in_meas(:, meas_log_idx)  = new_in(:, i+1);
                    meas_times(meas_log_idx)       = current_time_next;

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
            end

            % Tails
            u(end)     = u(end-1);
            A_log(end) = A_log(end-1);
            EF_log(end) = EF_log(end-1);
            T4_measured_log(end) = T4_measured_log(end-1);

            % ---- Trim measurement logs ----
            Weights_meas = Weights_meas(1:meas_log_idx, :);
            new_in_meas  = new_in_meas(:, 1:meas_log_idx);
            meas_times   = meas_times(1:meas_log_idx);
            centers_log  = centers_log(:,:,1:c_log_idx);

            % ---- Store results ----
            fieldname = sprintf('sp%g_sigma%.2f', T4_des, current_sigma);
            fieldname = strrep(fieldname, '.', 'p');
            fieldname = strrep(fieldname, '-', 'm');

            T4_true = y_log(4,:);

            ALL_RESULTS.(fieldname).T4_des         = T4_des;
            ALL_RESULTS.(fieldname).sigma          = current_sigma;
            ALL_RESULTS.(fieldname).time           = time_vector;
            ALL_RESULTS.(fieldname).T4_log         = T4_true;
            ALL_RESULTS.(fieldname).T4_meas_used   = T4_measured_log;
            ALL_RESULTS.(fieldname).T4_meas_raw    = T4_measured_raw_log;
            ALL_RESULTS.(fieldname).u              = u;
            ALL_RESULTS.(fieldname).A_log          = A_log;
            ALL_RESULTS.(fieldname).EF_log         = EF_log;
            ALL_RESULTS.(fieldname).Weights_final  = W;
            ALL_RESULTS.(fieldname).A_basal        = A_basal;
            ALL_RESULTS.(fieldname).band_lock_time = band_lock_time;
            ALL_RESULTS.(fieldname).RMSE           = sqrt(mean(log_d(3,:).^2));
            ALL_RESULTS.(fieldname).Weights        = Weights_meas;
            ALL_RESULTS.(fieldname).meas_times     = meas_times;
            ALL_RESULTS.(fieldname).centers_final  = c;
            ALL_RESULTS.(fieldname).centers_log    = centers_log;
            ALL_RESULTS.(fieldname).new_in         = new_in_meas;

            fprintf(' done (%.1f min)\n', toc(run_tic)/60);
        end
    end

    %% ---- Compute per-seed metrics ----
    seed_metrics = struct();
    seed_metrics.rise_time     = nan(num_setpoints, num_sigma);
    seed_metrics.overshoot     = nan(num_setpoints, num_sigma);
    seed_metrics.ss_mean_error = nan(num_setpoints, num_sigma);
    seed_metrics.ss_rmse       = nan(num_setpoints, num_sigma);
    seed_metrics.ss_nrmse      = nan(num_setpoints, num_sigma);
    seed_metrics.band_lock_hr  = nan(num_setpoints, num_sigma);

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

            % Rise time (10% -> 90%)
            idx_10 = find(T4_true >= 0.10 * T4_des_i, 1, 'first');
            idx_90 = find(T4_true >= 0.90 * T4_des_i, 1, 'first');
            if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
                seed_metrics.rise_time(sp_i, sigma_i) = t_hr(idx_90) - t_hr(idx_10);
            end

            % Overshoot
            idx_first_sp = find(T4_true >= T4_des_i, 1, 'first');
            if ~isempty(idx_first_sp)
                T4_peak = max(T4_true(idx_first_sp:end));
                if T4_peak > T4_des_i
                    seed_metrics.overshoot(sp_i, sigma_i) = (T4_peak - T4_des_i) / T4_des_i * 100;
                else
                    seed_metrics.overshoot(sp_i, sigma_i) = 0;
                end
            end

            % Steady-state (final 20%)
            ss_idx = round(0.80 * n_pts) : n_pts;
            ss_error = T4_true(ss_idx) - T4_des_i;
            seed_metrics.ss_mean_error(sp_i, sigma_i) = mean(ss_error);
            seed_metrics.ss_rmse(sp_i, sigma_i)       = sqrt(mean(ss_error.^2));
            seed_metrics.ss_nrmse(sp_i, sigma_i)      = seed_metrics.ss_rmse(sp_i, sigma_i) / T4_des_i * 100;

            % Band lock time
            if ~isnan(R.band_lock_time)
                seed_metrics.band_lock_hr(sp_i, sigma_i) = R.band_lock_time / 60;
            end
        end
    end

    %% ---- Save per-seed .mat ----
    seed_file = fullfile(output_dir, sprintf('RBF_MC_seed%03d.mat', RNGesus));
    save(seed_file, 'ALL_RESULTS', 'seed_metrics', 'pert_true', ...
         'setpoint_values', 'sigma_values', 'ABLATION_MODE', ...
         'BAND_pct_val', 'MEAS_template', 'RNGesus', '-v7.3');

    %% ---- Update aggregate ----
    MC_rise_time(seed_idx,:)     = seed_metrics.rise_time(:,1)';
    MC_overshoot(seed_idx,:)     = seed_metrics.overshoot(:,1)';
    MC_ss_mean_error(seed_idx,:) = seed_metrics.ss_mean_error(:,1)';
    MC_ss_rmse(seed_idx,:)       = seed_metrics.ss_rmse(:,1)';
    MC_ss_nrmse(seed_idx,:)      = seed_metrics.ss_nrmse(:,1)';
    MC_band_lock_hr(seed_idx,:)  = seed_metrics.band_lock_hr(:,1)';

    save(agg_file, 'MC_rise_time', 'MC_overshoot', 'MC_ss_mean_error', ...
         'MC_ss_rmse', 'MC_ss_nrmse', 'MC_band_lock_hr', ...
         'setpoint_values', 'sigma_values', 'ABLATION_MODE', ...
         'seed_list', 'num_seeds');

    elapsed_seed = toc(seed_timer) / 60;
    elapsed_total = toc(mc_timer) / 60;
    seeds_done = seed_idx - sum(completed_seeds(1:seed_idx-1));  % newly completed
    seeds_remaining = num_seeds - seed_idx;
    eta_min = (elapsed_total / max(seeds_done,1)) * seeds_remaining;
    fprintf('  Seed %d saved (%.1f min). ETA for remaining %d seeds: %.0f min (%.1f hr)\n', ...
        seed_idx, elapsed_seed, seeds_remaining, eta_min, eta_min/60);
end

fprintf('\n################################################################\n');
fprintf('  ALL %d SEEDS COMPLETE  (total wall time: %.1f hr)\n', ...
    num_seeds, toc(mc_timer)/3600);
fprintf('################################################################\n');

%% ========================================================================
%  POST-PROCESSING: Summary Statistics & Plots
%% ========================================================================

sp_labels = arrayfun(@(v) sprintf('%g', v), setpoint_values, 'UniformOutput', false);

%% ---- Console summary ----
fprintf('\n');
fprintf('============================================================================================\n');
fprintf('  MONTE CARLO SUMMARY — %d seeds,  ablation = %s,  sigma = %s\n', ...
    num_seeds, ABLATION_MODE, mat2str(sigma_values));
fprintf('============================================================================================\n');
fprintf('%-8s | %12s | %12s | %12s | %12s | %12s | %12s\n', ...
    'T4_des', 'Rise(hr)', 'OS(%)', 'SS_ME', 'SS_RMSE', 'SS_NRMSE%', 'Lock(hr)');
fprintf('         | %12s | %12s | %12s | %12s | %12s | %12s\n', ...
    'mean±std', 'mean±std', 'mean±std', 'mean±std', 'mean±std', 'mean±std');
fprintf('--------------------------------------------------------------------------------------------\n');

for sp_i = 1:num_setpoints
    fprintf('%-8g | %5.2f ± %4.2f | %5.2f ± %4.2f | %+6.3f ± %5.3f | %6.4f ± %5.4f | %5.2f ± %4.2f | %5.1f ± %4.1f\n', ...
        setpoint_values(sp_i), ...
        nanmean(MC_rise_time(:,sp_i)),     nanstd(MC_rise_time(:,sp_i)), ...
        nanmean(MC_overshoot(:,sp_i)),     nanstd(MC_overshoot(:,sp_i)), ...
        nanmean(MC_ss_mean_error(:,sp_i)), nanstd(MC_ss_mean_error(:,sp_i)), ...
        nanmean(MC_ss_rmse(:,sp_i)),       nanstd(MC_ss_rmse(:,sp_i)), ...
        nanmean(MC_ss_nrmse(:,sp_i)),      nanstd(MC_ss_nrmse(:,sp_i)), ...
        nanmean(MC_band_lock_hr(:,sp_i)),  nanstd(MC_band_lock_hr(:,sp_i)));
end
fprintf('============================================================================================\n');

%% ---- Figure: Mean ± σ SS_NRMSE bar chart ----
figure('Color', 'w', 'Position', [100 100 700 500]);
mean_nrmse = nanmean(MC_ss_nrmse, 1);
std_nrmse  = nanstd(MC_ss_nrmse, 0, 1);
lower_err  = min(std_nrmse, mean_nrmse);   % clamp so bar doesn't go negative
b = bar(categorical(sp_labels, sp_labels), mean_nrmse, 'FaceColor',  [0.8 0.6 0.4], ...
    'EdgeColor', 'k', 'LineWidth', 1.2);
hold on;
errorbar(1:num_setpoints, mean_nrmse, lower_err, std_nrmse, ...
    'k', 'LineStyle', 'none', 'LineWidth', 1.5, 'CapSize', 12);
xlabel('Setpoint (a.u.)', 'FontSize', 18);
ylabel('SS NRMSE (%)', 'FontSize', 18);
title(sprintf('RBF Controller:  SS NRMSE  (mean \\pm \\sigma,  N = %d seeds)', num_seeds), ...
    'FontSize', 20, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 18);
% Add text labels on bars
for sp_i = 1:num_setpoints
    text(sp_i, mean_nrmse(sp_i) + std_nrmse(sp_i) + 2, ...
        sprintf('%.2f \\pm %.2f', mean_nrmse(sp_i), std_nrmse(sp_i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 18, 'FontWeight', 'bold');
end
savePlot(gcf, output_dir, 'MC_ss_nrmse_bar_chart');

%% ---- Figure: Median + IQR SS_NRMSE bar chart ----
figure('Color', 'w', 'Position', [100 100 700 500]);
median_nrmse = nanmedian(MC_ss_nrmse, 1);
q1_nrmse     = prctile(MC_ss_nrmse, 25, 1);
q3_nrmse     = prctile(MC_ss_nrmse, 75, 1);
lower_err    = median_nrmse - q1_nrmse;   % distance from median down to Q1
upper_err    = q3_nrmse - median_nrmse;   % distance from median up to Q3

spruce = [15 82 87] / 255;   % #0F5257

b = bar(categorical(sp_labels, sp_labels), median_nrmse, ...
    'FaceColor', spruce, ...
    'EdgeColor', 'k', 'LineWidth', 1.2, ...
    'FaceAlpha', 0.85);
hold on;
errorbar(1:num_setpoints, median_nrmse, lower_err, upper_err, ...
    'k', 'LineStyle', 'none', 'LineWidth', 1.5, 'CapSize', 12);
xlabel('Setpoint (a.u.)', 'FontSize', 18);
ylabel('SS NRMSE (%)', 'FontSize', 18);
title(sprintf('RBF Controller:  SS NRMSE  (median [Q1, Q3],  N = %d seeds)', num_seeds), ...
    'FontSize', 20, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 18);

% Add text labels on bars
for sp_i = 1:num_setpoints
    text(sp_i, q3_nrmse(sp_i) + 2, ...
        sprintf('%.2f [%.2f, %.2f]', median_nrmse(sp_i), q1_nrmse(sp_i), q3_nrmse(sp_i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 16, 'FontWeight', 'bold');
end
hold off;


%% ---- Figure: Box + Swarm SS_NRMSE per setpoint ----
figure('Color', 'w', 'Position', [100 100 800 550]);
hold on;
[n_seeds, n_sp] = size(MC_ss_nrmse);

% ---- palette ----
dusky_blue = [70 100 140] / 255;
iqr_color  = [0.2 0.2 0.2];
whisker_color = [0.45 0.45 0.45];

% ---- geometry ----
box_hw   = 0.12;   % box half-width
cap_hw   = 0.08;   % whisker cap half-width
jit_w    = 0.25;   % swarm jitter width

for sp_i = 1:n_sp
    vals = MC_ss_nrmse(:, sp_i);
    vals = vals(~isnan(vals));
    if numel(vals) < 3, continue; end

    med = median(vals);
    q1  = prctile(vals, 25);
    q3  = prctile(vals, 75);
    iqr_val = q3 - q1;
    whi_lo  = max(min(vals), q1 - 1.5 * iqr_val);
    whi_hi  = min(max(vals), q3 + 1.5 * iqr_val);

    % --- swarm layer (behind box) ---
    swarmchart(sp_i * ones(size(vals)), vals, 18, dusky_blue, 'filled', ...
        'MarkerFaceAlpha', 0.35, 'XJitterWidth', jit_w);

    % --- whiskers ---
    plot([sp_i sp_i], [whi_lo q1], '-', 'Color', whisker_color, 'LineWidth', 1.2);
    plot([sp_i sp_i], [q3 whi_hi], '-', 'Color', whisker_color, 'LineWidth', 1.2);
    plot(sp_i + [-cap_hw cap_hw], [whi_lo whi_lo], '-', 'Color', whisker_color, 'LineWidth', 1.2);
    plot(sp_i + [-cap_hw cap_hw], [whi_hi whi_hi], '-', 'Color', whisker_color, 'LineWidth', 1.2);

    % --- box (IQR) ---
    rectangle('Position', [sp_i - box_hw, q1, 2*box_hw, iqr_val], ...
        'EdgeColor', iqr_color, 'LineWidth', 1.8, 'FaceColor', 'none');

    % --- median line ---
    plot(sp_i + [-box_hw box_hw], [med med], '-', 'Color', iqr_color, 'LineWidth', 2.5);

    % --- median annotation ---
    text(sp_i + box_hw + 0.08, med, sprintf('%.2f', med), ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', ...
        'FontSize', 12, 'FontWeight', 'bold');
end

xticks(1:n_sp);
xticklabels(sp_labels);
xlim([0.5, n_sp + 0.5]);
xlabel('Setpoint (a.u.)', 'FontSize', 18);
ylabel('SS NRMSE (%)',    'FontSize', 18);
title(sprintf('RBF Controller:  SS NRMSE distribution  (N = %d seeds)', n_seeds), ...
    'FontSize', 20, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 18, 'GridAlpha', 0.15);
%ylim([0, max(MC_ss_nrmse(:), [], 'omitnan') * 1.10]);
ylim([0, 40]);
ylim([0, 85]); % good for full ablation
hold off;

%% ---- Figure: Mean ± σ for all metrics ----
figure('Color', 'w', 'Position', [120 120 1800 450]);
sgtitle(sprintf('RBF Monte Carlo Summary  (N = %d seeds)', num_seeds), ...
    'FontSize', 20, 'FontWeight', 'bold');

metric_names  = {'Rise Time (h)', 'Overshoot (%)', 'SS Mean Error', 'SS RMSE', 'SS NRMSE (%)'};
metric_arrays = {MC_rise_time, MC_overshoot, MC_ss_mean_error, MC_ss_rmse, MC_ss_nrmse};
bar_colors    = {[0.3 0.6 0.9], [0.9 0.5 0.3], [0.4 0.7 0.7], [0.7 0.5 0.8], [0.8 0.6 0.4]};

for mi = 1:5
    subplot(1, 5, mi);
    m_vals = nanmean(metric_arrays{mi}, 1);
    s_vals = nanstd(metric_arrays{mi}, 0, 1);
    bar(categorical(sp_labels, sp_labels), m_vals, 'FaceColor', bar_colors{mi}, ...
        'EdgeColor', 'k', 'LineWidth', 0.8);
    hold on;
    errorbar(1:num_setpoints, m_vals, s_vals, s_vals, ...
        'k', 'LineStyle', 'none', 'LineWidth', 1.2, 'CapSize', 8);
    xlabel('Setpoint (a.u.)', 'FontSize', 18);
    title(metric_names{mi}, 'FontSize', 22);
    grid on;
    set(gca, 'FontSize', 18);
end

savePlot(gcf, output_dir, 'MC_all_metrics_bar_chart');

%% ---- CSV output: per-seed metrics ----
csv_file = fullfile(output_dir, 'MC_per_seed_metrics.csv');
fid = fopen(csv_file, 'w');
fprintf(fid, 'seed');
for sp_i = 1:num_setpoints
    sp_str = sprintf('%g', setpoint_values(sp_i));
    fprintf(fid, ',rise_hr_sp%s,os_pct_sp%s,ss_me_sp%s,ss_rmse_sp%s,ss_nrmse_pct_sp%s,lock_hr_sp%s', ...
        sp_str, sp_str, sp_str, sp_str, sp_str, sp_str);
end
fprintf(fid, '\n');

for s = 1:num_seeds
    fprintf(fid, '%d', seed_list(s));
    for sp_i = 1:num_setpoints
        fprintf(fid, ',%.4f,%.4f,%.6f,%.6f,%.4f,%.2f', ...
            MC_rise_time(s, sp_i), MC_overshoot(s, sp_i), ...
            MC_ss_mean_error(s, sp_i), MC_ss_rmse(s, sp_i), ...
            MC_ss_nrmse(s, sp_i), MC_band_lock_hr(s, sp_i));
    end
    fprintf(fid, '\n');
end

% Summary rows
fprintf(fid, 'mean');
for sp_i = 1:num_setpoints
    fprintf(fid, ',%.4f,%.4f,%.6f,%.6f,%.4f,%.2f', ...
        nanmean(MC_rise_time(:,sp_i)), nanmean(MC_overshoot(:,sp_i)), ...
        nanmean(MC_ss_mean_error(:,sp_i)), nanmean(MC_ss_rmse(:,sp_i)), ...
        nanmean(MC_ss_nrmse(:,sp_i)), nanmean(MC_band_lock_hr(:,sp_i)));
end
fprintf(fid, '\n');

fprintf(fid, 'std');
for sp_i = 1:num_setpoints
    fprintf(fid, ',%.4f,%.4f,%.6f,%.6f,%.4f,%.2f', ...
        nanstd(MC_rise_time(:,sp_i)), nanstd(MC_overshoot(:,sp_i)), ...
        nanstd(MC_ss_mean_error(:,sp_i)), nanstd(MC_ss_rmse(:,sp_i)), ...
        nanstd(MC_ss_nrmse(:,sp_i)), nanstd(MC_band_lock_hr(:,sp_i)));
end
fprintf(fid, '\n');
fclose(fid);
fprintf('\nCSV saved to: %s\n', csv_file);

fprintf('\nDone. Results in: %s\n', output_dir);

%% ========================================================================
%                              FUNCTIONS
%% ========================================================================

function dydt_min = eng_time_varying_ode_EF_MINUTES_PERT( ...
         t_min, y, EF_time_min, gammaext_T4_time_min, M_time_min, alpha1_func_min, pert, NOM)

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

    n = 2;

    alpha = NOM.alpha * (1 + pert.alpha_scale);
    K     = NOM.K     * (1 + pert.K_scale);
    gamma1= NOM.gamma1* (1 + pert.gamma1_scale);

    N = 10;  a=0.00002*N;
    U = zeros(N,1);  U(1) = u_f;

    EFv = EF_base;
    du_f    = alpha1 * EFv^n / (K^n + EFv^n) - gamma1*u_f;
    du_hist = (-a*eye(N) + a*diag(ones(1,N-1),-1)) * u_hist + a*U;
    dNKX21  = alpha*u_hist(end) - 0.01*nkx21 + 0.25;

    kappa = NOM.kappa * (1 + pert.kappa_scale);
    K2    = NOM.K2    * (1 + pert.K2_scale);
    k_1   = kappa * ((nkx21/K2)^3 / (1 + (nkx21/K2)^3));

    alpha_TG = 1;  alpha_I = 1;

    gamma_I     = NOM.gammaI  * (1 + pert.gammaI_scale);
    gamma_Tg    = NOM.gammaTg * (1 + pert.gammaTg_scale);
    gammaint_T4 = NOM.gint    * (1 + pert.gint_scale);

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
end

function pert = neutral_perturbation(d4_period_min)
    pert.alpha_scale   = 0;  pert.K_scale      = 0;  pert.gamma1_scale = 0;
    pert.kappa_scale   = 0;  pert.K2_scale     = 0;  pert.gammaI_scale = 0;
    pert.gammaTg_scale = 0;  pert.gint_scale   = 0;  pert.gext_scale   = 0;
    pert.M_scale       = 0;  pert.alpha1_scale = 0;
    pert.d4_bias = 0;  pert.d4_amp = 0;  pert.d4_phase = 0;
    pert.d4_period_min = d4_period_min;
    pert.act_gain = 0;  pert.delay_min = 0;  pert.jitter_min = 0;
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


