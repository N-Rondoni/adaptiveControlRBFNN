clear all; close all; clc;

%% ========================================================================
%  RBF CONTROLLER — TIMING STUDY
%
%  Produces the empirical timing numbers for the Computational Complexity
%  and Runtime supplement. This script does timing only: no trajectory
%  figures, no tracking metrics. Those come from RBF_no_noise.m.
%
%  METHOD
%  Mean wall clock time per controller update, measured inside the closed
%  loop, following the convention used for per-step computation time in the
%  predictive control literature. Each update spans two code regions — the
%  activation block and the adaptation/output block, with the plant step
%  between them — so two intervals are timed and summed per update. One
%  tic/toc pair cost is subtracted for each.
%
%  WARM-UP
%  One full length simulation is run and discarded before any measurement.
%  It is full length deliberately: a short warm-up would exercise the JIT
%  but would not allocate an ED array of the same size, and first-touch page
%  faults on a freshly grown heap were the dominant cost in the first
%  measured run when this warm-up was omitted (mean 126 us against 38 us
%  for the runs that followed it).
%
%  Run on an otherwise idle machine with no parallel pool.
%% ========================================================================

%% ==================== CONFIGURATION ====================
CFG.setpoints = [12, 16, 20, 25];
CFG.warm_setpoint = 20;          % discarded run, mid range

CFG.dt        = 0.1/12;          % ~0.5 s in minutes
CFG.ti        = 0;
CFG.tf        = 4500;            % minutes
CFG.T_measure = 20;              % minutes
CFG.Ts        = 120;             % amplitude hold window
CFG.r_duty    = 0.60;

CFG.Nc    = 40;
CFG.d_reg = 16;
CFG.beeta = 0.13;
CFG.ki    = 0;
CFG.e_int_max = 5;
CFG.UL = 100;  CFG.LL = 0;
CFG.eta_c = 1e-5;

CFG.thresh = 1e-4;
CFG.A_min  = 0.00;  CFG.A_max = 0.10;  CFG.kA = 0.20;
CFG.BAND_pct = 0.20;
CFG.BAND_holdN = 2;

CFG.alpha_y = 0.30;              % measurement low-pass, always active
CFG.meas_enable = false;         % noiseless configuration

output_dir = fullfile(pwd, 'results_timing_study_v3');
if ~exist(output_dir,'dir'), mkdir(output_dir); end

%% ==================== tic/toc calibration ====================
n_cal = 1e5;
t_cal = tic;
for k_cal = 1:n_cal
    t_dummy = tic;
    e_dummy = toc(t_dummy);
end
CFG.tictoc_pair_s = toc(t_cal) / n_cal;
fprintf('tic/toc pair cost: %.3f us (%d samples)\n', CFG.tictoc_pair_s*1e6, n_cal);

%% ==================== Derived timing / plant setup ====================
CFG.time_vector = CFG.ti:CFG.dt:CFG.tf;
CFG.len         = numel(CFG.time_vector);
CFG.learning_rate = 1e-6 * (CFG.T_measure / CFG.dt);

SEC_PER_MIN = 60;
toUnit = @(sec) sec / SEC_PER_MIN;

t1 = toUnit(500e-6);  t2 = toUnit(100e-6);
t3 = toUnit(500e-6);  t4 = toUnit(9.5e-4);
Tp = t1 + t2 + t3 + t4;
CFG.pulseDuty = (t1 + t3)/Tp;
CFG.t5 = 500 * Tp;
CFG.t6 = toUnit(0.5);
CFG.T_on    = CFG.r_duty * CFG.Ts;
CFG.T_cycle = CFG.Ts;
CFG.T_total = CFG.tf;

rng(1);
CFG.pert = neutral_perturbation(6*60);   % nominal plant

fprintf('Timing study: %d setpoints | tf %g min | dt %g min | N = %d, d = %d\n', ...
    numel(CFG.setpoints), CFG.tf, CFG.dt, CFG.Nc, CFG.d_reg);
fprintf('ED footprint per run: %.0f MB\n', CFG.len*CFG.Nc*8/1e6);

%% ==================== WARM-UP (discarded) ====================
fprintf('\n--- Warm-up run (setpoint %g), results discarded ---\n', CFG.warm_setpoint);
warm = simulate_run(CFG.warm_setpoint, CFG);
fprintf('    warm-up: sim %.2f s | adapting mean %.2f us (discarded)\n', ...
    warm.sim_wall_s, warm.adapt_mean*1e6);
clear warm;

%% ==================== MEASURED RUNS ====================
nsp = numel(CFG.setpoints);
results = struct([]);

for k = 1:nsp
    sp = CFG.setpoints(k);
    fprintf('\n--- Setpoint %g (%d of %d) ---\n', sp, k, nsp);
    R = simulate_run(sp, CFG);
    fprintf('    sim %.2f s | ode15s %.2f s (%.2f%%) | ctrl %.3f ms | RTF %.0fx\n', ...
        R.sim_wall_s, R.ode_wall_s, 100*R.ode_wall_s/R.sim_wall_s, ...
        R.ctrl_wall_s*1e3, R.rtf);
    fprintf('    adapting  %3d updates: mean %.2f us | median %.2f us | max %.2f us\n', ...
        R.n_adapt, R.adapt_mean*1e6, R.adapt_median*1e6, R.adapt_max*1e6);
    fprintf('    post-lock %3d updates: mean %.2f us | median %.2f us | max %.2f us\n', ...
        R.n_frozen, R.frozen_mean*1e6, R.frozen_median*1e6, R.frozen_max*1e6);
    if isempty(results), results = R; else, results(k) = R; end %#ok<AGROW>
end

%% ==================== REPORT ====================
fprintf('\n');
fprintf('=====================================================================================================\n');
fprintf('  TIMING STUDY — mean wall clock per controller update, measured in the closed loop\n');
fprintf('  N = %d | d = %d | dt = %g min | T_measure = %g min | tf = %g min\n', ...
    CFG.Nc, CFG.d_reg, CFG.dt, CFG.T_measure, CFG.tf);
fprintf('=====================================================================================================\n');
fprintf('%8s | %8s | %10s | %10s | %9s | %8s | %10s | %9s | %8s\n', ...
    'setpoint', 'n_adapt', 'adapt mean', 'adapt max', 'n_froz', 'froz mean', 'sim (s)', 'ode (%)', 'RTF');
fprintf('-----------------------------------------------------------------------------------------------------\n');
for k = 1:nsp
    R = results(k);
    fprintf('%8g | %8d | %7.2f us | %7.2f us | %9d | %5.2f us | %10.2f | %8.2f%% | %8.0f\n', ...
        R.setpoint, R.n_adapt, R.adapt_mean*1e6, R.adapt_max*1e6, ...
        R.n_frozen, R.frozen_mean*1e6, R.sim_wall_s, ...
        100*R.ode_wall_s/R.sim_wall_s, R.rtf);
end
fprintf('=====================================================================================================\n');

% ---- Pooled across setpoints: the numbers the manuscript reports ----
pool_adapt  = [];  pool_frozen = [];
for k = 1:nsp
    s = results(k).upd_samples_s;
    a = results(k).upd_adapting;
    pool_adapt  = [pool_adapt,  s(a)];   %#ok<AGROW>
    pool_frozen = [pool_frozen, s(~a)];  %#ok<AGROW>
end

SUMMARY.adapt_mean   = mean(pool_adapt);
SUMMARY.adapt_median = median(pool_adapt);
SUMMARY.adapt_max    = max(pool_adapt);
SUMMARY.adapt_n      = numel(pool_adapt);
SUMMARY.frozen_mean  = mean(pool_frozen);
SUMMARY.frozen_median= median(pool_frozen);
SUMMARY.frozen_max   = max(pool_frozen);
SUMMARY.frozen_n     = numel(pool_frozen);
SUMMARY.sim_mean_s   = mean([results.sim_wall_s]);
SUMMARY.sim_min_s    = min([results.sim_wall_s]);
SUMMARY.sim_max_s    = max([results.sim_wall_s]);
SUMMARY.ode_pct      = 100 * mean([results.ode_wall_s] ./ [results.sim_wall_s]);
SUMMARY.rtf_mean     = mean([results.rtf]);
SUMMARY.T_measure_s  = CFG.T_measure * 60;
SUMMARY.margin_orders = log10(SUMMARY.T_measure_s / SUMMARY.adapt_mean);

fprintf('\n  POOLED OVER ALL %d MEASURED RUNS\n', nsp);
fprintf('    adapting  (%d updates): mean %.2f us | median %.2f us | max %.2f us\n', ...
    SUMMARY.adapt_n, SUMMARY.adapt_mean*1e6, SUMMARY.adapt_median*1e6, SUMMARY.adapt_max*1e6);
fprintf('    post-lock (%d updates): mean %.2f us | median %.2f us | max %.2f us\n', ...
    SUMMARY.frozen_n, SUMMARY.frozen_mean*1e6, SUMMARY.frozen_median*1e6, SUMMARY.frozen_max*1e6);
fprintf('    one simulation: %.1f s (range %.1f to %.1f)\n', ...
    SUMMARY.sim_mean_s, SUMMARY.sim_min_s, SUMMARY.sim_max_s);
fprintf('    numerical integration: %.2f%% of loop time\n', SUMMARY.ode_pct);
fprintf('    real time factor: %.0fx\n', SUMMARY.rtf_mean);
fprintf('\n    Adapting update %.2f us against T_measure = %.0f s\n', ...
    SUMMARY.adapt_mean*1e6, SUMMARY.T_measure_s);
fprintf('    Margin: %.1f orders of magnitude\n', SUMMARY.margin_orders);

%% ==================== SAVE ====================
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
mat_file = fullfile(output_dir, sprintf('RBF_timing_study_%s.mat', stamp));
save(mat_file, 'results', 'SUMMARY', 'CFG', '-v7.3');
fprintf('\nSaved: %s\n', mat_file);

txt_file = fullfile(output_dir, sprintf('RBF_timing_study_%s.txt', stamp));
fid = fopen(txt_file, 'w');
fprintf(fid, 'RBF controller timing study\n');
fprintf(fid, 'Date: %s\n', stamp);
fprintf(fid, 'Configuration: noiseless, nominal plant, measurement model disabled\n');
fprintf(fid, 'Measurement low-pass active, alpha_y = %g\n', CFG.alpha_y);
fprintf(fid, 'N = %d | d = %d | dt = %g min | T_measure = %g min | tf = %g min\n', ...
    CFG.Nc, CFG.d_reg, CFG.dt, CFG.T_measure, CFG.tf);
fprintf(fid, 'Setpoints: %s\n', mat2str(CFG.setpoints));
fprintf(fid, 'tic/toc pair cost: %.3f us (two subtracted per update sample)\n', CFG.tictoc_pair_s*1e6);
fprintf(fid, 'One full length warm-up run at setpoint %g was executed and discarded\n\n', CFG.warm_setpoint);
fprintf(fid, '%8s | %8s | %10s | %10s | %9s | %8s | %10s | %9s | %8s\n', ...
    'setpoint', 'n_adapt', 'adapt mean', 'adapt max', 'n_froz', 'froz mean', 'sim (s)', 'ode (%)', 'RTF');
fprintf(fid, '-----------------------------------------------------------------------------------------------------\n');
for k = 1:nsp
    R = results(k);
    fprintf(fid, '%8g | %8d | %7.2f us | %7.2f us | %9d | %5.2f us | %10.2f | %8.2f%% | %8.0f\n', ...
        R.setpoint, R.n_adapt, R.adapt_mean*1e6, R.adapt_max*1e6, ...
        R.n_frozen, R.frozen_mean*1e6, R.sim_wall_s, ...
        100*R.ode_wall_s/R.sim_wall_s, R.rtf);
end
fprintf(fid, '-----------------------------------------------------------------------------------------------------\n');
fprintf(fid, '\nPooled over all %d measured runs\n', nsp);
fprintf(fid, '  adapting  (%d updates): mean %.2f us | median %.2f us | max %.2f us\n', ...
    SUMMARY.adapt_n, SUMMARY.adapt_mean*1e6, SUMMARY.adapt_median*1e6, SUMMARY.adapt_max*1e6);
fprintf(fid, '  post-lock (%d updates): mean %.2f us | median %.2f us | max %.2f us\n', ...
    SUMMARY.frozen_n, SUMMARY.frozen_mean*1e6, SUMMARY.frozen_median*1e6, SUMMARY.frozen_max*1e6);
fprintf(fid, '  one simulation: %.1f s (range %.1f to %.1f)\n', ...
    SUMMARY.sim_mean_s, SUMMARY.sim_min_s, SUMMARY.sim_max_s);
fprintf(fid, '  numerical integration: %.2f%% of loop time\n', SUMMARY.ode_pct);
fprintf(fid, '  real time factor: %.0f x\n', SUMMARY.rtf_mean);
fprintf(fid, '  margin against T_measure = %.0f s: %.1f orders of magnitude\n', ...
    SUMMARY.T_measure_s, SUMMARY.margin_orders);
fclose(fid);
fprintf('Saved: %s\n', txt_file);

%% ========================================================================
%                              FUNCTIONS
%% ========================================================================

%% ---- One simulation, fully instrumented --------------------------------
%  The warm-up and every measured run call this same function, so the
%  discarded run exercises exactly the code that is later timed.
function R = simulate_run(T4_des, CFG)

    len  = CFG.len;
    Nc   = CFG.Nc;
    d    = CFG.d_reg;
    tv   = CFG.time_vector;
    pert = CFG.pert;

    % ---- Network ----
    W_max = (T4_des/1000 + 0.005);
    c     = repmat(linspace(0,1,Nc), [d, 1]);
    W     = 0.5 * W_max * ones(1, Nc);

    % ---- State ----
    y_cur = [229.437253584811; 0.273804879029534; ...
             22.9437253584891; 4.10707318544213; ...
             zeros(11,1); 25];

    ref    = T4_des;
    x      = 0;
    e_int  = 0;
    u_hold = 0;

    new_in = zeros(d, len);
    new_in(1,:) = ref;

    ED = zeros(len, Nc);

    % ---- Amplitude / band lock ----
    A_hold = 0;  A_basal = 0;
    band_entered = false;  inband_ct = 0;  band_width = CFG.BAND_pct * T4_des;
    band_lock_time = NaN;
    amp_times = CFG.ti:CFG.Ts:(CFG.tf + CFG.Ts);
    m_amp = 1;  next_window = amp_times(1);

    T4_hold = 0;
    meas_times = CFG.ti:CFG.T_measure:(CFG.tf + CFG.T_measure);
    m_T4 = 1;  next_meas = meas_times(1);
    y_filt = NaN;

    % ---- Timing ----
    ode_time = 0;  n_ode = 0;
    n_upd = 0;  n_adapt = 0;  n_frozen = 0;
    max_upd = ceil(CFG.tf / CFG.T_measure) + 2;
    upd_samples  = zeros(1, max_upd);
    upd_adapting = false(1, max_upd);
    t_act = 0;

    t_run = tic;

    for i = 1 : len-1

        % ---- Activations: first timed region ----
        if tv(i+1) >= next_meas
            t_c1 = tic;

            norm_input = new_in(:,i) ./ (150 * ones(d,1));
            for j = 1:Nc
                ED(i,j) = exp(-((norm_input - c(:,j))' * (norm_input - c(:,j))) / (2*CFG.beeta^2));
            end

            t_act = toc(t_c1);
        end

        EF = u_hold;

        % ---- Amplitude hold / band lock ----
        if tv(i) >= next_window
            if EF < CFG.thresh
                A_hold = 0;
            else
                A_hold = min(max(CFG.kA*(EF - CFG.thresh), CFG.A_min), CFG.A_max);
            end

            if ~band_entered
                if abs(x - T4_des) <= band_width
                    inband_ct = inband_ct + 1;
                    if inband_ct >= CFG.BAND_holdN
                        band_entered = true;
                        A_basal = A_hold;
                        band_lock_time = tv(i);
                    end
                else
                    inband_ct = 0;
                end
            end

            m_amp = m_amp + 1;
            if m_amp <= numel(amp_times), next_window = amp_times(m_amp);
            else,                          next_window = inf;
            end
        end

        if band_entered, A_hold = max(A_hold, A_basal); end

        % ---- EF(t) ----
        if A_hold <= 0
            EF_fun = @(t_min) 0.0;
        else
            A_app = clamp(A_hold * (1 + pert.act_gain), CFG.A_min, CFG.A_max);
            EF_fun = @(t_min) ef_avg_burst_in_window_delay( ...
                t_min, CFG.T_on, CFG.T_cycle, CFG.T_total, CFG.t5, CFG.t6, ...
                A_app, CFG.pulseDuty, max(0,pert.delay_min), pert.jitter_min);
        end

        % ---- Plant step ----
        t_ode = tic;
        [~, y_tmp] = ode15s(@(t,y) plant_ode(t, y, EF_fun, pert), ...
                            [tv(i), tv(i+1)], y_cur);
        ode_time = ode_time + toc(t_ode);
        n_ode = n_ode + 1;

        y_cur = y_tmp(end,:)';
        T4_actual = y_cur(4);

        % ---- Measurement update ----
        if tv(i+1) >= next_meas

            n_upd = n_upd + 1;
            if band_entered, n_frozen = n_frozen + 1; else, n_adapt = n_adapt + 1; end

            y_raw = T4_actual;               % measurement model disabled
            if isnan(y_filt), y_filt = y_raw; end
            y_filt = (1-CFG.alpha_y)*y_filt + CFG.alpha_y*y_raw;
            x = y_filt;
            e = ref - x;
            T4_hold = x;

            % ---- Control law: second timed region ----
            t_c2 = tic;

            if ~band_entered
                e_int = clamp(e_int + e*CFG.T_measure, -CFG.e_int_max, CFG.e_int_max);

                m_sq = 1 + (e / max(ref, 1e-6))^2 + ED(i,:) * ED(i,:)';
                W = W + CFG.learning_rate * e * ED(i,:) / m_sq;
                W = clamp(W, -W_max, W_max);

                for j = 1:Nc
                    c(:,j) = c(:,j) + CFG.eta_c * e * W(j) * ED(i,j) ...
                             .* (norm_input - c(:,j)) / (CFG.beeta^2);
                end
            end

            u_hold = max(min(W * ED(i,:)' + CFG.ki*e_int, CFG.UL), CFG.LL);

            upd_samples(n_upd)  = max(t_act + toc(t_c2) - 2*CFG.tictoc_pair_s, 0);
            upd_adapting(n_upd) = ~band_entered;
            t_act = 0;

            new_in(3:end, i+1) = new_in(2:end-1, i);
            new_in(2, i+1)     = x;

            m_T4 = m_T4 + 1;
            if m_T4 <= numel(meas_times), next_meas = meas_times(m_T4);
            else,                          next_meas = inf;
            end
        else
            x = T4_hold;
            new_in(2:end, i+1) = new_in(2:end, i);
        end
    end

    sim_wall = toc(t_run);

    % ---- Pack ----
    upd_samples  = upd_samples(1:n_upd);
    upd_adapting = upd_adapting(1:n_upd);
    sa = upd_samples(upd_adapting);
    sf = upd_samples(~upd_adapting);

    R.setpoint     = T4_des;
    R.sim_wall_s   = sim_wall;
    R.ode_wall_s   = ode_time;
    R.ctrl_wall_s  = sum(upd_samples);
    R.instr_wall_s = (n_ode + 2*n_upd) * CFG.tictoc_pair_s;
    R.log_wall_s   = sim_wall - ode_time - R.ctrl_wall_s - R.instr_wall_s;
    R.rtf          = CFG.tf / (sim_wall/60);
    R.n_updates    = n_upd;
    R.n_adapt      = n_adapt;
    R.n_frozen     = n_frozen;
    R.band_lock_time = band_lock_time;
    R.T4_final     = y_cur(4);
    R.upd_samples_s  = upd_samples;
    R.upd_adapting   = upd_adapting;

    R.adapt_mean   = stat_or_nan(sa, @mean);
    R.adapt_median = stat_or_nan(sa, @median);
    R.adapt_max    = stat_or_nan(sa, @max);
    R.frozen_mean  = stat_or_nan(sf, @mean);
    R.frozen_median= stat_or_nan(sf, @median);
    R.frozen_max   = stat_or_nan(sf, @max);
end

%% ---- Plant ODE (minutes), nominal or perturbed -------------------------
function dydt_min = plant_ode(t_min, y, EF_fun, pert)

    EF_base     = EF_fun(t_min);
    gammaext_T4 = 0.01  * (1 + pert.gext_scale);
    M_fun       = 0.15  * (1 + pert.M_scale);
    alpha1      = 3.308 * (1 + pert.alpha1_scale);

    x      = y(1:4);
    u_f    = y(5);
    u_hist = y(6:15);
    nkx21  = y(16);

    alpha  = 20*8.43812 * (1 + pert.alpha_scale);
    K      = 10*0.1195  * (1 + pert.K_scale);
    gamma1 = 0.0296     * (1 + pert.gamma1_scale);
    n = 2;

    % L is the delay chain stage count — distinct from the RBF node count N
    L = 10;  a = 0.00002*L;
    U = zeros(L,1);  U(1) = u_f;

    du_f    = alpha1 * EF_base^n / (K^n + EF_base^n) - gamma1*u_f;
    du_hist = (-a*eye(L) + a*diag(ones(1,L-1),-1)) * u_hist + a*U;
    dNKX21  = alpha*u_hist(end) - 0.01*nkx21 + 0.25;

    kappa = 1    * (1 + pert.kappa_scale);
    K2    = 1000 * (1 + pert.K2_scale);
    k_1   = kappa * ((nkx21/K2)^3 / (1 + (nkx21/K2)^3));

    gamma_I     = 0.004         * (1 + pert.gammaI_scale);
    gamma_Tg    = 0.04          * (1 + pert.gammaTg_scale);
    gammaint_T4 = (0.15+0.0004) * (1 + pert.gint_scale);

    dx1 = -k_1*x(3)*x(1) - gamma_I*x(1) + 1;
    dx2 =  k_1*x(3)*x(1) - M_fun*x(2) - gammaint_T4*x(2);
    dx3 =  1 - k_1*x(3)*x(1) - gamma_Tg*x(3);

    d4  = pert.d4_bias + pert.d4_amp * sin(2*pi*(t_min/pert.d4_period_min + pert.d4_phase));
    dx4 = M_fun*x(2) - gammaext_T4*x(4) - d4;

    dydt_min = 60 * [dx1; dx2; dx3; dx4; du_f; du_hist; dNKX21];
end

%% ---- EF burst helper with delay + jitter -------------------------------
function ef = ef_avg_burst_in_window_delay(t, T_on, T_cycle, T_total, t5, t6, A, pulseDuty, delay_min, jitter_min)
    t_eff = t - delay_min + jitter_min;
    if t_eff < 0,       ef = 0; return; end
    if t_eff > T_total, ef = 0; return; end

    tau_win = mod(t_eff, T_cycle);
    if tau_win >= T_on, ef = 0; return; end

    xi = mod(tau_win, t5 + t6);
    ef = A * pulseDuty * double(xi < t5);
end

%% ---- Nominal plant, all offsets zero -----------------------------------
function pert = neutral_perturbation(d4_period_min)
    pert.alpha_scale   = 0;  pert.K_scale      = 0;  pert.gamma1_scale = 0;
    pert.kappa_scale   = 0;  pert.K2_scale     = 0;  pert.gammaI_scale = 0;
    pert.gammaTg_scale = 0;  pert.gint_scale   = 0;  pert.gext_scale   = 0;
    pert.M_scale       = 0;  pert.alpha1_scale = 0;
    pert.d4_bias = 0;  pert.d4_amp = 0;  pert.d4_phase = 0;
    pert.d4_period_min = d4_period_min;
    pert.act_gain = 0;  pert.delay_min = 0;  pert.jitter_min = 0;
end

%% ---- Utilities ---------------------------------------------------------
function v = clamp(x, xmin, xmax)
    v = min(max(x, xmin), xmax);
end

function v = stat_or_nan(x, fh)
    if isempty(x), v = NaN; else, v = fh(x); end
end