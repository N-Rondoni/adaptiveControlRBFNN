clear all; close all; clc;

%% ========================================================================
%  RBF CONTROLLER — NODE COUNT (N) SCALING SWEEP
%
%  Companion to RBF_timing_study.m, using the SAME measurement method, so
%  that the N = 40 point of this sweep is directly comparable to the table
%  in the empirical timing subsection.
%
%  METHOD (identical to the timing study)
%  Cost per controller update, measured in place inside the closed loop.
%  Each update spans two code regions — the activation block and the
%  adaptation/output block, with the plant step between them — so two
%  intervals are timed and summed per update, with one tic/toc pair cost
%  subtracted for each.
%
%  WARM-UP
%  One full length simulation is run and discarded before measurement. It is
%  executed at the LARGEST node count in the sweep, not the nominal one: the
%  activation store ED is len x N, so warming at max(N_values) maps the
%  largest heap the sweep will need and every smaller allocation that
%  follows reuses it. A warm-up at the nominal N would leave the large N
%  points paying first touch page faults.
%
%  ROBUST STATISTIC
%  Panel fits use the MEDIAN per-update cost, not the mean. A single
%  operating system scheduling event was enough to move a 97 sample mean by
%  6.6 us in the timing study; the median is unaffected. Means and maxima
%  are still reported alongside.
%
%  WEIGHT SCALING
%  The output is u = sum_j W_j phi_j, so a fixed per-node initial value and
%  clip make the initial control scale with N. Both are scaled by
%  N_nominal/Nc — the "Nc scaled" convention of the parameter sensitivity
%  study — which holds the reachable output range constant across the sweep
%  and reduces exactly to the main configuration at Nc = N_nominal.
%
%  SCOPE
%  Node counts away from the nominal characterize COST ONLY. Closed loop
%  performance at those configurations is not evaluated and is not
%  meaningful; the controller is not retuned for them.
%
%  MEMORY
%  ED is len x N: ~4.3 MB per node at tf = 4500 min, so N = 160 needs
%  ~690 MB. This caps N_values.
%
%  Run on an otherwise idle machine with no parallel pool.
%% ========================================================================

%% ==================== CONFIGURATION ====================
CFG.N_values  = [10, 20, 40, 80, 160];
CFG.N_nominal = 40;
CFG.setpoint  = 20;              % single setpoint; matches the sensitivity study

CFG.dt        = 0.1/12;          % ~0.5 s in minutes
CFG.ti        = 0;
CFG.tf        = 4500;            % minutes — matches RBF_timing_study.m
CFG.T_measure = 20;
CFG.Ts        = 120;
CFG.r_duty    = 0.60;

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

CFG.alpha_y = 0.30;
CFG.meas_enable = false;

DO_SAVE_PDF = false;
output_dir  = fullfile(pwd, 'results_N_sweep');
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

%% ==================== Derived setup ====================
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
CFG.pert = neutral_perturbation(6*60);

fprintf('\nN sweep: N = %s | setpoint %g | tf %g min | d = %d\n', ...
    mat2str(CFG.N_values), CFG.setpoint, CFG.tf, CFG.d_reg);
fprintf('ED footprint at largest N: %.0f MB\n', CFG.len*max(CFG.N_values)*8/1e6);

%% ==================== WARM-UP (largest N, discarded) ====================
N_warm = max(CFG.N_values);
fprintf('\n--- Warm-up run (N = %d, largest in sweep), results discarded ---\n', N_warm);
warm = simulate_run(N_warm, CFG);
fprintf('    warm-up: sim %.2f s | adapting median %.2f us (discarded)\n', ...
    warm.sim_wall_s, warm.adapt_median*1e6);
clear warm;

%% ==================== SWEEP ====================
num_N = numel(CFG.N_values);
results = struct([]);

for k = 1:num_N
    Nc = CFG.N_values(k);
    fprintf('\n--- N = %d (%d of %d) ---\n', Nc, k, num_N);
    R = simulate_run(Nc, CFG);
    fprintf('    sim %.2f s | ode15s %.2f s (%.2f%%) | ctrl %.3f ms\n', ...
        R.sim_wall_s, R.ode_wall_s, 100*R.ode_wall_s/R.sim_wall_s, R.ctrl_wall_s*1e3);
    fprintf('    adapting  %3d: mean %.2f us | median %.2f us | max %.2f us\n', ...
        R.n_adapt, R.adapt_mean*1e6, R.adapt_median*1e6, R.adapt_max*1e6);
    fprintf('    post-lock %3d: mean %.2f us | median %.2f us | max %.2f us\n', ...
        R.n_frozen, R.frozen_mean*1e6, R.frozen_median*1e6, R.frozen_max*1e6);
    if isempty(results), results = R; else, results(k) = R; end %#ok<AGROW>
end

%% ==================== AFFINE FIT ====================
% Equation (complexity_total) is affine in N, not proportional, so the model
% fitted is t = a + b*N. A log-log gradient is NOT reported: the fixed
% d + 14 terms make the local slope approach unity only asymptotically, and
% over N = 10..160 it would read below 1 for reasons unrelated to the method.
% Medians are fitted; see the header note on robustness.

Nv       = [results.N];
t_adapt  = [results.adapt_median]  * 1e6;   % us
t_frozen = [results.frozen_median] * 1e6;   % us

fit_adapt = polyfit(Nv, t_adapt, 1);
r_a       = t_adapt - polyval(fit_adapt, Nv);
R2_adapt  = 1 - sum(r_a.^2) / sum((t_adapt - mean(t_adapt)).^2);

vf = ~isnan(t_frozen);
if nnz(vf) >= 3
    fit_frozen = polyfit(Nv(vf), t_frozen(vf), 1);
    r_f        = t_frozen(vf) - polyval(fit_frozen, Nv(vf));
    R2_frozen  = 1 - sum(r_f.^2) / sum((t_frozen(vf) - mean(t_frozen(vf))).^2);
else
    fit_frozen = [NaN NaN];  R2_frozen = NaN;
end

%% ==================== REPORT ====================
fprintf('\n');
fprintf('=========================================================================================================\n');
fprintf('  NODE COUNT SCALING — cost per controller update, timed in place\n');
fprintf('  setpoint %g | tf %g min | dt %g min | d = %d | warm-up at N = %d, discarded\n', ...
    CFG.setpoint, CFG.tf, CFG.dt, CFG.d_reg, N_warm);
fprintf('=========================================================================================================\n');
fprintf('%6s | %7s | %10s | %11s | %10s | %7s | %11s | %11s | %8s\n', ...
    'N', 'n_adapt', 'adapt med', 'adapt mean', 'adapt max', 'n_froz', 'froz med', 'froz mean', 'sim (s)');
fprintf('---------------------------------------------------------------------------------------------------------\n');
for k = 1:num_N
    R = results(k);
    fprintf('%6d | %7d | %7.2f us | %8.2f us | %7.2f us | %7d | %8.2f us | %8.2f us | %8.1f\n', ...
        R.N, R.n_adapt, R.adapt_median*1e6, R.adapt_mean*1e6, R.adapt_max*1e6, ...
        R.n_frozen, R.frozen_median*1e6, R.frozen_mean*1e6, R.sim_wall_s);
end
fprintf('=========================================================================================================\n');
fprintf('\nAffine fit  t = a + b*N  on median per-update cost\n');
fprintf('  adapting : b = %.4f us/node, a = %.3f us, R^2 = %.4f\n', ...
    fit_adapt(1), fit_adapt(2), R2_adapt);
if ~isnan(fit_frozen(1))
    fprintf('  post-lock: b = %.4f us/node, a = %.3f us, R^2 = %.4f\n', ...
        fit_frozen(1), fit_frozen(2), R2_frozen);
end
fprintf('\n  FOR THE MANUSCRIPT: %.4f us per node, intercept %.3f us, R^2 = %.4f\n', ...
    fit_adapt(1), fit_adapt(2), R2_adapt);

%% ============ LOAD SAVED SWEEP (replot without re-running) ==============
%  During a normal full-script run this cell does nothing, because results
%  already exists in the workspace.
%
%  To iterate on the figure after a `clear all` or a fresh MATLAB start:
%  Ctrl+Enter this cell, then Ctrl+Enter the FIGURE cell below. It loads the
%  most recent saved sweep and rebuilds exactly the variables the figure
%  needs — about a second, rather than twelve minutes of re-simulation.
if ~exist('results','var') || isempty(results)

    replot_dir = fullfile(pwd, 'results_N_sweep');
    dlist = dir(fullfile(replot_dir, 'RBF_N_sweep_*.mat'));
    if isempty(dlist)
        error('No saved sweep found in %s — run the script first.', replot_dir);
    end
    [~, newest] = max([dlist.datenum]);
    S = load(fullfile(replot_dir, dlist(newest).name));
    fprintf('Replotting from %s\n', dlist(newest).name);

    results    = S.results;
    CFG        = S.CFG;
    fit_adapt  = S.fit_adapt;    R2_adapt  = S.R2_adapt;
    fit_frozen = S.fit_frozen;   R2_frozen = S.R2_frozen;

    output_dir = replot_dir;
    if ~exist('DO_SAVE_PDF','var'), DO_SAVE_PDF = false; end

    Nv       = [results.N];
    t_adapt  = [results.adapt_median]  * 1e6;
    t_frozen = [results.frozen_median] * 1e6;
    vf       = ~isnan(t_frozen);
    num_N    = numel(Nv);
end

%% ==================== FIGURE ====================
figure('Color','w','Position',[150 150 900 650]); hold on;

Nfit = linspace(0, max(Nv)*1.05, 100);
plot(Nfit, polyval(fit_adapt, Nfit), '--', 'Color', [0.4 0.4 0.4], ...
    'LineWidth', 1.8, 'DisplayName', sprintf('Affine fit, R^2 = %.4f', R2_adapt));

plot(Nv, t_adapt, 'o', 'MarkerSize', 12, 'LineWidth', 2.0, ...
    'MarkerFaceColor', [0.2 0.45 0.8], 'Color', [0.2 0.45 0.8], ...
    'DisplayName', 'Adapting update');

if any(vf)
    plot(Nv(vf), t_frozen(vf), 's', 'MarkerSize', 12, 'LineWidth', 2.0, ...
        'MarkerFaceColor', [0.85 0.55 0.2], 'Color', [0.85 0.55 0.2], ...
        'DisplayName', 'Post band lock update');
end

xline(CFG.N_nominal, ':k', 'LineWidth', 1.8, ...
    'Label', sprintf('N = %d (this work)', CFG.N_nominal), ...
    'FontSize', 16, 'LabelVerticalAlignment', 'top', ...
    'LabelHorizontalAlignment', 'right', 'HandleVisibility', 'off');

xlabel('RBF nodes N', 'FontSize', 20);
ylabel('Wall clock per update (\mus)', 'FontSize', 20);
title('Controller update cost against node count', 'FontSize', 20, 'FontWeight', 'bold');
legend('Location','southeast','FontSize',18); grid on;
set(gca, 'FontSize', 18);
xlim([0 max(Nv)*1.05]);

if DO_SAVE_PDF, savePlot(gcf, output_dir, 'timing_scaling'); end

%% ==================== SAVE ====================
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
mat_file = fullfile(output_dir, sprintf('RBF_N_sweep_%s.mat', stamp));
save(mat_file, 'results', 'CFG', 'fit_adapt', 'R2_adapt', ...
     'fit_frozen', 'R2_frozen', '-v7.3');
fprintf('\nSaved: %s\n', mat_file);

txt_file = fullfile(output_dir, sprintf('RBF_N_sweep_%s.txt', stamp));
fid = fopen(txt_file, 'w');
fprintf(fid, 'RBF node count scaling sweep\n');
fprintf(fid, 'Date: %s\n', stamp);
fprintf(fid, 'Setpoint %g | tf %g min | dt %g min | T_measure %g min | d = %d\n', ...
    CFG.setpoint, CFG.tf, CFG.dt, CFG.T_measure, CFG.d_reg);
fprintf(fid, 'Nominal plant, measurement noise and perturbations disabled\n');
fprintf(fid, 'Weight init and clip scaled by %d/Nc ("Nc scaled" convention)\n', CFG.N_nominal);
fprintf(fid, 'Warm-up at N = %d executed and discarded\n', N_warm);
fprintf(fid, 'tic/toc pair cost: %.3f us (two subtracted per update sample)\n\n', ...
    CFG.tictoc_pair_s*1e6);
fprintf(fid, '%6s | %7s | %10s | %11s | %10s | %7s | %11s | %11s | %8s\n', ...
    'N', 'n_adapt', 'adapt med', 'adapt mean', 'adapt max', 'n_froz', 'froz med', 'froz mean', 'sim (s)');
fprintf(fid, '---------------------------------------------------------------------------------------------------------\n');
for k = 1:num_N
    R = results(k);
    fprintf(fid, '%6d | %7d | %7.2f us | %8.2f us | %7.2f us | %7d | %8.2f us | %8.2f us | %8.1f\n', ...
        R.N, R.n_adapt, R.adapt_median*1e6, R.adapt_mean*1e6, R.adapt_max*1e6, ...
        R.n_frozen, R.frozen_median*1e6, R.frozen_mean*1e6, R.sim_wall_s);
end
fprintf(fid, '---------------------------------------------------------------------------------------------------------\n');
fprintf(fid, '\nAffine fit t = a + b*N on median per-update cost\n');
fprintf(fid, '  adapting : b = %.4f us/node, a = %.3f us, R^2 = %.4f\n', ...
    fit_adapt(1), fit_adapt(2), R2_adapt);
fprintf(fid, '  post-lock: b = %.4f us/node, a = %.3f us, R^2 = %.4f\n', ...
    fit_frozen(1), fit_frozen(2), R2_frozen);
fprintf(fid, '\nNode counts other than %d characterize cost only; closed loop\n', CFG.N_nominal);
fprintf(fid, 'performance at those configurations is not evaluated.\n');
fclose(fid);
fprintf('Saved: %s\n', txt_file);

%% ========================================================================
%                              FUNCTIONS
%  The simulation core below mirrors RBF_timing_study.m, with Nc as an
%  argument and the weight scaling described in the header. If one is
%  edited, edit both.
%% ========================================================================

function R = simulate_run(Nc, CFG)

    len = CFG.len;
    d   = CFG.d_reg;
    tv  = CFG.time_vector;
    pert = CFG.pert;
    T4_des = CFG.setpoint;

    % ---- Network: "Nc scaled" weight convention ----
    W_max   = (T4_des/1000 + 0.005);
    W_max_N = W_max * (CFG.N_nominal / Nc);
    c = repmat(linspace(0,1,Nc), [d, 1]);
    W = 0.5 * W_max_N * ones(1, Nc);

    % ---- State ----
    y_cur = [229.437253584811; 0.273804879029534; ...
             22.9437253584891; 4.10707318544213; ...
             zeros(11,1); 25];

    ref = T4_des;  x = 0;  e_int = 0;  u_hold = 0;

    new_in = zeros(d, len);
    new_in(1,:) = ref;

    ED = zeros(len, Nc);

    A_hold = 0;  A_basal = 0;
    band_entered = false;  inband_ct = 0;  band_width = CFG.BAND_pct * T4_des;
    band_lock_time = NaN;
    amp_times = CFG.ti:CFG.Ts:(CFG.tf + CFG.Ts);
    m_amp = 1;  next_window = amp_times(1);

    T4_hold = 0;
    meas_times = CFG.ti:CFG.T_measure:(CFG.tf + CFG.T_measure);
    m_T4 = 1;  next_meas = meas_times(1);
    y_filt = NaN;

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
                W = clamp(W, -W_max_N, W_max_N);

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

    upd_samples  = upd_samples(1:n_upd);
    upd_adapting = upd_adapting(1:n_upd);
    sa = upd_samples(upd_adapting);
    sf = upd_samples(~upd_adapting);

    R.N            = Nc;
    R.W_max_N      = W_max_N;
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
    R.upd_samples_s = upd_samples;
    R.upd_adapting  = upd_adapting;

    R.adapt_mean   = stat_or_nan(sa, @mean);
    R.adapt_median = stat_or_nan(sa, @median);
    R.adapt_max    = stat_or_nan(sa, @max);
    R.frozen_mean  = stat_or_nan(sf, @mean);
    R.frozen_median= stat_or_nan(sf, @median);
    R.frozen_max   = stat_or_nan(sf, @max);
end

%% ---- Plant ODE (minutes) -----------------------------------------------
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

%% ---- EF burst helper ---------------------------------------------------
function ef = ef_avg_burst_in_window_delay(t, T_on, T_cycle, T_total, t5, t6, A, pulseDuty, delay_min, jitter_min)
    t_eff = t - delay_min + jitter_min;
    if t_eff < 0,       ef = 0; return; end
    if t_eff > T_total, ef = 0; return; end

    tau_win = mod(t_eff, T_cycle);
    if tau_win >= T_on, ef = 0; return; end

    xi = mod(tau_win, t5 + t6);
    ef = A * pulseDuty * double(xi < t5);
end

%% ---- Nominal plant -----------------------------------------------------
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