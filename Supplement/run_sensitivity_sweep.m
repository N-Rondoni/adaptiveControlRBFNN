clear all; close all; clc;
%% ========================================================================
%  RUN_SENSITIVITY_SWEEP
%  One-at-a-time controller-parameter sensitivity sweep at a single
%  setpoint (set SETPOINT below), noiseless configuration (ablation
%  'none', sigma 0, measurement model disabled), the same configuration
%  that produces manuscript Figure 3.
%
%  Panels:
%    beta        RBF kernel width (enters BOTH the activation kernel and
%                the center-adaptation denominator — one symbol, one knob)
%    gamma       weight learning rate
%    eta         center adaptation gain
%    rho_BL      band-lock half-width as a fraction of setpoint
%    Nc raw      node count, per-node weight cap held fixed
%    Nc scaled   node count, per-node cap scaled 40/Nc so that total
%                representable control authority is held fixed
%
%  The two Nc variants separate representational capacity from effective
%  control gain: u = sum_j W_j phi_j sums over nodes, so with a fixed
%  per-node cap, adding nodes raises the loop gain as a side effect.
%
%  Each parameter combination is written to its own .mat plus a small
%  companion _metrics.mat. The filename encodes the swept values, so the
%  exist() check below gives both deduplication (the nominal point is
%  shared by all six panels) and crash resume.
%
%  Nominal values, sweep grids and the per-point keys are saved to
%  sweep_spec.mat, which is all parse_sensitivity_all needs to plot.
%
%  The manuscript figure uses setpoints 12, 16, 20 and 25: run this script
%  once per setpoint, then run parse_sensitivity_all.m.
%% ========================================================================

USE_PARFOR  = false;   % 25 runs are independent; flip to true once validated
FORCE_RERUN = false;
KEEP_FULL   = true;    % false = write metrics only, skipping ~200 MB/run

SETPOINT    = 25;      % run once each for 12, 16, 20, 25

output_dir = fullfile(pwd, sprintf('results_sensitivity_sp%g', SETPOINT));
if ~exist(output_dir,'dir'), mkdir(output_dir); end

%% ==================== NOMINAL CONFIGURATION ============================
% Single source of truth for this study. Everything the sweep varies is a
% field here, and the whole struct is stored inside every saved result.

NOM.T4_des = SETPOINT;

% ---- Timing ----
NOM.dt        = 0.1/12;      % ~0.5 s expressed in minutes
NOM.ti        = 0;
NOM.tf        = 4500;        % minutes (~33.3 h)
NOM.Ts        = 120;         % amplitude hold window (minutes)
NOM.r_duty    = 0.60;
NOM.T_measure = 20;          % measurement / adaptation cadence (minutes)

% ---- RBF network ----
NOM.Nc          = 40;
NOM.beeta       = 0.13;
NOM.gamma_w     = 1e-6 * (NOM.T_measure/NOM.dt);   % = 2.4e-3
NOM.eta_c       = 1e-5;
NOM.W_max_scale = 1;
NOM.input_scale = 150;

% ---- Integral action (off) ----
NOM.ki        = 0;
NOM.e_int_max = 5;

% ---- Output saturation ----
NOM.UL = 100;  NOM.LL = 0;

% ---- Band lock ----
NOM.rho_BL = 0.20;
NOM.holdN  = 2;

% ---- EF to amplitude mapping ----
NOM.thresh = 1e-4;
NOM.A_min  = 0.00;  NOM.A_max = 0.10;  NOM.kA = 0.20;

% ---- Measurement model (disabled here) ----
NOM.meas_enable    = false;
NOM.meas_alpha_y   = 0.30;
NOM.meas_bias_true = 0;
NOM.sigma          = 0;

% ---- Plant perturbation (neutral here) ----
NOM.ablation      = 'none';
NOM.pert_seed     = 1;
NOM.d4_period_min = 6*60;

NOM.pert_range.alpha_scale   = 0.25;
NOM.pert_range.K_scale       = 0.25;
NOM.pert_range.gamma1_scale  = 0.25;
NOM.pert_range.kappa_scale   = 0.20;
NOM.pert_range.K2_scale      = 0.20;
NOM.pert_range.gammaI_scale  = 0.25;
NOM.pert_range.gammaTg_scale = 0.25;
NOM.pert_range.gint_scale    = 0.25;
NOM.pert_range.gext_scale    = 0.25;
NOM.pert_range.M_scale       = 0.25;
NOM.pert_range.alpha1_scale  = 0.25;
NOM.pert_range.d4_bias       = 0.02;
NOM.pert_range.d4_amp        = 0.03;
NOM.pert_range.act_gain      = 0.20;
NOM.pert_range.delay_min     = 10/60;
NOM.pert_range.jitter_min    = 5/60;

% ---- Logging ----
NOM.keep_weights = KEEP_FULL;   % full len x Nc weight history
NOM.verbose      = false;       % per-window chatter is noise under parfor

fprintf('Nominal: beta=%.4g  gamma=%.4g  eta=%.4g  rho_BL=%.4g  Nc=%d\n', ...
    NOM.beeta, NOM.gamma_w, NOM.eta_c, NOM.rho_BL, NOM.Nc);

%% ==================== SWEEP SPECIFICATION ==============================
% field     : which NOM field the swept value is written to
% nominal   : recorded so the parser can draw the reference line
% scale_cap : also set W_max_scale = 40/Nc (the scaled node-count variant)

s = 0;

s = s+1;
S(s).name      = 'beta';
S(s).field     = 'beeta';
S(s).values    = [0.065, 0.10, 0.13, 0.18, 0.26];
S(s).nominal   = NOM.beeta;
S(s).xlabel    = '\beta (RBF width)';
S(s).scale_cap = false;
S(s).logx      = false;

s = s+1;
S(s).name      = 'gamma';
S(s).field     = 'gamma_w';
S(s).values    = NOM.gamma_w * [0.25, 0.5, 1, 2, 4];    % 0.6e-3 ... 9.6e-3
S(s).nominal   = NOM.gamma_w;
S(s).xlabel    = '\gamma (weight LR)';
S(s).scale_cap = false;
S(s).logx      = false;

s = s+1;
S(s).name      = 'eta';
S(s).field     = 'eta_c';
S(s).values    = NOM.eta_c * [0.25, 0.5, 1, 2, 4];      % 0.25e-5 ... 4e-5
S(s).nominal   = NOM.eta_c;
S(s).xlabel    = '\eta (centre gain)';
S(s).scale_cap = false;
S(s).logx      = false;

s = s+1;
S(s).name      = 'rho_BL';
S(s).field     = 'rho_BL';
S(s).values    = [0.10, 0.15, 0.20, 0.25, 0.30];
S(s).nominal   = NOM.rho_BL;
S(s).xlabel    = '\rho_{BL} (band width)';
S(s).scale_cap = false;
S(s).logx      = false;

s = s+1;
S(s).name      = 'Nc_raw';
S(s).field     = 'Nc';
S(s).values    = [10, 20, 40, 80, 160];
S(s).nominal   = NOM.Nc;
S(s).xlabel    = 'N_c (fixed per-node cap)';
S(s).scale_cap = false;
S(s).logx      = true;

s = s+1;
S(s).name      = 'Nc_scaled';
S(s).field     = 'Nc';
S(s).values    = [10, 20, 40, 80, 160];
S(s).nominal   = NOM.Nc;
S(s).xlabel    = 'N_c (cap scaled 40/N_c)';
S(s).scale_cap = true;
S(s).logx      = true;

num_panels = numel(S);

%% ==================== BUILD JOB LIST ===================================
% Keys are stored back into S, so the parser never reconstructs them.

JOBS = struct('key', {}, 'P', {});
seen = {};

for si = 1:num_panels
    S(si).keys = cell(1, numel(S(si).values));
    for vi = 1:numel(S(si).values)
        P = NOM;
        P.(S(si).field) = S(si).values(vi);
        if S(si).scale_cap
            P.W_max_scale = NOM.Nc / S(si).values(vi);
        end
        key = param_key(P);
        S(si).keys{vi} = key;

        if ~any(strcmp(seen, key))
            seen{end+1}     = key;             %#ok<SAGROW>
            JOBS(end+1).key = key;             %#ok<SAGROW>
            JOBS(end).P     = P;
        end
    end
end

save(fullfile(output_dir,'sweep_spec.mat'), 'S', 'NOM', '-v7');

num_jobs = numel(JOBS);
fprintf('%d panels x %d values = %d points -> %d unique simulations\n', ...
    num_panels, numel(S(1).values), num_panels*numel(S(1).values), num_jobs);

%% ==================== RUN (skips anything already on disk) =============
todo = true(1, num_jobs);
if ~FORCE_RERUN
    for jj = 1:num_jobs
        if exist(fullfile(output_dir,[JOBS(jj).key '_metrics.mat']),'file')
            todo(jj) = false;
        end
    end
end
run_idx = find(todo);
fprintf('%d of %d simulations still to run.\n', numel(run_idx), num_jobs);

job_P   = {JOBS.P};
job_key = {JOBS.key};

t_sweep = tic;
if USE_PARFOR && ~isempty(run_idx)
    parfor q = 1:numel(run_idx)
        jj = run_idx(q);
        run_and_save(job_P{jj}, job_key{jj}, output_dir, KEEP_FULL, q, numel(run_idx));
    end
else
    for q = 1:numel(run_idx)
        jj = run_idx(q);
        run_and_save(job_P{jj}, job_key{jj}, output_dir, KEEP_FULL, q, numel(run_idx));
    end
end
fprintf('\nSweep wall time: %.1f min\n', toc(t_sweep)/60);
fprintf('Now run parse_sensitivity_all to build the figure.\n');


%% ========================================================================
%                              FUNCTIONS
%% ========================================================================

function key = param_key(P)
% Deterministic, filename-safe identifier built from the swept quantities.
raw = sprintf('sp%g_b%.6g_g%.6g_e%.6g_r%.6g_N%d_w%.6g', ...
    P.T4_des, P.beeta, P.gamma_w, P.eta_c, P.rho_BL, P.Nc, P.W_max_scale);
key = regexprep(raw, '[^A-Za-z0-9_]', 'p');
end

function run_and_save(P, key, output_dir, keep_full, q, ntot)
fprintf('[%d/%d] %s ...\n', q, ntot, key);
R = simulate_rbf(P);

metrics = R.metrics;                                        %#ok<NASGU>
save(fullfile(output_dir,[key '_metrics.mat']), 'metrics', '-v7');
if keep_full
    save(fullfile(output_dir,[key '.mat']), 'R', '-v7.3');
end

fprintf('[%d/%d] %s done: SS NRMSE = %.3f%% (%.1f s)\n', ...
    q, ntot, key, R.metrics.ss_nrmse, R.metrics.wall_sec);
end

%% ---- Closed-loop simulation core ---------------------------------------
function R = simulate_rbf(P)
% Numerically identical to the Figure 3 loop; every quantity previously
% read from the enclosing workspace is now a field of P.
%
% Note: the activation is held as a 1 x Nc vector rather than a len x Nc
% matrix. Only the current row was ever read, and always in the iteration
% that wrote it, so this is identical arithmetic and saves ~4.3*Nc MB of
% live memory per simulation — which matters under parfor.

t_start = tic;

SEC_PER_MIN = 60;
toUnit   = @(sec)  sec / SEC_PER_MIN;
fromUnit = @(unit) unit * SEC_PER_MIN;

dt = P.dt;  ti = P.ti;  tf = P.tf;
time_vector = ti:dt:tf;
len = numel(time_vector);

T4_des = P.T4_des;
Nc     = P.Nc;

% ---- Time-varying rates ----
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

% ---- Burst & micro-pulse parameters ----
t1 = toUnit(500e-6);  t2 = toUnit(100e-6);
t3 = toUnit(500e-6);  t4 = toUnit(9.5e-4);
Tp = t1 + t2 + t3 + t4;
pulseDuty = (t1 + t3)/Tp;

np = 500;
t5 = np * Tp;          % burst ON (minutes)
t6 = toUnit(0.5);      % gap (minutes)

% ---- Window parameters ----
Ts        = P.Ts;
T_on      = P.r_duty * Ts;
T_cycle   = Ts;
T_total   = tf;
T_measure = P.T_measure;

% ---- EF to amplitude mapping ----
thresh = P.thresh;
A_min  = P.A_min;  A_max = P.A_max;  kA = P.kA;

% ---- RBF setup ----
c1d   = linspace(0, 1, Nc);
c     = repmat(c1d, [16, 1]);
beeta = P.beeta;
learning_rate = P.gamma_w;
eta_c = P.eta_c;

W_max = (T4_des/1000 + 0.005) * P.W_max_scale;
W     = 0.5 * W_max * ones(1, Nc);

ki = P.ki;  e_int_max = P.e_int_max;
UL = P.UL;  LL = P.LL;
input_scale = P.input_scale * ones(16,1);

% ---- Measurement model ----
MEAS.enable    = P.meas_enable;
MEAS.alpha_y   = P.meas_alpha_y;
MEAS.sigma     = P.sigma;
MEAS.bias_true = P.meas_bias_true;
meas_state.y_filt = NaN;

% ---- Plant perturbation ----
rng(P.pert_seed);
pert_true = build_perturbation(P.ablation, P.pert_range, P.d4_period_min);

% ---- State & log initialisation ----
y0 = [229.437253584811; 0.273804879029534; ...
      22.9437253584891; 4.10707318544213; ...
      zeros(11,1); 25];                    % previously identified stable state

y_cur = y0;                                % only the previous step is needed
T4_true = zeros(1, len);  T4_true(1) = y0(4);

ref = T4_des * ones(1, len);
new_in = zeros(16, len);
new_in(1,:) = ref;
x = 0;
new_in(2,1) = x;

u = zeros(1, len);
e = ref(1) - x;
e_int = 0;

log_d = zeros(3, len);
log_d(:,1) = [x; 0; e];

if P.keep_weights
    Weights = zeros(len, Nc);
    Weights(1,:) = W;
else
    Weights = [];
end

A_log  = zeros(1, len);
EF_log = zeros(1, len);
T4_measured_log     = zeros(1, len);
T4_measured_raw_log = zeros(1, len);
T4_measured_log(1)     = x;
T4_measured_raw_log(1) = x;

% ---- Amplitude hold logic ----
A_hold = 0;  A_basal = 0;
BAND.enable    = true;
BAND.pct       = P.rho_BL;
BAND.width     = P.rho_BL * T4_des;
BAND.holdN     = P.holdN;
BAND.entered   = false;
BAND.inband_ct = 0;
BAND.reported  = false;
band_lock_time = NaN;

sample_times_amp = ti:Ts:(tf + Ts);
m_amp = 1;
next_window_time = sample_times_amp(1);

% ---- T4 measurement hold logic ----
T4_hold = 0;
T4_sample_times = ti:T_measure:(tf + T_measure);
m_T4 = 1;
next_T4_measure_time = T4_sample_times(1);

u_hold = 0;
phi    = zeros(1, Nc);

%% -------------------- Main simulation loop --------------------
for i = 1 : len-1

    norm_input = new_in(:,i) ./ input_scale;

    for j = 1:Nc
        d = norm_input - c(:,j);
        phi(j) = exp(-(d' * d) / (2*beeta^2));
    end

    u(i) = u_hold;
    EF   = u(i);

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
                    if P.verbose
                        fprintf('  [%.1f h] First entered +/-%.0f%% band (%d/%d)\n', ...
                            current_time/60, BAND.pct*100, BAND.inband_ct, BAND.holdN);
                    end
                    BAND.reported = true;
                end
                if BAND.inband_ct >= BAND.holdN
                    BAND.entered   = true;
                    A_basal        = A_hold;
                    band_lock_time = current_time;
                    if P.verbose
                        fprintf('  [%.1f h] Band lock ACTIVATED after %d windows\n', ...
                            current_time/60, BAND.holdN);
                    end
                end
            else
                BAND.inband_ct = 0;
            end
        end

        m_amp = m_amp + 1;
        if m_amp <= numel(sample_times_amp)
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

    % ---- ODE step ----
    [~, y_temp] = ode15s( ...
        @(t,y) eng_time_varying_ode_EF_MINUTES_PERT( ...
            t, y, EF_time_min, gammaext_T4_time_min, ...
            M_time_min, alpha1_func_min, pert_true), ...
        [time_vector(i), time_vector(i+1)], y_cur);
    y_cur = y_temp(end,:)';
    T4_true(i+1) = y_cur(4);
    T4_actual    = y_cur(4);

    % ---- T4 measurement update ----
    if time_vector(i+1) >= next_T4_measure_time

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
            e_int = e_int + e * T_measure;
            e_int = clamp(e_int, -e_int_max, e_int_max);

            m_sq = 1 + (e / max(ref(i+1), 1e-6))^2 + phi * phi';
            W = W + learning_rate * e * phi / m_sq;
            W = clamp(W, -W_max, W_max);

            for j = 1:Nc
                c(:,j) = c(:,j) + eta_c * e * W(j) * phi(j) ...
                         .* (norm_input - c(:,j)) / (beeta^2);
            end
        end

        u_hold = max(min(W * phi' + ki*e_int, UL), LL);

        new_in(3:end, i+1) = new_in(2:end-1, i);
        new_in(2, i+1)     = x;
        T4_measured_raw_log(i+1) = y_raw;

        m_T4 = m_T4 + 1;
        if m_T4 <= numel(T4_sample_times)
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
    if P.keep_weights, Weights(i+1,:) = W; end
end

% Tails
u(end)      = u(end-1);
A_log(end)  = A_log(end-1);
EF_log(end) = EF_log(end-1);
T4_measured_log(end) = T4_measured_log(end-1);

%% -------------------- Package results --------------------
R.P              = P;
R.T4_log         = T4_true;
R.T4_meas_used   = T4_measured_log;
R.T4_meas_raw    = T4_measured_raw_log;
R.u              = u;
R.A_log          = A_log;
R.EF_log         = EF_log;
R.Weights_final  = W;
R.centres_final  = c;
R.A_basal        = A_basal;
R.band_lock_time = band_lock_time;
R.RMSE           = sqrt(mean(log_d(3,:).^2));
R.y_final        = y_cur;
if P.keep_weights, R.Weights = Weights; end

% Time vector is deterministic from (ti, dt, tf) — store the recipe rather
% than replicate a 4.3 MB ramp in every saved point.
R.time_spec = [ti, dt, tf];

R.metrics = compute_metrics(T4_true, time_vector, T4_des);
R.metrics.T4_des      = T4_des;
R.metrics.A_basal     = A_basal;
R.metrics.band_lock_h = band_lock_time / 60;
R.metrics.wall_sec    = toc(t_start);
end

%% ---- Metrics (definitions match the Figure 3 script) -------------------
function M = compute_metrics(T4_true, time_vector, T4_des)
t_hr  = time_vector / 60;
n_pts = numel(T4_true);

M.term_time = max(t_hr);

idx_10 = find(T4_true >= 0.10 * T4_des, 1, 'first');
idx_90 = find(T4_true >= 0.90 * T4_des, 1, 'first');
if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
    M.rise_time = t_hr(idx_90) - t_hr(idx_10);
else
    M.rise_time = NaN;
end

idx_first_sp = find(T4_true >= T4_des, 1, 'first');
if ~isempty(idx_first_sp)
    T4_peak = max(T4_true(idx_first_sp:end));
    M.overshoot = max(0, (T4_peak - T4_des) / T4_des * 100);
else
    M.overshoot = NaN;
end

ss_idx   = round(0.80 * n_pts) : n_pts;
ss_error = T4_true(ss_idx) - T4_des;
M.ss_mean_error = mean(ss_error);
M.ss_rmse       = sqrt(mean(ss_error.^2));
M.ss_nrmse      = M.ss_rmse / T4_des * 100;
end

%% ---- Perturbed plant ODE (minutes) -------------------------------------
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

alpha  = alpha0 * (1 + pert.alpha_scale);
K      = K0     * (1 + pert.K_scale);
gamma1 = gamma10* (1 + pert.gamma1_scale);

N = 10;  a = 0.00002*N;
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

d4  = pert.d4_bias + pert.d4_amp * sin(2*pi*(t_min/pert.d4_period_min + pert.d4_phase));
dx4 = M_fun*x(2) - gammaext_T4*x(4) - d4;

dydt_sec = [dx1; dx2; dx3; dx4; du_f; du_hist; dNKX21];
dydt_min = 60 * dydt_sec;
end

%% ---- EF burst helper with delay + jitter -------------------------------
function ef = ef_avg_burst_in_window_delay(t, T_on, T_cycle, T_total, t5, t6, A, pulseDuty, delay_min, jitter_min)
t_eff = t - delay_min + jitter_min;
if t_eff < 0,       ef = 0; return; end
if t_eff > T_total, ef = 0; return; end

tau_win = mod(t_eff, T_cycle);
if tau_win >= T_on, ef = 0; return; end

Tb = t5 + t6;
xi = mod(tau_win, Tb);
in_burst = (xi < t5);
A_eff = A * pulseDuty;
ef = A_eff * double(in_burst);
end

%% ---- Measurement LPF (no bias estimator — RBF absorbs bias) ------------
function [y_used, st] = measurement_filter(y_raw, st, MEAS)
if isnan(st.y_filt), st.y_filt = y_raw; end
a = MEAS.alpha_y;
st.y_filt = (1-a)*st.y_filt + a*y_raw;
y_used = st.y_filt;
end

%% ---- Perturbation construction -----------------------------------------
function pert = build_perturbation(mode, R, d4_period_min)
switch lower(mode)
    case 'none'
        pert = neutral_perturbation(d4_period_min);
    case 'half'
        f = fieldnames(R);
        for k = 1:numel(f), R.(f{k}) = R.(f{k}) * 0.5; end
        pert = sample_perturbation(R, d4_period_min);
    case 'full_no_rhythm'
        pert = sample_perturbation(R, d4_period_min);
        pert.d4_bias = 0;
        pert.d4_amp  = 0;
    otherwise  % 'full'
        pert = sample_perturbation(R, d4_period_min);
end
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

% Unused in this script, kept so the helper matches its copies elsewhere
% in the repository.
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

%% ---- Utilities ---------------------------------------------------------
function v = clamp(x, xmin, xmax)
v = min(max(x, xmin), xmax);
end