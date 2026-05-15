close all; 

% % This code generates figure 6 in the accompanying manuscript. 
%% ========================================================================
%  MC Seed Viewer — Load and plot dynamics from a single Monte Carlo seed
%% ========================================================================
% Helper file to examine specific seed outputs from the Monte Carlo sim. 
%
% requires RBF_Monte_Carlo.m to have looped over the desired SEED. 
% point to where the output was saved 
%   - set MC_DIR to what was used in output_dir of RBF_Monte_Carlo.m 
% Views specific seed trajectories, such as what is seen in figure 6 of the
% accompanying manuscript. 


%% ---- Configuration (edit this to see particular run/seed) ----------------------------------------
SEED = 70; % 70 good example for half ablation, seed 7 for full ablation. 
MC_DIR = fullfile(pwd, 'results_rbf_Monte_Carlo');  % <- match what is used in RBF_Monte_Carlo
%% ---- Load ---------------------------------------------------------------
seed_file = fullfile(MC_DIR, sprintf('RBF_MC_seed%03d.mat', SEED));
if ~exist(seed_file, 'file'), error('Not found: %s', seed_file); end
fprintf('Loading seed %d ...\n', SEED);
loaded = load(seed_file);
ALL_RESULTS = loaded.ALL_RESULTS;
pert_true   = loaded.pert_true;
sp_vals     = loaded.setpoint_values;
num_sp      = numel(sp_vals);

fprintf('Seed %d metrics:\n', SEED);
sm = loaded.seed_metrics;
fprintf('%-8s | %10s | %10s | %10s | %10s\n', 'sp', 'Rise(hr)', 'OS(%)', 'SS_NRMSE%', 'Lock(hr)');
fprintf('----------------------------------------------------------\n');
for i = 1:num_sp
    fprintf('%-8g | %10.2f | %10.2f | %10.2f | %10.1f\n', ...
        sp_vals(i), sm.rise_time(i,1), sm.overshoot(i,1), ...
        sm.ss_nrmse(i,1), sm.band_lock_hr(i,1));
end

%% ---- Colors -------------------------------------------------------------
sp_colors = lines(num_sp);

%% ---- Figure 1: T4 trajectories -----------------------------------------
figure('Color','w','Position',[100 100 1200 600]);
hold on;
for i = 1:num_sp
    fn = sprintf('sp%g_sigma2p00', sp_vals(i));
    fn = strrep(fn, '-', 'm');
    R = ALL_RESULTS.(fn);
    t_hr = R.time / 60;
    tf_h = t_hr(end);

    plot([0 tf_h], [sp_vals(i) sp_vals(i)], '--', 'Color', sp_colors(i,:), ...
        'LineWidth', 1.5, 'DisplayName', sprintf('sp = %g', sp_vals(i)));
    plot(t_hr, R.T4_log, '-', 'Color', sp_colors(i,:), 'LineWidth', 1.3, ...
        'HandleVisibility', 'off');

end
xlabel('Time (h)', 'FontSize', 18); ylabel('T_4^{ext} (a.u.)', 'FontSize', 18);
title(sprintf('Seed %d — T_4 Trajectories', SEED), 'FontSize', 22, 'FontWeight', 'bold');
legend('Location', 'eastoutside', 'FontSize', 16); grid on; xlim([0 tf_h]);
set(gca, 'FontSize', 16);

%% ---- Figure 2: Zoomed settling (last 15 h) -----------------------------
figure('Color','w','Position',[120 120 1200 600]);
hold on;
for i = 1:num_sp
    fn = sprintf('sp%g_sigma2p00', sp_vals(i));
    fn = strrep(fn, '-', 'm');
    R = ALL_RESULTS.(fn);
    t_hr = R.time / 60; tf_h = t_hr(end);

    plot([0 tf_h], [sp_vals(i) sp_vals(i)], '--', 'Color', sp_colors(i,:), ...
        'LineWidth', 1.5, 'DisplayName', sprintf('sp = %g', sp_vals(i)));
    plot(t_hr, R.T4_log, '-', 'Color', sp_colors(i,:), 'LineWidth', 1.3, ...
        'HandleVisibility', 'off');
end
xlabel('Time (h)', 'FontSize', 18); ylabel('T_4^{ext} (a.u.)', 'FontSize', 18);
title(sprintf('Seed %d — Settling Region (Last 15 h)', SEED), 'FontSize', 22, 'FontWeight', 'bold');
legend('Location', 'eastoutside', 'FontSize', 16); grid on;
xlim([max(0, tf_h - 15), tf_h]);
set(gca, 'FontSize', 16);

%% ---- Figure 3: Control signals (RBF output + Amplitude) ----------------
figure('Color','w','Position',[140 140 1400 500]);
sgtitle(sprintf('Seed %d — Control Signals', SEED), 'FontSize', 22, 'FontWeight', 'bold');

subplot(1,2,1); hold on;
for i = 1:num_sp
    fn = sprintf('sp%g_sigma2p00', sp_vals(i));
    fn = strrep(fn, '-', 'm');
    R = ALL_RESULTS.(fn);
    plot(R.time/60, R.u, '-', 'Color', sp_colors(i,:), 'LineWidth', 1.0, ...
        'DisplayName', sprintf('sp = %g', sp_vals(i)));
end
xlabel('Time (h)', 'FontSize', 18); ylabel('u (RBF output)', 'FontSize', 18);
title('RBF Control Output', 'FontSize', 20); legend('Location','southeast','FontSize',16); grid on; xlim([0 tf_h]);
set(gca, 'FontSize', 16);

subplot(1,2,2); hold on;
for i = 1:num_sp
    fn = sprintf('sp%g_sigma2p00', sp_vals(i));
    fn = strrep(fn, '-', 'm');
    R = ALL_RESULTS.(fn);
    stairs(R.time/60, R.A_log, '-', 'Color', sp_colors(i,:), 'LineWidth', 1.0, ...
        'DisplayName', sprintf('sp = %g', sp_vals(i)));
    if R.A_basal > 0
        yline(R.A_basal, '--', 'Color', sp_colors(i,:), 'LineWidth', 1.2, ...
            'Label', sprintf('A_{basal}=%.4f', R.A_basal), 'FontSize', 16, ...
            'LabelHorizontalAlignment', 'right', 'HandleVisibility', 'off');
    end
end
xlabel('Time (h)', 'FontSize', 18); ylabel('Amplitude A', 'FontSize', 18);
title('Control Amplitude', 'FontSize', 20); legend('Location','southeast','FontSize',16); grid on; xlim([0 tf_h]);
set(gca, 'FontSize', 16);

%% ---- Figure 4: Absolute error per setpoint -----------------------------
n_cols = min(num_sp, 4);
figure('Color','w','Position',[160 160 350*n_cols 350]);
sgtitle(sprintf('Seed %d — Absolute Error', SEED), 'FontSize', 22, 'FontWeight', 'bold');
for i = 1:num_sp
    fn = sprintf('sp%g_sigma2p00', sp_vals(i));
    fn = strrep(fn, '-', 'm');
    R = ALL_RESULTS.(fn);
    subplot(1, n_cols, i); hold on;
    yline(0, '--k', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    plot(R.time/60, R.T4_log - sp_vals(i), '-', 'Color', sp_colors(i,:), 'LineWidth', 1.0);
    xlabel('Time (h)', 'FontSize', 18); ylabel('Error (a.u.)', 'FontSize', 18);
    title(sprintf('sp = %g', sp_vals(i)), 'FontSize', 20);
    grid on; xlim([0 tf_h]);
    set(gca, 'FontSize', 16);
end

%% ---- Figure 5: Weight evolution (measurement cadence) -------------------
figure('Color','w','Position',[180 180 350*n_cols 350]);
sgtitle(sprintf('Seed %d — Weight Evolution', SEED), 'FontSize', 22, 'FontWeight', 'bold');
for i = 1:num_sp
    fn = sprintf('sp%g_sigma2p00', sp_vals(i));
    fn = strrep(fn, '-', 'm');
    R = ALL_RESULTS.(fn);
    subplot(1, n_cols, i); hold on;
    plot(R.meas_times/60, R.Weights, '-', 'LineWidth', 0.6);
    if ~isnan(R.band_lock_time)
        xline(R.band_lock_time/60, '--k', 'LineWidth', 1.4, ...
            'Label', sprintf('Lock @ %.1f h', R.band_lock_time/60), ...
            'FontSize', 16, 'LabelVerticalAlignment', 'bottom');
    end
    xlabel('Time (h)', 'FontSize', 18); ylabel('Weight', 'FontSize', 18);
    title(sprintf('sp = %g', sp_vals(i)), 'FontSize', 20);
    grid on; xlim([0 tf_h]);
    set(gca, 'FontSize', 16);
end



%% ---- Figure 6: SS_NRMSE bar chart for this seed ------------------------
figure('Color', 'w', 'Position', [200 200 700 500]);
sp_labels = arrayfun(@(v) sprintf('%g', v), sp_vals, 'UniformOutput', false);
bar(categorical(sp_labels, sp_labels), sm.ss_nrmse(:,1), ...
    'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'k', 'LineWidth', 1.2);
xlabel('Setpoint (a.u.)', 'FontSize', 18);
ylabel('SS NRMSE (%)', 'FontSize', 18);
title(sprintf('Seed %d — SS NRMSE (final 20%%)', SEED), 'FontSize', 22, 'FontWeight', 'bold');
grid on; set(gca, 'FontSize', 16);
for i = 1:num_sp
    text(i, sm.ss_nrmse(i,1) + 0.4, sprintf('%.2f%%', sm.ss_nrmse(i,1)), ... % do 0.7 for certain renders
        'HorizontalAlignment', 'center', 'FontSize', 16, 'FontWeight', 'bold');
end


fprintf('\nDone. %d figures generated for seed %d.\n', 5, SEED);



%% ---- Report active noise configuration ---- repeated from main scripts
% ==================== Nominal plant parameters (single source of truth) ==
%  Used by (1) the ODE and (2) the perturbation report.
%  Field names match the _scale suffix convention in the perturbation struct.
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


NOM.alpha   = 20*8.43812;
NOM.K       = 10*0.1195;
NOM.gamma1  = 0.0296;
NOM.kappa   = 1;
NOM.K2      = 1000;
NOM.gammaI  = 0.004;
NOM.gammaTg = 0.04;
NOM.gint    = 0.15 + 0.0004;
NOM.gext    = gammaext1_0;        % time-varying base (epsilon_gamma = 0)
NOM.M       = M0;                 % time-varying base (epsilon_M = 0)
NOM.alpha1  = alpha1_0;           % time-varying base (epsilon_alpha1 = 0)


rng(SEED)
ABLATION_MODE = 'half_no_rhythm';
fprintf('\n--- Noise Configuration (ABLATION_MODE = %s) ---\n', ABLATION_MODE);
if ~strcmp(ABLATION_MODE, 'none')
    report_perturbation(pert_true, NOM);
else
    fprintf('Plant perturbations DISABLED (nominal plant).\n');
end
fprintf('Measurement model: enable = %d\n', MEAS_template.enable);
fprintf('  LPF alpha_y = %.2f\n', MEAS_template.alpha_y);
fprintf('  Sigma values (sweep): %s\n', mat2str(sigma_values));
fprintf('  Bias: 0%% of setpoint (set per run)\n'); %hardcoded
fprintf('---\n\n');




%% ---- Neutral perturbation (nominal plant, all zeros) --------------------
function pert = neutral_perturbation(d4_period_min)
    pert.alpha_scale   = 0;  pert.K_scale      = 0;  pert.gamma1_scale = 0;
    pert.kappa_scale   = 0;  pert.K2_scale     = 0;  pert.gammaI_scale = 0;
    pert.gammaTg_scale = 0;  pert.gint_scale   = 0;  pert.gext_scale   = 0;
    pert.M_scale       = 0;  pert.alpha1_scale = 0;
    pert.d4_bias = 0;  pert.d4_amp = 0;  pert.d4_phase = 0;
    pert.d4_period_min = d4_period_min;
    pert.act_gain = 0;  pert.delay_min = 0;  pert.jitter_min = 0;
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


    % Measurement perts handled by HANDLED BY MEAS.sigma = sigma     
end

%%
function report_perturbation(pert, nominal_params, fid)

% REPORT_PERTURBATION  Print nominal vs perturbed parameter values.
%   nominal_params is a struct with fields matching the _scale names,
%   e.g. nominal_params.alpha, nominal_params.K, etc.
%   fid (optional) — file ID to write to; defaults to 1 (stdout).

    if nargin < 3, fid = 1; end

    fprintf(fid, '\n=== Perturbation Report ===\n');
    fprintf(fid, '%-14s  %12s  %12s  %8s\n', 'Parameter', 'Nominal', 'Perturbed', '% Diff');
    fprintf(fid, '%s\n', repmat('-', 1, 52));

    fnames = fieldnames(nominal_params);
    for i = 1:numel(fnames)
        nm = fnames{i};
        scale_field = [nm '_scale'];
        if isfield(pert, scale_field)
            nom = nominal_params.(nm);
            ptb = nom * (1 + pert.(scale_field));
            if nom ~= 0
                pct = 100 * pert.(scale_field);
            else
                pct = NaN;
            end
            fprintf(fid, '%-14s  %12.6g  %12.6g  %+7.2f%%\n', nm, nom, ptb, pct);
            if ptb <= 0 && nom > 0
                fprintf(fid, '  *** WARNING: sign flip on %s! ***\n', nm);
            end
        end
    end

    % Additive perturbations
    add_fields = {'d4_bias', 'd4_amp', 'delay_min', 'jitter_min', 'act_gain'};
    fprintf(fid, '\n%-14s  %12s\n', 'Additive', 'Value');
    for i = 1:numel(add_fields)
        if isfield(pert, add_fields{i})
            fprintf(fid, '%-14s  %+12.6g\n', add_fields{i}, pert.(add_fields{i}));
        end
    end
    fprintf(fid, '\n');
end