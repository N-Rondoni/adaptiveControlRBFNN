clear all; close all; clc;
%% ========================================================================
%  PARSE_SENSITIVITY_ALL
%  Reads the per-setpoint sensitivity sweeps and builds one figure with all
%  setpoints overlaid, plus the text table.
%
%  Only the small _metrics.mat companions are read, never the full
%  resolution logs, so the figure restyles in seconds.
%
%  LAYOUT
%  Five parameter panels in a 2 x 3 grid, with the sixth slot given over to
%  a single shared legend. SS NRMSE is normalized by setpoint, so curves
%  from different setpoints are directly comparable on one axis.
%
%  The unscaled node count panel (Nc_raw) is excluded: only the scaled
%  convention, which holds control authority fixed as nodes are added, is
%  reported.
%
%  Panel axis labels are overridden below so the figure uses the same
%  symbols as the manuscript (N for the node count, not Nc).
%% ========================================================================

%% ==================== CONFIGURATION ====================================
EXCLUDE_PANELS = {'Nc_raw'};        % report the scaled node count only
DO_SAVE_PDF    = false;
SHARED_YLIMS   = false;
% The eta panel spans a fraction of a percentage point while beta spans
% tens. Autoscaling makes a flat panel look like a trend; shared limits
% make it honest but compress the informative panels. Either is defensible
% so long as the caption says which was used.

%% ---- Discover setpoint directories: pwd/results_sensitivity_sp* -------
dd = dir(fullfile(pwd, 'results_sensitivity_sp*'));
dd = dd([dd.isdir]);
sp_found = [];
for k = 1:numel(dd)
    tok = regexp(dd(k).name, '^results_sensitivity_sp([\d.]+)$', 'tokens', 'once');
    if ~isempty(tok), sp_found(end+1) = str2double(tok{1}); end %#ok<SAGROW>
end
SETPOINTS = sort(unique(sp_found));
if isempty(SETPOINTS)
    error('No results_sensitivity_sp* directories found in %s', pwd);
end
fprintf('Found setpoints: %s\n', mat2str(SETPOINTS));

dir_for = @(sp) fullfile(pwd, sprintf('results_sensitivity_sp%g', sp));

% ---- Setpoint styling ----
% Colors match RBF_no_noise.m, which sets sp_colors = lines(num_setpoints)
% over the setpoints in ascending order, so Figure 3 and this figure agree
% setpoint for setpoint. Markers are added so the curves remain separable
% in grayscale.
SP_COLORS  = lines(numel(SETPOINTS));
SP_MARKERS = {'o', 's', '^', 'd', 'v', 'p'};

% ---- Font sizes ----
FS.axes   = 18;
FS.label  = 20;
FS.title  = 20;
FS.sgtitle= 24;
FS.legend = 20;

% ---- Panel labels, matching manuscript notation ----
% S(s).xlabel carries whatever the sweep generator stored. Override it
% here, keyed by panel name (S.name), so the printed figure uses the
% manuscript symbols; panels with no entry fall back to the stored label.
XLAB = containers.Map( ...
    {'beta', 'gamma', 'eta', 'rho_BL', 'Nc_scaled'}, ...
    {'\beta (RBF width)', ...
     '\gamma (weight LR)', ...
     '\eta (center gain)', ...
     '\rho_{BL} (band width)', ...
     'N (RBF nodes)'});

nsp = numel(SETPOINTS);

%% ==================== LOAD ============================================
% Panel structure comes from the first available setpoint; the others are
% checked against it rather than assumed to match.
S_ref = [];  NOM_ref = [];  ref_sp = NaN;
for k = 1:nsp
    f = fullfile(dir_for(SETPOINTS(k)), 'sweep_spec.mat');
    if exist(f, 'file')
        L = load(f, 'S', 'NOM');
        S_ref = L.S;  NOM_ref = L.NOM;  ref_sp = SETPOINTS(k);
        break;
    end
end
if isempty(S_ref)
    error('No sweep_spec.mat found for any setpoint in %s', pwd);
end

keep       = ~ismember({S_ref.name}, EXCLUDE_PANELS);
S          = S_ref(keep);
num_panels = numel(S);
num_vals   = numel(S(1).values);

fprintf('Panels: %s\n', strjoin({S.name}, ', '));
fprintf('Excluded: %s\n', strjoin(EXCLUDE_PANELS, ', '));

% nrmse(panel, value, setpoint)
D.ss_nrmse  = nan(num_panels, num_vals, nsp);
D.ss_rmse   = nan(num_panels, num_vals, nsp);
D.rise_time = nan(num_panels, num_vals, nsp);
D.overshoot = nan(num_panels, num_vals, nsp);
D.band_lock = nan(num_panels, num_vals, nsp);
D.wall_sec  = nan(num_panels, num_vals, nsp);

NOM_all  = cell(1, nsp);
have_sp  = false(1, nsp);
missing  = {};

for k = 1:nsp
    sp   = SETPOINTS(k);
    odir = dir_for(sp);
    spec = fullfile(odir, 'sweep_spec.mat');

    if ~exist(spec, 'file')
        fprintf('No sweep found for setpoint %g (%s) — skipping\n', sp, odir);
        continue;
    end
    have_sp(k) = true;

    Lk = load(spec, 'S', 'NOM');
    NOM_all{k} = Lk.NOM;
    Sk = Lk.S;

    for s = 1:num_panels
        is = find(strcmp({Sk.name}, S(s).name), 1);
        if isempty(is)
            fprintf('Setpoint %g has no panel %s — skipping\n', sp, S(s).name);
            continue;
        end
        if ~isequal(Sk(is).values, S(s).values)
            warning('Setpoint %g sweeps %s over a different grid than setpoint %g.', ...
                sp, S(s).name, ref_sp);
        end

        for v = 1:num_vals
            mf = fullfile(odir, [Sk(is).keys{v} '_metrics.mat']);
            if ~exist(mf, 'file')
                missing{end+1} = sprintf('sp %g, %s = %g', ...
                    sp, S(s).name, Sk(is).values(v)); %#ok<SAGROW>
                continue;
            end
            M = load(mf, 'metrics');
            D.ss_nrmse(s,v,k)  = M.metrics.ss_nrmse;
            D.ss_rmse(s,v,k)   = M.metrics.ss_rmse;
            D.rise_time(s,v,k) = M.metrics.rise_time;
            D.overshoot(s,v,k) = M.metrics.overshoot;
            D.band_lock(s,v,k) = M.metrics.band_lock_h;
            D.wall_sec(s,v,k)  = M.metrics.wall_sec;
        end
    end
end

if ~isempty(missing)
    fprintf('\nMissing %d point(s):\n', numel(missing));
    fprintf('  %s\n', missing{:});
end

sp_idx    = find(have_sp);
sp_labels = arrayfun(@(v) sprintf('%g', v), SETPOINTS(sp_idx), 'UniformOutput', false);

%% ==================== MAIN FIGURE =====================================
n_rows = 2;  n_cols = 3;      % five panels plus a legend slot

figure('Color','w','Position',[80 80 1700 1000]);
sgtitle('Controller-parameter sensitivity across setpoints (SS NRMSE)', ...
    'FontSize', FS.sgtitle, 'FontWeight', 'bold');

if SHARED_YLIMS
    ylims = [0, max(D.ss_nrmse(:)) * 1.08];
end

for s = 1:num_panels
    subplot(n_rows, n_cols, s); hold on;

    xv = S(s).values;

    for kk = 1:numel(sp_idx)
        k = sp_idx(kk);
        plot(xv, D.ss_nrmse(s,:,k), '-', ...
            'Marker', SP_MARKERS{mod(k-1,numel(SP_MARKERS))+1}, ...
            'Color', SP_COLORS(k,:), ...
            'MarkerFaceColor', SP_COLORS(k,:), ...
            'LineWidth', 2.2, 'MarkerSize', 9, ...
            'HandleVisibility', 'off');
    end

    xline(S(s).nominal, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.6, ...
        'HandleVisibility', 'off');

    if isKey(XLAB, S(s).name)
        xlabel(XLAB(S(s).name), 'FontSize', FS.label);
    else
        xlabel(S(s).xlabel, 'FontSize', FS.label);
    end
    ylabel('SS NRMSE (%)', 'FontSize', FS.label);
    grid on; box off;
    set(gca, 'FontSize', FS.axes, 'TickDir', 'out', 'LineWidth', 1.1);

    if S(s).logx
        % Geometric grid — a linear axis crushes the small-N end.
        set(gca, 'XScale', 'log');
        xticks(xv);
        xticklabels(arrayfun(@(z) sprintf('%d',z), xv, 'UniformOutput', false));
    else
        xlim([min(xv) - 0.08*range(xv), max(xv) + 0.08*range(xv)]);
    end

    if SHARED_YLIMS, ylim(ylims); end
end

% ---- Sixth slot: shared legend ----
% The nominal marker is carried in the legend rather than labelled inside
% each panel, where the text collided with the curves.
ax_leg = subplot(n_rows, n_cols, n_rows*n_cols); hold on;
h = gobjects(1, numel(sp_idx) + 1);
for kk = 1:numel(sp_idx)
    k = sp_idx(kk);
    h(kk) = plot(NaN, NaN, '-', ...
        'Marker', SP_MARKERS{mod(k-1,numel(SP_MARKERS))+1}, ...
        'Color', SP_COLORS(k,:), ...
        'MarkerFaceColor', SP_COLORS(k,:), ...
        'LineWidth', 2.6, 'MarkerSize', 12);
end
h(end) = plot(NaN, NaN, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.6);

axis(ax_leg, 'off');
lg = legend(ax_leg, h, [sp_labels, {'nominal value'}], ...
    'FontSize', FS.legend, 'Box', 'on');
title(lg, 'Setpoint: T_4^{ext} (a.u.)', 'FontSize', FS.legend);

% Let the legend size itself, then center it in the empty slot. Setting
% lg.Position to the slot outright would stretch the box to fill the whole
% subplot, which looks wrong once the box is visible.
drawnow;
lp = lg.Position;
ap = ax_leg.Position;
lg.Position = [ap(1) + (ap(3)-lp(3))/2, ...
               ap(2) + (ap(4)-lp(4))/2, ...
               lp(3), lp(4)];

if DO_SAVE_PDF
    ts = char(datetime('now','Format','yyyyMMdd_HHmmss'));
    pdfPath = fullfile(dir_for(ref_sp), sprintf('%s_sensitivity_allsp.pdf', ts));
    exportgraphics(gcf, pdfPath, 'ContentType','vector', 'Resolution',300);
    fprintf('Saved: %s\n', pdfPath);
end

%% ==================== CONSOLE + TEXT TABLE ============================
txt_path = fullfile(pwd, 'sensitivity_all_setpoints.txt');
fid = fopen(txt_path, 'w');

hdr = sprintf('Controller-parameter sensitivity, setpoints %s, noiseless configuration', ...
    mat2str(SETPOINTS(sp_idx)));
nomline = sprintf('Nominal: beta = %.4g, gamma = %.4g, eta = %.4g, rho_BL = %.4g, N = %d', ...
    NOM_ref.beeta, NOM_ref.gamma_w, NOM_ref.eta_c, NOM_ref.rho_BL, NOM_ref.Nc);
exline = sprintf('Excluded panels: %s', strjoin(EXCLUDE_PANELS, ', '));

fprintf('\n%s\n%s\n%s\n', hdr, nomline, exline);
fprintf(fid, '%s\n%s\n%s\n\n', hdr, nomline, exline);

col_hdr = sprintf('  %-14s', 'value');
for kk = 1:numel(sp_idx)
    col_hdr = [col_hdr sprintf(' | %10s', sprintf('sp %s', sp_labels{kk}))]; %#ok<AGROW>
end

for s = 1:num_panels
    fprintf('\n%s   (SS NRMSE %%)\n', S(s).name);
    fprintf(fid, '%s   (SS NRMSE %%)\n', S(s).name);
    fprintf('%s\n', col_hdr);
    fprintf(fid, '%s\n', col_hdr);

    for v = 1:num_vals
        line = sprintf('  %-14.6g', S(s).values(v));
        for kk = 1:numel(sp_idx)
            line = [line sprintf(' | %10.3f', D.ss_nrmse(s,v,sp_idx(kk)))]; %#ok<AGROW>
        end
        if S(s).values(v) == S(s).nominal, line = [line '  <- nominal']; end %#ok<AGROW>
        fprintf('%s\n', line);
        fprintf(fid, '%s\n', line);
    end
    fprintf(fid, '\n');
end

% The nominal point is shared by every panel, so runs are counted once per
% setpoint by run key.
total_sec = 0;  n_runs = 0;
for kk = 1:numel(sp_idx)
    k = sp_idx(kk);
    seen = {};
    Lk = load(fullfile(dir_for(SETPOINTS(k)), 'sweep_spec.mat'), 'S');
    for s = 1:num_panels
        is = find(strcmp({Lk.S.name}, S(s).name), 1);
        if isempty(is), continue; end
        for v = 1:num_vals
            key = Lk.S(is).keys{v};
            if ~any(strcmp(seen, key)) && ~isnan(D.wall_sec(s,v,k))
                seen{end+1} = key;  %#ok<SAGROW>
                total_sec = total_sec + D.wall_sec(s,v,k);
                n_runs = n_runs + 1;
            end
        end
    end
end

fprintf('\nTotal simulation time across %d unique runs: %.1f min\n', n_runs, total_sec/60);
fprintf('Text table saved to: %s\n', txt_path);
fprintf(fid, '\nTotal simulation time across %d unique runs: %.1f min\n', n_runs, total_sec/60);
fclose(fid);