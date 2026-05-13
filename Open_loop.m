clear all; close all; clc;

%  This code generates figure 1 of the accompanying manuscript. 

%% Open Loop simulations evolving the plant through time as a funciton of EF with no feedback
%  This version of the RBF code lacks many modificaitons, such as the
%  bandlock feature, and should only be used in the open loop context. 


% directory to save PDFs of plot outputs, can also set to pwd if you want
output_dir = fullfile(pwd, 'results_rbf_open_loop');
if ~exist(output_dir,'dir'), mkdir(output_dir); end



%% -------------------- Timing parameters --------------------
dt = 0.1/12; % in minutes, e.g., dt = 0.1 => stepsize of 6 seconds. 
ti = 0;      % STEPSIZES dt need to be small enough such that the solver takes fine enough steps to fully render the EF
tf = 2000;  % in minutes currently
time_vector = ti:dt:tf;
len = length(time_vector);


%% Open Loop Parameters

A_fixed = 0.01;   % open loop AMP to be supplied.
t_A_off = 300;    % minutes — amplitude goes to zero after this time.
                  % set to tf to leave on for the full sim.
%t_A_off = tf;    % Set t_A_off to tf for full duration stimulation. 




%% -------------------- Time Unit Conversion --------------------
SEC_PER_MIN = 60;
toUnit   = @(sec)  sec / SEC_PER_MIN;     % seconds -> minutes
fromUnit = @(unit) unit * SEC_PER_MIN;    % minutes -> seconds

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
Tp = t1 + t2 + t3 + t4;                    % minutes
pulseDuty = (t1 + t3)/Tp;

np = 500;
t5 = np * Tp;                              % burst ON (minutes)
t6 = toUnit(0.5);                          % gap (minutes)

%% -------------------- Window parameters --------------------
Ts      = 120;                   % window length in minutes, how often amp changes
r_duty  = 0.60;                 % duty ratio within window
T_on    = r_duty * Ts;
T_cycle = Ts;
T_total = tf;                   % total simulation time 

T_measure = 20;                 % T4 measurement interval in minutes


%% -------------------- EF to Amplitude mapping --------------------
thresh = 1e-4;                  % EF->A threshold
A_min = 0.00; A_max = 0.10; kA = 0.20;

%% -------------------- Hold logic initialization --------------------
A_hold = 0;                          % Held amplitude
sample_times = ti:Ts:(tf + Ts);      % Window boundary times
m = 1;                               % Window index
next_window_time = sample_times(1);  % First window boundary

%% Initial State (must be above measurement hold logic to avoid transient)
y0 = [229.437253584811; 0.273804879029534; ...
                  22.9437253584891; 4.10707318544213; ...
                  zeros(11,1); 25];
y_log = zeros(16, len);
y_log(:, 1) = y0;

%% -------------------- T4 Measurement hold logic initialization --------------------                                   
T4_hold = y0(4);                                  % Held T4 measurement

T4_sample_times = ti:T_measure:(tf + T_measure);  % T4 measurement times
m_T4 = 1;                                         % T4 measurement index
next_T4_measure_time = T4_sample_times(1);        % First T4 measurement time

%% Begin mostly unchanged RBF code (aside from construction of EF in loop)
% Reference Trajectory
% desired value here
r1 = 20* ones(1, 10*tf + 1); % amplitudes were 10, 20, 30, 40
ref = 15* ones(1, len);

% pick number of centers
Nc = 40;

% assume norm_input ∈ [0,1] in each dimension,
% so we spread Nc centers uniformly on [0,1]:
c1d = linspace(0, 1, Nc);

% replicate across all 16 dims:
c = repmat(c1d, [16,1]);    % size(c)==[16 x Nc]
rng(1);

W_max = (ref(1)/1000 + 0.005); % this linearly interpolates between points (10, 0.015) and (25, 0.030) for (T4 setpoint, max weight)

                                                                 
W = 0.5 * W_max * ones(1, length(c)); %  initial weights
%W = 0.001 * rand(1, length(c)); %  alternative initial weights, small t4 vals
beeta = 0.13;
learning_rate = 1e-6 * (T_measure/dt);

% Input structure: [ref; x; x(t-1); ...]
new_in = zeros(16, len);
x = y0(4);    % initial output (measured T4)
new_in(1, :) = ref;
new_in(2, 1) = x;



% Saturation limits
UL = 100;
LL = 0;
flag = 1;

% Initalize integral action, IS DISABLED IF KI=0
e_int = 0;
ki = 0; % not needed in unnoised cases. 
e_int_max = W_max/ki; % used for anti wind up
           

% Logging
u = zeros(1, len);
e = ref(1) - x;
log_d = zeros(3, len);  % x, u, e
log_d(:, 1) = [x; 0; e];
Weights = zeros(len, length(W));
A_log = zeros(1, len);
T4_measured_log = zeros(1, len);  % Log of held T4 measurements
T4_measured_log(1) = x;
EF_log = zeros(1, len);


% Loop over time steps
for i = 1:len-1

    OPEN_LOOP = true; % toggle between false and true. true => open loop. 
                      % Does not have bandlock functionality, so keep true, only good for open loop. 
    
    % Normalize inputs for RBF
    input_scale = 150*ones(16,1);
    %input_scale = [150; 150; 150; 150; 150; 150];
    norm_input = new_in(:, i) ./ input_scale;
   

    for j = 1:length(c)
        ED(i, j) = exp(-((norm_input - c(:, j))' * (norm_input - c(:, j))) / (2 * beeta^2)); % centers and new_in only update when measurements occur, 
    end                                                                                      % because of this, should just repeat values. 

    % Control input from RBF network
    e_int = e_int + e * dt;
    e_int = clamp(e_int, -e_int_max, e_int_max);
    
    u(i) = max(min(W * ED(i, :)' + ki*e_int, UL), LL) * flag;

   
    % Time-varying EF signal: square wave until t = 60s
    EF=u(i);

    % Check if we've reached a new window boundary
    current_time = time_vector(i);
    if current_time >= next_window_time
        % Update held amplitude at window boundary
        if EF < thresh
            A_hold = 0;
        else
            A_hold = min(max(kA * (EF - thresh), A_min), A_max);
        end
        
        m = m + 1;
        if m <= length(sample_times)
            next_window_time = sample_times(m);
        else
            next_window_time = inf;  % No more windows
        end
    end
    
    % this logic determines if we're in the open or closed loop situation
    if OPEN_LOOP 
        if current_time >= t_A_off
            A_now = 0;
        else
            A_now = A_fixed;
        end 
    else
        A_now = A_hold;
    end
    

    A_log(i) = A_now;  % record for plotting

    % Build EF(t) with burst pattern
    if A_now <= 0
        EF_time_min = @(t_min) 0.0;
    else
        EF_time_min = @(t_min) ef_avg_burst_in_window( ...
            t_min, T_on, T_cycle, T_total, t5, t6, A_now, pulseDuty);
    end
    EF_log(i) = EF_time_min(time_vector(i));

    % More accurate ODE integration
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
    [~, y_temp] = ode15s(@(t, y)  data_tuned_varying_ode_EF_MINUTES(t, y, EF_time_min, gammaext_T4_time_min, M_time_min, alpha1_func_min), [time_vector(i), time_vector(i+1)], y_log(:, i));
    
    y_new = y_temp(end, :)';
    y_log(:, i + 1) = y_new;

    % Get actual T4 value from simulation
    T4_actual = y_new(4);
    
    % Check if we've reached a new T4 measurement time
    current_time = time_vector(i + 1);
    if current_time >= next_T4_measure_time
        % Update held T4 measurement
        T4_hold = T4_actual; 
        
        % System output uses new measurement
        x = T4_hold;
        e = ref(i + 1) - x;
        
        % Update weights only when new measurement arrives
        %m_sq = 1 + ED(i,:) * ED(i,:)';  % Normalizing signal
        m_sq = 1 + (e/ref(i))^2 + ED(i,:) * ED(i,:)';
        W = W + learning_rate * e * ED(i, :) / m_sq;
        W = clamp(W, -W_max, W_max);
        
        % Adapt the centers only when new measurement arrives
        eta_c = 1e-5;
        for j = 1:size(c,2)
            phi_j   = ED(i,j);
            c(:,j)  = c(:,j) ...
                     + eta_c * e * W(j) * phi_j ...
                       .* (norm_input - c(:,j)) / (beeta^2);
        end
        
        % Update input history only at measurement times
        new_in(3:end, i + 1) = new_in(2:end-1, i);   % Shift previous measurements down
        new_in(2, i + 1) = x;                        % Store new measurement
        
        m_T4 = m_T4 + 1;
        if m_T4 <= length(T4_sample_times)
            next_T4_measure_time = T4_sample_times(m_T4);
        else
            next_T4_measure_time = inf;  % No more measurements
        end
    else
        % No new measurement - use held value, no weight update
        x = T4_hold;
        e = ref(i + 1) - x;
        
        % Carry forward the same input history (no shift)
        new_in(2:end, i + 1) = new_in(2:end, i);
    end
    
    T4_measured_log(i + 1) = x;  % Log the held measurement

    % Logging
    log_d(:, i + 1) = [x; EF; e];
    %log_d(:, i + 1) = [x; EF_fun(0); e]; 

    Weights(i + 1, :) = W;
end

% Final control input
u(end) = u(end-1);
A_log(end) = A_log(end-1);
T4_measured_log(end) = T4_measured_log(end-1);
RMSE = sqrt(mean(log_d(3, :).^2));

%% ================== PLOTTING SECTION ==================

% Convert time to hours
time_h = time_vector / 60;      % minutes to hours 
tf_h = time_h(end);

% Extract state variables from y_log
I_log      = y_log(1, :);       % Iodide
T4Int_log  = y_log(2, :);       % T4 internal
Tg_log     = y_log(3, :);       % Thyroglobulin
T4_log     = y_log(4, :);       % T4 external (actual value)
u_f_log    = y_log(5, :);       % Filtered input
NKX21_log  = y_log(16, :);      % NKX21

% =====================================================================
% FIGURE 1: Main Tracking Performance (3 subplots)
% =====================================================================


figure('Color', 'w', 'Position', [100, 100, 900, 800]);
% Subplot 1: T4 Tracking
ax1 = subplot(3, 1, 1);
if OPEN_LOOP % only plot ref if we're in closed loop case. 
    plot(time_h, T4_log, '--r', 'LineWidth', 1.5, 'DisplayName', 'T_4^{ext} (actual)'); hold on;
else
    plot(time_h, ref, '--k', 'LineWidth', 1, 'DisplayName', 'Setpoint'); hold on;
    plot(time_h, T4_log, '--r', 'LineWidth', 1.5, 'DisplayName', 'T_4^{ext} (actual)');
end
stairs(time_h, T4_measured_log, '-b', 'LineWidth', 1, 'DisplayName', 'T_4^{ext} (measured)');
xlabel('Time (h)', 'FontSize', 18);
ylabel('T_4^{ext} (a.u.)', 'FontSize', 18);
xlim([0 tf_h]);
title(sprintf('RBF Tracking of T_4^{ext} (measured every %d min)', T_measure), 'FontSize', 20);
grid on;
legend('Location', 'southeast', 'FontSize', 16);
set(ax1, 'FontSize', 16);

% Subplot 2: RBF Control Output
ax2 = subplot(3, 1, 2);
plot(time_h, u, '-b', 'LineWidth', 1.5, 'DisplayName', 'RBF Output');
xlabel('Time (h)', 'FontSize', 18);
ylabel('u (RBF)', 'FontSize', 18);
xlim([0 tf_h]);
title('RBF Network Output', 'FontSize', 18);
grid on;
legend('Location', 'northwest', 'FontSize', 16);
set(ax2, 'FontSize', 16);

% Subplot 3: Amplitude Applied (use stairs for discrete)
ax3 = subplot(3, 1, 3);
stairs(time_h, A_log, '-m', 'LineWidth', 1.5, 'DisplayName', 'A_{applied}');
hold on;
yline(A_max, '--r', 'LineWidth', 1, 'DisplayName', 'A_{max}');
xlabel('Time (h)', 'FontSize', 18);
ylabel('Amplitude', 'FontSize', 18);
xlim([0 tf_h]);
title('Held Amplitude (Sample-and-Hold)', 'FontSize', 20);
grid on;
legend('Location', 'northwest', 'FontSize', 16);
set(ax3, 'FontSize', 16);

sgtitle('RBF Controller Performance', 'FontSize', 22, 'FontWeight', 'bold');
drawnow;
%savePlot(gcf, output_dir, "main_performance_tracking")%, [8 10]) %force plot 8 by 10 inch

% =====================================================================
% FIGURE 2: State Variable Evolution (4 subplots)
% =====================================================================
figure('Color', 'w', 'Position', [150, 100, 900, 900]);

ax1 = subplot(4, 1, 1);
plot(time_h, I_log, '-b', 'LineWidth', 1.5);
xlabel('Time (h)', 'FontSize', 18);
ylabel('I', 'FontSize', 18);
xlim([0 tf_h]);
title('Iodide (I)', 'FontSize', 20);
grid on;
set(ax1, 'FontSize', 16);

ax2 = subplot(4, 1, 2);
plot(time_h, T4Int_log, '-g', 'LineWidth', 1.5);
xlabel('Time (h)', 'FontSize', 18);
ylabel('T_4^{int} (a.u.)', 'FontSize', 18);
xlim([0 tf_h]);
title('Internal T_4', 'FontSize', 20);
grid on;
set(ax2, 'FontSize', 16);

ax3 = subplot(4, 1, 3);
plot(time_h, Tg_log, '-c', 'LineWidth', 1.5);
xlabel('Time (h)', 'FontSize', 18);
ylabel('T_g', 'FontSize', 18);
xlim([0 tf_h]);
title('Thyroglobulin (T_g)', 'FontSize', 20);
grid on;
set(ax3, 'FontSize', 16);

ax4 = subplot(4, 1, 4);
plot(time_h, ref, '--k', 'LineWidth', 1); hold on;
plot(time_h, T4_log, '-r', 'LineWidth', 1.5);
stairs(time_h, T4_measured_log, '-b', 'LineWidth', 1);
xlabel('Time (h)', 'FontSize', 18);
ylabel('T_4^{ext} (a.u.)', 'FontSize', 18);
xlim([0 tf_h]);
title('External T_4 vs Setpoint', 'FontSize', 20);
grid on;
legend('Setpoint', 'T_4^{ext} (actual)', 'T_4^{ext} (measured)', 'Location', 'northeast', 'FontSize', 16);
set(ax4, 'FontSize', 16);

sgtitle('State Variable Evolution', 'FontSize', 22, 'FontWeight', 'bold');
savePlot(gcf, output_dir, "state_variables")

% =====================================================================
% FIGURE 3: Error and Weights (3 subplots)
% =====================================================================
figure('Color', 'w', 'Position', [200, 100, 900, 800]);

ax1 = subplot(3, 1, 1);
plot(time_h, log_d(3, :), '-c', 'LineWidth', 1.5);
hold on;
yline(0, '--k', 'LineWidth', 0.8);
xlabel('Time (h)', 'FontSize', 18);
ylabel('Error', 'FontSize', 18);
xlim([0 tf_h]);
title('Tracking Error (e = ref - T_4^{ext} measured)', 'FontSize', 20);
grid on;
set(ax1, 'FontSize', 16);
text(0.02, 0.85, sprintf('RMSE = %.4f', RMSE), ...
    'Units', 'normalized', 'FontSize', 16, ...
    'BackgroundColor', 'w', 'EdgeColor', 'k');

ax2 = subplot(3, 1, 2);
plot(time_h, Weights, 'LineWidth', 0.8);
xlabel('Time (h)', 'FontSize', 18);
ylabel('Weights', 'FontSize', 18);
xlim([0 tf_h]);
title('RBF Weight Evolution', 'FontSize', 20);
grid on;
set(ax2, 'FontSize', 16);

ax3 = subplot(3, 1, 3);
plot(time_h, NKX21_log, '-', 'Color', [0.5 0 0.5], 'LineWidth', 1.5);
xlabel('Time (h)', 'FontSize', 18);
ylabel('NKX21', 'FontSize', 18);
xlim([0 tf_h]);
title('NKX21 Signaling State', 'FontSize', 20);
grid on;
set(ax3, 'FontSize', 16);

sgtitle('Error Analysis & Internal States', 'FontSize', 22, 'FontWeight', 'bold');
%savePlot(gcf, output_dir, "error_weights_NKX21")

% =====================================================================
% FIGURE 5: Actual vs Measured T4 Comparison
% =====================================================================
figure('Color', 'w', 'Position', [300, 100, 900, 400]);
if OPEN_LOOP
    plot(time_h, T4_log, '--r', 'LineWidth', 1.5, 'DisplayName', 'T_4^{ext} (actual)'); hold on;
else
    plot(time_h, ref, '--k', 'LineWidth', 1, 'DisplayName', 'Setpoint'); hold on;
    plot(time_h, T4_log, '--r', 'LineWidth', 1.5, 'DisplayName', 'T_4^{ext} (actual)');
end
stairs(time_h, T4_measured_log, '-b', 'LineWidth', 1.5, 'DisplayName', sprintf('T_4^{ext} (measured every %d min)', T_measure));
xlabel('Time (h)', 'FontSize', 18);
ylabel('T_4^{ext} (a.u.)', 'FontSize', 18);
ylim([0 12]);
xlim([0 tf_h]);
title('Actual vs Measured T_4 (Sample-and-Hold)', 'FontSize', 20);
grid on;
legend('Location', 'southeast', 'FontSize', 16);
set(gca, 'FontSize', 16);

% =====================================================================
% FIGURE 6: Actual EF Signal with Pulses
% =====================================================================
figure('Color', 'w', 'Position', [100, 100, 1200, 600]);

ax1 = subplot(2,1,1);
plot(time_h, EF_log, '-m', 'LineWidth', 0.5);
xlabel('Time (h)', 'FontSize', 18);
ylabel('EF (a.u.)', 'FontSize', 18);
xlim([0 tf_h]);
title('EF Signal (Full Simulation)', 'FontSize', 20);
grid on;
set(ax1, 'FontSize', 16);

ax2 = subplot(2,1,2);
zoom_start = 1;
zoom_end = 4;
idx = (time_h >= zoom_start) & (time_h <= zoom_end);
plot(time_h(idx), EF_log(idx), '-m', 'LineWidth', 0.5);
xlabel('Time (h)', 'FontSize', 18);
ylabel('EF (a.u.)', 'FontSize', 18);
title(sprintf('EF Signal (Zoomed: %.1f to %.1f h)', zoom_start, zoom_end), 'FontSize', 20);
grid on;
set(ax2, 'FontSize', 16);

sgtitle('Actual EF Received', 'FontSize', 22, 'FontWeight', 'bold');
%savePlot(gcf, output_dir, "actual_EF")

% =====================================================================
% FIGURE 7: Actual EF Signal with Pulses and more zoom
% =====================================================================
figure('Color', 'w', 'Position', [100, 100, 1200, 800]);

ax1 = subplot(3,1,1);
plot(time_h, EF_log, '-m', 'LineWidth', 0.5);
xlabel('Time (h)', 'FontSize', 18);
ylabel('EF (a.u.)', 'FontSize', 18);
xlim([0 tf_h]);
title('EF Signal (Full Simulation)', 'FontSize', 20);
grid on;
set(ax1, 'FontSize', 16);

ax2 = subplot(3,1,2);
zoom_start_h = 1;
zoom_end_h = 4;
idx = (time_h >= zoom_start_h) & (time_h <= zoom_end_h);
plot(time_h(idx), EF_log(idx), '-m', 'LineWidth', 0.5);
xlabel('Time (h)', 'FontSize', 18);
ylabel('EF (a.u.)', 'FontSize', 18);
title(sprintf('EF Signal (Medium Zoom: %.1f to %.1f h)', zoom_start_h, zoom_end_h), 'FontSize', 20);
grid on;
set(ax2, 'FontSize', 16);

ax3 = subplot(3,1,3);
zoom_center_h = 2.5;
zoom_window_h = 60/3600;

A_at_zoom = A_log(find(time_h <= zoom_center_h, 1, 'last'));
EF_zoom_fun = @(t_min) ef_avg_burst_in_window( ...
    t_min, T_on, T_cycle, T_total, t5, t6, A_at_zoom, pulseDuty);

t_fine_h = linspace(zoom_center_h, zoom_center_h + zoom_window_h, 10000);
t_fine_min = t_fine_h * 60;
EF_fine = arrayfun(EF_zoom_fun, t_fine_min);

plot(t_fine_h, EF_fine, '-m', 'LineWidth', 0.5);
xlabel('Time (h)', 'FontSize', 18);
ylabel('EF (a.u)', 'FontSize', 18);
title(sprintf('EF Signal (Ultra Zoom: 60 sec at %.2f h, Evaluated on a Fine Grid)', zoom_center_h), 'FontSize', 20);
grid on;
set(ax3, 'FontSize', 16);

sgtitle('Actual EF Received', 'FontSize', 22, 'FontWeight', 'bold');

% =====================================================================
% Print summary to console
% =====================================================================
fprintf('\n========== RBF Controller Summary ==========\n');
fprintf('RMSE:            %.4f\n', RMSE);
fprintf('Final Error:     %.4f\n', log_d(3, end));
fprintf('Num Centers:     %d\n', Nc);
fprintf('Learning Rate:   %.2e\n', learning_rate);
fprintf('Beta:            %.3f\n', beeta);
fprintf('Simulation Time: %.2f hours\n', tf_h);
fprintf('T4 Measurement Interval: %d minutes\n', T_measure);
fprintf('Amplitude Update Interval: %d minutes\n', Ts);
fprintf('=============================================\n');


%% -------------------- EF averaging helper -------------------------------
function ef = ef_avg_burst_in_window(t, T_on, T_cycle, T_total, t5, t6, A, pulseDuty)
    if t > T_total, ef = 0; return; end
    tau_win = mod(t, T_cycle);
    if tau_win >= T_on, ef = 0; return; end
    Tb = t5 + t6;  xi = mod(tau_win, Tb);
    in_burst = (xi < t5);
    A_eff = A * pulseDuty;            % average; RMS would be A*sqrt(pulseDuty)
    ef = A_eff * double(in_burst);
end

function y = clamp(x, lo, hi)
    y = max(min(x, hi), lo);
end

%% a save plot function, make sure to use a name that is compatible with your directory pathways (output_dir above)
% specify folder as dir at the top, call with a specific name str, no "."
% supply a figSize if you want, otherwise fills to the size of the matlab
function savePlot(currentFig, folder, name, figSize)
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    name = sprintf('%s_%s', timestamp, name);
    
    fullname = fullfile(folder, name);
    
    set(currentFig, 'Units', 'Inches');
    
    if nargin >= 4 && ~isempty(figSize)
        set(currentFig, 'Position', [1 1 figSize(1) figSize(2)]);
        set(currentFig, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Inches', 'PaperSize', figSize);
    else
        pos = get(currentFig, 'Position');
        set(currentFig, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Inches', 'PaperSize', [pos(3) pos(4)]);
    end
    
    print(currentFig, fullname, '-dpdf', '-r300')
end

%% -------------------- Plant ODE -----------------------------------------
function dydt_min = data_tuned_varying_ode_EF_MINUTES( ...
         t_min, y, EF_time_min, gammaext_T4_time_min, M_time_min, alpha1_func_min)

    % inputs evaluated at minute time
    EF          = EF_time_min(t_min);
    gammaext_T4 = gammaext_T4_time_min(t_min);
    M_fun       = M_time_min(t_min);
    alpha1      = alpha1_func_min(t_min);

    % ---- original RHS in *per-second* units (unchanged) ----
    x      = y(1:4);      u_f = y(5);     u_hist = y(6:15);    nkx21 = y(16);

    %alpha = 8.43812;  K = .1195;  gamma1 = .0296;  n = 2;
    alpha = 20*8.43812;  K = 10*.1195;  gamma1 = .0296;  n = 2;
    N = 10;  a=0.00002*N;

    U = zeros(N,1);  U(1) = u_f;
    du_f    = alpha1 * EF^n / (K^n + EF^n) - gamma1*u_f;
    du_hist = (-a*eye(N) + a*diag(ones(1,N-1),-1)) * u_hist + a*U;
    dNKX21  = alpha*u_hist(end) - .01*nkx21 + .25;

    kappa = 1; K2 = 1000;
    k_1   = kappa * ((nkx21/K2)^3 / (1 + (nkx21/K2)^3));

    alpha_TG = 1;  alpha_I = 1;
    gamma_I = 0.004;
    gamma_Tg = .04;
    gammaint_T4 = 0.15+0.0004;

    dx1 = -k_1*x(3)*x(1) - gamma_I*x(1) + alpha_I;
    dx2 =  k_1*x(3)*x(1) - M_fun*x(2)-gammaint_T4*x(2);
    dx3 =  alpha_TG - k_1*x(3)*x(1) - gamma_Tg*x(3);
    dx4 =  M_fun*x(2) - gammaext_T4*x(4);

    dydt_sec = [dx1; dx2; dx3; dx4; du_f; du_hist; dNKX21];

    % ---- convert per-second RHS to per-minute derivative ----
    dydt_min = 60 * dydt_sec;
end