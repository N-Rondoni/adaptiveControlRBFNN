*temporary readme, to be updated*


adaptiveControlRBFNN
├── Open_loop.m                      % Open-loop response of the plant
├── RBF_no_noise.m                   % Setpoint sweep, no noise
├── RBF_setpoint_sequence.m          % Sequential setpoint tracking (warm-start)
├── RBF_measurement_noise.m          % Sensor-noise sigma sweep
├── Supplement_unstable_example.m    % Naive RBF without stabilizing modifications
└── Monte_Carlo/
    ├── RBF_Monte_Carlo.m         % 100-seed Monte Carlo, all noise sources active
    └── MC_seed_viewer.m          % Re-plot a single Monte-Carlo seed
