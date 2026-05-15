# adaptiveControlRBFNN

MATLAB implementation of an adaptive radial basis function (RBF)
neural-network controller for an artificial thyroid gland - a
bioelectronic device in which genetically modified thyroid cells
produce thyroxine (T₄) in response to an applied electric field.

This repository contains the simulation code used to generate every
figure in the accompanying manuscript:

>Neural Network Control of Electric Field Induced Gene Regulatory
>Pathways with Significant Delays. 2026.
>Nicholas A. Rondoni, Papri Dey, Ksenia Zlobina, Marcella M. Gomez.


The controller addresses simultaneous obstacles that defeat
classical PID when directed at this plant: a
~14-hour mean transport delay, infrequent (20-minute) measurement,
hardware-imposed millisecond pulse constraints, plant-parameter
uncertainty, sensor noise, actuator gain mismatch, timing jitter,
and a periodic unmodeled disturbance.

---
## Repository layout
```
adaptiveControlRBFNN
├── Open_loop.m                      % Open-loop response of the plant
├── RBF_no_noise.m                   % Setpoint sweep, no noise
├── RBF_setpoint_sequence.m          % Sequential setpoint tracking (warm-start)
├── RBF_measurement_noise.m          % Sensor-noise sigma sweep
├── Supplement_unstable_example.m    % Naive RBF without stabilizing modifications
└── Monte_Carlo/
    ├── RBF_Monte_Carlo.m         % 100-seed Monte Carlo, all noise sources active
    └── MC_seed_viewer.m          % Re-plot a single Monte-Carlo seed
```

Each script is self-contained: the plant ODE, EF burst-averaging
helper, measurement filter, perturbation sampler, and plotting
utilities are defined as local functions at the bottom of the file.
There is intentional code duplication across scripts so that any
one can be run on its own without the others.

---

## File-to-figure map

| Script                               | Manuscript figure(s)                                                          |
|--------------------------------------|-------------------------------------------------------------------------------|
| `Open_loop.m`                        | Fig. 1 (open-loop T₄ response to EF bursts)                                  |
| `RBF_no_noise.m`                     | Fig. 3 (noise-free setpoint sweep, NRMSE ≈ 3–5 %)                            |
| `RBF_setpoint_sequence.m`            | Fig. 4 (sequential setpoints with warm-start)                                |
| `RBF_measurement_noise.m`            | Fig. 5 (sensor-noise σ sweep at setpoint 22)                                 |
| `Monte_Carlo/RBF_Monte_Carlo.m`      | Figs. 7, 8 (Monte Carlo summary, NRMSE swarm/box) and data backing Fig. 6    |
| `Monte_Carlo/MC_seed_viewer.m`       | Fig. 6 (single Monte-Carlo seed in detail)                                   |
| `Supplement_unstable_example.m`      | Supplement 4 (naive RBF — illustrates that the modifications are necessary)  |

---

## How to run

All scripts are standalone, though `MC_seed_viewer.m`
relies on the output of `RBF_Monte_Carlo.m`. The Monte Carlo
scripts live in `Monte_Carlo/` and should be run from within
that subfolder.

From the MATLAB prompt, with the repository folder on the path:
```matlab
run('script_name.m')
```
or just click **Run** in the editor.

Each script (except `Open_loop.m`) creates its own `results_*`
subfolder in the current working directory and writes intermediate
`.mat` files there. If a results file already exists for a given
configuration, the script loads it and skips re-simulation; delete
the file or change `ABLATION_MODE` / setpoint / σ tags to force a
fresh run.

---

## Configuration knobs

The relevant per-script knobs sit at the top of each file:

- `setpoint_values` — target T₄ values to sweep (e.g., `[12 16 20 25]`).
- `sigma_values` — sensor-noise standard deviation(s).
- `ABLATION_MODE` — one of `'none' | 'half' | 'full' | 'full_no_rhythm'`,
  selecting which subsets of noise sources are active. See the
  comment header in each script for definitions.
- `BAND_pct_val` — ± width of the band-lock window (default 0.20).
- `DO_SAVE_PDF` — `true` to export each figure as PDF into the
  results folder. Manual saving (`false`) is safer for good viewing bounds.

Default RNG seeds are set near the top (e.g. `RNGesus = 1` in some
scripts; `seed_list = 1:100` in the Monte Carlo).

Numerical parameter values match Tables 1–6 of the manuscript's
Supplement 1. For noise ranges, `ABLATION_MODE = half` replicates the
presented tables in supplement 1A, while `full` doubles these values.
