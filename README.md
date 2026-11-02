# Fault detection and recovery control for a quadcopter under single motor failure


This repository holds everything needed to verify or repeat the analyses in the final report: the Simulink models, the parameter and test scripts, the results of the 136-run simulation campaign, the analysis scripts, the animation videos, the report source, the poster and the conference slides.

## What the project does

A quadcopter has no spare motors, and three motors cannot hold a static hover. This project designs and tests, in a nonlinear MATLAB/Simulink simulation, a scheme that:

1. **Detects** a single-motor failure with a dual-observer detector (Observer 1: healthy model with moving-average residual bounds, Flag 1; Observer 2: per-motor loss estimates, Flag 2).
2. **Isolates** the failed motor, confirms it (loss estimate above 0.7 held for 10 ms) and latches it.
3. **Recovers** bounded flight with a spinning-hover linear quadratic regulator (LQR) on the three remaining motors. Yaw is given up and the vehicle spins at 44.3 rad/s while its position is held.

The 17-state model uses m = 0.1207 kg, arm length 54.33 mm and a 20 ms motor time constant, with a fixed-step `ode4` solver at 1 ms.

## Headline results (136-run campaign: 49 fault-free, 87 failure runs)

| Result | Value |
|---|---|
| Failures detected, isolated and recovered | 87 of 87 |
| Confirmed false alarms in fault-free runs | 0 of 49 |
| Detection delay | 5 to 8 ms |
| Switching delay (confirmation hold) | 9 ms |
| Failure to recovery controller active | 14 to 17 ms |
| Recovery time after activation | typically 0.10 to 0.14 s (median about 125 ms) |
| Altitude lost | at most 10 cm (29 cm with model error) |
| Horizontal drift (without model error) | at most 47 cm |

The results hold only over the tested range: wind of at most 0.1 N, sensor-noise level up to 10, one model-error setting and one partial-loss severity (60 %). The model omits gyroscopic torque and airframe drag. There is no baseline-only comparison and no recovery-deadline or design sweep. See the report for the full discussion.

## Repository layout

```
model/            Simulink models and parameters
  FDI_FTC_MKGMPH017.slx     Submitted model: nonlinear 6-DOF quadcopter, baseline controller, FDI, FTC, wind, noise
  quad_full_6dof.slx        Plain 6-DOF quadcopter without FDI/FTC (used for the baseline video)
  combined_params_all.m     Parameter script stored with the submitted model
  quad_params.m             Physical parameters used by the animation
  ftc_params_x.mat          Stored LQR gains, equilibrium speeds and sequences for the four failed motors

campaign/         The 136-run test campaign
  fdi_ftc_test_suite.m      Builds a test copy of the model and runs the campaign
  all_parameters.m          Parameter file used for the 136 runs
  quadwind_noisy_v4.slx     Model version the suite builds its test copy from
  ftc_params_x.mat, ftc_params.mat   FTC parameter files used by the suite
  SUITE_README.md           How to use the test suite
  FLAG1_THRESHOLD_STUDY.md  Flag 1 threshold study
  results/                  suite_results.mat, per-run figures, tables and suite_log.txt

animation/        3D animation and simulation videos
  Animate3D_Quad.m          3D animation of position, attitude, motor speeds and trajectory
  run_model_videos.m        Records the videos below (motor 4 fails at 14 s, 20 s simulation)
  videos/                   1_baseline_6dof_no_FDI_FTC.mp4        baseline, vehicle loses control
                            2_FDI_FTC_MKG.mp4                     with FDI + FTC, follow camera
                            2b_FDI_FTC_MKG_overview.mp4           with FDI + FTC, fixed overview camera
                            2c_FDI_FTC_MKG_slowmo_13s-20s.mp4     with FDI + FTC, slow motion 13 s to 20 s

analysis_scripts/ MATLAB plotting scripts (graphs.m, plot_quadwind_results.m, plot_quadwind_results_named.m)

```

## How to repeat the work

**1. Simulation (MATLAB/Simulink R2026a, fixed-step `ode4`, 1 ms).**
Copy the files of `campaign/` and `model/` into one short folder (a long Windows path breaks Simulink model building), run `all_parameters.m`, then `fdi_ftc_test_suite.m`. A one-minute check is `fdi_ftc_test_suite('Level','demo')`; the standard set is about 136 runs. See `campaign/SUITE_README.md`.
To run the submitted model once, run `combined_params_all.m`, open `FDI_FTC_MKGMPH017.slx` and simulate. Its stored settings (sensor-noise level 20, wind on, wind coefficient 0.5, failure at 15 s) differ from the campaign (wind coefficients 0.2 and 0.1, noise levels 0 to 10, failure at 14 s, stop at 26 s).

**2. Animation videos.** Put `model/`, `animation/` and the model files in one folder and run `run_model_videos`. The original models are not modified; each is simulated from a temporary copy.

**3. Analysis (Python 3 with numpy, pandas, matplotlib, scipy).** From `report/source/`:

```
python3 -I analysis/recompute_results.py     # regenerates every table, chart and numbers.json from data/perrun.csv
python3 -I analysis/ftc_design_check.py      # independent check of the spinning-hover equilibrium and LQR gains against model/ftc_params_x.mat
```

**4. Report.** From `report/source/`, run `pdflatex report`, `biber report`, `pdflatex report`, `pdflatex report` (UCT `UCTebe-report` class). The word count method is in `report/source/WORD_COUNT.md`.

## Notes

- `ftc_design_check.py` in this repository reads `model/ftc_params_x.mat` by relative path.
- The video frames label the failed motor "ROTOR 4"; the report and slides use "motor".
- Files generated by MATLAB and LaTeX (`slprj/`, `*.slxc`, `*.aux` and so on) are excluded by `.gitignore`.

## Use of AI

Anthropic's Claude was used for code development and debugging (test suite, numerical iteration in the LQR design, animation code, PD and I controllers), checking the model, grammar and punctuation, diagrams, word counting, help with LaTeX and summarising papers. The full declaration is in the report.

## Licence

No licence has been chosen. Unless one is added, all rights are reserved by the author.
