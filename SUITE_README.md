# How to use the FDI / FTC test script (`fdi_ftc_test_suite.m`)

This guide is written in simple English. The script tests your quadcopter fault detection (FDI) and fault-tolerant control (FTC)
under wind, sensor noise and motor failures, and saves tables and graphs for your project report.

## 1. What you need

Put these files in the **same folder** as `fdi_ftc_test_suite.m` (they are your own files):

| File | Used for |
|---|---|
| `quad_params.m`, `fd_params.m`, `wind_params.m`, `noise_params.m`, `ftc_params.mat` | all parameters |
| `quadwind_noisy.slx` | system `'original'` |
| `quadwind_noisy_v3.slx` | system `'v3'` |
| `quadwind_noisy_v4.slx` and `ftc_params_x.mat` | system `'v4'` |

You need MATLAB R2026a (your models were saved with it), Simulink and the Control System Toolbox. No other toolbox.
Keep the folder path short. Windows cannot build Simulink files if the path is too long.

## 2. Quick start

```matlab
fdi_ftc_test_suite('Level','demo')        % 4 tests, about 1 minute: a good first check
fdi_ftc_test_suite                        % the standard set, about 136 tests, 10 to 15 minutes
```

The script prints its progress, one line per test, and a final summary:
`completed`, `failed` (solver error), `diverged` and `invalid`. Failed runs are never dropped. They stay in the tables.

## 3. Choose what to test

Open the script. The first block, **USER SETTINGS**, is where you choose everything. Every setting has its unit and meaning next to it.
You can also give a setting by name without editing the file:

```matlab
fdi_ftc_test_suite('Level','custom','Motors',[1 3],'WindSpeeds',[0.3 0.6],'NoiseLevels',[1 10],'NumCycles',5)
```

| Setting | What it does |
|---|---|
| `Level` | `'demo'` (4 tests), `'standard'` (planned set, about 136 tests), `'custom'` (every combination of your lists) |
| `System` | `'original'`, `'v3'` or `'v4'` |
| `NumCycles` | **how many cycles**: how many times each random test is repeated with a new random seed |
| `Seeds` | your own list of seeds (replaces `NumCycles`) |
| `SimTime`, `FaultTime` | run length and failure time, in seconds |
| `Motors`, `FailureSeverity` | which motors fail; 1 = complete failure, 0.6 = 60 % thrust lost |
| `WindSpeeds`, `WindDirections`, `WindTypes`, `GustOnsets`, `GustDurations`, `WindOnset`, `WindVerticalFraction` | the wind |
| `NoiseLevels`, `IncludeStressNoise`, `StressNoise`, `ProcessNoise`, `NoiseToControllers`, `Mismatch` | noise and model mismatch |
| `DetWindow`, `FtcWindow`, `RecAltTol`, `RecHorizTol`, `RecVzTol`, `RecTiltTolDeg`, `RecSettleTime` | when a test counts as a success |
| `ParamOverrides`, `WorkspaceOverrides` | change the system itself, for example `struct('thresh_flag2',0.6)` |
| `Tiers`, `Select`, `MaxRuns` | which groups to run, and a safety stop for very large lists |
| `Plots`, `DetailPlots`, `Fresh`, `Rebuild`, `Verify`, `OutputName` | output options |

**Cycles.** A test is "random" if it has sensor noise, process noise, turbulence or model mismatch. Random tests are repeated `NumCycles`
times with seeds 1, 2, 3 and so on. A test with nothing random is run once, because repeating it gives exactly the same answer.
The same seed always gives the same noise, so every result can be reproduced.

**Units.** Times in seconds, lengths in metres, wind in metres per second, wind direction in degrees (0 = forward, 90 = left),
angles in degrees where the name says `deg`.

## 4. How the noise and wind are made

* **Sensor noise** is white Gaussian noise with zero average. It is added to the attitude, body rates, position and speed that the detector
  receives (and to the controllers when `NoiseToControllers` is true). A noise **level** multiplies the sizes in `noise_params.m`
  (attitude 0.005 rad, body rates 0.005 rad/s, position 0.02 m, speed 0.02 m/s). Level 5 is 5 times those sizes. This is a fair scale-up,
  but it is not a claim that a particular sensor has that noise. Measure your own sensors if you need real numbers.
* **Process noise** (random pushes on the drone body: 0.1 m/s² and 0.5 rad/s²) is switched on whenever sensor noise is on. It is separate.
* **Wind** pushes with the force `Cd_wind × wind speed` (`Cd_wind` is in `wind_params.m`). Types: steady, gust (a smooth bump),
  turbulence (random, changing) and direction-changing wind.
* **Model mismatch** makes the real drone different from the model the controllers use (mass, inertia, thrust, torque, motor lag, arm length).

## 5. Where the results are

Everything goes in the folder `FDI_FTC_suite` next to the script:

```
FDI_FTC_suite/results/suite/<system>/<name>/
    tables/    scenarios.csv  perrun.csv  fdi.csv  ftc.csv  wind.csv  noise.csv  motor.csv  combined.csv  outcomes.csv  suite_tables.xlsx
    figures/   run_<test>.png/.pdf   compare_*.png/.pdf   summary_*.png/.pdf
    suite_results.mat    suite_log.txt
FDI_FTC_suite/results/suite/<system>/cache/    finished runs, reused when you run the same test again
```

The tables: `scenarios` = what was tested, `perrun` = one row per run with all metrics, `fdi` = detection and isolation summary,
`ftc` = takeover and recovery summary, `wind` / `noise` / `motor` = comparisons, `combined` = wind, noise and failure together.

## 6. How to read the results

Every run has one **outcome** label and one **status**.

| Outcome | Meaning |
|---|---|
| `healthy_ok` | no failure, no alarm, stable flight |
| `false_alarm` | no failure, but a confirmed fault or an FTC takeover |
| `unstable_healthy` | no failure, no alarm, but the flight left the allowed limits |
| `false_alarm_before_fault` | a confirmed fault or FTC takeover before the failure happened |
| `missed_detection` | the failure was never confirmed |
| `late_detection` | confirmed, but later than `DetWindow` |
| `misisolation` | the wrong motor was found |
| `ftc_activation_failure` | found in time, but the FTC did not take over in time |
| `recovered` | all of the above fine, and the flight settled inside the limits |
| `safe_descent` | no hover recovery, but a controlled landing (soft, small tilt, near the target) |
| `unsuccessful_recovery` | none of the above |
| `diverged`, `simulation_failed` | numbers blew up, or the solver stopped with an error |

`status` says if the simulation itself `completed`, `failed`, `diverged` or is `invalid`. A run is never counted as a success only because it finished.

## 7. What each metric means (short)

| Metric | Meaning |
|---|---|
| **Detection time** | time from the failure to the first confirmed fault. "Confirmed" = `Fault_True` is on **and** the thrust-loss estimate of the found motor is above `confirm_alpha` (0.7). Your FTC uses the same rule. The raw `Fault_True` flag is reported separately (`ft_on_frac`) because noise can keep it on all the time. |
| **Found in time** | confirmed within `DetWindow` of the failure, with no alarm before it |
| **Right motor** | the motor found at the moment of confirmation is the motor that failed |
| **False alarm** | a confirmed fault (or an FTC takeover) while nothing was wrong; the number of separate alarms is also saved |
| **FTC start time / delay** | the first time the motor commands differ from the normal controller; delay = that time minus the failure time. Switching delay = FTC start minus detection time |
| **Recovery** | all four conditions hold together until the end of the run, for at least `RecSettleTime`: height error ≤ `RecAltTol`, horizontal distance ≤ `RecHorizTol`, climb speed ≤ `RecVzTol`, roll and pitch within `RecTiltTolDeg` of the planned recovery tilt. Yaw is not checked, because the drone spins about the vertical on purpose |
| **Recovery time** | time from the FTC start until the start of that final stable period (0 if it was stable at once) |
| **Max roll / pitch deviation** | largest angle away from level after the failure; also away from the planned tilt after the FTC start |
| **Height lost** | height at the failure minus the lowest height afterwards |
| **Drift** | largest distance to the target position after the failure (also: distance from the place where the failure happened) |
| **Motor limit** | time at the upper limit and at zero, and the number of separate times a working motor hit a limit |
| **Control effort** | total change of the motor commands (sum of absolute steps ÷ w_max) and the squared size of the commands (integral of the normalised squared deviation from hover) |
| **RMS tracking error** | roll/pitch and 3-D position error over the 3 s before the failure, and over the last 3 s of the run |
| **Safe descent** | only checked when hover recovery failed: touchdown (height below 0.2 m) with sink speed ≤ 2 m/s, tilt ≤ 60° and distance ≤ 10 m |

## 8. Method in a few words

1. The script copies your model and adds recording blocks. The fault time, failed motor, severity, wind and noise are set from MATLAB for each run.
2. It checks that the copy gives exactly the same results as your model (difference zero).
3. It runs every test, saves each finished run, and computes the metrics from the full-speed signals.
4. It groups the runs into the tables and draws the graphs. Every graph marks the failure, the detection, the FTC start and the start of stable flight.

## 9. Changes made to your model (in the test copy only)

Your model files are never changed. The test copy has these changes:

* the four health step blocks read their time and final value from workspace variables (this is how the failure is injected);
* extra recording blocks: `health_log`, `pref_log`, `wnom_log`, `wcmd_log`, and the detector thresholds `fd_rupper`, `fd_rlower`;
* the wind block has one extra input, a gust table (with an empty table it behaves exactly like your wind block);
* the model's scopes are switched off for speed, and its PreLoad and Init callbacks are replaced by the script, which sets all the variables;
* for `'original'`, `'v3'` and `'v4'` the settings `obs_guard`, `ftc_latch_t` and the PID gains are variables. With the original values (0) the behaviour is the same as your original model.

## 10. Limits to remember

* Few runs per group: percentages move a lot if one run changes. 100 % is not a guarantee.
* One failure at a time, at one chosen time. White noise only (no bias, drift or delay).
* The model has no ground. `Cd_wind` is large for a drone of this size, so wind speeds look small.
* The success limits and the noise levels are my choices. Change them in the settings if you disagree.
* The script was run on your three models. Its test copy was checked to match each one exactly. The results summary is in `report_suite/suite_results_summary.pdf`.
