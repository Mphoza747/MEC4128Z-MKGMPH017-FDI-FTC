# LQR design: spinning-hover gains

This folder holds the code that **calculates** the LQR gains used by the fault-tolerant controller. The gains themselves (`K_all`, `seq_all`, `ueq_all`) are stored in `model/ftc_params_x.mat`, which the Simulink model loads.

| File | What it does |
|---|---|
| `ftc_design_delta.m` | The design function. For each failed motor it (1) solves for the spinning-hover equilibrium with a Newton iteration, (2) linearises the 14-state model about that equilibrium (A, B), and (3) calls `lqr(A, B, Qd, Rd)`. This is the same equilibrium solver and LQR as `ftc_design_param` inside `campaign/fdi_ftc_test_suite.m`, with the tilt `delta` as an input. |
| `run_lqr_design.m` | Driver. Loads the vehicle parameters from `model/combined_params_all.m`, runs the design with the report's settings, prints the spin rate, closed-loop eigenvalues and controllability rank, compares K with `model/ftc_params_x.mat` and saves `ftc_params_x_recomputed.mat`. Needs MATLAB with the Control System Toolbox. |
| `ftc_design_check.py` | Independent Python re-implementation (numpy and scipy, uses the Riccati solver). Recomputes the equilibrium, linearisation, gains, controllability and eigenvalues, and compares them with the stored gains. Run `python3 -I ftc_design_check.py` from this folder. |

## Settings used in the report
- X layout, spin direction `natural` (the way the lost yaw torque pushes), tilt delta = 0.5 rad toward the failed motor.
- State weights `Qd = diag([1 1 1, 2 2 2, 20 20, 0.02 0.02 0.02, 1e-8 1e-8 1e-8])` (position, velocity, roll and pitch, body rates, live motor speeds), input weight `Rd = 1e-6 * I`.
- Vehicle: m = 0.1207 kg, arm 54.33 mm, kf = 5e-8, kt = 0.025 kf, motor time constant 0.02 s, Jxx = 54737.66e-9, Jyy = 53435.96e-9, Jzz = 98934.3e-9 kg m^2.

## Check result
The Python check reproduces the stored gains for all four motors (largest relative difference about 1.5e-10). The equilibrium spin rate is 44.33 rad/s, the slowest closed-loop eigenvalue is -0.72, and the linearised pair is controllable in the sense used in the report.

`run_lqr_design.m` was written to the same settings but has not been run in MATLAB by the author of this README, so run it once and check that the printed relative differences are near zero.
