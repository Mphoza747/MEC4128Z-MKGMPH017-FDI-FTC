%% all_parameters.m
% Every parameter of the quadcopter / FDI / FTC / wind / noise system in ONE file.
% It defines exactly the workspace variables that quad_params.m, fd_params.m, wind_params.m,
% noise_params.m, ftc_params.mat and the model's InitFcn define, plus the new variables added
% for quadwind_noisy_v2.slx.  Run it instead of those five files (it was checked to give the
% same values, see the bottom of this file).
%
% HOW CHANGES ARE MARKED
%   * MATLAB cannot bold text inside code, so every changed or new parameter sits in its own
%     section whose title starts with  "CHANGED"  or  "NEW"  (MATLAB's editor shows section titles in BOLD),
%     and its line ends with  "<-- CHANGED"  /  "<-- NEW".
%   * The old value is always in a comment:   % OLD: ...
%   * "TESTED, NOT ADOPTED" = I tried a new value, it did not hold up on unseen test cases, so the
%     value stays as before; the tried value is in a comment so you can see it.
%
% SUMMARY OF WHAT WAS ACTUALLY CHANGED IN THE MODEL (quadwind_noisy_v3.slx = v2 + PID)
%   NEW  obs_guard     = 1      (observer numerical guard; 0 = your original behaviour)          [v2]
%   NEW  ftc_latch_t   = 0.010  (FTC latch hold in seconds; 0 = your original behaviour)         [v2]
%   NEW  ftc_rise_win  = 0      (optional check, tested, no benefit, left off)                   [v2]
%   NEW  ftc_rise_lo   = 0.3    (only used if ftc_rise_win > 0)                                  [v2]
%   NEW  Ki_pos = 0.3, xi_max = 3.3, pos_zone = [1;1;1]       position / height integral gain     [v3]
%   NEW  Ki_r = 20*Jxx, Ki_p = 20*Jyy, xa_max = 0.3, att_zone = 0.2   roll / pitch integral gain  [v3]
% Everything else keeps its original value.

%% ===================== QUAD (was quad_params.m) =====================
% Physical parameters -- unchanged
m   = (75 + 45.7)/1000;     % kg      mass                           (unchanged)
l   = 54.33e-3;            % m       arm length                      (unchanged)
Jxx = 54737.66e-9;         % kg m^2  roll inertia                    (unchanged)
Jyy = 53435.96e-9;         % kg m^2  pitch inertia                   (unchanged)
Jzz = 98934.3e-9;          % kg m^2  yaw inertia                     (unchanged)
g   = 9.81;                % m/s^2                                   (unchanged)

% Motor -- unchanged
kf  = 5e-8;                % N/(rad/s)^2   thrust coefficient        (unchanged)
kt  = 0.025*kf;            % N m/(rad/s)^2 torque coefficient        (unchanged)
tau = 0.02;                % s             motor time constant       (unchanged)
w_max = 2.5*sqrt(m*g/(4*kf));   % rad/s    rotor speed limit         (unchanged)

% Nominal controller gains -- unchanged (attitude PD in torque units, position PD in acceleration units)
Kp_r = 60*Jxx;   Kd_r = 12*Jxx;     % roll    (unchanged; not part of the recovery study)
Kp_p = 60*Jyy;   Kd_p = 12*Jyy;     % pitch   (unchanged)
Kp_y = 3*Jzz;    Kd_y = 2.5*Jzz;    % yaw     (unchanged)
Kp_pos  = 1.2;                      % [1/s^2] position gain          (unchanged)
Kd_pos  = 1.8;                      % [1/s]   velocity gain          (unchanged)
a_max   = 2.0;                      % m/s^2   limit on commanded acceleration (unchanged; it limits wind rejection, see report)
att_lim = 25*pi/180;                % rad     roll / pitch command limit      (unchanged; same remark)

% Initial conditions -- unchanged
p0  = [0; 0; 0];           % m        initial position
v0  = [0; 0; 0];           % m/s      initial velocity
q0  = [1; 0; 0; 0];        %          initial attitude quaternion
om0 = [0; 0; 0];           % rad/s    initial body rates

% Test scenario -- unchanged
sim_stop_time = 12;        % s   (the model's own StopTime is 20 s; the campaign copy uses 26 s)
sim_dt        = 0.001;     % s   fixed solver step
x_step_time   = 3.0;   x_step_value = 10;      % m   (the model's x_ref block uses 9 s -> 3 m)
y_step_time   = 5.0;   y_step_value = 10;      % m   (the model's y_ref block uses 6 s -> -5 m)
z_step_time   = 0.5;   z_step_value = 10;      % m   (the model's z_ref block uses 0.5 s -> 5 m)
psi_step_time = 7.0;   psi_step_deg = 30;      % s, degrees
psi_step_value = deg2rad(psi_step_deg);        % rad
% Values written in the model blocks (not workspace variables): x_ref 9 s -> 3 m, y_ref 6 s -> -5 m,
% z_ref 0.5 s -> 5 m. Fault blocks: health_1 3 s, health_2 15 s, health_3 4 s (all stay 1 = no fault),
% health_4 steps to health4_final at 15 s.

w_hover = sqrt(m*g/(4*kf));
fprintf('all_parameters: m = %.4f kg, l = %.4f m, hover speed = %.0f rad/s (limit %.0f rad/s)\n', m, l, w_hover, w_max);

%% ===================== FDI (was fd_params.m) =====================
% Observer 1 correction gains -- unchanged
L1_eul = [6; 6; 6];     % correction gain on [phi;theta;psi] estimate (unchanged; searched: scale 0.50 tested, NOT ADOPTED)
L1_om  = [12; 12; 8];   % body rate estimate                           (unchanged; same)
L1_p   = [3; 3; 3];     % position estimate                            (unchanged; same)
L1_v   = [4; 4; 4];     % velocity estimate                            (unchanged; same)

% Observer 2 gains -- unchanged
L2_eul = L1_eul;  L2_om = L1_om;  L2_p = L1_p;  L2_v = L1_v;
rho_alpha = 0.5;        % adaptation rate of alpha_hat (unchanged; searched best 0.579, TESTED, NOT ADOPTED)

% Adaptive residual threshold gains, residual order [phi; theta; x; y; z] -- unchanged
Ku1 = [1; 1; 1; 1; 1];
Ku2 = [0.05; 0.05; 0.05; 0.05; 0.05];
Ku3 = [0.0004; 0.0004; 0.002; 0.002; 0.002];   % unchanged; searched: x2.17 tested, NOT ADOPTED
Kl1 = Ku1;  Kl2 = Ku2;  Kl3 = Ku3;
T_res = 0.015;          % s moving-average window of the residual bound (unchanged)

% Flag thresholds -- unchanged
T_flag1      = 0.01;    % s   (unchanged; searched best 0.049, TESTED, NOT ADOPTED)
thresh_flag1 = 0.5;     %     (unchanged; searched best 0.548, TESTED, NOT ADOPTED)
T_flag2      = 0.1;     % s   (unchanged; searched best 0.091, TESTED, NOT ADOPTED)
thresh_flag2 = 0.7;     %     (unchanged; searched best 0.779, TESTED, NOT ADOPTED)

% Judgment -- unchanged
t_confirm = 0.5;        % s both flags must read "normal" before Fault_True clears (unchanged)
% In the WIND model (quadWithWind_dryden) the Judgment chart also has constants written inside it:
%   t_startup_grace = 1.0 s, t_confirm = 0.5 s, t_sustain = 1.0 s   (unchanged; t_sustain 1.00 searched best, NOT ADOPTED)

%% ===================== FTC (was ftc_params.mat and the model's InitFcn) =====================
% InitFcn of the model:   load ftc_params.mat; confirm_alpha = 0.7; clip_w = ftc_clip_w;
confirm_alpha = 0.7;    % alpha_hat of the faulty rotor must exceed this before the FTC latches (unchanged)
ftc_clip_w    = 20;     % rad/s limit on the body-rate error in the recovery law                 (unchanged)
clip_w        = ftc_clip_w;

% LQR design weights of ftc_design.m (these produce K_all, seq_all, ueq_all stored in ftc_params.mat)
delta  = 0.5;           % rad tilt of the thrust axis in the spinning hover (unchanged)
% State order: [pos(3) vel(3) phi theta om(3) w_live(3)]
Qd = diag([1 1 1, 2 2 2, 20 20, 0.02 0.02 0.02, 1e-8 1e-8 1e-8]);   % unchanged
%   TESTED, NOT ADOPTED (best of 41 searched designs; on unseen cases recovery 83.9 % -> 85.7 %, only 2 of 112 runs):
%   Qd = diag([4.92 4.92 1.93, 2*0.439 2*0.439 2*2.72, 20*1.92 20*1.92, 0.02*4.62 0.02*4.62 0.02*4.62, 1e-8 1e-8 1e-8]);
Rd = 1e-6*eye(3);                                                    % unchanged
%   TESTED, NOT ADOPTED: Rd = 1.69e-6*eye(3);
% Gain matrices (3 x 14 x 4), equilibrium states and inputs of the recovery law: loaded from your file, unchanged
here_ = fileparts(mfilename('fullpath'));
S_ = load(fullfile(here_, 'ftc_params_x.mat'));   % <-- CHANGED (quadwind_noisy_v4). OLD: 'ftc_params.mat' (plus layout, spin always +z)
K_all = S_.K_all;  seq_all = S_.seq_all;  ueq_all = S_.ueq_all;    % <-- CHANGED: redesigned for the 45-degree body frame with the natural spin direction
% OLD: the same three variables from ftc_params.mat (plus layout, equilibrium spin always +z). That file is kept.
% NEW layout and spin: equilibrium tilt of the thrust axis toward the dead rotor, now diagonal:
%   rotor 1: [phi theta] = [0.346  0.369]   rotor 2: [0.346 -0.369]   rotor 3: [-0.346 -0.369]   rotor 4: [-0.346  0.369]  rad
%   (OLD: rotor 1 [0 0.5], rotor 2 [0.5 0], rotor 3 [0 -0.5], rotor 4 [-0.5 0])
%   equilibrium spin about the vertical: rotor 1 -38.9, rotor 2 +38.9, rotor 3 -38.9, rotor 4 +38.9 rad/s (OLD: +39.0/+38.4 for all four)
% (to regenerate them: quad_params; ftc_design  -- or ftc_design_param.m in campaign/ with all weights = 1)

%% ===================== WIND (was wind_params.m) =====================
% Dryden-type gusts (Hancer et al., CDC 2010, Eqs. 15-19) -- all unchanged
if ~exist('wind_seed','var'), wind_seed = 1; end   % repeatable gust sequence
wind_on      = 1;          % 1 = wind active, 0 = no wind           (unchanged)
wind_t_start = 9;         % s gusts switch on                      (unchanged)
wind_t_end   = 14;         % s gusts switch off                     (unchanged)
rng(wind_seed);
wind_n      = 30;          % sinusoids per axis                     (unchanged)
wind_Om_rng = [0.1 1.5];   % rad/s                                  (unchanged)
wind_alt    = 10;          % m nominal altitude                     (unchanged)
wind_W20    = 10;          % m/s mean wind at 20 ft                 (unchanged)
wind_v0     = [0; 0; 0];   % m/s static wind vector, world frame    (unchanged)
wind_k    = 0.177 + 0.000823*(wind_alt*3.28084);
wind_Lv   = wind_alt;
wind_Lh   = wind_Lv/wind_k^1.2;
wind_sigv = 0.1*wind_W20;
wind_sigh = wind_sigv/wind_k^0.4;
wind_Om  = sort(wind_Om_rng(1) + diff(wind_Om_rng)*rand(3, wind_n), 2);
wind_Phi = 2*pi*rand(3, wind_n);
wind_dOm = gradient(wind_Om);
wind_L   = [wind_Lh; wind_Lh; wind_Lv];
wind_sig = [wind_sigh; wind_sigh; wind_sigv];
wind_PSD = zeros(3, wind_n);
wind_PSD(1:2,:) = (wind_sig(1:2).^2 .* 2.*wind_L(1:2)/pi) ./ (1 + (wind_L(1:2).*wind_Om(1:2,:)).^2);
wind_PSD(3,:)   = (wind_sig(3)^2 * 2*wind_L(3)/pi) * (1 + 3*(wind_L(3)*wind_Om(3,:)).^2) ./ (1 + (wind_L(3)*wind_Om(3,:)).^2).^2;
wind_A = sqrt(wind_dOm .* wind_PSD);
Cd_wind = 0.2*[1.0; 1.0; 0.5];  % N*s/m wind force = Cd .* wind velocity (unchanged; "TUNE THIS" -- 0.5 N per m/s is very large
                                % for a 120 g drone; a real value is nearer 0.02-0.05. Left as you set it.)

disp([wind_on wind_t_start wind_t_end]);  disp(Cd_wind.');  disp(size(wind_A));

out = sim('quadwind_noisy_v4');            % fresh run, so out is not stale
w = out.wind_log;  t = w.Time;  F = squeeze(w.Data);
if size(F,1) ~= numel(t), F = F.'; end       % N x 3, time down the rows

nz = find(any(abs(F) > 1e-9, 2));
if isempty(nz)
    disp('F_wind is zero for the whole run');
else
    fprintf('F_wind nonzero from %.2f s to %.2f s, max %.2f N\n', t(nz(1)), t(nz(end)), max(abs(F(:))));
end

%% ===================== NOISE (was noise_params.m) =====================
noise_scale   = 1;   % multiplies ALL sensor noise sigmas (unchanged file default; the tests ran 0, 1, 2, 5, 10 and 20)
proc_scale    = 1;    % multiplies process-noise sigmas             (unchanged)
noise_ctrl_on = 1;    % 1 = noisy measurements also feed the controllers (unchanged)
noise_seed    = 1;    % repeatable noise sequence                   (unchanged)
health4_final = 0;    % rotor-4 health after t = 15 s (0 = fails)   (unchanged)

% Sensor noise, 1-sigma per axis -- unchanged
sig_eul = [1.51e-3; 1.51e-3; 0.005];   % rad  attitude  <-- CHANGED. Roll/pitch = accel tilt noise: 1.51 mg-rms (datasheet, ODR 200 Hz) / 1 g = 1.51e-3 rad. Yaw has no reference in an IMU without a magnetometer, so it keeps its OLD value 0.005. OLD: 0.005*ones(3,1)
sig_om  = 0.09*pi/180*ones(3,1);   % rad/s  gyro  <-- CHANGED (BMI270 datasheet: 0.09 dps-rms, normal mode, ODR = 200 Hz, BW = 74.6 Hz = 1.571e-3 rad/s). OLD: 0.005*ones(3,1)
% Not in the BMI270 datasheet, left unchanged: sig_p, sig_v, Ts_pos (position / velocity come from other sensors),
%   sig_pa, sig_pw, Ts_proc (process noise).  Datasheet accelerometer noise (0.16 mg/sqrt(Hz), 1.51 mg-rms at 200 Hz) has no matching variable.
sig_p   = 0.02*ones(3,1);    % m      position
sig_v   = 0.02*ones(3,1);    % m/s    velocity
Ts_att  = 1/200;              % s      attitude / gyro noise update  <-- CHANGED (BMI270 ODR = 200 Hz, same ODR as the noise figures). OLD: 0.004
Ts_pos  = 0.02;              % s      position / velocity noise update

% Process noise -- unchanged
sig_pa  = 0.10*ones(3,1);    % m/s^2    random acceleration
sig_pw  = 0.50*ones(3,1);    % rad/s^2  random angular acceleration
Ts_proc = 0.005;             % s

% Plant-vs-observer mismatch (1 = nominal) -- unchanged
m_true          = m;         % kg
tau_true        = tau;       % s
unc_thrust_gain = 1;         % = (kf_true/kf) * (m/m_true)
unc_M_gain      = [1;1;1];   % [l*kf (roll); l*kf (pitch); kt (yaw)] true/nominal
unc_omdot_gain  = [1;1;1];   % = [Jxx;Jyy;Jzz] ./ J_true

%% CHANGED: body frame (quadwind_noisy_v4.slx) -- axes rotated 45 degrees away from the rotor arms
% OLD (plus layout, body axes along the arms): rotor 1 front (+x), 2 right (-y), 3 rear (-x), 4 left (+y)
%     Mx = l*kf*(w4^2 - w2^2),  My = l*kf*(w3^2 - w1^2),  Mz = kt*(w1^2 - w2^2 + w3^2 - w4^2)
% NEW (45 degrees): rotor positions in the body frame (l/sqrt(2))*[ 1 -1; -1 -1; -1 1; 1 1 ]
%     Mx = (l/sqrt(2))*kf*(-w1^2 - w2^2 + w3^2 + w4^2)
%     My = (l/sqrt(2))*kf*(-w1^2 + w2^2 + w3^2 - w4^2)
%     Mz = kt*(w1^2 - w2^2 + w3^2 - w4^2)               (unchanged)
% Changed in the model: Rotor_Forces, Control_Allocation, Obs2_Deriv (moment prediction); no new workspace variable.
% Jxx, Jyy, Jzz are used as they are (the small cross term from rotating a slightly unsymmetric inertia is neglected).

%% NEW: obs_guard  (observer numerical guard, quadwind_noisy_v2.slx)
% Wraps angle errors to [-pi, pi], uses tan(theta) = sin(theta)/cth, limits the body-rate estimate to +-300 rad/s,
% limits alpha_hat to [0,1], replaces non-finite derivatives by 0.
obs_guard = 1;          % <-- NEW   (OLD: variable did not exist = original unguarded observers; 0 reproduces that)

%% NEW: ftc_latch_t  (FTC latch hold)
% The FTC latches onto a "failed" rotor only after Fault_True and alpha_hat(faulty rotor) > confirm_alpha
% have held, for the same rotor, for this long.
ftc_latch_t = 0.010;    % <-- NEW   s   (OLD: none = latched on the very first sample, same as 0)
%   Tested at 20x noise: 0 ms -> 25 % recovered, 10 ms -> 100 %, 20 ms -> 100 %, 30 ms -> 92 %, 50 ms -> 96 %.

%% NEW: ftc_rise_win and ftc_rise_lo  (optional "estimate rose quickly from low" check -- tested, NO BENEFIT, left off)
ftc_rise_win = 0;       % <-- NEW   s   (OLD: did not exist; 0 = off.  0.1 s was tested: no effect)
ftc_rise_lo  = 0.3;     % <-- NEW       only used when ftc_rise_win > 0

%% NEW: integral gains of the position / height loop (PID, quadwind_noisy_v3.slx)
% a = Kp_pos*(p_ref - p) - Kd_pos*v + Ki_pos*xi ;  xi = integral of the position error (anti-windup, see campaign/harden_pid.m)
Ki_pos   = 0.3;         % <-- NEW  [1/s^3]  (OLD: did not exist = pure PD; 0 reproduces that)
xi_max   = 3.3;         % <-- NEW  [m s]    clamp of the integral state (Ki_pos*xi_max = 1 m/s^2 at most)
pos_zone = [1; 1; 1];   % <-- NEW  [m]      integrate only while |error| is below this (x, y, z); stops wind-up in reference steps
%   Tested: Ki_pos 0.15 / 0.3 / 0.6 and zones 1 / 2.5 / 4 m. 0.3 with 1 m gave the best balance (see report).

%% NEW: integral gains of the attitude loop (roll and pitch only, quadwind_noisy_v3.slx)
% M = Kp*(angle_cmd - angle) - Kd*rate + Ki*integral(angle error); yaw has NO integral term.
Ki_r     = 20*Jxx;      % <-- NEW  (OLD: did not exist = pure PD; 0 reproduces that)   roll
Ki_p     = 20*Jyy;      % <-- NEW  (OLD: did not exist)                                  pitch
xa_max   = 0.3;         % <-- NEW  [rad s]  clamp of the attitude integral state
att_zone = 0.2;         % <-- NEW  [rad]    integrate only while |angle error| is below this
%   Tested: 10, 20, 40 times the inertia; 20 chosen (40 gave little extra).

%% ===================== Values tried but NOT adopted (for reference only) =====================
% Best detector / recovery gain scales found on the wind model (they scored 3.85 vs 3.17 for the current values on
% unseen runs, so the current values above stay):
%   K_all(:,1:3,:) x1.17   K_all(:,4:6,:) x0.87   K_all(:,7:8,:) x0.68   K_all(:,9:11,:) x1.22
%   Ku3,Kl3 x2.17   thresh_flag1 0.548   thresh_flag2 0.779   T_flag1 0.049   T_flag2 0.091
%   rho_alpha 0.579   L1_*,L2_* x0.50   t_sustain 1.00 (wind-model Judgment only)
% None of these is applied anywhere.

%% ===================== Check (optional) =====================
% To confirm this file reproduces your original parameter files, run in a clean workspace:
%   clear; quad_params; fd_params; wind_params; noise_params; load ftc_params.mat; confirm_alpha = 0.7; clip_w = ftc_clip_w;
%   A = who; save('orig.mat'); clear; all_parameters; B = who; ... compare the variables in A with orig.mat.