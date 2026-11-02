%% Quadcopter Simulation: System Parameters
% Parameters for the plant, FDI, FTC, wind, and noise models.
% Run this script to initialize the MATLAB base workspace.

%% 1. Quadcopter Parameters

% Physical properties
m   = (75 + 45.7)/1000;                      % kg      mass
l   = 54.33e-3;                              % m       arm length
Jxx = 54737.66e-9;                           % kg m^2  roll inertia
Jyy = 53435.96e-9;                           % kg m^2  pitch inertia
Jzz = 98934.3e-9;                            % kg m^2  yaw inertia
g   = 9.81;                                  % m/s^2

% Motor properties
kf  = 5e-8;                                  % N/(rad/s)^2   thrust coefficient
kt  = 0.025*kf;                              % N m/(rad/s)^2 torque coefficient
tau = 0.02;                                  % s             motor time constant
w_max = 2.5*sqrt(m*g/(4*kf));                % rad/s    rotor speed limit

% Nominal PD controller
Kp_r = 60*Jxx;   Kd_r = 12*Jxx;              % roll
Kp_p = 60*Jyy;   Kd_p = 12*Jyy;              % pitch
Kp_y = 3*Jzz;    Kd_y = 2.5*Jzz;             % yaw
Kp_pos  = 1.2;                               % [1/s^2] position gain
Kd_pos  = 1.8;                               % [1/s]   velocity gain
a_max   = 2.0;                               % m/s^2   limit on commanded acceleration
att_lim = 25*pi/180;                         % rad     roll / pitch command limit

% Initial conditions
p0  = [0; 0; 0];                             % m        initial position
v0  = [0; 0; 0];                             % m/s      initial velocity
q0  = [1; 0; 0; 0];                          % initial attitude quaternion
om0 = [0; 0; 0];                             % rad/s    initial body rates

% Reference commands and simulation settings
sim_stop_time = 12;                          % s; workspace simulation duration
sim_dt        = 0.001;                       % s   fixed solver step
x_step_time   = 3.0;   x_step_value = 10;    % x reference step
y_step_time   = 5.0;   y_step_value = 10;    % y reference step
z_step_time   = 0.5;   z_step_value = 10;    % z reference step
psi_step_time = 7.0;   psi_step_deg = 30;    % s, degrees
psi_step_value = deg2rad(psi_step_deg);      % rad

w_hover = sqrt(m*g/(4*kf));
fprintf('all_parameters: m = %.4f kg, l = %.4f m, hover speed = %.0f rad/s (limit %.0f rad/s)\n', m, l, w_hover, w_max);

%% 2. Fault Detection and Isolation (FDI)

% Observer 1 gains
L1_eul = [6; 6; 6];                          % correction gain on [phi;theta;psi] estimate
L1_om  = [12; 12; 8];                        % body rate estimate
L1_p   = [3; 3; 3];                          % position estimate
L1_v   = [4; 4; 4];                          % velocity estimate

% Observer 2 gains
L2_eul = L1_eul;  L2_om = L1_om;  L2_p = L1_p;  L2_v = L1_v;
rho_alpha = 0.5;                             % adaptation rate of alpha_hat

% Residual bounds: [roll; pitch; x; y; z]
Ku1 = [1; 1; 1; 1; 1];
Ku2 = [0.05; 0.05; 0.05; 0.05; 0.05];
Ku3 = [0.0004; 0.0004; 0.002; 0.002; 0.002]; 
Kl1 = Ku1;  Kl2 = Ku2;  Kl3 = Ku3;
T_res = 0.015;                               % s moving-average window of the residual bound

% Fault flag thresholds
T_flag1      = 0.01;                         % s
thresh_flag1 = 0.5;
T_flag2      = 0.1;                          % s
thresh_flag2 = 0.7;

% Fault confirmation
t_confirm = 0.5;                             % s both flags must read "normal" before Fault_True clears

%% 3. Fault-Tolerant Control (FTC)
confirm_alpha = 0.7;                         % alpha_hat of the faulty rotor must exceed this before the FTC latches
ftc_clip_w    = 20;                          % rad/s limit on the body-rate error in the recovery law
clip_w        = ftc_clip_w;

delta  = 0.5;                                % rad tilt of the thrust axis in the spinning hover
% State order: [position(3), velocity(3), roll, pitch, rates(3), live rotors(3)]
Qd = diag([1 1 1, 2 2 2, 20 20, 0.02 0.02 0.02, 1e-8 1e-8 1e-8]);
Rd = 1e-6*eye(3);                           
here_ = fileparts(mfilename('fullpath'));
S_ = load(fullfile(here_, 'ftc_params_x.mat'));
K_all = S_.K_all;  seq_all = S_.seq_all;  ueq_all = S_.ueq_all;

%% 4. Wind Disturbance Model
% Dryden-type gust model.
if ~exist('wind_seed','var'), wind_seed = 1; end % repeatable gust sequence
wind_on      = 0;                            % 1 = wind active, 0 = no wind
wind_t_start = 9;                            % s gusts switch on
wind_t_end   = 14;                           % s gusts switch off
rng(wind_seed);
wind_n      = 30;                            % sinusoids per axis
wind_Om_rng = [0.1 1.5];                     % rad/s
wind_alt    = 10;                            % m nominal altitude
wind_W20    = 10;                            % m/s mean wind at 20 ft
wind_v0     = [0; 0; 0];                     % m/s static wind vector, world frame
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
Cd_wind = 0.2*[1.0; 1.0; 0.5];               % N*s/m wind force = Cd .* wind velocity

disp([wind_on wind_t_start wind_t_end]);  disp(Cd_wind.');  disp(size(wind_A));

)), max(abs(F(:))));
end

%% 5. Measurement and Process Noise
noise_scale   = 0;                           % multiplies ALL sensor noise sigmas
proc_scale    = 1;                           % multiplies process-noise sigmas
noise_ctrl_on = 1;                           % 1 = noisy measurements also feed the controllers
noise_seed    = 1;                           % repeatable noise sequence
health4_final = 0;                           % rotor-4 health after t = 15 s (0 = fails)

% Sensor noise (one standard deviation)
sig_eul = [1.51e-3; 1.51e-3; 0.005];         % rad  attitude
sig_om  = 0.09*pi/180*ones(3,1);             % rad/s  gyro
sig_p   = 0.02*ones(3,1);                    % m      position
sig_v   = 0.02*ones(3,1);                    % m/s    velocity
Ts_att  = 1/200;                             % s      attitude / gyro noise update
Ts_pos  = 0.02;                              % s      position / velocity noise update

% Process disturbances
sig_pa  = 0.10*ones(3,1);                    % m/s^2    random acceleration
sig_pw  = 0.50*ones(3,1);                    % rad/s^2  random angular acceleration
Ts_proc = 0.005;                             % s

% Plant and observer mismatch
m_true          = m;                         % kg
tau_true        = tau;                       % s
unc_thrust_gain = 1;                         % = (kf_true/kf) * (m/m_true)
unc_M_gain      = [1;1;1];                   % [l*kf (roll); l*kf (pitch); kt (yaw)] true/nominal
unc_omdot_gain  = [1;1;1];                   % = [Jxx;Jyy;Jzz] ./ J_true

%% 6. Observer Protection
obs_guard = 1;

%% 7. FTC Latching and Detection Options
ftc_latch_t = 0.010;

% Optional rotor-health estimate rise check (disabled when window = 0).
ftc_rise_win = 0;
ftc_rise_lo  = 0.3;

%% 8. Position Integral Controller
Ki_pos   = 0.3;
xi_max   = 3.3;
pos_zone = [1; 1; 1];

%% 9. Attitude Integral Controller
Ki_r     = 20*Jxx;
Ki_p     = 20*Jyy;
xa_max   = 0.3;
att_zone = 0.2;
