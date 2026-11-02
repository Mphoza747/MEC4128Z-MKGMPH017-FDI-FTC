
m   = (75 + 45.7)/1000;     % kg     
l   = 54.33e-3;            % m       
Jxx = 54737.66e-9;         % kg m^2  
Jyy = 53435.96e-9;         % kg m^2
Jzz = 98934.3e-9;          % kg m^2
g   = 9.81;                % m/s^2

%MOTOR

kf  = 5e-8;                % N/(rad/s)^2    
kt  = 0.025*kf;            % N m/(rad/s)^2 
tau = 0.02;                % s              
w_max = 2.5*sqrt(m*g/(4*kf));   % rad/s     

% CONTROLLER GAINS
% Attitude loops (PD on angle + damping on body rate), torque units so they scale with inertia
Kp_r = 60*Jxx;   Kd_r = 12*Jxx;     % roll   
Kp_p = 60*Jyy;   Kd_p = 12*Jyy;     % pitch
Kp_y = 3*Jzz;    Kd_y = 2.5*Jzz;    % yaw
% Position loop (acceleration domain, independent of mass)
Kp_pos  = 1.2;                      % [1/s^2]
Kd_pos  = 1.8;                      % [1/s]
a_max   = 2.0;                      % m/s^2   limit on commanded acceleration
att_lim = 25*pi/180;                % rad     limit on roll / pitch command

% INITIAL CONDITIONS
p0  = [0; 0; 0];           % m        initial position [x; y; z]
v0  = [0; 0; 0];           % m/s      initial velocity
q0  = [1; 0; 0; 0];        %          initial attitude quaternion [q0 q1 q2 q3] (level, facing +x)
om0 = [0; 0; 0];           % rad/s    initial body rates [p; q; r]

%TEST SCENARIO
sim_stop_time = 12;        % s
sim_dt        = 0.001;     % s   

% reference steps: [time, final value]; every reference starts at 0
x_step_time   = 3.0;   x_step_value = 10;      % m
y_step_time   = 5.0;   y_step_value = 10;      % m
z_step_time   = 0.5;   z_step_value = 10;      % m
psi_step_time = 7.0;   psi_step_deg = 30;       % s, degrees
psi_step_value = deg2rad(psi_step_deg);         % rad (derived - re-run this file after editing psi_step_deg)

% SUMMARY
w_hover = sqrt(m*g/(4*kf));
fprintf('quad_params: m = %.4f kg, l = %.4f m, hover speed = %.0f rad/s (limit %.0f rad/s)\n', ...
        m, l, w_hover, w_max);
if w_hover >= w_max
    warning('quad_params: hover speed is above the rotor speed limit - the quad cannot hover.');
end
