function R = fdi_ftc_test_suite(varargin)
%FDI_FTC_TEST_SUITE  One script that tests your quadcopter fault detection (FDI) and fault-tolerant control (FTC).
%
% WHAT IT DOES (all of this happens when you press Run)
%   1. Makes a TEST COPY of your Simulink model. Your own model file is never changed.
%   2. Checks that the test copy gives exactly the same results as your model.
%   3. Builds a list of tests from the settings below and runs them. Finished runs are saved and reused.
%   4. Measures: detection time, correct motor found, false alarms, FTC start time, recovery, angle / height / drift,
%      motor limits, control effort, tracking error. Every run gets one outcome label.
%   5. Saves 8 tables (CSV, Excel, MAT) and the graphs (PNG and PDF).
%   Your FDI (two observers, thresholds, flags) and your LQR recovery controller are used as they are.
%
% FILES THAT MUST BE IN THE SAME FOLDER AS THIS SCRIPT (your own files)
%   all_parameters.m (replaces quad_params.m, fd_params.m, wind_params.m, noise_params.m; loads ftc_params_x.mat)
%   quadwind_noisy.slx  ('original'),  quadwind_noisy_v3.slx  ('v3'),  quadwind_noisy_v4.slx + ftc_params_x.mat  ('v4')
%   You need MATLAB R2026a, Simulink and the Control System Toolbox. No other toolbox.
%
% HOW TO RUN
%   Option 1: change the numbers in the USER SETTINGS block below, then press Run.
%   Option 2: give a setting by name, without editing the file (capital letters do not matter):
%        fdi_ftc_test_suite('Level','demo')
%        fdi_ftc_test_suite('Level','custom','Motors',[1 3],'WindSpeeds',[0.3 0.6],'NoiseLevels',[1 10],'NumCycles',5)
%        fdi_ftc_test_suite('System','original')
%        fdi_ftc_test_suite('Select','C1')          % only one group of tests
%        fdi_ftc_test_suite('WorkspaceOverrides',struct('thresh_flag2',0.6))
%
% WHERE THE RESULTS GO (folder FDI_FTC_suite, next to this script)
%   FDI_FTC_suite/results/suite/<system>/<name>/tables/    the 8 tables as CSV, plus suite_tables.xlsx
%   FDI_FTC_suite/results/suite/<system>/<name>/figures/   one graph per run, comparison graphs, summary graphs (PNG + PDF)
%   FDI_FTC_suite/results/suite/<system>/<name>/suite_results.mat   all results in MATLAB format
%   FDI_FTC_suite/results/suite/<system>/cache/            finished runs (reused when you run the same test again)
%   (<name> is the level, unless you set OutputName)
%
% HOW TO READ THE RESULTS
%   * perrun.csv has one row per run. The column "outcome" is ONE label:
%       healthy runs : healthy_ok, false_alarm, unstable_healthy
%       failure runs : false_alarm_before_fault, missed_detection, late_detection, misisolation (wrong motor),
%                      ftc_activation_failure, recovered, safe_descent, unsuccessful_recovery
%       any run      : diverged, simulation_failed
%     The column "status" says if the simulation itself finished, failed, or blew up. A run is never called a success
%     just because it finished.
%   * "Detection" means the first confirmed fault: Fault_True is on AND the thrust-loss estimate of the found motor is above
%     confirm_alpha. Your FTC uses the same rule. The raw Fault_True flag is also reported (noise can keep it on all the time).
%   * "Recovered" means: height, position, climb speed and tilt stay inside the limits below, for at least RecSettleTime,
%     until the end of the run.
%   The exact definition of every metric is in the comments of SUITE_METRICS. Assumptions and limits are in SUITE_README.md.

%% =====================================================================================================
%%                                       USER SETTINGS  (edit here)
%% =====================================================================================================
% ---- what to run ----------------------------------------------------------------------------------------
U.Level   = 'standard';  % 'demo'     = 4 tests, about 1 minute
                         % 'standard' = a planned set of about 136 tests (groups A, B1, B2, B3, B4, C1, C2, D)
                         % 'custom'   = EVERY combination of the lists below (wind type x speed x direction x noise x motor)
U.System  = 'v4';        % which system to test: 'original' (your quadwind_noisy.slx) | 'v3' | 'v4'
U.NumCycles = 3;         % HOW MANY CYCLES: how many times each random test is repeated with a new random seed (seeds 1,2,3...).
                         % A test is "random" if it has sensor noise, process noise, turbulence or model mismatch.
                         % Tests with nothing random run once, because repeating them gives the same answer.
U.Seeds   = [];          % or give the seeds yourself, e.g. [11 12 13 14]. If not empty, this replaces NumCycles.
U.Tiers   = {'A','B1','B2','B3','B4','C1','C2','D','CMP'};   % which groups of the 'standard' level to run:
                         % A = baseline | B1 = wind only | B2 = noise only | B3 = process noise or model mismatch only
                         % B4 = partial motor loss | C1 = wind + failure | C2 = noise + failure | D = everything together
                         % CMP = extra runs for the comparison graphs
U.Select  = [];          % [] = run everything. Or one group ('C1'), a group name, a test name, or test numbers like 1:20
U.MaxRuns = 500;         % safety stop: do not start if the list has more tests than this

% ---- simulation -----------------------------------------------------------------------------------------
U.SimTime   = 26;        % s  how long each simulation runs
U.FaultTime = 14;        % s  time when the motor fails (the solver step is sim_dt in quad_params.m)

% ---- motor failure --------------------------------------------------------------------------------------
U.Motors    = 1:4;       % which motors to fail, one at a time
U.FailureSeverity = [1 0.6];  % how much thrust is lost. 1 = the motor fails COMPLETELY. 0.6 = it loses 60 % (PARTIAL loss).
                         % The biggest number is used as the "complete failure". The smaller ones are the partial-loss tests.
U.IncludeHealthy = true; % 'custom' level: also run every combination without a failure (to check for false alarms)

% ---- wind (speed in m/s; wind force = Cd_wind x speed, Cd_wind = [0.5 0.5 0.25] N s/m in wind_params.m) -----------------
U.WindSpeeds = [0.15 0.3 0.5];   % m/s  weak, moderate, strong (lowest, middle, highest). Zero wind is always included.
                         % To understand the size: 0.5 m/s gives 0.25 N. The drone weighs 1.18 N and can push sideways with about 0.24 N.
U.WindDirections = [0 90 45];    % deg  direction in the world frame: 0 = forward (+x), 90 = left (+y), 45 = in between
U.WindTypes = {'steady','gust','turbulence','dirchange'};
                         % steady = constant wind | gust = a short push (smooth bump) | turbulence = random changing wind
                         % dirchange = wind that turns by 90 degrees every 4 s
U.WindVerticalFraction = 0;      % share of the wind that blows up or down (0 = flat wind, 0.3 = 30 % updraft)
U.WindOnset = 6;         % s  when steady, turbulent or direction-changing wind starts
U.GustOnsets    = [6 12 16];     % s  when a gust starts (before, just before, and after the failure)
U.GustDurations = [1 3];         % s  how long a gust lasts

% ---- noise ----------------------------------------------------------------------------------------------
% The noise is white (random, average zero). It is added to the sensor signals that the detector gets (and the controllers,
% if NoiseToControllers is true). A noise LEVEL is a multiplier of the sizes in noise_params.m:
%   attitude 0.005 rad (0.29 deg), body rates 0.005 rad/s, position 0.02 m, speed 0.02 m/s.
% So level 5 means 5 times those sizes. It is a fair scale-up, but not a claim about one real sensor.
% Measure your own sensors and change noise_params.m if you want real numbers.
U.NoiseLevels = [1 5 10];        % low, moderate, high multipliers. Level 0 (no noise) is always included.
U.IncludeStressNoise = false;    % true = also run the extra stress level below
U.StressNoise = 20;              % stress level multiplier (more than the range you need)
U.ProcessNoise = true;           % true = also add random pushes on the drone body (0.1 m/s^2 and 0.5 rad/s^2, 1-sigma)
                         % whenever sensor noise is on. This is separate from sensor noise.
U.NoiseToControllers = true;     % true = the noisy signals also go to the controllers and the FTC. false = only to the detector.
U.Mismatch = 1;          % 0 to 1: how wrong the model is compared with the real drone (mass +-10 %, inertia +-15 %,
                         % thrust +-10 %, torque +-15 %, motor speed lag +-20 %, arm +-3 %). Used in the mismatch and combined tests.

% ---- when is a test a success ---------------------------------------------------------------------------
U.DetWindow  = 1.0;      % s  the fault must be found within this time after the failure
U.FtcWindow  = 1.0;      % s  the FTC must start within this time after the fault is found
U.RecAltTol    = 1.0;    % m   recovery: height error must stay below this
U.RecHorizTol  = 3.0;    % m   recovery: distance to the target position must stay below this
U.RecVzTol     = 1.0;    % m/s recovery: climb or sink speed must stay below this
U.RecTiltTolDeg = 15;    % deg recovery: roll and pitch must stay this close to the planned tilt of the recovery
U.RecSettleTime = 2.0;   % s   all of the above must hold for this long, until the end of the run

% ---- change the system you test (optional) --------------------------------------------------------------
U.ParamOverrides = struct();     % the system's own switches, for example struct('ftc_latch_t',0.02,'Ki_pos',0.2,'obs_guard',0)
U.WorkspaceOverrides = struct(); % ANY variable of your parameter files, for example struct('thresh_flag2',0.6,'L1_om',[10;10;8])

% ---- output ---------------------------------------------------------------------------------------------
U.Plots = true;          % false = make tables only (faster)
U.DetailPlots = 'tiers'; % one graph per run: 'all' | 'tiers' (one example per group) | 'none' | a list of test names
U.Fresh = false;         % true = throw away saved runs and run everything again
U.Rebuild = false;       % true = build the Simulink test copy again from your model
U.Verify = true;         % true = check that the test copy matches your model
U.OutputName = '';       % name of the results folder ('' = the level name)

%% =====================================================================================================
%%                                       END OF USER SETTINGS
%% =====================================================================================================
% name-value pairs override the settings above
if mod(numel(varargin), 2) ~= 0, error('fdi_ftc_test_suite:args', 'Arguments must be name-value pairs.'); end
names = fieldnames(U);
for i = 1:2:numel(varargin)
    k = find(strcmpi(names, varargin{i}), 1);
    if isempty(k), error('fdi_ftc_test_suite:name', 'Unknown setting "%s". Valid names: %s', varargin{i}, strjoin(names', ', ')); end
    U.(names{k}) = varargin{i+1};
end
cfg = suite_config(U.System);                    % defaults for everything, then the settings above
S = cfg.suite;
cfg.stopTime = U.SimTime;  cfg.tFault = U.FaultTime;  S.faultTime = U.FaultTime;
if ~isempty(U.Seeds), S.seeds = U.Seeds(:)'; else, S.seeds = 1:max(1, round(U.NumCycles)); end
S.tiers = U.Tiers;  S.maxRuns = U.MaxRuns;
S.motors = U.Motors(:)';
sev = sort(U.FailureSeverity(:)', 'descend');  S.severityMain = sev(1);  S.severityPartial = sev(sev < sev(1));
S.windLevels = levels3_(U.WindSpeeds, {'weak','moderate','strong'}, 'zero');
S.windSpeedList = sort(U.WindSpeeds(:)');
S.windDirList = U.WindDirections(:)';
d3 = U.WindDirections(:)';  while numel(d3) < 3, d3(end+1) = d3(end); end %#ok<AGROW>
S.windDirs = struct('longitudinal', d3(1), 'lateral', d3(2), 'combined', d3(3));
S.windTypes = U.WindTypes;  S.windVertical = U.WindVerticalFraction;  S.windOnset = U.WindOnset;
S.gustOnsets = U.GustOnsets(:)';  S.gustDurations = U.GustDurations(:)';
S.noiseLevels = levels3_(U.NoiseLevels, {'low','moderate','high'}, 'none');
S.noiseList = sort(U.NoiseLevels(:)');
S.noiseStress = U.StressNoise;  S.includeStress = U.IncludeStressNoise;
S.processNoise = double(U.ProcessNoise);  S.noiseCtrl = double(U.NoiseToControllers);  S.mismatch = U.Mismatch;
S.includeHealthy = U.IncludeHealthy;
S.detWindow = U.DetWindow;  S.ftcWindow = U.FtcWindow;
S.rec.altTol = U.RecAltTol;  S.rec.horizTol = U.RecHorizTol;  S.rec.vzTol = U.RecVzTol;
S.rec.tiltTol = deg2rad(U.RecTiltTolDeg);  S.rec.settleTime = U.RecSettleTime;
S.detailPlots = U.DetailPlots;
S.paramOverrides = U.ParamOverrides;
cfg.suite = S;
cfg.crit.recAltErr = S.rec.altTol;  cfg.crit.recHorizErr = S.rec.horizTol;
cfg.wsOverride = U.WorkspaceOverrides;
R = run_test_suite(cfg, 'Level', U.Level, 'Select', U.Select, 'Fresh', U.Fresh, 'Plots', U.Plots, ...
                   'Rebuild', U.Rebuild, 'Verify', U.Verify, 'Name', U.OutputName);
end

function L = levels3_(vals, names3, zeroName)
% three named levels (lowest, middle, highest of the positive values) plus the zero level
v = sort(vals(vals > 0));
if isempty(v), v = 1; end
mid = v(max(1, ceil(numel(v)/2)));
L = struct(zeroName, 0, names3{1}, v(1), names3{2}, mid, names3{3}, v(end));
end

%% =====================================================================================================
%% DEFAULT CONFIGURATION (used when a setting above is not given)
%% =====================================================================================================

%% ----------------------------------------------------------------------------------------------------
%% SUITE_CONFIG
%% ----------------------------------------------------------------------------------------------------
function cfg = suite_config(system)
%SUITE_CONFIG  Every setting of the FDI / FTC robustness test suite in ONE place (all units stated).
%
%   cfg = suite_config('v4')        % system under test: 'original' | 'v3' | 'v4'   (default 'v4')
%
% SYSTEM UNDER TEST
%   'original'  quadwind_noisy.slx exactly as you wrote it (plus layout, observers unguarded, FTC latches on the
%               first sample, PD controllers, recovery spin always +z).
%   'v3'        plus layout + observer guard + 10 ms FTC latch hold + PID.
%   'v4'        v3 + body axes rotated 45 deg from the arms + recovery spin in the natural direction (default).
%   The FDI (two observers, residual bounds, Flag1/Flag2, Judgment) and the LQR recovery controller are YOUR
%   algorithms in every case; the variants only differ by the documented changes (see SUITE_README.md, section 7).
%
% HOW TO CHANGE A SETTING: edit this file, or change a field after calling it:
%       cfg = suite_config('v4');  cfg.suite.tFault = 12;  cfg.suite.seeds = 1:5;
%       run_test_suite(cfg, 'Level', 'standard');
%
% All times are in seconds, lengths in metres, angles in radians unless the field name says "deg".

if nargin < 1 || isempty(system), system = 'v4'; end
switch system
    case 'v4',       setappdata(0, 'quad_geometry', 'x');
    case {'v3', 'original'}, setappdata(0, 'quad_geometry', 'plus');
    otherwise, error('suite_config:system', 'system must be original | v3 | v4');
end
cfg = campaign_config('smoke');                  % base settings shared with the older campaigns
cfg.system     = system;
cfg.dirSuite   = fullfile(cfg.dirCampaign, 'results', 'suite');
cfg.dirResults = fullfile(cfg.dirSuite, system);  % cache/, tables/, figures/ are created below this folder

%% ------------------------------------------------------------ SIMULATION
cfg.stopTime = 26;     % s    simulation length
cfg.tFault   = 14;     % s    default motor-failure time (a scenario may override it)
cfg.tGrace   = 1.0;    % s    detector start-up period (observers start from zero); alarms before this are ignored
cfg.winPre   = 3;      % s    "pre-fault" evaluation window  [tFault-winPre, tFault)
cfg.winFinal = 3;      % s    "final" evaluation window      [stopTime-winFinal, stopTime]
cfg.ftcTol   = 1e-6;   % rad/s  FTC is "active" when max_i |w_cmd_i - w_nom_i| exceeds this

%% ------------------------------------------------------------ TEST SUITE
S = struct();
S.system = system;
S.motors = 1:4;  S.severityMain = 1;  S.severityPartial = 0.6;  S.maxRuns = 500;
S.tiers = {'A','B1','B2','B3','B4','C1','C2','D','CMP'};
S.windSpeedList = [0.15 0.3 0.5];  S.windDirList = [0 90 45];  S.windTypes = {'steady','gust','turbulence','dirchange'};
S.windOnset = 6;  S.noiseList = [1 5 10];  S.includeHealthy = true;  S.paramOverrides = struct();
S.seeds        = [1 2 3];     % random seeds of noise, turbulence and plant mismatch (any list of integers)
S.faultTime    = cfg.tFault;  % s     failure time used by the matrix
S.partialSeverity = 0.6;      % -     partial loss of motor effectiveness: fraction of thrust lost (see note below)
% NOTE on partial loss: the model multiplies the rotor SPEED by h = sqrt(1 - severity), so thrust and torque of that
% motor fall to (1 - severity) of normal: severity 1 = complete failure, 0.6 = 60 % thrust loss.

% Wind: peak horizontal speed in m/s.  The force is Cd_wind .* v (Cd_wind = [0.5 0.5 0.25] N s/m from wind_params.m),
% so 0.5 m/s is already 0.25 N against a 1.18 N weight; the position loop can push sideways with about 0.24 N.
S.windLevels   = struct('zero', 0, 'weak', 0.15, 'moderate', 0.3, 'strong', 0.5);   % m/s
S.windDirs     = struct('longitudinal', 0, 'lateral', 90, 'combined', 45);          % deg, 0 = +x (forward), 90 = +y (left)
S.windVertical = 0.0;         % -     fraction of the wind speed blowing vertically (0 = horizontal only)
S.gustOnsets   = [6 12 16];   % s     gust start times (6 = well before the failure, 12 = just before, 16 = after)
S.gustDurations= [1 3];       % s     gust durations (one 1-cos pulse each)

% Noise: white Gaussian, zero mean, one sample every Ts (zero-order hold), added to the signals that the detector
% (and, if noiseCtrl = 1, the controllers) receive.  Standard deviations at level 1 are the ones in noise_params.m:
%   attitude 0.005 rad (0.29 deg), body rates 0.005 rad/s, position 0.02 m, velocity 0.02 m/s
%   (attitude/rates sampled every 4 ms, position/velocity every 20 ms).
% "Level" is a MULTIPLIER of all four standard deviations (cfg noise_scale in the model), so level 5 means
% 1.5 deg attitude noise, 0.025 rad/s ... and 10 cm position noise.  It is a controlled scaling, not a claim that
% a particular sensor reaches that noise: measure your own sensors and edit noise_params.m if you want absolute values.
S.noiseLevels  = struct('none', 0, 'low', 1, 'moderate', 5, 'high', 10);            % multiplier of noise_params.m sigmas
S.noiseStress  = 20;          % -     optional "stress" level (beyond the noise range of interest), see includeStress
S.includeStress = false;      % true -> also run noiseStress in the noise-only and combined tiers
S.noiseCtrl    = 1;           % 1 = the noisy signals also feed the position/attitude controllers and the FTC; 0 = detector only
S.processNoise = 1;           % 1 = process noise (random acceleration 0.1 m/s^2, angular acceleration 0.5 rad/s^2, 1-sigma) is
                              %     switched on whenever the sensor-noise level is above 0; separate from measurement noise
S.mismatch     = 1;           % 0..1: scales the plant-vs-model mismatch (campaign_config unc.pct: mass +-10 %, inertia +-15 %,
                              %     thrust coefficient +-10 %, torque coefficient +-15 %, motor time constant +-20 %, arm +-3 %)
S.mismatchSeedsOnly = true;   % true: mismatch is applied only in the "combined" tier and in the mismatch-only runs

% Detection / activation windows used to judge success
S.detWindow    = 1.0;         % s     a fault is "detected in time" if the confirmed declaration comes within this time
S.ftcWindow    = 1.0;         % s     the FTC must activate within this time after the confirmed declaration

% Recovery (stable flight) criteria.  All must hold CONTINUOUSLY for settleTime up to the end of the run.
S.rec.altTol    = 1.0;        % m     |z - z_ref|
S.rec.horizTol  = 3.0;        % m     horizontal distance to the reference position
S.rec.vzTol     = 1.0;        % m/s   |vertical speed| (central difference of z)
S.rec.tiltTol   = deg2rad(15);% rad   max(|roll - roll_eq|, |pitch - pitch_eq|) where (roll_eq, pitch_eq) is the designed
                              %       spinning-hover tilt of the failed rotor (seq_all(7:8,k)); 0 for healthy flight
S.rec.settleTime = 2.0;       % s     required duration of continuous compliance

% Controlled-descent criterion (only used when hover recovery fails; the FTC is DESIGNED to hover, so this is a
% separate, secondary classification and never turns a hover failure into a hover success)
S.descent.enable   = true;
S.descent.zGround  = 0.2;     % m     touchdown = first time z falls below this after the failure
S.descent.maxVz    = 2.0;     % m/s   |vertical speed| at touchdown
S.descent.maxTilt  = deg2rad(60); % rad max(|roll|,|pitch|) at touchdown
S.descent.maxDrift = 10;      % m     horizontal distance to the reference at touchdown

% Plots
S.detailPlots  = 'tiers';     % 'all' | 'tiers' (first run of each tier and group) | 'none' | cell array of scenario ids
S.pngDpi       = 200;
S.pdf          = true;
cfg.suite = S;

%% base criteria reused by compute_metrics (kept consistent with the suite criteria above)
cfg.crit.recAltErr   = S.rec.altTol;
cfg.crit.recHorizErr = S.rec.horizTol;
end

%% ----------------------------------------------------------------------------------------------------
%% RUN_TEST_SUITE: the runner
%% ----------------------------------------------------------------------------------------------------
function R = run_test_suite(cfg, varargin)
%RUN_TEST_SUITE  Run the FDI / FTC robustness test suite and produce all tables and figures.
%
%   run_test_suite                                   % demo level, system v4 (4 runs, about 1 minute)
%   run_test_suite(suite_config('v4'), 'Level', 'standard')           % full matrix (~136 runs)
%   run_test_suite(cfg, 'Level', 'standard', 'Select', 'C1')         % only tier C1
%   run_test_suite(cfg, 'Level', 'standard', 'Select', {'id1','id2'})% explicit scenario ids (see tables/scenarios.csv)
%   run_test_suite(cfg, 'Level', 'standard', 'Select', 1:20)         % scenario numbers in the matrix
%   run_test_suite(cfg, ..., 'Fresh', true)          % ignore cached results of this system and rerun everything
%   run_test_suite(cfg, ..., 'Plots', false)         % tables only
%   run_test_suite(cfg, ..., 'Rebuild', true)        % rebuild the Simulink test copy from your model files
%
% 'Select' also accepts a group name (e.g. 'cmp_noise', 'worst_case') or a tier ('A','B1','B2','B3','B4','C1','C2','D').
% Output folder:  campaign/results/suite/<system>/<level>/   tables/ (CSV, XLSX, MAT), figures/ (PNG, PDF), suite_results.mat
% Completed simulations are cached in campaign/results/suite/<system>/cache/ and reused by identical re-runs.

if nargin < 1 || isempty(cfg), cfg = suite_config('v4'); end
ip = inputParser;
addParameter(ip, 'Level', 'demo');  addParameter(ip, 'Select', []);  addParameter(ip, 'Fresh', false);
addParameter(ip, 'Plots', true);    addParameter(ip, 'Rebuild', false);   addParameter(ip, 'Verify', true);   addParameter(ip, 'Name', '');
parse(ip, varargin{:});  o = ip.Results;
setappdata(0, 'quad_geometry', ternary(strcmp(cfg.system, 'v4'), 'x', 'plus'));
addpath(cfg.dirCampaign);  addpath(cfg.dirProject);
outName = o.Name;  if isempty(outName), outName = o.Level; end      % results folder name (default: the level)
cfg.dirOut = fullfile(cfg.dirSuite, cfg.system, outName);
cfg.level = ['suite_' o.Level];
if o.Fresh && exist(fullfile(cfg.dirSuite, cfg.system, 'cache'), 'dir'), rmdir(fullfile(cfg.dirSuite, cfg.system, 'cache'), 's'); end
for d = {cfg.dirOut, fullfile(cfg.dirOut, 'tables'), fullfile(cfg.dirOut, 'figures')}, if ~exist(d{1}, 'dir'), mkdir(d{1}); end, end
diary(fullfile(cfg.dirOut, 'suite_log.txt'));  dcl = onCleanup(@() diary('off'));
fprintf('=== FDI/FTC test suite: system %s, level %s, %s ===\n', cfg.system, o.Level, datestr(now));

%% model (campaign copy of the real model: extra loggers + workspace-controlled fault / wind / noise, see SUITE_README.md)
mf = fullfile(cfg.modelDir, [cfg.model '.slx']);
need = o.Rebuild || ~exist(mf, 'file');
if ~need
    load_system(mf);
    need = isempty(find_system(cfg.model, 'SearchDepth', 3, 'Name', 'fd_rupper'));   % copy made before the threshold loggers existed
    close_system(cfg.model, 0);
end
if need, build_campaign_model(cfg); end
if bdIsLoaded(cfg.model), close_system(cfg.model, 0); end
load_system(mf);
V0 = nominal_vars(cfg);
p  = suite_system(cfg, V0);
verify = [];
if o.Verify, verify = verify_copy(cfg, V0, p); end

%% scenarios
[S, T1] = suite_matrix(cfg, o.Level);
sel = select_(S, T1, o.Select);
S = S(sel);  T1 = T1(sel, :);
nS = numel(S);
fprintf('%d scenarios selected\n', nS);
writetable(T1, fullfile(cfg.dirOut, 'tables', 'scenarios.csv'));

%% run
rows = cell(nS, 1);  status = strings(nS, 1);  t0 = tic;
for k = 1:nS
    [rows{k}, ~] = run_one(cfg, V0, p, S(k), false);
    M = rows{k};
    if ~isfield(M, 'outcome'), M.outcome = "simulation_failed"; rows{k} = M; end   % solver error: kept and reported, never dropped
    if ~M.sim_ok, status(k) = "failed (solver/simulation error)";
    elseif M.diverged > 0, status(k) = "diverged (non-finite or unbounded)";
    elseif ~isfield(M, 'outcome') || ~isfinite(M.t_fault_s) && S(k).rotor > 0, status(k) = "invalid (metrics undefined)";
    else, status(k) = "completed"; end
    fprintf('  [%3d/%3d] %-72s %-30s %s\n', k, nS, S(k).id, char(M.outcome), char(status(k)));
end
fprintf('Simulations finished in %.1f min\n', toc(t0)/60);

%% per-run table (Table 2) = configuration columns + metrics
T2 = struct2table(unify_(rows));
T2.status = status;
dup = intersect(T1.Properties.VariableNames, T2.Properties.VariableNames);
T2(:, dup) = [];                                        % configuration columns come from the matrix table
T = [T1, T2];
T.failedMotor = T1.failedMotor;  T.seed = T1.seed;
T.faultInjection_s = T.t_fault_s;
T.fdiDetection_s = T.t_decl_s;  T.ftcActivation_s = T.t_ftc;  T.recoveryStart_s = T.t_rec_s;
T = movevars(T, {'faultInjection_s','fdiDetection_s','ftcActivation_s','recoveryStart_s','recovery_time_s','outcome','status'}, 'After', 'seed');
Tab = suite_tables(cfg, T);  Tab.scenarios = T1;

%% save tables
td = fullfile(cfg.dirOut, 'tables');
fn = fieldnames(Tab);
for i = 1:numel(fn)
    X = Tab.(fn{i});  if isempty(X), continue; end
    writetable(X, fullfile(td, [fn{i} '.csv']));
    try, writetable(X, fullfile(td, 'suite_tables.xlsx'), 'Sheet', fn{i}); catch, end
end
R = struct('cfg', cfg, 'S', S, 'T', T, 'Tab', Tab, 'V0', V0, 'p', p, 'status', status, 'verify', verify);
save(fullfile(cfg.dirOut, 'suite_results.mat'), 'R', '-v7.3');

%% summary of run status
fprintf('\n=== SUMMARY: %d runs  |  completed %d  |  failed %d  |  diverged %d  |  invalid %d ===\n', nS, ...
    nnz(status == "completed"), nnz(startsWith(status, "failed")), nnz(startsWith(status, "diverged")), nnz(startsWith(status, "invalid")));
oc = groupsummary(T, 'outcome');  disp(oc);

%% figures
if o.Plots
    suite_plots(cfg, R);
end
fprintf('Results in %s\n', cfg.dirOut);
end

% -----------------------------------------------------------------------------------------------
function sel = select_(S, T1, s)
if isempty(s), sel = 1:numel(S); return; end
if isnumeric(s), sel = s(:)'; return; end
if ischar(s) || isstring(s), s = cellstr(s); end
sel = false(1, numel(S));
for q = 1:numel(s)
    sel = sel | ismember({S.id}, s(q)) | (T1.tier == string(s{q}))' | contains(T1.group, string(s{q}))';
end
sel = find(sel);
if isempty(sel), error('run_test_suite:select', 'No scenario matches the selection.'); end
end

function rows = unify_(rows)
fn = {};
for i = 1:numel(rows), fn = union(fn, fieldnames(rows{i}), 'stable'); end
for i = 1:numel(rows)
    for j = 1:numel(fn), if ~isfield(rows{i}, fn{j}), rows{i}.(fn{j}) = NaN; end, end
    rows{i} = orderfields(rows{i}, fn);
end
rows = [rows{:}]';
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end

function v = verify_copy(cfg, V0, p)
%VERIFY_COPY  Prove that the Simulink test copy reproduces YOUR model before any result is trusted.
% Runs your model file (default fault: motor 4 at 15 s, default wind window, sensor noise x1) and the test copy with the
% same settings, and compares position, attitude, rotor speeds, FDI flags, thrust-loss estimates and wind force.
refMap = struct('original', 'quadwind_noisy', 'v3', 'quadwind_noisy_v3', 'v4', 'quadwind_noisy_v4');
ref = refMap.(cfg.system);
v = struct('reference', ref, 'max_difference', NaN, 'passed', NaN);
if ~exist(fullfile(cfg.dirProject, [ref '.slx']), 'file')
    warning('verify_copy:missing', 'Reference model %s.slx not found next to the script: model-copy check skipped.', ref);  return
end
fprintf('Verifying that the test copy reproduces %s.slx ...\n', ref);
if bdIsLoaded(ref), close_system(ref, 0); end
load_system(fullfile(cfg.dirProject, [ref '.slx']));  set_param(ref, 'StopTime', '20');
for nm_ = {'noise_scale','proc_scale','noise_ctrl_on','noise_seed'}, assignin('base', nm_{1}, 1); end   % after loading: PreLoadFcn sets the file defaults
assignin('base', 'unc_thrust_gain', 1);  assignin('base', 'unc_M_gain', [1;1;1]);  assignin('base', 'unc_omdot_gain', [1;1;1]);
assignin('base', 'm_true', V0.m);  assignin('base', 'tau_true', V0.tau);
o1 = sim(ref);  close_system(ref, 0);
sc = make_scenarios(cfg, 'one', 'fault_only', 4, 1, 'calm', 0, 0, 1, 0, 1, 1, 1);
V = params_util('apply', cfg, V0, p, sc);
V.hs_t = [1e9 1e9 1e9 15];  V.hs_v = [1 1 1 0];
for f = {'wind_on','wind_A','wind_Om','wind_Phi','wind_v0','wind_t_start','wind_t_end','Cd_wind'}, V.(f{1}) = V0.(f{1}); end
V.wind_ev = zeros(12, 6);  V.noise_scale = 1;  V.proc_scale = 1;  V.noise_ctrl_on = 1;
fn = fieldnames(V);  for i = 1:numel(fn), assignin('base', fn{i}, V.(fn{i})); end
if bdIsLoaded(cfg.model), close_system(cfg.model, 0); end
load_system(fullfile(cfg.modelDir, [cfg.model '.slx']));  set_param(cfg.model, 'StopTime', '20');
o2 = sim(cfg.model);  close_system(cfg.model, 0);
chk = {'p_log', 'eul_log', 'w_log', 'fd_flags', 'fd_alpha', 'wind_log'};  worst = 0;
for i = 1:numel(chk)
    a = o1.get(chk{i}).Data;  b = o2.get(chk{i}).Data;  d = max(abs(a(:) - b(:)));
    fprintf('    %-9s max |difference| = %.3g\n', chk{i}, d);  worst = max(worst, d);
end
v.max_difference = worst;  v.passed = double(worst < 1e-9);
if v.passed, fprintf('  model copy verified (identical to %s.slx).\n', ref);
else, warning('verify_copy:mismatch', 'Test copy differs from %s.slx (max difference %.3g). Results are NOT guaranteed to describe your model.', ref, worst); end
load_system(fullfile(cfg.modelDir, [cfg.model '.slx']));
end

%% ----------------------------------------------------------------------------------------------------
%% SUITE_SYSTEM
%% ----------------------------------------------------------------------------------------------------
function p = suite_system(cfg, V0)
%SUITE_SYSTEM  Parameter overrides that select the system under test (see suite_config.m).
%   original : your quadwind_noisy.slx behaviour (campaign nominal values: observers unguarded, latch on first sample, PD)
%   v3       : + observer guard, 10 ms FTC latch hold, PID (all workspace switches; no other difference)
%   v4       : same switches as v3 and the 45-degree body frame with the natural-spin recovery LQR
p = params_util('nominal', cfg);
switch cfg.system
    case 'original'
        p.obs_guard = 0;  p.ftc_latch_t = 0;  p.Ki_pos = 0;  p.Ki_r = 0;  p.Ki_p = 0;
    case {'v3', 'v4'}
        p.obs_guard = 1;  p.ftc_latch_t = 0.01;
        p.Ki_pos = 0.3;  p.xi_max = 3.3;  p.pos_zone = [1; 1; 1];
        p.Ki_r = 20*V0.Jxx;  p.Ki_p = 20*V0.Jyy;  p.xa_max = 0.3;  p.att_zone = 0.2;
if isfield(cfg.suite, 'paramOverrides')               % user changes of the system's own switches
    for f_ = fieldnames(cfg.suite.paramOverrides)', p.(f_{1}) = cfg.suite.paramOverrides.(f_{1}); end
end
end
end

%% ----------------------------------------------------------------------------------------------------
%% SUITE_MATRIX: the test matrix
%% ----------------------------------------------------------------------------------------------------
function [S, T] = suite_matrix(cfg, level)
%SUITE_MATRIX  Build the list of tests (scenario structs S and the configuration table T).
%
%   level 'demo'      4 tests: healthy baseline, motor failure, healthy + wind + noise, everything together
%   level 'standard'  a planned set in groups (tiers). Which tiers run is set by Tiers in the user settings.
%   level 'custom'    EVERY combination of the lists in the user settings
%
% Tiers of the standard level (a test that belongs to several groups appears once)
%   A   baseline: healthy, no wind, no noise; each motor fails, no wind, no noise
%   B1  wind only (healthy): steady x strength x direction, gusts x start time x length, gusts x strength,
%       turbulence x strength x seeds, direction-changing wind x strength
%   B2  sensor noise only (healthy): low / moderate / high (x seeds)
%   B3  process noise only and model mismatch only (healthy)
%   B4  partial motor loss, each motor, no other disturbance
%   C1  wind + failure: each motor x six wind cases
%   C2  noise + failure: each motor x noise levels x seeds
%   D   everything together: moderate and strong wind + noise + mismatch + failure; healthy worst case
%   CMP extra runs that the comparison graphs need (wind strength, wind direction, noise level)
% All numbers (speeds, levels, motors, seeds, times) come from cfg.suite, which the user settings fill in.

if nargin < 2, level = 'standard'; end
s = cfg.suite;
wl = s.windLevels;  nl = s.noiseLevels;  seeds = s.seeds;  tF = s.faultTime;
motors = s.motors;  sevM = s.severityMain;
rows = {};                                          % each row = one test described by a struct (see addrow below)

switch level
    case 'demo'
        m1 = motors(1);
        rows = addrow(rows,'A','demo;baseline','calm','zero',0,0,6,2,'none',0,0,0,0,1,seeds(1));
        rows = addrow(rows,'A','demo;baseline','calm','zero',0,0,6,2,'none',0,0,0,m1,sevM,seeds(1));
        rows = addrow(rows,'B','demo','gust','moderate',wl.moderate,s.windDirs.combined,s.gustOnsets(min(2,end)),2,'moderate',nl.moderate,1,0,0,1,seeds(1));
        rows = addrow(rows,'D','demo;combined','gust','moderate',wl.moderate,s.windDirs.combined,s.gustOnsets(min(2,end)),2,'moderate',nl.moderate,1,1,m1,sevM,seeds(1));
    case 'standard'
        dn = fieldnames(s.windDirs);  wn = {'weak','moderate','strong'};  noiseN = {'low','moderate','high'};
        % ---- A: baseline
        rows = addrow(rows,'A','baseline','calm','zero',0,0,6,2,'none',0,0,0,0,1,seeds(1));
        for r = motors, rows = addrow(rows,'A','baseline;fault','calm','zero',0,0,6,2,'none',0,0,0,r,sevM,seeds(1)); end
        % ---- B1: wind only, healthy
        for a = 1:3, for d = 1:3
            rows = addrow(rows,'B1','wind;steady','steady',wn{a},wl.(wn{a}),s.windDirs.(dn{d}),6,inf,'none',0,0,0,0,1,seeds(1));
        end, end
        for o = s.gustOnsets, for du = s.gustDurations
            rows = addrow(rows,'B1','wind;gust_timing','gust','moderate',wl.moderate,s.windDirs.combined,o,du,'none',0,0,0,0,1,seeds(1));
        end, end
        for a = 1:3
            rows = addrow(rows,'B1','wind;gust_strength','gust',wn{a},wl.(wn{a}),s.windDirs.combined,s.gustOnsets(1),2,'none',0,0,0,0,1,seeds(1));
            rows = addrow(rows,'B1','wind;dirchange','dirchange',wn{a},wl.(wn{a}),s.windDirs.longitudinal,6,inf,'none',0,0,0,0,1,seeds(1));
            for sd = seeds, rows = addrow(rows,'B1','wind;turbulence','turbulence',wn{a},wl.(wn{a}),s.windDirs.combined,6,inf,'none',0,0,0,0,1,sd); end
        end
        % ---- B2: noise only, healthy
        lv = noiseN;  if s.includeStress, lv{end+1} = 'stress'; end
        for a = 1:numel(lv), for sd = seeds
            rows = addrow(rows,'B2','noise','calm','zero',0,0,6,2,lv{a},nz_(s,lv{a}),s.processNoise,0,0,1,sd);
        end, end
        % ---- B3: process noise only, mismatch only
        for sd = seeds
            rows = addrow(rows,'B3','process_noise','calm','zero',0,0,6,2,'process only',0,s.processNoise,0,0,1,sd);
            rows = addrow(rows,'B3','mismatch','calm','zero',0,0,6,2,'none',0,0,1,0,1,sd);
        end
        % ---- B4: partial loss
        for sv = s.severityPartial, for r = motors
            rows = addrow(rows,'B4','fault;partial','calm','zero',0,0,6,2,'none',0,0,0,r,sv,seeds(1));
        end, end
        % ---- C1: wind x failure
        for r = motors
            rows = addrow(rows,'C1','wind_x_fault','steady','moderate',wl.moderate,s.windDirs.longitudinal,6,inf,'none',0,0,0,r,sevM,seeds(1));
            rows = addrow(rows,'C1','wind_x_fault','steady','moderate',wl.moderate,s.windDirs.lateral,6,inf,'none',0,0,0,r,sevM,seeds(1));
            rows = addrow(rows,'C1','wind_x_fault','steady','strong',wl.strong,s.windDirs.combined,6,inf,'none',0,0,0,r,sevM,seeds(1));
            rows = addrow(rows,'C1','wind_x_fault','gust','moderate',wl.moderate,s.windDirs.combined,s.gustOnsets(min(2,end)),2,'none',0,0,0,r,sevM,seeds(1));
            rows = addrow(rows,'C1','wind_x_fault','gust','strong',wl.strong,s.windDirs.combined,s.gustOnsets(end),s.gustDurations(end),'none',0,0,0,r,sevM,seeds(1));
            rows = addrow(rows,'C1','wind_x_fault','turbulence','moderate',wl.moderate,s.windDirs.combined,6,inf,'none',0,0,0,r,sevM,seeds(1));
        end
        % ---- C2: noise x failure
        lv2 = noiseN;  if s.includeStress, lv2{end+1} = 'stress'; end
        for r = motors, for a = 1:numel(lv2), for sd = seeds(1:min(2,end))
            rows = addrow(rows,'C2','noise_x_fault','calm','zero',0,0,6,2,lv2{a},nz_(s,lv2{a}),s.processNoise,0,r,sevM,sd);
        end, end, end
        % ---- D: combined
        for r = motors, for sd = seeds(1:min(2,end))
            rows = addrow(rows,'D','combined;moderate','steady','moderate',wl.moderate,s.windDirs.combined,6,inf,'moderate',nl.moderate,s.processNoise,1,r,sevM,sd);
        end, end
        for r = motors, for sd = seeds
            rows = addrow(rows,'D','combined;worst_case','gust','strong',wl.strong,s.windDirs.combined,s.gustOnsets(end),s.gustDurations(end),'high',nl.high,s.processNoise,1,r,sevM,sd);
        end, end
        for sd = seeds
            rows = addrow(rows,'D','combined;healthy_worst_case','gust','strong',wl.strong,s.windDirs.combined,s.gustOnsets(min(2,end)),s.gustDurations(end),'high',nl.high,s.processNoise,1,0,1,sd);
        end
        % ---- CMP overlay groups (first motor)
        m1 = motors(1);
        for a = {'zero','weak','moderate','strong'}
            rows = addrow(rows,'CMP','cmp_wind_level','steady',a{1},wl.(a{1}),s.windDirs.combined,6,inf,'none',0,0,0,m1,sevM,seeds(1));
        end
        for dd = unique([s.windDirs.longitudinal s.windDirs.lateral s.windDirs.combined mod(s.windDirs.longitudinal + 180, 360)], 'stable')
            rows = addrow(rows,'CMP','cmp_wind_dir','steady','moderate',wl.moderate,dd,6,inf,'none',0,0,0,m1,sevM,seeds(1));
        end
        for a = {'none','low','moderate','high'}
            rows = addrow(rows,'CMP','cmp_noise','calm','zero',0,0,6,2,a{1},nl.(a{1}),double(nl.(a{1}) > 0)*s.processNoise,0,m1,sevM,seeds(1));
        end
        keep = cellfun(@(r) any(strcmp(r.tier, s.tiers)), rows);   % only the tiers chosen by the user
        rows = rows(keep);
    case 'custom'
        % every combination of the user lists
        noiseVals = [0, s.noiseList(s.noiseList > 0)];  if s.includeStress, noiseVals(end+1) = s.noiseStress; end
        motList = motors;  if s.includeHealthy, motList = [0 motors]; end
        sevList = [sevM, s.severityPartial];
        wcases = {struct('type','calm','name','zero','speed',0,'dir',0,'onset',6,'dur',2)};
        for ty = s.windTypes(:)'
            for sp = s.windSpeedList(s.windSpeedList > 0)
                nm = sprintf('%g', sp);
                switch ty{1}
                    case 'gust'
                        for d = s.windDirList, for o = s.gustOnsets, for du = s.gustDurations
                            wcases{end+1} = struct('type','gust','name',nm,'speed',sp,'dir',d,'onset',o,'dur',du); %#ok<AGROW>
                        end, end, end
                    case 'steady'
                        for d = s.windDirList, wcases{end+1} = struct('type','steady','name',nm,'speed',sp,'dir',d,'onset',s.windOnset,'dur',inf); end %#ok<AGROW>
                    otherwise                      % turbulence, dirchange: the direction does not matter, one direction is enough
                        wcases{end+1} = struct('type',ty{1},'name',nm,'speed',sp,'dir',s.windDirList(1),'onset',s.windOnset,'dur',inf); %#ok<AGROW>
                end
            end
        end
        for w = wcases, w = w{1};
            for nz = noiseVals, for mo = motList
                if mo == 0, svl = 1; else, svl = sevList; end
                for sv = svl, for sd = seeds
                    nzName = 'none';  if nz > 0, nzName = sprintf('x%g', nz); end
                    rows = addrow(rows,'CUSTOM','custom',w.type,w.name,w.speed,w.dir,w.onset,w.dur,nzName,nz,double(nz > 0)*s.processNoise,double(s.mismatch > 0 && nz > 0),mo,sv,sd);
                end, end
            end, end
        end
    otherwise
        error('suite_matrix:level', 'level must be demo | standard | custom');
end

%% ------------------------------------------------ rows -> scenario structs
S = [];
for k = 1:numel(rows)
    r = rows{k};
    prof = r.windType;
    if ~strcmp(prof, 'calm') && ~any(strcmp(prof, s.windTypes)), continue; end     % wind type switched off by the user
    W = r.windPeak;  if strcmp(prof, 'calm'), W = 0; end
    if ~strcmp(prof, 'gust'), r.onset = s.windOnset; end
    stoch = r.noiseLvl > 0 || r.proc > 0 || r.mism > 0 || strcmp(prof, 'turbulence');
    if ~stoch, r.seed = seeds(1); end                 % nothing random: one run is enough
    sc = make_scenarios(cfg, 'one', 'wind_only', r.rotor, r.sev, prof, W, r.dirDeg, r.seed, r.mism*s.mismatch, ...
                        r.noiseLvl, r.proc, s.noiseCtrl);
    sc.tFault = tF;
    sc.wind.onset = r.onset;  sc.wind.dur = r.dur;  sc.wind.vertFrac = s.windVertical;
    if strcmp(prof, 'gust'), sc.wind.nGust = 1; sc.wind.interval = 1e3; end
    sc.meta = struct('tier', r.tier, 'group', r.group, 'windType', prof, 'windName', r.windName, 'windPeak', W, ...
        'dirDeg', r.dirDeg, 'gustOnset', r.onset, 'gustDur', r.dur, 'noiseName', r.noiseName, 'noiseLevel', r.noiseLvl, ...
        'processNoise', r.proc, 'mismatch', r.mism*s.mismatch, 'failedMotor', r.rotor, 'severity', r.sev, ...
        'faultType', faultType_(r.rotor, r.sev), 'seed', r.seed);
    sc.id = sprintf('%s_%s_W%g_d%g_o%g_L%g_n%g_p%d_u%g_R%d_sev%g_tf%g_s%d', r.tier, prof, W, r.dirDeg, r.onset, ...
                    r.dur, r.noiseLvl, r.proc, r.mism, r.rotor, r.sev, tF, r.seed);
    sc.id = regexprep(sc.id, '[^\w]', '_');
    S = [S, sc]; %#ok<AGROW>
end
if isempty(S), error('suite_matrix:empty', 'The settings select no test at all.'); end
% a test requested by several groups appears once, with all group names joined
[~, ia, ic] = unique({S.id}, 'stable');
Sd = S(ia);
for k = 1:numel(ia)
    g = unique(cellfun(@(m) m.group, {S(ic == k).meta}, 'UniformOutput', false));
    Sd(k).meta.group = strjoin(g, ';');
    tiers = {S(ic == k).meta};  Sd(k).meta.tier = tiers{1}.tier;
end
S = Sd;
if numel(S) > s.maxRuns
    error('suite_matrix:tooMany', 'The settings give %d tests, more than MaxRuns = %d. Reduce the lists or raise MaxRuns.', numel(S), s.maxRuns);
end

%% ------------------------------------------------ configuration table (Table 1)
n = numel(S);
T = table();
T.scenario = string({S.id})';
T.tier = string(arrayfun(@(x) x.meta.tier, S, 'UniformOutput', false))';
T.group = string(arrayfun(@(x) x.meta.group, S, 'UniformOutput', false))';
T.system = repmat(string(cfg.system), n, 1);
T.windType = string(arrayfun(@(x) x.meta.windType, S, 'UniformOutput', false))';
T.windLevel = string(arrayfun(@(x) x.meta.windName, S, 'UniformOutput', false))';
T.windPeak_mps = arrayfun(@(x) x.meta.windPeak, S)';
T.windDir_deg = arrayfun(@(x) x.meta.dirDeg, S)';
T.windVertFrac = arrayfun(@(x) x.wind.vertFrac, S)';
T.gustOnset_s = arrayfun(@(x) x.meta.gustOnset, S)';
T.gustDuration_s = arrayfun(@(x) x.meta.gustDur, S)';
T.noiseLevel = string(arrayfun(@(x) x.meta.noiseName, S, 'UniformOutput', false))';
T.noiseMultiplier = arrayfun(@(x) x.meta.noiseLevel, S)';
T.processNoise = arrayfun(@(x) x.meta.processNoise, S)';
T.noiseToControllers = arrayfun(@(x) x.ctrlNoise, S)';
T.mismatchScale = arrayfun(@(x) x.meta.mismatch, S)';
T.failedMotor = arrayfun(@(x) x.meta.failedMotor, S)';
T.faultType = string(arrayfun(@(x) x.meta.faultType, S, 'UniformOutput', false))';
T.severity = arrayfun(@(x) x.meta.severity, S)';
T.faultTime_s = arrayfun(@(x) x.tFault, S)';
T.seed = arrayfun(@(x) x.meta.seed, S)';
end

function v = nz_(s, name)
if strcmp(name, 'stress'), v = s.noiseStress; else, v = s.noiseLevels.(name); end
end

function t = faultType_(rotor, sev)
if rotor == 0, t = 'none'; elseif sev >= 1, t = 'complete'; else, t = 'partial'; end
end

function rows = addrow(rows, tier, group, windType, windName, windPeak, dirDeg, onset, dur, noiseName, noiseLvl, proc, mism, rotor, sev, seed)
rows{end+1} = struct('tier',tier,'group',group,'windType',windType,'windName',windName,'windPeak',windPeak, ...
    'dirDeg',dirDeg,'onset',onset,'dur',dur,'noiseName',noiseName,'noiseLvl',noiseLvl,'proc',proc,'mism',mism, ...
    'rotor',rotor,'sev',sev,'seed',seed);
end

%% ----------------------------------------------------------------------------------------------------
%% MAKE_SCENARIOS
%% ----------------------------------------------------------------------------------------------------
function S = make_scenarios(cfg, stage, varargin)
%MAKE_SCENARIOS  Test scenarios for the tuning stage or the independent validation stage.
%
%   S = make_scenarios(cfg, 'tune')      small set, seeds cfg.tuneSeeds (used by the optimiser)
%   S = make_scenarios(cfg, 'validate')  wide set, seeds cfg.valSeeds (never seen by the optimiser)
%   S = make_scenarios(cfg, 'baseline')  the four canonical test classes, one run each
%
% Scenario kinds
%   baseline   healthy, no wind, no uncertainty
%   wind_only  healthy + wind            -> false alarms, wrongly latched FTC
%   fault_only single motor fault, no wind
%   combined   single motor fault + wind
% Plant uncertainty (uncScale = 1) is switched on for even seeds, off for odd seeds.

switch stage
    case 'tune',     S = tune_set(cfg);
    case 'validate', S = validate_set(cfg);
    case 'baseline', S = baseline_set(cfg);
    case 'one'       % make_scenarios(cfg,'one', kind, rotor, severity, profile, W, dirDeg, seed, unc, noise, proc, ctrlNoise)
        a = varargin;
        S = scn(cfg, a{1}, a{2}, a{3}, a{4}, a{5}, a{6}, a{7}, a{8});
        S.noiseScale = a{9};  S.procScale = a{10};  S.ctrlNoise = a{11};
        S.id = sprintf('%s_n%g_p%g_c%d', S.id, S.noiseScale, S.procScale, S.ctrlNoise);
    otherwise, error('make_scenarios:stage', 'stage must be tune | validate | baseline');
end
end

% ---------------------------------------------------------------------------
function S = tune_set(cfg)
S = [];
wm = cfg.windLevels(min(2, numel(cfg.windLevels)));       % mid wind level
for seed = cfg.tuneSeeds
    dirA = mod(37*seed + 20, 360);  u = double(mod(seed,2) == 0);
    S = [S, ...
      scn(cfg,'baseline', 0, 1, 'calm',      0,      dirA, seed, 0) ...
      scn(cfg,'wind_only',0, 1, 'gust',      wm,     dirA, seed, u) ...
      scn(cfg,'wind_only',0, 1, 'turbulence',wm,     dirA, seed, u) ...
      scn(cfg,'fault_only',1, 1, 'calm',     0,      dirA, seed, u) ...
      scn(cfg,'combined', 1, 1, 'gust',      wm,     dirA, seed, u) ...
      scn(cfg,'combined', 2, 1, 'mixed',     wm,     dirA, seed, u)]; %#ok<AGROW>
end
S = uniquify(S);
end

function S = validate_set(cfg)
profiles = {'steady','gust','dirchange','turbulence','mixed'};
seeds = cfg.valSeeds;
Sb = scn(cfg,'baseline',0,1,'calm',0,0,seeds(1),0);
Sw = []; Sf = []; Sc = [];
for seed = seeds
    u = double(mod(seed,2) == 0);
    dirA = mod(53*seed + 10, 360);
    for pr = profiles, for W = cfg.windLevels
        Sw = [Sw, scn(cfg,'wind_only',0,1,pr{1},W,dirA,seed,u)]; %#ok<AGROW>
    end, end
    for r = cfg.valRotors, for sev = cfg.valSeverity
        Sf = [Sf, scn(cfg,'fault_only',r,sev,'calm',0,dirA,seed,u)]; %#ok<AGROW>
        for pr = profiles, for W = cfg.windLevels
            Sc = [Sc, scn(cfg,'combined',r,sev,pr{1},W,dirA,seed,u)]; %#ok<AGROW>
        end, end
    end, end
end
% deterministic, stratified subsample so the cost stays bounded (own RNG stream)
cap = cfg.valMaxRuns;
if isfinite(cap)
    st = rng(1234, 'twister');  cleanup = onCleanup(@() rng(st)); %#ok<NASGU>
    nW = round(0.25*cap); nF = round(0.2*cap); nC = max(cap - 1 - nW - nF, 1);
    Sw = pick(Sw, nW);  Sf = pick(Sf, nF);  Sc = pick(Sc, nC);
end
S = uniquify([Sb, Sw, Sf, Sc]);
end

function S = baseline_set(cfg)
seed = 1;
S = [scn(cfg,'baseline', 0, 1, 'calm',   0, 0,  seed, 0), ...
     scn(cfg,'wind_only',0, 1, 'gust',   1, 30, seed, 0), ...
     scn(cfg,'fault_only',1, 1, 'calm',  0, 0,  seed, 0), ...
     scn(cfg,'combined', 1, 1, 'gust',   1, 30, seed, 0)];
end

function Sel = pick(S, n)
if isempty(S), Sel = S; return; end
n = min(n, numel(S));  idx = sort(randperm(numel(S), n));  Sel = S(idx);
end

function S = uniquify(S)
[~, ia] = unique({S.id}, 'stable');  S = S(ia);
end

function s = scn(cfg, kind, rotor, severity, profile, W, dirDeg, seed, uncScale)
w = struct('profile',profile,'W',W,'dirDeg',dirDeg,'onset',6,'dur',2,'interval',3,'nGust',6, ...
           'dirSeq',[0 90 90 90 90],'dwell',4,'seed',seed,'vertFrac',0,'tEnd',cfg.stopTime);
if strcmp(profile,'steady'), w.dur = inf; end
if strcmp(profile,'calm'), w.W = 0; end
s.kind = kind;  s.rotor = rotor;  s.severity = severity;  s.tFault = cfg.tFault;
s.wind = w;  s.seed = seed;  s.uncScale = uncScale;
s.noiseScale = 0;  s.procScale = 0;  s.ctrlNoise = 1;     % set by the robustness runner
s.id = sprintf('%s_%s_W%g_d%03d_R%d_sev%03d_s%d_u%d', kind, profile, W, round(dirDeg), rotor, ...
               round(100*severity), seed, uncScale);
end

%% ----------------------------------------------------------------------------------------------------
%% SUITE_METRICS: suite metrics and outcome classes
%% ----------------------------------------------------------------------------------------------------
function M = suite_metrics(cfg, sig, scn, V, M)
%SUITE_METRICS  Extra per-run metrics of the test suite (on top of compute_metrics.m), computed from the FULL-RATE signals.
%
% All times in s, lengths in m, angles in deg where the name ends in _deg, otherwise rad. Definitions:
%
% CONFIRMED DECLARATION (FDI output used for "detection")
%   Your detector raises Fault_True; with sensor noise this flag can be on all the time (see report). The FTC only acts
%   when Fault_True = 1 AND the estimated thrust loss of the isolated rotor alpha_hat(faulty_rotor) > confirm_alpha
%   (0.7). The suite therefore defines  decl(t) = Fault_True & faulty_rotor >= 1 & alpha_hat(faulty_rotor) > confirm_alpha
%   and calls the first t >= t_fault with decl = 1 the FDI detection time t_decl.  The raw Fault_True flag is still
%   reported (ft_on_frac, raw_*).
%
%   detect_delay_s   = t_decl - t_fault                              (NaN if never declared or declared before the fault)
%   det_success      = declared in [t_fault, t_fault + detWindow] and not declared before the fault
%   iso_correct      = faulty_rotor at t_decl equals the failed motor
%   false_alarm_conf = decl = 1 at any t >= tGrace before the fault (or anywhere, for healthy runs);
%   false_alarm_events = number of rising edges of decl in that period
%   ftc_delay_s      = t_ftc - t_fault, switch_delay_s = t_ftc - t_decl  (t_ftc: first t with max_i|w_cmd_i - w_nom_i| > ftcTol)
%   ftc_activation_failure = declared in time but the FTC took over later than ftcWindow (or never)
%
% RECOVERY (stable flight)
%   ok(t) = |z - z_ref| <= rec.altTol  &  horizontal error <= rec.horizTol  &  |vz| <= rec.vzTol
%           &  max(|wrap(roll - roll_eq)|, |wrap(pitch - pitch_eq)|) <= rec.tiltTol
%   (roll_eq, pitch_eq) = designed spinning-hover tilt of the failed rotor (seq_all(7:8,k)); zero for healthy runs.
%   Yaw is excluded: after the failure the vehicle spins about the vertical by design.
%   recovered_suite  = ok(t) holds continuously from some t_rec to the END of the run, for at least rec.settleTime
%   t_rec_s          = start of that final compliant interval
%   recovery_time_s  = max(0, t_rec - t_ftc)   (from FTC activation; NaN if the FTC never activated)
%   recovery_time_fault_s = max(0, t_rec - t_fault)
%
% SAFE DESCENT (only evaluated when recovered_suite = 0): touchdown = first t >= t_fault with z < descent.zGround.
%   safe_descent = |vz| <= maxVz, tilt <= maxTilt and horizontal error <= maxDrift at touchdown.
%
% OUTCOME (one label per run, the first that applies)
%   diverged | false_alarm_before_fault | missed_detection | late_detection | misisolation | ftc_activation_failure |
%   recovered | safe_descent | unsuccessful_recovery            (faulted runs)
%   diverged | false_alarm | unstable_healthy | healthy_ok                                 (healthy runs)
%
% Attitude / altitude / drift / saturation / effort / RMS metrics are defined next to their code below.

S = cfg.suite;  crit = S.rec;
t = sig.t(:);  N = numel(t);  dt = median(diff(t));
faulted = scn.rotor > 0;
tFw = scn.tFault;
tF  = tFw;  if faulted && isfinite(M.faultTime_true), tF = M.faultTime_true; end
M.t_fault_s = NaN;  if faulted, M.t_fault_s = tF; end

fields = {'t_decl_s','detect_delay_s','det_success','iso_correct','false_alarm_conf','false_alarm_events','declared', ...
 'ftc_delay_s','switch_delay_s','ftc_activation_failure','recovered_suite','t_rec_s','recovery_time_s','recovery_time_fault_s', ...
 'safe_descent','touchdown_vz','roll_dev_max_deg','pitch_dev_max_deg','yaw_dev_pre_max_deg','yaw_rate_max','roll_dev_eq_max_deg', ...
 'pitch_dev_eq_max_deg','alt_loss_m','drift_ref_max_m','drift_from_fault_max_m','sat_hi_time_s','sat_lo_time_s','sat_events', ...
 'sat_post_frac','sat_hi_time_pre_s','u_tv','u_energy_s','att_rms_pre_deg','att_rms_final_dev_deg','pos_rms_pre_m','pos_rms_final_m'};
for i = 1:numel(fields), M.(fields{i}) = NaN; end
M.outcome = "diverged";
if M.diverged > 0, return; end

eul = sig.eul;  p = sig.p;  pref = sig.pref;  wc = sig.wcmd;
wrap = @(a) atan2(sin(a), cos(a));
wmax = V.w_max;  whov = V.w_hover;

%% ---- confirmed declaration -------------------------------------------------------------
FT  = sig.FT > 0.5;  rot = round(sig.rotor);
rk  = max(1, rot);
aR  = sig.alpha(sub2ind(size(sig.alpha), (1:N)', rk));
decl = FT & rot >= 1 & aR > V.confirm_alpha & t >= cfg.tGrace;
edges = @(x) sum(diff([0; double(x(:))]) > 0);
if ~faulted
    M.false_alarm_conf = double(any(decl));
    M.false_alarm_events = edges(decl);
    M.declared = NaN;
else
    pre = decl & t < tFw;
    M.false_alarm_conf = double(any(pre));
    M.false_alarm_events = edges(pre);
    iD = find(decl & t >= tFw, 1);
    M.declared = double(~isempty(iD));
    if ~isempty(iD)
        M.t_decl_s = t(iD);
        if ~any(pre), M.detect_delay_s = M.t_decl_s - tF; end
        M.iso_correct = double(rot(iD) == scn.rotor);
    end
    M.det_success = double(~any(pre) && ~isempty(iD) && isfinite(M.detect_delay_s) && M.detect_delay_s <= S.detWindow);
    if M.ftc_false_activation == 0, M.ftc_delay_s = M.t_ftc - tF; end
    if isfinite(M.t_ftc) && isfinite(M.t_decl_s), M.switch_delay_s = M.t_ftc - M.t_decl_s; end
    M.ftc_activation_failure = double(M.det_success == 1 && ~(isfinite(M.t_ftc) && M.t_ftc - M.t_decl_s <= S.ftcWindow));
end

%% ---- errors, criteria, recovery ----------------------------------------------------------
e   = p - pref;  h = hypot(e(:,1), e(:,2));  alt = e(:,3);
vz  = gradient(movmean(p(:,3), 101), dt);
if faulted, eq = V.seq_all(7:8, scn.rotor); else, eq = [0; 0]; end
dRoll  = wrap(eul(:,1) - eq(1));  dPitch = wrap(eul(:,2) - eq(2));
tiltDev = max(abs(dRoll), abs(dPitch));
ok = abs(alt) <= crit.altTol & h <= crit.horizTol & abs(vz) <= crit.vzTol & tiltDev <= crit.tiltTol;
if faulted
    post = t >= tFw;
    iBad = find(~ok & post, 1, 'last');
    if isempty(iBad), tStart = tFw; elseif iBad == N, tStart = inf; else, tStart = t(iBad + 1); end
    M.recovered_suite = double(isfinite(tStart) && (t(end) - tStart) >= crit.settleTime);
    if M.recovered_suite
        M.t_rec_s = tStart;
        M.recovery_time_fault_s = max(0, tStart - tF);
        if isfinite(M.t_ftc), M.recovery_time_s = max(0, tStart - M.t_ftc); end
    end
    % safe descent (secondary classification)
    if S.descent.enable && M.recovered_suite == 0
        iT = find(post & p(:,3) < S.descent.zGround, 1);
        if ~isempty(iT)
            tiltTd = max(abs(wrap(eul(iT,1))), abs(wrap(eul(iT,2))));
            M.touchdown_vz = vz(iT);
            M.safe_descent = double(abs(vz(iT)) <= S.descent.maxVz && tiltTd <= S.descent.maxTilt && h(iT) <= S.descent.maxDrift);
        else
            M.safe_descent = 0;
        end
    else
        M.safe_descent = 0;
    end
end

%% ---- attitude, altitude, drift ----------------------------------------------------------------
post = t >= tFw;  preAll = t >= cfg.tGrace & t < tFw;
yawErr = wrap(eul(:,3) - sig.psiref);
M.yaw_dev_pre_max_deg = rad2deg(max(abs(yawErr(preAll))));
if faulted
    M.roll_dev_max_deg  = rad2deg(max(abs(wrap(eul(post,1)))));        % w.r.t. level (reference roll = 0)
    M.pitch_dev_max_deg = rad2deg(max(abs(wrap(eul(post,2)))));        % w.r.t. level (reference pitch = 0)
    yawRate = gradient(unwrap(eul(:,3)), dt);
    M.yaw_rate_max = max(abs(yawRate(post)));                          % rad/s, the designed recovery spin is ~39 rad/s
    iF = isfinite(M.t_ftc) * find(t >= M.t_ftc, 1);
    if ~isempty(iF) && iF > 0
        M.roll_dev_eq_max_deg  = rad2deg(max(abs(dRoll(iF:end))));     % w.r.t. the designed spinning-hover tilt
        M.pitch_dev_eq_max_deg = rad2deg(max(abs(dPitch(iF:end))));
    end
    iF0 = find(post, 1);
    M.alt_loss_m = max(0, p(iF0,3) - min(p(post,3)));                  % z(t_fault) - min z after the failure
    M.drift_ref_max_m = max(h(post));                                  % max distance to the reference position after the failure
    d0 = hypot(p(post,1) - p(iF0,1), p(post,2) - p(iF0,2));
    M.drift_from_fault_max_m = max(d0);                                % max displacement from the position at the failure
else
    M.roll_dev_max_deg  = rad2deg(max(abs(wrap(eul(preAll,1)))));
    M.pitch_dev_max_deg = rad2deg(max(abs(wrap(eul(preAll,2)))));
    M.drift_ref_max_m = max(h(t >= tFw - cfg.winPre));
end

%% ---- saturation (live rotors only) ---------------------------------------------------------------
live = true(N, 4);  if faulted, live(:, scn.rotor) = ~post; end
hiAny = any(wc >= wmax - 1e-9 & live, 2);  loAny = any(wc <= 1e-9 & live, 2);
M.sat_hi_time_pre_s = sum(hiAny(preAll))*dt;
if faulted
    anySat = hiAny | loAny;
    M.sat_hi_time_s = sum(hiAny(post))*dt;                             % time (s) with at least one live rotor at its upper limit w_max
    M.sat_lo_time_s = sum(loAny(post))*dt;                             % time (s) with at least one live rotor at its lower limit 0
    M.sat_events = edges(anySat(post));                                % number of separate saturation episodes
    M.sat_post_frac = mean(anySat(post));                              % share of the post-failure time spent saturated
end

%% ---- control effort ------------------------------------------------------------------------------------
if faulted
    lm = live(post, :);  W = wc(post, :);
    dW = abs(diff(W, 1, 1)) .* lm(2:end, :);
    M.u_tv = sum(dW, 'all')/wmax;                                      % total variation of the live motor commands (dimensionless)
    X = ((W - whov)/wmax).^2 .* lm;
    M.u_energy_s = sum(X, 'all')*dt;                                   % integral of squared normalised command deviation [s]
end

%% ---- RMS tracking errors ---------------------------------------------------------------------------------
pw = t >= tFw - cfg.winPre & t < tFw;    fw = t >= cfg.stopTime - cfg.winFinal;
M.att_rms_pre_deg = rad2deg(sqrt(mean(eul(pw,1).^2 + eul(pw,2).^2)));                 % roll/pitch vs level, pre-failure window
M.att_rms_final_dev_deg = rad2deg(sqrt(mean(dRoll(fw).^2 + dPitch(fw).^2)));          % roll/pitch vs the designed tilt, final window
M.pos_rms_pre_m = sqrt(mean(sum(e(pw,:).^2, 2)));                                     % 3-D position error, pre-failure window
M.pos_rms_final_m = sqrt(mean(sum(e(fw,:).^2, 2)));                                   % 3-D position error, final window

%% ---- outcome label ---------------------------------------------------------------------------------------
if ~faulted
    if M.false_alarm_conf > 0 || M.ftc_false_activation > 0, o = "false_alarm";
    elseif M.healthy_stable == 0, o = "unstable_healthy";
    else, o = "healthy_ok"; end
else
    if M.false_alarm_conf > 0 || M.ftc_false_activation > 0, o = "false_alarm_before_fault";
    elseif M.declared == 0, o = "missed_detection";
    elseif ~(M.det_success == 1), o = "late_detection";
    elseif M.iso_correct == 0, o = "misisolation";
    elseif M.ftc_activation_failure == 1, o = "ftc_activation_failure";
    elseif M.recovered_suite == 1, o = "recovered";
    elseif M.safe_descent == 1, o = "safe_descent";
    else, o = "unsuccessful_recovery"; end
end
M.outcome = o;
end

%% ----------------------------------------------------------------------------------------------------
%% COMPUTE_METRICS: base metrics
%% ----------------------------------------------------------------------------------------------------
function M = compute_metrics(cfg, sig, scn, V, simOK, msg)
%COMPUTE_METRICS  All per-run metrics (definitions: metric_definitions.m / README.md section 4).
%
%   sig  : struct from extract_signals (empty if the simulation failed)
%   scn  : scenario struct (make_scenarios.m)
%   V    : variables used for the run (needs seq_all, w_max, w_hover, psi_step_*)
%   M    : flat struct of scalars/strings -> one row of the results table
%
% Times are in s, angles in deg unless the field name says otherwise, lengths in m.
% NaN means "not defined for this run" (e.g. detection delay of a healthy run).

crit = cfg.crit;
M = struct();
M.scenario = string(scn.id);  M.kind = string(scn.kind);  M.profile = string(scn.wind.profile);
M.windSpeed = scn.wind.W;     M.windDirDeg = scn.wind.dirDeg;
M.noiseScale = scn.noiseScale;  M.procScale = scn.procScale;  M.ctrlNoise = scn.ctrlNoise;
M.failedRotor = scn.rotor;    M.severity = scn.severity;   M.seed = scn.seed;  M.uncScale = scn.uncScale;
M.faultTime_true = NaN;       M.sim_ok = simOK;            M.sim_msg = string(msg);

names = {'diverged','crash','recovered','survived','healthy_stable','success', ...
 'startup_alarm','ft_on_frac','flag2_false_alarm','flag2_delay','detected','false_alarm','prefault_alarm','ftc_activated','ftc_false_activation','iso_at_detect','iso_latched', ...
 'det_delay','ftc_delay','switch_delay','t_detect','t_ftc', 'flag1_alarm_frac','flag2_alarm_frac', ...
 'roll_max_pre','pitch_max_pre','roll_max_post','pitch_max_post','yaw_err_pre','spin_rate_final','tilt_dev_final', ...
 'rmse_h_pre','rmse_z_pre','max_h_pre','max_z_pre','max_h_post','max_z_post','z_drop','z_min_post','max_descent_rate', ...
 'alt_err_final','horiz_err_final','tilt_final', ...
 'effort_rms_pre','effort_rms_post','u_peak_ratio','sat_hi_pre','sat_hi_post','sat_lo_post','alpha_est_final'};
for i = 1:numel(names), M.(names{i}) = NaN; end

% ---- failed simulation ------------------------------------------------------
if ~simOK || isempty(sig)
    M.diverged = 1;  M.crash = 1;  M.recovered = 0;  M.survived = 0;  M.healthy_stable = 0;  M.success = 0;
    M.detected = 0;  M.ftc_activated = 0;
    if scn.rotor == 0, M.false_alarm = 0; end
    return
end

t = sig.t(:);  N = numel(t);  dt = median(diff(t));
eul = sig.eul;  p = sig.p;  pref = sig.pref;  wc = sig.wcmd;  wn = sig.wnom;
FT = sig.FT > 0.5;
M.startup_alarm = double(any(FT(sig.t < cfg.tGrace)));   % observers start from zero; reported, not counted
FT(sig.t < cfg.tGrace) = false;                            % (the older Judgment has no start-up grace)
F1a = sig.F1 < 1;  F2a = sig.F2 < 1;
faulted = scn.rotor > 0;

% true fault time, from the health signals actually applied to the plant
hit = any(sig.health < 1 - 1e-9, 2);
if any(hit), tF = t(find(hit,1)); else, tF = NaN; end
M.faultTime_true = tF;
tFw = scn.tFault;                               % window anchor (also used for healthy runs)

% ---- numerical divergence ---------------------------------------------------
fin = all(isfinite([eul p wc sig.w]), 'all');
M.diverged = double(~fin || t(end) < cfg.stopTime - 2*dt || max(abs(p(:))) > crit.divergeP);
if ~fin
    % keep the finite part for the remaining metrics
    good = all(isfinite([eul p wc]), 2);
    N = find(~good, 1) - 1;  if N < 5, N = 5; end
    t = t(1:N); eul = eul(1:N,:); p = p(1:N,:); pref = pref(1:N,:); wc = wc(1:N,:); wn = wn(1:N,:);
    FT = FT(1:N); F1a = F1a(1:N); F2a = F2a(1:N); sig.health = sig.health(1:N,:); sig.alpha = sig.alpha(1:N,:);
    sig.rotor = sig.rotor(1:N);
end

err   = p - pref;
horiz = hypot(err(:,1), err(:,2));
alt   = err(:,3);
tilt  = max(abs(eul(:,1)), abs(eul(:,2)));
yawErr = atan2(sin(eul(:,3) - sig.psiref(1:N)), cos(eul(:,3) - sig.psiref(1:N)));
wrapDeg = @(x) rad2deg(x);

preAll  = t >= cfg.tGrace & t < tFw;
preWin  = t >= tFw - cfg.winPre & t < tFw;
postAll = t >= tFw;
finWin  = t >= cfg.stopTime - cfg.winFinal;
liveMask = true(N,4);
if faulted, liveMask(:, scn.rotor) = ~(t >= tFw); end    % failed rotor excluded after onset

% ---- attitude ----------------------------------------------------------------
M.roll_max_pre   = wrapDeg(max(abs(eul(preAll,1))));
M.pitch_max_pre  = wrapDeg(max(abs(eul(preAll,2))));
M.yaw_err_pre    = wrapDeg(max(abs(yawErr(preWin))));
if any(postAll)
    M.roll_max_post  = wrapDeg(max(abs(eul(postAll,1))));
    M.pitch_max_post = wrapDeg(max(abs(eul(postAll,2))));
end
yawRate = gradient(unwrap(eul(:,3)), dt);
M.spin_rate_final = mean(yawRate(finWin));                       % rad/s, signed
M.tilt_final      = wrapDeg(max(tilt(finWin)));
if faulted
    sq = V.seq_all(7:8, scn.rotor);                              % designed spinning-hover tilt
    M.tilt_dev_final = wrapDeg(max(max(abs(eul(finWin,1) - sq(1)), abs(eul(finWin,2) - sq(2)))));
end

% ---- position / altitude ------------------------------------------------------
M.rmse_h_pre = sqrt(mean(horiz(preWin).^2));   M.rmse_z_pre = sqrt(mean(alt(preWin).^2));
M.max_h_pre  = max(horiz(preWin));             M.max_z_pre  = max(abs(alt(preWin)));
M.max_h_post = max(horiz(postAll));            M.max_z_post = max(abs(alt(postAll)));
M.alt_err_final   = max(abs(alt(finWin)));
M.horiz_err_final = max(horiz(finWin));
zz = p(:,3);
iF = find(postAll, 1);
M.z_min_post = min(zz(postAll));
M.z_drop = max(0, zz(iF) - M.z_min_post);                        % altitude lost after fault onset
vz = gradient(movmean(zz, 21), dt);
M.max_descent_rate = max(0, -min(vz(postAll)));                  % m/s

% ---- control effort / saturation ----------------------------------------------
whov = V.w_hover;  wmax = V.w_max;
ne = (wc - whov)/wmax;
pw = preWin;
M.effort_rms_pre = sqrt(mean(ne(pw,:).^2, 'all'));
lm = liveMask & postAll;
if any(lm(:))
    M.effort_rms_post = sqrt(mean(ne(lm).^2));
    wl = wc(lm);
    M.sat_hi_post = mean(wl >= wmax - 1e-9);
    M.sat_lo_post = mean(wl <= 1e-9);
end
M.sat_hi_pre = mean(wc(preAll,:) >= wmax - 1e-9, 'all');
lmAll = liveMask & (t >= cfg.tGrace);
M.u_peak_ratio = max(wc(lmAll)) / wmax;

% ---- detection / FTC timing -----------------------------------------------------
act = max(abs(wc - wn), [], 2) > cfg.ftcTol;                     % FTC is driving the rotors
M.flag1_alarm_frac = mean(F1a(t >= cfg.tGrace));
M.flag2_alarm_frac = mean(F2a(t >= cfg.tGrace));
M.alpha_est_final  = NaN;
if faulted
    M.alpha_est_final = mean(sig.alpha(finWin, scn.rotor));
end

M.ft_on_frac = mean(FT(t >= cfg.tGrace & t < tFw));        % share of pre-fault time Fault_True is up
pre2 = F2a & t >= cfg.tGrace & t < tFw;
M.flag2_false_alarm = double(any(pre2));
if faulted
    i2 = find(F2a & t >= tFw, 1);
    if ~isempty(i2) && ~M.flag2_false_alarm, M.flag2_delay = t(i2) - tFw; end
end
if ~faulted
    M.false_alarm = double(any(FT));
    M.ftc_false_activation = double(any(act));
    M.detected = NaN;  M.ftc_activated = double(any(act));
    if any(FT), M.t_detect = t(find(FT,1)); end
    if any(act), M.t_ftc = t(find(act,1)); end
else
    M.prefault_alarm = double(any(FT(t < tFw)));
    M.false_alarm = M.prefault_alarm;
    M.ftc_false_activation = double(any(act(t < tFw)));
    iDet = find(FT & t >= tFw, 1);
    M.detected = double(~isempty(iDet));
    iFtc = find(act & t >= tFw, 1);
    M.ftc_activated = double(~isempty(iFtc));
    if ~isempty(iDet), M.t_detect = t(iDet); end
    if ~isempty(iFtc), M.t_ftc = t(iFtc); end
    if M.prefault_alarm == 0
        M.det_delay = M.t_detect - tFw;
        M.switch_delay = M.t_ftc - M.t_detect;
    end
    if M.ftc_false_activation == 0, M.ftc_delay = M.t_ftc - tFw; end   % valid even if Fault_True was already up
    if ~isempty(iDet), M.iso_at_detect = double(sig.rotor(iDet) == scn.rotor); end
    if ~isempty(iFtc), M.iso_latched   = double(sig.rotor(iFtc) == scn.rotor); end
end

% ---- verdicts -----------------------------------------------------------------
late = t >= crit.zCrashAfter;
% Tilt is NOT a crash criterion: the recovery manoeuvre of this FTC legitimately passes
% through large roll/pitch angles (> 90 deg observed) while the vehicle spins up.
M.crash = double(any(zz(late) < crit.zCrash) || ...
                 any(horiz(late) > crit.flyaway) || any(abs(alt(late)) > crit.flyaway));
if M.diverged, M.crash = 1; end
M.survived = double(~M.diverged && ~M.crash);
if faulted
    okFinal = M.alt_err_final <= crit.recAltErr && M.horiz_err_final <= crit.recHorizErr && ...
              deg2rad(M.tilt_final) <= crit.recTilt;
    M.recovered = double(M.survived && okFinal);
    M.success = M.recovered;
else
    okH = deg2rad(max(M.roll_max_pre, M.pitch_max_pre)) <= crit.healthyTilt && ...
          max(M.max_h_pre, M.max_z_pre) <= crit.healthyPos;
    M.healthy_stable = double(M.survived && okH);
    M.success = double(M.healthy_stable && ~M.false_alarm && ~M.ftc_false_activation);
end
end

%% ----------------------------------------------------------------------------------------------------
%% SUITE_TABLES
%% ----------------------------------------------------------------------------------------------------
function Tab = suite_tables(cfg, T)
%SUITE_TABLES  The eight required tables from the per-run results table T (one row per run).
%
%   Tab.scenarios   1  test scenario configuration (written by run_test_suite from suite_matrix)
%   Tab.perrun      2  per-run performance results (T itself)
%   Tab.fdi         3  FDI detection and isolation summary (by tier and fault type)
%   Tab.ftc         4  FTC activation and recovery summary (by tier and fault type)
%   Tab.wind        5  performance across wind conditions (type x strength x direction)
%   Tab.noise       6  performance across noise levels
%   Tab.motor       7  performance across motor failure cases
%   Tab.combined    8  combined wind / noise / failure robustness summary
%   Tab.outcomes    additional: counts of every outcome label by tier
%
% Rates are fractions of the runs in the row (NaN-safe). "Faulted" rows count only runs with a failed motor, "healthy"
% rows only runs without one. Delay statistics use the runs where the delay is defined (NaN otherwise).
Tab = struct();
Tab.perrun = T;
F = T.failedMotor > 0;
rows = struct('name', {}, 'mask', {});
Tab.fdi = group_table(T, {'tier', 'faultType'}, @fdi_row);
Tab.ftc = group_table(T(F, :), {'tier', 'faultType'}, @ftc_row);
Tab.wind = group_table(T, {'windType', 'windLevel', 'windDir_deg'}, @generic_row);
Tab.noise = group_table(T, {'noiseLevel', 'noiseMultiplier'}, @generic_row);
Tab.motor = group_table(T, {'failedMotor', 'faultType'}, @generic_row);
Tab.combined = group_table(T, {'windLevel', 'noiseLevel', 'mismatchScale', 'failedMotor'}, @generic_row);
oc = groupsummary(T, {'tier', 'outcome'});
Tab.outcomes = unstack(oc, 'GroupCount', 'outcome');
Tab.outcomes = fillmissing(Tab.outcomes, 'constant', 0, 'DataVariables', @isnumeric);
clear rows
end

% ------------------------------------------------------------------------------------------
function G = group_table(T, keys, rowfn)
if isempty(T), G = table(); return; end
K = T(:, keys);
for j = 1:width(K), if iscategorical(K.(keys{j})), K.(keys{j}) = string(K.(keys{j})); end, end
[u, ~, ic] = unique(K, 'rows', 'stable');
G = table();
for i = 1:height(u)
    r = rowfn(T(ic == i, :));
    G = [G; [u(i, :), r]]; %#ok<AGROW>
end
end

function r = fdi_row(g)
H = g.failedMotor == 0;  F = ~H;
r = table(height(g), nnz(H), nnz(F), ...
    rate(g.false_alarm_conf(H) > 0 | g.ftc_false_activation(H) > 0), mean(g.ft_on_frac(H), 'omitnan'), ...
    rate(g.det_success(F) == 1), rate(g.declared(F) == 1), ...
    stat(g.detect_delay_s(F), @mean), stat(g.detect_delay_s(F), @median), stat(g.detect_delay_s(F), @(v) pct(v, 95)), stat(g.detect_delay_s(F), @max), ...
    rate(g.iso_correct(F) == 1), rate(g.false_alarm_conf(F) > 0 | g.ftc_false_activation(F) > 0), ...
    'VariableNames', {'n_runs','n_healthy','n_faulted','false_alarm_rate_healthy','raw_FaultTrue_on_frac_healthy', ...
    'det_success_rate','declared_rate','detect_delay_mean_s','detect_delay_median_s','detect_delay_p95_s','detect_delay_max_s', ...
    'isolation_accuracy','prefault_false_alarm_rate'});
end

function r = ftc_row(g)
r = table(height(g), ...
    rate(g.ftc_activated == 1), rate(g.ftc_activation_failure == 1), ...
    stat(g.ftc_delay_s, @mean), stat(g.ftc_delay_s, @median), stat(g.ftc_delay_s, @(v) pct(v, 95)), ...
    stat(g.switch_delay_s, @mean), stat(g.switch_delay_s, @median), ...
    rate(g.recovered_suite == 1), rate(g.safe_descent == 1), rate(g.outcome == "unsuccessful_recovery"), rate(g.diverged > 0), rate(g.crash > 0), ...
    stat(g.recovery_time_s, @mean), stat(g.recovery_time_s, @median), stat(g.recovery_time_s, @(v) pct(v, 95)), ...
    stat(g.alt_loss_m, @median), stat(g.alt_loss_m, @max), stat(g.drift_ref_max_m, @median), stat(g.drift_ref_max_m, @max), ...
    stat(g.roll_dev_max_deg, @max), stat(g.pitch_dev_max_deg, @max), ...
    stat(g.sat_hi_time_s, @mean), stat(g.sat_lo_time_s, @mean), stat(g.sat_events, @mean), stat(g.u_tv, @mean), stat(g.u_energy_s, @mean), ...
    stat(g.att_rms_final_dev_deg, @mean), stat(g.pos_rms_final_m, @mean), ...
    'VariableNames', {'n_faulted','ftc_activation_rate','ftc_activation_failure_rate','ftc_delay_mean_s','ftc_delay_median_s','ftc_delay_p95_s', ...
    'switch_delay_mean_s','switch_delay_median_s','recovery_rate','safe_descent_rate','unsuccessful_rate','diverged_rate','crash_rate', ...
    'recovery_time_mean_s','recovery_time_median_s','recovery_time_p95_s','alt_loss_median_m','alt_loss_max_m','drift_median_m','drift_max_m', ...
    'roll_dev_max_deg','pitch_dev_max_deg','sat_hi_time_mean_s','sat_lo_time_mean_s','sat_events_mean','u_tv_mean','u_energy_mean_s', ...
    'att_rms_final_dev_deg','pos_rms_final_m'});
end

function r = generic_row(g)
H = g.failedMotor == 0;  F = ~H;
r = table(height(g), nnz(H), nnz(F), ...
    rate(g.outcome(H) == "healthy_ok"), rate(g.false_alarm_conf(H) > 0 | g.ftc_false_activation(H) > 0), rate(g.diverged(H) > 0), ...
    stat(g.pos_rms_pre_m(H), @mean), ...
    rate(g.det_success(F) == 1), rate(g.iso_correct(F) == 1), rate(g.ftc_activated(F) == 1), rate(g.recovered_suite(F) == 1), ...
    rate(g.safe_descent(F) == 1), rate(g.diverged(F) > 0), ...
    stat(g.detect_delay_s(F), @median), stat(g.ftc_delay_s(F), @median), stat(g.recovery_time_s(F), @median), ...
    stat(g.alt_loss_m(F), @median), stat(g.alt_loss_m(F), @max), stat(g.drift_ref_max_m(F), @median), stat(g.sat_events(F), @mean), ...
    'VariableNames', {'n_runs','n_healthy','n_faulted','healthy_ok_rate','false_alarm_rate','healthy_diverged_rate','healthy_pos_rms_m', ...
    'det_success_rate','isolation_accuracy','ftc_activation_rate','recovery_rate','safe_descent_rate','faulted_diverged_rate', ...
    'detect_delay_median_s','ftc_delay_median_s','recovery_time_median_s','alt_loss_median_m','alt_loss_max_m','drift_median_m','sat_events_mean'});
end

function v = rate(x)
x = x(:);  if isempty(x), v = NaN; else, v = mean(double(x), 'omitnan'); end
end

function v = stat(x, f)
x = x(isfinite(x));  if isempty(x), v = NaN; else, v = f(x); end
end

function q = pct(x, p)
x = sort(x(:));  n = numel(x);
if n == 1, q = x; return; end
r = 1 + (n - 1)*p/100;  lo = floor(r);  hi = ceil(r);  q = x(lo) + (r - lo)*(x(hi) - x(lo));
end

%% ----------------------------------------------------------------------------------------------------
%% SUITE_PLOTS
%% ----------------------------------------------------------------------------------------------------
function suite_plots(cfg, R)
%SUITE_PLOTS  Individual-run plots, comparison overlays and summary charts (PNG + PDF).
%   R fields: S (scenario structs), T (per-run table), V0 (workspace variables), Tab (tables), p (parameter struct)
% Event markers on every time plot:  black dashed = failure injection, magenta dotted = confirmed FDI detection,
% green = FTC activation, blue dashed = start of the stable-flight interval (recovery).
fd = fullfile(cfg.dirOut, 'figures');   if ~exist(fd, 'dir'), mkdir(fd); end
S = R.S;  T = R.T;
sel = select_detail(cfg, S, T);
fprintf('suite_plots: %d detailed run plots\n', numel(sel));
for k = sel(:)'
    [~, sg] = run_one(cfg, R.V0, R.p, S(k), true);
    if isempty(sg), continue; end
    plot_run(cfg, sg, T(k, :), S(k), R.V0, fullfile(fd, ['run_' S(k).id]));
end
compare_plot(cfg, R, 'cmp_wind_level', 'Wind strength (steady wind, 45 deg, motor 1 fails)', @(m) sprintf('%s: %.2f m/s', m.windName, m.windPeak), fullfile(fd, 'compare_wind_strength'));
compare_plot(cfg, R, 'cmp_wind_dir',   'Wind direction (steady 0.3 m/s, motor 1 fails)', @(m) sprintf('%g deg', m.dirDeg), fullfile(fd, 'compare_wind_direction'));
compare_plot(cfg, R, 'cmp_noise',      'Sensor noise level (no wind, motor 1 fails)', @(m) sprintf('noise x%g (%s)', m.noiseLevel, m.noiseName), fullfile(fd, 'compare_noise'));
summary_charts(cfg, T, fd);
end

% ---------------------------------------------------------------------------------------------
function sel = select_detail(cfg, S, T)
d = cfg.suite.detailPlots;
if iscell(d), sel = find(ismember({S.id}, d)); return; end
switch d
    case 'none', sel = [];
    case 'all',  sel = 1:numel(S);
    otherwise                                   % first run of every tier / group
        keep = false(1, numel(S));  seen = containers.Map();
        for k = 1:numel(S)
            key = [S(k).meta.tier '|' num2str(S(k).meta.failedMotor > 0)];
            if ~isKey(seen, key), seen(key) = 1; keep(k) = true; end
        end
        for k = 1:numel(S)                      % plus one run per combined scenario family and per partial loss
            if contains(S(k).meta.group, 'worst_case') && S(k).meta.failedMotor == 1 && S(k).meta.seed == cfg.suite.seeds(1), keep(k) = true; end
            if contains(S(k).meta.group, 'baseline') && S(k).meta.failedMotor == 4, keep(k) = true; end
        end
        sel = find(keep);
end
end

% ---------------------------------------------------------------------------------------------
function plot_run(cfg, sg, Mr, scn, V, base)
t = sg.t;  tF = sg.faultTime;
faulted = scn.rotor > 0;
f = figure('Visible', 'off', 'Position', [30 30 1400 1250]);
tl = tiledlayout(5, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
tDecl = Mr.t_decl_s;  tFtc = Mr.t_ftc;  tRec = Mr.t_rec_s;
ax = gobjects(10, 1);  for i = 1:10, ax(i) = nexttile; hold(ax(i), 'on'); grid(ax(i), 'on'); end
act = max(abs(sg.wcmd - sg.wnom), [], 2) > cfg.ftcTol;
if faulted, eq = V.seq_all(7:8, scn.rotor); else, eq = [0; 0]; end
% 1 roll / pitch
plot(ax(1), t, rad2deg(sg.eul(:,1)), 'DisplayName', 'roll');  plot(ax(1), t, rad2deg(sg.eul(:,2)), 'DisplayName', 'pitch');
yline(ax(1), 0, 'k:', 'DisplayName', 'reference (level)');
if faulted, yline(ax(1), rad2deg(eq(1)), '--', 'Color', [0.2 0.4 0.8], 'DisplayName', 'designed tilt (roll)'); yline(ax(1), rad2deg(eq(2)), '--', 'Color', [0.8 0.4 0.1], 'DisplayName', 'designed tilt (pitch)'); end
title(ax(1), 'Roll and pitch'); ylabel(ax(1), 'angle [deg]');
% 2 yaw
plot(ax(2), t, rad2deg(atan2(sin(sg.eul(:,3)), cos(sg.eul(:,3)))), '.', 'MarkerSize', 3, 'DisplayName', 'yaw (wrapped)');
plot(ax(2), t, rad2deg(sg.psiref), 'k--', 'DisplayName', 'yaw reference');
title(ax(2), 'Yaw (spins about the vertical after the failure by design)'); ylabel(ax(2), 'angle [deg]');
% 3 altitude
plot(ax(3), t, sg.p(:,3), 'DisplayName', 'altitude z');  plot(ax(3), t, sg.pref(:,3), 'k--', 'DisplayName', 'reference');
title(ax(3), 'Altitude'); ylabel(ax(3), 'z [m]');
% 4 horizontal
plot(ax(4), t, sg.p(:,1), 'DisplayName', 'x');  plot(ax(4), t, sg.p(:,2), 'DisplayName', 'y');
plot(ax(4), t, sg.pref(:,1), ':', 'Color', [0.2 0.4 0.8], 'DisplayName', 'x ref');  plot(ax(4), t, sg.pref(:,2), ':', 'Color', [0.8 0.4 0.1], 'DisplayName', 'y ref');
title(ax(4), 'Horizontal position'); ylabel(ax(4), 'x, y [m]');
% 5 attitude residuals + bounds
cl = lines(5);  attNames = {'roll residual','pitch residual'};  posNames = {'x residual','y residual','z residual'};
for j = 1:2
    plot(ax(5), t, sg.resid(:,j), 'Color', cl(j,:), 'DisplayName', attNames{j});
    if any(isfinite(sg.rupper(:,j)))
        plot(ax(5), t, sg.rupper(:,j), 'k--', 'LineWidth', 1.6, 'HandleVisibility', 'off');  plot(ax(5), t, sg.rlower(:,j), 'k--', 'LineWidth', 1.6, 'DisplayName', 'threshold (upper/lower)');
    end
end
title(ax(5), 'FDI residuals (attitude) and adaptive thresholds'); ylabel(ax(5), 'residual [rad]');
% 6 position residuals + bounds
for j = 3:5
    plot(ax(6), t, sg.resid(:,j), 'Color', cl(j,:), 'DisplayName', posNames{j-2});
    if any(isfinite(sg.rupper(:,j)))
        plot(ax(6), t, sg.rupper(:,j), 'k--', 'LineWidth', 1.6, 'HandleVisibility', 'off');  plot(ax(6), t, sg.rlower(:,j), 'k--', 'LineWidth', 1.6, 'HandleVisibility', 'off');
    end
end
title(ax(6), 'FDI residuals (position) and adaptive thresholds (black dashed)'); ylabel(ax(6), 'residual [m]');
% 7 indicators
a = sg.alpha;  rk = max(1, round(sg.rotor));  aR = a(sub2ind(size(a), (1:numel(t))', rk));
decl = sg.FT > 0.5 & round(sg.rotor) >= 1 & aR > V.confirm_alpha & t >= cfg.tGrace;
stairs(ax(7), t, double(sg.FT > 0.5) + 4.4, 'DisplayName', 'Fault\_True (raw)');
stairs(ax(7), t, double(sg.F1 < 1) + 3.3, 'DisplayName', 'Flag 1 alarm');  stairs(ax(7), t, double(sg.F2 < 1) + 2.2, 'DisplayName', 'Flag 2 alarm');
stairs(ax(7), t, double(decl) + 1.1, 'DisplayName', 'confirmed detection');  stairs(ax(7), t, double(act), 'DisplayName', 'FTC active');
set(ax(7), 'YTick', [0.5 1.6 2.7 3.8 4.9], 'YTickLabel', {'FTC','confirmed','Flag2','Flag1','Fault\_True'}); ylim(ax(7), [-0.2 6]);
title(ax(7), 'Fault detection and FTC activation indicators');
% 8 motor commands
live = true(numel(t), 4);  if faulted, live(:, scn.rotor) = ~(t >= tF); end
W = sg.wcmd;  W(~live) = NaN;
for j = 1:4, plot(ax(8), t, W(:,j), 'DisplayName', sprintf('motor %d', j)); end
yline(ax(8), V.w_max, 'r-', 'DisplayName', 'upper limit w_{max}');  yline(ax(8), 0, 'r--', 'DisplayName', 'lower limit');
ylim(ax(8), [-0.15*V.w_max, 1.2*V.w_max]);
title(ax(8), 'Motor speed commands (failed motor masked after the failure)'); ylabel(ax(8), 'w_{cmd} [rad/s]');
% 9 recovery criteria
wrap = @(x) atan2(sin(x), cos(x));
eqT = repmat(eq(:)', numel(t), 1);  if faulted, eqT(t < tF, :) = 0; end      % designed tilt applies only after the failure
tilt = max(abs(wrap(sg.eul(:,1) - eqT(:,1))), abs(wrap(sg.eul(:,2) - eqT(:,2))));
e = sg.p - sg.pref;  vz = gradient(movmean(sg.p(:,3), max(3, round(0.1/median(diff(t))))), t);
S_ = cfg.suite.rec;
plot(ax(9), t, abs(e(:,3))/S_.altTol, 'DisplayName', '|z error| / tol');  plot(ax(9), t, hypot(e(:,1), e(:,2))/S_.horizTol, 'DisplayName', 'horizontal error / tol');
plot(ax(9), t, abs(vz)/S_.vzTol, 'DisplayName', '|vz| / tol');  plot(ax(9), t, tilt/S_.tiltTol, 'DisplayName', 'tilt deviation / tol');
yline(ax(9), 1, 'k-', 'DisplayName', 'criterion (1 = at tolerance)'); ylim(ax(9), [0 3]);
title(ax(9), 'Recovery criteria (each normalised by its tolerance; all must stay below 1)'); ylabel(ax(9), 'ratio [-]');
% 10 wind force
plot(ax(10), t, sg.windF(:,1), 'DisplayName', 'F_x');  plot(ax(10), t, sg.windF(:,2), 'DisplayName', 'F_y');  plot(ax(10), t, sg.windF(:,3), 'DisplayName', 'F_z');
title(ax(10), 'Wind disturbance force'); ylabel(ax(10), 'force [N]');
% markers
for i = 1:10
    if faulted && isfinite(tF), xline(ax(i), tF, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
    if isfinite(tDecl), xline(ax(i), tDecl, ':', 'Color', [0.8 0 0.8], 'LineWidth', 1.5, 'HandleVisibility', 'off'); end
    if isfinite(tFtc), xline(ax(i), tFtc, '-', 'Color', [0 0.6 0.2], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
    if isfinite(tRec), xline(ax(i), tRec, '--', 'Color', [0 0.3 0.9], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
    xlim(ax(i), [0 t(end)]);  if i >= 9, xlabel(ax(i), 't [s]'); end
    if i ~= 10, legend(ax(i), 'Location', 'best', 'FontSize', 7, 'NumColumns', 2); else, legend(ax(i), 'Location', 'best', 'FontSize', 7); end
end
mk = {}; if faulted, mk{end+1} = sprintf('failure %.2f s', tF); end
if isfinite(tDecl), mk{end+1} = sprintf('detection %.3f s', tDecl); end
if isfinite(tFtc), mk{end+1} = sprintf('FTC %.3f s', tFtc); end
if isfinite(tRec), mk{end+1} = sprintf('stable from %.2f s', tRec); end
title(tl, sprintf('%s   |   outcome: %s\n%s   (black dashed = failure, magenta = detection, green = FTC activation, blue = stable flight)', ...
    strrep(scn.id, '_', '\_'), char(Mr.outcome), strjoin(mk, ', ')), 'FontSize', 10);
save_fig(f, base, cfg);
end

% ---------------------------------------------------------------------------------------------
function compare_plot(cfg, R, grp, ttl, labelFn, base)
S = R.S;  idx = find(arrayfun(@(s) contains(s.meta.group, grp), S));
if numel(idx) < 2, return; end
[~, o] = sort(arrayfun(@(k) sortkey(S(k).meta, grp), idx));  idx = idx(o);
f = figure('Visible', 'off', 'Position', [30 30 1300 800]);
tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = gobjects(6, 1);  for i = 1:6, ax(i) = nexttile; hold(ax(i), 'on'); grid(ax(i), 'on'); end
cols = lines(numel(idx));
for q = 1:numel(idx)
    k = idx(q);  [~, sg] = run_one(cfg, R.V0, R.p, S(k), true);  if isempty(sg), continue; end
    t = sg.t;  c = cols(q, :);  lab = labelFn(S(k).meta);
    plot(ax(1), t, sg.p(:,3), 'Color', c, 'DisplayName', lab);
    plot(ax(2), t, hypot(sg.p(:,1) - sg.pref(:,1), sg.p(:,2) - sg.pref(:,2)), 'Color', c, 'DisplayName', lab);
    plot(ax(3), t, rad2deg(sg.eul(:,1)), 'Color', c, 'DisplayName', lab);
    plot(ax(4), t, rad2deg(sg.eul(:,2)), 'Color', c, 'DisplayName', lab);
    live = true(numel(t), 4);  if S(k).rotor > 0, live(:, S(k).rotor) = ~(t >= sg.faultTime); end
    W = sg.wcmd;  W(~live) = NaN;  plot(ax(5), t, max(W, [], 2), 'Color', c, 'DisplayName', lab);
    act = max(abs(sg.wcmd - sg.wnom), [], 2) > cfg.ftcTol;  stairs(ax(6), t, double(act) + 1.2*(q-1), 'Color', c, 'DisplayName', lab);
    if q == 1, tF = sg.faultTime; end
end
ttls = {'Altitude','Horizontal error to the reference','Roll','Pitch','Largest live motor command','FTC active (offset per case)'};
yl = {'z [m]','distance [m]','roll [deg]','pitch [deg]','w_{cmd} [rad/s]','indicator'};
for i = 1:6
    title(ax(i), ttls{i}); ylabel(ax(i), yl{i}); xlabel(ax(i), 't [s]');
    if exist('tF', 'var') && isfinite(tF), xline(ax(i), tF, 'k--', 'HandleVisibility', 'off'); end
    legend(ax(i), 'Location', 'best', 'FontSize', 7);
end
yline(ax(5), cfg_wmax(R), 'r-', 'HandleVisibility', 'off');
title(tl, ttl);
save_fig(f, base, cfg);
end

function k = sortkey(m, grp)
switch grp
    case 'cmp_wind_level', k = m.windPeak;
    case 'cmp_wind_dir',   k = m.dirDeg;
    otherwise,             k = m.noiseLevel;
end
end

function w = cfg_wmax(R), w = R.V0.w_max; end

% ---------------------------------------------------------------------------------------------
function summary_charts(cfg, T, fd)
F = T(T.failedMotor > 0 & T.faultType == "complete", :);
H = T(T.failedMotor == 0, :);
nlv = unique(F.noiseMultiplier)';  wlv = unique(F.windPeak_mps)';
% 1 detection / FTC delay and recovery time vs noise level (no wind), per motor
f = figure('Visible', 'off', 'Position', [30 30 1300 450]);  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
Fn = F(F.windPeak_mps == 0 & F.mismatchScale == 0, :);
mets = {'detect_delay_s','ftc_delay_s','recovery_time_s'};  tt = {'FDI detection delay','FTC activation delay','Recovery time (from FTC activation)'};
for m = 1:3
    ax = nexttile; hold(ax, 'on'); grid(ax, 'on');
    for mot = 1:4
        g = Fn(Fn.failedMotor == mot, :);  y = nan(size(nlv));
        for q = 1:numel(nlv), v = g.(mets{m})(g.noiseMultiplier == nlv(q)); v = v(isfinite(v)); if ~isempty(v), y(q) = median(v); end, end
        plot(ax, 1:numel(nlv), y, '-o', 'DisplayName', sprintf('motor %d', mot));
    end
    set(ax, 'XTick', 1:numel(nlv), 'XTickLabel', string(nlv)); xlabel(ax, 'sensor-noise multiplier [x noise\_params sigmas]'); ylabel(ax, 'median time [s]');
    title(ax, tt{m}); legend(ax, 'Location', 'best', 'FontSize', 7);
end
title(tl, 'Timing against sensor-noise level (complete failure, no wind)');  save_fig(f, fullfile(fd, 'summary_timing_vs_noise'), cfg);
% 2 altitude loss, drift, recovery vs wind level (no noise)
f = figure('Visible', 'off', 'Position', [30 30 1300 450]);  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
Fw = F(F.noiseMultiplier == 0 & F.mismatchScale == 0 & F.windType ~= "calm" | F.windPeak_mps == 0 & F.noiseMultiplier == 0 & F.mismatchScale == 0, :);
units3 = {'[m]','[m]','[-]'};  mets = {'alt_loss_m','drift_ref_max_m','recovered_suite'};  tt = {'Maximum altitude loss','Maximum horizontal distance to the reference','Recovery rate'};
for m = 1:3
    ax = nexttile; hold(ax, 'on'); grid(ax, 'on');
    for mot = 1:4
        g = Fw(Fw.failedMotor == mot, :);  y = nan(size(wlv));
        for q = 1:numel(wlv), v = g.(mets{m})(g.windPeak_mps == wlv(q)); v = v(isfinite(v)); if ~isempty(v), if m == 3, y(q) = mean(v); else, y(q) = median(v); end, end, end
        plot(ax, 1:numel(wlv), y, '-o', 'DisplayName', sprintf('motor %d', mot));
    end
    set(ax, 'XTick', 1:numel(wlv), 'XTickLabel', string(wlv)); xlabel(ax, 'peak wind speed [m/s]'); title(ax, tt{m});
    ylabel(ax, units3{m}); legend(ax, 'Location', 'best', 'FontSize', 7);
end
title(tl, 'Performance against wind strength (complete failure, no sensor noise; median over wind types/directions)'); save_fig(f, fullfile(fd, 'summary_vs_wind'), cfg);
% 3 recovery-rate heatmap wind x noise
f = figure('Visible', 'off', 'Position', [30 30 800 520]);
Z = nan(numel(wlv), numel(nlv));  Nn = Z;
for a = 1:numel(wlv), for b = 1:numel(nlv)
    g = F(F.windPeak_mps == wlv(a) & F.noiseMultiplier == nlv(b), :);
    if ~isempty(g), Z(a,b) = mean(g.recovered_suite == 1); Nn(a,b) = height(g); end
end, end
imagesc(Z, [0 1]); colormap(flipud(parula)); colorbar
set(gca, 'XTick', 1:numel(nlv), 'XTickLabel', string(nlv), 'YTick', 1:numel(wlv), 'YTickLabel', string(wlv), 'YDir', 'normal');
xlabel('sensor-noise multiplier [-]'); ylabel('peak wind speed [m/s]'); title('Recovery rate (cell = recovered / runs, all failed motors; blank = not tested)');
for a = 1:numel(wlv), for b = 1:numel(nlv), if ~isnan(Z(a,b)), text(b, a, sprintf('%.2f\n(n=%d)', Z(a,b), Nn(a,b)), 'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold'); end, end, end
save_fig(f, fullfile(fd, 'summary_recovery_heatmap'), cfg);
% 4 outcome stacked bars by tier
f = figure('Visible', 'off', 'Position', [30 30 1100 480]);
tiers = unique(T.tier, 'stable');  outs = unique(T.outcome, 'stable');  C = zeros(numel(tiers), numel(outs));
for a = 1:numel(tiers), for b = 1:numel(outs), C(a,b) = nnz(T.tier == tiers(a) & T.outcome == outs(b)); end, end
bar(C, 'stacked'); set(gca, 'XTick', 1:numel(tiers), 'XTickLabel', tiers); ylabel('number of runs'); legend(strrep(outs, '_', ' '), 'Location', 'eastoutside'); grid on
title('Outcome of every run, by test tier (healthy_ok / false alarm / missed / mis-isolated / recovered / safe descent / ...)');
save_fig(f, fullfile(fd, 'summary_outcomes'), cfg);
% 5 delays across all complete-failure runs, jittered scatter by group
f = figure('Visible', 'off', 'Position', [30 30 1300 450]);  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
grp = unique(F.tier, 'stable');  rs = RandStream('twister', 'Seed', 1);
spreadMet = {'detect_delay_s','ftc_delay_s','alt_loss_m'};  spreadUnit = {'[s]','[s]','[m]'};  spreadTitle = {'FDI detection delay','FTC activation delay','Altitude loss'};
for m = 1:3
    ax = nexttile; hold(ax, 'on'); grid(ax, 'on');
    for a = 1:numel(grp)
        v = F.(spreadMet{m})(F.tier == grp(a));  v = v(isfinite(v));
        plot(ax, a + 0.3*(rand(rs, size(v)) - 0.5), v, '.', 'MarkerSize', 10);
        if ~isempty(v), plot(ax, a + [-.3 .3], median(v)*[1 1], 'k-', 'LineWidth', 2); end
    end
    set(ax, 'XTick', 1:numel(grp), 'XTickLabel', grp); xlabel(ax, 'test tier'); ylabel(ax, spreadUnit{m});
    title(ax, spreadTitle{m});
end
title(tl, 'Spread of the main metrics over the complete-failure runs (black = median)'); save_fig(f, fullfile(fd, 'summary_metric_spread'), cfg);
end

function save_fig(f, base, cfg)
warning('off', 'MATLAB:print:ContentTypeImageSuggested');
exportgraphics(f, [base '.png'], 'Resolution', cfg.suite.pngDpi);
if cfg.suite.pdf, exportgraphics(f, [base '.pdf'], 'ContentType', 'vector'); end
close(f);
end

%% ----------------------------------------------------------------------------------------------------
%% RUN_ONE: one simulation + cache
%% ----------------------------------------------------------------------------------------------------
function [M, sigOut] = run_one(cfg, V0, p, scn, keepSignals)
%RUN_ONE  One simulation: set variables, simulate, extract signals, compute metrics.
%
%   [M, sig] = run_one(cfg, V0, p, scn, keepSignals)
%   Never throws: a failed or divergent simulation returns metrics with diverged = 1.
%   Results are cached on disk by a signature of (parameters, scenario, cfg criteria).

if nargin < 5, keepSignals = false; end
cacheDir = fullfile(cfg.dirResults, 'cache');
if ~exist(cacheDir,'dir'), mkdir(cacheDir); end
key = cache_key(cfg, p, scn);
cf  = fullfile(cacheDir, [key '.mat']);
sigOut = [];
if exist(cf, 'file')
    L = load(cf);
    if ~keepSignals || ~isempty(L.sigOut)
        M = L.M;  sigOut = L.sigOut;  return
    end
end

V = params_util('apply', cfg, V0, p, scn);
if isfield(cfg, 'wsOverride')                      % user overrides of any workspace variable
    for f_ = fieldnames(cfg.wsOverride)', V.(f_{1}) = cfg.wsOverride.(f_{1}); end
end
fn = fieldnames(V);
for i = 1:numel(fn), assignin('base', fn{i}, V.(fn{i})); end
if ~bdIsLoaded(cfg.model), load_system(fullfile(cfg.modelDir, cfg.model)); end

simOK = true;  msg = '';  out = [];  sig = [];
tic;
try
    set_param(cfg.model, 'StopTime', num2str(cfg.stopTime));
    out = sim(cfg.model);
catch ME
    simOK = false;  msg = ME.message;
end
wall = toc;
if simOK
    try
        sig = extract_signals(out, V);
    catch ME
        simOK = false;  msg = ['extract: ' ME.message];
    end
end
M = compute_metrics(cfg, sig, scn, V, simOK, msg);
if isfield(cfg, 'suite') && simOK && ~isempty(sig), M = suite_metrics(cfg, sig, scn, V, M); end
M.wall_s = wall;
if keepSignals && simOK
    sigOut = decimate_signals(sig, cfg.decimate);
    sigOut.scn = scn;  sigOut.faultTime = M.faultTime_true;
end
save(cf, 'M', 'sigOut', '-v7');
end

% ---------------------------------------------------------------------------
function key = cache_key(cfg, p, scn)
s = [sprintf('%.10g,', struct2array_(p)) '|' scn.id '|' sprintf('%g,', cfg.stopTime, cfg.tFault, cfg.tGrace) ...
     '|' jsonencode(scn.wind) '|' jsonencode(cfg.unc.pct) '|' jsonencode(cfg.crit) '|' jsonencode(cache_suite_(cfg)) '|' jsonencode(getfield_(cfg, 'wsOverride')) '|' env_stamp(cfg)];
key = sprintf('r_%08x_%s', fnv1a(s), regexprep(scn.id, '[^\w]', ''));
key = key(1:min(end, 120));
end

function s = cache_suite_(cfg)
% only the settings that change a single run (not the list of tests) belong to the cache key
if isfield(cfg, 'suite'), S = cfg.suite;  s = struct('rec', S.rec, 'descent', S.descent, 'detWindow', S.detWindow, 'ftcWindow', S.ftcWindow, 'system', cfg.system); else, s = 0; end
end

function v = getfield_(c, f)
if isfield(c, f), v = c.(f); else, v = 0; end
end

function s = env_stamp(cfg)
% changes whenever your model or parameter files change -> stale cache entries are not reused
persistent cached
if isempty(cached)
    files = {fullfile(cfg.dirProject, [cfg.srcModel '.slx']), fullfile(cfg.modelDir, [cfg.model '.slx']), ...
             fullfile(cfg.dirProject, 'all_parameters.m'), fullfile(cfg.dirProject, 'quadwind_noisy_v4.slx'), ...
             fullfile(cfg.dirProject, 'ftc_params.mat'), fullfile(cfg.dirProject, 'ftc_params_x.mat')};
    cached = '';
    for i = 1:numel(files)
        d = dir(files{i});
        if ~isempty(d), cached = [cached sprintf('%s:%d:%d;', d.name, d.bytes, round(d.datenum*1e4))]; end %#ok<AGROW>
    end
end
s = cached;
end

function v = struct2array_(s)
f = fieldnames(s);  v = [];
for i = 1:numel(f), v = [v, double(s.(f{i})(:))']; end
end

function h = fnv1a(str)
h = uint64(2166136261);  prime = uint64(16777619);  m32 = uint64(4294967295);
for c = uint8(str)
    h = bitand(bitxor(h, uint64(c)) * prime, m32);
end
h = double(h);
end

function sig = extract_signals(out, V)
ts = out.get('eul_log');  t = ts.Time(:);  N = numel(t);
sig.t     = t;
sig.eul   = mat(out.get('eul_log'), N);
sig.p     = mat(out.get('p_log'), N);
sig.pref  = mat(out.get('pref_log'), N);
sig.w     = mat(out.get('w_log'), N);
sig.alpha = mat(out.get('fd_alpha'), N);
fl        = mat(out.get('fd_flags'), N);          % [Flag1 Flag2 Fault_True faulty_rotor]
sig.F1 = fl(:,1);  sig.F2 = fl(:,2);  sig.FT = fl(:,3);  sig.rotor = round(fl(:,4));
sig.health = mat(out.get('health_log'), N);
sig.wnom   = mat(out.get('wnom_log'), N);
sig.wcmd   = mat(out.get('wcmd_log'), N);
sig.windF  = mat(out.get('wind_log'), N);
sig.resid  = mat(out.get('fd_residual'), N);                   % 5 residuals [phi theta x y z]
try, sig.rupper = mat(out.get('fd_rupper'), N);  sig.rlower = mat(out.get('fd_rlower'), N);
catch, sig.rupper = nan(N,5);  sig.rlower = nan(N,5); end   % older campaign copy without threshold loggers
sig.psiref = (t >= V.psi_step_time) * V.psi_step_value;
end

function A = mat(ts, N)
d = ts.Data;
if ndims(d) == 3, A = reshape(d, [], N)'; else, A = reshape(d, N, []); end
end

function s = decimate_signals(sig, k)
f = fieldnames(sig);
for i = 1:numel(f)
    x = sig.(f{i});  s.(f{i}) = x(1:k:end, :);
end
end

%% ----------------------------------------------------------------------------------------------------
%% CAMPAIGN_CONFIG: base settings
%% ----------------------------------------------------------------------------------------------------
function cfg = campaign_config(level)
%CAMPAIGN_CONFIG  Every setting of the robustness / tuning campaign in one place.
%
%   cfg = campaign_config('smoke')     % a few minutes, checks the plumbing only
%   cfg = campaign_config('standard')  % default, a few hours on one core
%   cfg = campaign_config('full')      % overnight
%
% Nothing in here invents model variables: every parameter that is tuned is
% a variable the existing model already reads (see README.md, section 2).

if nargin < 1 || isempty(level), level = 'standard'; end
here = fullfile(fileparts(which('fdi_ftc_test_suite')), 'FDI_FTC_suite');   % outputs, model copy and cache live here
if ~exist(here, 'dir'), mkdir(here); end

cfg.level       = level;
cfg.dirCampaign = here;
cfg.dirProject  = fileparts(here);              % folder holding quad_params.m, fd_params.m, ...
cfg.dirResults  = fullfile(here, 'results', level);
cfg.variant     = 'noisy';                      % 'noisy' = quadwind_noisy.slx (sensor/process noise, mismatch gains)
cfg.srcModel    = 'quadwind_noisy';             % your model with noise + wind
cfg.model       = 'quadCampaign';               % derived copy built by build_campaign_model.m
cfg.modelDir    = fullfile(here, 'model');
geom_ = getappdata(0, 'quad_geometry');  if isempty(geom_), geom_ = 'plus'; end
cfg.geometry    = geom_;                         % 'plus' = axes along the arms, 'x' = axes 45 deg off the arms
if strcmp(geom_, 'x'), cfg.srcModel = 'quadwind_noisy_v3';  cfg.model = 'quadCampaignX'; end

%% Timing (s) -------------------------------------------------------------
cfg.stopTime = 26;      % simulation length
cfg.tFault   = 14;      % known fault onset (after all reference steps have settled)
cfg.tGrace   = 1.0;     % Judgment ignores alarms before this (hard-coded in your chart)
cfg.winPre   = 3;       % pre-fault tracking window  [tFault-winPre, tFault)
cfg.winFinal = 3;       % final window used for the recovery verdict [stopTime-winFinal, stopTime]
cfg.ftcTol   = 1e-6;    % rad/s, |w_cmd - w_nom| above this means the FTC is driving the rotors

%% Pass / fail criteria (all editable) ------------------------------------
crit.zCrash      = 0.2;          % m   altitude below this after zCrashAfter = crash (no ground in the model)
crit.zCrashAfter = 6;            % s   (take-off to z_ref = 5 m is finished by then)
crit.flyaway     = 20;           % m   distance to reference above this = lost
crit.divergeP    = 1e3;          % m   any |position| above this = numerical divergence
crit.divergeOm   = 500;          % rad/s any |body rate| above this = numerical divergence
crit.recAltErr   = 1.0;          % m   final-window max |z - z_ref| for "recovered"
crit.recHorizErr = 3.0;          % m   final-window max horizontal error for "recovered"
crit.recTilt     = deg2rad(60);  % rad final-window max |roll|,|pitch| for "recovered"
crit.healthyTilt = deg2rad(45);  % rad healthy runs: tilt above this = not "stable"
crit.healthyPos  = 2;            % m   healthy runs: settled (pre-fault window) position/altitude error above this = not "stable"
cfg.crit = crit;

%% Tunable parameters -----------------------------------------------------
% Each row: name, lower, upper, nominal (= your current value), scale, group, description
% 'log' parameters are searched on a log axis, 'lin' on a linear axis.
rows = {
 'ftc_kp_pos',     0.5, 2.0, 1.0, 'log', 'FTC', 'multiplier on K_all(:,1:3,:)   position-error gain (Kp-like)'
 'ftc_kd_pos',     0.5, 2.0, 1.0, 'log', 'FTC', 'multiplier on K_all(:,4:6,:)   velocity gain (Kd-like)'
 'ftc_kp_att',     0.5, 2.0, 1.0, 'log', 'FTC', 'multiplier on K_all(:,7:8,:)   roll/pitch angle gain (Kp-like)'
 'ftc_kd_att',     0.5, 2.0, 1.0, 'log', 'FTC', 'multiplier on K_all(:,9:11,:)  body-rate gain (Kd-like)'
 'fd_res_margin',  0.5, 4.0, 1.0, 'log', 'FD',  'multiplier on Ku3/Kl3  (constant part of residual bound)'
 'fd_thresh_flag1',0.3, 0.9, 0.5, 'lin', 'FD',  'thresh_flag1'
 'fd_thresh_flag2',0.3, 0.95,0.7, 'lin', 'FD',  'thresh_flag2'
 'fd_T_flag1',     0.005,0.05,0.01,'log','FD',  'T_flag1  [s]'
 'fd_T_flag2',     0.05, 0.4, 0.1, 'log', 'FD',  'T_flag2  [s]'
 'fd_rho_alpha',   0.2,  2.0, 0.5, 'log', 'FD',  'rho_alpha (adaptation rate of alpha_hat)'
 'fd_L_scale',     0.5,  2.0, 1.0, 'log', 'FD',  'multiplier on L1_* and L2_* observer gains'
 'fd_t_sustain',   0.3,  2.0, 1.0, 'lin', 'FD',  'Judgment: unresolved Flag1 suspicion before confirming [s]'
};
P = cell2struct(rows', {'name','lo','hi','nom','scale','group','desc'});
cfg.params = P(:)';
cfg.fixed.fd_t_confirm = 0.5;     % Judgment clearing time [s]; not tuned (affects clearing only)
cfg.tuneMask = true(1, numel(cfg.params));   % set entries false to freeze a parameter at nominal
% quadwind_noisy has the older Judgment (no suspicion state, no start-up grace): fd_t_sustain does not exist
cfg.tuneMask(strcmp({cfg.params.name}, 'fd_t_sustain')) = false;

%% Uncertainty (plant only, controller/FD keep nominal values) ------------
cfg.unc.pct = struct('m',0.10, 'J',0.15, 'kf',0.10, 'kt',0.15, 'tau',0.20, 'l',0.03);

%% Wind levels (peak horizontal wind speed, m/s) --------------------------
% Peak horizontal wind speed in m/s ('turbulence': ~3 sigma, i.e. rms = level/3).
% Cd_wind in wind_params.m is 0.5 N*s/m, i.e. 0.5 N of wind force per m/s (the weight is 1.18 N).
% The position loop can only command a_max = 2 m/s^2 (~0.24 N of horizontal thrust), and the
% measured recoverability boundary of the nominal FTC is 0.25-0.5 m/s (force 0.12-0.25 N), so the
% levels are small numbers. If you lower Cd_wind (a real 120 g quadrotor has a linear drag of about
% 0.02-0.05 N*s/m) set cfg.CdWind below and raise the levels by the same factor.
cfg.windLevels = [0.15 0.3 0.5];
cfg.CdWind = [];            % [] = use Cd_wind from wind_params.m, else a 3x1 vector [N*s/m]

%% Cost function (tuning) -------------------------------------------------
% All terms are normalised to ~[0,1]; weights are the relative importance.
cfg.cost.tCap = 5;    % s, delay used when a fault is never detected / FTC never starts
cfg.cost.w = struct('falseAlarm',3, 'missed',2, 'notRecovered',4, 'diverged',4, ...
                    'detDelay',1, 'ftcDelay',1, 'attitude',0.5, 'tracking',1, ...
                    'effort',0.3, 'saturation',0.3);

%% Level-dependent campaign size ------------------------------------------
switch level
    case 'smoke'
        cfg.opt.nLHS = 3;  cfg.opt.nGen = 1;  cfg.opt.lambda = 3;  cfg.opt.mu = 2;
        cfg.tuneSeeds = 1;     cfg.valSeeds = 101;
        cfg.valRotors = [1 2]; cfg.valSeverity = 1.0; cfg.valMaxRuns = 8;
        cfg.nTopValidate = 1;
    case 'standard'
        cfg.opt.nLHS = 24; cfg.opt.nGen = 6;  cfg.opt.lambda = 8;  cfg.opt.mu = 3;
        cfg.tuneSeeds = [1 2]; cfg.valSeeds = [101 102];
        cfg.valRotors = 1:4;   cfg.valSeverity = [1.0 0.6]; cfg.valMaxRuns = 60;
        cfg.nTopValidate = 5;
    case 'full'
        cfg.opt.nLHS = 48; cfg.opt.nGen = 10; cfg.opt.lambda = 12; cfg.opt.mu = 4;
        cfg.tuneSeeds = [1 2 3]; cfg.valSeeds = [101 102 103 104];
        cfg.valRotors = 1:4;   cfg.valSeverity = [1.0 0.8 0.6]; cfg.valMaxRuns = inf;
        cfg.nTopValidate = 8;
    otherwise
        error('campaign_config:level', 'Unknown level "%s" (smoke | standard | full).', level);
end
cfg.opt.seed = 2024;       % RNG seed of the optimiser itself (repeatable search)
cfg.decimate = 10;         % stored signals are decimated by this factor (0.01 s)
end

%% ----------------------------------------------------------------------------------------------------
%% PARAMS_UTIL
%% ----------------------------------------------------------------------------------------------------
function varargout = params_util(action, varargin)
%PARAMS_UTIL  Parameter vector helpers for the optimiser and the run builder.
%
%   p = params_util('nominal', cfg)             struct of nominal values
%   p = params_util('decode',  cfg, x01)        unit-cube vector -> struct of values (frozen params stay nominal)
%   x = params_util('encode',  cfg, p)          struct -> unit-cube vector
%   V = params_util('apply',   cfg, V0, p, scn) workspace variables for one run
%   t = params_util('table',   cfg, plist)      table of parameter structs (one row each)

switch action
    case 'nominal', varargout{1} = nominal(varargin{1});
    case 'decode',  varargout{1} = decode(varargin{1}, varargin{2});
    case 'encode',  varargout{1} = encode(varargin{1}, varargin{2});
    case 'apply',   varargout{1} = apply(varargin{:});
    case 'table',   varargout{1} = ptable(varargin{1}, varargin{2});
    otherwise, error('params_util:action', 'unknown action %s', action);
end
end

function p = nominal(cfg)
p = struct();
for i = 1:numel(cfg.params), p.(cfg.params(i).name) = cfg.params(i).nom; end
end

function p = decode(cfg, x)
p = nominal(cfg);
for i = 1:numel(cfg.params)
    if ~cfg.tuneMask(i), continue; end
    q = cfg.params(i);  u = min(max(x(i),0),1);
    if strcmp(q.scale,'log'), v = exp(log(q.lo) + u*(log(q.hi) - log(q.lo)));
    else,                     v = q.lo + u*(q.hi - q.lo); end
    p.(q.name) = v;
end
end

function x = encode(cfg, p)
x = zeros(1, numel(cfg.params));
for i = 1:numel(cfg.params)
    q = cfg.params(i);  v = p.(q.name);
    if strcmp(q.scale,'log'), x(i) = (log(v) - log(q.lo))/(log(q.hi) - log(q.lo));
    else,                     x(i) = (v - q.lo)/(q.hi - q.lo); end
end
end

function V = apply(cfg, V, p, scn)
% ---- recovery controller: scale the LQR gain groups (K_all is 3x14x4) ------------
% state order of ftc_controller.m: [pos(3) vel(3) phi theta om(3) w_live(3)]
V.K_all(:, 1:3,  :) = V.K_all(:, 1:3,  :) * p.ftc_kp_pos;
V.K_all(:, 4:6,  :) = V.K_all(:, 4:6,  :) * p.ftc_kd_pos;
V.K_all(:, 7:8,  :) = V.K_all(:, 7:8,  :) * p.ftc_kp_att;
V.K_all(:, 9:11, :) = V.K_all(:, 9:11, :) * p.ftc_kd_att;
% ---- optional: redesigned recovery LQR (ftc_design_param.m) and hardened observers -------------
if isfield(p, 'q_pz')
    q = struct('pxy',p.q_pxy,'pz',p.q_pz,'vxy',p.q_vxy,'vz',p.q_vz,'att',p.q_att,'om',p.q_om,'r',p.q_r,'geom',cfg.geometry);
    if isfield(p, 'spin_natural') && p.spin_natural, q.spin = 'natural'; end
    Dg = ftc_design_param(V, q);
    V.K_all = Dg.K_all;  V.seq_all = Dg.seq_all;  V.ueq_all = Dg.ueq_all;
end
if isfield(p, 'obs_guard'), V.obs_guard = p.obs_guard; end
for f_ = {'ftc_latch_t','ftc_rise_win','ftc_rise_lo','Ki_pos','xi_max','pos_zone','Ki_r','Ki_p','xa_max','att_zone'}, if isfield(p, f_{1}), V.(f_{1}) = p.(f_{1}); end, end
% ---- fault detection -------------------------------------------------------------
V.Ku3 = V.Ku3 * p.fd_res_margin;   V.Kl3 = V.Kl3 * p.fd_res_margin;
V.thresh_flag1 = p.fd_thresh_flag1;  V.thresh_flag2 = p.fd_thresh_flag2;
V.T_flag1 = p.fd_T_flag1;            V.T_flag2 = p.fd_T_flag2;
V.rho_alpha = p.fd_rho_alpha;
for n = {'L1_eul','L1_om','L1_p','L1_v','L2_eul','L2_om','L2_p','L2_v'}
    V.(n{1}) = V.(n{1}) * p.fd_L_scale;
end
V.fd_t_sustain = p.fd_t_sustain;
V.fd_t_confirm = cfg.fixed.fd_t_confirm;

% ---- fault injection ---------------------------------------------------------------
V.hs_t = [1e9 1e9 1e9 1e9];  V.hs_v = [1 1 1 1];
if scn.rotor > 0
    V.hs_t(scn.rotor) = scn.tFault;
    % health multiplies rotor SPEED, thrust and torque go with speed^2:
    % severity = fraction of thrust lost -> speed factor sqrt(1 - severity); 1 = dead rotor.
    V.hs_v(scn.rotor) = sqrt(max(0, 1 - scn.severity));
end

% ---- plant-only uncertainty ----------------------------------------------------------
V.m_p = V.m; V.Jxx_p = V.Jxx; V.Jyy_p = V.Jyy; V.Jzz_p = V.Jzz;
V.kf_p = V.kf; V.kt_p = V.kt; V.l_p = V.l; V.tau_p = V.tau;
if strcmp(cfg.variant, 'noisy')
    % quadwind_noisy: sensor/process noise switches and mismatch gains (see noise_params.m)
    V.noise_scale = scn.noiseScale;  V.proc_scale = scn.procScale;
    V.noise_ctrl_on = scn.ctrlNoise; V.noise_seed = scn.seed;
    V.unc_thrust_gain = 1;  V.unc_M_gain = [1;1;1];  V.unc_omdot_gain = [1;1;1];
    V.m_true = V.m;  V.tau_true = V.tau;
    if scn.uncScale > 0
        st = rng(7000 + scn.seed, 'twister');  c = onCleanup(@() rng(st)); %#ok<NASGU>
        u = @(pct) 1 + scn.uncScale*pct*(2*rand - 1);  pc = cfg.unc.pct;
        mT = V.m*u(pc.m);  JT = [V.Jxx; V.Jyy; V.Jzz].*u(pc.J);
        kfT = V.kf*u(pc.kf); ktT = V.kt*u(pc.kt); lT = V.l*u(pc.l); tauT = V.tau*u(pc.tau);
        V.m_true = mT;  V.tau_true = tauT;
        V.unc_thrust_gain = (kfT/V.kf)*(V.m/mT);
        V.unc_M_gain = [(lT*kfT)/(V.l*V.kf); (lT*kfT)/(V.l*V.kf); ktT/V.kt];
        V.unc_omdot_gain = [V.Jxx; V.Jyy; V.Jzz]./JT;
    end
elseif scn.uncScale > 0
    st = rng(7000 + scn.seed, 'twister');  c = onCleanup(@() rng(st)); %#ok<NASGU>
    u = @(pct) 1 + scn.uncScale*pct*(2*rand - 1);
    pc = cfg.unc.pct;
    V.m_p   = V.m*u(pc.m);
    V.Jxx_p = V.Jxx*u(pc.J);  V.Jyy_p = V.Jyy*u(pc.J);  V.Jzz_p = V.Jzz*u(pc.J);
    V.kf_p  = V.kf*u(pc.kf);  V.kt_p  = V.kt*u(pc.kt);
    V.tau_p = V.tau*u(pc.tau); V.l_p  = V.l*u(pc.l);
end

% ---- wind ---------------------------------------------------------------------------------
if ~isempty(cfg.CdWind), V.Cd_wind = cfg.CdWind(:); end
W = make_wind_profile(V, scn.wind);
for f = fieldnames(W)'
    V.(f{1}) = W.(f{1});
end
end

function T = ptable(cfg, plist)
names = {cfg.params.name};
T = table();
for j = 1:numel(names)
    T.(names{j}) = arrayfun(@(s) s.(names{j}), plist(:));
end
end

%% ----------------------------------------------------------------------------------------------------
%% NOMINAL_VARS: your parameter files into a struct
%% ----------------------------------------------------------------------------------------------------
function V = nominal_vars(cfg)
%NOMINAL_VARS  Run your own parameter scripts and collect every variable they define.
% Output V is a struct (field = workspace variable name). Nothing is hard-coded here:
% the values come from quad_params.m, fd_params.m, wind_params.m and ftc_params.mat.
%
% Added on top of those:
%   hs_t, hs_v           health-step time / final value per rotor (no fault)
%   wind_ev              gust event table (empty = original Dryden model)
%   m_p, Jxx_p, ...      plant copies of the physical parameters (nominal = same value)
%   confirm_alpha, clip_w   what your model's InitFcn used to define (0.7, ftc_clip_w)
%   fd_t_confirm, fd_t_sustain   Judgment constants

oldDir = pwd; c = onCleanup(@() cd(oldDir)); cd(cfg.dirProject); %#ok<NASGU>
before = who;
wind_seed = 1;                     %#ok<NASGU>
all_parameters;                    %#ok<*NOPRT>  ONE parameter file: quad, FDI, FTC (K_all, seq_all, ueq_all, clip_w), wind, noise
confirm_alpha = 0.7;               % value of the original InitFcn: "confirm_alpha = 0.7"
fd_t_confirm  = 0.5;               % Judgment constants (see build_campaign_model.m)
fd_t_sustain  = 1.0;
ftc_latch_t = 0;  ftc_rise_win = 0;  ftc_rise_lo = 0.3;   % latch evidence check (harden_ftc.m); 0 = original
Ki_pos = 0;  xi_max = 3.3;  pos_zone = [1;1;1];            % integral action (harden_pid.m); Ki = 0 = original PD
Ki_r = 0;  Ki_p = 0;  xa_max = 0.3;  att_zone = 0.2;
obs_guard = 0;                     % 0 = original observers, 1 = hardened (build_campaign_model.m)
hs_t = [1e9 1e9 1e9 1e9];  hs_v = [1 1 1 1];
wind_ev = zeros(12, 6);
m_p = m; Jxx_p = Jxx; Jyy_p = Jyy; Jzz_p = Jzz; kf_p = kf; kt_p = kt; l_p = l; tau_p = tau; %#ok<NASGU>

if strcmp(cfg.geometry, 'x')
    % 45-degree body frame: recovery LQR redesigned for it, and the v3 settings (observer guard, latch hold, PID)
    Vt = struct('m',m,'g',g,'kf',kf,'kt',kt,'l',l,'tau',tau,'Jxx',Jxx,'Jyy',Jyy,'Jzz',Jzz,'clip_w',clip_w);
    Dx = ftc_design_param(Vt, struct('pxy',1,'pz',1,'vxy',1,'vz',1,'att',1,'om',1,'r',1,'geom','x','spin','natural'));
    K_all = Dx.K_all;  seq_all = Dx.seq_all;  ueq_all = Dx.ueq_all;
    obs_guard = 1;  ftc_latch_t = 0.01;
    Ki_pos = 0.3;  xi_max = 3.3;  pos_zone = [1;1;1];  Ki_r = 20*Jxx;  Ki_p = 20*Jyy;  xa_max = 0.3;  att_zone = 0.2;
end
after = who;
names = setdiff(after, [before; {'before';'after';'S';'oldDir';'c';'cfg'}]);
V = struct();
for i = 1:numel(names)
    V.(names{i}) = eval(names{i});
end
end

function [A, Om, Phi] = wind_from_params(seed)
%WIND_FROM_PARAMS  Run all_parameters.m (silently) with a given wind seed and return the Dryden sinusoids.
wind_seed = seed;  %#ok<NASGU>
[~] = evalc('all_parameters;');
A = wind_A;  Om = wind_Om;  Phi = wind_Phi;
end

%% ----------------------------------------------------------------------------------------------------
%% MAKE_WIND_PROFILE
%% ----------------------------------------------------------------------------------------------------
function W = make_wind_profile(V, spec)
%MAKE_WIND_PROFILE  Wind variables for one run (all derived from your wind_params.m).
%
%   spec fields
%     profile  'calm' | 'steady' | 'gust' | 'dirchange' | 'turbulence' | 'mixed'
%     W        peak horizontal wind speed [m/s]   ('turbulence': ~3 sigma, rms = W/3)
%     dirDeg   azimuth of the wind vector, 0 = +x, 90 = +y (world frame, z up)
%     onset    first onset time [s]
%     dur      gust duration [s]  (steady / dirchange: total duration, inf = to end of run)
%     interval gust start-to-start spacing [s]
%     nGust    number of gusts
%     dirSeq   'dirchange': direction increments [deg] applied at each dwell, e.g. [0 90 90]
%     dwell    'dirchange': seconds spent in each direction
%     seed     seed of the Dryden sinusoids (wind_params.m)
%     vertFrac fraction of W blown vertically (default 0)
%
%   W is a struct of the workspace variables the Wind_Model needs.

d = struct('profile','calm','W',0,'dirDeg',0,'onset',6,'dur',2,'interval',4,'nGust',3, ...
           'dirSeq',[0 90 90],'dwell',4,'seed',1,'vertFrac',0,'tEnd',26);
f = fieldnames(spec);
for i = 1:numel(f), d.(f{i}) = spec.(f{i}); end
spec = d;

% Dryden sinusoids of YOUR wind_params.m for this seed (run in this function's workspace)
rngState = rng;  cleanupRng = onCleanup(@() rng(rngState)); %#ok<NASGU>
[wind_A, wind_Om, wind_Phi] = wind_from_params(spec.seed);   % Dryden sinusoids of all_parameters.m for this seed
Wd.wind_Om = wind_Om;  Wd.wind_Phi = wind_Phi;

dirv = [cosd(spec.dirDeg); sind(spec.dirDeg); 0];
dirv = dirv*sqrt(1 - spec.vertFrac^2) + [0; 0; spec.vertFrac];
ev = zeros(12, 6);  nEv = 0;
addEv = @(ton, dur, vec, shape) [ton, dur, vec(:)', shape];

Wd.wind_on      = 1;
Wd.wind_v0      = [0; 0; 0];
Wd.wind_A       = zeros(size(wind_A));
Wd.wind_t_start = spec.onset;
Wd.wind_t_end   = spec.onset;                   % window closed unless turbulence is used
tEnd = spec.tEnd;                                % scenarios set this to cfg.stopTime

switch spec.profile
    case 'calm'
        Wd.wind_on = 0;

    case 'steady'
        nEv = nEv + 1;
        ev(nEv,:) = addEv(spec.onset, min(spec.dur, tEnd - spec.onset), spec.W*dirv, 1);

    case 'gust'
        for k = 1:spec.nGust
            nEv = nEv + 1;
            ev(nEv,:) = addEv(spec.onset + (k-1)*spec.interval, spec.dur, spec.W*dirv, 0);
        end

    case 'dirchange'
        dirs = spec.dirDeg + cumsum(spec.dirSeq);
        for k = 1:numel(dirs)
            t0 = spec.onset + (k-1)*spec.dwell;
            t1 = t0 + spec.dwell + 0.3;         % 0.3 s overlap = cross-fade with the next segment
            if k == numel(dirs), t1 = min(t1 + 0.3, tEnd); end
            v = spec.W*([cosd(dirs(k)); sind(dirs(k)); 0]*sqrt(1 - spec.vertFrac^2) + [0;0;spec.vertFrac]);
            nEv = nEv + 1;
            ev(nEv,:) = addEv(t0, t1 - t0, v, 1);
        end

    case 'turbulence'
        rmsH = sqrt(sum(wind_A(1:2,:).^2, 2)/2);
        gain = (spec.W/3) / mean(rmsH);          % horizontal rms = W/3, so W is roughly the 3-sigma peak
        Wd.wind_A       = wind_A*gain;
        Wd.wind_t_start = spec.onset;
        Wd.wind_t_end   = tEnd;

    case 'mixed'                                 % half-strength steady + turbulence + gusts
        rmsH = sqrt(sum(wind_A(1:2,:).^2, 2)/2);
        Wd.wind_A       = wind_A*((0.5*spec.W/3)/mean(rmsH));
        Wd.wind_t_start = spec.onset;
        Wd.wind_t_end   = tEnd;
        Wd.wind_v0      = 0.5*spec.W*dirv;
        for k = 1:2
            nEv = nEv + 1;
            ev(nEv,:) = addEv(spec.onset + 2 + (k-1)*spec.interval, spec.dur, 0.5*spec.W*dirv, 0);
        end

    otherwise
        error('make_wind_profile:profile', 'Unknown wind profile "%s".', spec.profile);
end
assert(nEv <= 12, 'wind_ev has room for 12 events');
Wd.wind_ev = ev;
W = Wd;
end

%% ----------------------------------------------------------------------------------------------------
%% FTC_DESIGN_PARAM: your ftc_design.m maths with weights/geometry/spin as inputs
%% ----------------------------------------------------------------------------------------------------
function D = ftc_design_param(V, q)
%FTC_DESIGN_PARAM  ftc_design.m with the LQR weights as inputs (same maths, no file output).
%   D = ftc_design_param(V, q) returns D.K_all (3x14x4), D.seq_all, D.ueq_all, D.clip_w.
%   V : workspace struct (m g kf kt l tau Jxx Jyy Jzz clip_w) ; q : multipliers of the default weights
%   q.pxy q.pz q.vxy q.vz q.att q.om q.r   (all 1 = your original ftc_design.m)
%   Results are cached per (V, q).
persistent cache
if isempty(cache), cache = containers.Map('KeyType','char','ValueType','any'); end
geom = 'plus';  if isfield(q, 'geom'), geom = q.geom; end   % 'plus' = body axes along the arms, 'x' = axes 45 deg off the arms
spinMode = 'pos';  if isfield(q, 'spin'), spinMode = q.spin; end   % 'pos': always spin +z (original); 'natural': spin the way the lost yaw torque pushes
key = sprintf('%.8g,', [V.m V.kf V.kt V.l V.tau V.Jxx V.Jyy V.Jzz q.pxy q.pz q.vxy q.vz q.att q.om q.r strcmp(geom,'x') strcmp(spinMode,'natural')]);
if isKey(cache, key), D = cache(key); return; end

P = struct('m',V.m,'g',V.g,'kf',V.kf,'kt',V.kt,'l',V.l,'tau',V.tau,'J',[V.Jxx;V.Jyy;V.Jzz],'geom',geom);
delta = 0.5;
tilt  = [0 delta; delta 0; 0 -delta; -delta 0];
if strcmp(geom, 'x')
    % rotor positions in the rotated body frame (unit vectors); thrust axis tilts by delta TOWARD the dead rotor
    pos = [1 -1; -1 -1; -1 1; 1 1]/sqrt(2);
    for kk = 1:4
        ph = -asin(sin(delta)*pos(kk,2));
        tilt(kk,:) = [ph, asin(sin(delta)*pos(kk,1)/cos(ph))];
    end
end
Qd = diag([1*q.pxy 1*q.pxy 1*q.pz, 2*q.vxy 2*q.vxy 2*q.vz, 20*q.att 20*q.att, 0.02*q.om 0.02*q.om 0.02*q.om, 1e-8 1e-8 1e-8]);
Rd = 1e-6*q.r*eye(3);
K_all = zeros(3,14,4); seq_all = zeros(14,4); ueq_all = zeros(3,4);
for k = 1:4
    live = setdiff(1:4,k);  phi = tilt(k,1); th = tilt(k,2);
    nb = [-sin(th); cos(th)*sin(phi); cos(th)*cos(phi)];
    sgn = 1;  if strcmp(spinMode, 'natural') && any(k == [1 3]), sgn = -1; end   % dead rotor 1 or 3 removes +Mz torque -> body yaws -z
    x  = [0.003;0;0;0.003; 44*sgn*nb; 2.4;2.4;2.4];
    for it = 1:80
        r = eqres(P,x,phi,th,live);  Jc = zeros(10,10);
        for i = 1:10, e = zeros(10,1); e(i) = 1e-7; Jc(:,i) = (eqres(P,x+e,phi,th,live)-eqres(P,x-e,phi,th,live))/2e-7; end
        dx = -(Jc + 1e-9*eye(10))\r;  a = 1;
        while norm(eqres(P,x+a*dx,phi,th,live)) > norm(r) && a > 1e-4, a = a/2; end
        x = x + a*dx;  if norm(eqres(P,x,phi,th,live)) < 1e-10, break; end
    end
    assert(norm(eqres(P,x,phi,th,live)) < 1e-6, 'equilibrium did not converge for rotor %d', k);
    seq  = [x(1);x(2);0; x(3);x(4);0; phi;th; x(5:7); 1000*x(8:10)];  ueq = 1000*x(8:10);
    A = zeros(14); B = zeros(14,3);
    for i = 1:14
        h = 1e-6*max(1,abs(seq(i)))*(1000*(i>=12)+(i<12)); e = zeros(14,1); e(i) = h;
        A(:,i) = (fd(P,seq+e,ueq,live)-fd(P,seq-e,ueq,live))/(2*h);
    end
    for j = 1:3, e = zeros(3,1); e(j) = 1e-3; B(:,j) = (fd(P,seq,ueq+e,live)-fd(P,seq,ueq-e,live))/2e-3; end
    K_all(:,:,k) = lqr(A,B,Qd,Rd);  seq_all(:,k) = seq;  ueq_all(:,k) = ueq;
end
D = struct('K_all',K_all,'seq_all',seq_all,'ueq_all',ueq_all,'clip_w',V.clip_w);
cache(key) = D;
end

function r = eqres(P,x,phi,th,live)
s = [x(1);x(2);0; x(3);x(4);0; phi;th; x(5:7); 1000*x(8:10)];
d = fd(P,s,1000*x(8:10),live);  r = [d(1:2); d(4:6); d(7:8); d(9:11)];
end

function d = fd(P,s,wc,live)
m=P.m; g=P.g; kf=P.kf; kt=P.kt; l=P.l; tau=P.tau; J=P.J;
p=s(1:3); v=s(4:6); phi=s(7); th=s(8); om=s(9:11); wl=s(12:14);
w=zeros(4,1); w(live)=wl; w2=w.^2; F=kf*sum(w2);
if strcmp(P.geom, 'x')
    M=[l/sqrt(2)*kf*(-w2(1)-w2(2)+w2(3)+w2(4)); l/sqrt(2)*kf*(-w2(1)+w2(2)+w2(3)-w2(4)); kt*(w2(1)-w2(2)+w2(3)-w2(4))];
else
    M=[l*kf*(w2(4)-w2(2)); l*kf*(w2(3)-w2(1)); kt*(w2(1)-w2(2)+w2(3)-w2(4))];
end
od=(M-cross(om,J.*om))./J;
psid=(sin(phi)*om(2)+cos(phi)*om(3))/cos(th);
phd=om(1)+sin(phi)*tan(th)*om(2)+cos(phi)*tan(th)*om(3);  thd=cos(phi)*om(2)-sin(phi)*om(3);
n=[sin(th)*cos(phi); -sin(phi); cos(th)*cos(phi)]; zc=[0;0;1];
d=[v-psid*cross(zc,p); F/m*n-[0;0;g]-psid*cross(zc,v); phd; thd; od; (wc-wl)/tau];
end

%% ----------------------------------------------------------------------------------------------------
%% BUILD_CAMPAIGN_MODEL: the Simulink test copy
%% ----------------------------------------------------------------------------------------------------
function build_campaign_model(cfg)
%BUILD_CAMPAIGN_MODEL  Make a campaign copy of your model without touching the original.
%
% Source : cfg.srcModel  (quadWithWind_dryden.slx, left unchanged)
% Result : campaign/model/quadCampaign.slx
%
% Changes (none alters the control architecture or the default numbers):
%   1. health_1..4 step blocks read their time/final value from the workspace
%      vectors hs_t(k), hs_v(k)  -> fault time, failed rotor and severity are run inputs.
%   2. The plant blocks (Translational_Dynamics, Rotational_Dynamics, Rotor_Forces,
%      the 1/tau lag gain and the 1/m wind gain) read m_p, Jxx_p, Jyy_p, Jzz_p, kf_p,
%      kt_p, l_p, tau_p instead of m, Jxx, ... so model uncertainty can be applied to the
%      plant only while controllers and observers keep the nominal values.
%   3. wind_fcn gets one extra parameter, wind_ev (gust-event table). With wind_ev = zeros
%      the output is identical to your original Dryden sinusoid model.
%   4. Judgment: t_confirm and t_sustain (hard-coded in the chart) become the parameters
%      fd_t_confirm and fd_t_sustain; defaults 0.5 / 1.0 equal the old constants.
%   5. Extra To Workspace loggers: health_log, pref_log, wnom_log, wcmd_log.
%   6. PreLoadFcn/InitFcn cleared (the campaign supplies every variable itself) and the
%      scopes are commented out (speed).

src = cfg.srcModel;  dst = cfg.model;
if ~exist(cfg.modelDir,'dir'), mkdir(cfg.modelDir); end
addpath(cfg.dirProject);                         % so the source model is found
if bdIsLoaded(dst), close_system(dst, 0); end
if bdIsLoaded(src), close_system(src, 0); end
load_system(src);
dstFile = fullfile(cfg.modelDir, [dst '.slx']);
save_system(src, dstFile);                       % copy; the model in memory is now `dst`
mdl = dst;
fprintf('build_campaign_model: building %s from %s\n', dstFile, src);

%% 6. configuration
set_param(mdl, 'PreLoadFcn', '', 'InitFcn', '');
set_param(mdl, 'StopTime', num2str(cfg.stopTime));
for s = find_system(mdl, 'BlockType', 'Scope')'
    set_param(s{1}, 'Commented', 'on');
end

%% 1. health steps -> workspace vectors
for k = 1:4
    b = sprintf('%s/health_%d', mdl, k);
    set_param(b, 'Time', sprintf('hs_t(%d)', k), 'InitialValue', '1', ...
                 'FinalValue', sprintf('hs_v(%d)', k));
end

%% 2. plant-only parameters (quadwind_noisy already has mismatch gains: m_true, tau_true, unc_*)
if strcmp(cfg.variant, 'wind')
rename_params([mdl '/Translational_Dynamics'], {'m'});
rename_params([mdl '/Rotational_Dynamics'],    {'Jxx','Jyy','Jzz'});
rename_params([mdl '/Rotor_Forces'],           {'kf','kt','l'});
assert(strcmp(get_param([mdl '/Lag_gain'],'Gain'), '1/tau'), 'Lag_gain is not 1/tau any more');
set_param([mdl '/Lag_gain'], 'Gain', '1/tau_p');
assert(strcmp(get_param([mdl '/Wind_to_accel'],'Gain'), '1/m'), 'Wind_to_accel is not 1/m any more');
set_param([mdl '/Wind_to_accel'], 'Gain', '1/m_p');
end

%% 3. wind function with gust events
wf = [mdl '/Wind_Model/wind_fcn'];
ch = sfroot().find('-isa','Stateflow.EMChart','Path',wf);
assert(~isempty(ch), 'wind_fcn not found');
ch.Script = sprintf([ ...
 'function [F_w, v_w] = wind_fcn(t, wind_A, wind_Om, wind_Phi, wind_v0, Cd_wind, wind_on, wind_t_start, wind_t_end, wind_ev)\n' ...
 '%% Dryden sinusoids (+ steady v0) inside [t_start,t_end], plus a table of gust events.\n' ...
 '%% wind_ev rows: [t_on, duration, ax, ay, az, shape]  shape 0 = 1-cos gust, 1 = trapezoid (0.3 s ramps)\n' ...
 'v_turb = (t >= wind_t_start && t <= wind_t_end) * (wind_v0 + sum(wind_A .* sin(wind_Om*t + wind_Phi), 2));\n' ...
 'v_g = zeros(3,1);\n' ...
 'for i = 1:size(wind_ev,1)\n' ...
 '    dur = wind_ev(i,2);\n' ...
 '    if dur > 0\n' ...
 '        tau = (t - wind_ev(i,1))/dur;\n' ...
 '        if tau >= 0 && tau <= 1\n' ...
 '            if wind_ev(i,6) < 0.5\n' ...
 '                s = 0.5*(1 - cos(2*pi*tau));\n' ...
 '            else\n' ...
 '                rr = min(0.5, 0.3/dur);\n' ...
 '                s = min(1, min(tau, 1 - tau)/rr);\n' ...
 '            end\n' ...
 '            v_g = v_g + s*wind_ev(i,3:5)'';\n' ...
 '        end\n' ...
 '    end\n' ...
 'end\n' ...
 'v_w = v_turb + v_g;\n' ...
 'F_w = wind_on * (Cd_wind .* v_w);\n' ...
 'end\n']);
set_scope(ch, {'wind_ev'}, 'Parameter');

%% 4. Judgment constants -> parameters (only for the newer Judgment with a suspicion state)
if strcmp(cfg.variant, 'wind')
jb = [mdl '/Fault_Detection/Judgment'];
jc = sfroot().find('-isa','Stateflow.EMChart','Path',jb);
assert(~isempty(jc), 'Judgment not found');
scr = jc.Script;
hdr = 'function [Fault_True, faulty_rotor] = judgment(Flag1, Flag2, alpha_hat, t)';
assert(contains(scr, hdr), 'Judgment signature changed - update build_campaign_model.m');
assert(~isempty(regexp(scr, 't_confirm\s*=\s*0\.5;', 'once')) && ...
       ~isempty(regexp(scr, 't_sustain\s*=\s*1\.0;', 'once')), ...
       'Judgment constants changed - update build_campaign_model.m');
scr = strrep(scr, hdr, 'function [Fault_True, faulty_rotor] = judgment(Flag1, Flag2, alpha_hat, t, fd_t_confirm, fd_t_sustain)');
scr = regexprep(scr, 't_confirm(\s*)=\s*0\.5;', 't_confirm$1= fd_t_confirm;', 'once');
scr = regexprep(scr, 't_sustain(\s*)=\s*1\.0;', 't_sustain$1= fd_t_sustain;', 'once');
jc.Script = scr;
set_scope(jc, {'fd_t_confirm','fd_t_sustain'}, 'Parameter');
end

%% 4b. observer hardening switch (obs_guard = 0 reproduces the original charts exactly)
if strcmp(cfg.geometry, 'x')
    harden_geometry(mdl);                 % v3 source already contains the observer / latch / PID changes
else
    harden_observers(mdl);
    harden_ftc(mdl);
    harden_pid(mdl);
end

%% 5. extra loggers
add_logger(mdl, 'Health_mux',         'health_log');
add_logger(mdl, 'p_ref_mux',          'pref_log');
add_logger(mdl, 'Control_Allocation', 'wnom_log');
add_logger(mdl, 'FTC_Controller',     'wcmd_log');
add_logger_sub([mdl '/Fault_Detection'], 'Threshold_Calc', 1, 'fd_rupper');   % adaptive residual bounds (for the residual plots)
add_logger_sub([mdl '/Fault_Detection'], 'Threshold_Calc', 2, 'fd_rlower');

save_system(mdl);
close_system(mdl, 0);
fprintf('build_campaign_model: done.\n');
end

% ---------------------------------------------------------------------------
function rename_params(blk, names)
ch = sfroot().find('-isa','Stateflow.EMChart','Path',blk);
assert(~isempty(ch), 'MATLAB Function block %s not found', blk);
scr = ch.Script;
for i = 1:numel(names)
    assert(~isempty(regexp(scr, ['\<' names{i} '\>'], 'once')), '%s missing in %s', names{i}, blk);
    scr = regexprep(scr, ['\<' names{i} '\>'], [names{i} '_p']);
end
ch.Script = scr;
set_scope(ch, strcat(names, '_p'), 'Parameter');
end

function set_scope(ch, names, scope)
for i = 1:numel(names)
    d = ch.find('-isa','Stateflow.Data','Name',names{i});
    assert(~isempty(d), 'data %s not created', names{i});
    d.Scope = scope;
end
end

function add_logger(mdl, srcBlk, var)
pos = get_param([mdl '/' srcBlk], 'Position');
nb  = [mdl '/' var];
add_block('simulink/Sinks/To Workspace', nb, 'VariableName', var, 'SaveFormat', 'Timeseries', ...
          'SampleTime', '-1', 'Position', [pos(3)+80 pos(4)+60 pos(3)+140 pos(4)+90]);
add_line(mdl, [srcBlk '/1'], [var '/1'], 'autorouting', 'on');
end

function add_logger_sub(parent, srcBlk, srcPort, var)
pos = get_param([parent '/' srcBlk], 'Position');
add_block('simulink/Sinks/To Workspace', [parent '/' var], 'VariableName', var, 'SaveFormat', 'Timeseries', ...
          'SampleTime', '-1', 'Position', [pos(3)+80 pos(4)+40 pos(3)+140 pos(4)+70]);
add_line(parent, sprintf('%s/%d', srcBlk, srcPort), [var '/1'], 'autorouting', 'on');
end

%% ----------------------------------------------------------------------------------------------------
%% HARDEN_OBSERVERS
%% ----------------------------------------------------------------------------------------------------
function harden_observers(mdl)
%HARDEN_OBSERVERS  Numerical guards for Obs1_Deriv, Obs2_Deriv and Residual_Calc, switched by obs_guard.
%   * angle innovations are wrapped to [-pi, pi]  (yaw spins, roll/pitch pass +-180 deg during recovery)
%   * tan(theta) is evaluated as sin(theta)/cth with the same floor the chart already uses for cth
%   * body-rate estimate is clamped to +-300 rad/s before it enters the cross-product and Euler-rate terms
%   * alpha_hat is clamped to [0,1] before it is used in the predictions (obobserver 2)
%   * non-finite derivatives are replaced by 0 and the result is limited to +-1e6
% With obs_guard = 0 every line behaves exactly as before.
for nm = {'Obs1_Deriv', 'Obs2_Deriv'}
    blk = [mdl '/Fault_Detection/' nm{1}];
    ch = sfroot().find('-isa','Stateflow.EMChart','Path',blk);
    assert(~isempty(ch), '%s not found', blk);
    scr = ch.Script;
    isO2 = strcmp(nm{1}, 'Obs2_Deriv');
    % signature
    scr = regexprep(scr, '(function xhat_dot = obs\d_deriv\([^)]*)\)', '$1, obs_guard)', 'once');
    % guard block right after the state unpack
    if isO2, anchor = 'a_hat = xhat(13:16);'; else, anchor = 'eul_hat = xhat(1:3); om_hat = xhat(4:6); p_hat = xhat(7:9); v_hat = xhat(10:12);'; end
    assert(contains(scr, anchor), 'anchor missing in %s', nm{1});
    g = [anchor newline 'if obs_guard > 0.5' newline '    om_hat = max(min(om_hat, 300), -300);' newline];
    if isO2, g = [g '    a_hat = max(min(a_hat, 1), 0);' newline]; end
    g = [g 'end'];
    scr = strrep(scr, anchor, g);
    % tan(theta) via sin/cth, wrapped angle innovation
    assert(contains(scr, 'tan(theta)'), 'tan(theta) missing in %s', nm{1});
    scr = strrep(scr, 'tan(theta)', 'tth');
    scr = strrep(scr, 'cth = sign(cth + (cth==0))*max(abs(cth), 1e-3);', ...
        ['cth = sign(cth + (cth==0))*max(abs(cth), 1e-3);' newline 'tth = tan(theta);  if obs_guard > 0.5, tth = sin(theta)/cth; end']);
    pat = 'L(\d)_eul\.\*\(eul_meas - eul_hat\)';
    assert(~isempty(regexp(scr, pat, 'once')), 'eul innovation missing in %s', nm{1});
    scr = regexprep(scr, pat, 'L$1_eul.*dE');
    scr = strrep(scr, 'Jv = [Jxx; Jyy; Jzz];', ['Jv = [Jxx; Jyy; Jzz];' newline 'dE = eul_meas - eul_hat;  if obs_guard > 0.5, dE = atan2(sin(dE), cos(dE)); end']);
    % non-finite guard at the end
    tailGuard = [newline 'if obs_guard > 0.5' newline '    xhat_dot(~isfinite(xhat_dot)) = 0;' newline '    xhat_dot = max(min(xhat_dot, 1e6), -1e6);' newline 'end' newline 'end' newline];
    scr = strtrim(scr);
    assert(endsWith(scr, 'end'), 'unexpected end of %s', nm{1});
    scr = [scr(1:end-3) tailGuard(2:end)];
    ch.Script = scr;
    set_scope(ch, {'obs_guard'}, 'Parameter');
end
% residual: wrap the roll / pitch residual
blk = [mdl '/Fault_Detection/Residual_Calc'];
ch = sfroot().find('-isa','Stateflow.EMChart','Path',blk);
scr = ch.Script;
scr = regexprep(scr, '(function r = residual_calc\([^)]*)\)', '$1, obs_guard)', 'once');
assert(contains(scr, 'p_meas      - p_hat];'), 'residual_calc changed');
scr = strrep(scr, 'p_meas      - p_hat];', ['p_meas      - p_hat];' newline ...
    'if obs_guard > 0.5, r(1:2) = atan2(sin(r(1:2)), cos(r(1:2))); end']);
ch.Script = scr;
set_scope(ch, {'obs_guard'}, 'Parameter');
end

%% ----------------------------------------------------------------------------------------------------
%% HARDEN_FTC
%% ----------------------------------------------------------------------------------------------------
function harden_ftc(mdl)
%HARDEN_FTC  Optional evidence check before the FTC latches onto a "failed" rotor.
%
% The original latch fires on the first sample where Fault_True = 1 and alpha_hat(faulty_rotor) > confirm_alpha.
% Added (all switched off by default, so the original behaviour is unchanged):
%   ftc_latch_t   [s]  the condition must hold continuously for this long (same rotor) before latching
%   ftc_rise_win  [s]  the estimate must have been below ftc_rise_lo at some point in the last ftc_rise_win
%                      seconds, i.e. a real failure makes alpha_hat jump from ~0 to ~1 within milliseconds,
%                      while noise pushes it up slowly and a rotor that has been "failed" for a long time
%                      is not a new failure.  0 = off.
%   ftc_rise_lo        threshold for the "was low" test (default 0.3)
blk = [mdl '/FTC_Controller'];
ch = sfroot().find('-isa','Stateflow.EMChart','Path',blk);
assert(~isempty(ch), 'FTC_Controller not found');
scr = ch.Script;
oldHdr = 'K_all, seq_all, ueq_all, clip_w, confirm_alpha)';
assert(contains(scr, oldHdr), 'FTC signature changed');
scr = strrep(scr, oldHdr, 'K_all, seq_all, ueq_all, clip_w, confirm_alpha, ftc_latch_t, ftc_rise_win, ftc_rise_lo, sim_dt)');
oldLatch = [ ...
 'persistent k_lat' newline ...
 'if isempty(k_lat), k_lat = 0; end' newline ...
 'if k_lat == 0 && Fault_True > 0.5 && faulty_rotor >= 1 && alpha_hat(max(1,round(faulty_rotor))) > confirm_alpha' newline ...
 '    k_lat = round(faulty_rotor);              % latch: a dead rotor does not come back' newline ...
 'end'];
assert(contains(scr, oldLatch), 'FTC latch block changed - update harden_ftc.m');
newLatch = [ ...
 'persistent k_lat n_ok k_cand abuf' newline ...
 'if isempty(k_lat), k_lat = 0; n_ok = 0; k_cand = 0; abuf = zeros(4,256); end' newline ...
 'abuf = [abuf(:,2:end), alpha_hat(:)];            % last 256 samples of the thrust-loss estimates' newline ...
 'if k_lat == 0' newline ...
 '    kk = max(1, round(faulty_rotor));' newline ...
 '    ok = Fault_True > 0.5 && faulty_rotor >= 1 && alpha_hat(kk) > confirm_alpha;' newline ...
 '    if ok && ftc_rise_win > 0          % the estimate must have risen from a low value recently' newline ...
 '        nw = min(256, max(1, round(ftc_rise_win/sim_dt)));' newline ...
 '        ok = min(abuf(kk, end-nw+1:end)) < ftc_rise_lo;' newline ...
 '    end' newline ...
 '    if ok && kk == k_cand, n_ok = n_ok + 1; elseif ok, n_ok = 1; k_cand = kk; else, n_ok = 0; k_cand = 0; end' newline ...
 '    if ok && n_ok*sim_dt >= ftc_latch_t - 1e-12' newline ...
 '        k_lat = kk;                    % latch: a dead rotor does not come back' newline ...
 '    end' newline ...
 'end'];
scr = strrep(scr, oldLatch, newLatch);
ch.Script = scr;
for nm = {'ftc_latch_t','ftc_rise_win','ftc_rise_lo','sim_dt'}
    d = ch.find('-isa','Stateflow.Data','Name',nm{1});
    assert(~isempty(d), 'data %s not created', nm{1});
    d.Scope = 'Parameter';
end
end

%% ----------------------------------------------------------------------------------------------------
%% HARDEN_PID
%% ----------------------------------------------------------------------------------------------------
function harden_pid(mdl)
%HARDEN_PID  Add integral action (PID) to the nominal Position_Controller and Attitude_Controller.
%
% Integral states are Simulink Integrator blocks (pos_int 3x1, att_int 3x1) outside the charts, because a
% persistent variable inside a MATLAB Function block would be updated at every ode4 minor step.
% The charts get one extra input (the integral state) and one extra output (its derivative, with anti-windup):
%   * integrate only while the error is small (|e| < zone): no wind-up during take-off / reference steps
%   * stop integrating while the command is saturated and the error pushes further into saturation
%   * stop integrating at the clamp (|xi| >= xi_max) when the error pushes outward
% Gains (workspace variables, all 0 = exactly the original PD controller):
%   Ki_pos [1/s^3] position integral gain (acceleration domain), xi_max [m s], pos_zone [m] (3x1: x, y, z)
%   Ki_r, Ki_p [N m/(rad s)] roll / pitch integral gains,         xa_max [rad s], att_zone [rad]
% Yaw has no integral term. After an FTC takeover the nominal controller output is discarded, so the
% integrators need no special handling.

%% position controller -------------------------------------------------------
blk = [mdl '/Position_Controller'];
ch = sfroot().find('-isa','Stateflow.EMChart','Path',blk);  assert(~isempty(ch), 'Position_Controller not found');
scr = ch.Script;
hdr = 'function [fc_des, phi_cmd, theta_cmd] = position_controller(p_ref, p, v, eul, Kp_pos, Kd_pos, a_max, att_lim, g, m)';
assert(contains(scr, hdr), 'position_controller signature changed');
scr = strrep(scr, hdr, 'function [fc_des, phi_cmd, theta_cmd, dxi] = position_controller(p_ref, p, v, eul, Kp_pos, Kd_pos, a_max, att_lim, g, m, xi, Ki_pos, xi_max, pos_zone)');
oldA = ['a = Kp_pos*(pr - pp) - Kd_pos*vv;' newline 'a = max(min(a, a_max), -a_max);'];
assert(contains(scr, oldA), 'position_controller body changed');
newA = [ ...
 'e_p = pr - pp;' newline ...
 'a_raw = Kp_pos*e_p - Kd_pos*vv + Ki_pos*xi(:);          % PD + integral' newline ...
 'a = max(min(a_raw, a_max), -a_max);' newline ...
 'dxi = e_p;                                              % integral of the position error' newline ...
 'for i = 1:3' newline ...
 '    if abs(e_p(i)) > pos_zone(i) || (abs(a_raw(i)) > a_max && a_raw(i)*e_p(i) > 0) || (abs(xi(i)) >= xi_max && xi(i)*e_p(i) > 0)' newline ...
 '        dxi(i) = 0;                                     % anti-windup' newline ...
 '    end' newline ...
 'end'];
scr = strrep(scr, oldA, newA);
ch.Script = scr;
for nm = {'Ki_pos','xi_max','pos_zone'}, setScope(ch, nm{1}, 'Parameter'); end
wireIntegrator(mdl, 'Position_Controller', ch, 'pos_int', 'zeros(3,1)', 'dxi', 'xi');

%% attitude controller -------------------------------------------------------
blk = [mdl '/Attitude_Controller'];
ch = sfroot().find('-isa','Stateflow.EMChart','Path',blk);  assert(~isempty(ch), 'Attitude_Controller not found');
scr = ch.Script;
hdr = 'function M = attitude_controller(phi_cmd, theta_cmd, psi_ref, eul, om, Kp_r, Kd_r, Kp_p, Kd_p, Kp_y, Kd_y)';
assert(contains(scr, hdr), 'attitude_controller signature changed');
scr = strrep(scr, hdr, 'function [M, dxa] = attitude_controller(phi_cmd, theta_cmd, psi_ref, eul, om, Kp_r, Kd_r, Kp_p, Kd_p, Kp_y, Kd_y, xa, Ki_r, Ki_p, xa_max, att_zone)');
assert(contains(scr, 'Mx = Kp_r*(phi_cmd   - e(1)) - Kd_r*o(1);') && contains(scr, 'My = Kp_p*(theta_cmd - e(2)) - Kd_p*o(2);'), 'attitude_controller body changed');
scr = strrep(scr, 'Mx = Kp_r*(phi_cmd   - e(1)) - Kd_r*o(1);', 'Mx = Kp_r*(phi_cmd   - e(1)) - Kd_r*o(1) + Ki_r*xa(1);');
scr = strrep(scr, 'My = Kp_p*(theta_cmd - e(2)) - Kd_p*o(2);', ['My = Kp_p*(theta_cmd - e(2)) - Kd_p*o(2) + Ki_p*xa(2);' newline ...
 'e_a = [phi_cmd - e(1); theta_cmd - e(2); 0];' newline ...
 'dxa = e_a;                                              % integral of the roll / pitch error (yaw: none)' newline ...
 'for i = 1:2' newline ...
 '    if abs(e_a(i)) > att_zone || (abs(xa(i)) >= xa_max && xa(i)*e_a(i) > 0)' newline ...
 '        dxa(i) = 0;                                     % anti-windup' newline ...
 '    end' newline ...
 'end']);
ch.Script = scr;
for nm = {'Ki_r','Ki_p','xa_max','att_zone'}, setScope(ch, nm{1}, 'Parameter'); end
wireIntegrator(mdl, 'Attitude_Controller', ch, 'att_int', 'zeros(3,1)', 'dxa', 'xa');
end

function setScope(ch, name, scope)
d = ch.find('-isa','Stateflow.Data','Name',name);
assert(~isempty(d), 'data %s not created', name);
d.Scope = scope;
end

function wireIntegrator(mdl, blkName, ch, intName, ic, outName, inName)
dOut = ch.find('-isa','Stateflow.Data','Name',outName);  dIn = ch.find('-isa','Stateflow.Data','Name',inName);
assert(~isempty(dOut) && ~isempty(dIn), 'ports not created for %s', blkName);
pos = get_param([mdl '/' blkName], 'Position');
add_block('simulink/Continuous/Integrator', [mdl '/' intName], 'InitialCondition', ic, ...
    'Position', [pos(1) pos(4)+60 pos(1)+30 pos(4)+90]);
add_line(mdl, sprintf('%s/%d', blkName, dOut.Port), [intName '/1'], 'autorouting', 'on');
add_line(mdl, [intName '/1'], sprintf('%s/%d', blkName, dIn.Port), 'autorouting', 'on');
end

%% ----------------------------------------------------------------------------------------------------
%% HARDEN_GEOMETRY
%% ----------------------------------------------------------------------------------------------------
function harden_geometry(mdl)
%HARDEN_GEOMETRY  Change the model from the "plus" body frame (axes along the arms) to the "X" body frame
%                 (axes rotated 45 deg from the arms, x_B = (x0 + y0)/sqrt(2), y_B = (y0 - x0)/sqrt(2)).
%
% Rotor positions in the new body frame (l = arm length):  1: (+,-)  2: (-,-)  3: (-,+)  4: (+,+)  times l/sqrt(2)
% Moments (tau = r x F, thrust along +z):
%   Mx = (l/sqrt2) kf ( -w1^2 - w2^2 + w3^2 + w4^2 )
%   My = (l/sqrt2) kf ( -w1^2 + w2^2 + w3^2 - w4^2 )
%   Mz = kt ( w1^2 - w2^2 + w3^2 - w4^2 )                (unchanged)
% Patched: Rotor_Forces (moments), Control_Allocation (inverse mixing), Obs2_Deriv (moment prediction and its
% sensitivity to alpha).  The recovery LQR must be redesigned for this layout: ftc_design_param(V, q) with q.geom = 'x'.

%% Rotor_Forces
ch = sfroot().find('-isa','Stateflow.EMChart','Path',[mdl '/Rotor_Forces']);  assert(~isempty(ch));
scr = ch.Script;
a1 = 'l*kf*(w2(4) - w2(2))';  a2 = 'l*kf*(w2(3) - w2(1))';
assert(contains(scr, a1) && contains(scr, a2), 'rotor_forces changed');
scr = strrep(scr, a1, '(l/sqrt(2))*kf*(-w2(1) - w2(2) + w2(3) + w2(4))');
scr = strrep(scr, a2, '(l/sqrt(2))*kf*(-w2(1) + w2(2) + w2(3) - w2(4))');
ch.Script = scr;

%% Control_Allocation
ch = sfroot().find('-isa','Stateflow.EMChart','Path',[mdl '/Control_Allocation']);  assert(~isempty(ch));
scr = ch.Script;
rep = { ...
 'w1sq = fc_des/(4*kf) - My/(2*l*kf) + Mz/(4*kt);', 'w1sq = fc_des/(4*kf) - (Mx + My)/(4*c) + Mz/(4*kt);'; ...
 'w2sq = fc_des/(4*kf) - Mx/(2*l*kf) - Mz/(4*kt);', 'w2sq = fc_des/(4*kf) - (Mx - My)/(4*c) - Mz/(4*kt);'; ...
 'w3sq = fc_des/(4*kf) + My/(2*l*kf) + Mz/(4*kt);', 'w3sq = fc_des/(4*kf) + (Mx + My)/(4*c) + Mz/(4*kt);'; ...
 'w4sq = fc_des/(4*kf) + Mx/(2*l*kf) - Mz/(4*kt);', 'w4sq = fc_des/(4*kf) + (Mx - My)/(4*c) - Mz/(4*kt);'};
for i = 1:size(rep,1)
    assert(contains(scr, rep{i,1}), 'control_allocation changed: %s', rep{i,1});
    scr = strrep(scr, rep{i,1}, rep{i,2});
end
scr = strrep(scr, 'Mx = Mv(1); My = Mv(2); Mz = Mv(3);', ['Mx = Mv(1); My = Mv(2); Mz = Mv(3);' newline 'c = l*kf/sqrt(2);']);
assert(contains(scr, 'c = l*kf/sqrt(2);'), 'control_allocation anchor missing');
ch.Script = scr;

%% Obs2_Deriv
ch = sfroot().find('-isa','Stateflow.EMChart','Path',[mdl '/Fault_Detection/Obs2_Deriv']);  assert(~isempty(ch));
scr = ch.Script;
r = { ...
 'Mx_pred\s*=\s*[^;]*;', 'Mx_pred = (l/sqrt(2))*((-f1 - f2 + f3 + f4) - (-a_hat(1)*f1 - a_hat(2)*f2 + a_hat(3)*f3 + a_hat(4)*f4));'; ...
 'My_pred\s*=\s*[^;]*;', 'My_pred = (l/sqrt(2))*((-f1 + f2 + f3 - f4) - (-a_hat(1)*f1 + a_hat(2)*f2 + a_hat(3)*f3 - a_hat(4)*f4));'; ...
 'S_Mx\s*=\s*\[[^\]]*\];', 'S_Mx = (l/sqrt(2))*[f1, f2, -f3, -f4];'; ...
 'S_My\s*=\s*\[[^\]]*\];', 'S_My = (l/sqrt(2))*[f1, -f2, -f3, f4];'};
for i = 1:size(r,1)
    assert(~isempty(regexp(scr, r{i,1}, 'once')), 'obs2 pattern missing: %s', r{i,1});
    scr = regexprep(scr, r{i,1}, strrep(r{i,2}, '\', '\'), 'once');
end
ch.Script = scr;
end