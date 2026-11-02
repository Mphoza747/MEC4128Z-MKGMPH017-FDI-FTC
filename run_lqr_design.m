%% run_lqr_design.m  --  calculates the spinning-hover LQR gains used by the FTC block
% For each failed motor k = 1..4 this (1) solves for the spinning-hover equilibrium (Newton iteration),
% (2) linearises the 14-state model about it, and (3) computes the LQR gain K = lqr(A,B,Qd,Rd).
% The result is the same K_all, seq_all and ueq_all that are stored in model/ftc_params_x.mat.
% Settings (as used in the report): X layout, spin direction 'natural', tilt delta = 0.5 rad,
% weights Qd = diag([1 1 1, 2 2 2, 20 20, 0.02 0.02 0.02, 1e-8 1e-8 1e-8]), Rd = 1e-6*I.
% Needs the Control System Toolbox (lqr). Run from this folder.
clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(here);
run(fullfile(here, '..', 'model', 'combined_params_all.m'));      % m, g, kf, kt, l, tau, Jxx, Jyy, Jzz, clip_w, delta

V = struct('m',m,'g',g,'kf',kf,'kt',kt,'l',l,'tau',tau,'Jxx',Jxx,'Jyy',Jyy,'Jzz',Jzz,'clip_w',clip_w);
q = struct('pxy',1,'pz',1,'vxy',1,'vz',1,'att',1,'om',1,'r',1,'geom','x','spin','natural');   % all multipliers 1
D = ftc_design_delta(V, q, delta);

%% report
for k = 1:4
    K = D.K_all(:,:,k);
    cl = eig(D.A_all(:,:,k) - D.B_all(:,:,k)*K);
    fprintf('Motor %d failed: equilibrium spin rate |omega| = %.2f rad/s, max Re(eig) = %.3f, rank(ctrb) = %d\n', ...
        k, norm(D.seq_all(9:11,k)), max(real(cl)), rank(ctrb(D.A_all(:,:,k), D.B_all(:,:,k)), 1e-9*norm(ctrb(D.A_all(:,:,k), D.B_all(:,:,k)))));
    disp(K);
end

%% compare with the gains stored in the model
S = load(fullfile(here, '..', 'model', 'ftc_params_x.mat'));
for k = 1:4
    fprintf('Motor %d: max |K - K_stored| / max|K_stored| = %.2e\n', k, max(abs(D.K_all(:,:,k) - S.K_all(:,:,k)), [], 'all') / max(abs(S.K_all(:,:,k)), [], 'all'));
end

%% save (new file, so the stored gains are not overwritten)
K_all = D.K_all;  seq_all = D.seq_all;  ueq_all = D.ueq_all;  ftc_clip_w = clip_w;  %#ok<NASGU>
save(fullfile(here, 'ftc_params_x_recomputed.mat'), 'K_all', 'seq_all', 'ueq_all', 'ftc_clip_w');
