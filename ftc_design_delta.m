function D = ftc_design_delta(V, q, delta)
%FTC_DESIGN_DELTA  Copy of the suite's ftc_design_param (fdi_ftc_test_suite.m) with the tilt delta as an input,
% no cache, and the linearisation A_all, B_all returned as well.  Same equilibrium solver and LQR as the suite.
%   V : struct (m g kf kt l tau Jxx Jyy Jzz clip_w);  q : weight multipliers + geom ('x'|'plus') + spin ('natural'|'pos')
geom = 'plus';  if isfield(q, 'geom'), geom = q.geom; end
spinMode = 'pos';  if isfield(q, 'spin'), spinMode = q.spin; end
P = struct('m',V.m,'g',V.g,'kf',V.kf,'kt',V.kt,'l',V.l,'tau',V.tau,'J',[V.Jxx;V.Jyy;V.Jzz],'geom',geom);
tilt  = [0 delta; delta 0; 0 -delta; -delta 0];
if strcmp(geom, 'x')
    pos = [1 -1; -1 -1; -1 1; 1 1]/sqrt(2);
    for kk = 1:4
        ph = -asin(sin(delta)*pos(kk,2));
        tilt(kk,:) = [ph, asin(sin(delta)*pos(kk,1)/cos(ph))];
    end
end
Qd = diag([1*q.pxy 1*q.pxy 1*q.pz, 2*q.vxy 2*q.vxy 2*q.vz, 20*q.att 20*q.att, 0.02*q.om 0.02*q.om 0.02*q.om, 1e-8 1e-8 1e-8]);
Rd = 1e-6*q.r*eye(3);
K_all = zeros(3,14,4); seq_all = zeros(14,4); ueq_all = zeros(3,4); A_all = zeros(14,14,4); B_all = zeros(14,3,4);
for k = 1:4
    live = setdiff(1:4,k);  phi = tilt(k,1); th = tilt(k,2);
    nb = [-sin(th); cos(th)*sin(phi); cos(th)*cos(phi)];
    sgn = 1;  if strcmp(spinMode, 'natural') && any(k == [1 3]), sgn = -1; end
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
    K_all(:,:,k) = lqr(A,B,Qd,Rd);  seq_all(:,k) = seq;  ueq_all(:,k) = ueq;  A_all(:,:,k) = A;  B_all(:,:,k) = B;
end
D = struct('A_all',A_all,'B_all',B_all,'K_all',K_all,'seq_all',seq_all,'ueq_all',ueq_all,'clip_w',V.clip_w);
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
