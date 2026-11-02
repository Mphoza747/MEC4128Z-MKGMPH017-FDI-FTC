"""Independent Python re-implementation of ftc_design_delta.m (X layout, spin mode 'natural').
Recomputes the spinning-hover equilibrium, linearisation, LQR gains, controllability rank and closed-loop eigenvalues,
and compares them with the K_all / seq_all / ueq_all stored in ftc_params_x.mat (used by the Simulink model)."""
import numpy as np, scipy.io as sio, scipy.linalg as sl, json, sys
m=(75+45.7)/1000; l=54.33e-3; Jxx=54737.66e-9; Jyy=53435.96e-9; Jzz=98934.3e-9; g=9.81
kf=5e-8; kt=0.025*kf; tau=0.02
J=np.array([Jxx,Jyy,Jzz])
def tilts(delta):
    pos=np.array([[1,-1],[-1,-1],[-1,1],[1,1]])/np.sqrt(2); out=[]
    for k in range(4):
        ph=-np.arcsin(np.sin(delta)*pos[k,1]); th=np.arcsin(np.sin(delta)*pos[k,0]/np.cos(ph)); out.append((ph,th))
    return out
def fd(s,wc,live):
    p=s[0:3]; v=s[3:6]; phi,th=s[6],s[7]; om=s[8:11]; wl=s[11:14]
    w=np.zeros(4); w[live]=wl; w2=w**2; F=kf*w2.sum()
    M=np.array([l/np.sqrt(2)*kf*(-w2[0]-w2[1]+w2[2]+w2[3]), l/np.sqrt(2)*kf*(-w2[0]+w2[1]+w2[2]-w2[3]), kt*(w2[0]-w2[1]+w2[2]-w2[3])])
    od=(M-np.cross(om,J*om))/J
    psid=(np.sin(phi)*om[1]+np.cos(phi)*om[2])/np.cos(th)
    phd=om[0]+np.sin(phi)*np.tan(th)*om[1]+np.cos(phi)*np.tan(th)*om[2]; thd=np.cos(phi)*om[1]-np.sin(phi)*om[2]
    n=np.array([np.sin(th)*np.cos(phi),-np.sin(phi),np.cos(th)*np.cos(phi)]); zc=np.array([0,0,1.])
    return np.concatenate([v-psid*np.cross(zc,p), F/m*n-np.array([0,0,g])-psid*np.cross(zc,v), [phd,thd], od, (wc-wl)/tau])
def full(x,phi,th): return np.concatenate([[x[0],x[1],0,x[2],x[3],0,phi,th],x[4:7],1000*x[7:10]])
def eqres(x,phi,th,live):
    s=full(x,phi,th); d=fd(s,1000*x[7:10],live); return np.concatenate([d[0:2],d[3:6],d[6:8],d[8:11]])
def design(delta=0.5,wrot=1e-8,spin='natural',qm=None):
    Qd=np.diag([1,1,1,2,2,2,20,20,.02,.02,.02,wrot,wrot,wrot]); Rd=1e-6*np.eye(3)
    Qd=np.diag([1,1,1,2,2,2,20,20,.02,.02,.02,wrot,wrot,wrot])[:11,:11]
    Qd=np.diag(np.concatenate([[1,1,1,2,2,2,20,20,.02,.02,.02],[wrot]*3]))
    res={}
    for k in range(4):
        live=[i for i in range(4) if i!=k]; phi,th=tilts(delta)[k]
        nb=np.array([-np.sin(th),np.cos(th)*np.sin(phi),np.cos(th)*np.cos(phi)])
        sgn=-1 if (spin=='natural' and k in (0,2)) else 1
        x=np.concatenate([[0.003,0,0,0.003],44*sgn*nb,[2.4]*3])
        for it in range(80):
            r=eqres(x,phi,th,live); Jc=np.zeros((10,10))
            for i in range(10):
                e=np.zeros(10); e[i]=1e-7; Jc[:,i]=(eqres(x+e,phi,th,live)-eqres(x-e,phi,th,live))/2e-7
            dx=-np.linalg.solve(Jc+1e-9*np.eye(10),r); a=1
            while np.linalg.norm(eqres(x+a*dx,phi,th,live))>np.linalg.norm(r) and a>1e-4: a/=2
            x=x+a*dx
            if np.linalg.norm(eqres(x,phi,th,live))<1e-10: break
        resn=np.linalg.norm(eqres(x,phi,th,live))
        seq=full(x,phi,th); ueq=1000*x[7:10]
        A=np.zeros((14,14)); B=np.zeros((14,3))
        for i in range(14):
            h=1e-6*max(1,abs(seq[i]))*(1000 if i>=11 else 1); e=np.zeros(14); e[i]=h
            A[:,i]=(fd(seq+e,ueq,live)-fd(seq-e,ueq,live))/(2*h)
        for j in range(3):
            e=np.zeros(3); e[j]=1e-3; B[:,j]=(fd(seq,ueq+e,live)-fd(seq,ueq-e,live))/2e-3
        P=sl.solve_continuous_are(A,B,Qd,Rd); K=np.linalg.solve(Rd,B.T@P)
        res[k]=dict(seq=seq,ueq=ueq,A=A,B=B,K=K,res=resn,live=live)
    return res
if __name__=='__main__':
    import os
    MAT=os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','model','ftc_params_x.mat')  # repository path
    S=sio.loadmat(MAT)
    Ks=S['K_all']; seqs=S['seq_all']; ueqs=S['ueq_all']
    print({k:Ks.shape if hasattr(Ks,'shape') else 0 for k in ['K_all']}, seqs.shape, ueqs.shape)
    out={}
    for wrot in (1e-8,0.0):
        try: R=design(wrot=wrot)
        except Exception as e: print('wrot',wrot,'failed',e); continue
        print('--- rotor weight',wrot)
        for k in range(4):
            r=R[k]; Ks_k=Ks[:,:,k]
            dK=np.abs(r['K']-Ks_k).max()/np.abs(Ks_k).max()
            dseq=np.abs(r['seq']-seqs[:,k]).max(); dueq=np.abs(r['ueq']-ueqs[:,k]).max()
            ctrb=np.hstack([np.linalg.matrix_power(r['A'],i)@r['B'] for i in range(14)])
            sv=np.linalg.svd(ctrb,compute_uv=False); rk=int((sv>sv[0]*1e-9).sum())
            ev=np.linalg.eigvals(r['A']-r['B']@r['K'])
            print(f"rotor {k+1}: eq residual {r['res']:.1e}, rel max |K-Kstored| {dK:.2e}, max|seq diff| {dseq:.2e}, max|ueq diff| {dueq:.2e}, rank ctrb(numerical,1e-9) {rk}, max Re eig {ev.real.max():.2f}, min Re {ev.real.min():.2f}, ueq {r['ueq'].round(1)}, spin om {r['seq'][8:11].round(3)} |om| {np.linalg.norm(r['seq'][8:11]):.3f}")
