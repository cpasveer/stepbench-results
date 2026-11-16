"""Crude voxel FE (1 mm hex8) of the bracket in the design frame (u,h,z) to rank variants."""
import math, sys, time
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu

ZA, ZB = -94.0, -25.5
BU, BH = 5.8556, 38.8925
ABOLT = [(-30, 3.7), (30, 3.7), (-30, 63.7), (30, 63.7)]
BBOLT = [(-28.999, 35.83), (20.647, 70.61), (25.93, 10.22)]   # filled from task below
SPAN = 68.95
SL = math.tan(math.radians(48))

def hex_ke(dx,dy,dz,nu=0.35):
    """8-node brick with Wilson-Taylor incompatible modes (good in bending)."""
    E = 1.0
    lam = E*nu/((1+nu)*(1-2*nu)); mu = E/(2*(1+nu))
    D = np.zeros((6,6)); D[:3,:3] = lam; D[np.arange(3),np.arange(3)] += 2*mu; D[3:,3:] = np.eye(3)*mu
    g = 1/math.sqrt(3)
    nodes = np.array([[i,j,k] for k in (0,1) for j in (0,1) for i in (0,1)], float)*2-1
    J = np.array([dx,dy,dz])/2.0
    K = np.zeros((33,33))
    def Bmat(dN):
        n = dN.shape[0]; B = np.zeros((6,3*n))
        for i in range(n):
            B[0,3*i]=dN[i,0]; B[1,3*i+1]=dN[i,1]; B[2,3*i+2]=dN[i,2]
            B[3,3*i]=dN[i,1]; B[3,3*i+1]=dN[i,0]
            B[4,3*i+1]=dN[i,2]; B[4,3*i+2]=dN[i,1]
            B[5,3*i]=dN[i,2]; B[5,3*i+2]=dN[i,0]
        return B
    for a in (-g,g):
        for b in (-g,g):
            for c in (-g,g):
                dN = np.zeros((11,3))
                for n,(xi,et,ze) in enumerate(nodes):
                    dN[n] = [xi*(1+et*b)*(1+ze*c)/8, et*(1+xi*a)*(1+ze*c)/8, ze*(1+xi*a)*(1+et*b)/8]
                dN[8] = [-2*a,0,0]; dN[9] = [0,-2*b,0]; dN[10] = [0,0,-2*c]
                dN = dN/J[None,:]
                B = Bmat(dN)
                K += B.T@D@B*J.prod()
    Kuu=K[:24,:24]; Kua=K[:24,24:]; Kaa=K[24:,24:]
    return Kuu-Kua@np.linalg.solve(Kaa,Kua.T)

def inpoly(pts, X, Y):
    inside = np.zeros(X.shape, bool)
    n = len(pts)
    for i in range(n):
        x1,y1 = pts[i]; x2,y2 = pts[(i+1)%n]
        cond = ((y1 > Y) != (y2 > Y))
        with np.errstate(divide='ignore', invalid='ignore'):
            xi = (x2-x1)*(Y-y1)/(y2-y1+1e-300)+x1
        inside ^= cond & (X < xi)
    return inside

DX = 1.5
U0, H0 = -36, 0
NU, NH = 48, 50
NZA, NZW, NZB = 2, 41, 2
NZ = NZA+NZW+NZB
DZ = [1.5]*NZA+[61.7/NZW]*NZW+[1.9]*NZB
ZN = np.concatenate([[0],np.cumsum(DZ)])

def rect(u0,u1,h0,h1): return [(u0,h0),(u1,h0),(u1,h1),(u0,h1)]

def build(P):
    """P: dict. returns E-scale grid [u,h,z] (0 = void)."""
    G = np.zeros((NU,NH,NZ))
    uc = U0+(np.arange(NU)+0.5)*DX; hc = H0+(np.arange(NH)+0.5)*DX
    UU,HH = np.meshgrid(uc,hc,indexing='ij')
    TW=P['TW']; U1=P['U1']; U2=P['U2']; HT=P.get('HT',67.7)
    fa = inpoly(rect(U1,U2,0,HT),UU,HH); fb = fa.copy()
    for poly in P.get('openA',[]): fa &= ~inpoly(poly,UU,HH)
    for poly in P.get('openB',[]): fb &= ~inpoly(poly,UU,HH)
    EA=34.5; h0=59.2
    for poly in [rect(-EA,U1,0,8.2),rect(U2,EA,0,8.2),[(U1,h0-(EA+U1)*SL),(-EA,h0),(-EA,HT),(U1,HT)][::-1],[(U2,h0-(EA-U2)*SL),(EA,h0),(EA,HT),(U2,HT)]]:
        fa |= inpoly(poly,UU,HH)
    e2=24.95; e1=-33.5
    for poly in [rect(U2,30.4,0,14.7),[(U2,66.1-(e2-U2)*SL),(e2,66.1),(e2,74.9),(U2,74.9)],[(U1,31.3-(U1-e1)*SL),(e1,31.3),(e1,40.3),(U1,40.3)]]:
        fb |= inpoly(poly,UU,HH)
    for extra in P.get('extraA',[]): fa |= inpoly(extra,UU,HH)
    for extra in P.get('extraB',[]): fb |= inpoly(extra,UU,HH)
    for (u,h) in ABOLT: fa &= ~((UU-u)**2+(HH-h)**2 < 2.25**2)
    for (u,h) in BBOLT: fb &= ~((UU-u)**2+(HH-h)**2 < 2.75**2)
    tA=P.get('TA',3.0); tB=P.get('TB',3.8)
    G[:,:,:NZA] = (fa*(tA/3.0)**3)[:,:,None]          # bending-equivalent scaling
    G[:,:,NZA+NZW:] = (fb*(tB/3.8)**3)[:,:,None]
    # upright webs: list of (ucentre, h0, h1, t, [window polys in (h,zfrac)])
    zc = (np.arange(NZW)+0.5)/NZW
    for (ucen,ha,hb,t,wins) in P['webs']:
        iu = int(math.floor((ucen-U0)/DX))
        m = ((hc>ha)&(hc<hb))[:,None]*np.ones((1,NZW),bool)
        HHw,ZZw = np.meshgrid(hc,zc*61.7,indexing='ij')
        for w in wins: m &= ~inpoly(w,HHw,ZZw)
        G[iu,:,NZA:NZA+NZW] = np.maximum(G[iu,:,NZA:NZA+NZW], m*t/DX)
    # diagonals: (ufoot, uhead, h0, h1, t)
    for (ub,ut,ha,hb,t) in P['diags']:
        th = t/math.sin(math.atan2(61.7,abs(ut-ub)))
        for k in range(NZW):
            ucen = ub+(ut-ub)*zc[k]
            sel = np.abs(uc-ucen) <= DX
            hs = (hc>ha)&(hc<hb)
            G[np.ix_(sel,hs,[NZA+k])] = np.maximum(G[np.ix_(sel,hs,[NZA+k])], th/(2.0*DX))
    return G

KES = {dz: hex_ke(DX,DX,dz) for dz in set(DZ)}
KEZ = np.stack([KES[dz].ravel() for dz in DZ])

def solve(G, bc='ring', verbose=False):
    t0=time.time()
    cells = np.argwhere(G>0)
    nn = (NU+1,NH+1,NZ+1)
    def nid(i,j,k): return (i*nn[1]+j)*nn[2]+k
    offs = [(i,j,k) for k in (0,1) for j in (0,1) for i in (0,1)]
    conn = np.stack([nid(cells[:,0]+o[0],cells[:,1]+o[1],cells[:,2]+o[2]) for o in offs],1)
    used = np.unique(conn); remap = -np.ones(nn[0]*nn[1]*nn[2],int); remap[used]=np.arange(len(used))
    conn = remap[conn]; N=len(used)
    dof = (conn[:,:,None]*3+np.arange(3)).reshape(len(cells),24)
    Es = G[cells[:,0],cells[:,1],cells[:,2]]
    rows = np.repeat(dof,24,1).ravel(); cols = np.tile(dof,(1,24)).ravel()
    vals = (Es[:,None]*KEZ[cells[:,2]]).ravel()
    K = sp.csr_matrix((vals,(rows,cols)),shape=(3*N,3*N))
    # node coordinates
    ii = used//(nn[1]*nn[2]); jj=(used//nn[2])%nn[1]; kk=used%nn[2]
    X = np.stack([U0+ii*DX, H0+jj*DX, ZN[kk]],1).astype(float)
    def near(bolts,r):
        m = np.zeros(N,bool)
        for (u,h) in bolts: m |= (X[:,0]-u)**2+(X[:,1]-h)**2 <= r*r
        return m
    if bc=='ring':
        fixed = (kk<=0)&near(ABOLT,3.5+0.5) | (kk<=NZA)&near(ABOLT,2.25+0.8)
        tied = (kk>=NZ)&near(BBOLT,4.25+0.5) | (kk>=NZ-NZB)&near(BBOLT,2.75+0.8)
    elif bc=='seat':
        fixed = (kk<=0)
        tied = (kk>=NZ)&((X[:,0]-BU)**2+(X[:,1]-BH)**2<40.9**2)
    elif bc=='seatA':
        fixed = (kk<=0)
        tied = (kk>=NZ)&near(BBOLT,4.25+0.5) | (kk>=NZ-NZB)&near(BBOLT,2.75+0.8)
    if bc.startswith('rbe3'):
        fixed = (kk<=0) if bc=='rbe3' else ((kk<=0)&near(ABOLT,4.0) | (kk<=NZA)&near(ABOLT,3.05))
        seat = (kk>=NZ)&((X[:,0]-BU)**2+(X[:,1]-BH)**2<40.9**2)
        # tributary weights: number of top-layer cells touching the node
        top = cells[cells[:,2]==NZ-1]
        w = np.zeros(N)
        for o in [(0,0),(1,0),(0,1),(1,1)]:
            np.add.at(w, remap[nid(top[:,0]+o[0],top[:,1]+o[1],NZ)], 1.0)
        w = w*seat
        sidx = np.where(w>0)[0]
        ref = np.array([BU,BH,ZN[-1]])
        Tm = np.zeros((3*len(sidx),6))
        for a_,n in enumerate(sidx):
            d = X[n]-ref
            M = np.zeros((3,6)); M[:,:3]=np.eye(3)
            M[0,4]=d[2]; M[0,5]=-d[1]; M[1,3]=-d[2]; M[1,5]=d[0]; M[2,3]=d[1]; M[2,4]=-d[0]
            Tm[3*a_:3*a_+3]=M
        Wd = np.repeat(w[sidx],3)
        A = (Wd[:,None]*Tm)@np.linalg.inv(Tm.T@(Wd[:,None]*Tm))
        freen = np.where(~fixed)[0]
        fd = (freen[:,None]*3+np.arange(3)).ravel()
        Kff = K[fd][:,fd].tocsc()
        Ffull = np.zeros((3*N,6)); sd=(sidx[:,None]*3+np.arange(3)).ravel(); Ffull[sd]=A
        lu = splu(Kff)
        Uf = lu.solve(Ffull[fd])
        C = Ffull[fd].T@Uf
        C = 0.5*(C+C.T)
        solve.C = C
        return None
    free = ~(fixed|tied)
    fidx = np.where(free)[0]; tidx=np.where(tied)[0]
    nf = len(fidx)
    # transformation
    fd = (fidx[:,None]*3+np.arange(3)).ravel()
    r_ = np.arange(3*nf); 
    Tr=[];Tc=[];Tv=[]
    Tr.append(fd); Tc.append(r_); Tv.append(np.ones(3*nf))
    ref = np.array([BU,BH,ZN[-1]])
    for a,n in enumerate(tidx):
        d = X[n]-ref
        # u = q_t + q_r x d
        M = np.zeros((3,6)); M[:,:3]=np.eye(3)
        M[0,4]=d[2]; M[0,5]=-d[1]; M[1,3]=-d[2]; M[1,5]=d[0]; M[2,3]=d[1]; M[2,4]=-d[0]
        for c in range(3):
            for q in range(6):
                if M[c,q]!=0: Tr.append([3*n+c]); Tc.append([3*nf+q]); Tv.append([M[c,q]])
    T = sp.csr_matrix((np.concatenate(Tv),(np.concatenate(Tr),np.concatenate(Tc))),shape=(3*N,3*nf+6))
    Kr = (T.T@K@T).tocsc()
    lu = splu(Kr)
    F = np.zeros((3*nf+6,6)); F[3*nf+np.arange(6),np.arange(6)]=1
    q = lu.solve(F)[3*nf:,:]      # 6x6 compliance at ref
    C = 0.5*(q+q.T)
    kt = 1/np.linalg.eigvalsh(C[:3,:3]).max()
    kr = 1/(np.linalg.eigvalsh(C[3:,3:]).max()*SPAN**2)
    diagk = [1/C[i,i] for i in range(3)]+[1/(C[i,i]*SPAN**2) for i in range(3,6)]
    solve.C = C
    if verbose: print(f"  cells {len(cells)} dof {3*N} t {time.time()-t0:.0f}s")
    return kt,kr,diagk,Es.sum() if False else float((G>0).sum())

def run1(): return dict(TW=1.6,U1=-22.8,U2=11.8,webs=[(-22,0,67.7,1.6,[]),(11,0,67.7,1.6,[])],diags=[(4,-15,0,67.7,1.6)])
def window(u0,u1,h0,htop):
    rise=0.5*(u1-u0)*SL
    return [(u0,h0),(u1,h0),(u1,htop-rise),(0.5*(u0+u1),htop),(u0,htop-rise)]
def run2():
    hw=0.659
    return dict(TW=1.2,U1=-22.8,U2=15.2,webs=[(-22.2,0,45,1.2,[]),(14.6,0,45,1.2,[])],diags=[(9.5,-18.5,0,20,1.2)],
        openA=[window(-22.8+1.2+5.6,9.5-hw-5.6,6,62.1)],openB=[window(-18.5+hw+5.6,15.2-1.2-5.6,6,62.1)])
def run3():
    hw=0.628
    return dict(TW=1.2,U1=-22.8,U2=15.2,webs=[(-22.2,0,67.7,1.2,[]),(14.6,0,67.7,1.2,[])],diags=[(4,-15,0,45,1.2)],
        openA=[rect(-16,4-hw-5.6,6,70)],openB=[rect(-15+hw+5.6,8.4,6,70)])

def metrics(C):
    S = np.diag([1,1,1,SPAN,SPAN,SPAN])
    m = {}
    m['t'] = 1/np.linalg.eigvalsh(C[:3,:3]).max()
    m['r'] = 1/(np.linalg.eigvalsh(C[3:,3:]).max()*SPAN**2)
    m['6d'] = 1/np.linalg.eigvalsh(S@C@S).max()
    # translation of ref under moment (cross term) : moment M=span -> |translation|
    X = C[:3,3:]*SPAN
    m['rt'] = 1/np.sqrt(np.linalg.eigvalsh(X.T@X).max())
    return m

if __name__=="__main__":
    import pickle
    out={}
    for name,P,obs in [("run1",run1(),119.9),("run2",run2(),19.6),("run3",run3(),17.8)]:
        for bc in ("rbe3","rbe3r"):
            solve(build(P),bc)
            C=solve.C; out[(name,bc)]=C
            m=metrics(C)
            print(name,bc," ".join(f"{k}:{v:.4f} E={obs/v:.0f}" for k,v in m.items()))
    pickle.dump(out,open("scratch/C.pkl","wb"))
