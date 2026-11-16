import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
import numpy as np
from p4 import Anodes, Bnodes
E=2000.0
DIRS=[]
for th in np.linspace(0,np.pi,7):
    for ph in np.linspace(0,2*np.pi,12,endpoint=False):
        DIRS.append(np.array([np.sin(th)*np.cos(ph),np.sin(th)*np.sin(ph),np.cos(th)]))
DIRS=np.array(DIRS)
def pin_k(nodes, members, fixed_idx=(0,1,2,3), load_idx=(4,5,6,7), Ecoef=E, dirs=DIRS):
    """members (i,j,A). Returns min translational stiffness (N/mm) of mean load-node displacement."""
    N=len(nodes); K=np.zeros((3*N,3*N))
    for i,j,A in members:
        d=nodes[j]-nodes[i]; L=np.linalg.norm(d); n=d/L; k=Ecoef*A/L
        T=k*np.outer(n,n)
        K[3*i:3*i+3,3*i:3*i+3]+=T; K[3*j:3*j+3,3*j:3*j+3]+=T
        K[3*i:3*i+3,3*j:3*j+3]-=T; K[3*j:3*j+3,3*i:3*i+3]-=T
    free=[x for x in range(3*N) if x//3 not in fixed_idx]
    fpos={x:k for k,x in enumerate(free)}
    Kf=K[np.ix_(free,free)]
    # remove floating nodes (no members) by regularising
    Kf=Kf+np.eye(len(free))*1e-6
    Kinv=np.linalg.inv(Kf)
    best=1e9
    for d in dirs:
        f=np.zeros(len(free))
        for i in load_idx:
            for c in range(3): f[fpos[3*i+c]]+=d[c]*4.91/len(load_idx)
        u=Kinv@f
        um=np.zeros(3)
        for i in load_idx:
            for c in range(3): um[c]+=u[fpos[3*i+c]]
        um/=len(load_idx)
        n=np.linalg.norm(um)
        if n>0: best=min(best,4.91/n)
    return best
if __name__=='__main__':
    nodes=np.vstack([Anodes,Bnodes])
    for pick in [((0,2),(1,3),(0,2),(1,3)), ((0,2),(0,2),(1,3),(1,3))]:
        m=[(4+i,j,np.pi*1.0**2) for i,p in enumerate(pick) for j in p]
        print(pick, round(pin_k(nodes,m),2))
