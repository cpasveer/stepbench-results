import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
from p4 import Anodes, Bnodes, from_B, solve
import numpy as np
def kmin(nodes, members, fixed=[0,1,2,3], E=2000., Bidx=[4,5,6,7]):
    best=1e9; worst=None
    for th in np.linspace(0,np.pi,13):
        for ph in np.linspace(0,2*np.pi,25):
            d=np.array([np.sin(th)*np.cos(ph),np.sin(th)*np.sin(ph),np.cos(th)])
            loads={i: np.r_[d*4.91/4,0,0,0] for i in Bidx}
            u=solve(nodes,members,fixed,loads,E)
            dB=u[Bidx,:3].mean(0); k=4.91/np.linalg.norm(dB)
            if k<best: best=k; worst=np.round(d,2)
    return round(best,1), worst
def build(Cloc, Tloc, r_s, r_t, r_l, bt=False, ring=False):
    C=from_B(Cloc); T=from_B(Tloc)
    nodes=np.vstack([Anodes,Bnodes,[C],[T]]); c=8; t=9
    m=[(4+i,c,r_s) for i in range(4)]+[(c,t,r_t)]+[(t,i,r_l) for i in range(4)]
    if bt: m+=[(4+i,t,r_s) for i in range(4)]
    if ring: m+=[(4+i,4+(i+1)%4,r_s) for i in range(4)]
    return nodes,m
if __name__=='__main__':
    for Cloc,Tloc in [((0,0,2),(-10,-6,45)),((0,0,2),(-14,-8,55)),((-4,-3,2),(-14,-8,55))]:
        for r in [1.0,1.5,2.0]:
            for opts in [dict(), dict(bt=True), dict(ring=True,bt=True)]:
                nodes,m=build(Cloc,Tloc,r,r*1.5,r,**opts)
                print(Cloc,Tloc,r,opts,kmin(nodes,m))
