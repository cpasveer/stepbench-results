import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
from p4 import *
def test(H_loc, r, extra_fn, E=2000.):
    H=from_B(H_loc)
    nodes=np.vstack([Anodes,Bnodes,[H]]); h=8
    m=[(4+i,h,r) for i in range(4)]+[(i,h,r) for i in range(4)]+[(4+i,4+(i+1)%4,r) for i in range(4)]
    m+= [(a,b,r) for a,b in extra_fn]
    best=1e9; worst=None
    for th in np.linspace(0,np.pi,13):
        for ph in np.linspace(0,2*np.pi,25):
            d=np.array([np.sin(th)*np.cos(ph),np.sin(th)*np.sin(ph),np.cos(th)])
            loads={4+i: np.r_[d*4.91/4,0,0,0] for i in range(4)}
            u=solve(nodes,m,[0,1,2,3],loads,E)
            dB=u[4:8,:3].mean(0); k=4.91/np.linalg.norm(dB)
            if k<best: best=k; worst=np.round(d,2)
    return round(best,1), worst
# B nodes 4..7 ; A nodes 0..3 ; H 8
variants={
 'none':[],
 'B-A direct (diag pairs)':[(4,0),(5,1),(6,2),(7,3)],
 'B-A two per B':[(4,0),(4,2),(5,1),(5,3),(6,0),(6,2),(7,1),(7,3)],
 'B-A all 16':[(i,j) for i in range(4,8) for j in range(4)],
}
for name,ex in variants.items():
    for r in [1.0,1.5,2.0]:
        print(name, r, test((0,0,30),r,ex))
