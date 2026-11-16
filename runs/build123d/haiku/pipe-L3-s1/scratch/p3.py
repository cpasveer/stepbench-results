import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
from geo import *
from frame import *
A_loc=[(-62.8,0.7,57.5),(-35.9,0.7,79.9),(-49.4,-35.6,41.4),(-22.5,-35.6,63.8)]
B_loc=[(-12.5,-12.5,2),(12.5,-12.5,2),(12.5,12.5,2),(-12.5,12.5,2)]
Anodes=np.array([from_B(q) for q in A_loc]); Bnodes=np.array([from_B(q) for q in B_loc])
Acen=Anodes.mean(0); Bcen=Bnodes.mean(0)
def evaluate(H_loc, r, E=2000.0, extra=[]):
    H=from_B(H_loc)
    nodes=np.vstack([Anodes,Bnodes,[H]])
    nA=4; nB=4; nH=8
    members=[(nA+ 0+i, 8, r) for i in range(4)]  # placeholder replaced below
    members=[(i, 8, r) for i in range(4)]+[(4+i, 8, r) for i in range(4)]+extra
    fixed=list(range(4))
    res=[]
    # translational loads at B nodes (F/4 each) in +x,+y,+z world directions and moment case
    dirs=np.eye(3)
    worst=1e9
    for d in dirs:
        loads={4+i: np.r_[d*4.91/4,0,0,0] for i in range(4)}
        u=solve(nodes,members,fixed,loads,E)
        dB=u[4:8,:3].mean(0)
        k=4.91/np.linalg.norm(dB)
        res.append(('F',np.round(d,0),round(np.linalg.norm(dB),4),round(k,1)))
    # moment about axis perpendicular to span (M = F*78.15 applied as couple at B nodes)
    span=Bcen-Acen; span/=np.linalg.norm(span)
    for d in dirs:
        m=np.cross(d,span)
        if np.linalg.norm(m)<1e-6: continue
        m/=np.linalg.norm(m)
        loads={4+i: np.r_[0,0,0,m*4.91*78.15/4] for i in range(4)}
        u=solve(nodes,members,fixed,loads,E)
        dB=u[4:8,:3].mean(0)
        rot_disp=np.linalg.norm(dB)
        res.append(('M',np.round(m,2),round(rot_disp,4)))
    return res
if __name__=='__main__':
    H=(-25,-12,35)
    for r in [1.0,1.5,2.0]:
        print('r',r)
        for x in evaluate(H,r): print('  ',x)
