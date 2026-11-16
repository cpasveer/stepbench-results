import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
from geo import *
from frame import *
A_loc=[(-62.8,0.7,57.5),(-35.9,0.7,79.9),(-49.4,-35.6,41.4),(-22.5,-35.6,63.8)]
B_loc=[(-12.5,-12.5,2),(12.5,-12.5,2),(12.5,12.5,2),(-12.5,12.5,2)]
Anodes=np.array([from_B(q) for q in A_loc]); Bnodes=np.array([from_B(q) for q in B_loc])
Acen=Anodes.mean(0); Bcen=Bnodes.mean(0)
def kstiff(H_loc, r, topo, E=2000.0):
    H=from_B(H_loc)
    nodes=np.vstack([Anodes,Bnodes,[H]]); h=8
    m=[]
    for i in range(4): m.append((4+i,h,r))            # B->H
    for i in range(4): m.append((i,h,r))              # A->H
    if 'ring' in topo:
        for i in range(4): m.append((4+i,4+(i+1)%4,r))
    if 'x' in topo:
        m.append((4,6,r)); m.append((5,7,r))
    fixed=[0,1,2,3]
    ks=[]
    for d in np.eye(3):
        loads={4+i: np.r_[d*4.91/4,0,0,0] for i in range(4)}
        u=solve(nodes,m,fixed,loads,E)
        dB=u[4:8,:3].mean(0)
        ks.append(4.91/np.linalg.norm(dB) if np.linalg.norm(dB)>0 else 1e9)
    # worst translational stiffness over many directions
    best=1e9
    for th in np.linspace(0,np.pi,19):
        for ph in np.linspace(0,2*np.pi,37):
            d=np.array([np.sin(th)*np.cos(ph),np.sin(th)*np.sin(ph),np.cos(th)])
            loads={4+i: np.r_[d*4.91/4,0,0,0] for i in range(4)}
            u=solve(nodes,m,fixed,loads,E)
            dB=u[4:8,:3].mean(0)
            best=min(best,4.91/np.linalg.norm(dB))
    return np.round(ks,1), round(best,1)
if __name__=="__main__":
    for H in [(-25,-12,35),(-30,-15,30)]:
      for topo in ['', 'ring', 'ring x']:
          for r in [1.0,1.5,2.0]:
              print(H,topo or 'plain',r, kstiff(H,r,topo))
