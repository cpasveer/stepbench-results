import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
from p4 import *
np.set_printoptions(suppress=True,precision=3)
H=from_B((0,0,30)); r=2.0
nodes=np.vstack([Anodes,Bnodes,[H]]); h=8
m=[(4+i,h,r) for i in range(4)]+[(i,h,r) for i in range(4)]+[(4,6,r),(5,7,r)]+[(4+i,4+(i+1)%4,r) for i in range(4)]
Bn_local=np.array([[1,0,0],[0,1,0],[0,0,1.]])  # B-local axes in world
ex,ey,ez=EX_B,EY_B,N_B
for name,d in [('ex',ex),('ey',ey),('ez(normal)',ez)]:
    u=solve(nodes,m,[0,1,2,3],{4+i: np.r_[d*4.91/4,0,0,0] for i in range(4)},2000.)
    dB=u[4:8,:3].mean(0)
    print(name,'mean B disp',np.round(dB,4),'k',round(4.91/np.linalg.norm(dB),1))
d=EX_B
u=solve(nodes,m,[0,1,2,3],{4+i: np.r_[d*4.91/4,0,0,0] for i in range(4)},2000.)
print('H disp', np.round(u[8,:3],3), 'rot', np.round(u[8,3:],4))
for (i,j,rr) in m:
    ax=(nodes[j]-nodes[i]); L=np.linalg.norm(ax); ax/=L
    el=(u[j,:3]-u[i,:3])@ax
    print(i,j,'L',round(L,1),'axial elong',round(el,4),'N',round(el*2000*np.pi*rr*rr/L,2))
