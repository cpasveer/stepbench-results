import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
from p4 import *
H=from_B((0,0,30)); r=2.0
nodes=np.vstack([Anodes,Bnodes,[H]]); h=8
m=[(4+i,h,r) for i in range(4)]+[(i,h,r) for i in range(4)]
u=solve(nodes,m,[0,1,2,3],{4+i: np.r_[np.array([0,0,1.0])*4.91/4,0,0,0] for i in range(4)},2000.)
np.set_printoptions(suppress=True,precision=4)
print('B disp', u[4:8,:3]); print('H disp', u[8,:3])
print('B to H local z of spoke', [round(to_B(H)[2]-2,1)])
print('B-local of H',to_B(H), 'B-local of nodes', [to_B(n) for n in nodes[4:8]])
for i in range(4):
    d=nodes[8]-nodes[4+i]; L=np.linalg.norm(d); ax=d/L
    rel=u[8,:3]-u[4+i,:3]
    print('spoke',i,'L',round(L,1),'axial elong',round(rel@ax,4),'transverse',np.round(rel-(rel@ax)*ax,4), 'axis',np.round(ax,2))
    print('   rot B',np.round(u[4+i,3:],4))
