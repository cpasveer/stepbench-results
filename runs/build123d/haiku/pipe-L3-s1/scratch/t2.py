import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
from p4 import *
H=from_B((-25,-12,35)); r=2.0
nodes=np.vstack([Anodes,Bnodes,[H]]); h=8
m=[(4+i,h,r) for i in range(4)]+[(i,h,r) for i in range(4)]
u=solve(nodes,m,[0,1,2,3],{4+i: np.r_[np.array([0,0,1.0])*4.91/4,0,0,0] for i in range(4)},2000.)
print(np.round(u[4:9,:3],3))
print('B nodes',np.round(nodes[4:8],1)); print('H',np.round(H,1))
