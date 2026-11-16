import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
import numpy as np
from frame import *
np.set_printoptions(suppress=True,precision=4)
p1=np.array([0,0,0.]); p2=np.array([10.,20,30])
r=2.0;E=2000.
L=np.linalg.norm(p2-p1); x=(p2-p1)/L; A=np.pi*r*r
for d in [x, np.array([1.,0,0]), np.array([0,0,1.])]:
    u=solve(np.vstack([p1,p2]),[(0,1,r)],[0],{1:np.r_[d*10,0,0,0]},E)
    print('dir',np.round(d,2),'disp along x',round(u[1,:3]@x,5),'expected', round(10*L/(E*A),5), 'total disp', np.round(u[1,:3],4))
