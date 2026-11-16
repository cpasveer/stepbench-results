import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
import numpy as np
from frame import *
nodes=np.array([[0,0,0],[50,0,0.0]])
r=2.0;E=2000.
A=np.pi*r*r; I=np.pi*r**4/4; L=50
u=solve(nodes,[(0,1,r)],[0],{1:[0,0,10,0,0,0]},E)
print('axial z? no: transverse z load 10N ->',u[1,2], 'expect', 10*L**3/(3*E*I))
u=solve(nodes,[(0,1,r)],[0],{1:[10,0,0,0,0,0]},E)
print('axial load ->',u[1,0],'expect',10*L/(E*A))
u=solve(nodes,[(0,1,r)],[0],{1:[0,10,0,0,0,0]},E)
print('lateral y ->',u[1,1],'expect',10*L**3/(3*E*I))
