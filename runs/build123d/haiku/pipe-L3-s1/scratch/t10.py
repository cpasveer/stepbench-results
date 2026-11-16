import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
from p4 import Anodes, Bnodes, from_B, solve
import numpy as np
from t8 import kmin
def model(rrib, rcol, rleg, zT, Tloc=None, ring=False, zC=2):
    C=from_B((0,0,zC)); T=from_B(Tloc if Tloc else (0,0,zT))
    nodes=np.vstack([Anodes,Bnodes,[C],[T]]); c=8; t=9
    m=[(4+i,c,rrib) for i in range(4)]+[(c,t,rcol)]+[(t,i,rleg) for i in range(4)]
    if ring: m+=[(4+i,4+(i+1)%4,rrib) for i in range(4)]
    return nodes,m
for rrib in [1.5,2.5]:
  for rcol in [2.0,3.0,4.0]:
    for rleg in [1.5,2.5]:
        for zT in [40]:
            nodes,m=model(rrib,rcol,rleg,zT)
            print('rrib',rrib,'rcol',rcol,'rleg',rleg,'zT',zT,kmin(nodes,m))
