import sys, itertools; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
from p4 import Anodes, Bnodes, from_B, solve, to_B
import numpy as np
from t8 import kmin
nodes=np.vstack([Anodes,Bnodes])  # A 0..3, B 4..7
# list distances
D=np.array([[np.linalg.norm(Bnodes[i]-Anodes[j]) for j in range(4)] for i in range(4)])
print('B-A distances\n',np.round(D,1))
res=[]
for pick in itertools.product(itertools.combinations(range(4),2), repeat=4):
    m=[(4+i, j, 1.0) for i,pair in enumerate(pick) for j in pair]
    k,_=kmin(nodes,m,Bidx=[4,5,6,7])
    res.append((k,pick))
res.sort(reverse=True)
for r in res[:8]: print(r)
print('count',len(res),'min',res[-1])
