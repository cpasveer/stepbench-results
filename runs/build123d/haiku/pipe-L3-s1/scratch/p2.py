import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
from geo import *
from frame import *
# pad centres: A pads on pipe at bolt + 2.5 along axis, B pads at z=2
Anodes=[]; 
for k in ['A1','A2','A3','A4']:
    b=BOLTS[k]; p=np.array(b['position']); a=np.array(b['axis'])
    Anodes.append(p+2.5*a)
Bnodes=[from_B(q) for q in [(-12.5,-12.5,2),(12.5,-12.5,2),(12.5,12.5,2),(-12.5,12.5,2)]]
Anodes=np.array(Anodes); Bnodes=np.array(Bnodes)
print('B-local of A pad centres', np.round(np.array([to_B(p) for p in Anodes]),1))
Bc=Bnodes.mean(0)
print('B load node (world)',Bc)
