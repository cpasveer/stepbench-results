import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
from geo import *
print('EX_B',EX_B,'EY_B',EY_B,'EZ_B',EZ_B,'N_B',N_B)
print('pipe origin',O_A, 'axis', RA[:,0])
for k,b in BOLTS.items():
    p=np.array(b['position']); a=np.array(b['axis'])
    qB=to_B(p); qA=to_A(p)
    print(k, 'B-local',np.round(qB,2),'axisB',np.round([a@EX_B,a@EY_B,a@N_B],3),'| A-local',np.round(qA,2),'axisA',np.round([a@RA[:,0],a@RA[:,1],a@RA[:,2]],3))
