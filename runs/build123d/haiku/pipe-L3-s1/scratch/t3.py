import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
from p4 import kstiff
for H in [(0,0,30),(0,0,40),(-8,-4,40),(-10,-6,50),(0,0,50)]:
    for topo in ['', 'ring']:
        for r in [1.5,2.0]:
            print(H,topo or 'plain',r, kstiff(H,r,topo))
