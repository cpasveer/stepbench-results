import itertools, sys
exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan13.py").read().split("best=[]")[0])
sys.path.insert(0,"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch")
import numpy as np
from frame import beam_k
E=2500.0; G=E/2.6
def xyz(u,v,n): return CB+UB*u+ZB*v+NB*n
cands=[]
SEC=11.0
roots=[(u,v) for u in np.arange(-14,15,2) for v in np.arange(-16,17,2)]
targets=[]
for s in [-8,-6,-4,-2,0,2,4,6,8]:
    targets.append(("S1",s,P0+AX*s+r1*38.5))
    targets.append(("S3",s,P0+AX*s+r3*38.5))
for ru,rv in roots:
    for nm,s,t in targets:
        ov=0.0
        L=bar(xyz(ru,rv,4.0),t,SEC)
        ov=sum(L.common(kk).Volume for kk in keep)
        if ov<0.05: cands.append((ru,rv,nm,s,t))
print("clear legs", len(cands))
import pickle
pickle.dump([(c[0],c[1],c[2],c[3]) for c in cands], open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/cands.pkl","wb"))
print("saved")
