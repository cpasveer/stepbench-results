import itertools, sys, pickle
exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan13.py").read().split("best=[]")[0])
import numpy as np
cands=[]
roots=[(float(u),float(v)) for u in np.arange(-11,12,1.0) for v in np.arange(-11,12,1.0)]
LEG_END=38.5
targets=[]
for s in [-8,-6,-4,-2,0,2,4,6,8]:
    targets.append(("S1",s,P0+AX*s+r1*LEG_END))
    targets.append(("S3",s,P0+AX*s+r3*LEG_END))
for ru,rv in roots:
    root=CB+UB*ru+ZB*rv+NB*4.0
    for nm,s,t in targets:
        L=bar(root,t,11.0)
        ov=sum(L.common(kk).Volume for kk in keep)
        if ov<0.05: cands.append((ru,rv,nm,s))
print("clear legs", len(cands), flush=True)
pickle.dump(cands, open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/cands2.pkl","wb"))
print("saved", flush=True)
