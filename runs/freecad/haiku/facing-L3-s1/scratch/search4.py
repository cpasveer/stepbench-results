import json, math
exec(open("/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/scratch/search2.py").read().split("ca = ")[0])
ca = (-126.0, 127.5)
P = (-120.0, 112.0)
print("reach B", [round(math.hypot(q[0]-P[0], q[1]-P[1]),1) for q in B])
print("reach A", [round(math.hypot(q[0]-P[0], q[1]-P[1]),1) for q in A])
print("segdist B-to-A", [[round(segdist(o,P,q),2) for o in A] for q in B])
print("segdist A-to-B", [[round(segdist(o,P,q),2) for o in B] for q in A])
print("A", A); print("B", B)
