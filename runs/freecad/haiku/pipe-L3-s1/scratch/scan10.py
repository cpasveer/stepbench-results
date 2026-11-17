exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan9.py").read().split('print("A:")')[0])
best=[]
for ru,rv in itertools.product([-4,-6,-8,-10,-12,-14],[0,2,4,6,8]):
    for s in [0,2,4,6]:
        best.append((leg_ok((ru,rv), P0+AX*s+r3*38.5), "C", (ru,rv), s))
for ru,rv in itertools.product([0,2,4,6,8],[-12,-14,-16]):
    for s in [-6,-4,-2,0,2,4]:
        best.append((leg_ok((ru,rv), P0+AX*s+r3*38.5), "B", (ru,rv), s))
best.sort(key=lambda x:x[0])
for b in best[:24]: print("cand", b)
