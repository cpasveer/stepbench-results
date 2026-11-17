exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan13.py").read().split("best=[]")[0])
best=[]
for ru,rv in itertools.product([-4,-3,-2,-1,0,1],[12,13,14,15,16]):
    for s in [4,5,6,7]:
        best.append((leg_ok2((ru,rv), P0+AX*s+r3*38.5), "C", (ru,rv), s))
best.sort(key=lambda x:x[0])
for b in best[:10]: print("cand", b)
